"""方案B第三步:采样式 MPC 跟踪器(实测四赛道平均 lap2 = 13.83s,119/120 完赛,cap5.0)。

背景:纯追踪 15.7s 封顶(调参无收益);0.8g 剖面(8.5s)不可达 —— Plant.grip 实测只有
2.3-2.9(10-24 IMU 标定),该抓地下剖面下界 13.4-14.4s,MPC 已贴住下界。
MPC 的预测模型逐项复刻仿真执行链:
  滞后队列(lag 2-4 拍,直接读 env.pending) → 舵机速率限幅(1.2-1.8 rad/s)
  → 摩擦圆 tanh 限幅(grip) → 速度死区/平台/弯中掉速(SpeedPlant-lite)
转向序列 3 段×0.2s 采样优化(128 条轨迹向量化 rollout),代价 = 走线偏差 + 航向
+ 锥桶接近 + 转角平滑 + 进度。速度 = 老师逻辑 + 有效抓地调速器(四项):
  ① v_over 扣除:速度环超调让实际车速 = 目标 + v_over(治 B 赛道 S 弯撞锥)
  ② 航向亏损:到前瞻点前要转掉的 |ψ| ≤ 最大偏航率×到达时间(治长弯出弯/S 弯震荡撞锥)
  ③ turn_drag 补偿:弯中掉速 10-25%,指令按 (1-td·δ²) 抬高(2026-10-30,14.2→13.8 的主因)
  ④ 特权自适应:grip_eff = pl.grip·0.95,低抓地局自动收紧(固定 2.9 在 grip 2.3 局翻车)

软件压开销历程(2026-10-30,三种子组 120 局口径):
  14.2(初版) → 13.75(turn_drag 补偿+自适应 ge) → 13.83(WIN_D 0.9,修 S 弯峰间
  再加速;SPEED_MARGIN 0.5 / 自适应剖面 a_lat / GE_FRAC≥1.0 / 固定 ge≥2.7 均更差)。
  剩余 1/120 失败是 grip 2.32 + 弱转向的极限抽签,再压就要牺牲圈速换可靠性。

最终配置(2026-10-30,runs/plan_b_mpc_v6_*.json):
  cap 5.0 / a-lat 3.0 / 自适应 ge(pl.grip·0.95) / WIN_T 0.20 / WIN_D 0.9
  A 14.13  B 13.29  C 14.23  D 13.62,均 13.83,单圈最好 12.70。

    python -m evaluation.plan_b_mpc --tracks A B C D --caps 5.0 --a-lat 3.0 --count 10
"""
from __future__ import annotations

import argparse
import json
import math
import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import numpy as np

from common.params import CarParams
from common.pwm import steer_to_turn_float
from common.road_policy import BRAKE_CMD, STOP_CMD, speed_to_action, spec_of
from evaluation.road_track import (CONE, LAPS, MAPS, _ring_centre, _ring_road, _sort_along,
                                   color_cones, official, real_track, recovered_road)
from simulation.fastsim.road_env import CONTROL_DT, PHYSICS_DT, Plant, RoadEnv
from training import road_teacher as RT
from training.road_teacher import (BRAKE_FLOOR, BRAKE_MARGIN, REPLAN_DEV,
                                   REPLAN_EVERY, Teacher, plan, set_grip_feedback)

ENC = 'spica_v3_s5_v0.45-5_d0.2_log'
SUB = int(round(CONTROL_DT / PHYSICS_DT))       # 5 个物理子步/控制拍


class MpcTeacher(Teacher):
    K = 128                # 采样轨迹数
    H = 12                 # 预测时域(控制拍,0.6s)
    SEG = 3                # 转向序列分段(每段 4 拍 0.2s)
    W_DEV = 60.0           # 走线偏差 m^-2
    W_YAW = 8.0            # 航向误差
    W_PROG = 25.0          # 进度奖励(每米)
    W_COL = 400.0          # 锥桶接近(安全半径内)
    W_DST = 4.0            # 转角变化平滑
    W_SEG = 3.0            # 序列段间差分(治抖振)
    W_SEG0 = 6.0           # 首段相对上拍指令(治跨拍抖振)
    CONE_D0 = 0.55         # 锥桶安全半径(车半宽+锥半径+余量)

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self._u = np.zeros(3)                   # 上一拍解(热启动)
        self._rng = np.random.default_rng(7)
        self.last_steer_a = 0.0
        self.ge_eff = self.GRIP_EFF             # 实例级(可被 run() 按该局 pl.grip 自适应覆盖)
        self.cones_map = None                   # 感知锥桶(第一圈建图结果);None = 用真桶(特权)
        self.plan_road = None                   # 规划用几何路;None = env.road(特权中心线)
        self._road_i = 0                        # 规划路上的当前投影下标(重规划 project 的近邻初值)
        # 向量化转向曲线:steer_a → 前轮角(rad)。env.step: servo = turn_mid + a·span + offset
        _sv, _dg = zip(*self.env.cp.steer_curve)
        self._a_grid = (np.asarray(_sv, float) - self.env.cp.turn_mid - self.pl.servo_offset) \
            / self.env.cp.turn_span
        self._d_grid = np.radians(np.asarray(_dg, float)) * self.pl.steer_gain

    def _action_to_delta(self, a):
        return np.interp(a, self._a_grid, self._d_grid)

    def _replan(self):
        """重规划在规划路(self.plan_road)上做,而不是 env.road —— 全自主时 env.road 是真实物理路
        (碰撞/圈时),规划几何却是第一圈建图恢复出来的那条;两者要分开。"""
        env, car = self.env, self.env.car
        road = self.plan_road if self.plan_road is not None else env.road
        i, d = road.project((car.x, car.y), self._road_i)
        self._road_i = i
        P = plan(road, i, d, env.cp.width, self.pl, RT.HORIZON, RT.R_REPLAN,
                 self.a_lat, self.v_end, self.brake)
        if isinstance(P, str):
            self.fails += 1
            return
        self.plan, self.j = P, 0

    # ---- 速度指令:老师逻辑 + 有效抓地调速器 ----
    GRIP_EFF = 0.0             # 摩擦圆 tanh 后的有效抓地;0 = 特权自适应 pl.grip·GE_FRAC(默认,稳健)
    GE_FRAC = 0.95             # 自适应模式:ge_eff = pl.grip·GE_FRAC(老师特权,按该局真实抓地;0.95 实测稳健上限)
    WIN_T = 0.20               # 调速器前瞻时长 s
    WIN_D = 0.9               # 调速器前瞻距离 m
    K_MARGIN = 1.0            # 曲率安全系数:走线平滑后的 κ 低估 S 弯反向段的几何需求
    TD_COMP = True             # 弯中掉速补偿:指令按 (1-turn_drag·δ²) 抬高,让掉完速的实际车速 = 抓地上限
    L_REC = 2.0               # 航向亏损恢复长度 m:ψ/L_REC 折算成额外曲率需求
    PSI_DB = 0.05             # 航向亏损死区 rad(~3°),弯中常态滞后不罚
    LAG_MIN = 0               # 执行链路延迟写回(拍):rollout 至少假设这么多拍指令在途,
                              # 观测队列不足时按队尾补齐。实车标定 0.18-0.26s(4-5 拍)时设 4-5;
                              # 0 = 只信观测队列(仿真队列是准的,实车 FIFO 未必观测得全)
    EMERG_D = 0.0             # 雷达近距应急(m):感知锥桶进入车前 EMERG_D+0.2m 半圆内强制刹车;
                              # 0 = 关。保护低速段(起步/泊车/突发),高速上刹车距离不够、靠 MPC 锥桶代价

    def _speed_cmd(self):
        env, car, P, pl, cp = self.env, self.env.car, self.plan, self.pl, self.env.cp
        t = CONTROL_DT * (pl.lag + 0.5)
        v = max(car.v, 0.3)
        ahead = P.s[self.j] + v * (pl.dead_time + CONTROL_DT * (pl.lag + 1)) + RT.SPEED_MARGIN
        kv = min(len(P.v) - 1, int(np.searchsorted(P.s, ahead)))
        q = env.speed.queue
        v_eff = car.v - RT.A_BRK_EST * PHYSICS_DT * sum(1 for c in q if c < 0.0)
        if (P.v_coast is not None and car.v > self.release
                and v_eff > max(P.v_coast[kv], BRAKE_FLOOR) + BRAKE_MARGIN):
            return BRAKE_CMD
        v_cmd = float(P.v[kv])
        # 调速器:前方 WIN_T·v+WIN_D 内最大曲率,按有效抓地限速(tanh 削减后 2.6 只有 ~2.0)。
        # 扣 pl.v_over:速度环超调让实际车速 = 目标 + v_over(96200000 局 v_over 0.17,
        # S 弯反向段实测 3.3 m/s 就是它叠出来的);κ 乘安全系数治走线平滑低估。
        kmax = float(np.max(np.abs(P.kappa[kv:min(len(P.kappa), kv + int((v * self.WIN_T + self.WIN_D) / 0.05))]))) \
            * self.K_MARGIN if kv < len(P.kappa) else 0.0
        # 航向亏损:入弯超速/换向震荡留下的车头偏差,在满舵下也追不回(C 96200006 局 -18°
        # 穿完 κ=0.59 长弯后放速撞锥;B 96220003 局 S 弯 ±50° 震荡出弯 -35° 撞锥)。
        # 统一公式:车到前瞻点 kv 前要转掉 |ψ_kv|(车头 vs 线在 kv 的切线),最大偏航率
        # ≈ G_eff/v、到达耗时 d/v ⇒ |ψ_kv| ≤ G_eff·d/v² ⇒ v ≤ sqrt(G_eff·d/|ψ_kv|)。
        # 稳态弯中 ψ_kv = κ·d,退化成标准曲率限速 sqrt(G/κ),不误罚;有亏损时自动收紧。
        kw = min(kv + 2, len(P.pts) - 1)
        hd = math.atan2(P.pts[kw][1] - P.pts[kv][1], P.pts[kw][0] - P.pts[kv][0])
        psi = abs((car.yaw - hd + math.pi) % (2 * math.pi) - math.pi)
        d_ahead = max(P.s[kv] - P.s[self.j], 0.05)
        if kmax > 1e-3:
            v_cmd = min(v_cmd, math.sqrt(self.ge_eff / kmax) - pl.v_over)
        if psi > self.PSI_DB:
            v_cmd = min(v_cmd, math.sqrt(self.ge_eff * d_ahead / psi) - pl.v_over)
        # 弯中掉速补偿:SpeedPlant 里目标 ×(1-turn_drag·δ²)(实测 10-24,td 1~3),
        # 不补偿的话弯中实际车速只有指令的 75~90% —— 之前弯道实测比理想剖面低
        # 0.3-0.5 m/s 的主因(自适应 grip/减速均无收益,已排除调速器保守)。
        # 按窗口曲率估前轮角,把指令抬高让"掉完速的实际"等于抓地上限(特权:知道 td)。
        if self.TD_COMP and kmax > 1e-3:
            delta = math.atan(kmax * cp.wheelbase)
            v_cmd = v_cmd / max(1.0 - pl.turn_drag * delta * delta, 0.6)
        return max(v_cmd, pl.v_plateau)

    # ---- MPC 转向 ----
    def _rollout(self, seqs, v_cmd, v0, x0, y0, yaw0, d0, pend, pl, cp, win):
        """seqs (K,H) steer_a;win = (pts, s, head, cones) 预投影窗口。返回代价 (K,)。"""
        K, H = seqs.shape
        pts, s_w, head, cones = win
        W = len(pts)
        x = np.full(K, x0)
        y = np.full(K, y0)
        yaw = np.full(K, yaw0)
        v = np.full(K, v0)
        d = np.full(K, d0)
        cost = np.zeros(K)
        lag_a = [p[0] for p in pend]                       # 已入队、马上要执行的转向
        if len(lag_a) < self.LAG_MIN:                      # 延迟写回:观测不全的链路段按队尾补齐
            lag_a = lag_a + [lag_a[-1] if lag_a else 0.0] * (self.LAG_MIN - len(lag_a))
        dt = PHYSICS_DT
        grip = cp.a_lat_max
        for h in range(H):
            a = seqs[:, h].copy()
            if h < len(lag_a):
                a[:] = lag_a[h]
            d_t = np.clip(self._action_to_delta(a), -cp.max_steer, cp.max_steer)
            for _ in range(SUB):
                d += np.clip(d_t - d, -cp.steer_rate * dt, cp.steer_rate * dt)
                yaw_rate = v * np.tan(d) / cp.wheelbase
                a_lat = np.abs(v * yaw_rate)
                got = grip * np.tanh(a_lat / grip)
                yaw_rate = np.where(a_lat > 1e-9, yaw_rate * (got / np.maximum(a_lat, 1e-9)), yaw_rate)
                yaw_mid = yaw + 0.5 * yaw_rate * dt
                x += v * np.cos(yaw_mid) * dt
                y += v * np.sin(yaw_mid) * dt
                yaw = (yaw + yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
                # SpeedPlant-lite:死区后一阶趋近目标(含弯中掉速)
                tt = h * CONTROL_DT
                if tt >= pl.dead_time:
                    if v_cmd <= 0.0:
                        v = np.maximum(0.0, v - pl.brake_decel * dt)
                    else:
                        tgt = min(max(v_cmd, pl.v_plateau), pl.v_top) * (1.0 - pl.turn_drag * d * d)
                        tgt = np.maximum(pl.v_plateau, tgt)
                        v = np.where(tgt > v, np.minimum(tgt, v + pl.accel_up * dt),
                                     np.maximum(tgt, v - pl.decel * dt))
            # 代价:走线偏差 + 航向
            dd = (x[:, None] - pts[None, :, 0]) ** 2 + (y[:, None] - pts[None, :, 1]) ** 2
            jn = np.argmin(dd, axis=1)
            cost += self.W_DEV * dd[np.arange(K), jn]
            cost += self.W_YAW * wrap_pi_arr(yaw - head[jn]) ** 2
            if len(cones):
                dc = np.hypot(x[:, None] - cones[None, :, 0], y[:, None] - cones[None, :, 1])
                cost += self.W_COL * np.minimum(np.maximum(self.CONE_D0 - dc.min(axis=1), 0.0), 1.0) ** 2
            if h > 0:
                cost += self.W_DST * 0.0     # 平滑代价在序列参数上算(见下)
        cost -= self.W_PROG * s_w[np.clip(jn, 0, W - 1)]     # 终端进度
        return cost

    def _mpc(self, v_cmd):
        env, car, P, pl, cp = self.env, self.env.car, self.plan, self.pl, self.env.cp
        # 窗口:当前 j 前后 1.2m / 前向 v*0.7+2m
        v = max(car.v, 0.3)
        i0 = max(0, self.j - int(1.2 / 0.05))
        i1 = min(len(P.pts), self.j + int((v * 0.7 + 2.0) / 0.05))
        pts = P.pts[i0:i1]
        head = np.arctan2(*(pts - np.roll(pts, 1, axis=0)).T[::-1])[1:]
        head = np.concatenate([[head[0]], head])
        cones = self.cones_map if self.cones_map is not None else self.env.road.cones
        if len(cones):
            rel = np.hypot(cones[:, 0] - car.x, cones[:, 1] - car.y)
            cones = cones[rel < v * 0.7 + 2.5]
        win = (pts, P.s[i0:i1], head, cones)
        # 采样:热启动 + 高斯 + 随机
        K, H, SEG = self.K, self.H, self.SEG
        base = np.roll(self._u, -1) if SEG > 1 else self._u
        seqs = np.empty((K, H))
        cand = np.vstack([
            np.tile(base, (K // 4, 1)) + self._rng.normal(0, 0.10, (K // 4, SEG)),
            self._rng.normal(0, 0.30, (K // 2, SEG)) + base,
            self._rng.uniform(-1, 1, (K - K // 4 - K // 2, SEG)),
        ])
        cand = np.clip(cand, -1, 1)
        idx = np.arange(SEG).repeat(H // SEG)
        seqs = cand[:, idx]
        cost = self._rollout(seqs, v_cmd, car.v, car.x, car.y, car.yaw, car.steer,
                             env.pending, pl, cp, win)
        # 抖振抑制:段间差分 + 首段偏离上拍指令
        cost = cost + self.W_SEG * np.sum(np.diff(cand, axis=1) ** 2, axis=1) \
            + self.W_SEG0 * (cand[:, 0] - self.last_steer_a) ** 2
        b = int(np.argmin(cost))
        self._u = cand[b]
        return float(seqs[b, 0])

    def __call__(self):
        env, car = self.env, self.env.car
        if self.replan:
            dd = self.plan.pts[max(0, self.j - 20):self.j + 60] - (car.x, car.y)
            dev = math.sqrt(float(np.min(np.einsum('ij,ij->i', dd, dd))))
            if self.n % REPLAN_EVERY == 0 or dev > REPLAN_DEV:
                self._replan()
        self.n += 1
        P = self.plan
        lo, hi = max(0, self.j - 20), min(len(P.pts), self.j + 60)
        dd = P.pts[lo:hi] - (car.x, car.y)
        self.j = lo + int(np.argmin(np.einsum('ij,ij->i', dd, dd)))
        v_cmd = self._speed_cmd()
        if self.EMERG_D > 0.0:
            # 雷达近距应急:感知锥桶进入车正前方走廊(纵向 0.05~EMERG_D+0.2m、横向 <0.3m)强制刹车。
            # 不动转向 —— 高速下横向 escape 本身可能甩出走廊,只兜低速段与突发侵入;
            # 半圆口径(2026-11-08 初版)会把弯中横向 0.5m 的正常锥桶也刹停,爬行超时,已否决。
            cones = self.cones_map if self.cones_map is not None else env.road.cones
            if len(cones):
                rel = cones - (car.x, car.y)
                fw = np.array([math.cos(car.yaw), math.sin(car.yaw)])
                lo = rel[:, 0] * fw[0] + rel[:, 1] * fw[1]           # 纵向
                la = np.abs(rel[:, 0] * -fw[1] + rel[:, 1] * fw[0])  # 横向
                if np.any((lo > 0.05) & (lo < self.EMERG_D + 0.2) & (la < 0.3)):
                    v_cmd = BRAKE_CMD
        steer_a = self._mpc(min(v_cmd, self.pl.cap))
        self.last_steer_a = steer_a
        if v_cmd == BRAKE_CMD:
            speed_a = speed_to_action(BRAKE_CMD, self.pl.enc)
        else:
            speed_a = speed_to_action(v_cmd, self.pl.enc)
        return np.array([steer_a, speed_a], np.float32)


def wrap_pi_arr(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---- 第一圈建图:多帧累积 + 跨帧轨迹一致性过滤 ----
MAP_FRAMES = 3              # 建图累积帧数
MAP_P_TRUE = 0.97           # 真桶单帧被看到概率(实桶几乎每帧都见)
MAP_GHOST_PERSIST = 0.25    # 幽灵桶单帧复现概率(反光/杂物多为瞬态)
MAP_MIN_HITS = 2            # 至少命中不同帧数,才认定为真桶
MAP_CLUSTER_R = 0.15        # 跨帧聚类半径 m(真桶抖动 σ=0.05,三帧内 ≥2 帧命中仍 ~99% 保留)


def _ghosts(cones, rng, fp):
    """误检候选:撒在真桶 ±0.3~0.6m(超过 NEW_CONE_MIN=0.3,单帧看是独立假桶)。"""
    n = int(round(fp * len(cones)))
    k = rng.integers(0, len(cones), n)
    ang = rng.uniform(0, 2 * math.pi, n)
    rad = rng.uniform(0.3, 0.6, n)
    return cones[k] + np.c_[rad * np.cos(ang), rad * np.sin(ang)]


def _cluster_tracks(det, fids, radius, min_hits):
    """贪心跨帧聚类:命中不同帧数 >= min_hits 的轨迹取均值入图,瞬态幽灵桶丢弃。"""
    used = np.zeros(len(det), bool)
    out = []
    for i in range(len(det)):
        if used[i]:
            continue
        d = np.hypot(det[:, 0] - det[i, 0], det[:, 1] - det[i, 1])
        m = (d <= radius) & ~used
        used |= m
        if len(np.unique(fids[m])) >= min_hits:
            out.append(det[m].mean(0))
    return np.asarray(out, float).reshape(-1, 2)


def _perturb_cones(cones, seed, jitter, drop, fp, cone_filter='track'):
    """单色锥桶第一圈建图误差 + 误检过滤。

    none : 旧口径,单帧静态建图(抖动/漏检/幽灵桶直接入图)。
    track: 按 MAP_FRAMES 帧累积。真桶每帧约 MAP_P_TRUE 被看到、幽灵桶只有
           MAP_GHOST_PERSIST 概率复现;跨帧聚类后只保留命中 >= MAP_MIN_HITS 帧
           的轨迹 —— 单帧反光/杂物形成的幽灵桶在时间上不连续,自然被滤掉。
    """
    cones = np.asarray(cones, float).copy()
    if not len(cones):
        return cones
    rng = np.random.default_rng(seed + 11)
    keep = rng.random(len(cones)) >= drop
    cones = cones[keep]
    if not len(cones):
        return np.empty((0, 2))
    if cone_filter == 'none':
        if jitter > 0:
            cones = cones + rng.normal(0.0, jitter, cones.shape)
        if fp > 0:
            cones = np.vstack([cones, _ghosts(cones, rng, fp)])
        return cones
    det, fids = [], []
    for f in range(MAP_FRAMES):
        seen = rng.random(len(cones)) < MAP_P_TRUE
        pts = cones[seen]
        if jitter > 0 and len(pts):
            pts = pts + rng.normal(0.0, jitter, pts.shape)
        if fp > 0:
            g = _ghosts(cones, rng, fp)
            g = g[rng.random(len(g)) < MAP_GHOST_PERSIST]
            pts = np.vstack([pts, g]) if len(pts) else g
        det.append(pts)
        fids.append(np.full(len(pts), f))
    det, fids = np.vstack(det), np.concatenate(fids)
    if not len(det):
        return np.empty((0, 2))
    return _cluster_tracks(det, fids, MAP_CLUSTER_R, MAP_MIN_HITS)


def _map_cones(road, seed, jitter, drop, fp, cone_filter='track'):
    """第一圈建图锥桶(半特权口径):边界/中心线仍用特权,锥桶按建图误差扰动。
    jitter 位置高斯抖动 m;drop 漏检概率;fp 误检率(相对真桶数,幽灵桶撒在真桶 ±0.3~0.6m,
    超过锥桶合并门限 NEW_CONE_MIN=0.3,当独立假桶 —— 雷达把桶旁杂物/反光误认)。"""
    return _perturb_cones(road.cones, seed, jitter, drop, fp, cone_filter)


def _map_cones_color(name, seed, jitter, drop, fp, cone_filter='track'):
    """第一圈建图分色锥桶(全自主口径):直接读 maps 分色真桶(蓝=外/红=内)按误差扰动,
    不含 fill_v 在黄 V 处补的假桶(第一圈那里是胶带、没有桶)。返回 (blue, red)。
    real* 实车赛道没有分色 csv,maps/<name>.yaml 的 outer/inner 就是外/内圈真桶。"""
    if name.startswith('real'):
        import yaml
        lay = yaml.safe_load((MAPS / f'{name}.yaml').read_text(encoding='utf-8'))
        blue, red = np.array(lay['outer'], float), np.array(lay['inner'], float)
    else:
        blue, red = color_cones(name)
    return (_perturb_cones(blue, seed, jitter, drop, fp, cone_filter),
            _perturb_cones(red, seed + 1_000_003, jitter, drop, fp, cone_filter))


def recovered_road_real(blue, red, anchor, ref, laps=LAPS, sigma=0.7):
    """实车赛道的全自主边界恢复:实车图只有桶、没有胶带(黄 V),排序参照用真值中心线
    (特权只用来定方向,同 recovered_road 里第一圈轨迹的用法),其余同 recovered_road。"""
    outer = _sort_along(blue, ref)
    inner = _sort_along(red, ref)
    cen, od, idn, dl, dr = _ring_centre(outer, inner, sigma=sigma)
    i0 = int(np.argmin(np.hypot(*(cen - np.asarray(anchor, float)).T)))
    mat = np.full(len(cen), CONE)
    return _ring_road(cen, dl, dr, mat, np.vstack([blue, red]), outer, inner, i0, laps,
                      dict(kind='real_auto'))


def _standing_reset(env, pl):
    """实际赛制(2026-11-07):第一圈跑完停进框、从 0 重新加速跑第二圈。

    这里把车定格在过线点(≈ START 框)、速度清 0,并把 SpeedPlant 恢复到"停稳待起步"
    状态。第二圈第一拍给定油后自然走 relaunch_delay(破静)→平台跳跃→加速,与实车
    停车再起步一致。第一圈过线点到停稳之间的刹车/滑行距离不建模:实际第一圈是慢速
    建图圈(而非本仿真的竞速圈),该距离本来就短,且不影响第二圈 0 起步的圈时测量。
    """
    env.car.v = 0.0
    env.car.accel = 0.0
    env.speed.v = 0.0
    env.speed.stopped = True
    env.speed.hold = None
    env.speed.reverse_s = 0.0
    env.speed.queue = [STOP_CMD] * len(env.speed.queue)   # 死区内不再有在途油门
    env.pending = [(0.0, STOP_CMD)] * pl.lag              # 清掉第一圈末尾的在途指令
    env.cmd = (0.0, STOP_CMD)


def run(job):
    name, seed, cap, a_lat, gfb, grip_eff, win_t, adapt, standing, jitter, drop, fp, autonomous, \
        cone_filter, lag_min, emerg_d, cone_margin = job
    MpcTeacher.GRIP_EFF, MpcTeacher.WIN_T = grip_eff, win_t
    MpcTeacher.LAG_MIN, MpcTeacher.EMERG_D = lag_min, emerg_d
    is_real = name.startswith('real')
    # 无条件写:进程池 worker 会复用,不能让上一局的覆盖值泄漏到这一局(0.29 = 0.14+0.15 默认)。
    # 实车图全锥桶边界、恢复走廊在弯出口比真值窄,plan 贴外侧线 + 跟踪误差会擦锥
    # (seed 5 实测:plan 线距锥 0.519,跟踪误差 0.06 即蹭);0.40 实测救回且不窄路。
    RT.SIDE_MARGIN[RT.CONE] = cone_margin if cone_margin > 0.0 else 0.29
    road, lap_len = real_track(name) if is_real else official(name, True)
    pl = replace(Plant.sample(np.random.default_rng(seed), spec_of(ENC)), cap=cap)
    if adapt > 0.0:
        # 特权自适应:该局真实滑行/刹车倒推减速(原全局 0.9/1.9 是对 0.85-1.0 / 1.8-2.1 的保守值)
        RT.set_decel(pl.decel * adapt, pl.brake_decel * adapt)
        RT.A_BRK_EST = pl.brake_decel * 0.9
    # 剖面 a_lat 特权自适应:grip 2.3 的局按 3.0 规划会在 S 弯峰间放速到打不住(96300004 局);
    # 刹车倒推会带着正确的物理自动限住入口速度,比调速器滑窗更可靠。
    # 实测(2026-10-30,三种子组 120 局):自适应 a_lat 慢 0.15s 且未修复撞车(倒推用的
    # A_DEC 全局保守,入口提前松油),不如固定 3.0 + 调速器自适应 ge 的组合,保留开关供对比。
    if a_lat <= 0.0:
        a_lat = pl.grip * MpcTeacher.GE_FRAC
    plan_road, perception = road, None
    if autonomous:
        blue, red = _map_cones_color(name, seed, jitter, drop, fp, cone_filter)
        perception = np.vstack([blue, red])
        plan_road = (recovered_road_real(blue, red, road.centre[0], road.centre)[0] if is_real
                     else recovered_road(name, blue, red, road.centre[0])[0])
    P = plan(plan_road, 0, 0.0, CarParams().width, pl, a_lat=a_lat)
    if isinstance(P, str):
        return dict(track=name, seed=seed, cap=cap, ok=False, why=P, laps=[], fails=0)
    env = RoadEnv(road, pl, seed, noise=False, sense=False)
    t = MpcTeacher(env, P, a_lat=a_lat)
    if autonomous:
        t.plan_road = plan_road
        t.cones_map = perception
    elif jitter > 0 or drop > 0 or fp > 0:
        t.cones_map = _map_cones(road, seed, jitter, drop, fp, cone_filter)   # 半特权:感知锥桶换成第一圈建图结果
    if grip_eff <= 0.0:
        t.ge_eff = pl.grip * MpcTeacher.GE_FRAC        # 按该局真实抓地自适应
    why, laps = None, []
    if standing:
        # 实际赛制:第一圈过线后停车、第二圈从 0 重新起步。单循环跑到 why 被置位(goal)才停,
        # 过线只记圈时、不 break —— 否则 why 停在 None,ok=why=='goal' 恒为 False。
        laps = [None, None]
        s_start = t0 = None
        while why is None:
            why = env.step(t())
            s_cur = env.road.s[env.idx]
            if laps[0] is None and s_cur >= lap_len:
                laps[0] = env.t                        # lap1 = 启动 → 第一圈过线(参照)
                _standing_reset(env, pl)
                t._u = np.zeros(3)                     # 停车后转向从中位重新起步,无上一拍残留
                t.last_steer_a = 0.0
                s_start = env.road.s[env.idx]          # 定格点的投影里程(≈ lap_len)
                t0 = env.t
            elif laps[0] is not None and laps[1] is None and s_cur >= s_start + lap_len:
                laps[1] = env.t - t0                   # lap2 = 0 起步 → 第二圈过线
        if laps[0] is None:
            laps = []
        elif laps[1] is None:
            laps = [laps[0]]
    else:
        tl = []
        while why is None:
            why = env.step(t())
            if road.s[env.idx] >= lap_len * (len(tl) + 1) and len(tl) < LAPS:
                tl.append(env.t)
        laps = [tl[0]] + list(np.diff(tl)) if tl else []
    return dict(track=name, seed=seed, cap=cap, ge=grip_eff, wt=win_t, ad=adapt, standing=standing,
                jit=jitter, drop=drop, fp=fp, auto=autonomous, filter=cone_filter,
                lag_min=lag_min, emerg=emerg_d, cm=cone_margin,
                ok=why == 'goal', why=why, laps=laps, fails=t.fails)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tracks', nargs='+', default=['A'])
    ap.add_argument('--caps', type=float, nargs='+', default=[2.4])
    ap.add_argument('--a-lat', type=float, default=0.0,
                    help='剖面横向加速度;0(默认) = 特权自适应 pl.grip·GE_FRAC')
    ap.add_argument('--grip-fb', type=float, default=0.0)
    ap.add_argument('--count', type=int, default=3)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--seed-start', type=int, default=96_200_000)
    ap.add_argument('--a-acc', type=float, default=1.5)
    ap.add_argument('--a-dec', type=float, default=0.9)
    ap.add_argument('--a-brk', type=float, default=1.9)
    ap.add_argument('--grip-effs', type=float, nargs='+', default=[0.0],
                    help='调速器有效抓地;0(默认) = 特权自适应 pl.grip·GE_FRAC')
    ap.add_argument('--win-ts', type=float, nargs='+', default=[0.20])
    ap.add_argument('--adapts', type=float, nargs='+', default=[0.0],
                    help='特权自适应系数:滑行/刹车倒推用该局 pl.decel/pl.brake_decel×该值;0 = 用 --a-dec/--a-brk')
    ap.add_argument('--standing', action='store_true',
                    help='实际赛制:第一圈停车、第二圈从 0 重新起步(默认滚动连续两圈)')
    ap.add_argument('--cone-jitter', type=float, default=0.0, help='第一圈建图锥桶位置高斯抖动 m')
    ap.add_argument('--cone-drop', type=float, default=0.0, help='第一圈建图漏检概率(0~1)')
    ap.add_argument('--cone-fp', type=float, default=0.0, help='第一圈建图误检率(相对真桶数,0~1)')
    ap.add_argument('--cone-filter', choices=('none', 'track'), default='track',
                    help='误检过滤:track=多帧累积+跨帧轨迹一致性(默认);none=旧单帧口径(可复现历史结果)')
    ap.add_argument('--lag-mins', type=int, nargs='+', default=[0],
                    help='执行链路延迟写回(拍):rollout 至少假设这么多拍在途,队列不足按队尾补齐;0 = 关')
    ap.add_argument('--emerg-ds', type=float, nargs='+', default=[0.0],
                    help='雷达近距应急(m):感知锥桶进入车前该距离+0.2m 半圆强制刹车;0 = 关')
    ap.add_argument('--cone-margins', type=float, nargs='+', default=[0.0],
                    help='实车图锥桶侧边距覆盖(m):>0 时 real* 赛道把规划锥桶侧边距从 0.29 抬到该值;0 = 不改')
    ap.add_argument('--autonomous', action='store_true',
                    help='全自主:规划几何从第一圈建图分色锥桶+胶带恢复(否则用特权中心线)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    RT.A_ACC = a.a_acc
    RT.set_decel(a.a_dec, a.a_brk)
    jobs = [(t, a.seed_start + i, cap, a.a_lat, a.grip_fb if a.grip_fb > 0 else None, ge, wt, ad, a.standing,
             a.cone_jitter, a.cone_drop, a.cone_fp, a.autonomous, a.cone_filter, lm, ed, cm)
            for t in a.tracks for cap in a.caps for ge in a.grip_effs for wt in a.win_ts
            for ad in a.adapts for lm in a.lag_mins for ed in a.emerg_ds
            for cm in a.cone_margins for i in range(a.count)]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run, jobs))
    summary = {}
    mode = ('auto' if a.autonomous else ('semi' if (a.cone_jitter or a.cone_drop or a.cone_fp) else 'priv')) \
        + ('/track' if a.cone_filter == 'track' else '/none')
    for t in a.tracks:
        for cap in a.caps:
            for ge in a.grip_effs:
                for wt in a.win_ts:
                    for ad in a.adapts:
                        for lm in a.lag_mins:
                            for ed in a.emerg_ds:
                                for cm in a.cone_margins:
                                    rs = [r for r in rows if r['track'] == t and r['cap'] == cap
                                          and r['ge'] == ge and r['wt'] == wt and r['ad'] == ad
                                          and r['lag_min'] == lm and r['emerg'] == ed
                                          and r['cm'] == cm]
                                    if not rs:
                                        continue
                                    ok = [r for r in rs if r['ok']]
                                    reasons = {}
                                    for r in rs:
                                        reasons[r['why']] = reasons.get(r['why'], 0) + 1
                                    summary[f'{t} cap{cap:.1f} ge{ge:g} wt{wt:g} ad{ad:g} '
                                            f'lm{lm} ed{ed:g} cm{cm:g} {mode}'] = dict(
                                        ok=f'{len(ok)}/{len(rs)}', reasons=reasons,
                                        lap2=round(float(np.mean([r['laps'][1] for r in ok])), 2) if ok else None,
                                        fails_mean=round(float(np.mean([r['fails'] for r in rs])), 1))
    out = dict(jobs=len(jobs), summary=summary, rows=rows)
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
    print(json.dumps(dict(jobs=len(jobs), summary=summary), ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
