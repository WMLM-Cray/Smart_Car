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
from common.road_policy import BRAKE_CMD, speed_to_action, spec_of
from evaluation.road_track import LAPS, official
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
        # 向量化转向曲线:steer_a → 前轮角(rad)。env.step: servo = turn_mid + a·span + offset
        _sv, _dg = zip(*self.env.cp.steer_curve)
        self._a_grid = (np.asarray(_sv, float) - self.env.cp.turn_mid - self.pl.servo_offset) \
            / self.env.cp.turn_span
        self._d_grid = np.radians(np.asarray(_dg, float)) * self.pl.steer_gain

    def _action_to_delta(self, a):
        return np.interp(a, self._a_grid, self._d_grid)

    # ---- 速度指令:老师逻辑 + 有效抓地调速器 ----
    GRIP_EFF = 0.0             # 摩擦圆 tanh 后的有效抓地;0 = 特权自适应 pl.grip·GE_FRAC(默认,稳健)
    GE_FRAC = 0.95             # 自适应模式:ge_eff = pl.grip·GE_FRAC(老师特权,按该局真实抓地;0.95 实测稳健上限)
    WIN_T = 0.20               # 调速器前瞻时长 s
    WIN_D = 0.9               # 调速器前瞻距离 m
    K_MARGIN = 1.0            # 曲率安全系数:走线平滑后的 κ 低估 S 弯反向段的几何需求
    TD_COMP = True             # 弯中掉速补偿:指令按 (1-turn_drag·δ²) 抬高,让掉完速的实际车速 = 抓地上限
    L_REC = 2.0               # 航向亏损恢复长度 m:ψ/L_REC 折算成额外曲率需求
    PSI_DB = 0.05             # 航向亏损死区 rad(~3°),弯中常态滞后不罚

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
        cones = self.env.road.cones
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
        steer_a = self._mpc(min(v_cmd, self.pl.cap))
        self.last_steer_a = steer_a
        if v_cmd == BRAKE_CMD:
            speed_a = speed_to_action(BRAKE_CMD, self.pl.enc)
        else:
            speed_a = speed_to_action(v_cmd, self.pl.enc)
        return np.array([steer_a, speed_a], np.float32)


def wrap_pi_arr(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def run(job):
    name, seed, cap, a_lat, gfb, grip_eff, win_t, adapt = job
    MpcTeacher.GRIP_EFF, MpcTeacher.WIN_T = grip_eff, win_t
    road, lap_len = official(name, True)
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
    P = plan(road, 0, 0.0, CarParams().width, pl, a_lat=a_lat)
    if isinstance(P, str):
        return dict(track=name, seed=seed, cap=cap, ok=False, why=P, laps=[], fails=0)
    env = RoadEnv(road, pl, seed, noise=False, sense=False)
    t = MpcTeacher(env, P, a_lat=a_lat)
    if grip_eff <= 0.0:
        t.ge_eff = pl.grip * MpcTeacher.GE_FRAC        # 按该局真实抓地自适应
    why, tl = None, []
    while why is None:
        why = env.step(t())
        if road.s[env.idx] >= lap_len * (len(tl) + 1) and len(tl) < LAPS:
            tl.append(env.t)
    return dict(track=name, seed=seed, cap=cap, ge=grip_eff, wt=win_t, ad=adapt,
                ok=why == 'goal', why=why,
                laps=[tl[0]] + list(np.diff(tl)) if tl else [], fails=t.fails)


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
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    RT.A_ACC = a.a_acc
    RT.set_decel(a.a_dec, a.a_brk)
    jobs = [(t, a.seed_start + i, cap, a.a_lat, a.grip_fb if a.grip_fb > 0 else None, ge, wt, ad)
            for t in a.tracks for cap in a.caps for ge in a.grip_effs for wt in a.win_ts
            for ad in a.adapts for i in range(a.count)]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run, jobs))
    summary = {}
    for t in a.tracks:
        for cap in a.caps:
            for ge in a.grip_effs:
                for wt in a.win_ts:
                    for ad in a.adapts:
                        rs = [r for r in rows if r['track'] == t and r['cap'] == cap
                              and r['ge'] == ge and r['wt'] == wt and r['ad'] == ad]
                        if not rs:
                            continue
                        ok = [r for r in rs if r['ok']]
                        reasons = {}
                        for r in rs:
                            reasons[r['why']] = reasons.get(r['why'], 0) + 1
                        summary[f'{t} cap{cap:.1f} ge{ge:g} wt{wt:g} ad{ad:g}'] = dict(
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
