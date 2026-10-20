"""方案C「第二圈」仿真:第一圈低速探索,第二圈先验速度规划+安全监督竞速,终点框滑行停车。

    python -m evaluation.road_lap2 --model runs/road_v2.5_td3/candidate_004.npz \
        --tracks A B C D --a-lat-sup 1.0 --count 30 --out runs/eval_final/lap2.json

按 docs/HW_SchemeC_MPC_RL_Detailed_Plan.md 的第二圈流程在 FastSim 上验证:
  第一圈  低速(cap1)跑完 —— 对应"探索建图圈",这里地图本就是先验,只计时间
  第二圈  先验速度规划压顶模型输出(方案 C1 的安全监督形态):
            v_plan = min( cap, sqrt(a_lat_sup/|κ|), sqrt(2·decel·(s_stop-s)) )
          曲率限速(横向摩擦假设)+滑行制动曲线(纵向摩擦实测 0.73)+前/后向传播
  停车    v_plan 低于最低指令速度后松油(cmd=0),按本局真实 decel 滑行到停,
          停稳判定 v<0.02 持续 0.5s(方案 12.1 的 FINISH_APPROACH→STOP_HOLD)

摩擦的两处体现:
  横向  a_lat_sup 是监督器的弯道限速假设(车物理上限 7.85=0.8g 不变),档位越低越保守
  纵向  规划用名义 decel(默认 0.73),实际每局 Plant.decel∈[0.68,0.78] 随机 ——
        停车误差分布即"摩擦估计不准"的真实后果,如实报告

停车目标:START 框(发车区黄框,第二圈过线处)。official() 把中心线起点滚到框中间,
s=2·lap_len 即框中心;STOP_OFFSET 再往后放一点,留出车头与滑距。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import json
import math
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

from common.params import CarParams
from common.road_policy import DEFAULT_ENC, RoadPolicy, action_to_speed, speed_to_action, spec_of
from evaluation.road_track import _enc, _schema, official
from simulation.fastsim.road import DS
from simulation.fastsim.road_env import PHYSICS_DT, Plant, RoadEnv
from simulation.fastsim.vehicle import Car
from common.pwm import turn_to_steer
from training.road_teacher import A_DEC, curvature

STOP_OFFSET = 0.05        # 停车目标点在框中心(过线处)之后的距离 m;框半长 0.57。
                          # 松油点在 s_target-~0.14(v_plan 低于 v_lo 处),按本局真实 decel
                          # 滑 0.36~0.52m(冒烟实测),0.05+0.4≈0.45,留 0.12m 出框余量
CREEP_MARGIN = 0.02       # v_plan 低于 v_lo+这个值就松油
HOLD_V = 0.02             # 停稳判定:低于此速度
HOLD_T = 0.5              # 并持续这么久 s
PARK_BUDGET = 12.0        # 停车段时间上限 s
A_ACC = 0.4               # 前向传播加速 m/s²(与老师一致)


def v_profile(road, lap_len: float, s_target: float, a_lat_sup: float | None,
              v_cap: float, v_top: float, v_start: float, decel_nom: float):
    """第二圈的速度上限曲线(先验地图 + 摩擦假设)。返回 (s_grid, v_grid) 供插值。"""
    i_lo, i_hi = int((lap_len - 1.0) / DS), int((s_target + 1.0) / DS)
    i_hi = min(i_hi, len(road.s) - 1)
    pts = road.centre[i_lo:i_hi]
    s = road.s[i_lo:i_hi]
    if a_lat_sup is None:
        v = np.full(len(s), min(v_cap, v_top))
    else:
        kap = curvature(pts, gap=6)       # 0.3m 基线,和老师一致
        v = np.minimum(min(v_cap, v_top),
                       np.sqrt(a_lat_sup / np.maximum(np.abs(kap), 1e-6)))
    v[0] = min(v[0], v_start)
    for i in range(1, len(v)):            # 前向:加速上限
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * A_ACC * (s[i] - s[i - 1])))
    for i in range(len(v) - 2, -1, -1):   # 后向:制动上限(保守 0.45)
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * A_DEC * (s[i + 1] - s[i])))
    # 停车曲线:距目标点 s_target 剩余的距离只允许滑行可到的速度
    v = np.minimum(v, np.sqrt(np.maximum(0.0, 2.0 * decel_nom * (s_target - s))))
    return s, v


def run(args):
    """一局:同一条车(同 Plant)跑两圈+停车。args=(model, track, seed, cap1, cap2, a_lat_sup, decel_nom[, record]):
    record=True 时每 0.1s 记 [t,x,y,v,s,lap] 进 out['trace'](供走线动画用)。"""
    record = len(args) > 7 and args[7]   # args=(model,track,seed,cap1,cap2,a_lat,decel[,record])
    model, name, seed, cap1, cap2, a_lat_sup, decel_nom = args[:7]
    road, lap_len = official(name, True, laps=3)      # 3 圈长:跑 2 圈+停车余量,不触发 goal
    rng = np.random.default_rng(seed)
    enc = _enc(model)
    pl = replace(Plant.sample(rng, enc), cap=cap1)
    env = RoadEnv(road, pl, seed, noise=True)
    pol = RoadPolicy(model)
    out = dict(track=name, seed=seed, cap1=cap1, cap2=cap2, a_lat_sup=a_lat_sup,
               v_plateau=pl.v_plateau, v_top=pl.v_top, decel=pl.decel)

    s_target = 2.0 * lap_len + STOP_OFFSET
    vg_s = vg_v = None      # 第二圈速度曲线在过线时再算(v_start 衔接第一圈末速)

    def act_capped(v_plan_now: float | None):
        a = pol(env.observation())
        if v_plan_now is not None:
            v_cmd = min(action_to_speed(float(a[1]), enc), v_plan_now)
            return [float(a[0]), speed_to_action(v_cmd, enc)]
        return a

    laps, why, v_end_lap1 = [], None, None
    trace = [] if record else None      # [t, x, y, v, s, lap] 每 0.1s 一个点

    def tick(lap_now: int):
        if trace is not None and int(round(env.t * 10)) > len(trace):
            trace.append([env.t, env.car.x, env.car.y, env.car.v,
                          road.s[env.idx], lap_now])

    while why is None:
        s_now = road.s[env.idx]
        if s_now >= lap_len and len(laps) == 0:                 # 过线:第一圈结束
            laps.append(env.t)
            v_end_lap1 = env.car.v
            env.pl.cap = cap2                                   # 第二圈放开速度上限
            vg_s, vg_v = v_profile(road, lap_len, s_target, a_lat_sup,
                                   cap2, pl.v_top, max(v_end_lap1, 0.3), decel_nom)
        if s_now >= 2.0 * lap_len and len(laps) == 1:           # 第二圈过线(计时点)
            laps.append(env.t - laps[0])                        # 第二圈纯用时
        tick(len(laps))
        v_plan_now = (float(np.interp(s_now, vg_s, vg_v))
                      if vg_s is not None and s_now <= s_target else None)
        near_stop = s_now >= s_target - 0.5                     # 只有终点附近才允许进停车段
        # 必须先记完第二圈过线(len(laps)==2)才允许松油:停车曲线把松油点压到过线点
        # 之前时,车会以 v_lo 蠕行过线再停(晚 ~0.1m),不能没过线就 break
        if (len(laps) == 2 and near_stop and v_plan_now is not None
                and v_plan_now <= enc.v_lo + CREEP_MARGIN):
            break                                               # 进入停车段
        why = env.step(act_capped(v_plan_now))
    if why is not None:                                         # 两圈内就撞/出界/超时
        out.update(reason=why, laps=laps + [None] * (2 - len(laps)))
        return out
    if len(laps) < 2:                                           # 兜底:松油前没检测到过线
        laps.append(env.t - laps[0])

    # ---- 停车段:舵机回正 + 松油滑行(cmd=0),按本局真实 decel 减速 ----
    servo = env.cp.turn_mid + pl.servo_offset
    steer = turn_to_steer(servo, env.cp)
    hold, t_park = 0.0, 0.0
    while hold < HOLD_T and t_park < PARK_BUDGET:
        v = env.speed.step(0.0)                                 # 松油(经 dead_time 队列)
        env.car.step(v, steer, PHYSICS_DT)
        env.t += PHYSICS_DT
        t_park += PHYSICS_DT
        if env._check() is not None:                            # 停车段撞/出界也算失败
            out.update(reason='park_fail', laps=laps, time=env.t)
            return out
        hold = hold + PHYSICS_DT if env.car.v < HOLD_V else 0.0
        tick(2)

    # ---- 停车指标:车体中心 vs 框中心(过线点),沿赛道方向误差 ----
    i, d = road.project((env.car.x, env.car.y), env.idx)
    box_c = road.centre[int(round(2.0 * lap_len / DS)) % len(road.centre)]
    along = road.s[i] - 2.0 * lap_len                          # 沿赛道误差(停车点-框中心)
    lat_err = float(d)
    if trace is not None:
        tick(2)
        out.update(trace=trace)
    out.update(reason='goal', laps=laps, time=env.t,
               park_time=t_park, stop_v=float(env.car.v),
               stop_s=float(road.s[i]), stop_along=float(along),
               stop_lat=lat_err, stop_xy=[float(env.car.x), float(env.car.y)],
               box_xy=[float(box_c[0]), float(box_c[1])],
               in_box=bool(abs(along) <= 0.57 and abs(lat_err) <= 0.57),
               heading_err=float((env.car.yaw - road.heading[i] + math.pi)
                                 % (2 * math.pi) - math.pi),
               min_clear=float(env.min_clear))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', required=True)
    ap.add_argument('--tracks', nargs='+', default=['A', 'B', 'C', 'D'])
    ap.add_argument('--cap1', type=float, default=0.5, help='第一圈(探索)速度上限')
    ap.add_argument('--cap2', type=float, default=1.0, help='第二圈(竞速)速度上限')
    ap.add_argument('--a-lat-sup', nargs='+', default=[None],
                    help='监督器弯道横向加速度假设 m/s²;多项就逐档扫描,none=不压顶')
    ap.add_argument('--decel-nom', type=float, default=0.42,
                    help='停车规划用的名义滑行减速度;实际能力 0.68~0.78,但指令有 '
                         '0.35s 死区滞后,高速档按 0.42 规划才停得进框(cap1.0 时 0.73 也够)')
    ap.add_argument('--count', type=int, default=30)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--seed-start', type=int, default=96_000_000)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    enc = _enc(a.model)
    if max(a.cap1, a.cap2) > enc.v_hi:
        raise SystemExit(f'速度上限超出契约 {enc}')
    lats = [None if x is None or str(x).lower() == 'none' else float(x) for x in a.a_lat_sup]
    jobs = [(a.model, t, a.seed_start + i, a.cap1, a.cap2, lat, a.decel_nom)
            for t in a.tracks for lat in lats for i in range(a.count)]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run, jobs))
    summary = {}
    for t in a.tracks:
        for lat in lats:
            key = f'{t} a_lat_sup={"无" if lat is None else lat}'
            rs = [r for r in rows if r['track'] == t and r['a_lat_sup'] == lat]
            ok = [r for r in rs if r['reason'] == 'goal']
            summary[key] = dict(
                finish=len(ok) / len(rs),
                reasons={k: sum(r['reason'] == k for r in rs) for k in {r['reason'] for r in rs}},
                lap1=float(np.mean([r['laps'][0] for r in ok])) if ok else None,
                lap2=float(np.mean([r['laps'][1] for r in ok if len(r['laps']) > 1])) if ok else None,
                total=float(np.mean([r['laps'][0] + r['laps'][1] for r in ok
                                     if len(r['laps']) > 1])) if ok else None,
                park_time=float(np.mean([r['park_time'] for r in ok])) if ok else None,
                stop_along_mean=float(np.mean([r['stop_along'] for r in ok])) if ok else None,
                stop_along_absmax=float(np.max(np.abs([r['stop_along'] for r in ok]))) if ok else None,
                stop_lat_absmax=float(np.max(np.abs([r['stop_lat'] for r in ok]))) if ok else None,
                in_box=sum(r['in_box'] for r in ok) / len(ok) if ok else None)
    print(json.dumps(dict(model=a.model, cap1=a.cap1, cap2=a.cap2, decel_nom=a.decel_nom,
                          summary=summary), ensure_ascii=False, indent=1))
    if a.out:
        Path(a.out).write_text(json.dumps(dict(
            model=a.model, cap1=a.cap1, cap2=a.cap2, decel_nom=a.decel_nom,
            summary=summary, episodes=rows), ensure_ascii=False, indent=1), encoding='utf-8')


if __name__ == '__main__':
    main()
