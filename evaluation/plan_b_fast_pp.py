"""方案B第二步:高速跟踪器调参扫描(纯追踪 lookahead 系数 + 曲率前馈)。

第一步结论:满抓地(7.85)+cap5 的速度剖面理论圈时 8.3-8.7s,但现有纯追踪跟踪器
在 S 弯 3.2 m/s 以上切内侧 apex 锥桶(15.5s 实测 vs 8.5s 剖面,差距全在跟踪)。

本脚本不动训练代码,子类化 Teacher 试三件事:
  1. lookahead 曲线 look = look_a + look_b·v(原 0.35+0.3v,4m/s 时仅 1.55m,S 弯反向太晚)
  2. 曲率前馈 ff·atan(L·κ_target) —— 治 0.28s 死区在 κ 每 3m 反向一次的 slalom 里的相位滞后
  3. 重规划窗口 horizon(原 8m,4m/s 以上刹车距离超窗,隐式压速度)

    python -m evaluation.plan_b_fast_pp --tracks A --caps 3.2 4.0 5.0 --a-lat 7.85 \
        --look-bs 0.3 0.45 0.6 --ffs 0 0.7 1.0 --count 4 --out runs/plan_b_pp.json
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
from training.road_teacher import (A_BRK_EST, BRAKE_FLOOR, BRAKE_MARGIN, REPLAN_DEV,
                                   REPLAN_EVERY, SPEED_MARGIN, Teacher, plan, set_grip_feedback)

ENC = 'spica_v3_s5_v0.45-5_d0.2_log'


class FastTeacher(Teacher):
    """纯追踪 + 可调 lookahead + 曲率前馈。类属性在 run() 里按任务设(fork 后各进程独立)。"""
    LOOK_A = 0.35
    LOOK_B = 0.30
    FF = 0.0
    PRED = 0.0            # 额外位姿预瞄时长(乘 dead_time,0=原版只补 lag 半拍)

    def __call__(self):
        env, car = self.env, self.env.car
        if self.replan:
            dd = self.plan.pts[max(0, self.j - 20):self.j + 60] - (car.x, car.y)
            dev = math.sqrt(float(np.min(np.einsum('ij,ij->i', dd, dd))))
            if self.n % REPLAN_EVERY == 0 or dev > REPLAN_DEV:
                self._replan()
        self.n += 1
        P, pl, cp = self.plan, self.pl, env.cp
        t = CONTROL_DT * (pl.lag + 0.5) + self.PRED * pl.dead_time
        v = max(car.v, 0.3)
        yaw = car.yaw + car.yaw_rate * t
        x = car.x + v * math.cos(yaw) * t
        y = car.y + v * math.sin(yaw) * t
        lo, hi = max(0, self.j - 20), min(len(P.pts), self.j + 60)
        dd = P.pts[lo:hi] - (x, y)
        self.j = lo + int(np.argmin(np.einsum('ij,ij->i', dd, dd)))
        look = self.LOOK_A + self.LOOK_B * v
        k = min(len(P.pts) - 1, int(np.searchsorted(P.s, P.s[self.j] + look)))
        tx, ty = P.pts[k]
        alpha = math.atan2(ty - y, tx - x) - yaw
        ld = max(math.hypot(tx - x, ty - y), 0.2)
        delta = math.atan2(2 * cp.wheelbase * math.sin(alpha), ld) / pl.steer_gain
        if self.FF:
            delta += self.FF * math.atan(cp.wheelbase * P.kappa[k]) / pl.steer_gain
        servo = steer_to_turn_float(delta, cp) - pl.servo_offset
        steer_a = max(-1.0, min(1.0, (servo - cp.turn_mid) / cp.turn_span))
        ahead = P.s[self.j] + v * (pl.dead_time + CONTROL_DT * (pl.lag + 1)) + SPEED_MARGIN
        kv = min(len(P.v) - 1, int(np.searchsorted(P.s, ahead)))
        q = env.speed.queue
        v_eff = car.v - A_BRK_EST * PHYSICS_DT * sum(1 for c in q if c < 0.0)
        if (P.v_coast is not None and car.v > self.release
                and v_eff > max(P.v_coast[kv], BRAKE_FLOOR) + BRAKE_MARGIN):
            return np.array([steer_a, speed_to_action(BRAKE_CMD, env.pl.enc)], np.float32)
        v_plan = float(P.v[kv])
        if RT.GRIP_FB is not None:
            tt = math.tan(abs(car.steer))
            if tt > 1e-3:
                v_plan = min(v_plan, max(BRAKE_FLOOR, math.sqrt(RT.GRIP_FB * cp.wheelbase / tt) - pl.v_over))
        return np.array([steer_a, speed_to_action(v_plan, env.pl.enc)], np.float32)


def run(job):
    name, seed, cap, a_lat, gfb, lb, ff, pred = job
    set_grip_feedback(gfb)
    FastTeacher.LOOK_B, FastTeacher.FF, FastTeacher.PRED = lb, ff, pred
    road, lap_len = official(name, True)
    pl = replace(Plant.sample(np.random.default_rng(seed), spec_of(ENC)), cap=cap)
    P = plan(road, 0, 0.0, CarParams().width, pl, a_lat=a_lat)
    if isinstance(P, str):
        return dict(track=name, seed=seed, cap=cap, ok=False, why=P, laps=[], fails=0)
    env = RoadEnv(road, pl, seed, noise=False, sense=False)
    t = FastTeacher(env, P, a_lat=a_lat)
    why, tl = None, []
    while why is None:
        why = env.step(t())
        if road.s[env.idx] >= lap_len * (len(tl) + 1) and len(tl) < LAPS:
            tl.append(env.t)
    return dict(track=name, seed=seed, cap=cap, lb=lb, ff=ff, pred=pred, ok=why == 'goal', why=why,
                laps=[tl[0]] + list(np.diff(tl)) if tl else [], fails=t.fails)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tracks', nargs='+', default=['A'])
    ap.add_argument('--caps', type=float, nargs='+', default=[3.2, 4.0])
    ap.add_argument('--a-lat', type=float, default=7.85)
    ap.add_argument('--grip-fb', type=float, default=3.0)
    ap.add_argument('--look-bs', type=float, nargs='+', default=[0.3, 0.45, 0.6])
    ap.add_argument('--ffs', type=float, nargs='+', default=[0.0, 0.7, 1.0])
    ap.add_argument('--preds', type=float, nargs='+', default=[0.0],
                    help='额外位姿预瞄 = pred×dead_time(死区补偿)')
    ap.add_argument('--count', type=int, default=3)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--seed-start', type=int, default=96_100_000)
    ap.add_argument('--horizon', type=float, default=None, help='重规划窗口(默认 road_teacher.HORIZON)')
    ap.add_argument('--a-acc', type=float, default=1.5)
    ap.add_argument('--a-dec', type=float, default=0.7)
    ap.add_argument('--a-brk', type=float, default=1.3)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    RT.A_ACC = a.a_acc
    RT.set_decel(a.a_dec, a.a_brk)
    if a.horizon is not None:
        RT.HORIZON = a.horizon
    jobs = [(t, a.seed_start + i, cap, a.a_lat, a.grip_fb if a.grip_fb > 0 else None, lb, ff, pred)
            for t in a.tracks for cap in a.caps for lb in a.look_bs for ff in a.ffs
            for pred in a.preds for i in range(a.count)]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run, jobs))
    summary = {}
    for t in a.tracks:
        for cap in a.caps:
            for lb in a.look_bs:
                for ff in a.ffs:
                    for pred in a.preds:
                        rs = [r for r in rows if r['track'] == t and r['cap'] == cap
                              and r['lb'] == lb and r['ff'] == ff and r['pred'] == pred]
                        if not rs:
                            continue
                        ok = [r for r in rs if r['ok']]
                        reasons = {}
                        for r in rs:
                            reasons[r['why']] = reasons.get(r['why'], 0) + 1
                        summary[f'{t} cap{cap:.1f} lb{lb:g} ff{ff:g} pd{pred:g}'] = dict(
                            ok=f'{len(ok)}/{len(rs)}', reasons=reasons,
                            lap2=round(float(np.mean([r['laps'][1] for r in ok])), 2) if ok else None,
                            fails_mean=round(float(np.mean([r['fails'] for r in rs])), 1))
    out = dict(jobs=len(jobs), summary=summary)
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
    print(json.dumps(out, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
