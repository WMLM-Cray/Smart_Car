"""方案B第一步:全知老师天花板实测(官方赛道 A-D,2 圈,最小曲率走线 + 满抓地速度剖面)。

老师看得到整条路(training/road_teacher.plan:曲率平方和最小的走线,直角弯外-内-外),
速度 = sqrt(a_lat/R) 倒推滑行/刹车,纯追踪跟踪。--a-lat 推到抓地上限(7.85=0.8g),
--grip-fb 开推头反馈(按实际前轮转角限速,治"弯中推头撞外侧",2026-10-24 realA 实测)。

这是"如果地图完美、控制器完美"的圈速上界;可部署版(从 lap1 BEV 地图提取走廊)的
成绩只会更低。契约用 s5(0.45-5.0 m/s,带刹车),和 gripF3_c011 一致。

    python -m evaluation.plan_b_ceiling --tracks A B C D --a-lats 3 5 7.85 \
        --grip-fbs 3 5 --caps 3 4 5 --count 8 --out runs/plan_b_ceiling.json
"""
from __future__ import annotations

import argparse
import json
import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import numpy as np

from common.params import CarParams
from common.road_policy import spec_of
from evaluation.road_track import LAPS, official
from simulation.fastsim.road_env import Plant, RoadEnv
from training import road_teacher as RT
from training.road_teacher import Teacher, plan, set_grip_feedback

ENC = 'spica_v3_s5_v0.45-5_d0.2_log'


def run(job):
    name, seed, cap, a_lat, gfb = job
    set_grip_feedback(gfb)                       # 进程内全局,逐任务设
    road, lap_len = official(name, True)
    pl = replace(Plant.sample(np.random.default_rng(seed), spec_of(ENC)), cap=cap)
    P = plan(road, 0, 0.0, CarParams().width, pl, a_lat=a_lat)
    if isinstance(P, str):
        return dict(track=name, seed=seed, cap=cap, a_lat=a_lat, grip_fb=gfb,
                    teacher=P, laps=[], time=0.0, fails=0)
    env = RoadEnv(road, pl, seed, noise=False, sense=False)   # 全知:无传感仿真
    t = Teacher(env, P, a_lat=a_lat)
    why, tl = None, []
    while why is None:
        why = env.step(t())
        if road.s[env.idx] >= lap_len * (len(tl) + 1) and len(tl) < LAPS:
            tl.append(env.t)
    return dict(track=name, seed=seed, cap=cap, a_lat=a_lat, grip_fb=gfb,
                teacher=why, laps=[tl[0]] + list(np.diff(tl)) if tl else [],
                time=env.t, fails=t.fails)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tracks', nargs='+', default=['A', 'B', 'C', 'D'])
    ap.add_argument('--caps', type=float, nargs='+', default=[4.0])
    ap.add_argument('--a-lats', type=float, nargs='+', default=[5.0])
    ap.add_argument('--grip-fbs', type=float, nargs='+', default=[3.0],
                    help='推头反馈侧向加速度(0 或负数 = 关)')
    ap.add_argument('--count', type=int, default=8)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--seed-start', type=int, default=96_000_000)
    ap.add_argument('--a-acc', type=float, default=None, help='规划前向加速(默认 road_teacher.A_ACC)')
    ap.add_argument('--a-dec', type=float, default=None, help='规划滑行减速(默认 road_teacher.A_DEC)')
    ap.add_argument('--a-brk', type=float, default=None, help='规划刹车减速(默认 road_teacher.A_BRK)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    if a.a_acc is not None:
        RT.A_ACC = a.a_acc
    RT.set_decel(a.a_dec, a.a_brk)
    jobs = [(t, a.seed_start + i, cap, al, gfb if gfb > 0 else None)
            for t in a.tracks for cap in a.caps for al in a.a_lats
            for gfb in a.grip_fbs for i in range(a.count)]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run, jobs))
    summary = {}
    for t in a.tracks:
        for cap in a.caps:
            for al in a.a_lats:
                for gfb in a.grip_fbs:
                    rs = [r for r in rows if r['track'] == t and r['cap'] == cap
                          and r['a_lat'] == al and r['grip_fb'] == (gfb if gfb > 0 else None)]
                    if not rs:
                        continue
                    ok = [r for r in rs if r['teacher'] == 'goal']
                    reasons = {}
                    for r in rs:
                        reasons[r['teacher']] = reasons.get(r['teacher'], 0) + 1
                    summary[f'{t} cap{cap:.1f} alat{al:g} gfb{gfb:g}'] = dict(
                        ok=f'{len(ok)}/{len(rs)}', reasons=reasons,
                        lap1=round(float(np.mean([r['laps'][0] for r in ok])), 2) if ok else None,
                        lap2=round(float(np.mean([r['laps'][1] for r in ok])), 2) if ok else None,
                        fails_mean=round(float(np.mean([r['fails'] for r in rs])), 1))
    out = dict(jobs=len(jobs), summary=summary, rows=rows)
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
    print(json.dumps(dict(jobs=len(jobs), summary=summary), ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
