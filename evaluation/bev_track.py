"""bev_v1 模型在官方赛道 A-D 上跑(和 road_track 同口径:黄 V 补桶、START 起步、连跑 2 圈)。

    python -m evaluation.bev_track --model runs/bev_v1_a/candidate_030.npz --caps 0.5 0.7 1.0 1.5 1.8 --count 16
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

from common.bev_policy import BevPolicy
from evaluation.road_track import LAPS, official
from simulation.fastsim.road_env import Plant
from training.bev_env import BevEnv


def run(args):
    model, name, seed, cap = args
    road, lap_len = official(name, True)
    pol = BevPolicy(model)
    pl = replace(Plant.sample(np.random.default_rng(seed), pol.enc), cap=cap)
    benv = BevEnv(road, pl, seed, noise=True, free=pol.free, near=pol.near)
    laps, why = [], None
    while why is None:
        why = benv.step(pol(*benv.observation()))
        if road.s[benv.env.idx] >= lap_len * (len(laps) + 1) and len(laps) < LAPS:
            laps.append(benv.env.t)
    return dict(track=name, seed=seed, cap=cap, reason=why,
                laps=[laps[0]] + list(np.diff(laps)) if laps else [], time=benv.env.t)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', required=True)
    ap.add_argument('--caps', type=float, nargs='+', default=[0.5, 0.7, 1.0, 1.5, 1.8])
    ap.add_argument('--count', type=int, default=16)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--seed-start', type=int, default=95_000_000)
    ap.add_argument('--tracks', nargs='+', default=['A', 'B', 'C', 'D'])
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    jobs = [(a.model, t, a.seed_start + i, cap) for t in a.tracks for cap in a.caps for i in range(a.count)]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run, jobs))
    summary = {}
    for t in a.tracks:
        for cap in a.caps:
            rs = [r for r in rows if r['track'] == t and r['cap'] == cap]
            ok = [r for r in rs if r['reason'] == 'goal']
            summary[f'{t} {cap:.2f}'] = dict(
                student=len(ok) / len(rs),
                reasons={k: sum(r['reason'] == k for r in rs) for k in {r['reason'] for r in rs}},
                lap1=float(np.mean([r['laps'][0] for r in ok])) if ok else None,
                lap2=float(np.mean([r['laps'][1] for r in ok])) if ok else None)
    print(json.dumps(dict(model=a.model, summary=summary), ensure_ascii=False, indent=1))
    if a.out:
        Path(a.out).write_text(json.dumps(dict(model=a.model, summary=summary, episodes=rows),
                                          ensure_ascii=False, indent=1), encoding='utf-8')


if __name__ == '__main__':
    main()
