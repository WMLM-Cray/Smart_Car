"""bev_v1 模型在官方赛道 A-D 上跑(和 road_track 同口径:黄 V 补桶、START 起步、连跑 2 圈)。

    python -m evaluation.bev_track --model runs/bev_v1_a/candidate_030.npz --caps 0.5 0.7 1.0 1.5 1.8 --count 16
    # 按车上条件(截停、无 YOLO、地面分割 3 Hz / 0.35 s 车上投影 + 补缝),赛道 A 不补黄 V:
    python -m evaluation.bev_track --model m.npz --tracks A --caps 0.7 --count 32 --car --no-fill-v
    # 实车 A 赛道原始布局(maps/realA_1020.yaml,10-20 seg_04 雷达图抠桶):
    python -m evaluation.bev_track --model m.npz --tracks realA_1020 --caps 1.2 --count 32 --car
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
from evaluation.road_track import LAPS, official, real_track
from simulation.fastsim.road_env import Plant
from training.bev_env import BevEnv


CAR = dict(guard=True, yolo=False, seg_lut=True, seg_fill=True, seg_period=0.33, seg_lag=0.35)     # 同 l_test / bev_testset 的部署参数


def perturb(road, rng, jitter, drop):
    """实车摆桶和规则图不一样:每个桶位置加高斯抖动 jitter m,再按概率 drop 拿掉(大空档压力测试)。出界仍按规则图的区域判。"""
    if not (jitter or drop):
        return road
    c = road.cones + rng.normal(0, jitter, road.cones.shape)
    return replace(road, cones=c[rng.random(len(c)) >= drop])


def run(args):
    model, name, seed, cap, car, fill_v, jitter, drop, escape, no_seg = (*args, False, False)[:10]
    road, lap_len = official(name, fill_v) if name in 'ABCD' else real_track(name)    # 其他名字 = maps/<name>.yaml 实车赛道
    road = perturb(road, np.random.default_rng(seed + 7), jitter, drop)
    pol = BevPolicy(model)
    pl = replace(Plant.sample(np.random.default_rng(seed), pol.enc), cap=cap)
    kw = ({**CAR, 'guard': 'escape'} if escape else CAR) if car else {}
    kw = {**kw, 'no_seg': True} if no_seg else kw
    benv = BevEnv(road, pl, seed, noise=True, free=pol.free, near=pol.near, **kw)
    laps, why = [], None
    while why is None:
        why = benv.step(pol(*benv.observation()))
        if road.s[benv.env.idx] >= lap_len * (len(laps) + 1) and len(laps) < LAPS:
            laps.append(benv.env.t)
    car_ = benv.env.car
    return dict(track=name, seed=seed, cap=cap, reason=why, n_escape=benv.n_escape,
                laps=[laps[0]] + list(np.diff(laps)) if laps else [], time=benv.env.t,
                s_end=float(road.s[benv.env.idx] % lap_len), xy_end=(float(car_.x), float(car_.y)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', required=True)
    ap.add_argument('--caps', type=float, nargs='+', default=[0.5, 0.7, 1.0, 1.5, 1.8])
    ap.add_argument('--count', type=int, default=16)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--seed-start', type=int, default=95_000_000)
    ap.add_argument('--tracks', nargs='+', default=['A', 'B', 'C', 'D'])
    ap.add_argument('--out', default=None)
    ap.add_argument('--car', action='store_true', help='按车上条件跑(见 CAR)')
    ap.add_argument('--guard-escape', action='store_true', help="配 --car:截停先找逃生转角(BevEnv guard='escape')")
    ap.add_argument('--no-seg', action='store_true', help='纯雷达模型(bev_train --no-seg 训的):FREE 层恒空,同车上 road_car_node --no-seg')
    ap.add_argument('--no-fill-v', action='store_true', help='黄 V 不补桶(按规则只有胶带)')
    ap.add_argument('--cone-jitter', type=float, default=0.0, help='每局桶位置高斯抖动 m')
    ap.add_argument('--cone-drop', type=float, default=0.0, help='每局每个桶被拿掉的概率')
    a = ap.parse_args()
    jobs = [(a.model, t, a.seed_start + i, cap, a.car, not a.no_fill_v, a.cone_jitter, a.cone_drop, a.guard_escape, a.no_seg)
            for t in a.tracks for cap in a.caps for i in range(a.count)]
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
                lap2=float(np.mean([r['laps'][1] for r in ok])) if ok else None,
                escapes_per_ep=float(np.mean([r['n_escape'] for r in rs])))
    print(json.dumps(dict(model=a.model, summary=summary), ensure_ascii=False, indent=1))
    if a.out:
        Path(a.out).write_text(json.dumps(dict(model=a.model, summary=summary, episodes=rows),
                                          ensure_ascii=False, indent=1), encoding='utf-8')


if __name__ == '__main__':
    main()
