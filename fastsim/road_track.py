"""把比赛赛道(目前只有 A)转成路段环境的 Road,跑速度 x 通过性测试。

    python -m fastsim.road_track --model runs/road_v2_a/candidate_050.npz --workers 14
    python -m fastsim.road_track --model runs/low_speed_a/candidate_060.npz     # 旧模型对照

几何单源 3Dsim/examples/trackA_layout.yaml:外/内边界各 10 段,按段一一配对,中心线取
两边对应点的中点。锥桶用 gen_trackA_cones.py 同一套排法(直道 0.5m、弯 0.4m,2026-09-22
用户看比赛录像定的);黄 V 四段(E3/E4/I3/I4)按规则铺胶带不放锥桶 —— 这里就是两侧都没有边界。

一局 = 从 layout 的 start_pose 静止起步,连跑 LAPS 圈。记每圈用时:第 1 圈含起步。
通过性 = 老师在同一局(同样的车辆参数)能不能跑完;学生 = 待测模型。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import importlib.util
import json
import math
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

from . import low_speed_policy as v1
from .params import CarParams
from .road import CONE, DS, NONE, Road
from .road_env import Plant, RoadEnv
from .road_policy import RoadPolicy, speed_to_action
from .road_teacher import Teacher, plan

ROOT = Path(__file__).resolve().parents[1]
LAYOUT = ROOT / '3Dsim/examples/trackA_layout.yaml'
LAPS = 2
SPACING, CURVE_SPACING = 0.5, 0.4


def _gen():
    spec = importlib.util.spec_from_file_location('gen_trackA_cones', ROOT / '3Dsim/tools/gen_trackA_cones.py')
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def track_a(laps: int = LAPS) -> tuple[Road, float]:
    """返回 (Road, 一圈长度)。Road 的中心线是 laps 圈再多 1.5m,锥桶只有一圈的。"""
    lay = yaml.safe_load(LAYOUT.read_text(encoding='utf-8'))
    g = _gen()
    outer, inner = lay['boundaries']['outer'], lay['boundaries']['inner']
    v_segs = set(lay['yellow_v_band']['no_cone_segments'])
    cen, half, mat = [], [], []
    for eo, ei in zip(outer, inner):
        _, lo = g.segment_points(eo, 1.0)
        _, li = g.segment_points(ei, 1.0)
        n = max(2, int(math.ceil(max(lo, li) / DS)))
        po, _ = g.segment_points(eo, max(lo, 1e-6) / n)
        pi, _ = g.segment_points(ei, max(li, 1e-6) / n)
        po, pi = np.array(po[:-1]), np.array(pi[:-1])
        k = min(len(po), len(pi))
        po, pi = po[:k], pi[:k]
        cen.append((po + pi) / 2)
        half.append(np.hypot(*(po - pi).T) / 2)
        m = NONE if eo['seg'] in v_segs else CONE
        mat.append(np.full(k, m))
    cen, half, mat = np.concatenate(cen), np.concatenate(half), np.concatenate(mat)
    # 左端方角处两条边界配对出来的中心线是个尖折点,沿法向偏移会自交。按周期高斯平滑
    # (sigma 0.5m)把折点磨圆,半宽改成到两侧边界折线的较小距离(偏保守)。
    k = np.arange(-30, 31)
    w = np.exp(-0.5 * (k * DS / 0.5) ** 2)
    w /= w.sum()
    idx = (np.arange(len(cen))[:, None] + k[None, :]) % len(cen)
    cen = np.einsum('ijk,j->ik', cen[idx], w)
    edges = []
    for items in (outer, inner):
        pts = np.concatenate([np.array(g.segment_points(it, 0.05)[0]) for it in items])
        edges.append(pts)
    half = np.minimum(*(np.min(np.hypot(cen[:, None, 0] - e[None, :, 0],
                                        cen[:, None, 1] - e[None, :, 1]), axis=1) for e in edges))
    # 起点挪到 start_pose(A 在底边直道 11.0, 0.75,朝 +x)
    sx, sy = lay['start_pose']['xy']
    i0 = int(np.argmin(np.hypot(cen[:, 0] - sx, cen[:, 1] - sy)))
    cen, half, mat = (np.roll(a, -i0, axis=0) for a in (cen, half, mat))
    step = np.hypot(*np.diff(np.vstack([cen, cen[:1]]), axis=0).T)
    lap_len = float(step.sum())
    extra = int(1.5 / DS)
    reps = np.r_[np.tile(np.arange(len(cen)), laps), np.arange(extra)]
    c = cen[reps]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(c, axis=0).T))])
    d = np.gradient(c, axis=0)
    heading = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    road = Road(c, heading, s, half[reps], mat[reps], mat[reps], elems=[dict(kind='trackA')])
    # 锥桶直接按 gen_trackA_cones 的排法(和 Gazebo 那边同源),不走 _boundary_objects
    cones = []
    for items in (outer, inner):
        cones += [(x, y) for _, x, y in g.boundary_cones(items, SPACING, CURVE_SPACING, False)]
    road.cones = np.array(cones, float)
    road.walls = np.zeros((0, 2, 2))
    return road, lap_len


def _schema(path):
    with np.load(path) as z:
        return str(z['schema'])


def run(args):
    model, seed, cap = args
    road, lap_len = track_a()
    rng = np.random.default_rng(seed)
    pl = replace(Plant.sample(rng), cap=cap)
    out = dict(seed=seed, cap=cap, v_plateau=pl.v_plateau, v_top=pl.v_top)
    # 通过性:老师用同一组车辆参数跑
    P = plan(road, 0, 0.0, CarParams().width, pl)
    if isinstance(P, str):
        out['teacher'] = P
    else:
        env = RoadEnv(road, pl, seed, noise=False, sense=False)
        t = Teacher(env, P)
        why = None
        while why is None:
            why = env.step(t())
        out['teacher'] = why
        out['teacher_clear'] = float(env.min_clear)
    # 学生
    env = RoadEnv(road, pl, seed, noise=True)
    if model is None:
        return out
    if _schema(model) == v1.SCHEMA:
        pol, last = v1.SteeringPolicy(model), 0.0
        v_a = speed_to_action(cap)

        def act():
            nonlocal last
            last = pol(v1.observe(env.ranges, env.angles, env.car.v, cap, env.car.yaw_rate, last))
            return [last, v_a]
    else:
        pol = RoadPolicy(model)

        def act():
            return pol(env.observation())
    laps, why, speeds = [], None, []
    while why is None:
        why = env.step(act())
        speeds.append(env.car.v)
        if road.s[env.idx] >= lap_len * (len(laps) + 1) and len(laps) < LAPS:
            laps.append(env.t)
    out.update(reason=why, laps=[laps[0]] + list(np.diff(laps)) if laps else [],
               time=env.t, min_clear=float(env.min_clear), mean_speed=float(np.mean(speeds)),
               stop_s=float(road.s[env.idx]), stop_xy=[float(env.car.x), float(env.car.y)])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=None)
    ap.add_argument('--caps', type=float, nargs='+', default=[0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
    ap.add_argument('--count', type=int, default=30, help='每档速度上限跑几局(车辆参数每局随机)')
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--seed-start', type=int, default=95_000_000)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    jobs = [(a.model, a.seed_start + i, cap) for cap in a.caps for i in range(a.count)]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run, jobs))
    summary = {}
    for cap in a.caps:
        rs = [r for r in rows if r['cap'] == cap]
        ok = [r for r in rs if r.get('reason') == 'goal']
        summary[f'{cap:.2f}'] = dict(
            teacher=sum(r['teacher'] == 'goal' for r in rs) / len(rs),
            student=len(ok) / len(rs) if a.model else None,
            reasons={k: sum(r.get('reason') == k for r in rs) for k in {r.get('reason') for r in rs}},
            lap1=float(np.mean([r['laps'][0] for r in ok])) if ok else None,
            lap2=float(np.mean([r['laps'][1] for r in ok])) if ok else None,
            mean_speed=float(np.mean([r['mean_speed'] for r in ok])) if ok else None)
    print(json.dumps(dict(model=a.model, summary=summary), ensure_ascii=False, indent=1))
    if a.out:
        Path(a.out).write_text(json.dumps(dict(model=a.model, summary=summary, episodes=rows),
                                          ensure_ascii=False, indent=1), encoding='utf-8')


if __name__ == '__main__':
    main()
