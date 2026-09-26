"""把官方赛道 A-D(maps/ 下规则图估读数据)转成路段环境的 Road,跑速度 x 通过性测试。

    python -m evaluation.road_track --model runs/road_v2_b/candidate_030.npz --tracks A B C D --fill-v
    python -m evaluation.road_track --model models/steering_v1/candidate_060.npz --tracks A    # 旧模型对照

数据(maps/,2026-09-25 从规则图 9 逐桶读图,见 docs/track/TRACK_AD_STATUS.md):
  A  trackA_cones.csv(71 桶逐点)+ trackA_layout.yaml 的黄 V 多边形
  B-D trackX_layout.json 的内外边界折线(同点数、一一配对)+ trackX_cones.csv
边界 = 外/内两条闭合折线(A 直接用锥桶连线,黄 V 处补上谷点);中心线取两边中点,
周期高斯平滑(sigma 0.5m)磨掉方角折点,半宽取到两侧边界的较小距离(偏保守)。

黄 V:按规则铺胶带不放锥桶。--fill-v 时沿边界把没有桶的地方按 0.5m 补上锥桶(测其余部分)。
一局 = 从 START 框中间静止起步,逆时针连跑 LAPS 圈。记每圈用时:第 1 圈含起步。
通过性 = 老师在同一局(同样的车辆参数)能不能跑完;学生 = 待测模型。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import csv
import json
import math
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from functools import lru_cache
from pathlib import Path

import numpy as np
import yaml

from common import low_speed_policy as v1
from common.params import CarParams
from simulation.road import CONE, DS, NONE, Road
from simulation.road_env import Plant, RoadEnv
from common.road_policy import RoadPolicy, speed_to_action
from training.road_teacher import Teacher, plan

ROOT = Path(__file__).resolve().parents[1]
MAPS = ROOT / 'maps'
LAPS = 2
FILL_SPACING = 0.5          # --fill-v 补桶间距
GAP = 0.8                   # 边界点离最近锥桶超过这么远 = 这里没桶(黄 V 胶带)


def _cones(name):
    with open(MAPS / f'track{name}_cones.csv', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    return rows


def _closed_dense(poly, step=DS):
    poly = np.vstack([poly, poly[:1]])
    seg = np.hypot(*np.diff(poly, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    t = np.arange(0, s[-1], step)
    return np.stack([np.interp(t, s, poly[:, 0]), np.interp(t, s, poly[:, 1])], axis=1)


def _boundaries(name):
    """(外边界折线, 内边界折线, 锥桶坐标, START 框 x 范围)"""
    rows = _cones(name)
    cones = np.array([[float(r['x']), float(r['y'])] for r in rows])
    if name == 'A':
        lay = yaml.safe_load((MAPS / 'trackA_layout.yaml').read_text(encoding='utf-8'))
        v = lay['yellow_v_band']
        polys = []
        for color, valley in (('blue', v['valley_outer']), ('red', v['valley_inner'])):
            pts = [(float(r['x']), float(r['y'])) for r in rows if r['color'] == color]
            out = []
            for i, p in enumerate(pts):             # 桶按逆时针排;黄 V 断口(>2m)处补谷点
                out.append(p)
                q = pts[(i + 1) % len(pts)]
                if math.dist(p, q) > 2.0:
                    out.append(tuple(valley))
            polys.append(np.array(out))
        start = lay['zones']['zone_start']['x']
        return polys[0], polys[1], cones, start
    lay = json.loads((MAPS / f'track{name}_layout.json').read_text(encoding='utf-8'))
    start = next(z['x_range_m'] for z in lay['zones'] if z['name'] == 'START')
    return (np.array(lay['boundaries']['outer']), np.array(lay['boundaries']['inner']),
            cones, start)


@lru_cache(maxsize=16)
def official(name: str, fill_v: bool = False, laps: int = LAPS) -> tuple[Road, float]:
    """返回 (Road, 一圈长度)。Road 的中心线是 laps 圈再多 1.5m,锥桶只有一圈的。"""
    outer, inner, cones, start = _boundaries(name)
    od, idn = _closed_dense(outer), _closed_dense(inner)
    if len(outer) == len(inner):                    # B-D:一一配对
        mid = (outer + inner) / 2
    else:                                           # A:内边界每点配最近的外边界点
        near = np.argmin(np.hypot(idn[:, None, 0] - od[None, :, 0],
                                  idn[:, None, 1] - od[None, :, 1]), axis=1)
        mid = (idn + od[near]) / 2
    cen = _closed_dense(mid)
    k = np.arange(-30, 31)
    w = np.exp(-0.5 * (k * DS / 0.5) ** 2)
    w /= w.sum()
    cen = np.einsum('ijk,j->ik', cen[(np.arange(len(cen))[:, None] + k[None, :]) % len(cen)], w)
    cen = _closed_dense(cen)
    area = 0.5 * np.sum(cen[:, 0] * np.roll(cen[:, 1], -1) - np.roll(cen[:, 0], -1) * cen[:, 1])
    if area < 0:                                    # 统一逆时针(底边直道朝 +x)
        cen = cen[::-1].copy()

    def dist(pts, edge):
        return np.min(np.hypot(pts[:, None, 0] - edge[None, :, 0], pts[:, None, 1] - edge[None, :, 1]), axis=1)
    half = np.minimum(dist(cen, od), dist(cen, idn))
    # 黄 V(没桶的边界段)
    bare = [e[dist(e, cones) > GAP] for e in (od, idn)]
    if fill_v:
        extra = [b[::int(round(FILL_SPACING / DS))] for b in bare]
        cones = np.vstack([cones] + [e for e in extra if len(e)])
        mat = np.full(len(cen), CONE)
    else:
        allbare = np.vstack([b for b in bare if len(b)]) if any(len(b) for b in bare) else np.zeros((0, 2))
        mat = np.where(len(allbare) and dist(cen, allbare) < half + 0.3, NONE, CONE)             if len(allbare) else np.full(len(cen), CONE)
    # 起点:START 框中间、底边直道
    xm = sum(start) / 2
    lowband = cen[:, 1] < cen[:, 1].min() + 0.5
    i0 = int(np.argmin(np.where(lowband, np.abs(cen[:, 0] - xm), np.inf)))
    cen, half, mat = (np.roll(x, -i0, axis=0) for x in (cen, half, np.asarray(mat)))
    lap_len = float(np.hypot(*np.diff(np.vstack([cen, cen[:1]]), axis=0).T).sum())
    reps = np.r_[np.tile(np.arange(len(cen)), laps), np.arange(int(1.5 / DS))]
    c = cen[reps]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(c, axis=0).T))])
    d = np.gradient(c, axis=0)
    heading = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    road = Road(c, heading, s, half[reps], mat[reps], mat[reps].copy(),
                elems=[dict(kind=f'track{name}', fill_v=fill_v)])
    road.cones = np.asarray(cones, float)
    road.walls = np.zeros((0, 2, 2))
    road.region = (np.asarray(outer, float), np.asarray(inner, float))
    return road, lap_len


def _schema(path):
    with np.load(path) as z:
        return str(z['schema'])


def run(args):
    model, name, seed, cap, fill_v = args
    road, lap_len = official(name, fill_v)
    rng = np.random.default_rng(seed)
    pl = replace(Plant.sample(rng), cap=cap)
    out = dict(track=name, seed=seed, cap=cap, v_plateau=pl.v_plateau, v_top=pl.v_top)
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
    ap.add_argument('--tracks', nargs='+', default=['A', 'B', 'C', 'D'])
    ap.add_argument('--fill-v', action='store_true', help='黄 V 也排锥桶(测黄 V 以外的部分)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    jobs = [(a.model, t, a.seed_start + i, cap, a.fill_v)
            for t in a.tracks for cap in a.caps for i in range(a.count)]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run, jobs))
    summary = {}
    for t, cap in ((t, cap) for t in a.tracks for cap in a.caps):
        rs = [r for r in rows if r['cap'] == cap and r['track'] == t]
        ok = [r for r in rs if r.get('reason') == 'goal']
        summary[f'{t} {cap:.2f}'] = dict(
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
