"""把官方赛道 A-D(maps/ 下规则图估读数据)转成路段环境的 Road,跑速度 x 通过性测试。

    python -m evaluation.road_track --model runs/road_v2_b/candidate_030.npz --tracks A B C D --fill-v
    python -m evaluation.road_track --model models/steering_v1/candidate_060.npz --tracks A    # 旧模型对照

数据(maps/,2026-09-25 从规则图 9 逐桶读图,见 docs/track/TRACK_AD_STATUS.md):
  A  trackA_cones.csv(71 桶逐点)+ trackA_layout.yaml 的黄 V 多边形
  B-D trackX_layout.json 的内外边界折线(同点数、一一配对)+ trackX_cones.csv
边界 = 外/内两条闭合折线(A 直接用锥桶连线,黄 V 处补上谷点)。中心线先取两边中点、
周期高斯平滑(sigma 0.5m)磨掉方角折点,再用射线法量出中心线**左右各自**到边界的距离,
当作这一点的左右余量(2026-09-29)。

为什么左右要分开:A 的内外边界点数不同(走"最近点配对"),左下直角弯处配出一根竖直弦,
中心线在那里偏了 0.52m —— 左侧只剩 0.483m、右侧有 1.646m(路面总宽 2.13m,很宽)。
旧的 half = min(左,右) 把右边 1.16m 白扔掉,规划可行域只剩 0.019m,所以 A 上 0.000
是仪器问题不是模型缺陷。把中心线挪到中轴是行不通的:直角弯的中轴自带折点,逐点挪会造出
R=0.24~0.36m 的尖角,老师自己就规划不出来(实测通过率 1.000 -> 0.000)。
B-D 内外同点数、一一配对,本来没偏心(中位修正 0.02m),只把偏保守的半宽还原成真值。

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
from simulation.fastsim.road import CONE, DS, NONE, Road
from simulation.fastsim.road_env import Plant, RoadEnv
from common.road_policy import DEFAULT_ENC, RoadPolicy, spec_of, speed_to_action
from training.road_teacher import A_LAT, Teacher, plan

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


RAY_MAX = 6.0               # 射线最远打这么远,打不到就按这个算(比任何一处半宽都大)


def _ray_len(P, U, edge):
    """P (n,2) 每点沿单位向量 U (n,2) 打到闭合折线 edge 的最近正向距离,返回 (n,)。

    解 P + t·U = a + s·e(t>=0, 0<=s<=1):
      t = cross(a-P, e) / cross(U, e)
      s = cross(a-P, U) / cross(U, e)
    一段都没打中给 RAY_MAX。
    """
    a = edge
    b = np.roll(edge, -1, axis=0)
    e = b - a                                                   # (m,2)
    d0 = a[None, :, 0] - P[:, None, 0]                          # (n,m) = (a-p)
    d1 = a[None, :, 1] - P[:, None, 1]
    den = U[:, None, 0] * e[None, :, 1] - U[:, None, 1] * e[None, :, 0]
    num = d0 * e[None, :, 1] - d1 * e[None, :, 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = num / den
        s = (d0 * U[:, None, 1] - d1 * U[:, None, 0]) / den
    good = (np.abs(den) > 1e-12) & (t >= 0) & (s >= 0) & (s <= 1)
    t = np.where(good, t, np.inf)
    return np.where(np.isfinite(t).any(axis=1), t.min(axis=1), RAY_MAX)


def _ray_measure(cen, od, idn, u=None):
    """量中心线两侧到边界的距离,返回 (dl, dr, 法向 u)。u 给了就用给的,不给从中心线取。"""
    if u is None:
        d = np.roll(cen, -1, axis=0) - np.roll(cen, 1, axis=0)
        d = d / np.maximum(np.hypot(d[:, 0], d[:, 1]), 1e-12)[:, None]
        u = np.stack([-d[:, 1], d[:, 0]], axis=1)
    dl = np.minimum(_ray_len(cen, u, od), _ray_len(cen, u, idn))
    dr = np.minimum(_ray_len(cen, -u, od), _ray_len(cen, -u, idn))
    return dl, dr, u


def _ray_corridor(cen, od, idn, iters: int = 3):
    """把中心线拉到走廊中线。**当前未使用** —— 见 official() 里的说明:直角弯的中轴
    自带折点,挪线会造出 R<0.4m 的尖角,老师直接规划不出来。留在这里当反例。
    """
    _, _, u = _ray_measure(cen, od, idn)
    c = cen.copy()
    for _ in range(iters):
        dl, dr, _ = _ray_measure(c, od, idn, u)
        step = np.where((dl >= RAY_MAX) | (dr >= RAY_MAX), 0.0, (dl - dr) / 2)
        c = c + u * step[:, None]
    dl, dr, _ = _ray_measure(c, od, idn, u)
    return c, dl, dr


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
    # 中心线**不挪** —— 只把半宽换成射线量到的左右两侧实际余量。
    #
    # 为什么不是"把中心线拉到走廊中线":直角弯的等距线(中轴)自己就带折点,逐点挪会造出
    # R=0.24~0.36m 的尖角(实测),老师规划直接 curvature 失败(通过率 1.000 -> 0.000)。
    # 而且 A 的偏心根本不该靠挪线解决 —— 它的走廊本来就不对称:左下直角弯左侧 0.483m、
    # 右侧 1.646m(总宽 2.13m)。旧代码 half = min(左,右) 把右边 1.16m 扔掉,规划可行域
    # 只剩 0.019m,这才是 A 全灭的直接原因。改成路自己带左右余量,中心线一步不动。
    #
    # B-D 内外边界同点数、一一配对,本来就没偏心(中位修正 0.02m),射线法只把保守的
    # 半宽还原成真实值(min 0.611->0.615 / 0.597->0.598),路宽基本不变。
    dl, dr, _ = _ray_measure(cen, od, idn)
    half = np.minimum(dl, dr)

    def dist(pts, edge):
        return np.min(np.hypot(pts[:, None, 0] - edge[None, :, 0], pts[:, None, 1] - edge[None, :, 1]), axis=1)
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
    cen, dl, dr, mat = (np.roll(x, -i0, axis=0) for x in (cen, dl, dr, np.asarray(mat)))
    half = np.minimum(dl, dr)
    lap_len = float(np.hypot(*np.diff(np.vstack([cen, cen[:1]]), axis=0).T).sum())
    reps = np.r_[np.tile(np.arange(len(cen)), laps), np.arange(int(1.5 / DS))]
    c = cen[reps]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(c, axis=0).T))])
    d = np.gradient(c, axis=0)
    heading = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    road = Road(c, heading, s, half[reps], mat[reps], mat[reps].copy(),
                elems=[dict(kind=f'track{name}', fill_v=fill_v)])
    # 左右各自的余量。A 的走廊不对称,规划要用两侧各自的宽度,不能用 min。
    road.half_left = dl[reps]
    road.half_right = dr[reps]
    road.cones = np.asarray(cones, float)
    road.walls = np.zeros((0, 2, 2))
    road.region = (np.asarray(outer, float), np.asarray(inner, float))
    return road, lap_len


def _schema(path):
    with np.load(path) as z:
        return str(z['schema'])


def run(args):
    model, name, seed, cap, fill_v, a_lat = args
    road, lap_len = official(name, fill_v)
    rng = np.random.default_rng(seed)
    # 速度契约跟模型走(见 common/road_policy.py):v_hi=3 的模型要按 3 的区间观测/解码
    enc = DEFAULT_ENC
    if model is not None and not _schema(model).startswith(v1.SCHEMA_PREFIX):
        enc = spec_of(_schema(model))
    pl = replace(Plant.sample(rng, enc), cap=cap)
    out = dict(track=name, seed=seed, cap=cap, v_plateau=pl.v_plateau, v_top=pl.v_top)
    # 通过性:老师用同一组车辆参数跑(它只解释"这条路老师过不过得去",不是学生成绩)
    P = plan(road, 0, 0.0, CarParams().width, pl, a_lat=a_lat)
    if isinstance(P, str):
        out['teacher'] = P
    else:
        env = RoadEnv(road, pl, seed, noise=False, sense=False)
        t = Teacher(env, P, a_lat=a_lat)
        why = None
        while why is None:
            why = env.step(t())
        out['teacher'] = why
        out['teacher_clear'] = float(env.min_clear)
    # 学生
    env = RoadEnv(road, pl, seed, noise=True)
    if model is None:
        return out
    # v1 按 schema 前缀认(带速度尺度后缀的也算),观测用模型自己的 scale
    if _schema(model).startswith(v1.SCHEMA_PREFIX):
        pol, last = v1.SteeringPolicy(model), 0.0
        v_a = speed_to_action(cap)

        def act():
            nonlocal last
            last = pol(v1.observe(env.ranges, env.angles, env.car.v, cap, env.car.yaw_rate, last,
                                  scale=pol.scale))
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
    ap.add_argument('--a-lat', type=float, default=A_LAT, help='老师的弯中横向加速度上限(见 road_teacher.A_LAT)')
    a = ap.parse_args()
    jobs = [(a.model, t, a.seed_start + i, cap, a.fill_v, a.a_lat)
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
