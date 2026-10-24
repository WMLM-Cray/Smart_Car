"""边界条件测试:同一个 L 型拐角(evaluation/l_test),只换墙边和地面分割边界的样子,看模型的判断变不变。

    python -m evaluation.boundary_test models/a.npz models/b.npz [--variants clean boxes ...] [--open-left --widths 5 6]

变体(墙边的东西对所有墙都加,每局按种子随机摆):
  clean      光墙(对照)
  boxes      墙边堆箱子:凸出 0.1~0.5 m、沿墙长 0.2~0.6 m,平均隔 1 m 一个
  legs       墙边放椅子:4 条 4 cm 腿、脚印 0.4 m 见方,离墙 0.05~0.3 m,平均隔 1.5 m 一把(雷达只扫到零散的点)
  gaps       墙断断续续:平均每 1.2 m 缺 0.1~0.4 m(门缝、家具之间),缺口后面什么都没有
  seg_ragged 地面分割边界抖得厉害:每帧整体偏 0.15 m、逐点抖 0.10 m(原来 0.05 / 0.03)
  seg_foot   墙脚被当成地面:边界固定外扩 0.15 m(点落到墙后)
  seg_short  地面分割保守:离墙 0.3 m 就不认了
部署条件同 l_test,地面分割按车上的投影 + 补缝(seg_lut、seg_fill)。
报每个变体的通过率,和"开始拐"(朝向转过 20°)时车离前墙多远:比光墙晚拐就是被边界骗了。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import itertools
import math
from collections import Counter
from dataclasses import replace

import numpy as np

VARIANTS = ('clean', 'boxes', 'legs', 'gaps', 'seg_ragged', 'seg_foot', 'seg_short')
SEG = {'seg_ragged': dict(edge_sigma=0.15, jitter=0.10), 'seg_foot': dict(edge_bias=0.15),
       'seg_short': dict(edge_bias=-0.3)}
TURNED = math.radians(20.0)         # 朝向转过这么多算开始拐
LEG = 0.04                          # 椅子腿边长 m
CLEAR = 1.2                         # 出发点(后轴)这么近内不摆东西:窄路上椅子会直接压在车上,第一拍就截停


def _wall_frame(road):
    """每段墙的中点、沿墙方向、朝路内的法向(指向最近的中心线点)。"""
    from scipy.spatial import cKDTree
    w = road.walls
    mid = (w[:, 0] + w[:, 1]) / 2
    t = w[:, 1] - w[:, 0]
    t /= np.maximum(np.hypot(t[:, 0], t[:, 1]), 1e-9)[:, None]
    n = np.stack([-t[:, 1], t[:, 0]], 1)
    _, i = cKDTree(road.centre).query(mid)
    n *= np.sign(np.einsum('ij,ij->i', road.centre[i] - mid, n))[:, None]
    return mid, t, n


def _pick(rng, mid, spacing, min_gap):
    """沿墙随机挑位置:个数 = 墙总长 / spacing,彼此至少隔 min_gap。"""
    from simulation.fastsim.road import WALL_STEP
    want = int(len(mid) * WALL_STEP / spacing)
    out = []
    for k in rng.permutation(len(mid)):
        if len(out) == want:
            break
        if all(np.hypot(*(mid[k] - mid[j])) >= min_gap for j in out):
            out.append(k)
    return out


def _box(c, t, n, along, depth):
    return np.array([c + a * along / 2 * t + b * depth / 2 * n for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))])


def _away(boxes, clear):
    """去掉离 clear 点不到 CLEAR 的箱子 / 椅子(一把椅子 4 条腿一起留或一起去)。"""
    if clear is None:
        return [g for grp in boxes for g in grp]
    return [g for grp in boxes if min(np.hypot(*(np.asarray(grp).reshape(-1, 2) - clear).T)) >= CLEAR for g in grp]


def _add_boxes(road, boxes):
    """箱子 / 椅子腿:墙线段给雷达和碰撞,obstacles 给地面分割挡视线(同 road.add_obstacles)。"""
    from simulation.fastsim.road import WALL_STEP, _resample
    segs = []
    for b in boxes:
        for k in range(4):
            p, _ = _resample(np.array([b[k], b[(k + 1) % 4]]), min(WALL_STEP, LEG))
            segs.append(np.stack([p[:-1], p[1:]], axis=1))
    road.walls = np.concatenate([road.walls] + segs)
    road.obstacles = np.concatenate([np.asarray(getattr(road, 'obstacles', np.zeros((0, 4, 2)))).reshape(-1, 4, 2),
                                     np.asarray(boxes, float)])


def perturb(road, variant, rng, clear=None):
    """就地改 road 的墙(seg_* 变体不动路,只改地面分割)。clear:车的出发点 (x, y),周围 CLEAR 内不摆东西。"""
    if variant in ('clean',) or variant in SEG:
        return
    mid, t, n = _wall_frame(road)
    if variant == 'boxes':
        boxes = []
        for k in _pick(rng, mid, 1.0, 0.5):
            depth, along = rng.uniform(0.1, 0.5), rng.uniform(0.2, 0.6)
            boxes.append([_box(mid[k] + n[k] * (depth / 2 + 0.01), t[k], n[k], along, depth)])
        _add_boxes(road, _away(boxes, clear))
    elif variant == 'legs':
        boxes = []
        for k in _pick(rng, mid, 1.5, 0.6):
            off = rng.uniform(0.05, 0.3)
            boxes.append([_box(mid[k] + a * t[k] + n[k] * (off + b + LEG / 2), t[k], n[k], LEG, LEG)
                          for a, b in itertools.product((-0.2, 0.2), (0.0, 0.4))])
        _add_boxes(road, _away(boxes, clear))
    elif variant == 'gaps':
        drop = np.zeros(len(mid), bool)
        for k in _pick(rng, mid, 1.2, 0.6):
            drop |= np.hypot(*(mid - mid[k]).T) < rng.uniform(0.05, 0.2)
        road.walls = road.walls[~drop]
    else:
        raise ValueError(variant)


def run_one(args):
    model, variant, width, yaw, seed, open_left = args
    from common.bev_policy import BevPolicy
    from common.road_policy import V_LO, Encoding
    from evaluation.l_test import FRONT, RIGHT, ell
    from simulation.fastsim.road_env import Plant
    from training.bev_env import BevEnv
    road = ell(width, open_left)
    i0 = road.elems[1]['_i0'] - int(round((FRONT - width - 0.05) / 0.05))
    off = -width / 2 + RIGHT
    perturb(road, variant, np.random.default_rng(seed + 100), clear=road.centre[i0] + off * road.normal[i0])
    pol = BevPolicy(model)
    pl = replace(Plant.sample(np.random.default_rng(seed), Encoding(3.0, V_LO, 3.0)), cap=0.7, enc=pol.enc)   # 动作按模型自己的速度契约解码(带 _d 的模型能停车 / 刹车);老模型的契约和这里一样,结果逐位不变
    b = BevEnv(road, pl, seed, (i0, off, math.radians(yaw)), noise=True, free=True,
               seg_period=0.33, seg_lag=0.35, yolo=False, guard=True, seg_lut=True, seg_fill=True)
    if variant in SEG:
        b.seg.cfg = replace(b.seg.cfg, **SEG[variant])
    e = road.elems[1]
    d0 = np.array([math.cos(road.heading[e['_i0']]), math.sin(road.heading[e['_i0']])])
    front = float((road.centre[e['_i0']] + d0 * (e['r_in'] + width)) @ d0)       # 前墙在 d0 方向上的位置
    yaw0, why, onset = b.pose[2], None, None
    while why is None and b.env.t < 30.0:
        why = b.step(pol(*b.observation()))
        x, y, th = b.pose
        if onset is None and (th - yaw0 + math.pi) % (2 * math.pi) - math.pi > TURNED:
            onset = front - float(np.array([x, y]) @ d0)
    return why or 'timeout', onset


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('models', nargs='+')
    ap.add_argument('--variants', nargs='+', default=list(VARIANTS), choices=VARIANTS)
    ap.add_argument('--seeds', type=int, default=2)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--open-left', action='store_true')
    ap.add_argument('--widths', type=float, nargs='+', default=None, help='默认同 l_test')
    a = ap.parse_args(argv)
    from concurrent.futures import ProcessPoolExecutor
    from evaluation.l_test import WIDTHS, YAWS
    grid = list(itertools.product(a.widths or WIDTHS, YAWS, range(a.seeds)))
    jobs = [(m, v, w, y, 7000 + s, a.open_left) for m in a.models for v in a.variants for w, y, s in grid]
    with ProcessPoolExecutor(a.workers) as ex:
        res = list(ex.map(run_one, jobs, chunksize=2))
    k = 0
    for m in a.models:
        print(os.path.basename(m))
        for v in a.variants:
            rows = res[k:k + len(grid)]
            k += len(grid)
            ok = np.mean([r[0] == 'goal' for r in rows])
            on = [r[1] for r in rows if r[1] is not None]
            on_s = f'开始拐时离前墙 中位 {np.median(on):.2f} m' if on else '没拐'
            print(f'  {v:11s} 过 {ok:4.0%}  {on_s:22s} {dict(Counter(r[0] for r in rows))}')


if __name__ == '__main__':
    main()
