"""L 型拐角测试:按实车 1014 seg_06 的几何(m5 直冲墙角、截停)在 fastsim 里搭一个左拐的 L,看哪些条件下模型不拐。

    python -m evaluation.l_test models/a.npz models/b.npz [teacher] [--seeds 2]

实车那段(车的起点坐标系,x 前 y 左):右边是墙(y ≈ -1.9),正前方 7 m 是墙,出口在左边(一大片屋子,家具零散);
车贴右墙往前开了 6.3 m,停在离前墙 0.7 m 处。这里:进口直道 + 尖 90° 左弯(外侧方角)+ 出口直道,
车(后轴)离前墙 FRONT 米、离右墙 RIGHT 米;路宽(= 左边多空)和初始朝向扫一遍。开到出口直道终点算过,
截停 / 撞 / 超时算失败。部署条件同 tee_test:不用 YOLO、地面分割 0.33 s / 0.35 s。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import itertools
import math
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import numpy as np

FRONT = 7.0                 # 车(后轴)离前墙 m(实车 seg_06)
RIGHT = 1.9                 # 车离右墙 m
WIDTHS = (2.5, 3.5, 4.5)    # 路宽:实车左边是开阔的屋子,扫一个范围
YAWS = (-10.0, 0.0, 10.0)   # 初始朝向偏差 °(左正;实车起步时大致平行右墙)
EXIT = 5.0                  # 出口直道长 m


def ell(width, open_left=False):
    """左拐 L:外侧(右墙 -> 前墙)是方角(见 _square_corner)。
    open_left:内侧(左边)不立墙、只是地面边界(EDGE)—— 实车左边是开阔的屋子,墙在 BEV 视野(左右 4 m)外。"""
    from simulation.fastsim.road import EDGE, WALL, build
    lead = FRONT - width + 1.5
    left = EDGE if open_left else WALL
    elems = [dict(kind='straight', length=lead, width=width, mat=(left, WALL)),
             dict(kind='arc', angle=90.0, r_in=0.05, width=width, mat=(left, WALL)),
             dict(kind='straight', length=EXIT, width=width, mat=(left, WALL))]
    road = build(elems)
    _square_corner(road, road.elems[1])
    road.region = (np.array([[-100.0, -100.0], [100.0, -100.0], [100.0, 100.0], [-100.0, 100.0]]),
                   np.array([[500.0, 500.0], [501.0, 500.0], [501.0, 501.0]]))
    return road


def _square_corner(road, e):
    """把弯外侧的圆角墙换成方角(同 road.add_pockets 的做法、口袋深度 0)。不直接用 add_pockets:
    它的"口袋压到别的路段"检查在出口直道紧接弯道时会误拒,结果外侧一直是半径 = 路宽的大圆弧,把车顺着引过去
    (2026-10-17 第一版 l_test 就是这样,全部模型 100% 过)。方角里那块也算地面(road.pockets)。"""
    from simulation.fastsim.road import WALL_STEP, _line_x, _resample
    sign = 1.0 if e['angle'] > 0 else -1.0
    i0, i1 = e['_i0'], e['_i1']
    outer = road.right if sign > 0 else road.left
    d0 = np.array([math.cos(road.heading[i0]), math.sin(road.heading[i0])])
    d1 = np.array([math.cos(road.heading[i1]), math.sin(road.heading[i1])])
    p0, p1 = outer[i0], outer[i1]
    b = _line_x(p0, d0, p1, d1)
    c = road.centre[i0] + sign * (e['r_in'] + e['width'] / 2) * road.normal[i0]
    w = road.walls.reshape(-1, 2) - c
    on_arc = ((np.abs(np.hypot(w[:, 0], w[:, 1]) - (e['r_in'] + e['width'])) < 0.05) & (w @ d0 > 1e-3) & (w @ d1 < -1e-3))
    add = []
    for q0, q1 in ((p0 - WALL_STEP * d0, b), (b, p1 + WALL_STEP * d1)):
        pts, _ = _resample(np.array([q0, q1]), WALL_STEP)
        add.append(np.stack([pts[:-1], pts[1:]], axis=1))
    road.walls = np.concatenate([road.walls[~on_arc.reshape(-1, 2).any(axis=1)]] + add)
    road.pockets = np.array([[p0, b, b, b, p1]], float)


def run_one(args):
    model, width, yaw, seed, open_left = args
    from common.bev_policy import BevPolicy
    from common.road_policy import V_LO, Encoding
    from simulation.fastsim.road_env import Plant
    from training.bev_env import BevEnv
    road = ell(width, open_left)
    pl = replace(Plant.sample(np.random.default_rng(seed), Encoding(3.0, V_LO, 3.0)), cap=0.7)
    if model != 'teacher':
        pl = replace(pl, enc=BevPolicy(model).enc)    # 动作按模型自己的速度契约解码(带 _d 的模型能停车 / 刹车);老模型的契约和这里一样,结果逐位不变
    dist = FRONT - width - 0.05                          # 后轴到弯道起点(弯内角);前墙在弯起点外 r_in + 路宽
    i0 = road.elems[1]['_i0'] - int(round(dist / 0.05))
    off = -width / 2 + RIGHT                             # 横向:左正
    b = BevEnv(road, pl, seed, (i0, off, math.radians(yaw)), noise=True, free=True, seg_period=0.33,
               seg_lag=0.35, yolo=False, guard=True)
    if model == 'teacher':
        from common.params import CarParams
        from training.bev_env import SeenTeacher
        from training.road_teacher import R_MIN, plan
        P = plan(road, i0, off, CarParams().width, pl, r_min=R_MIN, a_lat=1.47)
        if isinstance(P, str):
            return 'no_plan', []
        t = SeenTeacher(b, P, 1.47)
        pol = lambda img, sc: t()
    else:
        net = BevPolicy(model)
        pol = net
    why, path = None, []
    while why is None and b.env.t < 30.0:
        why = b.step(pol(*b.observation()))
        path.append(b.pose)
    return why or 'timeout', np.array(path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('models', nargs='+')
    ap.add_argument('--seeds', type=int, default=2)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--open-left', action='store_true', help='左边不立墙(EDGE),配 --widths 5 6')
    ap.add_argument('--widths', type=float, nargs='+', default=list(WIDTHS))
    a = ap.parse_args(argv)
    grid = list(itertools.product(a.widths, YAWS, range(a.seeds)))
    with ProcessPoolExecutor(a.workers) as ex:
        for m in a.models:
            res = list(ex.map(run_one, [(m, w, y, 7000 + s, a.open_left) for w, y, s in grid]))
            ok = [r[0] == 'goal' for r in res]
            print(f'{os.path.basename(m)}: 过 {np.mean(ok):.0%} ({sum(ok)}/{len(ok)})  {dict(Counter(r[0] for r in res))}')
            for w in a.widths:
                sub = [r[0] for (ww, _, _), r in zip(grid, res) if ww == w]
                print(f'   路宽 {w:.1f} m: 过 {np.mean([x == "goal" for x in sub]):.0%}  {dict(Counter(sub))}')


if __name__ == '__main__':
    main()
