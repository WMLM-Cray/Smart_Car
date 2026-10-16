"""三通 T 字口测试:从一条走廊(门)出来,正前方是墙,左右两臂都是 6 m 长的走廊。两边都算对(还没定往哪边拐的规则),
撞墙 / 车上截停(guard)/ 卡住超时算失败。车的初始朝向、离路口距离、横向位置都扫一遍(实车出门不一定正)。

    python -m evaluation.tee_test models/a.npz [models/b.npz ...] [teacher] [--seeds 3]

teacher = 对照:老师按规划走出口那条臂,同样的截停规则,看哪些初始状态本来就过不去。

几何:进口直道 + 尖 90° 弯 + 6 m 出口直道,弯外侧用直角口袋(simulation.fastsim.road.add_pockets)做成另一条 6 m 臂。
开进口袋那条臂 1.5 m 以上记 "左/右"(看口袋在哪侧),开到出口直道终点记另一侧。按车上部署条件:不用 YOLO、
地面分割 0.33 s / 0.35 s、截停算失败。
"""
from __future__ import annotations

import argparse
import itertools
import math
from dataclasses import replace

import numpy as np

YAWS = (-30.0, -15.0, 0.0, 15.0, 30.0)     # 初始朝向偏差 °(左正)
DISTS = (0.8, 1.5)                         # 车(后轴)离路口(进口直道末端)多远 m
WIDTHS = (1.0, 1.3, 1.6)
ARM = 6.0                                  # 两臂都比 BEV 左右视野(4 m)长:看不到哪条臂是死胡同


def tee(width, turn, depth=ARM, lead=3.0):
    from simulation.fastsim.road import LEVELS, WALL, add_pockets, build
    elems = [dict(kind='straight', length=lead, width=width, mat=(WALL, WALL)),
             dict(kind='arc', angle=90.0 * turn, r_in=0.05, width=width, mat=(WALL, WALL)),
             dict(kind='straight', length=depth, width=width, mat=(WALL, WALL))]
    road = build(elems)
    lv = replace(next(l for l in LEVELS if l.name == 'LWT3c'), pocket_chance=1.0, pocket_depth=(depth, depth))
    add_pockets(road, np.random.default_rng(0), lv)
    assert len(road.pockets) == 1
    # 四周全是墙:出界判断没意义,而且开进口袋那条臂会被按"离中心线太远"误判出界。给一个包住一切的区域关掉它
    road.region = (np.array([[-100.0, -100.0], [100.0, -100.0], [100.0, 100.0], [-100.0, 100.0]]),
                   np.array([[500.0, 500.0], [501.0, 500.0], [501.0, 501.0]]))
    return road


def run_one(pol, road, turn, dist, off, yaw, seed):
    from common.road_policy import V_LO, Encoding
    from simulation.fastsim.road_env import Plant
    from simulation.fastsim.road import inside
    from training.bev_env import BevEnv
    pl = replace(Plant.sample(np.random.default_rng(seed), Encoding(3.0, V_LO, 3.0)), cap=0.7)   # 车上限速 0.7
    i0 = road.elems[1]['_i0'] - int(round(dist / 0.05))
    b = BevEnv(road, pl, seed, (i0, off, math.radians(yaw)), noise=True, free=True, seg_period=0.33,
               seg_lag=0.35, yolo=False, guard=True)
    if pol == 'teacher':                # 对照:老师(特权参数、只走出口那条臂)在同样的截停规则下能不能过
        from common.params import CarParams
        from training.bev_env import SeenTeacher
        from training.road_teacher import R_MIN, plan
        P = plan(road, i0, off, CarParams().width, pl, r_min=R_MIN, a_lat=1.47)
        if isinstance(P, str):
            return 'no_plan', []
        t = SeenTeacher(b, P, 1.47)

        def pol(img, sc, t=t):
            return t()
    poly = road.pockets[0]
    exit_side = 'R' if turn < 0 else 'L'
    pocket_side = 'L' if turn < 0 else 'R'
    why, steers = None, []
    while why is None and b.env.t < 25.0:
        a = pol(*b.observation())
        steers.append(float(a[0]))
        why = b.step(a)
        c = b.env.car
        # 进了口袋那条臂而且已经过了来车走廊外侧墙 1.5 m:算选了口袋那一侧
        if inside(poly, (c.x, c.y)):
            d1 = np.array([math.cos(road.heading[road.elems[1]['_i1']]), math.sin(road.heading[road.elems[1]['_i1']])])
            if (poly[0] - (c.x, c.y)) @ d1 > 1.5:
                return pocket_side, steers
    if why == 'goal':
        return exit_side, steers
    return why or 'timeout', steers


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('models', nargs='+')
    ap.add_argument('--seeds', type=int, default=3)
    a = ap.parse_args(argv)
    from common.bev_policy import BevPolicy
    for m in a.models:
        pol = 'teacher' if m == 'teacher' else BevPolicy(m)
        rows = []
        for w, turn, dist, yaw, s in itertools.product(WIDTHS, (1, -1), DISTS, YAWS, range(a.seeds)):
            road = tee(w, turn)
            off = float(np.random.default_rng(1000 + s).uniform(-0.1, 0.1)) * w
            res, _ = run_one(pol, road, turn, dist, off, yaw, 7000 + s)
            rows.append((w, turn, dist, yaw, res))
        ok = [r[4] in ('L', 'R') for r in rows]
        print(f'{m}: 成功 {np.mean(ok):.0%}  ({sum(ok)}/{len(rows)})  '
              f'左 {sum(r[4] == "L" for r in rows)} 右 {sum(r[4] == "R" for r in rows)}  '
              + '  '.join(f'{k} {sum(r[4] == k for r in rows)}' for k in ('collision', 'guard', 'off_track', 'timeout')))
        for yaw in YAWS:
            sub = [r for r in rows if r[3] == yaw]
            print(f'   初始朝向 {yaw:+4.0f}°: 成功 {np.mean([r[4] in ("L", "R") for r in sub]):.0%}  '
                  f'左 {sum(r[4] == "L" for r in sub)} 右 {sum(r[4] == "R" for r in sub)}')
        for w in WIDTHS:
            sub = [r for r in rows if r[0] == w]
            print(f'   路宽 {w:.1f} m: 成功 {np.mean([r[4] in ("L", "R") for r in sub]):.0%}')


if __name__ == '__main__':
    main()
