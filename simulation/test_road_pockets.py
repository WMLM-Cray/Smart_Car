"""直角口袋(LT3a/b/c):尖 90° 弯出去的走廊往反方向多一截死胡同 —— 雷达扫得到口袋末端的墙、假地面分割把口袋算地面、
开进口袋算出界;别的级别逐位不变。"""
import math

import numpy as np

from simulation.fastsim.road import LEVELS, add_pockets, generate
from simulation.fastsim.road_env import Plant, RoadEnv

LV = {l.name: l for l in LEVELS}


def pocket_road(name='LT3b'):
    for seed in range(50):
        road = generate(np.random.default_rng(seed), LV[name])
        if len(road.pockets):
            return road, next(e for e in road.elems if 'pocket' in e)
    raise AssertionError('50 条路都没有口袋')


def test_levels_without_pockets_do_not_touch_the_rng():
    road = generate(np.random.default_rng(5), LV['L3'])
    rng = np.random.default_rng(7)
    before = rng.bit_generator.state
    add_pockets(road, rng, LV['L3'])
    assert rng.bit_generator.state == before and len(road.pockets) == 0


def test_depth_follows_the_level():
    for name in ('LT3a', 'LT3b', 'LT3c'):
        lo, hi = LV[name].pocket_depth
        for seed in range(10):
            for e in generate(np.random.default_rng(seed), LV[name]).elems:
                if 'pocket' in e:
                    assert lo <= e['pocket'] <= hi


def _corner(road, e):
    sign = 1.0 if e['angle'] > 0 else -1.0
    i1 = e['_i1']
    d1 = np.array([math.cos(road.heading[i1]), math.sin(road.heading[i1])])
    return sign, i1, d1


def test_lidar_sees_the_pocket_end_wall():
    road, e = pocket_road()
    _, i1, d1 = _corner(road, e)
    # 站在出弯处中心线上,朝口袋方向(出弯的反方向)看:先穿过来车那条路,再到口袋末端
    env = RoadEnv(road, Plant.sample(np.random.default_rng(0)), 0, start=(i1, 0.0, 0.0), noise=False)
    x, y = road.centre[i1]
    r = env.scan_at(x, y, math.atan2(-d1[1], -d1[0]))
    ahead = r[np.abs(env.angles) < 0.02] if hasattr(env, 'angles') else r[len(r) // 2 - 2:len(r) // 2 + 3]
    from common.params import CarParams
    reach = e['r_in'] + e['width'] + e['pocket'] - CarParams().lidar_x   # 出弯点 -> 来车路外侧墙线 -> 口袋末端
    assert np.isfinite(ahead).all() and abs(float(np.median(ahead)) - reach) < 0.05


def test_floor_seg_sees_into_the_pocket_and_driving_in_is_off_track():
    from simulation.fastsim.camera import FakeFloorSeg
    road, e = pocket_road()
    poly = road.pockets[0]
    deep = (poly[2] + poly[3]) / 2 * 0.7 + poly.mean(0) * 0.3   # 口袋深处一点
    _, i1, d1 = _corner(road, e)
    seg = FakeFloorSeg(road, seed=0)
    seg.cfg = type(seg.cfg)(miss=0.0, edge_sigma=0.0, jitter=0.0)
    x, y = road.centre[i1] + 0.3 * d1
    yaw = math.atan2(*(deep - (x, y))[::-1])
    xy = seg.detect((x, y, yaw))
    c, s = math.cos(yaw), math.sin(yaw)
    w = np.c_[x + c * xy[:, 0] - s * xy[:, 1], y + s * xy[:, 0] + c * xy[:, 1]]
    assert np.hypot(*(w - deep).T).min() < 0.1
    env = RoadEnv(road, Plant.sample(np.random.default_rng(0)), 0, start=(i1, 0.0, 0.0), noise=False)
    env.car.x, env.car.y, env.car.yaw = *deep, float(math.atan2(-d1[1], -d1[0]))
    assert env._check() in ('off_track', 'collision')
