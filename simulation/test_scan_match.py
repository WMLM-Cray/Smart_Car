"""扫描匹配(simulation/fastsim/scan_match.py):里程计带漂移时,修正后的位姿回到真值附近。"""
import math

import numpy as np

from simulation.fastsim.real_map import RealMap
from simulation.fastsim.scan_match import match_trajectory, to_world


def room():
    """6 m × 3 m 的房间 + 中间一根 20 cm 方柱(5 cm 格)。"""
    i, j = np.arange(-20, 101), np.arange(-30, 31)
    occ = [np.stack([i, np.full_like(i, -30)], 1), np.stack([i, np.full_like(i, 30)], 1),
           np.stack([np.full_like(j, -20), j], 1), np.stack([np.full_like(j, 100), j], 1),
           np.array([[a, b] for a in range(40, 44) for b in range(8, 12)])]
    return RealMap(np.concatenate(occ), 0.05)


def scan(world, pose, n=720):
    ang = -math.pi + np.arange(n) * 2 * math.pi / n
    r = world.raycast(pose[0], pose[1], ang + pose[2], 8.0)
    ok = np.isfinite(r)
    return np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], 1)     # 车体系(这里雷达就在原点)


def test_match_removes_odometry_drift():
    world = room()
    k = np.arange(60)
    true = np.stack([0.0 + k * 0.06, 0.3 * np.sin(k / 15), 0.2 * np.sin(k / 20)], 1)
    odom = true.copy()
    odom[:, 0] += k * 0.004                   # 每帧多走 4 mm、多转 0.1°:60 帧后差 24 cm / 6°
    odom[:, 2] += np.radians(0.1) * k
    body = [scan(world, p) for p in true]
    est, pairs = match_trajectory(body, odom)
    err_odom = np.hypot(*(odom[-1, :2] - true[-1, :2]))
    err = np.hypot(*(est[:, :2] - true[:, :2]).T)
    assert err_odom > 0.2
    assert err.max() < 0.05
    assert np.abs(est[:, 2] - true[:, 2]).max() < math.radians(1.0)
    assert (pairs[1:] >= 80).all()


def test_to_world_matches_raycast_hit():
    world = room()
    pose = (1.0, 0.2, 0.3)
    pts = to_world(scan(world, pose), pose)
    i = np.round(pts / 0.05).astype(int)
    near = [world.occ[a - world.i0 - 1:a - world.i0 + 2, b - world.j0 - 1:b - world.j0 + 2].any() for a, b in i]
    assert np.mean(near) > 0.99
