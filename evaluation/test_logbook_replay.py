import math

import numpy as np

from common.params import CarParams
from evaluation.logbook_replay import AHEAD, CELL, SIDE, align_to_map, cones_from_points, frame_points


def cell(x, y):
    return int((AHEAD - x) / CELL), int((SIDE - y) / CELL)


def frame_with(points):
    bev = np.zeros((6, 80, 80), np.float32)
    for x, y in points:
        bev[(4, *cell(x, y))] = 1.0
    return bev


def test_frame_points_drop_the_rear_antennas():
    """车尾天线 (-0.6, ±0.25)(10-24 实车每帧都有)不当成锥桶;车前的点照留。"""
    pts = frame_points(frame_with([(-0.6, 0.25), (-0.6, -0.25), (2.05, 0.55)]))
    assert len(pts) == 1 and abs(pts[0, 0] - 2.05) < CELL and abs(pts[0, 1] - 0.55) < CELL
    x0, x1, hy = CarParams().self_box
    assert x0 <= -0.6 <= x1 and hy >= 0.25


def test_cones_are_small_clusters_and_walls_are_not():
    cone = [(2.05, 0.55), (2.15, 0.55), (2.05, 0.65)]
    wall = [(1.05 + 0.1 * i, -2.05) for i in range(20)]           # 2 m 长的一条线
    c = cones_from_points(frame_points(frame_with(cone + wall)))
    assert len(c) == 1 and np.hypot(*(c[0] - (2.1, 0.6))) < 0.1


def test_align_recovers_a_known_map_offset():
    """车在推算位姿下看到的锥桶 = 地图锥桶整体平移旋转后的样子;对齐应找回这个变换。"""
    rng = np.random.default_rng(0)
    cones_map = rng.uniform(-3, 3, (25, 2))                         # 不规则(规则网格会周期混叠)
    th, tx, ty = 0.4, 1.0, -0.5
    c, s = math.cos(th), math.sin(th)
    inv = lambda p: np.c_[c * (p[:, 0] - tx) + s * (p[:, 1] - ty), -s * (p[:, 0] - tx) + c * (p[:, 1] - ty)]
    seen = inv(cones_map)                                          # 推算位姿系(= 车体系,位姿取原点)
    near = seen[(np.hypot(*seen.T) < 4.5) & (seen[:, 0] > -1.8) & (np.abs(seen[:, 1]) < 3.8)]   # 裁图范围内
    bev = frame_with(near)                                          # 一桶一格
    n = 6                                                          # 整段每隔一帧取锥桶,至少 20 个观测才迭代
    score, T = align_to_map(np.zeros((n, 7)), np.stack([bev] * n), np.zeros((n, 3)), cones_map)
    assert score > 0.5
    assert abs((T[2] - th + math.pi) % (2 * math.pi) - math.pi) < 0.05
    assert math.hypot(T[0] - tx, T[1] - ty) < 0.1
