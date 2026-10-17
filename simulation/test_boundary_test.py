"""evaluation/boundary_test 的边界变体:墙边的东西放在路内侧、不堵路;地面分割外扩真的落到墙后。"""
import math

import numpy as np

from evaluation.boundary_test import perturb
from evaluation.l_test import ell


def _inside(road, pts):
    """点到中心线的有符号横向距离 / 半宽(|.| < 1 在路内)。"""
    from scipy.spatial import cKDTree
    _, i = cKDTree(road.centre).query(pts)
    d = np.einsum('ij,ij->i', pts - road.centre[i], road.normal[i])
    return np.abs(d) / road.half[i]


def test_clutter_sits_inside_the_road_next_to_the_wall():
    for v in ('boxes', 'legs'):
        road = ell(3.5)
        n0 = len(road.walls)
        perturb(road, v, np.random.default_rng(1))
        assert len(road.obstacles) > 5
        c = road.obstacles.mean(axis=1)
        straight = c[:, 0] < road.centre[road.elems[1]['_i0'], 0] - 0.5      # 拐角处中心线最近点不准,只看进口直道(沿 +x)
        assert straight.sum() > 3
        assert np.all(_inside(road, c[straight]) < 1.0), v
        assert len(road.walls) > n0


def test_nothing_is_placed_on_the_start():
    start = np.array([2.0, 0.6])
    for v in ('boxes', 'legs'):
        r = ell(2.5)
        perturb(r, v, np.random.default_rng(0), clear=start)
        d = np.hypot(*(r.obstacles.reshape(-1, 2) - start).T)
        assert d.min() >= 1.2, v


def test_gaps_remove_wall_and_seg_variants_leave_road_alone():
    road = ell(3.5)
    n0 = len(road.walls)
    perturb(road, 'gaps', np.random.default_rng(1))
    assert len(road.walls) < n0
    road = ell(3.5)
    perturb(road, 'seg_foot', np.random.default_rng(1))
    assert len(road.walls) == n0


def test_edge_bias_puts_floor_behind_the_wall():
    from dataclasses import replace

    from simulation.fastsim.camera import FakeFloorSeg, FloorSegConfig
    road = ell(3.5)
    i = road.elems[0]['_i0'] + 20
    pose = (*road.centre[i], road.heading[i] - math.radians(60))     # 车头斜对右墙
    cfg = replace(FloorSegConfig(), edge_sigma=0.0, jitter=0.0, miss=0.0)
    far = []
    for bias in (0.0, 0.2):
        pts = FakeFloorSeg(road, cfg=replace(cfg, edge_bias=bias)).detect(pose)
        x, y, yaw = pose
        w = np.stack([x + math.cos(yaw) * pts[:, 0] - math.sin(yaw) * pts[:, 1],
                      y + math.sin(yaw) * pts[:, 0] + math.cos(yaw) * pts[:, 1]], 1)
        far.append(_inside(road, w).max())
    assert far[0] <= 1.0 + 1e-6 < far[1]
