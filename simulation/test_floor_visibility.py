"""EDGE 标签必须来自已送达学生的地面证据,包含距离截断、分割失败与延迟。"""
from dataclasses import replace

import numpy as np
import pytest

from common.bev import body_to_odom
from simulation.fastsim.camera import FakeFloorSeg, FloorSegConfig
from simulation.fastsim.road import EDGE, build
from simulation.fastsim.road_env import Plant
from training.bev_env import BevEnv, FLOOR_SEEN_R, SeenTeacher
from training.road_teacher import plan


def _env(sure=0.0, miss=0.0, lag=0.35):
    road = build([dict(kind='straight', length=8.0, width=1.0, mat=(EDGE, EDGE))])
    pl = replace(Plant.sample(np.random.default_rng(1)), cap=0.7)
    env = BevEnv(road, pl, 1, noise=False, free=True, yolo=False,
                 seg_lut=True, seg_fill=True, seg_period=0.33, seg_lag=lag)
    env.seg = FakeFloorSeg(road, cfg=replace(env.seg.cfg, sure=sure, miss=miss,
                                           edge_sigma=0.0, jitter=0.0), seed=12)
    env.seg_queue.clear()
    env.seg_frame = None
    env.next_seg = env.env.t = 0.0
    env.bev.reset()
    teacher = SeenTeacher(env, plan(road, 0, 0.0, env.env.cp.width, pl, a_lat=1.47), 1.47)
    return env, teacher


def test_lut_projection_includes_its_own_samples_and_respects_frame_range():
    env, _ = _env()
    seg = env.seg
    assert seg.in_view(seg.grid_body, (0, 0, 0)).all()
    inside = seg.in_view(seg.grid_body, (0, 0, 0), reach=2.0)
    assert inside.any() and not inside.all()
    assert np.all(seg.grid_r[inside] <= 2.0 + 1e-9)


@pytest.mark.parametrize('sure,miss', [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
def test_edge_visibility_uses_delivered_points_not_scan_or_ideal_fov(sure, miss):
    from scipy.spatial import cKDTree
    env, teacher = _env(sure, miss)
    env._camera()
    assert not teacher.seen.any() and not env.bev.crop_layers(env.pose, 0)[5].any()
    env.env.t = 0.34
    env._camera()
    assert not teacher.seen.any()
    env.env.t = 0.35
    env._camera()
    pose, xy, stamp = env.seg_frame
    assert stamp == 0.0
    if miss:
        assert not len(xy) and not teacher.seen.any()
    else:
        world = body_to_odom(xy, pose)
        expected = cKDTree(world).query(teacher.tree.data)[0] <= FLOOR_SEEN_R
        assert expected.any() and np.array_equal(teacher.seen, expected)
        radius = np.hypot(*(teacher.tree.data - (env.seg.p.camera_x, 0)).T)
        assert not teacher.seen[radius > (3.0 if sure else 2.0) + FLOOR_SEEN_R].any()
        assert env.bev.crop_layers(env.pose, 0.35)[5].any()


def test_initial_zero_lag_frame_is_available_to_teacher():
    env, teacher = _env(lag=0.0)
    env._camera()
    assert teacher.seen.any()
    another = SeenTeacher(env, teacher.plan, 1.47)
    assert np.array_equal(teacher.seen, another.seen)


def test_sampled_launch_does_not_allow_seconds_of_stationary_evidence():
    rng = np.random.default_rng(10)
    assert all(0 <= Plant.sample(rng).launch_delay <= 0.2 for _ in range(100))
