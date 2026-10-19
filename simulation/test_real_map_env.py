"""实景地图闭环(simulation/fastsim/real_map_env.py)和 RealMap.from_seg 的去零散格。"""
import math
import os
import tempfile

import numpy as np

from simulation.fastsim.real_map import RealMap
from simulation.fastsim.real_map_env import MapFloorSeg, RealMapEnv
from common.road_policy import spec_of
from simulation.fastsim.road_env import STOP, Plant


def corridor(length=8.0, half=0.6, cell=0.05):
    """x 方向直走廊:两侧墙格,中间全是见过的地面。"""
    xs = np.arange(-20, int(length / cell))
    w = int(round(half / cell))
    occ = np.concatenate([np.stack([xs, np.full_like(xs, s * w)], 1) for s in (-1, 1)])
    fi, fj = np.meshgrid(xs, np.arange(-w + 1, w), indexing='ij')
    free = np.stack([fi.ravel(), fj.ravel()], 1)
    return RealMap(occ, cell, free, np.concatenate([occ, free]))


def test_scan_sees_side_walls():
    env = RealMapEnv(corridor(), Plant(launch_delay=0.0), 0, (0.0, 0.0, 0.0), noise=False)
    left = np.argmin(np.abs(env.angles - math.pi / 2))
    assert abs(env.ranges[left] - 0.6) < 0.05


def test_straight_ok_turn_collides_or_leaves():
    env = RealMapEnv(corridor(), Plant(launch_delay=0.0), 0, (0.0, 0.0, 0.0), noise=False, duration=3.0)
    why = None
    while why is None:
        why = env.step((0.0, 0.0))
    assert why == 'timeout' and env.dist > 1.0
    env = RealMapEnv(corridor(), Plant(launch_delay=0.0), 0, (0.0, 0.0, 0.0), noise=False, duration=10.0)
    why = None
    while why is None:
        why = env.step((1.0, 0.0))
    assert why == 'collision'


def test_floor_only_on_seen_free_cells():
    world = corridor()
    seg = MapFloorSeg(world, seed=0)
    seg.cfg = type(seg.cfg)(**{**seg.cfg.__dict__, 'miss': 0.0, 'sure': 1.0})
    pts = seg.detect((0.0, 0.0, 0.0))
    assert len(pts) > 0
    assert np.all(np.abs(pts[:, 1]) < 0.6)                 # 墙外(没见过的地方)没有地面点


def test_from_seg_drops_small_blobs():
    lidar = np.array([[0, 0], [0, 1], [0, 2], [0, 3], [10, 10]], np.int32)    # 一段 4 格的墙 + 一个孤格
    with tempfile.TemporaryDirectory() as d:
        f = os.path.join(d, 'seg.npz')
        np.savez(f, cell=0.05, lidar_ij=lidar, lidar_n=np.full(5, 3, np.int8),
                 free_ij=np.zeros((0, 2), np.int32), free_n=np.zeros(0, np.int8), path=np.zeros((1, 7)))
        assert RealMap.from_seg(f).occ.sum() == 4
        assert RealMap.from_seg(f, min_blob=1).occ.sum() == 5


def test_clear_swept_removes_cells_under_real_path():
    world = corridor()
    world.occ[world.occ.shape[0] // 2, -world.j0] = True            # 走廊正中一个障碍格(走动的人)
    n = world.occ.sum()
    xs = np.arange(-0.5, 7.0, 0.1)
    world.clear_swept(np.stack([xs, np.zeros_like(xs), np.zeros_like(xs)], 1), 0.45, 0.25, 0.1)
    assert world.occ.sum() == n - 1                                  # 两侧墙不动


def test_dead_band_contract_brakes_and_starts_neutral():
    """能刹车的契约(v4.5):开局队列是中位,刹车动作走发指令侧的松刹(RoadEnv.step 用 self.release)。"""
    pl = Plant(launch_delay=0.0, enc=spec_of('spica_v3_s5_v0.45-5_d0.2_log'))
    env = RealMapEnv(corridor(), pl, 0, (0.0, 0.0, 0.0), noise=False, duration=1.0)
    assert all(c == STOP for _, c in env.pending) and env.cmd[1] == STOP
    why = None
    while why is None:
        why = env.step((0.0, -1.0))
    assert why == 'timeout'
