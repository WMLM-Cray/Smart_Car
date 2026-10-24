"""simulation/fastsim/floorplan 随机楼层:墙是封闭的、车能去的地方连成一片、出发点安全、能接 BevEnv 开。"""
import math

import numpy as np
import pytest

from simulation.fastsim.floorplan import CELL, generate, sample_start


@pytest.fixture(scope='module')
def plan():
    rng = np.random.default_rng(3)
    world, info = generate(rng)
    return rng, world, info


def test_same_seed_same_plan():
    a, _ = generate(np.random.default_rng(5))
    b, _ = generate(np.random.default_rng(5))
    assert np.array_equal(a.occ, b.occ) and np.array_equal(a.free, b.free)


def test_walls_enclose_the_floor(plan):
    from scipy.ndimage import binary_dilation
    _, world, _ = plan
    ring = binary_dilation(world.free, np.ones((3, 3))) & ~world.free     # 地面外面紧挨的一圈:墙或实心家具
    assert world.occ[ring].all()
    x, y = (np.argwhere(world.free).mean(0) + (world.i0, world.j0)) * CELL
    r = world.raycast(x, y, np.linspace(-math.pi, math.pi, 360, endpoint=False), 100.0)
    assert np.all(np.isfinite(r))


def test_furniture_and_rooms_exist(plan):
    _, _, info = plan
    assert info['legs'].sum() > 20
    assert len(info['corridors']) >= 1
    assert info['reach'].sum() * CELL ** 2 > 40          # 车能去的地方 > 40 m²


def test_start_is_clear_and_env_runs(plan):
    rng, world, info = plan
    from dataclasses import replace

    from common.road_policy import V_LO, Encoding
    from simulation.fastsim.road_env import Plant
    from training.bev_env import BevEnv
    x, y, yaw = sample_start(rng, world, info)
    assert world.raycast(x, y, np.array([yaw]), 3.0)[0] > 1.5
    pl = replace(Plant.sample(np.random.default_rng(0), Encoding(3.0, V_LO, 3.0)), cap=0.7)
    b = BevEnv(None, pl, 0, (x, y, yaw), free=True, guard=True, world=world)
    for _ in range(10):
        assert b.step(np.array([0.0, -1.0])) is None
    assert b.seg is not None


def test_fast_raycast_matches_step_by_step(plan):
    """RealMap.raycast 按距离场跳步,结果要和逐个半格采样一样。"""
    rng, world, info = plan
    step = world.cell / 2
    for mx, mn in ((10.0, 0.0), (3.0, 0.15)):
        x, y, _ = sample_start(rng, world, info)
        a = np.linspace(-math.pi, math.pi, 720, endpoint=False)
        s = np.arange(max(mn, step), mx + step, step)
        hit, _ = world._lookup(world.occ, x + np.cos(a)[:, None] * s, y + np.sin(a)[:, None] * s)
        ref = np.where(hit.any(1), s[hit.argmax(1)] - step / 2, np.inf)
        got = world.raycast(x, y, a, mx, mn)
        assert np.all(np.isclose(got, ref, atol=1e-9) | (np.isinf(got) & np.isinf(ref)))
