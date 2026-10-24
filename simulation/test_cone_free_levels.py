"""v4 无锥桶级(LW*):边界只有墙 / EDGE / 无边界段,不出锥桶;老级别逐位不变。"""
import numpy as np

from simulation.fastsim.road import CONE, LEVELS, generate

LV = {l.name: l for l in LEVELS}


def test_cone_free_levels_have_no_cones():
    for name in ('LW3', 'LW4', 'LWO3', 'LWT3a', 'LWT3b', 'LWT3c'):
        for seed in range(15):
            road = generate(np.random.default_rng(seed), LV[name])
            assert len(road.cones) == 0, (name, seed)
            assert not np.any(road.mat_left == CONE) and not np.any(road.mat_right == CONE)


def test_cone_free_levels_mix_walls_and_edges():
    from simulation.fastsim.road import EDGE, WALL
    mats = np.concatenate([np.r_[r.mat_left, r.mat_right] for r in
                           (generate(np.random.default_rng(s), LV['LW3']) for s in range(20))])
    assert (mats == WALL).mean() > 0.2 and (mats == EDGE).mean() > 0.2


def test_old_levels_unchanged():
    # L3 仍然有锥桶(cone_free 默认关,抽随机数的次数和顺序不变)
    assert any(len(generate(np.random.default_rng(s), LV['L3']).cones) for s in range(5))
