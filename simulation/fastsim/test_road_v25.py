"""road_v2.5(契约 scale 3.0、速度 0.45~3.0,和 spica_v3_s3_v0.45-3 相同)在仿真里的行为。

python -m pytest simulation/fastsim/test_road_v25.py
"""
import math
from dataclasses import replace

import numpy as np
import pytest

from common.road_policy import DEFAULT_ENC, DIM, FRAMES, RoadPolicy, spec_of
from simulation.fastsim.road import LEVELS, generate
from simulation.fastsim.road_env import Plant, RoadEnv
from training.road_check import sample_episode
from training.road_teacher import HORIZON, R_MIN, Teacher

V25 = spec_of('road_v2.5')
MODEL = 'models/road_v2.5/candidate_b036.npz'


def test_b036_loads_with_the_s3_contract():
    pol = RoadPolicy(MODEL)
    assert pol.enc == V25 == spec_of('spica_v3_s3_v0.45-3')
    assert pol.w[0].shape[1] == DIM * FRAMES


def test_cap_max_limits_only_the_cap_and_keeps_draw_order():
    a = Plant.sample(np.random.default_rng(7), V25)
    b = Plant.sample(np.random.default_rng(7), V25, cap_max=1.5)
    assert V25.v_lo <= b.cap <= 1.5
    assert replace(b, cap=a.cap) == a


def test_cap_max_episode_roads_match_for_the_same_seed():
    r1 = sample_episode(10_000_123, 'L3', enc=V25)
    r2 = sample_episode(10_000_123, 'L3', enc=V25, cap_max=1.5)
    np.testing.assert_array_equal(r1[0].centre, r2[0].centre)
    assert r2[1].cap <= 1.5


@pytest.mark.parametrize('a_lat', [0.30, 0.45])
def test_high_speed_teacher_is_slow_enough_at_the_end_of_its_window(a_lat):
    """看不到窗口外:高速契约下窗口末端必须已降到能过最急弯的速度;默认契约不加这条。"""
    v_end = math.sqrt(a_lat * R_MIN)
    rng = np.random.default_rng(1)
    road = generate(rng, next(l for l in LEVELS if l.name == 'L1'))
    assert road.length > HORIZON + 1
    pl = replace(Plant.sample(rng, V25), cap=3.0, v_top=3.0)
    t = Teacher(RoadEnv(road, pl, 1, noise=False), None, a_lat=a_lat)
    t._replan()
    assert t.v_end == pytest.approx(v_end) and t.plan.v[-1] <= v_end + 1e-9
    pl1 = replace(pl, enc=DEFAULT_ENC, cap=1.0, v_top=1.0)
    t1 = Teacher(RoadEnv(road, pl1, 1, noise=False), None, a_lat=a_lat)
    t1._replan()
    assert t1.v_end is None and t1.plan.v[-1] > v_end


def test_robust_noise_is_off_by_default_and_adds_spikes_and_blocks():
    """加强噪声默认关:不开时观测序列和原来逐拍一致(不多抽随机数);开了有编码器毛刺和雷达整段缺失。"""
    rng = np.random.default_rng(4)
    road = generate(rng, next(l for l in LEVELS if l.name == 'L1'))
    pl = Plant.sample(rng, V25)
    a, b = RoadEnv(road, pl, 4), RoadEnv(road, pl, 4, robust=False)
    for _ in range(60):
        np.testing.assert_array_equal(a.observation(), b.observation())
        a.step([0.0, 0.0]); b.step([0.0, 0.0])
    env = RoadEnv(road, pl, 5, robust=True)
    spikes, blocks = 0, 0
    for _ in range(600):
        o = env.observation()
        spikes += abs(o[48] * V25.scale - env.car.v) > 0.09
        blocks += int(np.isinf(env.ranges).mean() > 0.04)
        if env.step([0.0, 0.0]) is not None:
            break
    assert spikes > 0 and blocks > 0


def test_training_refuses_an_init_with_a_different_contract(tmp_path):
    """b036(s3 契约)不能直接灌进默认契约的训练:速度通道会按错的尺度解释,要先 rescale_spica。"""
    import subprocess
    import sys
    r = subprocess.run([sys.executable, '-m', 'training.road_train', '--out', str(tmp_path / 'x'),
                        '--init', MODEL, '--rounds', '1'], capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    assert r.returncode != 0 and 'rescale_spica' in r.stderr
