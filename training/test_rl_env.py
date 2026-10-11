"""RlEnv 适配层测试:接口形状、奖励语义、截断/终止、可复现性。

不跑老师三层检查(太慢),直接 generate + Plant.sample 抽一局 —— 测的是接口,不是可通性。
"""
from __future__ import annotations

import numpy as np
import pytest

from common.road_policy import DIM, FRAMES
from simulation.fastsim.road import LEVELS, generate
from simulation.fastsim.road_env import Plant
from training.rl_env import RlEnv


def _make(seed=123, start=(0, 0.0, 0.0), **kw):
    rng = np.random.default_rng(seed)
    lv = next(l for l in LEVELS if l.name == 'L3')
    road = generate(rng, lv)
    pl = Plant.sample(rng)
    return RlEnv(road, pl, seed, start=start, **kw)


def test_reset_shape():
    env = _make()
    x = env.reset()
    assert x.shape == (DIM * FRAMES,)
    assert x.dtype == np.float32


def test_step_contract():
    env = _make()
    env.reset()
    for _ in range(5):
        out = env.step(np.zeros(2, np.float32))
        assert len(out) == 5
        obs, r, term, trunc, info = out
        assert obs.shape == (DIM * FRAMES,)
        assert np.isfinite(r)
        assert isinstance(term, bool) and isinstance(trunc, bool)
        assert not term and not trunc
        assert info['reason'] is None
        for k in ('reason', 'progress', 'min_clear', 'time', 'cap', 'slip'):
            assert k in info
        assert info['time'] > 0.0


def test_reward_stationary_is_time_cost():
    """起步死区(launch_delay>=0.5s=10 拍)里车不动:Δs=0、无打滑,每拍奖励 = -time_cost。"""
    env = _make(time_cost=0.02, slip=0.5, crash=50.0)
    env.reset()
    for _ in range(8):
        _, r, term, trunc, _ = env.step(np.zeros(2, np.float32))
        assert not term and not trunc
        assert r == pytest.approx(-0.02)


def test_off_track_penalty():
    """起点横向偏 5m:第一步就出界,奖励 = -time_cost - crash(车没动,Δs=0)。"""
    env = _make(seed=7, start=(0, 5.0, 0.0), time_cost=0.02, crash=50.0, slip=0.5)
    env.reset()
    _, r, term, trunc, info = env.step(np.zeros(2, np.float32))
    assert term and not trunc
    assert info['reason'] == 'off_track'
    assert r == pytest.approx(-0.02 - 50.0)


def test_max_steps_truncation():
    """步数上限截断:reason=timeout、terminated=False(GAE 要 bootstrap)。"""
    env = _make(max_steps=3)
    env.reset()
    for k in range(3):
        _, r, term, trunc, info = env.step(np.zeros(2, np.float32))
        if k < 2:
            assert not term and not trunc and info['reason'] is None
        else:
            assert trunc and not term
            assert info['reason'] == 'timeout'


def test_reproducible():
    """同种子同动作序列:奖励与观测逐位一致(噪声种子固定)。"""
    acts = [np.array([0.1, 0.3], np.float32)] * 30
    outs = []
    for _ in range(2):
        env = _make(seed=42, time_cost=0.02, slip=0.5, crash=50.0, max_steps=1000)
        x0 = env.reset()
        rs, xs = [], []
        for a in acts:
            x, r, term, trunc, _ = env.step(a)
            rs.append(float(r)); xs.append(x)
            if term or trunc:
                break
        outs.append((x0, rs, xs))
    assert np.array_equal(outs[0][0], outs[1][0])
    assert outs[0][1] == pytest.approx(outs[1][1])
    for u, v in zip(outs[0][2], outs[1][2]):
        assert np.array_equal(u, v)


def test_reset_new_seed_changes_noise():
    env = _make(seed=1)
    x1 = env.reset()
    x2 = env.reset(seed=999)
    assert not np.array_equal(x1, x2)      # 噪声种子不同 -> 观测不同


def test_step_before_reset_raises():
    env = _make()
    with pytest.raises(RuntimeError):
        env.step(np.zeros(2))
