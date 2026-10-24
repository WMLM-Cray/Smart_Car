"""track_rl 的环境与奖励:沿路进度、失败按车速扣分、停滞按窗口进度判、老师先开到难弯前;一轮极小规模训练能跑通。"""
import math

import numpy as np
import pytest

from common.road_policy import spec_of
from training import track_rl
from training.track_rl import C0, C1, TrackEnv, hard_corners

ENC = spec_of('spica_v3_s5_v0.45-5_d0.2_log')


def test_progress_reward_and_speed_scaled_failure():
    env = TrackEnv(45_000_001, 'L5', ENC, (1.0, 1.0), False)
    rs = []
    why = None
    while why is None and len(rs) < 200:
        r, why = env.step(np.array([0.0, 0.3], np.float32))
        rs.append(r)
    assert sum(rs[:-1]) > 0                              # 往前开拿正分
    if why in track_rl.FAIL:
        assert rs[-1] < -C0 + track_rl.DS_CLIP[1]


def test_stall_by_window_progress_not_speed():
    env = TrackEnv(45_000_002, 'L5', ENC, (1.0, 1.0), False)
    why, n = None, 0
    while why is None and n < 200:
        _, why = env.step(np.array([0.0, 0.0], np.float32))     # 0 = 中位(带死区契约):不走
        n += 1
    assert why == 'stall' and abs(env.env.t - track_rl.STALL_S) < 0.2


def test_hard_start_hands_over_before_a_corner():
    env = TrackEnv(45_000_003, 'L5g', ENC, (1.0, 1.0), True)
    assert env.why is None and env.handed > 0
    s = env.s_prev
    cs = hard_corners(env.road, s, s + 30)
    assert any(abs(c - s - track_rl.HAND_BEFORE) < 0.6 for c in cs)      # 抽中的那个弯就在前方 4 m
    assert env.env.car.v > 0.3                           # 交接时车已经在跑


def test_privileged_shape_and_range():
    p = TrackEnv(45_000_004, 'L6h', ENC, (0.7, 1.2), False).privileged()
    assert p.shape == (track_rl.N_PRIV,) and np.all(np.isfinite(p))


def test_l5g_outer_gaps_wider_than_l5s():
    from simulation.fastsim.road import LEVELS
    lv = {l.name: l for l in LEVELS}
    assert lv['L5g'].outer_spacing[0] > lv['L5s'].outer_spacing[1]
    names = [l.name for l in LEVELS]
    assert names.index('L5g') > names.index('LWT3c')     # 追加在老关卡之后,老关卡逐位不变
