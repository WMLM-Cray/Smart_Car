"""v4 训练开关:默认关时采集逐位不变;--consist 的第二张图是同一帧另一种亚格偏移;lag_max 跑得通。"""
import numpy as np

from common.road_policy import V_LO, Encoding
from training.bev_train import run_episode

ENC = Encoding(3.0, V_LO, 3.0)
KW = dict(collect=True, enc=ENC, free=True, a_lat=1.47, cap_max=2.5)


def test_defaults_unchanged_and_pair_differs():
    a = run_episode(3, 'LW3', **KW)
    b = run_episode(3, 'LW3', **KW)
    assert np.array_equal(a[0], b[0]) and np.array_equal(a[2], b[2]) and a[4] is None
    c = run_episode(3, 'LW3', aug=(0.05, 0.017), pair=True, **KW)
    assert c[4].shape == c[0].shape
    assert (c[4] != c[0]).any(axis=1).mean() > 0.5          # 大多数帧两张图量化相位不同


def test_lag_max_runs():
    rows = [run_episode(100 + s, 'LW3', enc=ENC, a_lat=1.47, cap_max=2.5, lag_max=3) for s in range(3)]
    assert all(r is None or r['reason'] in ('goal', 'collision', 'off_track', 'timeout') for r in rows)
