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


def test_no_yolo_keeps_cones_on_the_road_but_empties_the_cone_layer():
    from training.bev_env import BevEnv
    from training.road_check import sample_episode
    road, pl, start, _, _ = sample_episode(11, 'L5', enc=ENC, cap_max=2.5, a_lat=1.47)
    assert len(road.cones)                                  # 路上有锥桶
    on = BevEnv(road, pl, 11, start, noise=False, free=True)
    off = BevEnv(road, pl, 11, start, noise=False, free=True, yolo=False)
    for _ in range(20):
        on.step((0.0, 0.0)); off.step((0.0, 0.0))
    img_on, img_off = on.observation()[0], off.observation()[0]
    assert img_on[1].any() and not img_off[1].any()         # 锥桶层:有 YOLO 才有
    assert img_off[0].any() or img_off[4].any()             # 雷达照样扫到(锥桶 + 其他)
