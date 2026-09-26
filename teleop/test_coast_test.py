"""coast_test.py 的判读逻辑测试。不需要 ROS —— 那边的 rclpy 导入在 main() 里。

只测 summarize()/verdict():它们决定"这次滑行算滑行还是算刹车",在车上跑之前
先用合成数据确认一遍,免得数据是对的、结论是反的。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from coast_test import MOVING, summarize, verdict


def ramp_down(v0, decel, hz=50.0, floor=0.0):
    """合成一条匀减速的速度曲线,到 floor 为止再补几帧 0。"""
    out, t, v = [], 1000.0, v0
    while v > floor:
        out.append((t, v))
        t += 1 / hz
        v -= decel / hz
    for _ in range(5):
        out.append((t, 0.0))
        t += 1 / hz
    return out


def test_v0_is_speed_at_release():
    # v0 取前 3 帧平均去噪,所以在减速段会略低于真实的松油瞬时速度:
    # 50Hz 下 3 帧跨 40ms,1 m/s² 大概低 0.02。这个偏差对判断无所谓。
    v0, _, _, _ = summarize(ramp_down(0.60, 1.0))
    assert 0.57 < v0 <= 0.60


def test_stop_time_matches_constant_decel():
    # 0.6 m/s 以 1.0 m/s² 减速,0.6s 到零(+ STOP_HOLD 那几帧的余量)
    _, stop_t, _, _ = summarize(ramp_down(0.60, 1.0))
    assert 0.55 < stop_t < 0.75


def test_distance_matches_v0_squared_over_2a():
    # 匀减速滑行距离 = v0²/(2a) = 0.36/2 = 0.18m
    _, _, dist, _ = summarize(ramp_down(0.60, 1.0))
    assert 0.16 < dist < 0.20


def test_decel_recovered():
    _, _, _, decel = summarize(ramp_down(0.60, 1.0))
    assert 0.8 < decel < 1.2


def test_never_stopping_reports_none_and_lower_bound_decel():
    # 一直在动:停车时间 None,减速度按整段算,是个下界(不该报成刹车)
    rolling = [(1000.0 + i / 50.0, 0.60) for i in range(400)]
    v0, stop_t, dist, decel = summarize(rolling)
    assert stop_t is None
    assert decel < 0.2
    assert verdict(decel, False) == '观察结束仍在运动，需结合目标 PWM 判断是否持续驱动'


def test_verdict_separates_coast_from_brake():
    assert verdict(0.5, True) == '自由滑行'
    assert verdict(4.0, True) == '电调在刹车'
    assert verdict(1.5, True) == '介于两者之间'


def test_too_few_samples_returns_none():
    assert summarize([]) is None
    assert summarize([(1000.0, 0.6), (1000.02, 0.5)]) is None


def test_brake_looks_different_from_coast():
    """同一个初速,刹车和滑行必须落进不同的判断 —— 这个脚本的全部意义。"""
    _, _, coast_d, coast_a = summarize(ramp_down(0.60, 0.5))
    _, _, brake_d, brake_a = summarize(ramp_down(0.60, 5.0))
    assert verdict(coast_a, True) == '自由滑行'
    assert verdict(brake_a, True) == '电调在刹车'
    assert coast_d > brake_d * 5          # 滑行距离差一个量级


def test_noise_below_moving_does_not_end_run_early():
    """单帧掉到 MOVING 以下不算停 —— 要连续 STOP_HOLD 帧。"""
    s = [(1000.0 + i / 50.0, 0.60) for i in range(20)]
    s[5] = (s[5][0], MOVING / 2)          # 一帧毛刺
    s += [(s[-1][0] + (i + 1) / 50.0, 0.0) for i in range(5)]
    _, stop_t, _, _ = summarize(s)
    assert stop_t > 0.3                   # 没有在第 5 帧就判停


def test_accel_guard_leaves_room_to_coast_to_a_stop():
    from coast_test import ABORT_GAP, COAST_DECEL, stop_gap
    assert stop_gap(0.0) == ABORT_GAP
    # 1 m/s 自由滑行要 v²/2a 米,固定 0.6m 是不够的
    assert stop_gap(1.0) == ABORT_GAP + 1.0 / (2 * COAST_DECEL)
    assert stop_gap(1.0) > 1.2


def test_speed_at_tells_driving_from_coasting():
    from coast_test import speed_at
    driving = [(i * 0.02, 0.47) for i in range(150)]                       # 接着出力
    coasting = [(i * 0.02, max(0.0, 0.9 - 0.78 * i * 0.02)) for i in range(150)]
    assert abs(speed_at(driving, 1.0) - 0.47) < 1e-9
    assert speed_at(coasting, 1.0) < 0.2
    assert speed_at([], 1.0) is None
    assert speed_at(driving, 5.0) is None                                  # 超出记录范围
