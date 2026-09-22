#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_loc_health_core.py —— loc_health_core 状态机离线单测（纯 Python，pytest，无 ROS）。

覆盖 2026-09-20 用户点名的全部失效安全场景：
- 启动无数据 / 阈值未标定
- 静止且 AMCL 不更新（应判健康：静止不更新≠失效）
- 累计运动超门限后 AMCL 仍不更新（异常）；累计运动未达门限（静止/微动）不判失效
- scan/odom 断流
- TF 缺失 / 不可查 / 过期
- NaN / Inf / 无效四元数
- 协方差超阈值（位置、角度分开）、一致性超阈值
- header.stamp 回退
- 故障恢复后重新发 True（不锁死）

用单调时钟假时间（recv_t / now 秒），不依赖真实时钟、不依赖消息 header.stamp 作新鲜度依据。
"""
import math

import pytest

from loc_health_core import (
    HEALTHY, EVIDENCE_INSUFFICIENT, INPUT_ABNORMAL, TIME_ABNORMAL,
    TF_ABNORMAL, THRESHOLD_EXCEEDED, UNVERIFIED_THRESHOLDS,
    LocalizationHealth, Thresholds, verdict_to_bool,
)

# 一致的证据布局：amcl pose=(1,2)，odom=(0,0)，map→odom TF=(1,2,0) → odom 转 map 系=(1,2)=amcl，一致。
AMCL = dict(x=1.0, y=2.0, q=(1.0, 0.0, 0.0, 0.0), cov=(0.04, 0.09, 0.01))
ODOM = dict(x=0.0, y=0.0, q=(1.0, 0.0, 0.0, 0.0), v=(0.0, 0.0, 0.0))
TF_BASE = dict(x=1.0, y=2.0)
TF_ODOM = dict(dx=1.0, dy=2.0, dyaw=0.0)


def _feed(h, t=0.0, amcl=None, odom=None, tf_base=None, tf_odom=None):
    """按给定时刻喂入全部证据（缺省用一致布局）。"""
    a = amcl if amcl is not None else AMCL
    o = odom if odom is not None else ODOM
    h.on_amcl_pose(t, a["x"], a["y"], *a["q"], *a["cov"], stamp=100.0 + t)
    h.on_odom(t, o["x"], o["y"], *o["q"], *o["v"], stamp=100.0 + t)
    h.on_scan(t, stamp=100.0 + t)
    tb = tf_base if tf_base is not None else TF_BASE
    h.on_tf_base(t, True, tb["x"], tb["y"], stamp=100.0 + t)
    to = tf_odom if tf_odom is not None else TF_ODOM
    h.on_tf_odom(t, True, to["dx"], to["dy"], to["dyaw"], stamp=100.0 + t)


def _state(h, now):
    return h.evaluate(now)


def test_init_no_data():
    h = LocalizationHealth(Thresholds(verified=True))
    v = _state(h, 0.0)
    assert v.healthy is False and v.state == EVIDENCE_INSUFFICIENT


def test_unverified_thresholds_never_healthy():
    h = LocalizationHealth(Thresholds(verified=False))
    _feed(h)
    v = _state(h, 0.0)
    assert v.healthy is False and v.state == UNVERIFIED_THRESHOLDS
    assert verdict_to_bool(v) is False


def test_healthy_when_all_fresh_and_bounded():
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h)
    v = _state(h, 0.0)
    assert v.healthy is True and v.state == HEALTHY
    assert verdict_to_bool(v) is True


def test_static_amcl_not_updating_is_healthy():
    """静止时 AMCL 不更新是正常现象，不是失效（用户判据局限 1 正面验证）。"""
    h = LocalizationHealth(Thresholds(verified=True))
    # amcl 在 t=0 更新，之后静止；其余证据在 t=5 仍新鲜
    _feed(h, t=0.0)
    h.on_scan(5.0, stamp=105.0)
    h.on_odom(5.0, ODOM["x"], ODOM["y"], *ODOM["q"], *ODOM["v"], stamp=105.0)
    h.on_tf_base(5.0, True, TF_BASE["x"], TF_BASE["y"], stamp=105.0)
    h.on_tf_odom(5.0, True, TF_ODOM["dx"], TF_ODOM["dy"], TF_ODOM["dyaw"], stamp=105.0)
    v = _state(h, 5.0)  # amcl_age=5s < amcl_stale_static=10s
    assert v.healthy is True and v.state == HEALTHY


def test_static_amcl_long_no_update_is_healthy():
    """静止无论多久，AMCL 不更新都不判失效（累计运动不足门限，无固定超时）。"""
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)
    # 静止 60s，amcl 停更，但累计运动恒 0，scan/tf/odom 保持新鲜 → 健康
    for t in (5.0, 20.0, 60.0):
        h.on_scan(t, stamp=100.0 + t)
        h.on_odom(t, ODOM["x"], ODOM["y"], *ODOM["q"], *ODOM["v"], stamp=100.0 + t)
        h.on_tf_base(t, True, TF_BASE["x"], TF_BASE["y"], stamp=100.0 + t)
        h.on_tf_odom(t, True, TF_ODOM["dx"], TF_ODOM["dy"], TF_ODOM["dyaw"], stamp=100.0 + t)
    v = _state(h, 60.0)  # amcl_age=60s，但累计运动=0，不判失效
    assert v.healthy is True and v.state == HEALTHY


def test_moving_amcl_not_updating():
    """累计位移超门限后 AMCL 仍不更新 → 异常（累计运动判据，非瞬时速度）。"""
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)  # amcl 更新于 t=0，此时 odom=(0,0)，累计运动起点=(0,0)
    # odom 位置随移动推进到 (1.0,0)，累计位移 1.0m > motion_cum_linear=0.5；amcl 停在 t=0
    for t, ox in ((0.5, 0.3), (1.0, 0.6), (1.5, 1.0)):
        h.on_scan(t, stamp=100.0 + t)
        h.on_odom(t, ox, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, stamp=100.0 + t)
        h.on_tf_base(t, True, 1.0, 2.0, stamp=100.0 + t)
        h.on_tf_odom(t, True, 1.0, 2.0, 0.0, stamp=100.0 + t)
    v = _state(h, 1.5)  # 累计位移 1.0m > 0.5，amcl_age=1.5s > 1s
    assert v.state == TIME_ABNORMAL and "累计运动" in v.reasons[0]


def test_scan_stop():
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)
    # scan 停，其余在 t=2 新鲜
    h.on_odom(2.0, ODOM["x"], ODOM["y"], *ODOM["q"], *ODOM["v"], stamp=102.0)
    h.on_tf_base(2.0, True, TF_BASE["x"], TF_BASE["y"], stamp=102.0)
    h.on_tf_odom(2.0, True, TF_ODOM["dx"], TF_ODOM["dy"], TF_ODOM["dyaw"], stamp=102.0)
    v = _state(h, 2.0)
    assert v.state == TIME_ABNORMAL and "/scan 断流" in v.reasons[0]


def test_odom_stop():
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)
    # odom 停，其余新鲜
    h.on_scan(2.0, stamp=102.0)
    h.on_tf_base(2.0, True, TF_BASE["x"], TF_BASE["y"], stamp=102.0)
    h.on_tf_odom(2.0, True, TF_ODOM["dx"], TF_ODOM["dy"], TF_ODOM["dyaw"], stamp=102.0)
    v = _state(h, 2.0)
    assert v.state == TIME_ABNORMAL and "/odom_combined 断流" in v.reasons[0]


def test_tf_missing():
    h = LocalizationHealth(Thresholds(verified=True))
    h.on_amcl_pose(0.0, AMCL["x"], AMCL["y"], *AMCL["q"], *AMCL["cov"], stamp=100.0)
    h.on_odom(0.0, ODOM["x"], ODOM["y"], *ODOM["q"], *ODOM["v"], stamp=100.0)
    h.on_scan(0.0, stamp=100.0)
    v = _state(h, 0.0)
    assert v.state == TF_ABNORMAL


def test_tf_not_lookup():
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)
    h.on_tf_base(1.0, False, 0.0, 0.0, stamp=101.0)  # 当前不可查
    v = _state(h, 1.0)
    assert v.state == TF_ABNORMAL


def test_nan_position():
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)
    h.on_amcl_pose(1.0, float("nan"), 2.0, 1.0, 0.0, 0.0, 0.0, 0.04, 0.09, 0.01, stamp=101.0)
    v = _state(h, 1.0)
    assert v.state == INPUT_ABNORMAL


def test_inf_covariance():
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)
    h.on_amcl_pose(1.0, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0, float("inf"), 0.09, 0.01, stamp=101.0)
    v = _state(h, 1.0)
    assert v.state == INPUT_ABNORMAL


def test_bad_quaternion():
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)
    h.on_amcl_pose(1.0, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.09, 0.01, stamp=101.0)
    v = _state(h, 1.0)
    assert v.state == INPUT_ABNORMAL


def test_cov_position_exceed():
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)
    # cov_xx=4.0 → σ=2.0 > cov_pos_sigma=0.5
    h.on_amcl_pose(1.0, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 4.0, 0.09, 0.01, stamp=101.0)
    v = _state(h, 1.0)
    assert v.state == THRESHOLD_EXCEEDED and "位置 σ" in v.reasons[0]


def test_cov_yaw_exceed():
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)
    # cov_yaw=1.0 → σ=1.0 rad > cov_yaw_sigma=0.5
    h.on_amcl_pose(1.0, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.04, 0.09, 1.0, stamp=101.0)
    v = _state(h, 1.0)
    assert v.state == THRESHOLD_EXCEEDED and "角度 σ" in v.reasons[0]


def test_consistency_exceed():
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)
    # map→odom TF 改恒等，odom 转 map 系=(0,0)，与 amcl(1,2) 差 2.236 > 0.5
    h.on_tf_odom(1.0, True, 0.0, 0.0, 0.0, stamp=101.0)
    v = _state(h, 1.0)
    assert v.state == THRESHOLD_EXCEEDED and "一致性" in v.reasons[0]


def test_stamp_regress():
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)  # amcl stamp=100
    # 回退 2s > stamp_regress_tol=0.5
    h.on_amcl_pose(1.0, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.04, 0.09, 0.01, stamp=98.0)
    v = _state(h, 1.0)
    assert v.state == TIME_ABNORMAL and "回退" in v.reasons[0]


def test_recovery_after_fault():
    """故障恢复后重新发 True，不锁死。"""
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)
    assert _state(h, 0.0).healthy is True
    # 制造一次 NaN
    h.on_amcl_pose(1.0, float("nan"), 2.0, 1.0, 0.0, 0.0, 0.0, 0.04, 0.09, 0.01, stamp=101.0)
    assert _state(h, 1.0).state == INPUT_ABNORMAL
    # 下一帧正常 → 恢复健康
    h.on_amcl_pose(2.0, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.04, 0.09, 0.01, stamp=102.0)
    h.on_odom(2.0, ODOM["x"], ODOM["y"], *ODOM["q"], *ODOM["v"], stamp=102.0)
    h.on_scan(2.0, stamp=102.0)
    h.on_tf_base(2.0, True, TF_BASE["x"], TF_BASE["y"], stamp=102.0)
    h.on_tf_odom(2.0, True, TF_ODOM["dx"], TF_ODOM["dy"], TF_ODOM["dyaw"], stamp=102.0)
    assert _state(h, 2.0).healthy is True


def test_angular_motion_counts_as_moving():
    """累计转角超门限后 AMCL 仍不更新 → 异常（原地转向也算累计运动）。"""
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)  # amcl 更新于 t=0，odom yaw=0
    for t, yaw in ((0.5, 0.2), (1.0, 0.4), (1.5, 0.6)):
        h.on_scan(t, stamp=100.0 + t)
        h.on_odom(t, 0.0, 0.0, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2),
                  0.0, 0.0, 0.1, stamp=100.0 + t)
        h.on_tf_base(t, True, 1.0, 2.0, stamp=100.0 + t)
        h.on_tf_odom(t, True, 1.0, 2.0, 0.0, stamp=100.0 + t)
    v = _state(h, 1.5)  # 累计转角 0.6rad > motion_cum_angular=0.5，amcl_age=1.5s > 1s
    assert v.state == TIME_ABNORMAL and "累计运动" in v.reasons[0]


def test_small_motion_below_threshold_is_healthy():
    """累计运动未达门限（微动）→ AMCL 不更新不判失效。"""
    h = LocalizationHealth(Thresholds(verified=True))
    _feed(h, t=0.0)
    # odom 从 (0,0) 微动到 (0.2,0)，累计位移 0.2 < motion_cum_linear=0.5；amcl 停更
    for t, ox in ((0.5, 0.1), (1.0, 0.2)):
        h.on_scan(t, stamp=100.0 + t)
        h.on_odom(t, ox, 0.0, 1.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, stamp=100.0 + t)
        h.on_tf_base(t, True, 1.0, 2.0, stamp=100.0 + t)
        h.on_tf_odom(t, True, 1.0, 2.0, 0.0, stamp=100.0 + t)
    v = _state(h, 1.0)  # 累计位移 0.2m < 0.5，不判失效
    assert v.healthy is True and v.state == HEALTHY


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
