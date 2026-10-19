"""动力学实测工具:安全检查、脚本流程、结果统计(不碰 ROS)。"""
import math

import numpy as np

from calibration.dynamics_test import (PROGS, PWM_MID, TICK, TURN_MID, Safety, Sensors, steer_of_turn,
                                       stop_distance, summarize)


def _sensors(**kw):
    s = Sensors(enc_t=100.0, imu_t=100.0, gap_fn=lambda steer: 9.0, imu=(0, 0, 0, 0, 0, 9.8))
    for k, v in kw.items():
        setattr(s, k, v)
    return s


def test_steer_curve_signs():
    assert steer_of_turn(TURN_MID) == 0.0
    assert steer_of_turn(141) > 0 > steer_of_turn(21)
    assert math.isclose(math.degrees(steer_of_turn(21)), -21.7)


def test_safety_reasons():
    ok = Safety()
    assert ok.check(100.0, _sensors(), TURN_MID, 100.0) is None
    assert '网页' in Safety().check(101.0, _sensors(enc_t=101.0), TURN_MID, 100.0)
    assert '编码器' in Safety().check(100.5, _sensors(), TURN_MID, 100.5)
    assert '围栏' in Safety(fence=3.0).check(100.0, _sensors(x=3.5), TURN_MID, 100.0)
    assert Safety().check(100.0, _sensors(imu=(0, 0, 0, 5.0, 0, 9.8)), TURN_MID, 100.0) is None   # 起步加速度不算倾斜
    assert '倾斜' in Safety().check(100.0, _sensors(roll=math.radians(20)), TURN_MID, 100.0)
    near = _sensors(v=2.0, gap_fn=lambda steer: stop_distance(2.0) - 0.1)
    assert '前方' in Safety().check(100.0, near, TURN_MID, 100.0)
    assert '雷达' in Safety().check(100.0, _sensors(gap_fn=lambda steer: None), TURN_MID, 100.0)
    assert Safety(guard=False).check(100.0, _sensors(gap_fn=lambda steer: None), TURN_MID, 100.0) is None
    s = Safety()
    s.estop = True
    assert s.check(100.0, _sensors(), TURN_MID, 100.0) == '急停'


def test_imu_must_be_fresh_even_when_encoder_and_heartbeat_are_current():
    for stamp in (0.0, 1.0, 99.6, 101.0, float('nan')):
        assert Safety().check(100.0, _sensors(imu_t=stamp, v=1.0), TURN_MID, 100.0) == 'IMU 掉线'
    assert Safety().check(100.0, _sensors(imu_t=99.9), TURN_MID, 100.0) is None


def _drive(prog, params, plant):
    """简单车:PWM >= 1594 才动,速度一阶跟随 (pwm-1594)*0.03+0.6;低于中位按 3 m/s² 减速。"""
    s = _sensors()
    gen = PROGS[prog](lambda: s, **params)
    rows = []
    for _ in range(5000):
        try:
            pwm, turn, ph = next(gen)
        except StopIteration:
            break
        s.v = plant(s.v, pwm)
        s.dist += s.v * TICK
        R = 0.35 / max(math.tan(abs(steer_of_turn(turn))), 1e-3)
        s.imu = (0, 0, s.v / R if turn != TURN_MID else 0.0, 0, 0, 9.8)
        rows.append((pwm, turn, ph, s.v))
    return rows


def _plant(v, pwm):
    if pwm >= 1594 or (v > 0.05 and pwm > PWM_MID + 4):
        tgt = 0.6 + max(0, pwm - 1594) * 0.03
        return v + (tgt - v) * 0.1
    if pwm < PWM_MID:
        return max(0.0, v - 3.0 * TICK)
    return max(0.0, v - 0.73 * TICK)


def test_accel_then_coasts_to_stop():
    rows = _drive('accel', dict(hi=1640, v_max=1.5, t_max=5, d_max=50, up_rate=40), _plant)
    ph = [r[2] for r in rows]
    assert ph[0] == 'launch' and 'accel' in ph and ph[-1] == 'coast'
    assert max(r[3] for r in rows) >= 1.5 and rows[-1][3] <= 0.03


def test_brake_releases_before_reverse():
    rows = _drive('brake', dict(hi=1630, v0=1.2, brake=1440, t_max=5, d_max=50, up_rate=40), _plant)
    br = [r for r in rows if r[2] == 'brake']
    assert br and all(r[0] == 1440 for r in br)
    after = rows[rows.index(br[-1]) + 1:]
    assert after and all(r[0] == PWM_MID for r in after)          # 降到放刹车速度后只发中位


def test_circle_holds_turn_and_stops_at_v_max():
    rows = _drive('circle', dict(turn=136, start=1600, rate=10, top=1680, v_max=1.5, t_max=20, up_rate=40), _plant)
    assert all(r[1] == 136 for r in rows)
    assert max(r[3] for r in rows) >= 1.5 and rows[-1][2] == 'coast'


def test_summary_numbers():
    t = np.arange(0, 6, 0.02)
    v = np.clip(t - 1.0, 0, 2.0)                                  # 1 s 后 1 m/s² 加到 2
    ph = np.where(t < 1.05, 'launch', 'accel')
    ph[t > 3.2] = 'coast'
    v[t > 3.2] = np.clip(2.0 - 0.75 * (t[t > 3.2] - 3.2), 0, None)
    rec = dict(t=t, v=v, phase=ph, pwm=np.full(len(t), 1640), wz=np.zeros(len(t)), ay=np.zeros(len(t)))
    out = summarize(rec, 'accel')
    assert abs(out['加速峰值_m/s2'] - 1.0) < 0.1
    assert abs(out['滑行平均减速_m/s2'] - 0.75) < 0.05
    assert abs(out['到 1.0 m/s_s'] - 2.0) < 0.05
