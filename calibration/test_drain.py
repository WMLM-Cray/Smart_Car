"""drain.py 的纯逻辑部分。不需要 ROS。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from calibration.drain import GROUND_HITS, PWM_CAP, UP_RATE, DT, ground_hits, next_pwm  # noqa: E402


def test_ramps_at_measured_rate_and_stops_at_target():
    pwm, seq = 1540.0, []
    for _ in range(200):
        pwm = next_pwm(pwm, 1615)
        seq.append(pwm)
    assert max(b - a for a, b in zip(seq, seq[1:])) <= UP_RATE * DT + 1e-9
    assert seq[-1] == 1615


def test_single_encoder_spike_is_not_ground():
    hits = 0
    for v in (0, 0.2, 0, 0, -0.2, 0):
        hits = ground_hits(hits, v)
        assert hits < GROUND_HITS


def test_sustained_speed_is_ground():
    hits = 0
    for _ in range(GROUND_HITS):
        hits = ground_hits(hits, 0.5)
    assert hits >= GROUND_HITS


def test_cap_matches_speed_loop():
    from vehicle.speed_loop import PWM_HARD_MAX
    assert PWM_CAP == PWM_HARD_MAX
