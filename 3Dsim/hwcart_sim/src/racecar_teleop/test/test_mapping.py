"""Unit tests for racecar_teleop.mapping (pure numeric mapping).

Run from the package root:

    python -m pytest test/test_mapping.py -v

These tests do NOT require ROS.  They cover the numeric acceptance items from
the task spec (sections 8.1 and 11.B) plus parameter validation.
"""
import math

import pytest

from racecar_teleop.mapping import Parameters, apply_deadzone, map_joy_to_twist


def make_params(**overrides):
    """Parameters with small indices so tests can use tiny Joy arrays."""
    base = dict(
        speed_axis=0,
        steering_axis=1,
        speed_axis_inverted=False,
        steering_axis_inverted=False,
        enable_button=0,
        low_speed_button=-1,
        deadzone=0.08,
        max_speed_forward=0.5,
        max_speed_reverse=0.3,
        max_steering_angle=0.35,
        wheelbase=0.36,
        low_speed_scale=0.4,
        publish_rate=20.0,
        input_timeout=0.3,
    )
    base.update(overrides)
    return Parameters(**base)


# ---------------------------------------------------------------------------
# 11.B numeric mapping
# ---------------------------------------------------------------------------

def test_stick_centered_outputs_zero():
    p = make_params()
    v, w = map_joy_to_twist([0.0, 0.0], [1, 0], p)
    assert (v, w) == (0.0, 0.0)


def test_enable_not_held_outputs_zero():
    p = make_params()
    v, w = map_joy_to_twist([0.8, 0.0], [0, 0], p)
    assert (v, w) == (0.0, 0.0)


def test_forward_positive():
    p = make_params()
    v, w = map_joy_to_twist([1.0, 0.0], [1, 0], p)
    assert v > 0
    assert v == pytest.approx(p.max_speed_forward)
    assert w == 0.0


def test_reverse_negative():
    p = make_params()
    v, w = map_joy_to_twist([-1.0, 0.0], [1, 0], p)
    assert v < 0
    assert v == pytest.approx(-p.max_speed_reverse)


def test_forward_left_turn_positive_omega():
    p = make_params()
    v, w = map_joy_to_twist([0.5, 0.5], [1, 0], p)
    assert v > 0
    assert w > 0


def test_forward_right_turn_negative_omega():
    p = make_params()
    v, w = map_joy_to_twist([0.5, -0.5], [1, 0], p)
    assert v > 0
    assert w < 0


def test_reverse_left_steer_negative_omega():
    # v < 0, delta > 0 -> omega < 0 (sign is NOT forced positive in reverse)
    p = make_params()
    v, w = map_joy_to_twist([-0.5, 0.5], [1, 0], p)
    assert v < 0
    assert w < 0


def test_zero_speed_steer_outputs_zero_omega():
    # Ackermann cannot steer while stationary through Twist
    p = make_params()
    v, w = map_joy_to_twist([0.0, 0.9], [1, 0], p)
    assert v == 0.0
    assert w == 0.0


def test_full_scale_clamped():
    # a raw axis can exceed 1.0; outputs must not exceed configured limits
    p = make_params()
    v, w = map_joy_to_twist([5.0, 3.0], [1, 0], p)
    assert v == pytest.approx(p.max_speed_forward)
    expected_w = p.max_speed_forward * math.tan(p.max_steering_angle) / p.wheelbase
    assert w == pytest.approx(expected_w)


def test_matches_bicycle_formula():
    p = make_params()
    speed_raw, steer_raw = 0.6, 0.3
    v, w = map_joy_to_twist([speed_raw, steer_raw], [1, 0], p)
    speed_norm = apply_deadzone(speed_raw, p.deadzone)
    steer_norm = apply_deadzone(steer_raw, p.deadzone)
    expected_v = speed_norm * p.max_speed_forward
    expected_delta = steer_norm * p.max_steering_angle
    expected_w = expected_v * math.tan(expected_delta) / p.wheelbase
    assert v == pytest.approx(expected_v)
    assert w == pytest.approx(expected_w)


def test_low_speed_mode_scales_speed():
    p = make_params(low_speed_button=1)
    v_normal, _ = map_joy_to_twist([1.0, 0.0], [1, 0], p)
    v_low, _ = map_joy_to_twist([1.0, 0.0], [1, 1], p)
    assert v_low == pytest.approx(v_normal * p.low_speed_scale)


def test_low_speed_mode_scales_omega_too():
    p = make_params(low_speed_button=1)
    _, w_normal = map_joy_to_twist([0.8, 0.6], [1, 0], p)
    _, w_low = map_joy_to_twist([0.8, 0.6], [1, 1], p)
    assert w_low == pytest.approx(w_normal * p.low_speed_scale)


# ---------------------------------------------------------------------------
# 8.1 deadzone / invalid input
# ---------------------------------------------------------------------------

def test_deadzone_jitter_outputs_zero():
    p = make_params(deadzone=0.08)
    v, w = map_joy_to_twist([0.05, -0.05], [1, 0], p)
    assert (v, w) == (0.0, 0.0)


def test_deadzone_edge_is_continuous():
    # just above deadzone gives a small (not full-scale) output: no jump
    p = make_params(deadzone=0.08)
    v, _ = map_joy_to_twist([0.081, 0.0], [1, 0], p)
    assert 0.0 < v < 0.1 * p.max_speed_forward


def test_out_of_range_axis_index_outputs_zero():
    p = make_params(speed_axis=3)  # Joy only has 2 axes
    v, w = map_joy_to_twist([0.5, 0.5], [1, 0], p)
    assert (v, w) == (0.0, 0.0)


def test_nan_axis_outputs_zero():
    p = make_params()
    v, w = map_joy_to_twist([float('nan'), 0.0], [1, 0], p)
    assert (v, w) == (0.0, 0.0)


def test_inf_axis_outputs_zero():
    p = make_params()
    v, w = map_joy_to_twist([float('inf'), 0.0], [1, 0], p)
    assert (v, w) == (0.0, 0.0)


def test_speed_axis_inverted_flips_speed():
    p = make_params(speed_axis_inverted=True)
    v, _ = map_joy_to_twist([0.5, 0.0], [1, 0], p)
    assert v < 0


def test_steering_axis_inverted_flips_omega():
    p = make_params(steering_axis_inverted=True)
    _, w = map_joy_to_twist([0.5, 0.5], [1, 0], p)
    assert w < 0


def test_both_axes_inverted_keeps_omega_sign():
    # inverting both v and delta flips the omega sign twice -> unchanged
    p = make_params(speed_axis_inverted=True, steering_axis_inverted=True)
    _, w = map_joy_to_twist([0.5, 0.5], [1, 0], p)
    assert w > 0


# ---------------------------------------------------------------------------
# parameter validation (section 9)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("field,value", [
    ("wheelbase", 0.0),
    ("wheelbase", -0.1),
    ("max_speed_forward", -0.1),
    ("max_speed_reverse", -0.1),
    ("max_speed_forward", float('nan')),
    ("max_steering_angle", 0.0),
    ("max_steering_angle", math.pi / 2),
    ("max_steering_angle", 2.0),
    ("deadzone", 1.0),
    ("deadzone", -0.1),
    ("publish_rate", 0.0),
    ("input_timeout", 0.0),
    ("low_speed_scale", 0.0),
    ("speed_axis", -1),
    ("enable_button", -1),
])
def test_invalid_parameters_raise(field, value):
    with pytest.raises(ValueError):
        make_params(**{field: value})


def test_same_speed_and_steering_axis_raises():
    with pytest.raises(ValueError):
        make_params(speed_axis=1, steering_axis=1)


def test_valid_defaults_construct():
    Parameters()  # default generic-gamepad params must pass validation
