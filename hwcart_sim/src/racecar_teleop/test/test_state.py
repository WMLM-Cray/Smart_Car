"""Unit tests for racecar_teleop.state (enable/timeout state machine).

Run from the package root:

    python -m pytest test/test_state.py -v

No ROS required.  Covers task-spec sections 8.2 / 8.3 / 8.4 and 11.C.
"""
from racecar_teleop.mapping import Parameters
from racecar_teleop.state import TeleopStateMachine


def make_params(**overrides):
    base = dict(
        speed_axis=0,
        steering_axis=1,
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


def joy_axes(speed=0.8, steer=0.0):
    return [speed, steer]


# ---------------------------------------------------------------------------
# 8.4 startup / recovery gating
# ---------------------------------------------------------------------------

def test_startup_held_enable_does_not_move():
    p = make_params()
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(), [1, 0])  # enable already held at startup
    assert sm.on_tick(0.01) == (0.0, 0.0)
    assert sm.armed is False


def test_release_then_press_moves():
    p = make_params()
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(), [0, 0])   # observe release
    assert sm.armed is True
    sm.on_joy(0.01, joy_axes(), [1, 0])  # re-press
    v, w = sm.on_tick(0.02)
    assert v > 0


def test_release_stops_immediately():
    p = make_params()
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(), [0, 0])   # arm
    sm.on_joy(0.01, joy_axes(), [1, 0])  # move
    assert sm.on_tick(0.02)[0] > 0
    sm.on_joy(0.03, joy_axes(), [0, 0])  # release
    assert sm.on_tick(0.04) == (0.0, 0.0)


# ---------------------------------------------------------------------------
# 8.3 input timeout
# ---------------------------------------------------------------------------

def test_timeout_stops_and_requires_reenable():
    p = make_params(input_timeout=0.3)
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(), [0, 0])    # arm
    sm.on_joy(0.05, joy_axes(), [1, 0])   # move
    assert sm.on_tick(0.1)[0] > 0
    # stop sending; advance well past the timeout
    assert sm.on_tick(0.5) == (0.0, 0.0)
    assert sm.armed is False
    # input resumes with enable STILL held -> must NOT auto-move
    sm.on_joy(0.6, joy_axes(), [1, 0])
    assert sm.on_tick(0.61) == (0.0, 0.0)
    # release, then re-press -> moves again
    sm.on_joy(0.62, joy_axes(), [0, 0])
    sm.on_joy(0.63, joy_axes(), [1, 0])
    assert sm.on_tick(0.64)[0] > 0


def test_timeout_stops_within_timeout_plus_one_period():
    p = make_params(input_timeout=0.3, publish_rate=20.0)  # period 0.05 s
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(), [0, 0])
    sm.on_joy(0.05, joy_axes(), [1, 0])
    assert sm.on_tick(0.34)[0] > 0      # elapsed 0.29 < 0.3: still fresh
    assert sm.on_tick(0.36) == (0.0, 0.0)  # elapsed 0.31 > 0.3: stopped


def test_continuous_zero_while_timed_out():
    p = make_params(input_timeout=0.3)
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(), [0, 0])
    sm.on_joy(0.05, joy_axes(), [1, 0])
    for t in (0.5, 0.6, 0.7, 0.8):
        assert sm.on_tick(t) == (0.0, 0.0)


# ---------------------------------------------------------------------------
# 8.1 invalid input
# ---------------------------------------------------------------------------

def test_invalid_input_revokes_and_zero():
    p = make_params()
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(), [0, 0])   # arm
    sm.on_joy(0.01, joy_axes(), [1, 0])  # move
    assert sm.on_tick(0.02)[0] > 0
    sm.on_joy(0.03, [float('nan'), 0.0], [1, 0])  # bad axis
    assert sm.armed is False
    assert sm.on_tick(0.04) == (0.0, 0.0)


def test_out_of_range_button_index_revokes():
    p = make_params(enable_button=5)  # but Joy only has 2 buttons
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(), [0, 0])
    assert sm.armed is False
    assert sm.on_tick(0.01) == (0.0, 0.0)


# ---------------------------------------------------------------------------
# 8.2 no input yet / idle
# ---------------------------------------------------------------------------

def test_no_input_yet_outputs_zero():
    p = make_params()
    sm = TeleopStateMachine(p)
    assert sm.on_tick(0.0) == (0.0, 0.0)
