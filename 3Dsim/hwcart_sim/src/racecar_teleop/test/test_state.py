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


def test_first_frame_full_throttle_and_held_enable_still_zero():
    """首次连接(第一条 Joy)即"满杆+按住使能"也不动:默认锁定+油门门双钳。
    (2026-09-17 主线确认项:首次连接默认锁定。)"
    """
    p = make_params()
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(1.0), [1, 0])   # very first message: stick out + held
    assert sm.on_tick(0.01) == (0.0, 0.0)
    assert sm.armed is False and sm.throttle_centered is False


def test_release_then_press_moves():
    p = make_params()
    sm = TeleopStateMachine(p)
    # 松手动作=使能松开+摇杆回中同帧(2026-09-17 油门门:恢复需观测回死区)
    sm.on_joy(0.0, joy_axes(0.0), [0, 0])   # observe release + centred stick
    assert sm.armed is True and sm.throttle_centered is True
    sm.on_joy(0.01, joy_axes(), [1, 0])  # re-press
    v, w = sm.on_tick(0.02)
    assert v > 0


def test_release_stops_immediately():
    p = make_params()
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(0.0), [0, 0])   # arm (release + centred)
    sm.on_joy(0.01, joy_axes(), [1, 0])  # move
    assert sm.on_tick(0.02)[0] > 0
    sm.on_joy(0.03, joy_axes(), [0, 0])  # release
    assert sm.on_tick(0.04) == (0.0, 0.0)


# ---------------------------------------------------------------------------
# 8.4b throttle-centering gate (2026-09-17)
# ---------------------------------------------------------------------------

def test_recovery_with_stick_still_pushed_does_not_move():
    """锁定恢复时若摇杆仍在死区外,仅松开+重按使能不得起步;
    观测到回死区一帧后才允许杆值生效(防"重连后仍推着杆突然起步")。"""
    p = make_params()
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(0.0), [0, 0])   # arm
    sm.on_joy(0.05, joy_axes(0.8), [1, 0])  # move
    assert sm.on_tick(0.1)[0] > 0
    sm.on_tick(0.5)                          # timeout -> lock, throttle gate reset
    assert sm.armed is False and sm.throttle_centered is False
    # user reconnects with the stick STILL at 0.8: release+press alone must
    # not start the car
    sm.on_joy(0.6, joy_axes(0.8), [0, 0])   # release observed, stick still out
    sm.on_joy(0.61, joy_axes(0.8), [1, 0])  # re-press, stick still out
    assert sm.armed is True and sm.throttle_centered is False
    assert sm.on_tick(0.62) == (0.0, 0.0)
    # stick passes through the deadzone once (enable may stay held)
    sm.on_joy(0.63, joy_axes(0.0), [1, 0])
    sm.on_joy(0.64, joy_axes(0.8), [1, 0])
    assert sm.on_tick(0.65)[0] > 0


def test_reverse_stick_also_gated():
    """倒车方向的杆偏出死区同样受门约束(门取绝对值,与轴反转无关)。"""
    p = make_params()
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(0.0), [0, 0])
    sm.on_joy(0.05, joy_axes(-0.5), [1, 0])
    assert sm.on_tick(0.1)[0] < 0           # reverse works after centring
    sm.on_tick(0.5)                          # timeout lock
    sm.on_joy(0.6, joy_axes(-0.9), [0, 0])  # release with stick still reversed
    sm.on_joy(0.61, joy_axes(-0.9), [1, 0])
    assert sm.on_tick(0.62) == (0.0, 0.0)   # gated
    sm.on_joy(0.63, joy_axes(0.0), [1, 0])  # centred once
    sm.on_joy(0.64, joy_axes(-0.9), [1, 0])
    assert sm.on_tick(0.65)[0] < 0


# ---------------------------------------------------------------------------
# 8.3 input timeout
# ---------------------------------------------------------------------------

def test_timeout_stops_and_requires_reenable():
    p = make_params(input_timeout=0.3)
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(0.0), [0, 0])  # arm (release + centred)
    sm.on_joy(0.05, joy_axes(), [1, 0])   # move
    assert sm.on_tick(0.1)[0] > 0
    # stop sending; advance well past the timeout
    assert sm.on_tick(0.5) == (0.0, 0.0)
    assert sm.armed is False
    # input resumes with enable STILL held -> must NOT auto-move
    sm.on_joy(0.6, joy_axes(), [1, 0])
    assert sm.on_tick(0.61) == (0.0, 0.0)
    # release (with stick centred, per the throttle gate), then re-press
    sm.on_joy(0.62, joy_axes(0.0), [0, 0])
    sm.on_joy(0.63, joy_axes(), [1, 0])
    assert sm.on_tick(0.64)[0] > 0


def test_timeout_stops_within_timeout_plus_one_period():
    p = make_params(input_timeout=0.3, publish_rate=20.0)  # period 0.05 s
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(0.0), [0, 0])
    sm.on_joy(0.05, joy_axes(), [1, 0])
    assert sm.on_tick(0.34)[0] > 0      # elapsed 0.29 < 0.3: still fresh
    assert sm.on_tick(0.36) == (0.0, 0.0)  # elapsed 0.31 > 0.3: stopped


def test_continuous_zero_while_timed_out():
    p = make_params(input_timeout=0.3)
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(0.0), [0, 0])
    sm.on_joy(0.05, joy_axes(), [1, 0])
    for t in (0.5, 0.6, 0.7, 0.8):
        assert sm.on_tick(t) == (0.0, 0.0)


# ---------------------------------------------------------------------------
# 8.1 invalid input
# ---------------------------------------------------------------------------

def test_invalid_input_revokes_and_zero():
    p = make_params()
    sm = TeleopStateMachine(p)
    sm.on_joy(0.0, joy_axes(0.0), [0, 0])  # arm (release + centred)
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
