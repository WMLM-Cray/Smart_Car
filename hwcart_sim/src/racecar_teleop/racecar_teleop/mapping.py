"""Pure numeric mapping for ``racecar_teleop``.

This module has **no ROS dependency** and is unit-testable with plain pytest on
any Python >= 3.7.  It converts raw ``sensor_msgs/Joy`` axes/buttons into a
standard ``geometry_msgs/Twist``-style ``(linear.x, angular.z)`` pair for an
Ackermann (car-like) vehicle.

Conventions (see README):
  * positive speed axis  -> forward  (linear.x > 0, m/s)
  * positive steer axis  -> left turn (delta > 0 -> angular.z > 0, rad/s)
  * omega = v * tan(delta) / wheelbase   (bicycle model)
      - v == 0  => omega == 0  (no spin-in-place, no static steering)
      - reverse keeps the sign: v < 0, delta > 0  =>  omega < 0

The release-then-press enable state machine, input timeout and publishing are
**not** part of this module; they live in ``teleop_node.py``.  Here we only
compute the command for a single valid Joy message when the enable button is
held.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Parameters:
    """Tunable parameters, loaded from config/generic.yaml by the node.

    All numeric bounds are validated on construction; invalid parameters raise
    ``ValueError`` so the node can fail fast at startup with a clear reason.
    Defaults are the generic dual-stick gamepad example (see config/).
    """

    speed_axis: int = 1              # index of the speed axis in Joy.axes
    steering_axis: int = 3           # index of the steering axis in Joy.axes
    speed_axis_inverted: bool = False
    steering_axis_inverted: bool = False
    enable_button: int = 5           # hold-to-enable button index
    low_speed_button: int = 4        # low-speed button index; -1 disables it
    deadzone: float = 0.08           # axis deadzone, in [0, 1)
    max_speed_forward: float = 0.5   # m/s
    max_speed_reverse: float = 0.3   # m/s
    max_steering_angle: float = 0.35  # rad (equivalent bicycle steer angle)
    wheelbase: float = 0.36           # m
    low_speed_scale: float = 0.4
    publish_rate: float = 20.0        # Hz
    input_timeout: float = 0.3        # s

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        """Raise ValueError with a clear message if any parameter is invalid."""
        errors = []

        def finite(x):
            return (isinstance(x, (int, float)) and not isinstance(x, bool)
                    and math.isfinite(x))

        def check(ok, msg):
            if not ok:
                errors.append(msg)

        check(finite(self.wheelbase) and self.wheelbase > 0,
              "wheelbase must be a finite number > 0, got {!r}".format(self.wheelbase))
        check(finite(self.max_speed_forward) and self.max_speed_forward >= 0,
              "max_speed_forward must be finite and >= 0, got {!r}".format(self.max_speed_forward))
        check(finite(self.max_speed_reverse) and self.max_speed_reverse >= 0,
              "max_speed_reverse must be finite and >= 0, got {!r}".format(self.max_speed_reverse))
        check(finite(self.max_steering_angle)
              and 0 < self.max_steering_angle < math.pi / 2,
              "max_steering_angle must be finite and in (0, pi/2), got {!r}"
              .format(self.max_steering_angle))
        check(finite(self.deadzone) and 0 <= self.deadzone < 1,
              "deadzone must be in [0, 1), got {!r}".format(self.deadzone))
        check(finite(self.publish_rate) and self.publish_rate > 0,
              "publish_rate must be finite and > 0, got {!r}".format(self.publish_rate))
        check(finite(self.input_timeout) and self.input_timeout > 0,
              "input_timeout must be finite and > 0, got {!r}".format(self.input_timeout))
        check(finite(self.low_speed_scale) and self.low_speed_scale > 0,
              "low_speed_scale must be finite and > 0, got {!r}".format(self.low_speed_scale))
        check(isinstance(self.speed_axis, int) and not isinstance(self.speed_axis, bool)
              and self.speed_axis >= 0,
              "speed_axis must be a non-negative int, got {!r}".format(self.speed_axis))
        check(isinstance(self.steering_axis, int) and not isinstance(self.steering_axis, bool)
              and self.steering_axis >= 0,
              "steering_axis must be a non-negative int, got {!r}".format(self.steering_axis))
        check(isinstance(self.enable_button, int) and not isinstance(self.enable_button, bool)
              and self.enable_button >= 0,
              "enable_button must be a non-negative int, got {!r}".format(self.enable_button))
        check(isinstance(self.low_speed_button, int) and not isinstance(self.low_speed_button, bool)
              and self.low_speed_button >= -1,
              "low_speed_button must be >= -1 (-1 = disabled), got {!r}"
              .format(self.low_speed_button))
        check(self.speed_axis != self.steering_axis,
              "speed_axis and steering_axis must be different axes")

        if errors:
            raise ValueError(
                "invalid racecar_teleop parameters:\n  - " + "\n  - ".join(errors)
            )


def apply_deadzone(value: float, deadzone: float) -> float:
    """Map a raw axis value through a deadzone with continuous rescaling.

    ``|value| <= deadzone`` -> 0.0.  Otherwise the magnitude is rescaled
    linearly from ``[deadzone, 1]`` onto ``[0, 1]`` so there is no jump at the
    deadzone edge.  The caller must ensure ``value`` is finite and clamped.
    """
    if abs(value) <= deadzone:
        return 0.0
    return (value - math.copysign(deadzone, value)) / (1.0 - deadzone)


def _axis_norm(raw: float, deadzone: float) -> float:
    """Clamp a raw axis to [-1, 1] then apply the deadzone."""
    raw = max(-1.0, min(1.0, raw))
    return apply_deadzone(raw, deadzone)


def compute_speed(speed_norm: float, params: Parameters) -> float:
    """Turn a normalized [-1,1] speed input into m/s.

    Positive = forward (bounded by max_speed_forward), negative = reverse
    (bounded by max_speed_reverse).  Forward/reverse limits may differ.
    """
    if speed_norm >= 0:
        return speed_norm * params.max_speed_forward
    return speed_norm * params.max_speed_reverse


def compute_steering(steer_norm: float, params: Parameters) -> float:
    """Turn a normalized [-1,1] steering input into an equivalent bicycle
    steering angle delta in radians (positive = left)."""
    return steer_norm * params.max_steering_angle


def compute_angular(v: float, delta: float, params: Parameters) -> float:
    """Ackermann/bicycle yaw rate: omega = v * tan(delta) / wheelbase.

    ``v == 0`` yields ``omega == 0`` (no spin-in-place).  The sign of ``delta``
    is preserved, so reverse + left steer gives a negative omega.
    """
    return v * math.tan(delta) / params.wheelbase


def validate_joy(axes, buttons, params: Parameters) -> bool:
    """Return True if the Joy message has usable values.

    Checks that every required index exists and that the values we read are
    finite (defends against NaN/Inf and truncated arrays).
    """
    if params.speed_axis >= len(axes) or params.steering_axis >= len(axes):
        return False
    if params.enable_button >= len(buttons):
        return False
    if params.low_speed_button >= 0 and params.low_speed_button >= len(buttons):
        return False
    for idx in (params.speed_axis, params.steering_axis):
        if not math.isfinite(axes[idx]):
            return False
    for idx in [params.enable_button] + (
            [params.low_speed_button] if params.low_speed_button >= 0 else []):
        if not math.isfinite(buttons[idx]):
            return False
    return True


def map_joy_to_twist(axes, buttons, params: Parameters):
    """Compute (linear.x, angular.z) from a single Joy message.

    Returns ``(0.0, 0.0)`` when movement is not permitted: enable not held,
    invalid/out-of-range/non-finite input, or zero requested speed.  This is a
    pure function of the message; the node layers the release-then-press state
    machine and the input timeout on top of it.
    """
    if not validate_joy(axes, buttons, params):
        return 0.0, 0.0
    if not buttons[params.enable_button]:
        return 0.0, 0.0

    speed_norm = _axis_norm(axes[params.speed_axis], params.deadzone)
    if params.speed_axis_inverted:
        speed_norm = -speed_norm

    steer_norm = _axis_norm(axes[params.steering_axis], params.deadzone)
    if params.steering_axis_inverted:
        steer_norm = -steer_norm

    v = compute_speed(speed_norm, params)
    delta = compute_steering(steer_norm, params)

    if params.low_speed_button >= 0 and buttons[params.low_speed_button]:
        v *= params.low_speed_scale

    omega = compute_angular(v, delta, params)
    return v, omega
