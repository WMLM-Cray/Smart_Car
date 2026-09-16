"""Enable/teleop state machine for racecar_teleop (pure logic, no ROS).

Implements the safety behaviours required by the task spec, independent of
rclpy so it can be unit-tested on any Python:

  * hold-to-enable (release -> stop immediately)
  * startup/recovery gating: a press only counts AFTER an enable release is
    observed, so the car never starts on a stale "held" state
  * input timeout -> lock + zero command, using a monotonic clock (never sim
    time, so pausing the simulation cannot freeze the safety timer)
  * invalid input (NaN/Inf/out-of-range) -> revoke permission + zero command

The rclpy node (next milestone) passes in monotonic timestamps and drives this
from the /joy subscription and the publish-rate timer.

Note on reconnect: a raw Joy stream cannot reliably distinguish "driver still
publishing the last valid state after unplug" from a real connection.  The
timeout here is the baseline safety net; any additional physical-connection
signal (e.g. a driver-specific "connected" flag) is the node's responsibility
and must be documented, not silently assumed solved.
"""
from __future__ import annotations

from racecar_teleop.mapping import map_joy_to_twist, validate_joy


class TeleopStateMachine:
    """Pure enable/timeout state machine.

    ``now`` values passed to ``on_joy``/``on_tick`` must all come from the same
    monotonic clock (e.g. ``time.monotonic()``); the class itself only compares
    them, it never reads a wall/sim clock.
    """

    def __init__(self, params):
        self.params = params
        self._armed = False              # "have we seen enable released yet"
        self._last_axes = None
        self._last_buttons = None
        self._last_input_time = None     # monotonic seconds of last message
        self._timeout_occurred = False

    @property
    def armed(self):
        """True once an enable release has been observed since startup/lock."""
        return self._armed

    @property
    def timed_out(self):
        """True if the most recent tick stopped because of an input timeout."""
        return self._timeout_occurred

    def on_joy(self, now, axes, buttons):
        """Handle a received Joy message (valid or not)."""
        self._last_input_time = now
        if not validate_joy(axes, buttons, self.params):
            # Garbage / truncated / non-finite input: revoke permission.
            # Do not update stored input; movement stays blocked regardless.
            self._armed = False
            return
        self._last_axes = axes
        self._last_buttons = buttons
        if not bool(buttons[self.params.enable_button]):
            # Enable released -> (re)arm.  Only after this may a later press
            # actually move the vehicle.
            self._armed = True

    def on_tick(self, now):
        """Return the ``(linear_x, angular_z)`` to publish this tick.

        Implements the timeout, the enable gate and continuous zero-command
        publishing while locked.
        """
        if self._last_input_time is not None and \
                (now - self._last_input_time) > self.params.input_timeout:
            self._timeout_occurred = True
            self._armed = False
            return 0.0, 0.0
        self._timeout_occurred = False
        if self._last_axes is None or not self._armed:
            return 0.0, 0.0
        return map_joy_to_twist(self._last_axes, self._last_buttons,
                                self.params)
