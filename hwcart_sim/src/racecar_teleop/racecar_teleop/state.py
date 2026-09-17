"""Enable/teleop state machine for racecar_teleop (pure logic, no ROS).

Implements the safety behaviours required by the task spec, independent of
rclpy so it can be unit-tested on any Python:

  * hold-to-enable (release -> stop immediately)
  * startup/recovery gating: a press only counts AFTER an enable release is
    observed, so the car never starts on a stale "held" state
  * throttle-centering gate (2026-09-17): after startup or any lock
    (timeout/gap/invalid), movement additionally requires that one valid Joy
    frame with the speed axis inside the deadzone has been observed.  This
    prevents "reconnect while the stick is still pushed": without it, a
    release+press of enable alone would start the car at the stick's current
    deflection.  The gate re-arms whenever the stick passes through the
    deadzone; the steering axis is NOT gated (steer alone produces no
    longitudinal motion).
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
        self._throttle_centered = False  # speed axis seen inside deadzone since lock
        self._last_axes = None
        self._last_buttons = None
        self._last_input_time = None     # monotonic seconds of last message
        self._timeout_occurred = False

    @property
    def armed(self):
        """True once an enable release has been observed since startup/lock."""
        return self._armed

    @property
    def throttle_centered(self):
        """True once the speed axis has been observed inside the deadzone
        since startup/lock (throttle-centering gate, 2026-09-17)."""
        return self._throttle_centered

    @property
    def timed_out(self):
        """True if the most recent tick stopped because of an input timeout."""
        return self._timeout_occurred

    def _lock(self):
        """Enter the locked state: revoke permission AND the throttle gate."""
        self._armed = False
        self._throttle_centered = False

    def on_joy(self, now, axes, buttons):
        """Handle a received Joy message (valid or not)."""
        if self._last_input_time is not None and \
                (now - self._last_input_time) > self.params.input_timeout:
            # Gap between consecutive Joy messages exceeded the input
            # timeout.  A tick may not have run inside that gap (blocked
            # executor etc.), so treat the gap itself as an observed
            # timeout: lock now, before this message refreshes the
            # timestamp.  Recovery still requires an enable release+press.
            # (2026-09-17 主线复验判据3b补丁:原版仅靠 on_tick 观察超时,
            #  断流窗口内无tick时恢复消息刷新时间戳可漏撤销使能。)
            self._timeout_occurred = True
            self._lock()
        self._last_input_time = now
        if not validate_joy(axes, buttons, self.params):
            # Garbage / truncated / non-finite input: revoke permission.
            # Do not update stored input; movement stays blocked regardless.
            self._lock()
            return
        self._last_axes = axes
        self._last_buttons = buttons
        if abs(axes[self.params.speed_axis]) <= self.params.deadzone:
            # Throttle observed centred: (re)open the throttle gate.  Absolute
            # value is used, so axis inversion does not change the gate.
            self._throttle_centered = True
        if not bool(buttons[self.params.enable_button]):
            # Enable released -> (re)arm.  Only after this may a later press
            # actually move the vehicle.
            self._armed = True

    def on_tick(self, now):
        """Return the ``(linear_x, angular_z)`` to publish this tick.

        Implements the timeout, the enable gate, the throttle-centering gate
        and continuous zero-command publishing while locked.
        """
        if self._last_input_time is not None and \
                (now - self._last_input_time) > self.params.input_timeout:
            self._timeout_occurred = True
            self._lock()
            return 0.0, 0.0
        self._timeout_occurred = False
        if self._last_axes is None or not self._armed or not self._throttle_centered:
            return 0.0, 0.0
        return map_joy_to_twist(self._last_axes, self._last_buttons,
                                self.params)
