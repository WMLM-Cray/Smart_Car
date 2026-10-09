"""Pure replay of the guard state machine in ``vehicle/road_car_node.py``.

Reference (this repository, ``main`` @ ``ad25f56``, merged 2026-10-07):

* ``vehicle/road_car_node.py:44`` ``GUARD_STOP = 0.25``
* ``vehicle/road_car_node.py:45`` ``STUCK_SECONDS = 3.0``
* ``vehicle/road_car_node.py:92-113`` ``RoadCarController.tick`` (guard branch)

Mirrored behaviour:

* the gap is measured along the **commanded** steer; only ``gap < GUARD_STOP``
  blocks, so ``None`` / ``NaN`` / ``inf`` never block (the deployed test is a
  plain ``<``);
* the first blocked tick latches ``block_t``; the steering angle keeps being
  published while the throttle goes to ``CarParams.pwm_mid`` (中位);
* ``event`` is ``'stuck'`` once ``now - block_t > STUCK_SECONDS``, else
  ``'guard'``; clearing the block clears ``block_t``/``event``, and re-enabling
  from released (``on_enable(True)``) clears them as well;
* **there is no reverse**: the policy never commands reverse and the lidar
  cannot see behind the car (``road_car_node.py:102-103``, ``:197-198``).

Scope: this module only states the guard. The deployed node does not import it,
and it deliberately does not model the throttle on unblocked ticks (that value
comes from ``vehicle/speed_loop.py``); ``GuardDecision.throttle`` is ``None``
there. Sensor staleness (``'stale'``) and the disabled case belong to
``common.road_runtime.RoadRuntime.decide``, not to the guard.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from common.params import CarParams
from common.pwm import steer_to_turn

GUARD_STOP = 0.25           # 同 vehicle/road_car_node.py:44
STUCK_SECONDS = 3.0         # 同 vehicle/road_car_node.py:45


@dataclass(frozen=True)
class GuardDecision:
    """One guard decision.

    event: None = 放行；'guard' = 截停；'stuck' = 连续截停超过 STUCK_SECONDS。
    blocked_s: now - block_t；未截停时为 None。
    throttle: 截停时为 CarParams.pwm_mid（中位）；放行时为 None（油门由 SpeedLoop 出）。
    steer: 实发舵机刻度；截停时方向照发。
    """

    event: str | None
    blocked_s: float | None
    throttle: float | None
    steer: int


class DeploymentGuardContract:
    """Replay of ``RoadCarController.tick``'s guard branch, wind tunnel only."""

    def __init__(self, params=None, threshold=GUARD_STOP, stuck_seconds=STUCK_SECONDS):
        for value, name in ((threshold, 'threshold'), (stuck_seconds, 'stuck_seconds')):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        self.p = params or CarParams()
        self.threshold = float(threshold)
        self.stuck_seconds = float(stuck_seconds)
        self.reset()

    def reset(self):
        self.block_t = None
        self.event = None
        self.gap = None

    def release(self):
        """Same effect as ``on_enable(True)`` after a release: drop the block timer.

        Faithful to ``road_car_node.py:87-89``: only ``block_t`` is cleared there;
        ``event`` keeps its last value until the next ``tick``.
        """
        self.block_t = None

    def step(self, now, gap, steer=0.0):
        now = float(now)
        if not math.isfinite(now):
            raise ValueError('now must be finite')
        servo = steer_to_turn(float(steer), self.p)
        if gap is not None and gap < self.threshold:
            if self.block_t is None:
                self.block_t = now
            self.event = 'stuck' if now - self.block_t > self.stuck_seconds else 'guard'
            self.gap = float(gap)
            return GuardDecision(self.event, now - self.block_t, self.p.pwm_mid, servo)
        self.block_t = None
        self.event = None
        return GuardDecision(None, None, None, servo)
