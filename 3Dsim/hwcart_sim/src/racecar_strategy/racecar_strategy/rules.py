"""Event ledger, not a perception detector. Times must come from the simulation clock.
Rules: 2026-08 draft, sections 2.4.1 and 2.4.2. Ambiguous stacking is caller-adjudicated.
"""
from dataclasses import dataclass, field
import math

PENALTIES = dict(cone_collision=5, zigzag_line=10, wrong_cone=5,
                 red_crossing=30, red_no_stop=50, red_wheel_outside=20,
                 parking_failure=20, map_unclear=5, map_invalid=50,
                 ros1_present=100, human_intervention=100)

def seconds(value):
    if not math.isfinite(value) or value < 0:
        raise ValueError('time must be finite and nonnegative')
    return value

def effective_time(lap1, lap2, penalties, red1, red2):
    for value in (lap1, lap2, penalties, red1, red2):
        seconds(value)
    if red1 > lap1 or red2 > lap2:
        raise ValueError('red wait cannot exceed its lap duration')
    # Literal official formula: subtract the unweighted sum after weighting laps.
    return .3 * lap1 + .7 * lap2 + penalties - red1 - red2

def score(t, tmin, tmax, manual):
    if not all(math.isfinite(x) for x in (t, tmin, tmax, manual)):
        raise ValueError('nonfinite score input')
    if not tmin <= t <= tmax or tmax <= tmin or not 0 <= manual <= 20:
        raise ValueError('invalid ranking interval or manual score')
    return ((tmax - t) / (tmax - tmin)) ** 2 * 80 + manual

@dataclass
class Ledger:
    events: list = field(default_factory=list)
    failed: bool = False
    _ids: set = field(default_factory=set)
    _line_laps: set = field(default_factory=set)
    wrong_cones: int = 0

    @property
    def penalty_seconds(self):
        return sum(e['penalty'] for e in self.events)

    def record(self, event_id, kind, stamp, lap):
        seconds(stamp)
        if lap not in (1, 2) or kind not in PENALTIES or not event_id:
            raise ValueError('invalid event')
        if event_id in self._ids:
            return False
        if self.events and stamp < self.events[-1]['stamp']:
            raise ValueError('clock moved backwards: start a new ledger after reset')
        self._ids.add(event_id)
        if kind == 'zigzag_line' and lap in self._line_laps:
            return False
        if kind == 'zigzag_line':
            self._line_laps.add(lap)
        if kind == 'wrong_cone':
            self.wrong_cones += 1
            self.failed |= self.wrong_cones > 3
        self.events.append(dict(id=event_id, kind=kind, stamp=stamp,
                                lap=lap, penalty=PENALTIES[kind]))
        return True

    def check_timeout(self, stopped_seconds=0., offtrack_seconds=0.):
        seconds(stopped_seconds)
        seconds(offtrack_seconds)
        # Strict >10 per draft; final successful parking is outside active racing.
        self.failed |= stopped_seconds > 10 or offtrack_seconds > 10
        return self.failed

def parking_success(wheels_inside, expected, observed):
    if len(wheels_inside) != 4 or any(type(x) is not bool for x in wheels_inside):
        raise ValueError('four boolean wheel-containment results required')
    if expected not in ('A', 'B') or observed not in ('A', 'B'):
        raise ValueError('parking labels must be A/B')
    return sum(wheels_inside) >= 3 and expected == observed
