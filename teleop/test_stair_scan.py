"""stair_scan 的纯逻辑:每档的爬升速率必须是能起转的那个,终点正好落在候选值上。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from stair_scan import RATE, TICK, candidates, ramp  # noqa: E402


def test_ramp_ends_exactly_on_target():
    assert ramp(1540, 1583)[-1] == 1583


def test_ramp_rate_is_the_one_that_starts_the_esc():
    steps = ramp(1540, 1600)
    diffs = [b - a for a, b in zip([1540.0] + steps, steps)]
    assert max(diffs) <= RATE * TICK + 1e-9
    assert min(diffs[:-1]) >= RATE * TICK - 1e-9      # 不会越爬越慢(慢了电调不起转)
    assert 10.0 <= RATE <= 40.0


def test_candidates_include_both_ends():
    assert candidates(1560, 1620, 2)[0] == 1560
    assert candidates(1560, 1620, 2)[-1] == 1620
    assert candidates(1580, 1590, 1) == list(range(1580, 1591))


def test_ramp_honours_a_slower_rate():
    steps = ramp(1540, 1560, rate=5.0)
    assert len(steps) == 80                       # 20 PWM / (5 PWM/s * 0.05s)
    assert steps[-1] == 1560
