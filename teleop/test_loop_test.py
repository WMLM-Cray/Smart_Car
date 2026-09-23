"""loop_test.py 的纯逻辑部分。不需要 ROS。"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from loop_test import parse_plan, summarize  # noqa: E402


def test_parse_plan():
    assert parse_plan('0.75:2,0.9:1.5') == [(0.75, 2.0), (0.9, 1.5)]


@pytest.mark.parametrize('bad', ['0:2', '1.5:2', '0.8:0', '0.8'])
def test_parse_plan_rejects_nonsense(bad):
    with pytest.raises(ValueError):
        parse_plan(bad)


def test_summarize_uses_second_half_only():
    rows = [(1620, 0.2)] * 10 + [(1610, 0.8), (1612, 0.9)] * 5
    mean, pp, lo, hi = summarize(rows)
    assert (round(mean, 2), round(pp, 2), lo, hi) == (0.85, 0.1, 1610, 1612)


def test_summarize_empty():
    assert summarize([]) is None
