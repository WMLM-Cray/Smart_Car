"""手动前进油门限加速(FWD_SLEW):摇杆一把推到底不瞬间给满。不 import ROS。"""
import ast
import os
import sys
import threading
from pathlib import Path

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.params import CarParams


@pytest.fixture
def srv():
    tree = ast.parse(Path(__file__).with_name('server.py').read_text(encoding='utf-8'))

    def safe(node):
        return isinstance(node, ast.Assign) and all(
            isinstance(c.func, ast.Name) and c.func.id == 'CarParams'
            for c in ast.walk(node) if isinstance(c, ast.Call))
    keep = [n for n in tree.body if safe(n) or getattr(n, 'name', '') in ('State', 'compute', 'creep_or_pwm')]
    ns = {'threading': threading, 'CarParams': CarParams}
    exec(compile(ast.Module(body=keep, type_ignores=[]), 'server_control', 'exec'), ns)
    ns['S'] = s = ns['State']()
    s.guard, s.mode, s.thr = 0, 2, 1000
    return ns


def run(ns, t0, t1, dt=0.05):
    out, t = [], t0
    while t <= t1 + 1e-9:
        ns['S'].last_t = t
        out.append(ns['compute'](t)[0])
        t += dt
    return out


def test_full_stick_ramps_at_slew_rate(srv):
    pwm = run(srv, 10.0, 12.0)
    start, cap, slew = srv['FWD_START'], srv['FWD_CAP'][2], srv['FWD_SLEW']
    assert pwm[0] <= start + 1
    assert all(b - a <= slew * 0.05 + 1 for a, b in zip(pwm, pwm[1:]))
    assert pwm[-1] == cap


def test_release_and_push_again_restarts_from_launch_pwm(srv):
    run(srv, 10.0, 12.0)
    srv['S'].thr = 0
    run(srv, 12.05, 12.5)
    srv['S'].thr = 1000
    assert run(srv, 12.55, 12.55)[0] <= srv['FWD_START'] + 1


def test_policy_caps_are_the_three_modes():
    assert CarParams().policy_mode_caps == (0.7, 1.2, 2.5)
