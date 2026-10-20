#!/usr/bin/env python3
"""Tests for evaluation/gazebo_speed_supervisor.py. No ROS, no Gazebo, no subprocesses.

The clearance path is checked against whichever guard module gets loaded — this repo's
vehicle/scan_guard.py by default, which is also what a supervised run pins SCAN_GUARD_DIR to
(gazebo_supervised_runner.py:386); setting SCAN_GUARD_DIR by hand (e.g. to a legacy
<team_repo>/teleop/scan_guard.py) only switches to the old-geometry guard for comparison.
The supervisor must report bit-for-bit the same gap as calling ScanGuard directly, so every
guard constant is read from the loaded module.
"""
from __future__ import annotations

import math
import os
import random
import sys
from types import SimpleNamespace

import pytest

from common.control import to_cmd_vel
from common.params import CarParams
from evaluation import gazebo_speed_supervisor as s

def production_guard_dir():
    """Guard directory: SCAN_GUARD_DIR when set, else this repo's own vehicle/."""
    return os.environ.get('SCAN_GUARD_DIR') or s.SCAN_GUARD_DIR


GUARD_DIR = production_guard_dir()
ScanGuard = s.load_scan_guard(GUARD_DIR)

# scan_guard internals, used only to invert its own transform when building synthetic scans.
# Read off the module that actually got loaded: the in-repo guard and the sim-side one
# disagree (LIDAR_X 0.145 / NOSE_AHEAD 0.415 vs 0.07 / 0.43), so hardcoding either breaks
# the other — and the inversion has to match the transform under test, not a chosen one.
_GUARD_MOD = sys.modules[ScanGuard.__module__]
LIDAR_X = _GUARD_MOD.LIDAR_X
NOSE_AHEAD = _GUARD_MOD.NOSE_AHEAD


class FakeScan:
    """The subset of sensor_msgs/LaserScan that scan_guard.update() reads."""

    def __init__(self, ranges, angle_min=0.0, angle_increment=None, range_min=0.15,
                 range_max=50.0):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = ((2.0 * math.pi / len(ranges)) if angle_increment is None
                                else angle_increment)
        self.range_min = range_min
        self.range_max = range_max


class FakeOdom:
    def __init__(self, v):
        self.twist = SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=v)))


class ExplodingGuard:
    """A guard that raises on every gap() call (fail-safe path)."""

    def update(self, msg):
        pass

    def gap(self, steer):
        raise RuntimeError('gap blew up')


def scan_from_points(points, n=721, tube=0.05, noise=0.0, seed=7):
    """Build a LaserScan from obstacle points given in the rear-axle frame.

    Rays are 0..2pi like the Gazebo lidar, and each ray gets the nearest point that falls
    inside a narrow tube around it (infinite range otherwise, i.e. "nothing there").
    """
    rng = random.Random(seed)
    inc = 2.0 * math.pi / n
    ranges = []
    for i in range(n):
        ang = (i * inc + math.pi) % (2.0 * math.pi) - math.pi
        best = float('inf')
        for (x, y) in points:
            dx, dy = x - LIDAR_X, y
            along = dx * math.cos(ang) + dy * math.sin(ang)
            if along <= 0.0:
                continue
            perp = abs(-dx * math.sin(ang) + dy * math.cos(ang))
            if perp <= tube:
                best = min(best, along)
        if math.isfinite(best) and noise:
            best += rng.uniform(-noise, noise)
        ranges.append(best)
    return FakeScan(ranges, 0.0, inc)


SCENARIOS = [
    ([], [0.0, 0.2, -0.379]),
    ([(1.2, 0.0)], [0.0, 0.05, -0.05, 0.3, -0.3]),
    ([(0.8, 0.35)], [0.0, 0.3, -0.3]),
    ([(0.6, -0.5), (2.0, 0.1)], [0.0, 0.2, -0.2]),
    ([(0.25, 0.0)], [0.0, 0.379]),
    ([(3.0, 0.0), (1.5, 0.6)], [0.0, 0.3]),
    ([(-0.5, 0.0)], [0.0, 0.3]),
    ([(0.9, 0.0), (0.9, 0.12), (0.9, -0.12)], [0.0, 0.05, -0.05]),
]


def gap_for_case(points, steer, n=721):
    """(supervisor gap, direct production guard gap) for one synthetic scan."""
    scan = scan_from_points(points, n=n)
    core = s.SupervisorCore(ScanGuard())
    core.update_scan(scan)
    reference = ScanGuard()
    reference.update(scan)
    return core.gap_for(steer), reference.gap(steer)


def test_default_params_match_the_preregistered_values():
    assert s.DEFAULT_PARAMS == {
        'raw_topic': '/cmd_vel_raw',
        'out_topic': '/cmd_vel',
        'scan_topic': '/scan',
        'odom_topic': '/ackermann_steering_controller/odometry',
        'L': 0.053618,
        'u': 0.103084,
        'reserve_m': 0.0,
        'scan_stale_s': 0.5,
        'wheelbase': CarParams().wheelbase,
        'debug_csv': '',
        'publish_epsilon': 0.0,
        'a_lat_max': 0.0,                 # opt-in: 0 keeps the lateral cap off
    }
    # No machine path is baked in: the default is this repo's own vehicle/ directory.
    assert s.SCAN_GUARD_DIR == os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'vehicle')
    # The first 13 columns are frozen (existing parsers key off them); the lateral columns
    # are appended, never inserted.
    assert s.CSV_HEADER == ('t_wall,t_ros,v_raw,w_raw,gap,gap_valid,scan_age_s,v_measured,'
                            'v_allow,k,v_out,w_out,passthrough_reason,'
                            'a_lat_max,v_lat,v_allow_total')
    assert s.CSV_HEADER.split(',')[:13] == ('t_wall,t_ros,v_raw,w_raw,gap,gap_valid,scan_age_s,'
                                            'v_measured,v_allow,k,v_out,w_out,'
                                            'passthrough_reason').split(',')
    assert s.CLI_A_LAT_MAX == '--a-lat-max'
    assert s.load_scan_guard(GUARD_DIR) is ScanGuard
    assert ScanGuard.__module__ == 'scan_guard'
    assert ScanGuard().gap(0.0) is None          # no scan yet -> no gap


def test_stopping_distance_and_allowable_speed_are_inverses():
    for v in (0.05, 0.45, 1.0, 2.0, 3.0, 5.0):
        d = s.stopping_distance(v, s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u'])
        back = s.allowable_speed(d, s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u'])
        assert back == pytest.approx(v, rel=1e-9, abs=1e-12)
    # The v4 numbers imply the a = 1/(2u) deceleration quoted in the file header.
    assert 1.0 / (2.0 * s.DEFAULT_PARAMS['u']) == pytest.approx(4.8504, rel=1e-4)
    assert s.stopping_distance(3.0, s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']) == \
        pytest.approx(1.0885, rel=1e-3)
    assert s.allowable_speed(0.0, 0.05, 0.1) == 0.0
    assert s.allowable_speed(-1.0, 0.05, 0.1) == 0.0
    assert s.allowable_speed(float('nan'), 0.05, 0.1) == 0.0


def test_allowable_speed_is_monotonic_in_gap():
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    values = [s.allowable_speed(g, L, u) for g in (0.0, 0.02, 0.1, 0.5, 1.0, 2.0, 10.0)]
    assert values == sorted(values)
    assert values[0] == 0.0 and values[-1] > values[0]


def test_steer_from_yaw_rate_is_inverse_of_production_to_cmd_vel():
    wheelbase = s.DEFAULT_PARAMS['wheelbase']
    for v in (0.2, 0.45, 1.0, 3.0):
        for steer in (-0.379, -0.1, 0.0, 0.05, 0.379):
            _, omega = to_cmd_vel(v, steer)
            back = s.steer_from_yaw_rate(v, omega, wheelbase)
            assert back == pytest.approx(steer, rel=1e-12, abs=1e-12)
    assert s.steer_from_yaw_rate(1.0, 0.0, wheelbase) == 0.0


def test_gap_none_and_inf_pass_through_unchanged():
    for gap in (None, float('inf'), float('-inf'), float('nan')):
        out = s.decide(1.0, 0.3, gap, 0.1, None, **{'L': 0.053618, 'u': 0.103084})
        assert out['k'] == 1.0 and out['v_out'] == 1.0 and out['w_out'] == 0.3
        assert out['passthrough_reason'] == 'gap_unavailable'
        assert out['v_allow'] is None


def test_nonpositive_raw_speed_passes_through_for_reverse_escape():
    for v in (0.0, -0.1, -1.5):
        out = s.decide(v, 0.8, 0.05, 0.0, 1.0, **{'L': 0.053618, 'u': 0.103084})
        assert out['k'] == 1.0 and out['v_out'] == v and out['w_out'] == 0.8
        assert out['passthrough_reason'] == 'nonpositive_raw_speed'


def test_non_finite_raw_command_passes_through_and_is_sanitized_on_the_wire():
    out = s.decide(float('nan'), 0.2, 0.5, 0.0, None, **{'L': 0.053618, 'u': 0.103084})
    assert out['passthrough_reason'] == 'non_finite_raw_command'
    row = s._row(0.0, None, float('nan'), 0.2, 0.5, 0.0, None, out['k'], out['v_out'],
                 out['w_out'], out['v_allow'], out['passthrough_reason'])
    assert s.publish_values(row) == (0.0, 0.2)      # only the broken field is replaced
    both = s._row(0.0, None, float('nan'), float('inf'), 0.5, 0.0, None, 1.0, float('inf'),
                  float('nan'), None, 'non_finite_raw_command')
    assert s.publish_values(both) == (0.0, 0.0)
    assert s.publish_values({'v_out': 'x', 'w_out': None}) == (0.0, 0.0)


def test_stale_scan_passes_through_and_fresh_scan_does_not():
    clock = [0.0]
    core = s.SupervisorCore(ScanGuard(), scan_stale_s=0.5, now_fn=lambda: clock[0])
    scan = scan_from_points([(0.4, 0.0)])
    core.update_scan(scan, now=0.0)
    stale = core.command(1.0, 0.0, now=0.5 + 1e-6)
    assert stale['passthrough_reason'] == 'scan_stale'
    assert stale['k'] == 1.0 and stale['v_out'] == 1.0
    assert stale['gap_valid'] == 1                      # the guard itself still has data
    fresh = core.command(1.0, 0.0, now=0.5)             # exactly at the limit is not stale
    assert fresh['passthrough_reason'] == ''
    assert fresh['gap_valid'] == 1 and fresh['scan_age_s'] == 0.5
    # A 0.4m obstacle leaves 0.4 - NOSE_AHEAD < 0 clearance, so the command goes to zero.
    assert fresh['k'] == 0.0 and fresh['v_out'] == 0.0 and fresh['w_out'] == 0.0


def test_missing_scan_age_passes_through():
    core = s.SupervisorCore(ScanGuard())
    row = core.command(1.0, 0.0)
    assert row['scan_age_s'] is None
    assert row['passthrough_reason'] == 'gap_unavailable'
    assert row['k'] == 1.0


def test_scaling_preserves_curvature():
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    # gap 0.1m leaves v_allow = 0.7586 m/s, so every v_cmd above that must be cut.
    for v_cmd, omega in ((1.0, 0.3), (1.0, -0.3), (3.0, 0.9), (1.0, 0.0), (0.8, -0.0001)):
        out = s.decide(v_cmd, omega, 0.1, 0.0, v_cmd, L=L, u=u)
        assert out['passthrough_reason'] == ''
        assert 0.0 < out['k'] < 1.0
        assert out['v_out'] == pytest.approx(out['k'] * v_cmd, rel=1e-12)
        assert out['w_out'] == pytest.approx(out['k'] * omega, rel=1e-12, abs=1e-15)
        if omega == 0.0:
            assert out['w_out'] == 0.0
        else:
            assert out['w_out'] / out['v_out'] == pytest.approx(omega / v_cmd, rel=1e-12)
    # Very small v_cmd with omega = 0: the cut still never divides by zero.
    tiny = s.decide(0.02, 0.0, 0.001, 0.0, None, L=L, u=u)
    assert tiny['passthrough_reason'] == '' and tiny['w_out'] == 0.0
    assert tiny['v_out'] == pytest.approx(tiny['k'] * 0.02, rel=1e-12)
    # reserve_m reduces the usable clearance and therefore the allowed speed
    plain = s.decide(1.0, 0.0, 0.5, 0.0, 1.0, L=L, u=u)['v_allow']
    reserved = s.decide(1.0, 0.0, 0.5, 0.0, 1.0, L=L, u=u, reserve_m=0.3)['v_allow']
    assert reserved < plain


def test_denominator_uses_the_larger_of_command_and_measured_speed():
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    v_allow = s.allowable_speed(0.1, L, u)
    assert v_allow == pytest.approx(0.75864, rel=1e-4)
    coasting = s.decide(0.5, 0.0, 0.1, 0.0, 2.5, L=L, u=u)
    assert coasting['k'] == pytest.approx(v_allow / 2.5, rel=1e-12)
    assert coasting['v_out'] == pytest.approx(0.5 * v_allow / 2.5, rel=1e-12)
    no_odom = s.decide(0.5, 0.0, 0.1, 0.0, None, L=L, u=u)
    assert no_odom['passthrough_reason'] == 'clearance_sufficient'
    assert no_odom['v_out'] == 0.5
    slow_odom = s.decide(2.5, 0.0, 0.1, 0.0, 0.2, L=L, u=u)
    assert slow_odom['k'] == pytest.approx(v_allow / 2.5, rel=1e-12)
    unknown_odom = s.decide(0.5, 0.0, 0.1, 0.0, float('nan'), L=L, u=u)
    assert unknown_odom['k'] == 1.0 and unknown_odom['v_out'] == 0.5


def test_measured_speed_comes_from_odom_and_is_recorded():
    core = s.SupervisorCore(ScanGuard())
    assert core.v_measured is None and core.odom_count == 0
    core.update_odom(FakeOdom(1.25))
    assert core.v_measured == 1.25 and core.odom_count == 1
    core.update_odom(FakeOdom(-0.4))
    assert core.v_measured == -0.4


def test_publish_epsilon_treats_a_tiny_cut_as_passthrough():
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    gap = s.stopping_distance(0.99, L, u)
    scaled = s.decide(1.0, 0.0, gap, 0.0, 1.0, L=L, u=u)
    assert 0.0 < scaled['k'] < 1.0 and scaled['passthrough_reason'] == ''
    kept = s.decide(1.0, 0.0, gap, 0.0, 1.0, L=L, u=u, publish_epsilon=0.05)
    assert kept['k'] == 1.0 and kept['v_out'] == 1.0
    assert kept['passthrough_reason'] == 'within_publish_epsilon'


def test_exception_falls_back_to_publishing_the_raw_command():
    core = s.SupervisorCore(ExplodingGuard())
    errors = []
    row = s.supervised_command(core, 1.0, 0.2, t_ros=12.5, on_error=errors.append)
    assert row['passthrough_reason'] == 'exception:RuntimeError'
    assert row['k'] == 1.0 and row['v_out'] == 1.0 and row['w_out'] == 0.2
    assert s.publish_values(row) == (1.0, 0.2)
    assert isinstance(errors[0], RuntimeError) and 'gap blew up' in str(errors[0])


def test_exception_fallback_reports_one_explicit_stderr_line(capsys):
    core = s.SupervisorCore(ExplodingGuard())
    s.supervised_command(core, 1.0, 0.2)
    err = capsys.readouterr().err
    assert 'gazebo_speed_supervisor' in err and 'RuntimeError' in err
    assert 'publishing the raw command unchanged' in err


def test_guard_unavailable_still_forwards_every_command():
    core = s.SupervisorCore(None)
    row = s.supervised_command(core, 2.0, -0.5)
    assert row['passthrough_reason'] == 'guard_unavailable'
    assert s.publish_values(row) == (2.0, -0.5)
    garbage = s.supervised_command(core, None, 'x')
    assert garbage['passthrough_reason'] == 'guard_unavailable'
    assert s.publish_values(garbage) == (0.0, 0.0)


def test_zero_raw_command_publishes_zero():
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    core = s.SupervisorCore(ScanGuard(), L=L, u=u)
    core.update_scan(scan_from_points([(1.0, 0.0)]), now=0.0)
    row = core.command(0.0, 0.0, now=0.0)
    assert row['v_out'] == 0.0 and row['w_out'] == 0.0 and row['k'] == 1.0
    assert s.publish_values(row) == (0.0, 0.0)


def test_gap_geometry_matches_the_production_scan_guard_bit_for_bit():
    comparisons = 0
    for points, steers in SCENARIOS:
        for steer in steers:
            got, want = gap_for_case(points, steer)
            assert got == want, (points, steer, got, want)
            if want is None:
                assert got is None
            elif math.isinf(want):
                assert math.isinf(got)
            else:
                assert got == want and type(got) is type(want)
            comparisons += 1
    assert comparisons >= 20


def test_gap_geometry_matches_on_noisy_scans():
    for seed in range(6):
        points = [(0.9 + 0.1 * seed, 0.0), (1.6, 0.25 - 0.05 * seed), (2.4, -0.3)]
        scan = scan_from_points(points, n=401, noise=0.01, seed=seed)
        core = s.SupervisorCore(ScanGuard())
        core.update_scan(scan)
        reference = ScanGuard()
        reference.update(scan)
        for steer in (0.0, 0.1, -0.2, 0.379):
            assert core.gap_for(steer) == reference.gap(steer)


def test_failed_scan_update_leaves_the_core_in_a_passthrough_state():
    core = s.SupervisorCore(ScanGuard(), scan_stale_s=0.5)
    with pytest.raises(AttributeError):
        core.update_scan(object(), now=0.0)         # the node catches this and logs it
    assert core.scan_wall is None and core.scan_count == 0
    assert core.command(1.0, 0.0, now=0.0)['passthrough_reason'] == 'gap_unavailable'


def test_csv_row_matches_the_declared_header():
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    header = s.CSV_HEADER.split(',')
    assert len(header) == 16
    assert header[-3:] == ['a_lat_max', 'v_lat', 'v_allow_total']
    core = s.SupervisorCore(ScanGuard(), L=L, u=u)
    core.update_scan(scan_from_points([(0.5, 0.0)]), now=0.0)
    core.update_odom(FakeOdom(0.7))
    row = core.command(1.0, 0.2, t_ros=3.25, now=0.01)
    fields = s.format_csv_row(row).split(',')
    assert len(fields) == len(header)
    values = dict(zip(header, fields))
    assert values['t_ros'] == '3.250000'
    assert values['v_raw'] == '1.000000' and values['w_raw'] == '0.200000'
    assert values['gap_valid'] == '1' and values['v_measured'] == '0.700000'
    assert values['passthrough_reason'] == ''
    assert float(values['k']) < 1.0
    assert float(values['v_out']) == pytest.approx(float(values['k']), rel=1e-6)
    # The lateral term is off by default, so v_lat is +inf and v_allow_total is v_allow.
    assert values['a_lat_max'] == '0.000000' and values['v_lat'] == 'inf'
    assert values['v_allow_total'] == values['v_allow']
    empty = s.format_csv_row({'gap': None, 'gap_valid': 0, 'passthrough_reason': 'a,b',
                              'k': 1.0, 'v_out': 0.0, 'w_out': 0.0})
    assert empty.split(',')[4] == '' and empty.split(',')[5] == '0'
    assert empty.endswith('a;b,,,')                 # a comma in the reason never splits a field
    infinite = s.format_csv_row({'gap': float('inf'), 'gap_valid': 0, 'k': 1.0, 'v_out': 1.0,
                                 'w_out': 0.0, 'passthrough_reason': 'gap_unavailable'})
    assert infinite.split(',')[4] == 'inf'


def test_core_uses_the_injected_clock_for_scan_age():
    clock = [100.0]
    core = s.SupervisorCore(ScanGuard(), now_fn=lambda: clock[0])
    core.update_scan(scan_from_points([(2.0, 0.0)]))
    assert core.scan_count == 1
    clock[0] = 100.25
    assert core.scan_age() == pytest.approx(0.25)
    clock[0] = 99.0                                # monotonic never goes back; never negative
    assert core.scan_age() == 0.0


# --- lateral acceleration cap (opt-in --a-lat-max) -------------------------------------
# The first case of every tuple below was recorded from the file before the lateral term
# existed, so a regression that only shows up when both paths move together still fails.

DISABLED_REFERENCE = [
    # (v_cmd, omega, gap, scan_age_s, v_measured, k, v_out, w_out, v_allow, reason)
    (1.0, 0.3, 0.1, 0.0, 1.0, 0.7586153677194062, 0.7586153677194062, 0.22758461031582183,
     0.7586153677194062, ''),
    (3.0, 0.9, 0.5, 0.0, 3.0, 0.6525326985502672, 1.9575980956508015, 0.5872794286952405,
     1.9575980956508015, ''),
    (0.5, 0.0, 0.1, 0.0, None, 1.0, 0.5, 0.0, 0.7586153677194062, 'clearance_sufficient'),
    (2.5, 0.0, 0.1, 0.0, 0.2, 0.30344614708776246, 0.7586153677194062, 0.0,
     0.7586153677194062, ''),
    (1.0, 0.0, 0.4, 0.5, 1.0, 1.0, 1.0, 0.0, 1.7268796423636048, 'clearance_sufficient'),
    (0.02, 0.0, 0.001, 0.0, None, 0.9012879301792953, 0.018025758603585906, 0.0,
     0.018025758603585906, ''),
]


def test_a_lat_max_zero_is_bit_for_bit_the_old_behaviour():
    """(a) Disabled: k, v_out, w_out, v_allow and the reason are exactly the old values."""
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    assert s.DEFAULT_PARAMS['a_lat_max'] == 0.0
    for v_cmd, omega, gap, age, measured, k, v_out, w_out, v_allow, reason in DISABLED_REFERENCE:
        base = dict(L=L, u=u)
        old = s.decide(v_cmd, omega, gap, age, measured, **base)          # parameter omitted
        off = s.decide(v_cmd, omega, gap, age, measured, a_lat_max=0.0, **base)
        for out in (old, off):
            assert out['k'] == k and out['v_out'] == v_out and out['w_out'] == w_out
            assert out['v_allow'] == v_allow
            assert out['passthrough_reason'] == reason
            assert out['v_lat'] == float('inf') and out['v_allow_total'] == v_allow
            assert out['a_lat_max'] == 0.0
    # Negative and non-finite values are also "off", never a crash or a silent limit.
    for off_value in (-1.0, float('nan'), float('-inf')):
        out = s.decide(1.0, 0.3, 0.1, 0.0, 1.0, L=L, u=u, a_lat_max=off_value)
        assert out['k'] == DISABLED_REFERENCE[0][5]
        assert out['passthrough_reason'] == ''


def test_lateral_speed_limit_formula_and_off_cases():
    assert s.lateral_speed_limit(2.0, 1.5, 0.5) == pytest.approx(0.816496580927726, rel=1e-12)
    # kappa = |w| / v, so the sign of the turn does not matter.
    assert s.lateral_speed_limit(2.0, -1.5, 0.5) == s.lateral_speed_limit(2.0, 1.5, 0.5)
    # a_lat_max <= 0, a straight command, reverse and a non-finite command are all "no limit".
    assert s.lateral_speed_limit(2.0, 1.5, 0.0) == float('inf')
    assert s.lateral_speed_limit(2.0, 1.5, -3.0) == float('inf')
    assert s.lateral_speed_limit(2.0, 0.0, 0.5) == float('inf')
    assert s.lateral_speed_limit(2.0, 1e-12, 0.5) == float('inf')
    assert s.lateral_speed_limit(0.0, 1.5, 0.5) == float('inf')
    assert s.lateral_speed_limit(-0.5, 0.8, 0.5) == float('inf')
    assert s.lateral_speed_limit(float('nan'), 0.8, 0.5) == float('inf')
    # A larger budget is always at least as permissive.
    assert s.lateral_speed_limit(2.0, 1.5, 7.85) > s.lateral_speed_limit(2.0, 1.5, 0.5)


def test_straight_line_is_never_laterally_limited():
    """(b) w_raw == 0 -> kappa == 0 -> v_lat is +inf and the clearance cap is untouched."""
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    for gap, measured in ((0.1, 1.0), (50.0, 2.0), (50.0, None)):
        out = s.decide(2.0, 0.0, gap, 0.0, measured, L=L, u=u, a_lat_max=0.05)
        assert out['v_lat'] == float('inf')
        assert out['v_allow_total'] == out['v_allow']
        assert out['k'] == min(1.0, out['v_allow'] / max(2.0, measured or 2.0))
        assert out['w_out'] == 0.0
        assert out['passthrough_reason'] != 'lateral_limit'


def test_high_curvature_at_speed_is_capped_to_v_lat():
    """(c) k < 1 and k * v_raw == v_lat, with the lateral term as the binding constraint."""
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    v_cmd, omega, a_lat_max = 2.0, 1.5, 0.5
    out = s.decide(v_cmd, omega, 50.0, 0.0, v_cmd, L=L, u=u, a_lat_max=a_lat_max)
    v_lat = s.lateral_speed_limit(v_cmd, omega, a_lat_max)
    assert v_lat == pytest.approx(0.816496580927726, rel=1e-12)
    assert out['v_allow'] > v_lat                    # clearance is not what limited us
    assert out['v_lat'] == v_lat and out['v_allow_total'] == v_lat
    assert out['k'] < 1.0
    assert out['k'] * v_cmd == pytest.approx(v_lat, rel=1e-12)
    assert out['v_out'] == pytest.approx(v_lat, rel=1e-12)
    assert out['passthrough_reason'] == 'lateral_limit'
    # The realized lateral acceleration really is the budget, no more.
    assert abs(out['w_out']) * out['v_out'] == pytest.approx(a_lat_max, rel=1e-9)


def test_lateral_cap_never_loosens_a_clearance_limit():
    """The cap only ever takes the minimum: a tighter gap still wins."""
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    out = s.decide(2.0, 1.5, 0.1, 0.0, 2.0, L=L, u=u, a_lat_max=0.5)
    v_allow = s.allowable_speed(0.1, L, u)
    assert out['v_lat'] > v_allow                     # the gap is the binding one here
    assert out['v_allow_total'] == v_allow
    assert out['v_out'] == pytest.approx(v_allow, rel=1e-12)
    assert out['passthrough_reason'] == ''            # not 'lateral_limit': clearance did it
    # And a cap so generous it cannot bind leaves the old reason untouched.
    generous = s.decide(1.0, 0.3, 0.1, 0.0, 1.0, L=L, u=u, a_lat_max=7.85)
    assert generous['v_lat'] > generous['v_allow']
    assert generous['k'] == DISABLED_REFERENCE[0][5]
    assert generous['passthrough_reason'] == ''


def test_scaling_by_the_lateral_term_preserves_the_steering_angle():
    """(d) w and v share one factor, so delta = atan(w * wheelbase / v) is unchanged."""
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    wheelbase = s.DEFAULT_PARAMS['wheelbase']
    checked = 0
    for v_cmd, omega, a_lat_max in ((2.0, 1.5, 0.5), (3.0, -2.0, 1.0), (1.0, 0.6, 0.2),
                                    (0.5, 0.4, 0.1), (4.0, 4.0, 7.85)):
        out = s.decide(v_cmd, omega, 50.0, 0.0, v_cmd, L=L, u=u, a_lat_max=a_lat_max)
        assert out['k'] < 1.0, (v_cmd, omega, a_lat_max)
        before = math.atan(omega * wheelbase / v_cmd)
        after = math.atan(out['w_out'] * wheelbase / out['v_out'])
        assert after == pytest.approx(before, rel=1e-9, abs=1e-12)
        assert out['w_out'] == pytest.approx(out['k'] * omega, rel=1e-12, abs=1e-15)
        checked += 1
    assert checked == 5


def test_reverse_passes_through_verbatim_even_with_a_tiny_lateral_budget():
    """(e) v_raw <= 0 is never laterally limited; backing out stays an escape manoeuvre."""
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    for v in (0.0, -0.05, -0.5, -1.5):
        out = s.decide(v, 0.8, 0.05, 0.0, 1.0, L=L, u=u, a_lat_max=0.01)
        assert out['k'] == 1.0 and out['v_out'] == v and out['w_out'] == 0.8
        assert out['passthrough_reason'] == 'nonpositive_raw_speed'
        assert out['v_lat'] == float('inf')
    core = s.SupervisorCore(ScanGuard(), a_lat_max=0.01)
    core.update_scan(scan_from_points([(0.4, 0.0)]), now=0.0)
    row = core.command(-0.5, 0.8, now=0.0)
    assert row['passthrough_reason'] == 'nonpositive_raw_speed'
    assert s.publish_values(row) == (-0.5, 0.8)


def test_core_records_the_lateral_columns_and_still_scales_through_the_node_path():
    core = s.SupervisorCore(ScanGuard(), a_lat_max=0.5)
    # 0.9m of clearance allows 1.631 m/s, more than the 0.8165 m/s the lateral budget allows.
    core.update_scan(scan_from_points([(0.9, 0.0)]), now=0.0)
    row = core.command(2.0, 1.5, t_ros=1.0, now=0.0)
    assert row['a_lat_max'] == 0.5
    assert row['v_lat'] == pytest.approx(0.816496580927726, rel=1e-12)
    assert row['v_allow'] > row['v_lat']
    assert row['v_allow_total'] == row['v_lat']
    assert row['passthrough_reason'] == 'lateral_limit'
    assert s.publish_values(row) == (pytest.approx(row['v_out']), pytest.approx(row['w_out']))
    # guard unavailable: the lateral term is still a property of the raw command alone
    bare = s.SupervisorCore(None, a_lat_max=0.5).command(2.0, 1.5, now=0.0)
    assert bare['passthrough_reason'] == 'guard_unavailable'
    assert bare['v_lat'] == pytest.approx(0.816496580927726, rel=1e-12)
    assert bare['v_allow_total'] is None and bare['k'] == 1.0


def test_every_emitted_row_has_exactly_the_header_field_count():
    """(f) header length == field count for every row shape the supervisor can emit."""
    L, u = s.DEFAULT_PARAMS['L'], s.DEFAULT_PARAMS['u']
    header = s.CSV_HEADER.split(',')
    core = s.SupervisorCore(ScanGuard(), L=L, u=u, a_lat_max=0.5)
    core.update_scan(scan_from_points([(0.9, 0.0)]), now=0.0)
    core.update_odom(FakeOdom(0.7))
    rows = [
        core.command(2.0, 1.5, t_ros=1.0, now=0.0),            # scaled by the lateral term
        core.command(0.1, 0.0, t_ros=1.1, now=0.0),            # passthrough
        core.command(-0.5, 0.2, t_ros=1.2, now=0.0),           # reverse
        core.command(float('nan'), 0.2, t_ros=1.3, now=0.0),   # non-finite
        s.SupervisorCore(None, a_lat_max=0.5).command(1.0, 0.0, now=0.0),   # guard_unavailable
        s.supervised_command(s.SupervisorCore(ExplodingGuard(), a_lat_max=0.5), 1.0, 0.2,
                             on_error=lambda exc: None),       # exception:RuntimeError
    ]
    reasons = set()
    for row in rows:
        line = s.format_csv_row(row)
        assert len(line.split(',')) == len(header) == 16, line
        reasons.add(row['passthrough_reason'])
    assert {'lateral_limit', 'nonpositive_raw_speed', 'non_finite_raw_command',
            'guard_unavailable', 'exception:RuntimeError'} <= reasons
    # The lateral columns sit after the frozen 13 and are never empty on a live row.
    values = dict(zip(header, s.format_csv_row(rows[0]).split(',')))
    assert values['a_lat_max'] == '0.500000' and values['v_lat'] == '0.816497'
    straight = dict(zip(header, s.format_csv_row(rows[1]).split(',')))
    assert straight['a_lat_max'] == '0.500000' and straight['v_lat'] == 'inf'


def test_a_lat_max_flag_is_stripped_before_rclpy_sees_argv():
    assert s.parse_a_lat_max(['--a-lat-max', '0.5', '--ros-args', '-p', 'L:=0.05']) == \
        (0.5, ['--ros-args', '-p', 'L:=0.05'])
    assert s.parse_a_lat_max(['--a-lat-max=2.0']) == (2.0, [])
    assert s.parse_a_lat_max(['-p', 'a_lat_max:=1.0']) == (None, ['-p', 'a_lat_max:=1.0'])
    assert s.parse_a_lat_max([]) == (None, [])
    assert s.parse_a_lat_max(['--a-lat-max', '-1.0']) == (-1.0, [])
    with pytest.raises(SystemExit):
        s.parse_a_lat_max(['--a-lat-max'])
    with pytest.raises(SystemExit):
        s.parse_a_lat_max(['--a-lat-max', 'fast'])


def test_rclpy_argv_keeps_the_program_name_in_slot_zero():
    # rclpy parses from argv[1] and calls argv[0] the program name; if that slot goes missing,
    # '--ros-args' is eaten as the program name and every -p parameter is silently dropped.
    a_lat_max, argv = s.rclpy_argv(['prog', '--a-lat-max', '0.5', '--ros-args', '-p', 'L:=0.05'])
    assert a_lat_max == 0.5
    assert argv == ['prog', '--ros-args', '-p', 'L:=0.05']
    assert s.rclpy_argv(['prog', '--ros-args', '-p', 'L:=0.05']) == \
        (None, ['prog', '--ros-args', '-p', 'L:=0.05'])
    assert s.rclpy_argv(['prog']) == (None, ['prog'])
    # argv=None must still mean "read sys.argv", with the program name kept in front
    assert s.rclpy_argv(None) == (None, list(sys.argv))
