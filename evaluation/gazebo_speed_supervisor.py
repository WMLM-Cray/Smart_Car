#!/usr/bin/env python3
"""Self-owned speed supervisor: keep the commanded speed inside the braking budget.

WHY THIS EXISTS
    The production guard (闸门模块 `vehicle/scan_guard.py`；实车节点 `vehicle/road_car_node.py:44`
    `GUARD_STOP = 0.25`, 仓内实验宿主 `arms/arm_host.py` 同值，旧件是
    `Auto_Car-main/teleop/road_policy_node.py`)
    only starts stopping when the measured clearance is already below 0.25 m, but the car
    still needs d(v) = L*v + u*v^2 metres to stop after a zero command. The v4 empty-arena
    braking probe measured L = 0.053618 s and u = 0.103084 s^2/m (a = 1/(2u) = 4.8504 m/s^2,
    rms 0.0241 m), so at cap 3.0 m/s d(3.0) = 1.09 m - far more than the 0.25 m the guard
    leaves. This node sits between the policy and the controller and lowers the *speed* to
    what the *current* clearance can still absorb, scaling the yaw rate by the same factor
    so command curvature is unchanged; actual trajectory preservation is not guaranteed.

DESIGN
    * Event driven: every message on raw_topic triggers exactly one computation and exactly
      one publish on out_topic. There is no timer, so this node never invents a command.
    * The clearance is computed by the *production* guard module
      (SCAN_GUARD_DIR/scan_guard.py, imported read-only) instantiated exactly the way
      road_policy_node.py does: ScanGuard(), update(msg) once per scan, then gap(steer)
      with steer = atan(w * wheelbase / v) - the inverse of common/control.py to_cmd_vel.
    * v_cmd <= 0 (reverse / escape) and any unusable gap (never received, non-finite, stale)
      pass through unchanged. This node must never be the reason a command disappears or
      the car cannot back out.
    * Fail-safe: any exception (guard import failure, missing fields, NaN) publishes the raw
      command unchanged and writes one explicit line to stderr. It never silently drops a
      command.
    * Opt-in lateral cap (`--a-lat-max`, default 0 = off): the raw command already carries a
      curvature kappa = |w|/v, so v_lat = sqrt(a_lat_max / kappa) is the speed whose lateral
      acceleration is a_lat_max. It joins the clearance cap as v_allow_total = min(v_allow,
      v_lat) and the same k scales both fields, so the steering angle is untouched. This is a
      command-space limit only: the *actual* trajectory can still be tighter than the command
      implies (see KNOWN LIMITS 1-2).
    * The stale-scan age uses wall time.monotonic(), not ROS time: the age is a property of
      the process ("how long since a scan arrived"), and the braking probe already showed
      that this host can stall /clock for seconds while the process keeps running. A wall
      basis degrades to passthrough on stalls instead of freezing the supervisor.

KNOWN LIMITS (read before trusting any number this node produces)
    1. Only the *current* gap is used - no prediction, no lookahead. A clearance that
       collapses quickly because of steering or track geometry cannot be anticipated here;
       the node only reacts to what the lidar has already seen.
    2. L and u come from an empty-arena straight-line braking probe. Steering coupling and
       a full-track controller are not in that model, so the real stopping distance on a
       curved, loaded run can be longer than d(v) predicts.
    3. The lateral cap budgets the *commanded* curvature of a single raw message, not the
       curvature the car actually tracks, and a_lat_max is a caller-supplied assumption
       (common/params.py quotes 7.85 m/s^2 as an uncalibrated 0.8 g guess) - not a measured
       grip limit. The default 0 keeps this term off.

    This module imports no ROS at module level, so its pure parts are testable under a
    plain interpreter (including the training venv, which has no rclpy).

REVIEW STATUS (2026-10-05, source-checked read-only, no production file changed)
    * gap() here is an EXPERIMENTAL HEURISTIC, not a safety proof. It is the production
      ScanGuard's single-frame clearance for the current steer only: no obstacle prediction,
      no lidar/odom fusion, no braking-distance margin beyond the cap equation, and L/u were
      fitted on a straight empty-arena probe without steering coupling.
    * use_sim_time is NOT set by the production node (road_policy_node.py:106), so its 20 Hz
      timer (:141), its scan age (:152, :226) and ScanGuard's own stamp (scan_guard.py:59,
      time.time()) are all wall clock - self-consistent, with no sim/wall mixing bug. This
      node is wall-clock too (time.monotonic for age). The consequence is that a host stall
      is indistinguishable from a live run using wall time alone; whether a host stall also
      freezes simulation time is NOT asserted by this module - only measured /clock receipts
      (recorded by the launcher) are evidence.
    * The production node has NO odom age check: on_enc (road_policy_node.py:155-156) stores
      twist.linear.x with no timestamp, so a silent odom stream is invisible there. This node
      cannot fix that, but it uses max(v_cmd, v_measured) in the denominator so a stale reading
      can only lower the cap; odom receipts/age are recorded by the launcher timeline instead.
    * odom is never treated as world truth anywhere in this chain: the gap comes from /scan via
      the production guard, and world truth comes from /model_states (the braking probe and
      track_eval). odom only enters the speed cap denominator.
    * A dead supervisor does not by itself stop the car. In this simulation the stop comes from
      sim_wiring.py:33-58, whose cmd_timeout (0.5 SIM seconds) publishes a zero Twist when
      /cmd_vel goes silent; that is a sim-only watchdog and is NOT verified for the real-car
      stack (there the analogues are the /policy_enable heartbeat and neutral()).
"""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

# Guard module (read-only import). Default = this repo's own vehicle/scan_guard.py, so no
# machine path is baked in, and a supervised run pins SCAN_GUARD_DIR to **this repo's**
# vehicle/ (gazebo_supervised_runner.py:386) —— 面板那批数字用的就是实测几何那套闸门
# (LIDAR_X 0.145 / NOSE_AHEAD 0.415)，控制器与被仿真的车同一把尺。SCAN_GUARD_DIR 仍可
# 手工覆盖成旧件 <team_repo>/teleop/scan_guard.py（老几何 0.07 / 0.43）做对照，两套闸门
# 数值不一致。
SCAN_GUARD_DIR = str(Path(__file__).resolve().parent.parent / 'vehicle')

# Wheelbase: single-sourced from this repo's measured CarParams (common/params.py:26 = 0.35).
# The steer estimate below is defined as the inverse of common/control.py to_cmd_vel, which
# commands with CarParams().wheelbase, so the two MUST be the same number. It used to be the
# sim world's 0.33 (Auto_Car-main/3Dsim/hwcart_sim/src/racecar_sim/config/params.yaml), which
# made the round trip 5.7% off。
_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
from common.params import CarParams  # noqa: E402  (path set up just above)

WHEELBASE = CarParams().wheelbase

DEFAULT_PARAMS = {
    'raw_topic': '/cmd_vel_raw',
    'out_topic': '/cmd_vel',
    'scan_topic': '/scan',
    'odom_topic': '/ackermann_steering_controller/odometry',
    'L': 0.053618,
    'u': 0.103084,
    'reserve_m': 0.0,
    'scan_stale_s': 0.5,
    'wheelbase': WHEELBASE,
    'debug_csv': '',
    'publish_epsilon': 0.0,
    'a_lat_max': 0.0,
}

# The first 13 names and their order are frozen: existing parsers key off them. The three
# lateral columns are appended, never inserted.
CSV_HEADER = ('t_wall,t_ros,v_raw,w_raw,gap,gap_valid,scan_age_s,v_measured,v_allow,k,'
              'v_out,w_out,passthrough_reason,a_lat_max,v_lat,v_allow_total')

# Our own command-line flag. The node's tunables are otherwise ROS parameters (-p name:=value),
# but the lateral cap is specified as a plain flag; --ros-args/-p/-r still pass through to rclpy.
CLI_A_LAT_MAX = '--a-lat-max'


def load_scan_guard(directory=None):
    """Return the production ScanGuard class from SCAN_GUARD_DIR/scan_guard.py.

    Mirrors road_policy_node.py, which puts its own directory on sys.path and then does
    ``from scan_guard import ScanGuard``. Raises on failure; the caller turns that into a
    permanent passthrough (see make_node_class).
    """
    directory = SCAN_GUARD_DIR if directory is None else directory
    if directory not in sys.path:
        sys.path.insert(0, directory)
    from scan_guard import ScanGuard       # noqa: E402  (path is set up above)
    return ScanGuard


def guard_load_report(directory=None):
    """One machine-readable line proving whether the production ScanGuard really loaded.

    A candidate arm that silently degrades to permanent passthrough is not evidence for the
    supervisor, so the launcher greps this exact prefix from the node's own log instead of
    assuming the import worked. Returns dict(ok, scan_guard_py, report).
    """
    directory = SCAN_GUARD_DIR if directory is None else directory
    path = os.path.join(str(directory), 'scan_guard.py')
    try:
        cls = load_scan_guard(directory)
    except BaseException as exc:                                  # noqa: BLE001
        return dict(ok=False, scan_guard_py=path,
                    report='SUPERVISOR_GUARD_LOAD ok=0 error=%s: %s'
                           % (type(exc).__name__, exc))
    where = getattr(sys.modules.get(cls.__module__), '__file__', '?')
    return dict(ok=True, scan_guard_py=path, class_module=cls.__module__,
                report='SUPERVISOR_GUARD_LOAD ok=1 class_scan_guard=%s module=%s'
                       % (where, cls.__module__))


def stopping_distance(v, L, u):
    """d(v) = L*v + u*v^2 [m]: the measured empty-arena stopping distance from a v command."""
    v = float(v)
    return L * v + u * v * v


def allowable_speed(avail_m, L, u):
    """Largest v with d(v) <= avail_m: v = (-L + sqrt(L^2 + 4*u*avail_m)) / (2*u).

    avail_m <= 0 or non-finite means nothing is left in front of the car, so 0.
    """
    avail_m = float(avail_m)
    if not math.isfinite(avail_m) or avail_m <= 0.0:
        return 0.0
    return (-L + math.sqrt(L * L + 4.0 * u * avail_m)) / (2.0 * u)


def lateral_speed_limit(v, omega, a_lat_max):
    """v_lat = sqrt(a_lat_max / kappa) with kappa = |omega| / v, or +inf when the term is off.

    a_lat_max <= 0 (or non-finite) disables the term, and so does a straight command
    (|omega| <= 1e-9) or a non-positive v: with no curvature there is no lateral budget to
    spend. The raw command is the only curvature source here - nothing is estimated from the
    scan - so a command the car cannot actually track is not caught by this limit.
    """
    a_lat_max = float(a_lat_max)
    if not (math.isfinite(a_lat_max) and a_lat_max > 0.0):
        return float('inf')
    v = float(v)
    omega = float(omega)
    # A non-finite command has no meaningful curvature; +inf keeps it out of the min().
    if not (math.isfinite(v) and math.isfinite(omega)):
        return float('inf')
    if v <= 0.0 or abs(omega) <= 1e-9:
        return float('inf')
    return math.sqrt(a_lat_max / (abs(omega) / v))


def steer_from_yaw_rate(v, omega, wheelbase):
    """delta = atan(omega * wheelbase / v); inverse of common/control.py to_cmd_vel.

    to_cmd_vel(v, steer) returns (v, v*tan(steer)/wheelbase), so this recovers the front
    wheel angle the raw command corresponds to. Only called with v > 0.
    """
    return math.atan(float(omega) * wheelbase / float(v))


def _finite(x):
    try:
        return math.isfinite(float(x))
    except (TypeError, ValueError):
        return False


def _gap_valid(gap):
    return 1 if _finite(gap) and not math.isinf(float(gap)) else 0


def _row(t_wall, t_ros, v_raw, w_raw, gap, scan_age_s, v_measured, k, v_out, w_out,
         v_allow, reason, steer=None, a_lat_max=None, v_lat=None, v_allow_total=None):
    """One audit row. Keys beyond CSV_HEADER (steer) are for tests and humans only."""
    return dict(t_wall=t_wall, t_ros=t_ros, v_raw=v_raw, w_raw=w_raw, gap=gap,
                gap_valid=_gap_valid(gap), scan_age_s=scan_age_s, v_measured=v_measured,
                steer=steer, k=k, v_out=v_out, w_out=w_out, v_allow=v_allow,
                passthrough_reason=reason, a_lat_max=a_lat_max, v_lat=v_lat,
                v_allow_total=v_allow_total)


def decide(v_cmd, omega, gap, scan_age_s, v_measured, L, u, reserve_m=0.0,
           scan_stale_s=0.5, publish_epsilon=0.0, a_lat_max=0.0):
    """Pure speed decision for one raw command.

    Returns dict(k, v_out, w_out, v_allow, passthrough_reason) plus the lateral columns
    (a_lat_max, v_lat, v_allow_total). k == 1.0 with a non-empty passthrough_reason means
    "forward the raw command unchanged"; an empty reason means the command was scaled by k.
    Order of the rules matters: reverse first (so a car that is backing out is never
    speed-limited), then anything that makes the gap unusable, then the stop-distance cap.
    The lateral cap is folded into the same k, so v_out and w_out are always scaled by one
    factor and the steering angle is preserved.

    a_lat_max <= 0 leaves v_lat at +inf, so v_allow_total is v_allow and nothing changes.
    """
    v_cmd = float(v_cmd)
    omega = float(omega)
    v_lat = lateral_speed_limit(v_cmd, omega, a_lat_max)
    if not (math.isfinite(v_cmd) and math.isfinite(omega)):
        return dict(k=1.0, v_out=v_cmd, w_out=omega, v_allow=None, a_lat_max=a_lat_max,
                    v_lat=v_lat, v_allow_total=None,
                    passthrough_reason='non_finite_raw_command')
    if v_cmd <= 0.0:
        return dict(k=1.0, v_out=v_cmd, w_out=omega, v_allow=None, a_lat_max=a_lat_max,
                    v_lat=v_lat, v_allow_total=None,
                    passthrough_reason='nonpositive_raw_speed')
    if not _finite(gap) or math.isinf(float(gap)):
        return dict(k=1.0, v_out=v_cmd, w_out=omega, v_allow=None, a_lat_max=a_lat_max,
                    v_lat=v_lat, v_allow_total=None,
                    passthrough_reason='gap_unavailable')
    if not _finite(scan_age_s):
        return dict(k=1.0, v_out=v_cmd, w_out=omega, v_allow=None, a_lat_max=a_lat_max,
                    v_lat=v_lat, v_allow_total=None,
                    passthrough_reason='scan_age_unknown')
    if float(scan_age_s) > scan_stale_s:
        return dict(k=1.0, v_out=v_cmd, w_out=omega, v_allow=None, a_lat_max=a_lat_max,
                    v_lat=v_lat, v_allow_total=None,
                    passthrough_reason='scan_stale')
    avail = max(0.0, float(gap) - float(reserve_m))
    v_allow = allowable_speed(avail, L, u)
    if not math.isfinite(v_allow):
        return dict(k=1.0, v_out=v_cmd, w_out=omega, v_allow=None, a_lat_max=a_lat_max,
                    v_lat=v_lat, v_allow_total=None,
                    passthrough_reason='v_allow_not_finite')
    v_allow_total = min(v_allow, v_lat)
    lateral_binds = v_lat < v_allow
    measured = float(v_measured) if _finite(v_measured) else v_cmd
    denom = max(v_cmd, measured)
    if denom <= 0.0:
        return dict(k=1.0, v_out=v_cmd, w_out=omega, v_allow=v_allow, a_lat_max=a_lat_max,
                    v_lat=v_lat, v_allow_total=v_allow_total,
                    passthrough_reason='denominator_not_positive')
    k = min(1.0, v_allow_total / denom)
    if k >= 1.0:
        return dict(k=1.0, v_out=v_cmd, w_out=omega, v_allow=v_allow, a_lat_max=a_lat_max,
                    v_lat=v_lat, v_allow_total=v_allow_total,
                    passthrough_reason='clearance_sufficient')
    if (1.0 - k) <= publish_epsilon:
        return dict(k=1.0, v_out=v_cmd, w_out=omega, v_allow=v_allow, a_lat_max=a_lat_max,
                    v_lat=v_lat, v_allow_total=v_allow_total,
                    passthrough_reason='within_publish_epsilon')
    return dict(k=k, v_out=k * v_cmd, w_out=k * omega, v_allow=v_allow, a_lat_max=a_lat_max,
                v_lat=v_lat, v_allow_total=v_allow_total,
                passthrough_reason='lateral_limit' if lateral_binds else '')


def publish_values(row):
    """(v, w) safe to put on the wire: anything non-finite or non-numeric becomes 0.0.

    A NaN/inf command cannot be forwarded verbatim, and 0.0 is the fail-safe value.
    """
    def _safe(x):
        try:
            x = float(x)
        except (TypeError, ValueError):
            return 0.0
        return x if math.isfinite(x) else 0.0
    return _safe(row.get('v_out')), _safe(row.get('w_out'))


def format_csv_row(row):
    """One CSV line in CSV_HEADER order; floats keep 6 decimals, missing values are empty.

    Non-finite numbers keep the convention the file already used for gap: +inf prints as
    'inf' (%.6f would too, but the None/inf cases are decided here on purpose).
    """
    def num(x):
        if x is None:
            return ''
        value = float(x)
        if math.isinf(value):
            return 'inf' if value > 0 else '-inf'
        return '%.6f' % value

    gap = row.get('gap')
    if gap is None:
        gap_s = ''
    elif _finite(gap) and math.isinf(float(gap)):
        gap_s = 'inf'
    else:
        gap_s = num(gap)
    reason = str(row.get('passthrough_reason') or '').replace(',', ';')
    fields = [num(row.get('t_wall')), num(row.get('t_ros')), num(row.get('v_raw')),
              num(row.get('w_raw')), gap_s, str(int(row.get('gap_valid') or 0)),
              num(row.get('scan_age_s')), num(row.get('v_measured')), num(row.get('v_allow')),
              num(row.get('k')), num(row.get('v_out')), num(row.get('w_out')), reason,
              num(row.get('a_lat_max')), num(row.get('v_lat')),
              num(row.get('v_allow_total'))]
    return ','.join(fields)


def report_failure(exc, context='runtime failure'):
    """One explicit stderr line per failure; the raw command is still forwarded."""
    print('gazebo_speed_supervisor: %s: %s: %s -> publishing the raw command unchanged'
          % (context, type(exc).__name__, exc), file=sys.stderr, flush=True)


def supervised_command(core, v_cmd, omega, t_ros=None, now=None, on_error=None):
    """Fail-safe wrapper around SupervisorCore.command: never raises, never drops a command.

    On any exception the raw command is returned unchanged with reason 'exception:<Type>'
    and one message is written to stderr (or handed to on_error, which tests use).
    """
    try:
        return core.command(v_cmd, omega, t_ros=t_ros, now=now)
    except BaseException as exc:                                  # noqa: BLE001
        (on_error or report_failure)(exc)
        return _row(now, t_ros, v_cmd, omega, None, None, core.v_measured, 1.0, v_cmd, omega,
                    None, 'exception:' + type(exc).__name__)


class SupervisorCore:
    """All supervisor behaviour without ROS types: guard state, ages, one decision row."""

    def __init__(self, guard, L=0.053618, u=0.103084, reserve_m=0.0, scan_stale_s=0.5,
                 publish_epsilon=0.0, wheelbase=WHEELBASE, a_lat_max=0.0, now_fn=time.monotonic):
        self.guard = guard                      # None = guard unavailable -> passthrough
        self.L = float(L)
        self.u = float(u)
        self.reserve_m = float(reserve_m)
        self.scan_stale_s = float(scan_stale_s)
        self.publish_epsilon = float(publish_epsilon)
        self.wheelbase = float(wheelbase)
        self.a_lat_max = float(a_lat_max)       # <= 0 disables the lateral cap
        self.now_fn = now_fn
        self.v_measured = None                  # last odom longitudinal speed [m/s]
        self.scan_wall = None                   # monotonic receipt time of the last scan
        self.scan_count = 0
        self.odom_count = 0

    def update_scan(self, msg, now=None):
        """Feed one LaserScan to the production guard and remember its receipt time."""
        now = self.now_fn() if now is None else now
        self.guard.update(msg)
        self.scan_wall = now
        self.scan_count += 1

    def update_odom(self, msg, now=None):
        """Remember the measured longitudinal speed (same field road_policy_node.on_enc uses)."""
        self.v_measured = float(msg.twist.twist.linear.x)
        self.odom_count += 1

    def scan_age(self, now=None):
        """Seconds since the last scan was received, or None if none ever arrived."""
        if self.scan_wall is None:
            return None
        now = self.now_fn() if now is None else now
        return max(0.0, float(now) - self.scan_wall)

    def gap_for(self, steer):
        """Production guard clearance for this steering angle (None when it has no data)."""
        return self.guard.gap(steer)

    def command(self, v_cmd, omega, t_ros=None, now=None):
        """Audit row for one raw command; may raise, so call it through supervised_command."""
        now = self.now_fn() if now is None else now
        age = self.scan_age(now)
        # The lateral term is a property of the raw command alone, so it is known even when the
        # guard is missing; the row records it either way.
        v_lat = lateral_speed_limit(v_cmd, omega, self.a_lat_max)
        if self.guard is None:
            return _row(now, t_ros, v_cmd, omega, None, age, self.v_measured, 1.0, v_cmd,
                        omega, None, 'guard_unavailable', a_lat_max=self.a_lat_max,
                        v_lat=v_lat, v_allow_total=None)
        steer = None
        gap = None
        if _finite(v_cmd) and float(v_cmd) > 0.0 and _finite(omega):
            steer = steer_from_yaw_rate(float(v_cmd), float(omega), self.wheelbase)
            gap = self.gap_for(steer)
        out = decide(v_cmd, omega, gap, age, self.v_measured, self.L, self.u, self.reserve_m,
                     self.scan_stale_s, self.publish_epsilon, self.a_lat_max)
        return _row(now, t_ros, float(v_cmd), float(omega), gap, age, self.v_measured,
                    out['k'], out['v_out'], out['w_out'], out['v_allow'],
                    out['passthrough_reason'], steer=steer, a_lat_max=self.a_lat_max,
                    v_lat=out['v_lat'], v_allow_total=out['v_allow_total'])


def make_node_class(Node, Twist, LaserScan, Odometry, a_lat_max=None):
    """Build the ROS2 node class (imports stay in main, so this module loads without ROS).

    a_lat_max comes from our own --a-lat-max flag; None leaves the ROS parameter default
    (0.0, i.e. off) in charge, so -p a_lat_max:=... keeps working on its own.
    """

    class SpeedSupervisorNode(Node):
        def __init__(self):
            super().__init__('gazebo_speed_supervisor')
            for name, default in DEFAULT_PARAMS.items():
                self.declare_parameter(name, default)
            p = {name: self.get_parameter(name).value for name in DEFAULT_PARAMS}
            if a_lat_max is not None:
                p['a_lat_max'] = float(a_lat_max)     # the flag beats the ROS parameter
            self.csv = None
            self.csv_errors = 0
            path = str(p['debug_csv'] or '')
            if path:
                self.csv = open(path, 'a', buffering=1)
                if self.csv.tell() == 0:
                    self.csv.write(CSV_HEADER + '\n')
            guard = None
            guard_dir = os.environ.get('SCAN_GUARD_DIR') or SCAN_GUARD_DIR
            load = guard_load_report(guard_dir)
            try:
                guard = load_scan_guard(guard_dir)()
            except BaseException as exc:                          # noqa: BLE001
                report_failure(exc, 'scan_guard import failed')
            self.guard_ok = guard is not None
            self.core = SupervisorCore(guard, L=p['L'], u=p['u'], reserve_m=p['reserve_m'],
                                       scan_stale_s=p['scan_stale_s'],
                                       publish_epsilon=p['publish_epsilon'],
                                       wheelbase=p['wheelbase'],
                                       a_lat_max=p['a_lat_max'])
            self.pub = self.create_publisher(Twist, str(p['out_topic']), 10)
            self.create_subscription(Twist, str(p['raw_topic']), self.on_raw, 10)
            self.create_subscription(LaserScan, str(p['scan_topic']), self.on_scan, 1)
            self.create_subscription(Odometry, str(p['odom_topic']), self.on_odom, 10)
            self.get_logger().info(
                'speed supervisor: %s -> %s, scan %s, odom %s, L=%.6f u=%.6f reserve=%.3f '
                'stale=%.2fs wheelbase=%.3f a_lat_max=%.3f%s guard=%s csv=%s'
                % (p['raw_topic'], p['out_topic'], p['scan_topic'], p['odom_topic'], p['L'],
                   p['u'], p['reserve_m'], p['scan_stale_s'], p['wheelbase'], p['a_lat_max'],
                   ' (lateral cap off)' if not float(p['a_lat_max']) > 0.0 else '',
                   'loaded' if guard is not None else 'UNAVAILABLE (passthrough)', path or '-'))
            self.get_logger().info(load['report'])

        def now_s(self):
            try:
                return self.get_clock().now().nanoseconds * 1e-9
            except BaseException:                                 # noqa: BLE001
                return None

        def on_scan(self, msg):
            try:
                self.core.update_scan(msg)
            except BaseException as exc:                          # noqa: BLE001
                report_failure(exc, 'scan update failed')

        def on_odom(self, msg):
            try:
                self.core.update_odom(msg)
            except BaseException as exc:                          # noqa: BLE001
                report_failure(exc, 'odom update failed')

        def on_raw(self, msg):
            row = supervised_command(self.core, msg.linear.x, msg.angular.z,
                                     t_ros=self.now_s())
            v_out, w_out = publish_values(row)
            tw = Twist()
            tw.linear.x = v_out
            tw.angular.z = w_out
            self.pub.publish(tw)          # publish first: the CSV must never block a command
            self.write_csv(row)

        def write_csv(self, row):
            if self.csv is None:
                return
            try:
                self.csv.write(format_csv_row(row) + '\n')
            except BaseException as exc:                          # noqa: BLE001
                self.csv_errors += 1
                report_failure(exc, 'debug csv write failed')

        def close(self):
            if self.csv is not None:
                try:
                    self.csv.close()
                except BaseException:                             # noqa: BLE001
                    pass
                self.csv = None

    return SpeedSupervisorNode


def parse_a_lat_max(argv):
    """Pull our own --a-lat-max out of argv -> (value or None, argv without it).

    Everything else (--ros-args, -p, -r, ...) is handed to rclpy untouched, so ROS parameters
    keep working exactly as before. Accepts both '--a-lat-max 0.5' and '--a-lat-max=0.5'.
    """
    value = None
    rest = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == CLI_A_LAT_MAX or a.startswith(CLI_A_LAT_MAX + '='):
            if a == CLI_A_LAT_MAX:
                if i + 1 >= len(argv):
                    sys.exit('%s needs a value in m/s^2 (0 disables the lateral cap)'
                             % CLI_A_LAT_MAX)
                raw = argv[i + 1]
                i += 2
            else:
                raw = a.split('=', 1)[1]
                i += 1
            try:
                value = float(raw)
            except ValueError:
                sys.exit('%s needs a float, got %r' % (CLI_A_LAT_MAX, raw))
            continue
        rest.append(a)
        i += 1
    return value, rest


def rclpy_argv(argv):
    """(a_lat_max, argv for rclpy.init) -- keeps the program name in slot 0.

    rclpy's parser starts at argv[1] and treats argv[0] as the program name, so the slot has to
    stay in front: hand it sys.argv[1:] and '--ros-args' gets swallowed as the program name,
    silently dropping every -p parameter. argv=None means "read sys.argv", as before.
    """
    argv = list(sys.argv) if argv is None else list(argv)
    prog = argv[0] if argv else sys.argv[0]
    a_lat_max, rest = parse_a_lat_max(argv[1:])
    return a_lat_max, [prog] + rest


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    a_lat_max, ros_argv = rclpy_argv(argv)
    rclpy.init(args=ros_argv)      # -p/-r/--ros-args reach rclpy; --a-lat-max was stripped
    node = make_node_class(Node, Twist, LaserScan, Odometry, a_lat_max=a_lat_max)()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
