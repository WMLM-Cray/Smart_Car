"""网页手动防撞(vehicle/server.py compute):按当前转角扫掠净空;不够刹停距离(随车速)就压到最慢档减速,贴到 0.25 m 才回中位。
server 顶层 import 了 rclpy / ROS 消息,这里用空模块顶上,只测 compute。"""
import sys
import types

import pytest


def _stub_ros():
    for name in ('rclpy', 'rclpy.node', 'rclpy.signals', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
                 'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg'):
        if name not in sys.modules:
            mod = types.ModuleType(name)
            for attr in ('Node', 'SignalHandlerOptions', 'Twist', 'Odometry', 'Imu', 'LaserScan', 'Bool', 'Float32'):
                setattr(mod, attr, object)
            sys.modules[name] = mod


_stub_ros()
import vehicle.server as srv  # noqa: E402
from common.params import CarParams  # noqa: E402
from vehicle.scan_guard import guard_stop  # noqa: E402


class FakeGuard:
    def __init__(self, gap):
        self.g, self.steers = gap, []

    def gap(self, steer):
        self.steers.append(steer)
        return self.g


@pytest.fixture
def S(monkeypatch):
    st = srv.State()
    monkeypatch.setattr(srv, 'S', st)
    st.last_t = st.enc_t = 100.0
    st.mode, st.thr, st.steer = 2, 1000, 0
    st.dir_change_t = st.fwd_t = -10.0
    st.last_dir = 1
    return st


def test_fast_and_close_slows_down_without_braking(S):
    S.spd, S.sg = 3.0, FakeGuard(2.0)               # 3 m/s 刹停要 ~3.5 m,前方只有 2 m:压到最慢档,不刹车
    assert guard_stop(3.0, CarParams()) > 2.0
    pwm, _, active = srv.compute(100.0)
    assert active and srv.SPEED_MID < pwm <= srv.FWD_CAP[0] and S.blocked == 0


def test_touching_goes_neutral(S):
    S.spd, S.sg = 0.2, FakeGuard(0.1)
    pwm, _, _ = srv.compute(100.0)
    assert pwm == srv.SPEED_MID and S.blocked == 1


def test_same_gap_is_full_gear_when_slow(S):
    S.spd, S.sg = 0.5, FakeGuard(2.0)               # 慢的时候 2 m 足够,不压档
    S.fwd_pwm, S.fwd_t = srv.FWD_CAP[2], 99.99
    pwm, _, _ = srv.compute(100.0)
    assert pwm > srv.FWD_CAP[0] and S.blocked == 0


def test_clearance_follows_current_steering(S):
    S.spd, S.steer, S.sg = 1.0, 800, FakeGuard(9.0)
    srv.compute(100.0)
    turn = round(srv.TURN_MID + 0.8 * srv.TURN_SPAN)
    assert S.sg.steers == [pytest.approx(srv.turn_to_steer(turn))] and abs(S.sg.steers[0]) > 0.1


def test_no_scan_still_blocks(S):
    S.spd, S.sg = 1.0, FakeGuard(None)
    pwm, _, _ = srv.compute(100.0)
    assert S.blocked == 1 and pwm == srv.SPEED_MID
