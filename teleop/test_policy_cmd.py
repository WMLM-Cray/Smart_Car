"""策略下发口径:python3 -m pytest teleop/test_policy_cmd.py

只测 to_cmd —— 实车和仿真的差别全在这一个函数里,它错了车就按 PWM 的数量级
去理解 m/s(或反过来),现象是"一使能就全速冲出去"。不需要 ROS。
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fastsim.interop import from_cmd_vel, to_cmd_vel
from fastsim.params import CarParams
from fastsim.pwm import pwm_to_speed, turn_to_steer

P = CarParams()


PWM_CAP = 1610


def to_cmd(v, steer, sim, p, pwm_cap=PWM_CAP):
    """policy_node.to_cmd 的副本 —— 原文件 import rclpy,PC 上装不了。

    两边必须一致,由 test_matches_the_node_source 保证。
    """
    from fastsim.pwm import speed_to_pwm, steer_to_turn
    if sim:
        return to_cmd_vel(v, steer, p)
    return float(min(speed_to_pwm(v, p), pwm_cap)), float(steer_to_turn(steer, p))


def test_pwm_cap_bounds_the_car_even_at_full_command():
    """v_fwd_full 还没实测,限幅是唯一保证最坏情况也慢的东西。"""
    assert to_cmd(1.5, 0.0, False, P)[0] == PWM_CAP          # 策略满速也顶在限幅
    assert to_cmd(99.0, 0.0, False, P)[0] == PWM_CAP         # 再离谱也不越界
    assert to_cmd(0.0, 0.0, False, P)[0] == P.pwm_mid        # 停车不受限幅影响
    assert to_cmd(-0.5, 0.0, False, P)[0] < P.pwm_mid        # 限幅不该挡住倒车


def test_pwm_cap_does_not_touch_sim():
    """仿真走物理量,限幅是 PWM 刻度上的事,不能漏到那边去。"""
    assert to_cmd(1.5, 0.0, True, P)[0] == pytest.approx(1.5)


def test_sim_sends_yaw_rate_not_steer_angle():
    """racecar_sim 的 /cmd_vel angular.z 是 rad/s。发成转角车会原地乱转。"""
    v, steer = 1.5, 0.3
    lin, ang = to_cmd(v, steer, True, P)
    assert lin == pytest.approx(v)
    assert ang == pytest.approx(v * math.tan(steer) / P.wheelbase)
    assert ang != pytest.approx(steer)                 # 不是把转角原样发出去
    assert to_cmd(0.0, 0.0, True, P) == (0.0, 0.0)


@pytest.mark.parametrize('v,steer', [(1.5, 0.55), (0.8, -0.2), (0.3, 0.0)])
def test_sim_conversion_round_trips(v, steer):
    lin, ang = to_cmd(v, steer, True, P)
    assert from_cmd_vel(lin, ang, P) == pytest.approx((v, steer), abs=1e-9)


def test_car_converts_to_pwm_and_servo_scale():
    # 限幅放开,这里测的是换算本身;限幅另有 test_pwm_cap_bounds_the_car_even_at_full_command
    lin, ang = to_cmd(1.5, 0.55, False, P, pwm_cap=P.pwm_fwd_full)
    assert lin == pytest.approx(1800, abs=1)        # 1.5/3.0 -> 1600 + 半程
    assert ang == P.turn_mid + P.turn_span          # 左满舵
    assert to_cmd(0.0, 0.0, False, P) == (P.pwm_mid, P.turn_mid)


@pytest.mark.parametrize('v,steer', [(0.0, 0.0), (1.5, 0.55), (0.8, -0.2), (-0.5, 0.1)])
def test_car_conversion_round_trips(v, steer):
    """换算回来要还原 —— 死区外的值不能在这一层丢掉。"""
    lin, ang = to_cmd(v, steer, False, P, pwm_cap=P.pwm_fwd_full)
    assert pwm_to_speed(lin, P) == pytest.approx(v, abs=0.01)
    assert turn_to_steer(ang, P) == pytest.approx(steer, abs=0.01)


def test_matches_the_node_source():
    """防止改了节点忘了改这里的副本。"""
    src = (Path(__file__).resolve().parent / 'policy_node.py').read_text(encoding='utf-8')
    body = src.split('def to_cmd(')[1].split('\nclass ')[0]
    for line in ('if sim:', 'return to_cmd_vel(v, steer, p)',
                 'return float(min(speed_to_pwm(v, p), pwm_cap)), float(steer_to_turn(steer, p))'):
        assert line in body, f'policy_node.to_cmd 变了,同步这个副本:{line}'


def test_sim_lidar_convention_still_sees_the_right_front():
    """Gazebo 雷达按 0..2π 发角度,防撞不折回去就只看得见左前方半边。"""
    from scan_guard import BUMPER_AHEAD, ScanGuard

    n, ahead = 360, 1.0

    class Scan:
        range_min, range_max = 0.24, 25.0
        angle_min, angle_increment = 0.0, 2 * math.pi / n

        def __init__(self, at):
            self.ranges = [ahead / math.cos(at) if abs(i * self.angle_increment
                           - at) < self.angle_increment else float('inf')
                           for i in range(n)]

    for at in (0.2, 2 * math.pi - 0.2):             # 左前 11°、右前 11°(横向仍在走廊内)
        g = ScanGuard()
        g.update(Scan(at))
        assert g.gap() == pytest.approx(ahead - BUMPER_AHEAD, abs=0.05), f'{at} rad 没看见'
