"""road_v2 实车节点的纯逻辑测试(不需要 ROS):python -m pytest vehicle/test_road_car_node.py"""
import math

import numpy as np
import pytest

from common.params import CarParams
from common.pwm import action_to_steer, steer_to_turn
from common.road_policy import RoadPolicy
from simulation.fastsim.road import LEVELS, generate
from simulation.fastsim.road_env import Plant, RoadEnv
from vehicle import road_car_node
from vehicle.road_car_node import GUARD_STOP, LAUNCH_FROM, STUCK_SECONDS, RoadCarController
from vehicle.scan_guard import BUMPER_AHEAD
from vehicle.speed_loop import PWM_HARD_MAX

MODEL = 'models/road_v2/candidate_167.npz'
P = CarParams()
N = 1667
INC = 2 * math.pi / N


class FakeScan:
    """LaserScan 形状:和实车/仿真一样从 -π 开始逆时针。"""
    angle_min, angle_increment = -math.pi, INC
    range_min, range_max = 0.15, 25.0

    def __init__(self, ranges):
        self.ranges = list(ranges)


class FakePolicy:
    def __init__(self, action=(0.5, 1.0)):
        self.action = np.asarray(action, np.float32)

    def reset(self):
        pass

    def __call__(self, obs):
        return self.action.copy()


def ready(policy=None, cap=0.6, ranges=None, t=10.0):
    ctl = RoadCarController(policy or FakePolicy(), cap)
    ctl.on_scan(FakeScan(np.full(N, 5.0) if ranges is None else ranges), t)
    ctl.on_encoder(0.0, t)
    ctl.on_imu(0.0, t)
    ctl.on_enable(True, t)
    return ctl


def feed(ctl, t, speed=0.0):
    ctl.on_encoder(speed, t)
    ctl.on_imu(0.0, t)
    ctl.on_enable(True, t)


def test_same_actions_as_training_env_on_a_fastsim_episode():
    """同一局 FastSim:训练环境 env.observation() 喂模型 vs 实车控制器吃原始传感器,动作逐拍一致。"""
    rng = np.random.default_rng(3)
    road = generate(rng, next(l for l in LEVELS if l.name == 'L3'))
    pl = Plant.sample(rng)
    env = RoadEnv(road, pl, seed=3, noise=False)
    ref = RoadPolicy(MODEL)
    ctl = RoadCarController(RoadPolicy(MODEL), pl.cap)
    ctl.on_enable(True, 0.0)
    for _ in range(120):
        t = env.t
        ctl.on_scan(FakeScan(env.ranges), t)
        ctl.on_encoder(env.car.v, t)
        ctl.on_imu(env.car.yaw_rate, t)
        ctl.on_enable(True, t)
        want = ref(env.observation())
        cmd = ctl.tick(t)
        assert cmd is not None
        np.testing.assert_allclose(ctl.rt.last_action, want, atol=1e-6)
        assert cmd[1] == steer_to_turn(action_to_steer(float(want[0]), P), P)
        if env.step(want) is not None:
            break


def test_silent_until_enabled():
    ctl = RoadCarController(FakePolicy(), 0.6)
    ctl.on_scan(FakeScan(np.full(N, 5.0)), 10.0)
    assert ctl.tick(10.0) is None


def test_output_is_pwm_and_servo_units_starting_from_neutral():
    ctl = ready()
    pwm, servo = ctl.tick(10.05)
    assert LAUNCH_FROM <= pwm <= LAUNCH_FROM + 5       # 从中位爬,不跳变
    assert pwm < ctl.loop.cfg.pwm_breakaway
    assert servo == steer_to_turn(action_to_steer(0.5, P), P)


def test_pwm_never_exceeds_hard_max_even_if_wheels_never_turn():
    """架空台编码器恒 0 的最坏情况:只能爬到破静点 + 余量,不能顶到上限。"""
    ctl = ready(cap=1.0)
    t, top = 10.0, 0
    for _ in range(1200):                            # 60 秒
        t += 0.05
        ctl.on_scan(FakeScan(np.full(N, 5.0)), t)
        feed(ctl, t, speed=0.0)
        top = max(top, ctl.tick(t)[0])
    assert top <= PWM_HARD_MAX
    assert top <= ctl.loop.cfg.pwm_breakaway + ctl.loop.cfg.start_margin


def test_release_sends_neutral_immediately_then_silence():
    ctl = ready()
    ctl.tick(10.05)
    assert ctl.on_enable(False, 10.1) is True
    assert ctl.neutral() == (P.pwm_mid, steer_to_turn(0.0, P))
    assert ctl.tick(10.15) is None


def test_heartbeat_timeout_sends_neutral_once_then_silence():
    ctl = ready()
    ctl.on_scan(FakeScan(np.full(N, 5.0)), 10.4)
    ctl.on_encoder(0.0, 10.4)
    ctl.on_imu(0.0, 10.4)
    assert ctl.tick(10.4) == (P.pwm_mid, steer_to_turn(0.0, P))
    assert ctl.event == 'stale'
    assert ctl.tick(10.45) is None


@pytest.mark.parametrize('stale', ['scan', 'odom', 'imu'])
def test_any_stale_sensor_sends_neutral(stale):
    ctl = ready()
    t = 10.4
    ctl.on_enable(True, t)
    if stale != 'scan':
        ctl.on_scan(FakeScan(np.full(N, 5.0)), t)
    if stale != 'odom':
        ctl.on_encoder(0.0, t)
    if stale != 'imu':
        ctl.on_imu(0.0, t)
    assert ctl.tick(t) == (P.pwm_mid, steer_to_turn(0.0, P))
    assert ctl.rt.stale_sensors(t) == [stale]


def test_guard_stops_throttle_keeps_steering_and_reports_stuck():
    wall = np.full(N, 5.0)
    ang = -math.pi + np.arange(N) * INC
    wall[np.abs(ang) < 0.3] = BUMPER_AHEAD + 0.10   # 保险杠前约 0.10m
    ctl = ready(FakePolicy(action=(0.0, 1.0)), ranges=wall)
    pwm, servo = ctl.tick(10.05)
    assert ctl.event == 'guard' and ctl.gap < GUARD_STOP
    assert pwm == P.pwm_mid and servo == steer_to_turn(action_to_steer(0.0, P), P)
    t = 10.05
    while t < 10.05 + STUCK_SECONDS + 0.2:
        t += 0.05
        ctl.on_scan(FakeScan(wall), t)
        feed(ctl, t)
        assert ctl.tick(t)[0] == P.pwm_mid
    assert ctl.event == 'stuck'


@pytest.mark.parametrize('cap', ['0.3', '1.2'])
def test_speed_cap_must_be_in_training_range(cap):
    with pytest.raises(SystemExit, match='训练范围'):
        road_car_node.main([MODEL, '--speed-cap', cap])


@pytest.mark.parametrize('cap', ['0.3', '3.5'])
def test_speed_cap_range_follows_the_model_contract(cap):
    """road_v2.5(b036)的训练范围是 0.45~3.0:1.2 合法,0.3 / 3.5 不合法。"""
    with pytest.raises(SystemExit, match='训练范围'):
        road_car_node.main(['models/road_v2.5/candidate_b036.npz', '--speed-cap', cap])


def test_speed_cap_is_required():
    with pytest.raises(SystemExit):
        road_car_node.main([MODEL])


def test_launch_starts_from_neutral():
    """死区会漂 40 个 PWM(2026-09-29 下午破静 1552),起步不能跳到任何写死的"死区内"点。"""
    assert LAUNCH_FROM == P.pwm_mid
