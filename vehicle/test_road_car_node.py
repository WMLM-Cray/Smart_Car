"""road_v2 实车节点的纯逻辑测试(不需要 ROS):python -m pytest vehicle/test_road_car_node.py"""
import math

import numpy as np
import pytest

from common.params import CarParams
from common.pwm import action_to_steer, steer_to_turn
from common.road_policy import Encoding, RoadPolicy
from simulation.fastsim.road import LEVELS, generate
from simulation.fastsim.road_env import Plant, RoadEnv
from vehicle import road_car_node
from vehicle.road_car_node import AVOID_CLEAR, GUARD_STOP, LAUNCH_FROM, STUCK_SECONDS, RoadCarController, guard_speed, guard_stop
from vehicle.scan_guard import BUMPER_AHEAD, LIDAR_X
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
    ctl.set_speed_cap(pl.cap)       # 开局是 Sloth 档(0.7);网页切档后才到 pl.cap
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
    assert pwm == LAUNCH_FROM                         # 第一拍就是跳点,之后再爬
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
    wall[np.abs(ang) < 1.2] = BUMPER_AHEAD + 0.10   # 正前方一整面墙,保险杠前约 0.10m:往哪打都绕不开
    ctl = ready(FakePolicy(action=(0.0, 1.0)), ranges=wall)
    pwm, servo = ctl.tick(10.05)
    assert ctl.event == 'guard' and ctl.gap < GUARD_STOP
    assert pwm == P.pwm_mid and servo == steer_to_turn(ctl.steer_cmd, P)
    t = 10.05
    while t < 10.05 + STUCK_SECONDS + 0.2:
        t += 0.05
        ctl.on_scan(FakeScan(wall), t)
        feed(ctl, t)
        assert ctl.tick(t)[0] == P.pwm_mid
    assert ctl.event == 'stuck'


def _box(ranges, x0, x1, y0, y1):
    """后轴系里 x∈[x0,x1]、y∈[y0,y1] 的一块障碍(雷达看到它的近边)。"""
    ang = -math.pi + np.arange(N) * INC
    for x in np.arange(x0, x1, 0.01):
        for y in np.arange(y0, y1, 0.01):
            dx = x - LIDAR_X
            a = math.atan2(y, dx)
            k = int(round((a + math.pi) / INC)) % N
            ranges[k] = min(ranges[k], math.hypot(dx, y))
    return ranges


def test_avoid_steers_around_an_obstacle_ahead_instead_of_driving_into_it():
    """正前方 0.8 m 一块 0.4 m 宽的板,模型要直走:避障换一个走得开的转角,不截停。"""
    r = _box(np.full(N, 5.0), 1.2, 1.25, -0.2, 0.2)
    ctl = ready(FakePolicy(action=(0.0, 1.0)), ranges=r)
    pwm, servo = ctl.tick(10.05)
    assert ctl.event == 'avoid' and abs(ctl.steer_cmd) > 0.05
    assert ctl.guard.gap(ctl.steer_cmd) >= AVOID_CLEAR and pwm > P.pwm_mid
    side = np.sign(ctl.steer_cmd)
    for k in range(5):                                           # 接着几拍还往同一边绕,不左右来回切
        t = 10.1 + 0.05 * k
        ctl.on_scan(FakeScan(r), t)
        feed(ctl, t)
        ctl.tick(t)
        assert np.sign(ctl.steer_cmd) == side


def test_avoid_leaves_the_model_alone_when_the_path_is_clear():
    r = _box(np.full(N, 5.0), 1.2, 1.25, 0.5, 0.9)              # 板在左前方,不挡直行
    ctl = ready(FakePolicy(action=(0.0, 1.0)), ranges=r)
    ctl.tick(10.05)
    assert ctl.event is None and ctl.steer_cmd == action_to_steer(0.0, P)


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


def test_launch_jump_stays_below_measured_breakaway():
    """起步跳点紧贴高位实测最低破静点 1594 下方;不能跳过它。
    低位(上电锁成 ~1500)时破静 1552,这个跳点会冲出去 —— 靠"先开服务再开电调"的规程保证高位。"""
    assert P.pwm_mid < LAUNCH_FROM < 1594


class DeadPolicy(FakePolicy):
    """带停车 / 刹车死区的新契约(v4.5)。"""
    enc = Encoding(5.0, 0.45, 5.0, 0.2, True)     # _s5_v0.45-5_d0.2_log


def drive(ctl, t, speed):
    ctl.on_scan(FakeScan(np.full(N, 5.0)), t)
    feed(ctl, t, speed)
    return ctl.tick(t)


def test_dead_band_contract_stop_is_neutral():
    ctl = ready(DeadPolicy((0.0, 0.0)))
    assert drive(ctl, 10.05, 1.0)[0] == P.pwm_mid


def test_dead_band_contract_brakes_only_above_release_speed():
    """刹车:车速高于 brake_release_speed 发 brake_pwm;低了改中位,不倒车(同仿真 release_brake)。"""
    ctl = ready(DeadPolicy((0.0, -1.0)))
    assert drive(ctl, 10.05, P.brake_release_speed + 0.3)[0] == P.brake_pwm
    assert drive(ctl, 10.10, P.brake_release_speed)[0] == P.pwm_mid


def test_dead_band_contract_forward_after_brake_relaunches_by_jump():
    """刹停后再给油:速度环清过状态,第一拍是起步跳点,不从刹车 PWM 往上爬。"""
    pol = DeadPolicy((0.0, -1.0))
    ctl = ready(pol)
    drive(ctl, 10.05, 1.0)
    pol.action = np.asarray((0.0, 1.0), np.float32)
    assert drive(ctl, 10.10, 0.0)[0] == LAUNCH_FROM


def test_guard_brakes_when_fast_and_sends_neutral_when_slow():
    """截停:车速高于 brake_release_speed 发刹车(更快的模式要刹得住),低了回中位(防倒车)。"""
    wall = np.full(N, 5.0)
    ang = -math.pi + np.arange(N) * INC
    wall[np.abs(ang) < 1.2] = BUMPER_AHEAD + 0.10
    ctl = ready(DeadPolicy((0.0, -1.0)), ranges=wall)
    feed(ctl, 10.05, 1.0)
    assert ctl.tick(10.05)[0] == P.brake_pwm and ctl.event == 'guard'
    feed(ctl, 10.10, P.brake_release_speed)
    assert ctl.tick(10.10)[0] == P.pwm_mid and ctl.event == 'guard'


def test_guard_stop_unchanged_up_to_base_speed_and_grows_with_speed():
    """0.7 m/s 及以下还是 0.25 m(实车验证过的);1.2 / 2.5 m/s 加上多出来的延迟 + 刹车距离。"""
    for v in (0.0, 0.5, P.guard_base_speed):
        assert guard_stop(v, P) == GUARD_STOP
    assert guard_stop(1.2, P) == pytest.approx(GUARD_STOP + 0.2 * 0.5 + (1.44 - 0.49) / 2.6)
    assert guard_stop(2.5, P) > 2.5


def test_guard_trips_earlier_at_higher_speed():
    """同一面墙在 0.6 m 外:0.7 m/s 不截停,1.2 m/s 截停并刹车。"""
    wall = np.full(N, 5.0)
    ang = -math.pi + np.arange(N) * INC
    wall[np.abs(ang) < 1.2] = BUMPER_AHEAD + 0.60
    ctl = ready(FakePolicy(action=(0.0, 1.0)), ranges=wall)
    feed(ctl, 10.05, 0.7)
    ctl.tick(10.05)
    assert ctl.event not in ('guard', 'stuck')
    feed(ctl, 10.10, 1.2)
    assert ctl.tick(10.10)[0] == P.brake_pwm and ctl.event == 'guard'


def test_speed_cap_starts_at_sloth_and_follows_web_mode_within_max():
    """开局 Sloth(0.7);网页切档改限速,但不超过 --speed-cap,也不低于模型训练下限。"""
    ctl = RoadCarController(DeadPolicy(), 2.5)
    assert ctl.rt.speed_cap == P.policy_mode_caps[0]
    assert ctl.set_speed_cap(1.2) == 1.2
    assert ctl.set_speed_cap(2.5) == 2.5
    assert RoadCarController(DeadPolicy(), 1.2).set_speed_cap(2.5) == 1.2
    assert ctl.set_speed_cap(0.0) == DeadPolicy.enc.v_lo
    assert RoadCarController(DeadPolicy(), 0.5).rt.speed_cap == 0.5


def test_guard_speed_inverts_guard_stop():
    p = CarParams()
    for v in (0.8, 1.2, 1.8, 2.5):
        assert guard_speed(guard_stop(v, p), p) == pytest.approx(v)
    assert guard_speed(GUARD_STOP - 0.01, p) == 0.0 and guard_speed(GUARD_STOP, p) == pytest.approx(p.guard_base_speed)
