import math

import numpy as np
import pytest

from common.params import CarParams
from common.pwm import action_to_steer
from simulation.fastsim.road import CONE, LEVELS, WALL, build, generate, self_overlaps
from training.road_check import check
from simulation.fastsim.road_env import BRAKE, BRAKE_A0, BRAKE_K, STOP, Plant, RoadEnv, SpeedPlant, footprint_clearance
from common.road_policy import DIM, FRAMES, V_HI, V_LO, action_to_speed, speed_to_action
from training.road_teacher import R_MIN, Teacher, plan


def corridor(width, r_in=0.0, angle=90.0, mat=(WALL, WALL)):
    elems = [dict(kind='straight', length=3.0, width=width),
             dict(kind='arc', angle=angle, r_in=r_in, width=width),
             dict(kind='straight', length=4.0, width=width)]
    for e in elems:
        e['mat'] = mat
    return build(elems)


def test_arc_geometry_ends_where_it_should():
    r = corridor(1.2, r_in=0.5)
    rc = 0.5 + 0.6
    assert r.centre[-1] == pytest.approx([3.0 + rc, rc + 4.0], abs=1e-6)
    assert r.heading[-1] == pytest.approx(math.pi / 2)
    assert not self_overlaps(r)


def test_generated_roads_never_overlap_themselves():
    for lv in LEVELS:
        rng = np.random.default_rng(0)
        for _ in range(10):
            assert not self_overlaps(generate(rng, lv))


def test_sharp_corner_in_a_wide_corridor_is_planned_within_the_turning_radius():
    p = plan(corridor(1.4), 0, 0.0, CarParams().width, Plant())
    assert not isinstance(p, str)
    assert np.max(np.abs(p.kappa)) <= 1 / R_MIN
    assert np.all(np.abs(p.d) <= p.bound + 1e-3)


def test_the_planning_gate_is_injectable():
    """`plan(..., r_min=)` 与 `check(..., r_min=)` 必须真的改变规划结果。

    踩过的坑:`plan` 的默认参数在 **def 时**就绑定了 `R_MIN`,所以
    `road_teacher.R_MIN = x` 这种 monkeypatch 对已定义好的函数一点用没有 ——
    一个"0.95 vs 0.85"的测量跑完全程其实两臂都是 0.85,数字还逐位相同。
    要换闸只能走参数。
    """
    road = corridor(1.4)
    tight = plan(road, 0, 0.0, CarParams().width, Plant(), r_min=0.95)
    loose = plan(road, 0, 0.0, CarParams().width, Plant(), r_min=0.85)
    assert not isinstance(tight, str) and not isinstance(loose, str)
    assert np.max(np.abs(tight.kappa)) <= 1 / 0.95 + 1e-9
    assert np.max(np.abs(loose.kappa)) > np.max(np.abs(tight.kappa)), \
        '放宽闸必须解出更急的路线,否则参数没接上'


def test_too_narrow_sharp_corner_is_rejected():
    # 0.6m 宽的尖直角:扣掉车宽和余量已经没有横向空间
    assert plan(corridor(0.6), 0, 0.0, CarParams().width, Plant()) == 'too_narrow'


def test_teacher_drives_a_sharp_corridor_corner_at_top_speed_with_a_slow_servo():
    road = corridor(1.2)
    pl = Plant(steer_rate=3.5, lag=1, cap=1.0, v_top=1.1, v_plateau=0.7, launch_delay=0.5)
    ok, why, _ = check(road, pl, (0, 0.0, 0.0))
    assert ok, why


def test_speed_plant_floors_at_plateau_and_only_coasts_down():
    pl = Plant(v_plateau=0.6, v_top=1.0, launch_delay=0.5, accel_up=0.6, decel=0.73)
    sp = SpeedPlant(pl)
    v = [sp.step(0.45) for _ in range(300)]
    assert v[40] == 0.0                         # 起步前在死区里
    assert v[-1] == pytest.approx(0.6, abs=0.01)   # 要 0.45 也只能停在平台
    for _ in range(300):
        sp.step(1.0)
    top = sp.v
    v = [sp.step(0.45) for _ in range(200)]
    drops = -np.diff(v)
    assert top == pytest.approx(1.0, abs=0.01)
    assert drops.max() <= 0.73 * 0.01 + 1e-9    # 不刹车,只滑



def _run_to(sp, v0, dt=0.01):
    """前进指令把车速推到 v0(平台以上按 accel_up 爬),返回所用拍数。"""
    n = 0
    while sp.v < v0 - 1e-6:
        sp.step(v0)
        n += 1
    return n


def test_understeer_matches_1024_imu():
    """runs/drive_1024c:按转角要 2.3 / 3.0 / 4.5 m/s² 横向加速度,实际 1.80 / 1.99 / 2.45(抓地 2.6 的 tanh)。"""
    import math
    from common.params import CarParams
    from simulation.fastsim.vehicle import Car
    from dataclasses import replace as rp
    p = rp(CarParams(), steer_rate=100.0, speed_tau=1e-4, accel_max=1e3)
    for want, got in ((2.3, 1.80), (3.0, 1.99), (4.5, 2.45)):
        car = Car(p); car.reset(0.0, 0.0, 0.0); car.v = 2.0
        steer = math.atan(want * p.wheelbase / 4.0)
        car.step(2.0, steer, 0.01)
        assert car.v * car.yaw_rate == pytest.approx(got, abs=0.15)


def test_turn_drag_lowers_the_speed_reached_in_a_turn():
    """runs/drive_1024:实车 |δ|~0.3 rad 时车速只有目标的 0.8~0.88。turn_drag 只打折平台以上的目标,直行不变。"""
    pl = Plant(v_plateau=0.6, v_top=4.0, launch_delay=0.0, accel_up=5.0, decel=0.9, dead_time=0.2, turn_drag=2.0)
    straight, turn = SpeedPlant(pl), SpeedPlant(pl)
    for _ in range(400):
        straight.step(2.0)
        turn.step(2.0, steer=0.3)
    assert straight.v == pytest.approx(2.0)
    assert turn.v == pytest.approx(2.0 * (1 - 2.0 * 0.09))
    low = SpeedPlant(pl)
    for _ in range(400):
        low.step(0.8, steer=0.38)
    assert low.v >= pl.v_plateau                   # 打折不低于平台(电调不能慢爬)


def test_stop_coasts_to_zero_and_never_reverses():
    pl = Plant(v_plateau=0.6, v_top=1.0, launch_delay=0.0, accel_up=5.0, decel=0.73, dead_time=0.2)
    sp = SpeedPlant(pl)
    _run_to(sp, 0.7)
    v = [sp.step(STOP) for _ in range(300)]
    assert min(v) == 0.0 and all(x >= 0.0 for x in v)
    assert -np.diff(v).max() <= 0.73 * 0.01 + 1e-9         # 中位只是滑行


def test_brake_matches_1012_measurements_above_the_release_speed():
    """docs/实测/2026-10-12-brake:1470 从 1.51 m/s 刹到 0.15 用 1.22~1.24 s、走 1.03~1.07 m(稳定段 1.28~1.35 m/s²);
    从 2.0 m/s 用 1.40~1.44 s、走 1.45~1.51 m(1.49~1.50)。刹车滞后统一用前进的 dead_time(取 0.2,实测 0.12~0.2)。"""
    for v0, dec, t_meas, d_meas in ((1.51, 1.31, 1.23, 1.05), (2.0, 1.495, 1.42, 1.48)):
        pl = Plant(v_plateau=0.6, v_top=2.5, launch_delay=0.0, accel_up=50.0, dead_time=0.2, brake_decel=dec)
        sp = SpeedPlant(pl)
        _run_to(sp, v0)
        sp.queue = [v0] * len(sp.queue)                       # 发刹车前一直在给前进
        t = d = 0.0
        while sp.v > 0.15:
            d += sp.v * 0.01
            sp.step(BRAKE)
            t += 0.01
        assert t == pytest.approx(t_meas, rel=0.1)
        # 距离只许偏长(偏保守):实测的"距离"基本等于一按就减速,仿真的 0.2 s 滞后里车还按原速走(2 m/s 时 +0.4 m,约 +18%)
        assert d_meas * 0.98 <= d <= d_meas * 1.2


def test_plant_brakes_literally_and_counts_low_speed_braking():
    """动力学照字面执行刹车(松刹在发指令那一侧);低于 REVERSE_RISK_V 还在刹的时间记进 reverse_s。"""
    pl = Plant(v_plateau=0.6, launch_delay=0.0, accel_up=5.0, decel=0.73, brake_decel=2.0, dead_time=0.2)
    sp = SpeedPlant(pl)
    _run_to(sp, 0.9)
    assert sp.reverse_s == 0.0
    v = [sp.step(BRAKE) for _ in range(150)]
    assert min(v) == 0.0 and all(x >= 0.0 for x in v)                  # 仿真里不倒车
    drops = -np.diff(v)
    want = np.minimum(2.0, BRAKE_A0 + BRAKE_K * np.array(v[:-1])) * 0.01      # 刹车减速度随车速变小(10-23 实测)
    live = (np.array(v[1:]) > 0) & (drops > 0)                                # 前 dead_time 拍刹车还没起效
    assert drops[live] == pytest.approx(want[live], rel=1e-6)
    assert sp.reverse_s > 0.3


def test_env_releases_brake_on_measured_speed_like_the_car():
    """RoadEnv 发指令时测得车速 <= brake_release_speed 就把刹车改成中位(release_brake,车上同一段);
    因为 dead_time 滞后,已发出的刹车照样执行,车速会降到松刹点以下。"""
    from common.road_policy import spec_of
    enc = spec_of('spica_v3_s3_v0.45-1_d0.2')
    pl = Plant(v_plateau=0.6, v_top=1.0, cap=1.0, launch_delay=0.0, accel_up=5.0, decel=0.73, brake_decel=1.4,
               dead_time=0.2, lag=0, enc=enc)
    env = RoadEnv(corridor(2.0, 30.0), pl, 0, noise=False, sense=False)
    while env.car.v < 0.99:
        env.step((0.0, 1.0))
    sent, v = [], []
    for _ in range(40):
        env.step((0.0, -1.0))
        sent.append(env.speed.queue[-1])
        v.append(env.car.v)
    rel = CarParams().brake_release_speed
    k = sent.index(STOP)                                             # 第一次松刹(之后一直是中位)
    assert k > 0 and set(sent[k:]) == {STOP}
    assert v[k - 1] > rel - 0.1 and v[k + 4] < rel - 0.2               # 已发出的刹车在滞后里照样执行,刹过松刹点


def test_relaunch_after_stop_waits_then_jumps_to_plateau():
    """死区:停稳后再给油不会慢慢爬,等 dead_time + relaunch_delay 原地不动,然后一跳到平台。还在滚的时候给油照常加速。"""
    pl = Plant(v_plateau=0.6, v_top=1.0, launch_delay=0.0, accel_up=0.6, decel=0.73, dead_time=0.2, relaunch_delay=0.2)
    sp = SpeedPlant(pl)
    _run_to(sp, 0.6)
    while sp.v > 0:
        sp.step(STOP)
    v = [sp.step(0.45) for _ in range(100)]
    assert v[:39] == [0.0] * 39                            # 0.2 s 指令延迟里还在发 STOP + 0.2 s 再起步等待
    assert v[45] > 0 and v[95] == pytest.approx(0.6, abs=0.01)   # 一动就往平台跳(0.1 s 时间常数);要 0.45 也是平台,不爬
    sp2 = SpeedPlant(pl)
    _run_to(sp2, 0.6)
    for _ in range(25):                                      # 只松 0.25 s,车还在滚
        sp2.step(STOP)
    rolling = sp2.v
    v2 = [sp2.step(0.6) for _ in range(40)]
    assert rolling > 0.3 and min(v2) > 0.3                  # 不停下来等电调


def test_clearance_matches_simple_geometry():
    road = corridor(1.0)
    env = RoadEnv(road, Plant(), 0, noise=False, sense=False)
    car = env.car
    car.x, car.y, car.yaw = 1.0, 0.0, 0.0
    assert footprint_clearance(car, road) == pytest.approx(0.5 - CarParams().width / 2, abs=0.01)
    car.y = 0.4                                 # 车身压到左墙
    assert footprint_clearance(car, road) == 0.0


def test_observation_and_action_shapes():
    road = corridor(1.5, mat=(CONE, WALL))
    env = RoadEnv(road, Plant(), 0)
    assert env.observation().shape == (DIM,)
    assert DIM * FRAMES == 265
    assert action_to_speed(-1) == V_LO and action_to_speed(1) == V_HI
    assert speed_to_action(action_to_speed(0.3)) == pytest.approx(0.3)


@pytest.mark.parametrize('steer_action', [-0.5, 0.5])
def test_fastsim_applies_the_measured_steering_curve(steer_action):
    road = corridor(1.5)
    pl = Plant(steer_rate=5.0, launch_delay=2.0)
    env = RoadEnv(road, pl, 0, noise=False, sense=False)

    env.step((steer_action, -1.0))

    expected = action_to_steer(steer_action, env.cp)
    assert env.car.steer == pytest.approx(expected, abs=1e-6)
    assert abs(expected - steer_action * env.cp.max_steer) > 0.01


def test_teacher_label_is_what_the_car_would_send():
    road = corridor(1.5)
    pl = Plant()
    env = RoadEnv(road, pl, 0, noise=False, sense=False)
    t = Teacher(env, plan(road, 0, 0.0, CarParams().width, pl))
    a = t()
    assert a.shape == (2,) and np.all(np.abs(a) <= 1)


def test_l6f_is_a_narrow_sparse_square_corner_after_a_long_lead_in():
    """L6f:路宽 1.15~1.5、外侧稀桶;方角前若是缓 S 弯或纯直道,那段直道 4~7m(约六成的角),
    若是急弯(45/90°)后的短直道则照旧 0.5~2.5m。其余关卡 gap_lead=None,抽取顺序不变。"""
    lv = next(l for l in LEVELS if l.name == 'L6f')
    assert lv.gap_lead == (4.0, 7.0) and lv.width == (1.15, 1.5) and lv.gap_spacing == (1.2, 1.5)
    # L6d 是后来新增的密桶方角级,同样用 gap_lead —— 这条断言守的是"加 L6f 没改动
    # 已有各级的抽取顺序",所以只对非 gap_lead 关卡成立。
    assert all(l.gap_lead is None for l in LEVELS if l.name not in ('L6f', 'L6d'))
    leads = []
    for seed in range(10):
        road = generate(np.random.default_rng(seed), lv)
        for i, e in enumerate(road.elems):
            if e.get('gap_corner') is not None:
                assert road.elems[i - 1]['kind'] == 'straight'
                leads.append(road.elems[i - 1]['length'])
    assert all(4.0 <= x <= 7.0 or 0.5 <= x <= 2.5 for x in leads)
    assert sum(4.0 <= x <= 7.0 for x in leads) >= len(leads) // 3


def test_teacher_brakes_only_with_a_dead_band_contract():
    """能刹车的契约(_d):老师在弯前光滑行来不及时给刹车标签,照样开到终点;老契约不规划刹车(标签逐位不变)。"""
    from common.road_policy import spec_of
    from training.road_check import sample_episode
    enc = spec_of('spica_v3_s5_v0.45-5_d0.2_log')
    road, pl, start, P, _ = sample_episode(2, 'L5', enc=enc, cap_max=2.0)
    env = RoadEnv(road, pl, 0, start, noise=False, sense=False)
    t = Teacher(env, P)
    why, labels = None, []
    while why is None:
        a = t()
        labels.append(action_to_speed(float(a[1]), enc))
        why = env.step(a)
    assert why == 'goal' and labels.count(BRAKE) > 10 and STOP not in labels
    old = Teacher(RoadEnv(road, Plant(), 0, start, noise=False, sense=False), P)
    assert old.brake is None and plan(road, 0, 0.0, CarParams().width, Plant()).v_coast is None
