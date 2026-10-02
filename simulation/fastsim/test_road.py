import math

import numpy as np
import pytest

from common.params import CarParams
from simulation.fastsim.road import CONE, LEVELS, WALL, build, generate, self_overlaps
from training.road_check import check
from simulation.fastsim.road_env import Plant, RoadEnv, SpeedPlant, footprint_clearance
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
