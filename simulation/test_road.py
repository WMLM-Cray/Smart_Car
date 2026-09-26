import math

import numpy as np
import pytest

from common.params import CarParams
from simulation.road import CONE, LEVELS, WALL, build, generate, self_overlaps
from training.road_check import check
from simulation.road_env import Plant, RoadEnv, SpeedPlant, footprint_clearance
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
