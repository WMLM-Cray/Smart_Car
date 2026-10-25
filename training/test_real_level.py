"""实车赛道关卡(LRA20/LRA23)和老师计划减速度可调(road_teacher.set_decel,bev_train --teacher-dec/--teacher-brk)。"""
import numpy as np

import training.road_teacher as RT
from common.params import CarParams
from common.road_policy import spec_of
from evaluation.road_track import real_track
from simulation.fastsim.road_env import Plant
from training.road_check import sample_episode

ENC = spec_of('spica_v3_s5_v0.45-5_d0.2_log')


def test_real_level_uses_cone_map():
    road, *_ = sample_episode(7, 'LRA23', enc=ENC, a_lat=3.5, cap_max=2.5, cap_min=1.2)
    assert road is real_track('realA_1023')[0]


def test_set_decel_raises_planned_speed_before_corners():
    road, _ = real_track('realA_1023')
    pl = Plant.sample(np.random.default_rng(0), ENC, 2.5, 2.5)
    base = RT.plan(road, 0, 0.0, CarParams().width, pl, a_lat=3.5, brake=RT.A_BRK)
    old = RT.A_DEC, RT.A_BRK
    try:
        RT.set_decel(0.7, 1.2)
        fast = RT.plan(road, 0, 0.0, CarParams().width, pl, a_lat=3.5, brake=RT.A_BRK)
    finally:
        RT.set_decel(*old)
    assert np.all(fast.v >= base.v - 1e-9) and fast.v.mean() > base.v.mean()
    assert np.all(fast.v_coast >= base.v_coast - 1e-9) and fast.v_coast.mean() > base.v_coast.mean()
    assert (RT.A_DEC, RT.A_BRK) == old


def test_grip_feedback_caps_speed_label_when_wheels_are_turned_hard():
    """推头反馈(road_teacher.set_grip_feedback,bev_train --teacher-grip-fb):前轮实际打 19° 时速度标签
    不超过 sqrt(a·L/tan δ) − v_over;关着时和原来逐位一样。"""
    import math
    from dataclasses import replace

    from common.road_policy import action_to_speed
    from simulation.fastsim.road_env import RoadEnv
    road, _ = real_track('realA_1023')
    pl = replace(Plant.sample(np.random.default_rng(0), ENC, 4.0, 4.0), cap=4.0)
    cp = CarParams()
    labels = []
    for fb in (None, 2.0):
        env = RoadEnv(road, pl, seed=0)
        env.car.v, env.car.steer = 3.0, math.radians(19)
        P = RT.plan(road, 0, 0.0, cp.width, pl, a_lat=2.0)
        RT.set_grip_feedback(fb)
        try:
            labels.append(RT.Teacher(env, P, replan=False, a_lat=2.0)())
        finally:
            RT.set_grip_feedback(None)
    off, on = (action_to_speed(float(a[1]), ENC) for a in labels)
    v_ok = math.sqrt(2.0 * cp.wheelbase / math.tan(math.radians(19))) - pl.v_over
    assert labels[0][0] == labels[1][0]                      # 转角标签不受影响
    assert on <= max(v_ok, RT.BRAKE_FLOOR) + 0.05 < off


def test_grip_feedback_zero_is_rejected_not_silently_on():
    """0 不是关(会把速度标签恒压到 BRAKE_FLOOR),要报错;关就传 None。"""
    import pytest
    for bad in (0.0, -1.0):
        with pytest.raises(ValueError):
            RT.set_grip_feedback(bad)
    assert RT.GRIP_FB is None
