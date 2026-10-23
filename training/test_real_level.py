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
