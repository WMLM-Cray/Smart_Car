"""路中障碍(LO3 / LOE3):雷达扫得到、碰了算撞、老师绕得过去、假地面分割扣掉它和它挡住的地面;别的级别逐位不变。"""
import math

import numpy as np

from simulation.fastsim.road import LEVELS, OBST_BLOCK, OBST_PASS, add_obstacles, generate
from simulation.fastsim.road_env import Plant, RoadEnv

LV = {l.name: l for l in LEVELS}


def test_levels_without_obstacles_do_not_touch_the_rng():
    road = generate(np.random.default_rng(5), LV['L3'])
    rng = np.random.default_rng(7)
    before = rng.bit_generator.state
    add_obstacles(road, rng, LV['L3'])
    assert rng.bit_generator.state == before and len(road.obstacles) == 0


def test_obstacles_block_one_side_and_leave_room_on_the_other():
    for seed in range(6):
        road = generate(np.random.default_rng(seed), LV['LO3'])
        assert len(road.obstacles) >= 1
        for box in road.obstacles:
            i, d = road.project(box.mean(0))
            n = road.normal[i]
            lat = (box - road.centre[i]) @ n                     # 四个角的横向位置
            gaps = sorted([road.half[i] - lat.max(), lat.min() + road.half[i]])   # 按原路宽(可行域已被障碍收窄)
            assert gaps[1] >= OBST_PASS - 1e-6                   # 可行域已收窄,但还留着绕行空间
            assert gaps[0] <= OBST_BLOCK + 1e-6                  # 被挡那一侧车过不去:往哪绕从画面看得出来


def test_lidar_sees_the_obstacle_and_driving_into_it_is_a_collision():
    road = generate(np.random.default_rng(1), LV['LOE3'])       # 无锥桶无墙:雷达只能扫到障碍
    box = road.obstacles[0]
    i, _ = road.project(box.mean(0))
    back = max(0, i - int(1.5 / 0.05))
    env = RoadEnv(road, Plant.sample(np.random.default_rng(0)), 0, start=(back, 0.0, 0.0), noise=False)
    c = box.mean(0)
    r = env.scan_at(*road.centre[back], math.atan2(*(c - road.centre[back])[::-1]))
    assert np.isfinite(r).any() and r.min() < 2.0
    car = env.car
    car.x, car.y = c
    car.yaw = float(road.heading[i])
    assert env._check() == 'collision'


def test_floor_seg_skips_the_obstacle_and_its_shadow():
    from simulation.fastsim.camera import FakeFloorSeg
    from common.params import CarParams
    road = generate(np.random.default_rng(1), LV['LOE3'])
    box = road.obstacles[0]
    i, _ = road.project(box.mean(0))
    k = max(0, i - int(1.2 / 0.05))
    p = CarParams()
    pose = (*(road.centre[k] - p.camera_x * np.array([math.cos(road.heading[k]), math.sin(road.heading[k])])),
            float(math.atan2(*(box.mean(0) - road.centre[k])[::-1])))
    seg = FakeFloorSeg(road, seed=0)
    seg.cfg = type(seg.cfg)(miss=0.0, edge_sigma=0.0, jitter=0.0)
    xy = seg.detect(pose)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    w = np.c_[x + c * xy[:, 0] - s * xy[:, 1], y + s * xy[:, 0] + c * xy[:, 1]]
    behind = box.mean(0) + 0.4 * (box.mean(0) - road.centre[k]) / np.linalg.norm(box.mean(0) - road.centre[k])
    assert np.hypot(*(w - box.mean(0)).T).min() > 0.02           # 障碍底下没有地面点
    assert np.hypot(*(w - behind).T).min() > 0.08                # 正后方被挡住
    assert len(xy) > 200                                         # 其余地面还在
