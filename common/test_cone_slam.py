"""锥桶地标定位(common/cone_slam.py):锥桶提取、配准能从偏了的初值回到真位姿、建图后第二圈配准。"""
import math

import numpy as np

from common.cone_slam import ConeLocalizer, extract_cones, register, to_world
from common.params import CarParams, TrackParams
from simulation.fastsim.raycast import cast_cones

CP, TP = CarParams(), TrackParams()
ANG = -math.pi + np.arange(CP.lidar_points) * (2 * math.pi / CP.lidar_points)


def scan(cones, pose):
    x, y, th = pose
    o = np.array([x + CP.lidar_x * math.cos(th), y + CP.lidar_x * math.sin(th), CP.lidar_z])
    d = np.stack([np.cos(ANG + th), np.sin(ANG + th), np.zeros_like(ANG)], 1)
    return cast_cones(o, d, cones, 0.14, TP.cone_top_radius, TP.cone_height, 10.5)


def corridor():
    """两排桶夹一条 1.5 m 宽的路,中间拐个弯(不全是等间距直排,配准才有唯一解)。"""
    s = np.arange(0, 12, 1.0)
    left = np.c_[s, np.full_like(s, 0.75)]
    right = np.c_[s, np.full_like(s, -0.75)]
    bend = np.c_[12 + 0.75 * np.sin(np.linspace(0, 1.2, 4)), 0.75 + np.linspace(0.6, 3, 4)]
    return np.vstack([left, right, bend])


def test_extract_cones_finds_centres():
    cones = corridor()
    pose = (2.0, 0.1, 0.05)
    obs = to_world(extract_cones(scan(cones, pose), ANG), pose)
    d = np.min(np.hypot(*(obs[:, None, :] - cones[None]).transpose(2, 0, 1)), axis=1)
    assert len(obs) >= 10 and np.all(d < 0.08)


def test_register_recovers_pose_from_offset_guess():
    cones = corridor()
    pose = (3.0, 0.1, 0.1)
    obs = extract_cones(scan(cones, pose), ANG)
    m = register(obs, cones, (3.25, -0.1, 0.0))          # 初值偏 0.3 m、6°
    assert m.ok and math.hypot(m.pose[0] - 3.0, m.pose[1] - 0.1) < 0.05 and abs(m.pose[2] - 0.1) < 0.02


def test_localizer_builds_map_then_relocalizes_on_it():
    cones = corridor()
    path = [(x, 0.05 * math.sin(x), 0.05 * math.cos(x)) for x in np.arange(0.0, 9.0, 0.15)]
    cut = 2 * len(path) // 3
    loc = ConeLocalizer(path[0], every=1, build=True)      # 第一帧扫描就在 pose0 上:定义地图系
    for p in path[:cut]:
        est, _ = loc.step(scan(cones, p), ANG)
        assert math.hypot(est[0] - p[0], est[1] - p[1]) < 0.1
    loc.finish_map()                                       # 转成只对固定地图配准,接着往前开(前方的桶已经看到过)
    loc.every = 3
    for p in path[cut:]:
        est, m = loc.step(scan(cones, p), ANG)
        if m is not None:
            assert m.ok and math.hypot(est[0] - p[0], est[1] - p[1]) < 0.1
