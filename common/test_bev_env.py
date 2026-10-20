"""BevEnv 的本帧雷达对齐(PR #30 评审第 1 条)。"""
import math

import numpy as np

from common.bev import Bev, BevConfig
from common.bev_policy import bev_image
from common.road_policy import V_LO, Encoding
from training.bev_env import BevEnv, SeenTeacher

ENC = Encoding(3.0, V_LO, 3.0)


def _benv(level='L6f', seed=5):
    from simulation.fastsim.road import LEVELS, generate
    from simulation.fastsim.road_env import Plant
    rng = np.random.default_rng(seed)
    road = generate(rng, next(l for l in LEVELS if l.name == level))
    from common.params import CarParams
    from training.road_teacher import plan
    pl = Plant.sample(rng, ENC, 2.5)
    benv = BevEnv(road, pl, seed, noise=False)
    return benv, SeenTeacher(benv, plan(road, 0, 0.0, CarParams().width, pl, a_lat=1.47), 1.47)


def test_current_scan_layer_lines_up_with_memory_after_moving():
    """固定障碍:在 pose0 扫一帧,车平移 + 转弯到 pose1 后还没新扫描。本帧层要按扫描位姿投影,
    和记忆层落在同一批格子;按当前位姿投影(旧做法)就会错开。"""
    ang = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    ranges = np.where(np.abs(ang) < math.radians(10), 3.0, np.inf)     # 正前方 3 m 一段墙
    bev = Bev(BevConfig())
    pose0, pose1 = (0.0, 0.0, 0.0), (0.15, 0.02, math.radians(6))
    bev.update_lidar(ranges, ang, pose0, 0.0)
    img = bev_image(bev, pose1, 0.05, ranges, ang, scan_pose=pose0)
    assert img[4].sum() > 5
    assert _gap(img) <= 1                                   # 记忆层按 odom 格子重采样,允许 1 格(10 cm)取整差
    stale = bev_image(bev, pose1, 0.05, ranges, ang)
    assert _gap(stale) >= 2                                 # 旧做法:平移 0.15 m + 转 6°,3 m 外错开 ~0.3 m


def _gap(img):
    """本帧雷达每个格子到最近记忆格子的最大距离(格)。"""
    now, mem = np.argwhere(img[4] > 0), np.argwhere(img[0] > 0)
    return float(np.max(np.min(np.abs(now[:, None] - mem[None]).max(2), 1)))


def test_benv_feeds_scan_pose_between_scans():
    benv, teacher = _benv()
    for _ in range(40):
        img, _ = benv.observation()
        if img[4].any():
            assert _gap(img) <= 1                           # 本帧雷达的格子在记忆层里都有(差 1 格内)
        benv.step(teacher())
