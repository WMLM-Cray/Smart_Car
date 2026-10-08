"""bev_v1 的训练/评估环境:RoadEnv + BEV 累积 + 假 YOLO,以及"只用已扫到信息"的老师。

位姿:用 FastSim 的真位姿(车上用雷达里程计,用户实测漂移很小,先不建漂移模型)。
雷达:每次 RoadEnv 真扫描时,按**扫描那一刻**的位姿登记进 BEV。
摄像头:每 yolo_period 秒检测一次(香橙派上 YOLO 帧率待实测,先按 10 Hz [假设])。

老师(SeenTeacher):特权车辆参数(和 road_train 一样),但重规划窗口只到"左右边界都被雷达
实际扫到过"的地方;窗口没到路终点时末端限速 sqrt(a_lat·R_MIN)。标签因此只依赖学生 BEV 里
有的信息(BEV 第 0 步实验:这个老师在 L5/L6h/L4c/L6f 上 98.3%~100%)。
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree

from common.bev import Bev
from common.bev_policy import bev_image, scalars
from simulation.fastsim.camera import FakeYolo
from simulation.fastsim.road import DS
from simulation.fastsim.road_env import RoadEnv
from training.road_teacher import HORIZON, R_MIN, R_REPLAN, Teacher, plan

SEEN_R = 0.8            # 边界点离雷达回波这么近算"看到过"(桶距最大 1.5 m)
MIN_WIN = 0.5           # 重规划窗口至少看这么远 m
YOLO_PERIOD = 0.1       # [假设] 摄像头检测周期 s


class BevEnv:
    """包一层 RoadEnv:每拍维护 BEV,observation() 给 (图, 标量)。"""

    def __init__(self, road, pl, seed, start=(0, 0.0, 0.0), noise=True, robust=False,
                 yolo_period=YOLO_PERIOD):
        self.env = RoadEnv(road, pl, seed, start, noise=noise, robust=robust)
        self.bev = Bev(lidar_yaw=pl.lidar_yaw)
        self.cam = FakeYolo(road, seed=seed + 7)
        self.yolo_period, self.next_yolo = yolo_period, 0.0
        self.scan_hooks = []
        env, scan = self.env, self.env._scan

        def scan_and_register():
            scan()
            car = env.car
            pose = (car.x, car.y, car.yaw)
            self.bev.update_lidar(env.ranges, env.angles, pose, env.t)
            for hook in self.scan_hooks:
                hook(pose)

        env._scan = scan_and_register
        self.bev.update_lidar(env.ranges, env.angles, self.pose, 0.0)    # 构造时已扫过的那一帧
        self._camera()

    @property
    def pose(self):
        c = self.env.car
        return (c.x, c.y, c.yaw)

    def _camera(self):
        if self.env.t >= self.next_yolo:
            kinds, xy = self.cam.detect(self.pose)
            self.bev.update_camera(kinds, xy, self.pose, self.env.t)
            self.next_yolo += self.yolo_period

    def observation(self):
        env = self.env
        enc_speed = env.car.v + (env.rng.normal(0, 0.02) if env.noise else 0.0)
        gyro = env.car.yaw_rate + (env.rng.normal(0, env.cp.gyro_sigma) if env.noise else 0.0)
        img = bev_image(self.bev, self.pose, env.t, env.ranges, env.angles)
        return img, scalars(enc_speed, env.pl.cap, gyro, env.pl.enc)

    def step(self, action):
        why = self.env.step(action)
        self._camera()
        return why


class SeenTeacher(Teacher):
    """重规划窗口只到左右边界都被雷达扫到过的地方。"""

    def __init__(self, benv: BevEnv, P, a_lat: float):
        super().__init__(benv.env, P, a_lat=a_lat)
        road = benv.env.road
        self.nb = len(road.left)
        self.tree = cKDTree(np.vstack([road.left, road.right]))
        self.seen = np.zeros(2 * self.nb, bool)
        self.v_end = math.sqrt(a_lat * R_MIN)
        benv.scan_hooks.append(self._mark)
        self._mark(benv.pose)

    def _mark(self, pose):
        env = self.env
        from common.bev import lidar_points
        pts = lidar_points(env.ranges, env.angles, pose, env.cp, env.pl.lidar_yaw, 10.0)
        for idx in self.tree.query_ball_point(pts, SEEN_R):
            self.seen[idx] = True

    def known_ahead(self, i):
        both = self.seen[:self.nb] & self.seen[self.nb:]
        j = i
        while j < self.nb - 1 and both[j + 1]:
            j += 1
        return (j - i) * DS

    def _replan(self):
        env, car = self.env, self.env.car
        i, d = env.road.project((car.x, car.y), env.idx)
        win = min(HORIZON, max(MIN_WIN, self.known_ahead(i)))
        P = plan(env.road, i, d, env.cp.width, self.pl, win, R_REPLAN, self.a_lat, self.v_end)
        if isinstance(P, str):
            self.fails += 1
            return
        self.plan, self.j = P, 0
