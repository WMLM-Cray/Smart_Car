"""bev_v1 的训练/评估环境:RoadEnv + BEV 累积 + 假 YOLO,以及"只用已扫到信息"的老师。

位姿:用 FastSim 的真位姿(车上用雷达里程计,用户实测漂移很小,先不建漂移模型)。
雷达:每次 RoadEnv 真扫描时,按**扫描那一刻**的位姿登记进 BEV。
摄像头:每 yolo_period 秒检测一次(香橙派上 YOLO 帧率待实测,先按 10 Hz [假设])。

老师(SeenTeacher)的信息合同:**道路几何**只用"左右边界都被雷达扫到过、且没过 BEV 的
max_age"的那一段 —— 和学生 BEV 里还留着的证据同一时效。初始化、每拍重规划、重规划失败的
回退都守这一条:
  - 规划窗口 = 已知前方长度(不设真值保底);窗口没到路终点时末端限速 sqrt(a_lat·R_MIN)
  - 一开始就没有合规计划、或旧计划剩下不到 MIN_LEFT,就"摸着走":不转向、最低速度,等雷达扫到更多
仍然是特权的:车辆参数(延迟、增益、中位等,和 road_train 一样)和"哪些格子算边界"(用真值
边界点去对雷达回波)。所以它是"只用学生已看到的路"的老师,不是"只用 BEV 像素"的老师。
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
from training.road_teacher import HORIZON, R_MIN, R_REPLAN, STEP, Teacher, plan

SEEN_R = 0.8            # 边界点离雷达回波这么近算"看到过"(桶距最大 1.5 m)
MIN_LEFT = 0.3          # 重规划失败时旧计划在车前至少还剩这么长 m,否则改"摸着走"
MIN_PLAN = 2 * STEP * DS    # plan() 至少要 3 个粗网格点(0.5 m);已知更短就当没看见,不去补真值
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
            self.scan_pose = (car.x, car.y, car.yaw)
            self.bev.update_lidar(env.ranges, env.angles, self.scan_pose, env.t)
            for hook in self.scan_hooks:
                hook(self.scan_pose)

        env._scan = scan_and_register
        self.scan_pose = self.pose                                        # 构造时已扫过的那一帧
        self.bev.update_lidar(env.ranges, env.angles, self.scan_pose, 0.0)
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
        img = bev_image(self.bev, self.pose, env.t, env.ranges, env.angles, self.scan_pose)
        return img, scalars(enc_speed, env.pl.cap, gyro, env.pl.enc)

    def step(self, action):
        why = self.env.step(action)
        self._camera()
        return why


class SeenTeacher(Teacher):
    """重规划窗口只到左右边界都被雷达扫到过(且未过期)的地方;合同见文件头。"""

    def __init__(self, benv: BevEnv, P, a_lat: float):
        # 不接全路计划 P(参数留着和 Teacher 同签名):初始计划也只用已知段。重规划由这里每拍做,
        # 父类不再自己重规划(它的失败回退会沿用旧计划,不守合同)。
        super().__init__(benv.env, None, replan=False, a_lat=a_lat)
        road = benv.env.road
        self.nb = len(road.left)
        self.tree = cKDTree(np.vstack([road.left, road.right]))
        self.seen_t = np.full(2 * self.nb, -np.inf)          # 每个边界点最后一次被扫到的时刻
        self.max_age = benv.bev.cfg.max_age
        self.v_end = math.sqrt(a_lat * R_MIN)
        self.creeps = 0
        benv.scan_hooks.append(self._mark)
        self._mark(benv.scan_pose)

    def _mark(self, pose):
        env = self.env
        from common.bev import lidar_points
        pts = lidar_points(env.ranges, env.angles, pose, env.cp, env.pl.lidar_yaw, 10.0)
        for idx in self.tree.query_ball_point(pts, SEEN_R):
            self.seen_t[idx] = env.t

    def known(self):
        fresh = self.env.t - self.seen_t <= self.max_age
        return fresh[:self.nb] & fresh[self.nb:]

    def known_ahead(self, i):
        both = self.known()
        if not both[i]:
            return 0.0
        j = i
        while j < self.nb - 1 and both[j + 1]:
            j += 1
        return (j - i) * DS

    def _left(self):
        """旧计划在车前还剩多长。"""
        car = self.env.car
        dd = self.plan.pts - (car.x, car.y)
        k = int(np.argmin(np.einsum('ij,ij->i', dd, dd)))
        return float(self.plan.s[-1] - self.plan.s[k])

    def _replan(self):
        env, car = self.env, self.env.car
        i, d = env.road.project((car.x, car.y), env.idx)
        win = min(HORIZON, self.known_ahead(i))
        P = plan(env.road, i, d, env.cp.width, self.pl, win, R_REPLAN, self.a_lat, self.v_end) if win >= MIN_PLAN else 'unseen'
        if isinstance(P, str):
            self.fails += 1
            if self.plan is not None and self._left() < MIN_LEFT:
                self.plan = None
            return
        self.plan, self.j = P, 0

    def __call__(self) -> np.ndarray:
        self._replan()
        if self.plan is None:                                # 摸着走:不转向、最低速度
            self.n += 1
            self.creeps += 1
            return np.array([0.0, -1.0], np.float32)
        return super().__call__()
