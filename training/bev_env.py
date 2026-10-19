"""bev_v1 的训练/评估环境:RoadEnv + BEV 累积 + 假 YOLO,以及"只用已扫到信息"的老师。

位姿:用 FastSim 的真位姿(车上用雷达里程计,用户实测漂移很小,先不建漂移模型)。
雷达:每次 RoadEnv 真扫描时,按**扫描那一刻**的位姿登记进 BEV。
摄像头:每 yolo_period 秒检测一次(香橙派上 YOLO 帧率待实测,先按 10 Hz [假设])。
地面分割(bev_v2,free=True):每 seg_period 秒拍一次,seg_lag 秒后(推理延迟)按拍摄那一刻的位姿登记进 FREE 层
(FakeFloorSeg;和车上 BevRuntime.on_free 一样)。默认 0.1 s、无延迟是训练时的设定;实车 M2F-T 288 是 ~0.33 s / ~0.35 s。

老师(SeenTeacher):特权车辆参数(和 road_train 一样);重规划窗口只到"左右边界都被雷达扫到过"
的地方,窗口没到路终点时末端限速 sqrt(a_lat·R_MIN)。它**不是**严格只用学生 BEV 里的信息:
  - 已知前方不足 MIN_WIN 时仍按真值道路规划 MIN_WIN(plan() 至少要 3 个粗网格点)
  - 初始计划是全路计划,重规划失败时沿用旧计划(开局连续失败就会用上它)
  - "扫到过"不过期,学生 BEV 20 s 后会忘
2026-10-09 试过严格版(去掉以上三条,未知就最低速直行):老师自己开成功率 99.4% -> 95.4%
(0.45 m/s²/限速 1.8)、98.4% -> 95.3%(1.47/2.5),L6h/L6f 掉 5~9 个点,所以保留现在的特权。
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree

from common.bev import Bev, BevConfig, CellType
from common.bev_policy import bev_image, bev_images, scalars
from simulation.fastsim.camera import FakeFloorSeg, FakeYolo
from simulation.fastsim.road import DS, EDGE
from simulation.fastsim.road_env import RoadEnv
from training.road_teacher import HORIZON, R_MIN, R_REPLAN, Teacher, plan

SEEN_R = 0.8            # 边界点离雷达回波这么近算"看到过"(桶距最大 1.5 m)
FLOOR_SEEN_R = math.sqrt(2) * BevConfig().res  # 地面只容许一格量化误差,不能沿用稀疏锥桶的 0.8 m
MIN_WIN = 0.5           # 重规划窗口至少看这么远 m
YOLO_PERIOD = 0.1       # [假设] 摄像头检测周期 s
SEG_PERIOD = 0.1        # [假设] 地面分割周期 s(NPU 上 SegFormer-B0 待实测)


class BevEnv:
    """包一层 RoadEnv:每拍维护 BEV,observation() 给 (图, 标量),near=True 时追加第三项近场图。"""

    def __init__(self, road, pl, seed, start=(0, 0.0, 0.0), noise=True, robust=False,
                 yolo_period=YOLO_PERIOD, free=False, seg_period=SEG_PERIOD, seg_lag=0.0, aug=(0.0, 0.0),
                 yolo=True, guard=False, seg_lut=False, seg_fill=False, world=None, near=False):
        # world(RealMap):在车上建的实景地图里开,road 不用,start = (x, y, yaw);没有 YOLO,地面分割按车上的投影 + 补缝
        if world is None:
            self.env = RoadEnv(road, pl, seed, start, noise=noise, robust=robust)
        else:
            from simulation.fastsim.real_map_env import RealMapEnv
            self.env = RealMapEnv(world, pl, seed, start, noise=noise)
            yolo = False
        # 裁图位姿抖动 (平移 m, 转角 rad),只在训练采集时开:BEV 是 5 cm 格,墙跨一格网络输出就跳(v3.5m5 横移 0.5 cm
        # 转角跳 10°,2026-10-15)。每拍按随机亚格偏移裁图,同一几何见过各种量化相位,学成对格子位置不敏感。车上不抖
        self.aug = aug
        self.aug_rng = np.random.default_rng(seed + 13) if aug[0] > 0 or aug[1] > 0 else None   # 独立随机流
        self.bev = Bev(lidar_yaw=pl.lidar_yaw)
        self.cam = FakeYolo(road, seed=seed + 7) if world is None else None
        self.yolo_period, self.next_yolo = yolo_period, 0.0
        # yolo=False:不登记假 YOLO(锥桶/胶带/区域层恒空),和车上一样(车上没跑 YOLO,2026-10-16)。
        # 路上的锥桶照样在,雷达照样扫得到、撞了照样算撞
        self.yolo = yolo
        self.free = free
        self.near = near
        if self.near and not self.free:
            raise ValueError('bev_v2.5 近场输入需要同时启用 6 通道主图')
        # seg_lut:假地面分割按车上的像素投影出点(远处条纹、没确认时只到 2 m),见 FloorSegConfig.lut
        from dataclasses import replace as _replace
        from simulation.fastsim.camera import FloorSegConfig
        seg_cfg = _replace(FloorSegConfig(), lut=(72, 96), fill=seg_fill) if seg_lut else FloorSegConfig()
        self.seg = FakeFloorSeg(road, cfg=seg_cfg, seed=seed + 11) if free and world is None else None    # 独立随机流:不改 bev_v1 的局
        if world is not None and free:
            from simulation.fastsim.real_map_env import MapFloorSeg
            self.seg = MapFloorSeg(world, seed=seed + 11)
        self.seg_period, self.seg_lag, self.next_seg = seg_period, seg_lag, 0.0
        self.seg_queue = []                                               # (到达时刻, 拍摄位姿, 点)
        self.seg_hooks = []                                               # 已送到 BEV 的帧:pose, xy, stamp
        self.seg_frame = None
        self.scan_hooks = []
        # guard=True:和车上 road_car_node 一样按将要打的转角扫车身矩形,保险杠前净空 < GUARD_STOP 就算这局失败('guard')。
        # 车上截停后油门回中位、策略不出倒车,基本就卡死了;仿真里原来不管,模型在锥桶方角贴着开也算过(2026-10-16:
        # 无锥桶 r29 在 L6f 16 局里 12 局会触发截停,老师 1 局)
        self.guard = None
        if guard:
            from vehicle.road_car_node import GUARD_STOP
            from vehicle.scan_guard import ScanGuard
            self.guard, self.guard_stop = ScanGuard(), GUARD_STOP
            self.scan_hooks.append(lambda pose: self._guard_scan())
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
        if self.guard is not None:
            self._guard_scan()
        self.bev.update_lidar(env.ranges, env.angles, self.scan_pose, 0.0)
        self._camera()

    @property
    def pose(self):
        c = self.env.car
        return (c.x, c.y, c.yaw)

    def _camera(self):
        if self.yolo and self.env.t >= self.next_yolo:
            kinds, xy = self.cam.detect(self.pose)
            self.bev.update_camera(kinds, xy, self.pose, self.env.t)
            self.next_yolo += self.yolo_period
        if self.seg is not None and self.env.t >= self.next_seg:
            self.seg_queue.append((self.env.t + self.seg_lag, self.pose, self.seg.detect(self.pose)))
            self.next_seg += self.seg_period
        while self.seg_queue and self.env.t >= self.seg_queue[0][0] - 1e-9:
            arrival, pose, xy = self.seg_queue.pop(0)
            stamp = arrival - self.seg_lag
            self.bev.update_camera(np.full(len(xy), int(CellType.FREE)), xy, pose, stamp)
            self.seg_frame = (pose, xy, stamp)
            for hook in self.seg_hooks:
                hook(pose, xy, stamp)

    def observation(self):
        env = self.env
        if self.near:
            img, near_img = self.images()
            enc_speed = env.car.v + (env.rng.normal(0, 0.02) if env.noise else 0.0)
            gyro = env.car.yaw_rate + (env.rng.normal(0, env.cp.gyro_sigma) if env.noise else 0.0)
            return img, scalars(enc_speed, env.pl.cap, gyro, env.pl.enc), near_img
        enc_speed = env.car.v + (env.rng.normal(0, 0.02) if env.noise else 0.0)
        gyro = env.car.yaw_rate + (env.rng.normal(0, env.cp.gyro_sigma) if env.noise else 0.0)
        return self.image(), scalars(enc_speed, env.pl.cap, gyro, env.pl.enc)

    def images(self):
        """同一裁图位姿下的主图和 5 cm 近场图;供 bev_v2.5 和成对增强采集使用。"""
        if not self.near:
            raise ValueError('当前 BevEnv 未启用近场分支')
        env, pose = self.env, self.pose
        if self.aug_rng is not None:
            d, a = self.aug
            pose = (pose[0] + self.aug_rng.uniform(-d, d), pose[1] + self.aug_rng.uniform(-d, d),
                    pose[2] + self.aug_rng.uniform(-a, a))
        return bev_images(self.bev, pose, env.t, env.ranges, env.angles, self.scan_pose)

    def image(self):
        """这一拍的主 BEV 图;开了 aug 就按一个新的随机亚格偏移裁。"""
        env, pose = self.env, self.pose
        if self.aug_rng is not None:
            d, a = self.aug
            pose = (pose[0] + self.aug_rng.uniform(-d, d), pose[1] + self.aug_rng.uniform(-d, d),
                    pose[2] + self.aug_rng.uniform(-a, a))
        return bev_image(self.bev, pose, env.t, env.ranges, env.angles, self.scan_pose, free=self.free)

    def _guard_scan(self):
        """把这一帧雷达按 ScanGuard.update 的规则换成后轴系前方点(车上喂的是同一份 /scan)。"""
        from vehicle.scan_guard import FRONT_HALF_ANGLE, LIDAR_X
        e = self.env
        a = (np.asarray(e.angles, float) + np.pi) % (2 * np.pi) - np.pi
        r = np.asarray(e.ranges, float)
        ok = np.isfinite(r) & (r > 0.05) & (np.abs(a) <= FRONT_HALF_ANGLE)
        x, y = r[ok] * np.cos(a[ok]) + LIDAR_X, r[ok] * np.sin(a[ok])
        self.guard.points = list(zip(x[x > 0], y[x > 0]))

    def step(self, action):
        if self.guard is not None:
            from common.pwm import action_to_steer
            steer = action_to_steer(float(np.clip(action[0], -1, 1)), self.env.cp)
            if self.guard._clear(steer) < self.guard_stop:
                return 'guard'
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
        # EDGE 只能由实际送到学生 BEV 的地面点确认,不能用理想视场提前登记。
        self.edge = np.concatenate([road.mat_left, road.mat_right]) == EDGE
        self.seg = benv.seg
        self.v_end = math.sqrt(a_lat * R_MIN)
        benv.scan_hooks.append(self._mark)
        self._mark(benv.pose)
        benv.seg_hooks.append(self._mark_floor)
        if benv.seg_frame is not None:
            self._mark_floor(*benv.seg_frame)

    def _mark(self, pose):
        env = self.env
        from common.bev import lidar_points
        pts = lidar_points(env.ranges, env.angles, pose, env.cp, env.pl.lidar_yaw, 10.0)
        for idx in self.tree.query_ball_point(pts, SEEN_R):
            self.seen[idx] |= ~self.edge[idx]

    def _mark_floor(self, pose, xy, stamp):
        if not len(xy) or not self.edge.any():
            return
        from common.bev import body_to_odom
        dist, _ = cKDTree(body_to_odom(xy, pose)).query(self.tree.data[self.edge])
        self.seen[self.edge] |= dist <= FLOOR_SEEN_R

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
