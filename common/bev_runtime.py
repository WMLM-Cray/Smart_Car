"""bev_v1 的运行时:和 RoadRuntime 同一套使能心跳、传感器过期、限速和输出,只换"大脑"的输入。

车上没有真位姿:编码器车速 + 陀螺偏航角速度用 PoseIntegrator 推 odom 位姿(用户实测雷达
里程计漂移很小,BEV 证据 20 s 过期,先不做扫描匹配)。每帧雷达按**收到那一刻**的位姿登记进
BEV;每拍按当前位姿出 5×80×80 图(本帧雷达按扫描位姿投影,和 training/bev_env.py 一致)。

摄像头:车上还没有 YOLO 节点,锥桶/胶带/区域三层是空的 —— 和训练时(FastSim 假 YOLO)不同,
上车前要先在仿真里关掉假 YOLO 验证过。接上 YOLO 后在 on_detections() 里登记。
地面分割(bev_v2):vehicle/ground_seg_node.py 发车体系地面点,on_free() 按**拍摄时刻**的位姿登记进 FREE 层
(分割要 ~60 ms,1.8 m/s 下按收到时刻登记会错 10 cm)。
"""
from __future__ import annotations

from collections import deque

import numpy as np

from common.bev import Bev, CellType, PoseIntegrator
from common.bev_policy import bev_image, scalars
from common.road_runtime import RoadRuntime

MAX_DT = 0.2            # 两次积分间隔超过这个就当传感器断过,不积分(停车/过期时 RoadRuntime 已回中位)
POSE_HISTORY = 1.0      # 位姿历史留这么久 s,给按拍摄时刻登记的摄像头结果查
FREE_TIMEOUT = 1.0      # bev_v2 地面分割 M2F-T 288 ~3 Hz(每帧 ~0.33 s,B0 时是 10 Hz / 0.5 s);断了 FREE 层只会变空、模型照开,所以当传感器过期回中位


class BevRuntime(RoadRuntime):

    def reset(self):
        self.bev = Bev(params=self.params)
        self.odom = PoseIntegrator()
        self.pose_t = None
        self.scan_pose = (0.0, 0.0, 0.0)
        self.history = deque()
        self.free_t = None
        super().reset()

    def _integrate(self, now):
        now = float(now)
        if self.pose_t is not None and 0.0 < now - self.pose_t <= MAX_DT:
            self.odom.step(self.speed, self.yaw_rate, now - self.pose_t)
        if self.pose_t is None or now > self.pose_t:
            self.pose_t = now
            self.history.append((now, self.odom.pose))
            while self.history and self.history[0][0] < now - POSE_HISTORY:
                self.history.popleft()

    def pose_at(self, t):
        """历史里离 t 最近的位姿;没有历史就用当前位姿。"""
        if not self.history:
            return self.odom.pose
        return min(self.history, key=lambda h: abs(h[0] - t))[1]

    def on_odometry(self, speed, now):
        self._integrate(now)            # 先用旧车速积到现在,再换新车速
        super().on_odometry(speed, now)

    def on_imu(self, yaw_rate, now):
        self._integrate(now)
        super().on_imu(yaw_rate, now)

    def on_scan(self, ranges, angle_min, angle_increment, now):
        super().on_scan(ranges, angle_min, angle_increment, now)
        self._integrate(now)
        self.scan_pose = self.odom.pose
        self.bev.update_lidar(self.scan, self.angles, self.scan_pose, float(now))
        self.bev.prune(float(now))

    def on_detections(self, kinds, xy_body, now):
        """摄像头检测(车体系落地点)登记进 BEV;接口同 FastSim 的 FakeYolo。"""
        self._integrate(now)
        self.bev.update_camera(kinds, xy_body, self.odom.pose, float(now))

    def on_free(self, xy_body, stamp, now):
        """stamp 和 now 同属 ROS 时钟;旧帧、倒序帧不能续期或重新登记。"""
        stamp, now = float(stamp), float(now)
        if not (np.isfinite(stamp) and np.isfinite(now) and 0.0 <= now - stamp <= FREE_TIMEOUT):
            return False
        if self.free_t is not None and stamp <= self.free_t:
            return False
        self._integrate(now)
        self.free_t = stamp
        if len(xy_body):
            self.bev.update_camera(np.full(len(xy_body), int(CellType.FREE)), xy_body,
                                   self.pose_at(stamp), stamp)
        return True

    def stale_sensors(self, now):
        stale = super().stale_sensors(now)
        if self.policy.free and not (self.free_t is not None and 0.0 <= float(now) - self.free_t <= FREE_TIMEOUT):
            stale.append('free')
        return stale

    def _reset_history(self):
        # 重新使能 / 传感器过期后从空 BEV 开始:停过车,旧证据和积分起点都不可信
        super()._reset_history()
        self.bev.reset()
        self.history.clear()

    def _act(self, now):
        self._integrate(now)
        img = bev_image(self.bev, self.odom.pose, float(now), self.scan, self.angles, self.scan_pose,
                        free=self.policy.free)
        self.last_img = img             # 影子模式网页直接画这张,不再重算
        return self.policy(img, scalars(self.speed, self.speed_cap, self.yaw_rate, self.enc))
