"""bev_v1 的运行时:和 RoadRuntime 同一套使能心跳、传感器过期、限速和输出,只换"大脑"的输入。

车上没有真位姿:编码器车速 + 陀螺偏航角速度用 PoseIntegrator 推 odom 位姿(用户实测雷达
里程计漂移很小,BEV 证据 20 s 过期,先不做扫描匹配)。每帧雷达按**收到那一刻**的位姿登记进
BEV;每拍按当前位姿出 5×80×80 图(本帧雷达按扫描位姿投影,和 training/bev_env.py 一致)。

摄像头:车上还没有 YOLO 节点,锥桶/胶带/区域三层是空的 —— 和训练时(FastSim 假 YOLO)不同,
上车前要先在仿真里关掉假 YOLO 验证过。接上 YOLO 后在 on_detections() 里登记。
"""
from __future__ import annotations

import numpy as np

from common.bev import Bev, PoseIntegrator
from common.bev_policy import bev_image, scalars
from common.road_runtime import RoadRuntime

MAX_DT = 0.2            # 两次积分间隔超过这个就当传感器断过,不积分(停车/过期时 RoadRuntime 已回中位)


class BevRuntime(RoadRuntime):

    def reset(self):
        self.bev = Bev(params=self.params)
        self.odom = PoseIntegrator()
        self.pose_t = None
        self.scan_pose = (0.0, 0.0, 0.0)
        super().reset()

    def _integrate(self, now):
        now = float(now)
        if self.pose_t is not None and 0.0 < now - self.pose_t <= MAX_DT:
            self.odom.step(self.speed, self.yaw_rate, now - self.pose_t)
        if self.pose_t is None or now > self.pose_t:
            self.pose_t = now

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

    def _reset_history(self):
        # 重新使能 / 传感器过期后从空 BEV 开始:停过车,旧证据和积分起点都不可信
        super()._reset_history()
        self.bev.reset()

    def _act(self, now):
        self._integrate(now)
        img = bev_image(self.bev, self.odom.pose, float(now), self.scan, self.angles, self.scan_pose)
        return self.policy(img, scalars(self.speed, self.speed_cap, self.yaw_rate, self.enc))
