"""实景地图闭环:车上建的图(RealMap)当场地,从实车那一段的起点重新开,复现实车上的失败。

和 RoadEnv 同一套车辆 / 速度 / 舵机 / 指令延迟 / 雷达噪声(step 不变),换掉的只有场地:
  雷达   在网格上打射线(RealMap.raycast)
  碰撞   车身矩形里有障碍格
  结束   没有"终点":开出建图时见过的范围('unknown')、碰撞、超时
地面分割(MapFloorSeg):按车上的像素投影点(GroundLut,同 FakeFloorSeg lut 模式),落在建图时摄像头见过的地面格上、
且相机到它的视线没被障碍格挡住才算地面。建图时没看过的地方没有地面 —— 车开到实车没去过的角落,这一层是空的。
"""
from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from common.params import CarParams
from simulation.fastsim.camera import FloorSegConfig
from simulation.fastsim.real_map import RealMap
from simulation.fastsim.road_env import STOP, Plant, RoadEnv, SpeedPlant
from simulation.fastsim.vehicle import Car


class RealMapEnv(RoadEnv):
    def __init__(self, world: RealMap, pl: Plant, seed: int, start: tuple, noise: bool = True,
                 car_params: CarParams = CarParams(), duration: float = 40.0):
        """start = (x, y, yaw) 后轴位姿(和车上里程计 / Logbook 的 path 同一个系)。"""
        self.world, self.pl, self.sense, self.road = world, pl, True, None
        self.rng = np.random.default_rng(seed)
        self.noise, self.robust = noise, False
        self.cp = replace(car_params, steer_rate=pl.steer_rate, speed_tau=1e-4, accel_max=1e3)
        self.car = Car(self.cp)
        self.car.reset(*start, steer_gain=pl.steer_gain)
        self.speed = SpeedPlant(pl)
        self.release = car_params.brake_release_speed
        self.t, self.next_scan = 0.0, 0.0
        first = STOP if pl.enc.dead else pl.enc.v_lo       # 同 RoadEnv:能停车的契约开局是中位
        self.pending = [(0.0, first)] * pl.lag
        self.cmd = (0.0, first)
        n = car_params.lidar_points
        self.angles = -math.pi + np.arange(n) * (2 * math.pi / n)
        self.min_clear = math.inf
        self.last = np.zeros(2, np.float32)
        self.budget_s = duration
        self.dist = 0.0
        p = self.cp                                         # 车身矩形里的采样点(车体系,2.5 cm 间距)
        bx, by = np.meshgrid(np.arange(-p.rear_overhang, p.length - p.rear_overhang + 1e-9, 0.025),
                             np.arange(-p.width / 2, p.width / 2 + 1e-9, 0.025), indexing='ij')
        self.body = np.stack([bx.ravel(), by.ravel()], 1)
        self._scan()

    def scan_at(self, x, y, yaw) -> np.ndarray:
        p = self.cp
        return self.world.raycast(x + p.lidar_x * math.cos(yaw), y + p.lidar_x * math.sin(yaw),
                                  self.angles + yaw + self.pl.lidar_yaw, p.lidar_max, 0.0)

    def _check(self) -> str | None:
        car = self.car
        self.dist += abs(car.v) * 0.01
        c, s = math.cos(car.yaw), math.sin(car.yaw)
        b = self.body
        hit, _ = self.world._lookup(self.world.occ, car.x + c * b[:, 0] - s * b[:, 1], car.y + s * b[:, 0] + c * b[:, 1])
        if hit.any():
            return 'collision'
        if not self.world.is_known(car.x, car.y):
            return 'unknown'
        if self.t > self.budget_s:
            return 'timeout'
        return None


class MapFloorSeg:
    """实景地图上的假地面分割,接口同 FakeFloorSeg.detect(返回车体系地面点)。"""

    def __init__(self, world: RealMap, params: CarParams = CarParams(),
                 cfg: FloorSegConfig = replace(FloorSegConfig(), lut=(72, 96), fill=True), seed: int = 0):
        from common.ground_proj import GroundLut
        self.world, self.p, self.cfg = world, params, cfg
        self.rng = np.random.default_rng(seed)
        self.lut = GroundLut(*cfg.lut, params, cfg.range)
        self.lut_idx = np.flatnonzero(self.lut.valid)
        cam = self.lut.xy[self.lut_idx] - (params.camera_x, 0.0)
        self.grid_body = cam + (params.camera_x, 0.0)
        self.grid_r = np.hypot(cam[:, 0], cam[:, 1])
        self.grid_a = np.arctan2(cam[:, 1], cam[:, 0])

    def detect(self, pose) -> np.ndarray:
        cfg, rng = self.cfg, self.rng
        if rng.random() < cfg.miss:
            return np.zeros((0, 2))
        from common.ground_fix import RANGE_UNSURE
        reach = cfg.range if rng.random() < cfg.sure else RANGE_UNSURE
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        b = self.grid_body
        w = np.stack([x + c * b[:, 0] - s * b[:, 1], y + s * b[:, 0] + c * b[:, 1]], axis=1)
        cx, cy = x + c * self.p.camera_x, y + s * self.p.camera_x
        hit = self.world.raycast(cx, cy, self.grid_a + yaw, cfg.range + 0.1)     # 相机视线被障碍格挡住的点看不到
        free = self.world.is_free(w[:, 0], w[:, 1]) & (self.grid_r < hit)
        if cfg.fill:
            full = np.zeros(self.lut.valid.size, bool)
            full[self.lut_idx] = free
            pts = np.vstack([b[free], self.lut.fill_rows(full.reshape(self.lut.shape))])
            return pts[np.hypot(pts[:, 0] - self.p.camera_x, pts[:, 1]) <= reach]
        return b[free & (self.grid_r <= reach)]
