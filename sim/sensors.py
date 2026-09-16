"""传感器模型,输出格式对齐实车话题。

/scan      LaserScan:angle_min=-π,increment=2π/1667,1667 个点,frame laser_link
/IMU_data  Imu:只给 yaw 角速度与平面加速度(z 含重力)
/encoder   Odometry.twist.linear.x:车尾从动轮测的纵向速度
"""
from __future__ import annotations

import math

import numpy as np

from .params import CarParams, TrackParams
from .raycast import cast_cones
from .vehicle import Car


class Lidar:
    def __init__(self, params: CarParams):
        self.p = params
        n = params.lidar_points
        self.angles = -math.pi + np.arange(n) * (2 * math.pi / n)
        self.angle_increment = 2 * math.pi / n

    def scan(self, car: Car, cones_xy: np.ndarray, track: TrackParams,
             rng: np.random.Generator | None) -> np.ndarray:
        p = self.p
        c, s = math.cos(car.yaw), math.sin(car.yaw)
        origin = np.array([car.x + p.lidar_x * c, car.y + p.lidar_x * s, p.lidar_z])
        world = self.angles + car.yaw
        dirs = np.stack([np.cos(world), np.sin(world), np.zeros_like(world)], axis=1)
        r = cast_cones(origin, dirs, cones_xy, track.cone_base_radius,
                       track.cone_top_radius, track.cone_height, p.lidar_max)
        if rng is not None:
            r = r + rng.normal(0, p.lidar_sigma, r.shape)
        return np.where((r >= p.lidar_min) & (r <= p.lidar_max), r, np.inf)


def imu(car: Car, params: CarParams, rng: np.random.Generator | None) -> dict:
    gz, ax, ay = car.yaw_rate, car.accel, car.v * car.yaw_rate
    if rng is not None:
        gz += rng.normal(0, params.gyro_sigma)
        ax += rng.normal(0, params.accel_sigma)
        ay += rng.normal(0, params.accel_sigma)
    return dict(angular_velocity_z=gz, linear_acceleration=(ax, ay, 9.80665), yaw=car.yaw)


def encoder(car: Car, params: CarParams, rng: np.random.Generator | None) -> float:
    return car.v + (rng.normal(0, params.encoder_sigma) if rng is not None else 0.0)
