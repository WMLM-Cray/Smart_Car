"""阿克曼车(运动学自行车模型 + 摩擦圆限幅,参考点=后轴中心)。

输入是物理量:目标车速 m/s 与目标前轮转角 rad(左正)。
与实车 PWM 的换算在 pwm.py,不进这一层。

纯运动学模型只在低速可信:1.5m/s 过 1m 半径的弯只有 0.23g,横向力不成约束;
到 3m/s 就是 0.92g,已经到摩擦极限。所以 step() 里按 a_lat_max 对横摆角速度限幅,
超出部分表现为推头(self.slip 记录超了多少),否则高速训练学到的是物理上做不到的动作。
"""
from __future__ import annotations

import math

import numpy as np

from .params import CarParams


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Car:
    def __init__(self, params: CarParams = CarParams()):
        self.p = params
        self.reset(0.0, 0.0, 0.0)

    def reset(self, x, y, yaw, v_gain: float = 1.0, steer_gain: float = 1.0):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.v = 0.0            # 纵向速度 m/s
        self.steer = 0.0        # 前轮转角 rad,左正
        self.yaw_rate = 0.0
        self.accel = 0.0        # 纵向加速度
        # 域随机化用:指令与实际的增益差(电池电压、舵机行程不准)
        self.v_gain, self.steer_gain = float(v_gain), float(steer_gain)

    def step(self, v_cmd: float, steer_cmd: float, dt: float):
        p = self.p
        target_v = max(-p.v_rev_full, min(p.v_fwd_full, v_cmd)) * self.v_gain
        dv = (target_v - self.v) * min(1.0, dt / p.speed_tau)
        dv = max(-p.accel_max * dt, min(p.accel_max * dt, dv))
        self.accel = dv / dt
        self.v += dv

        target_steer = max(-p.max_steer, min(p.max_steer, steer_cmd)) * self.steer_gain
        ds = max(-p.steer_rate * dt, min(p.steer_rate * dt, target_steer - self.steer))
        self.steer += ds

        # 摩擦圆:指令转角要求的横向加速度 a = v²·tan(δ)/L,超过 a_lat_max 就抓不住,
        # 实际横摆角速度按比例打折 —— 车听话地少转,也就是推头。纵向也占用抓地力,
        # 所以可用的横向预算先扣掉纵向那一份(简化的摩擦圆,不做轮胎滑移)。
        self.slip = 0.0
        yaw_rate = self.v * math.tan(self.steer) / p.wheelbase
        a_lat = abs(self.v * yaw_rate)
        budget = math.sqrt(max(p.a_lat_max ** 2 - min(self.accel ** 2, p.a_lat_max ** 2), 0.0))
        if a_lat > budget > 0.0:
            self.slip = 1.0 - budget / a_lat        # 0=抓得住,接近 1=完全打滑
            yaw_rate *= budget / a_lat
        self.yaw_rate = yaw_rate
        yaw_mid = self.yaw + 0.5 * self.yaw_rate * dt   # 中点积分
        self.x += self.v * math.cos(yaw_mid) * dt
        self.y += self.v * math.sin(yaw_mid) * dt
        self.yaw = wrap_pi(self.yaw + self.yaw_rate * dt)

    def footprint_hits_circles(self, centres_xy: np.ndarray, radius: float) -> np.ndarray:
        """车体矩形与圆(锥桶底面)是否相交,逐个返回 bool。"""
        p = self.p
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        rel = np.asarray(centres_xy, float).reshape((-1, 2)) - (self.x, self.y)
        lx = rel[:, 0] * c + rel[:, 1] * s            # 车体系,x 前
        ly = -rel[:, 0] * s + rel[:, 1] * c
        x_lo, x_hi = -p.rear_overhang, p.length - p.rear_overhang
        dx = np.maximum(np.maximum(x_lo - lx, 0.0), lx - x_hi)
        dy = np.maximum(np.abs(ly) - p.width / 2, 0.0)
        return np.hypot(dx, dy) < radius
