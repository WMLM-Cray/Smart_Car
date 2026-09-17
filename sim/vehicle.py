"""阿克曼车(运动学自行车模型,参考点=后轴中心)。

输入是物理量:目标车速 m/s 与目标前轮转角 rad(左正)。
与实车 PWM 的换算在 pwm.py,不进这一层。
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

        self.yaw_rate = self.v * math.tan(self.steer) / p.wheelbase
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

    def footprint_hits_squares(self, centres_xy: np.ndarray, half: float) -> np.ndarray:
        """车体矩形与锥桶方形基座(边长 2·half,世界系轴对齐)是否相交。

        2026-09-17 审查修正:官方锥底座是 0.28×0.28m 方形;原先 footprint_hits_
        circles 用内切圆 r=0.14 是**乐观近似**——方形对角伸出圆外 (√2−1)·half
        ≈0.058m,角对角擦碰会漏判。本判定用分离轴(SAT):车 OBB 的两条轴 +
        方形的两条轴,任一轴上投影分离即不相交。
        朝向假设:实际摆锥的底座朝向未记录,这里取世界系轴对齐;朝向偏差最多
        45° 时方形投影宽度在 [0.28, 0.396] 之间,方向可正可负,如实披露。
        粗筛:先用外接圆半径 half·√2 调 circles 取超集,再对候选做精确 SAT。
        """
        centres = np.asarray(centres_xy, float).reshape((-1, 2))
        near = self.footprint_hits_circles(centres, half * 1.4143)   # 外接圆粗筛(超集)
        hits = np.zeros(len(centres), bool)
        idx = np.flatnonzero(near)
        if not len(idx):
            return hits
        p = self.p
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        # 车体四角(世界系),顺序绕矩形一圈
        lx = np.array([-p.rear_overhang, p.length - p.rear_overhang,
                       p.length - p.rear_overhang, -p.rear_overhang])
        ly = np.array([-p.width / 2, -p.width / 2, p.width / 2, p.width / 2])
        car_pts = np.stack([self.x + lx * c - ly * s, self.y + lx * s + ly * c], axis=1)
        axes = np.array([[c, s], [-s, c], [1.0, 0.0], [0.0, 1.0]])   # 车两轴 + 世界两轴
        off = np.array([[-half, -half], [half, -half], [half, half], [-half, half]])
        for i in idx:
            cone_pts = centres[i] + off
            separated = False
            for ax in axes:
                pc = car_pts @ ax
                pk = cone_pts @ ax
                if pc.max() < pk.min() or pk.max() < pc.min():
                    separated = True
                    break
            if not separated:
                hits[i] = True
        return hits
