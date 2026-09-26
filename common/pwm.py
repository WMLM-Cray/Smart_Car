"""物理量与实车 PWM 刻度之间的换算层。

仿真内部一律用物理量(m/s, rad);只有和实车 /teleop_cmd_vel 对接时才过这一层。
本模块只保留实测转向映射。速度目标由 vehicle.speed_loop 控制，不使用旧线性 PWM 速度换算。

实车约定(racecar_teleop.py / web_teleop):linear.x = 电调 PWM, angular.z = 舵机值。
"""
from __future__ import annotations

import math

import numpy as np

from common.params import CarParams


def turn_to_steer(turn: float, p: CarParams = CarParams()) -> float:
    """舵机值 -> 等效前轮角(rad,左正),按实测 steer_curve 插值,两端钳住。"""
    servo, deg = zip(*p.steer_curve)
    return math.radians(float(np.interp(turn, servo, deg)))


def steer_to_turn_float(steer: float, p: CarParams = CarParams()) -> float:
    """前轮角(rad) -> 舵机值(不取整)。steer_curve 单调,反过来插值即可。"""
    servo, deg = zip(*p.steer_curve)
    return float(np.interp(math.degrees(steer), deg, servo))


def steer_to_turn(steer: float, p: CarParams = CarParams()) -> int:
    return round(steer_to_turn_float(steer, p))


def action_to_steer(a: float, p: CarParams = CarParams()) -> float:
    """策略转向输出(±1) -> 前轮角。车上发 turn_mid + a·turn_span,这里算它实际转多少。"""
    a = max(-1.0, min(1.0, a))
    return turn_to_steer(p.turn_mid + a * p.turn_span, p)


def steer_to_action(steer: float, p: CarParams = CarParams()) -> float:
    """action_to_steer 的反函数,老师用它把想要的转角换成标签。"""
    return max(-1.0, min(1.0, (steer_to_turn_float(steer, p) - p.turn_mid) / p.turn_span))
