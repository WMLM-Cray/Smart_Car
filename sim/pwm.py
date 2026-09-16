"""物理量与实车 PWM 刻度之间的换算层。

仿真内部一律用物理量(m/s, rad);只有和实车 /teleop_cmd_vel 对接时才过这一层。
车速曲线实测出来以后只改这里和 params.py,策略与环境不用动。

实车约定(racecar_teleop.py / web_teleop):linear.x = 电调 PWM, angular.z = 舵机值。
"""
from __future__ import annotations

from .params import CarParams


def pwm_to_speed(pwm: float, p: CarParams = CarParams()) -> float:
    """PWM -> 稳态车速。(rev_start, fwd_start) 之间是电调死区,不转。"""
    if pwm >= p.pwm_fwd_start:
        frac = (min(pwm, p.pwm_fwd_full) - p.pwm_fwd_start) / (p.pwm_fwd_full - p.pwm_fwd_start)
        return frac * p.v_fwd_full
    if pwm <= p.pwm_rev_start:
        frac = (p.pwm_rev_start - max(pwm, p.pwm_rev_full)) / (p.pwm_rev_start - p.pwm_rev_full)
        return -frac * p.v_rev_full
    return 0.0


def speed_to_pwm(v: float, p: CarParams = CarParams()) -> int:
    """车速 -> PWM。死区让车跑不出 0 与起转速度之间的值,故 |v| 很小时给中位。"""
    if v > 1e-6:
        frac = min(v, p.v_fwd_full) / p.v_fwd_full
        return round(p.pwm_fwd_start + frac * (p.pwm_fwd_full - p.pwm_fwd_start))
    if v < -1e-6:
        frac = min(-v, p.v_rev_full) / p.v_rev_full
        return round(p.pwm_rev_start - frac * (p.pwm_rev_start - p.pwm_rev_full))
    return p.pwm_mid


def steer_to_turn(steer: float, p: CarParams = CarParams()) -> int:
    """前轮转角(rad,左正) -> 舵机值。84 中位,144 左满舵。"""
    frac = max(-1.0, min(1.0, steer / p.max_steer))
    return round(p.turn_mid + frac * p.turn_span)


def turn_to_steer(turn: float, p: CarParams = CarParams()) -> float:
    frac = max(-1.0, min(1.0, (turn - p.turn_mid) / p.turn_span))
    return frac * p.max_steer
