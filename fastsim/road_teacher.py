"""老师:看得到整条路,先规划一条车真能开的路线,再跟踪它。

规划  在路面内求一条"曲率平方和最小"的路线(横向偏移 d(s) 的有界最小二乘),
      直角弯自然变成外-内-外。车宽和余量从可用宽度里先扣掉。
      曲率上限按实测最小转弯半径 0.83~0.88m 再留 10%:R >= 0.95m。
速度  按曲率限横向加速度,再倒推滑行减速(电调不刹车)、正推加速,
      上限取 min(速度上限, 这块电池能跑到的速度)。
跟踪  纯追踪,按指令延迟往前推一点位姿;转角经实测舵机曲线反查成动作。
      老师知道这一局的真实车辆参数(舵机增益、中位偏差),这是它的特权。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import lsq_linear

from .pwm import steer_to_turn_float
from .road import CONE, NONE, WALL, Road
from .road_env import CONTROL_DT, RoadEnv
from .road_policy import speed_to_action

R_MIN = 0.95                # 规划用最小转弯半径 m
# 车身两侧到边界至少留这么多(规划时)。锥桶立在边界线上,半径 0.14 要额外扣;
# 再加 0.05 给入弯时车头外甩(车头在后轴前 0.43m)。
SIDE_MARGIN = {NONE: 0.10, WALL: 0.15, CONE: 0.14 + 0.15}
STEP = 5                    # 每隔几个中心线点取一个规划点(0.25m)
A_LAT = 0.6                 # 老师过弯的横向加速度上限 m/s²
A_DEC = 0.6                 # 计划减速度,比实测滑行 0.68~0.78 保守
A_ACC = 0.4


@dataclass
class Plan:
    pts: np.ndarray         # (m,2) 后轴要走的路线
    s: np.ndarray           # (m,)
    kappa: np.ndarray       # (m,) 有符号曲率
    v: np.ndarray           # (m,) 计划车速
    bound: np.ndarray       # (m,) 每点允许的最大 |d|
    d: np.ndarray           # (m,) 求出的横向偏移


def curvature(pts: np.ndarray, gap: int = 3) -> np.ndarray:
    """隔 gap 个点取三点外接圆,有符号(左正)。gap=3 即 0.3m 基线,压掉离散噪声。"""
    n = len(pts)
    k = np.zeros(n)
    if n < 2 * gap + 1:
        return k
    a, b, c = pts[:-2 * gap], pts[gap:-gap], pts[2 * gap:]
    ab, bc, ca = b - a, c - b, a - c
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    den = np.hypot(*ab.T) * np.hypot(*bc.T) * np.hypot(*ca.T)
    k[gap:-gap] = 2 * cross / np.maximum(den, 1e-12)
    k[:gap], k[-gap:] = k[gap], k[-gap - 1]
    return k


def plan(road: Road, i0: int, d0: float, car_width: float, pl) -> Plan | str:
    """返回 Plan,或者失败原因字符串('too_narrow' / 'curvature' / 'plan_error')。

    在 0.25m 一个的粗网格上解,再三次样条插回 5cm。粗点的边界取它前后半格内
    最紧的那个,插值后才不会在尖内角处越界。
    """
    margin = np.vectorize(SIDE_MARGIN.get)
    hi_all = road.half[i0:] - car_width / 2 - margin(road.mat_left[i0:])
    lo_all = -(road.half[i0:] - car_width / 2 - margin(road.mat_right[i0:]))
    idx = np.arange(0, len(hi_all), STEP)
    if idx[-1] != len(hi_all) - 1:
        idx = np.r_[idx, len(hi_all) - 1]
    half = STEP // 2 + 1
    hi = np.array([hi_all[max(0, k - half):k + half + 1].min() for k in idx])
    lo = np.array([lo_all[max(0, k - half):k + half + 1].max() for k in idx])
    if np.any(hi[1:] < lo[1:]):
        return 'too_narrow'
    c = road.centre[i0 + idx]
    h = road.heading[i0 + idx]
    n = np.stack([-np.sin(h), np.cos(h)], axis=1)
    m = len(idx)
    lo[0], hi[0] = d0 - 1e-6, d0 + 1e-6             # 从车现在的位置出发
    # 目标:sum w_i |p_{i-1} - 2p_i + p_{i+1}|²,p = c + d·n,对 d 线性
    rhs0 = c[:-2] - 2 * c[1:-1] + c[2:]
    w = np.ones(m - 2)
    rows = np.arange(m - 2)
    for _ in range(4):
        A = np.zeros((2 * (m - 2), m))
        for ax in range(2):
            r = 2 * rows + ax
            A[r, rows] = w * n[:-2, ax]
            A[r, rows + 1] = -2 * w * n[1:-1, ax]
            A[r, rows + 2] = w * n[2:, ax]
        b = -(rhs0 * w[:, None]).reshape(-1)
        try:
            d = lsq_linear(A, b, bounds=(lo, hi), method='bvls').x
        except np.linalg.LinAlgError:
            return 'plan_error'
        kap = curvature(c + d[:, None] * n, gap=1)
        over = np.abs(kap[1:-1]) * R_MIN
        if over.max() <= 0.95:
            break
        w = w * (1 + 4 * np.maximum(0, over - 0.8) ** 2)      # 超限处加权,把弯摊开
    # 插回中心线的 5cm 采样
    dense = CubicSpline(idx, d)(np.arange(len(hi_all)))
    cd = road.centre[i0:]
    hd = road.heading[i0:]
    pts = cd + dense[:, None] * np.stack([-np.sin(hd), np.cos(hd)], axis=1)
    kap = curvature(pts, gap=6)
    if np.max(np.abs(kap)) > 1 / R_MIN:
        return 'curvature'
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    v_top = min(pl.cap, pl.v_top)
    v = np.minimum(v_top, np.sqrt(A_LAT / np.maximum(np.abs(kap), 1e-6)))
    for i in range(len(v) - 2, -1, -1):     # 倒推:前面要慢,这里就得提前松油
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * A_DEC * (s[i + 1] - s[i])))
    return Plan(pts, s, kap, v, np.minimum(hi_all, -lo_all), dense)


class Teacher:
    def __init__(self, env: RoadEnv, pl_: Plan):
        self.env, self.plan = env, pl_
        self.j = 0

    def __call__(self) -> np.ndarray:
        env, P = self.env, self.plan
        car, pl, cp = env.car, env.pl, env.cp
        # 按指令延迟往前推位姿:这一拍的动作要 lag 拍后才生效
        t = CONTROL_DT * (pl.lag + 0.5)
        v = max(car.v, 0.3)
        yaw = car.yaw + car.yaw_rate * t
        x = car.x + v * math.cos(yaw) * t
        y = car.y + v * math.sin(yaw) * t
        lo, hi = max(0, self.j - 20), min(len(P.pts), self.j + 60)
        dd = P.pts[lo:hi] - (x, y)
        self.j = lo + int(np.argmin(np.einsum('ij,ij->i', dd, dd)))
        look = 0.35 + 0.3 * v
        k = min(len(P.pts) - 1, int(np.searchsorted(P.s, P.s[self.j] + look)))
        tx, ty = P.pts[k]
        alpha = math.atan2(ty - y, tx - x) - yaw
        ld = max(math.hypot(tx - x, ty - y), 0.2)
        delta = math.atan2(2 * cp.wheelbase * math.sin(alpha), ld) / pl.steer_gain
        servo = steer_to_turn_float(delta, cp) - pl.servo_offset
        steer_a = max(-1.0, min(1.0, (servo - cp.turn_mid) / cp.turn_span))
        # 速度看 (纯滞后 + 一拍) 之后要到的地方
        ahead = P.s[self.j] + v * (pl.dead_time + CONTROL_DT * (pl.lag + 1))
        kv = min(len(P.v) - 1, int(np.searchsorted(P.s, ahead)))
        return np.array([steer_a, speed_to_action(float(P.v[kv]))], np.float32)
