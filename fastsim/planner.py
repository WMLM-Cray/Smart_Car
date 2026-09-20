"""会规划速度的参考控制器 —— 读赛道真值,只当 DAgger 的老师用,策略不能用。

为什么需要它:原来的老师是纯追踪 + 恒定满油门,在 1.5m/s 下恰好最优(实测 s3
满速 20/20 零撞锥,那条赛道在那个速度下根本不需要减速)。但加上摩擦圆之后,
3m/s 过 1m 半径的弯就是 0.92g,已经超出抓地力,恒速老师会直接推头出界。

算法是赛车工程里的准稳态速度规划(和 work/speed_limit.py 同一套):
  1. 中心线每点曲率 k -> 过弯极限 v = sqrt(a_lat / |k|)
  2. 前向扫一遍,受加速度上限约束
  3. 反向扫一遍,受制动上限约束(决定入弯前多早开始收油门)
转向仍用纯追踪,但预瞄距离随速度增长 —— 固定 1.2m 在 3m/s 下只有 0.4 秒的预瞄,
来不及反应。

注意:制动能力用的是仿真的 accel_max。实车电调松油门不主动刹车,只靠阻力滑行,
真实减速度大概率比这个小,入弯点要更早。这个数必须实测,否则仿真会高估过弯能力。
"""
from __future__ import annotations

import math

import numpy as np


def curvature(pts: np.ndarray) -> np.ndarray:
    """闭合折线每点曲率,取相邻三点外接圆半径的倒数。"""
    a, b, c = np.roll(pts, 1, 0), pts, np.roll(pts, -1, 0)
    ab, bc, ca = np.hypot(*(b - a).T), np.hypot(*(c - b).T), np.hypot(*(a - c).T)
    cross = np.abs((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1])
                   - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0]))
    with np.errstate(divide='ignore', invalid='ignore'):
        return np.where(ab * bc * ca > 1e-12, 2 * cross / (ab * bc * ca), 0.0)


def speed_profile(pts: np.ndarray, a_lat: float, a_acc: float, a_brk: float,
                  v_cap: float) -> tuple[np.ndarray, np.ndarray]:
    """返回(每点目标速度, 每点到下一点的弧长)。"""
    ds = np.hypot(*np.diff(np.vstack([pts, pts[:1]]), axis=0).T)
    k = curvature(pts)
    v = np.minimum(v_cap, np.where(k > 1e-6, np.sqrt(a_lat / np.maximum(k, 1e-9)), v_cap))
    n = len(v)
    for _ in range(2):                       # 绕两圈让首尾衔接收敛
        for i in range(n):
            j = (i + 1) % n
            v[j] = min(v[j], math.sqrt(max(v[i] ** 2 + 2 * a_acc * ds[i], 0.0)))
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v[i] = min(v[i], math.sqrt(max(v[j] ** 2 + 2 * a_brk * ds[i], 0.0)))
    return v, ds


class SpeedPlanTeacher:
    """policy(obs, env) -> 动作。读赛道真值,只能当老师。

    margin < 1 是留给学生的抓地力余量:老师贴着极限开的话,学生那点跟踪误差
    就会直接变成推头。0.85 表示老师只用 85% 的横向加速度预算。
    """

    def __init__(self, margin: float = 0.85, lookahead_time: float = 0.5,
                 lookahead_min: float = 1.2, lookahead_max: float = 3.0):
        self.margin = margin
        self.lookahead_time = lookahead_time
        self.lookahead_min, self.lookahead_max = lookahead_min, lookahead_max
        self._cache_key = None

    def reset(self):
        self._cache_key = None

    def _profile(self, env):
        key = id(env.track)
        if key != self._cache_key:
            p = env.cp
            self.v_ref, ds = speed_profile(
                env.track.centerline, a_lat=p.a_lat_max * self.margin,
                a_acc=p.accel_max, a_brk=p.accel_max, v_cap=env.v_cmd_fwd)
            self.cum = np.concatenate([[0.0], np.cumsum(ds)])
            self._cache_key = key
        return self.v_ref

    def __call__(self, obs, env) -> np.ndarray:
        v_ref = self._profile(env)
        car, track = env.car, env.track
        s, _, _ = track.project((car.x, car.y))
        total = self.cum[-1]
        # 目标速度取当前位置到一个制动距离之内的最小值,提前收油门
        ahead = max(0.5, car.v ** 2 / (2 * env.cp.accel_max))
        lo = np.searchsorted(self.cum, s % total)
        hi = lo + max(1, int(ahead / max(total / len(v_ref), 1e-6)))
        idx = np.arange(lo, hi + 1) % len(v_ref)
        v_target = float(v_ref[idx].min())

        # 预瞄距离跟着"规划速度"走,不跟实际车速 —— 后者会让老师的舵角依赖车辆状态,
        # 同一个位置以不同速度经过时给出不同的舵角,学生很难还原这个耦合。
        # 2026-09-19 实测:变预瞄的转角拟合误差比固定预瞄高 60%(0.00485 vs 0.00301)。
        # 用规划速度之后老师的动作只是位置和航向的函数,既保留了随速度伸长的好处,
        # 又不引入状态耦合。
        lookahead = float(np.clip(self.lookahead_time * float(v_ref[lo % len(v_ref)]),
                                  self.lookahead_min, self.lookahead_max))
        pts = track.centerline
        target = pts[np.searchsorted(self.cum, (s + lookahead) % total) % len(pts)]
        alpha = math.atan2(target[1] - car.y, target[0] - car.x) - car.yaw
        steer = math.atan2(2 * env.cp.wheelbase * math.sin(alpha), lookahead)
        return np.array([np.clip(v_target / env.v_cmd_fwd, -1.0, 1.0),
                         np.clip(steer / env.cp.max_steer, -1.0, 1.0)], np.float32)
