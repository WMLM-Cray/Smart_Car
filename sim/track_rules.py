"""按比赛规则拼出来的训练赛道:直道 + 45°/90°/135° 弯,双边锥桶,随机蓝挡板。

和 track.py 的随机环形赛道不同 —— 那个是任意曲率的光滑闭环,实际比赛不长那样。
比赛当天从 A/B/C/D 四套模块化赛道里随机抽一套(培训笔记 05:49),四套都由同一批
元素拼成,所以这里按元素随机拼闭环,训练出来的策略对哪套都见过同类几何。

规则依据(标注同 params.py):
  [规则] 赛道宽 1.5m,局部收窄 1.25m;转角只有 45°/90°/135°;总横向尺寸约 16.8m
         —— 队友 docs/SOURCE_AUDIT.md 转述规则图 9(第 11-12 页)
  [规则] 圆形赛道内径 1.25m / 外径 2.75m -> 中心线转弯半径 2.0m(培训笔记 11:06)
  [规则] 5 块随机放置的蓝色挡板,裁判靠地图里挡板位置验证建图真实(培训笔记 06:45)
  [假设] 锥桶间距、直道长度范围、挡板具体尺寸 —— 规则原文还没拿到,先给合理值

闭环怎么保证:先抽一串转角(和必须 ±360°),直道长度留作未知数,
末端回到起点这两个方程对长度是线性的,用最小范数修正把长度解出来(_close_loop)。
"""
from __future__ import annotations

import math

import numpy as np

from .params import RuleTrackParams
from .track import Track, _resample_closed


def _turn_sequence(rng: np.random.Generator, p: RuleTrackParams) -> np.ndarray:
    """抽一串转角(度,左正),和为 +360;左转为主,允许少量右转形成 S 弯。"""
    for _ in range(400):
        n = int(rng.integers(p.min_turns, p.max_turns + 1))
        angles = rng.choice(p.turn_angles, size=n) * np.where(
            rng.random(n) < p.right_turn_ratio, -1.0, 1.0)
        missing = 360.0 - angles.sum()
        if missing == 0:
            return angles
        # 差多少就用一个合法转角补上(可能要补几个)
        for a in sorted(p.turn_angles, reverse=True):
            while missing >= a and len(angles) < p.max_turns:
                angles = np.append(angles, a)
                missing -= a
        if missing == 0:
            return angles
    raise RuntimeError("凑不出和为 360° 的转角序列")


def _close_loop(headings: np.ndarray, arc_shift: np.ndarray, lengths: np.ndarray,
                min_len: float) -> np.ndarray:
    """调整直道长度让首尾闭合:解 A·L = b(A 是各直道方向,b 是弯道位移的相反数)。

    交替投影:先最小范数修正 L += Aᵀ(AAᵀ)⁻¹(b - A·L) 投到闭合面上,
    再把越界的长度截回 [min_len, max_len],反复直到两个条件同时满足。
    """
    A = np.stack([np.cos(headings), np.sin(headings)])          # (2,n)
    b = -arc_shift
    for _ in range(200):
        residual = b - A @ lengths
        lengths = lengths + A.T @ np.linalg.solve(A @ A.T, residual)
        if lengths.min() >= min_len:
            break
        lengths = np.maximum(lengths, min_len)
    if np.linalg.norm(b - A @ lengths) > 1e-6 or lengths.min() < min_len:
        raise ValueError("这组转角配不出合法直道长度")
    return lengths


def _centerline(rng: np.random.Generator, p: RuleTrackParams) -> np.ndarray:
    turns = _turn_sequence(rng, p)
    n = len(turns)
    headings = np.radians(np.concatenate([[0.0], np.cumsum(turns)[:-1]]))

    # 每个弯的位移(在自身入弯朝向下)与直道无关,先算出来
    shift = np.zeros(2)
    yaw = 0.0
    for turn in turns:
        a = math.radians(turn)
        sign = 1.0 if a >= 0 else -1.0
        # 圆心在左(左转)或右(右转),入弯点到出弯点的弦
        local = np.array([p.turn_radius * math.sin(abs(a)),
                          sign * p.turn_radius * (1 - math.cos(a))])
        c, s = math.cos(yaw), math.sin(yaw)
        shift += np.array([c * local[0] - s * local[1], s * local[0] + c * local[1]])
        yaw += a

    lengths = _close_loop(headings, shift,
                          rng.uniform(p.straight_min, p.straight_max, n), p.straight_min)

    pts, pos, yaw = [], np.zeros(2), 0.0
    for length, turn in zip(lengths, turns):
        step = max(2, int(length / 0.05))
        d = np.array([math.cos(yaw), math.sin(yaw)])
        pts.append(pos + np.linspace(0, length, step, endpoint=False)[:, None] * d)
        pos = pos + length * d
        a = math.radians(turn)
        sign = 1.0 if a >= 0 else -1.0
        centre = pos + sign * p.turn_radius * np.array([-math.sin(yaw), math.cos(yaw)])
        phi0 = math.atan2(*(pos - centre)[::-1])
        # 弯道采样要比后面 5cm 的重采样密,否则重采样点落在折线顶点上,转角会被切碎
        phi = phi0 + np.linspace(0, a, max(2, int(p.turn_radius * abs(a) / 0.02)), endpoint=False)
        pts.append(centre + p.turn_radius * np.stack([np.cos(phi), np.sin(phi)], axis=1))
        pos = centre + p.turn_radius * np.array([math.cos(phi0 + a), math.sin(phi0 + a)])
        yaw += a
    return np.vstack(pts)


def _boundaries(centre: np.ndarray, p: RuleTrackParams):
    tangent = np.roll(centre, -1, axis=0) - np.roll(centre, 1, axis=0)
    tangent /= np.hypot(*tangent.T)[:, None]
    normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)
    half = np.full(len(centre), p.width / 2)
    if p.narrow_fraction > 0:                       # [规则] 局部收窄到 1.25m
        start = int(len(centre) * p.narrow_start)
        span = int(len(centre) * p.narrow_fraction)
        half[start:start + span] = p.narrow_width / 2
    return centre + half[:, None] * normal, centre - half[:, None] * normal


def _place_boards(rng: np.random.Generator, left: np.ndarray, right: np.ndarray,
                  p: RuleTrackParams) -> np.ndarray:
    """在边界相邻两锥桶之间放挡板,同一条边界上不挨着放。"""
    slots = [(side, i) for side, pts in enumerate((left, right)) for i in range(len(pts))]
    rng.shuffle(slots)
    boards, used = [], set()
    for side, i in slots:
        pts = (left, right)[side]
        j = (i + 1) % len(pts)
        if (side, i) in used or (side, j) in used or len(boards) >= p.n_boards:
            continue
        a, b = pts[i], pts[j]
        mid, d = (a + b) / 2, b - a
        d = d / np.hypot(*d)
        boards.append([mid - d * p.board_length / 2, mid + d * p.board_length / 2])
        used |= {(side, i), (side, j)}
    if len(boards) < p.n_boards:
        raise ValueError("锥桶太少,放不下要求数量的挡板")
    return np.array(boards)


def make_rule_track(rng: np.random.Generator, p: RuleTrackParams = RuleTrackParams(),
                    attempts: int = 100) -> Track:
    for _ in range(attempts):
        try:
            centre = _resample_closed(_centerline(rng, p), 0.05)
        except (ValueError, RuntimeError):
            continue
        if max(np.ptp(centre[:, 0]), np.ptp(centre[:, 1])) > p.max_extent:
            continue                                 # [规则] 场地总横向尺寸有限
        centre = centre - (centre.max(axis=0) + centre.min(axis=0)) / 2    # 摆到原点附近,别跑出建图栅格
        inner, outer = _boundaries(centre, p)
        # 自交检查:沿赛道隔得远的两点,几何上不能靠得比赛道宽还近(否则两段路挤在一起)
        sample = centre[::10]
        d = np.hypot(*(sample[:, None, :] - sample[None, :, :]).transpose(2, 0, 1))
        idx = np.arange(len(d))
        gap = np.abs(idx[:, None] - idx[None])
        far = np.minimum(gap, len(d) - gap) > len(d) // 8      # 闭环,索引距离要绕回去算
        if (d[far] < p.width).any():
            continue
        left = _resample_closed(inner, p.cone_spacing)
        right = _resample_closed(outer, p.cone_spacing)
        track = Track(centre, left, right, p, _place_boards(rng, left, right, p))
        return track
    raise RuntimeError("按规则元素拼不出闭环赛道,放宽 max_extent 或转角/直道范围")
