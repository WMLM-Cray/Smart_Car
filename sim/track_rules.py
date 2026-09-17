"""按比赛规则拼出来的训练赛道:直道 + 45°/90°/135° 弯,双边锥桶,随机蓝挡板。

和 track.py 的随机环形赛道不同 —— 那个是任意曲率的光滑闭环,实际比赛不长那样。
比赛当天从 A/B/C/D 四套模块化赛道里随机抽一套(培训笔记 05:49),四套都由同一批
元素拼成,所以这里按元素随机拼闭环,训练出来的策略对哪套都见过同类几何。

形状照着官方赛道示意图(sim/reference_track_figure.png,规则图 1):长条形,
一条大直道,两端各一个 180° 掉头大弯,回程侧连续小 S 弯,可能含直角区。

规则依据(标注同 params.py):
  [规则] 赛道宽 1.5m;含直线、S 弯、圆弯、直角区(规则第 2-6 页)
  [规则] 转角 45°/90°/135°,总横向尺寸 16.8m —— 队友转述规则图 9
  [规则] 圆形赛道内径 1.25m / 外径 2.75m(培训笔记 11:06),与 1.5m 宽一致
  [规则] 5 块随机放置的蓝色挡板,裁判靠地图里挡板位置验证建图真实(培训笔记 06:45)
  [图1]  锥桶间距 ≈ 赛道宽/1.32 ≈ 1.15m;S 弯比圆弯紧;短边远小于长边
  [假设] 直道长度范围、挡板尺寸、端部大弯半径 —— 规则原文没拿到,先给合理值

闭环怎么保证:两端 180° 同半径同转向抵消横向位移,S 弯成镜像对出现
((+a,-a) 配 (-a,+a))也互相抵消;剩下的直道长度对闭合方程是线性的,
用交替投影解出满足长度下限的那组解(_close_loop)。
"""
from __future__ import annotations

import math

import numpy as np

from .params import RuleTrackParams
from .track import Track, _resample_closed


def _end(rng: np.random.Generator, p: RuleTrackParams):
    """端部掉头:方角(两个直角夹一段横直道)或大圆弯。元素 = (转角°, 半径, 前置直道 lo, hi)。

    比赛方常把端部做成方角(赛道图上 ±7.5m 处那种直角弯),所以默认多给方角。
    方角中间那段横直道同时是闭环方程里唯一能直接改横向的自由量。
    """
    r = rng.uniform(p.bend_radius_min, p.bend_radius_max)
    if rng.random() < p.square_end_chance:
        return [(90.0, r, p.straight_min, p.straight_max),
                (90.0, r, p.width, p.max_short_side)]        # 中间横过去的一段
    return [(180.0, rng.uniform(p.end_radius_min, p.end_radius_max), p.straight_min, p.straight_max)]


def _layout(rng: np.random.Generator, p: RuleTrackParams):
    """按规则图 1 的长条形布局排元素,返回 [(转角°, 半径), ...] 与每段直道的长度上下限。

    顺序:主直道 -> 端部掉头 -> 回程侧若干矩形折角 -> 另一端掉头 -> 回到主直道起点。
    转角和 = 180+180 + 折角正负抵消 = 360°,闭环成立;直道长度由 _close_loop 解。
    """
    # 折角成镜像对出现((+a,-a) 配 (-a,+a)):每对结束后航向都回到回程方向,横向漂移抵消。
    # 90° 一对就是一个矩形台阶,赛道看着方正;45° 用来出斜切段。
    jogs = []
    for _ in range(int(rng.integers(p.s_bends_min, p.s_bends_max + 1))):
        a = 90.0 if rng.random() < p.right_angle_ratio else float(rng.choice([45.0, 135.0]))
        r = rng.uniform(p.bend_radius_min, p.bend_radius_max)   # 同一组用同一半径,漂移才抵消
        step = (p.straight_min, p.straight_max) if a != 90.0 else (p.jog_min, p.jog_max)
        jogs += [((a, -a), r, step), ((-a, a), r, step)]
    rng.shuffle(jogs)

    elems = list(_end(rng, p))                     # 跑完主直道,端部掉头
    for (a, b), r, step in jogs:
        elems.append((a, r, p.straight_min, p.straight_max))
        elems.append((b, r, *step))                # 台阶的横向段
    elems += _end(rng, p)                          # 另一端掉头回主直道

    turns = [(a, r) for a, r, _, _ in elems]
    lo = np.array([x for _, _, x, _ in elems])
    hi = np.array([x for _, _, _, x in elems])
    lo[0] = rng.uniform(p.main_straight_min, p.main_straight_max)      # 主直道单独给上下限
    hi[0] = p.main_straight_max
    return turns, lo, np.maximum(hi, lo)


def _close_loop(headings: np.ndarray, arc_shift: np.ndarray, lengths: np.ndarray,
                lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """调整直道长度让首尾闭合:解 A·L = b(A 是各直道方向,b 是弯道位移的相反数)。

    交替投影:先最小范数修正 L += A⁺(b - A·L) 投到闭合面上,再把短于下限的截回 lo,
    反复直到两边同时满足。只压下限:两边都夹的话可行集太小,九成抽不出解;
    长度拉太长导致赛道超尺寸,由 make_rule_track 里的骨架粗筛便宜地挡掉。
    """
    A = np.stack([np.cos(headings), np.sin(headings)])          # (2,n)
    pinv = np.linalg.pinv(A)
    b = -arc_shift
    for _ in range(200):
        lengths = lengths + pinv @ (b - A @ lengths)
        if (lengths >= lo - 1e-9).all():
            break
        lengths = np.maximum(lengths, lo)
    if np.linalg.norm(b - A @ lengths) > 1e-6 or (lengths < lo - 1e-6).any():
        raise ValueError("这组转角配不出合法直道长度")
    return lengths


def _solve(rng: np.random.Generator, p: RuleTrackParams):
    """抽一套元素并解出直道长度。只做代数,不生成点,失败就重抽(很便宜)。"""
    layout, lo, hi = _layout(rng, p)
    angles = np.array([a for a, _ in layout])
    headings = np.radians(np.concatenate([[0.0], np.cumsum(angles)[:-1]]))

    # 每个弯的位移(在自身入弯朝向下)与直道无关,先算出来
    shift = np.zeros(2)
    yaw = 0.0
    for turn, radius in layout:
        a = math.radians(turn)
        sign = 1.0 if a >= 0 else -1.0
        # 圆心在左(左转)或右(右转),入弯点到出弯点的弦
        local = np.array([radius * math.sin(abs(a)), sign * radius * (1 - math.cos(a))])
        c, s = math.cos(yaw), math.sin(yaw)
        shift += np.array([c * local[0] - s * local[1], s * local[0] + c * local[1]])
        yaw += a

    lengths = _close_loop(headings, shift, lo + rng.uniform(0, 1.0, len(lo)) * (hi - lo), lo, hi)
    return layout, lengths


def _skeleton(layout, lengths) -> np.ndarray:
    """只取各元素端点和弯道中点,用来先粗筛尺寸 —— 比生成整条中心线便宜 50 倍。"""
    pts, pos, yaw = [], np.zeros(2), 0.0
    for length, (turn, radius) in zip(lengths, layout):
        pos = pos + length * np.array([math.cos(yaw), math.sin(yaw)])
        pts.append(pos.copy())
        a = math.radians(turn)
        sign = 1.0 if a >= 0 else -1.0
        centre = pos + sign * radius * np.array([-math.sin(yaw), math.cos(yaw)])
        phi0 = math.atan2(*(pos - centre)[::-1])
        for phi in (phi0 + a / 2, phi0 + a):
            pts.append(centre + radius * np.array([math.cos(phi), math.sin(phi)]))
        pos = pts[-1]
        yaw += a
    return np.array(pts)


def _centerline(layout, lengths) -> np.ndarray:
    pts, pos, yaw = [], np.zeros(2), 0.0
    for length, (turn, radius) in zip(lengths, layout):
        step = max(2, int(length / 0.05))
        d = np.array([math.cos(yaw), math.sin(yaw)])
        pts.append(pos + np.linspace(0, length, step, endpoint=False)[:, None] * d)
        pos = pos + length * d
        a = math.radians(turn)
        sign = 1.0 if a >= 0 else -1.0
        centre = pos + sign * radius * np.array([-math.sin(yaw), math.cos(yaw)])
        phi0 = math.atan2(*(pos - centre)[::-1])
        # 弯道采样要比后面 5cm 的重采样密,否则重采样点落在折线顶点上,转角会被切碎
        phi = phi0 + np.linspace(0, a, max(2, int(radius * abs(a) / 0.02)), endpoint=False)
        pts.append(centre + radius * np.stack([np.cos(phi), np.sin(phi)], axis=1))
        pos = centre + radius * np.array([math.cos(phi0 + a), math.sin(phi0 + a)])
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
                    attempts: int = 600) -> Track:
    for _ in range(attempts):
        try:
            layout, lengths = _solve(rng, p)
        except (ValueError, RuntimeError):
            continue
        span = sorted(np.ptp(_skeleton(layout, lengths), axis=0))
        if span[1] > p.max_extent or span[0] > p.max_short_side:
            continue                                 # [规则] 场地尺寸有限,且赛道是长条形
        centre = _resample_closed(_centerline(layout, lengths), 0.05)
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
