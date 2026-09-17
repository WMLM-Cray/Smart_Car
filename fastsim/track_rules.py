"""按比赛规则拼出来的训练赛道:直道 + 45°/90°/135° 弯,双边锥桶,随机蓝挡板。

和 track.py 的随机环形赛道不同 —— 那个是任意曲率的光滑闭环,实际比赛不长那样。
比赛当天从 A/B/C/D 四套模块化赛道里随机抽一套(培训笔记 05:49),四套都由同一批
元素拼成,所以这里按元素随机拼闭环,训练出来的策略对哪套都见过同类几何。

形状照着官方赛道示意图(fastsim/reference_track_figure.png,规则图 1):长条形,
一条大直道,两端各一个 180° 掉头大弯,回程侧连续小 S 弯,可能含直角区。

规则依据(标注同 params.py):
  [规则] 赛道宽 1.5m;含直线、S 弯、圆弯、直角区(规则第 2-6 页)
  [规则] 转角 45°/90°/135°,总横向尺寸 16.8m —— 队友转述规则图 9
  [规则] 圆形赛道内径 1.25m / 外径 2.75m(培训笔记 11:06),与 1.5m 宽一致
  [规则] 5 块随机放置的蓝色挡板,裁判靠地图里挡板位置验证建图真实(培训笔记 06:45)
  [图1]  锥桶间距 ≈ 赛道宽/1.32 ≈ 1.15m;S 弯比圆弯紧;短边远小于长边
  [假设] 直道长度范围、挡板尺寸、端部大弯半径 —— 规则原文没拿到,先给合理值

闭环怎么保证:见 _layout —— 两端做法相同、台阶成镜像对,纵向位移和转角都结构上抵消,
只剩主直道一个未知数,一个减法解完。
"""
from __future__ import annotations

import math

import numpy as np

from .params import RuleTrackParams
from .track import Track, _resample_closed


def _layout(rng: np.random.Generator, p: RuleTrackParams):
    """排出一圈元素:[(该段直道长度, 转角°, 弯半径), ...],主直道长度先留 None 由闭合定。

    形状是矩形回路:主直道 -> 端部拐上去 -> 回程侧若干矩形台阶 -> 另一端拐回来。
    闭合不靠迭代求解,靠结构:
      · 两端用同一种做法、同一个半径、同一段横向长度 -> 纵向位移正好抵消
      · 台阶成镜像对((+a,-a) 配 (-a,+a))-> 纵向位移也抵消
      · 转角和 = 4x90(或 2x180)+ 台阶正负抵消 = 360°
    于是只剩主直道一个未知数,一个减法就解完(_solve),而且赛道长宽都由参数直接控制。
    """
    square = rng.random() < p.square_end_chance
    height = rng.uniform(p.height_min, p.height_max)          # 主直道到回程直道的间距
    # 方角要看得出是直角:半径先让位于横直道,不然两个 90° 弯接在一起就成了一个圆弧掉头
    r_end = min(rng.uniform(p.bend_radius_min, p.bend_radius_max), (height - p.cross_min) / 2)

    def end(lead):
        if square:                                            # 方角:两个直角夹一段横直道
            return [(lead, 90.0, r_end), (height - 2 * r_end, 90.0, r_end)]
        return [(lead, 180.0, max(p.bend_radius_min, height / 2))]     # 圆弯掉头

    jogs = []
    for _ in range(int(rng.integers(p.s_bends_min, p.s_bends_max + 1))):
        a = 90.0 if rng.random() < p.right_angle_ratio else float(rng.choice([45.0, 135.0]))
        r = rng.uniform(p.bend_radius_min, p.bend_radius_max)
        step = rng.uniform(p.jog_min, p.jog_max) if a == 90.0 else rng.uniform(p.straight_min, p.straight_max)
        lead = rng.uniform(p.straight_min, p.straight_max)
        jogs += [((a, -a), r, step, lead), ((-a, a), r, step, lead)]   # 镜像对,纵向抵消
    rng.shuffle(jogs)

    elems = end(None)                                         # 主直道长度待定
    for (a, b), r, step, lead in jogs:
        elems += [(lead, a, r), (step, b, r)]
    elems += end(rng.uniform(p.straight_min, p.straight_max))
    return elems


def _shift(elems, main: float) -> np.ndarray:
    """走一圈的总位移(主直道长度取 main)。"""
    pos, yaw = np.zeros(2), 0.0
    for length, turn, radius in elems:
        pos = pos + (main if length is None else length) * np.array([math.cos(yaw), math.sin(yaw)])
        a = math.radians(turn)
        sign = 1.0 if a >= 0 else -1.0
        centre = pos + sign * radius * np.array([-math.sin(yaw), math.cos(yaw)])
        phi0 = math.atan2(*(pos - centre)[::-1])
        pos = centre + radius * np.array([math.cos(phi0 + a), math.sin(phi0 + a)])
        yaw += a
    return pos


def _solve(rng: np.random.Generator, p: RuleTrackParams):
    """抽一套元素,解出主直道长度。只做代数不生成点,不合法就重抽(约 0.1ms)。"""
    elems = _layout(rng, p)
    shift = _shift(elems, 0.0)                                # 主直道方向是 +x,长度线性叠加
    if abs(shift[1]) > 1e-6:
        raise ValueError("这套元素纵向不闭合")                 # 结构上不该发生,防回归
    main = -shift[0]
    if not p.main_straight_min <= main <= p.main_straight_max:
        raise ValueError("主直道长度超出范围")
    return [(main if length is None else length, turn, radius) for length, turn, radius in elems]


def _skeleton(elems) -> np.ndarray:
    """只取各元素端点和弯道中点,用来先粗筛尺寸 —— 比生成整条中心线便宜 50 倍。"""
    pts, pos, yaw = [], np.zeros(2), 0.0
    for length, turn, radius in elems:
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


def _centerline(elems) -> np.ndarray:
    pts, pos, yaw = [], np.zeros(2), 0.0
    for length, turn, radius in elems:
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
    """抽到一条合法赛道为止。

    极少数随机组合在场地尺寸内怎么摆都不合法(实测约 0.3%),这时逐轮放宽尺寸上限 ——
    环境 reset 不允许失败,一次抛异常就会让整轮训练挂掉(2026-09-17 踩过)。
    """
    for relax, tries in ((1.0, attempts), (1.2, 200), (1.5, 200), (2.0, 200)):
        track = _try_make(rng, p, tries, p.max_extent * relax, p.max_short_side * relax)
        if track is not None:
            return track
    raise RuntimeError("按规则元素拼不出闭环赛道,放宽 max_extent 或转角/直道范围")


def _try_make(rng, p: RuleTrackParams, attempts: int, limit_long: float, limit_short: float):
    for _ in range(attempts):
        try:
            elems = _solve(rng, p)
        except (ValueError, RuntimeError):
            continue
        span = sorted(np.ptp(_skeleton(elems), axis=0))
        if span[1] > limit_long or span[0] > limit_short:
            continue                                 # [规则] 场地尺寸有限,且赛道是长条形
        centre = _resample_closed(_centerline(elems), 0.05)
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
        try:
            return Track(centre, left, right, p, _place_boards(rng, left, right, p))
        except ValueError:
            continue                                 # 锥桶太少放不下挡板,换一条
    return None
