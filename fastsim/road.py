"""路段生成器:把直道、弯、掉头、台阶、变宽、无边界段串成一条不闭合的路。

走廊和赛道是同一套东西,只是边界材质不同(墙 / 锥桶 / 没有)。赛道 = 同样的路段
拼成闭环再换成锥桶,放在课程最后。

几何约定:
  中心线按 DS 等距采样,左右边界 = 中心线 ± 法向 · 半宽。
  弯的中心线半径 = 内侧半径 r_in + 半宽,所以 r_in=0 就是走廊的尖内角
  (外侧是半径为路宽的圆角,比真实方角空间小,偏保守)。
  车走不走得过去不在这里判断,见 road_check.py。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

DS = 0.05                   # 中心线采样间距 m
NONE, WALL, CONE = 0, 1, 2  # 边界材质
WALL_HEIGHT = 1.0           # 墙比雷达扫描面(0.21m)高就行
WALL_STEP = 0.2             # 墙按这个间距切成线段给射线求交


@dataclass(frozen=True)
class Level:
    """一级课程 = 一组生成参数的范围。"""
    name: str
    width: tuple = (1.6, 2.5)           # 路宽 m
    angles: tuple = (45.0, 90.0)        # 弯角可选值
    r_in: tuple = (1.0, 2.0)            # 弯内侧半径 m
    sharp_chance: float = 0.0           # 弯是尖角(r_in 0~0.1)的概率
    uturn_chance: float = 0.0           # 一个元素是 180° 掉头的概率
    uturn_r_in: tuple = (0.5, 1.25)
    jog_chance: float = 0.0             # 台阶(两个反向 90°)的概率
    taper_chance: float = 0.0           # 变宽段的概率
    gap_chance: float = 0.0             # 直道做成无边界段的概率
    gap_len: tuple = (3.0, 6.0)
    one_side_none_chance: float = 0.0   # 某一侧没有边界的概率
    n_elems: tuple = (2, 3)             # 起点直道之后的元素个数
    straight: tuple = (1.5, 4.0)        # 元素之间的直道长
    cone_spacing: tuple = (0.5, 0.9)
    wall_chance: float = 0.5            # 一侧用墙(否则锥桶)的概率


LEVELS = (
    Level('L1'),
    Level('L2', width=(1.2, 2.5), angles=(45.0, 90.0, 135.0), r_in=(0.3, 2.0),
          uturn_chance=0.15, jog_chance=0.15, taper_chance=0.1, n_elems=(3, 5),
          straight=(0.8, 4.0)),
    Level('L3', width=(0.9, 2.5), angles=(45.0, 90.0, 135.0), r_in=(0.0, 2.0),
          sharp_chance=0.4, uturn_chance=0.15, jog_chance=0.2, taper_chance=0.15,
          n_elems=(3, 6), straight=(0.4, 3.0)),
    Level('L4', width=(0.9, 2.5), angles=(45.0, 90.0, 135.0), r_in=(0.0, 2.0),
          sharp_chance=0.4, uturn_chance=0.15, jog_chance=0.2, taper_chance=0.15,
          gap_chance=0.2, one_side_none_chance=0.1, n_elems=(4, 7), straight=(0.3, 3.0),
          cone_spacing=(0.4, 1.2)),
)


@dataclass
class Road:
    centre: np.ndarray          # (n,2)
    heading: np.ndarray         # (n,)
    s: np.ndarray               # (n,) 弧长
    half: np.ndarray            # (n,) 半宽
    mat_left: np.ndarray        # (n,) 材质
    mat_right: np.ndarray
    cone_radius: float = 0.14
    walls: np.ndarray = field(default_factory=lambda: np.zeros((0, 2, 2)))
    cones: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    elems: list = field(default_factory=list)     # 生成记录,便于复现和统计

    @property
    def normal(self) -> np.ndarray:
        return np.stack([-np.sin(self.heading), np.cos(self.heading)], axis=1)

    @property
    def left(self) -> np.ndarray:
        return self.centre + self.normal * self.half[:, None]

    @property
    def right(self) -> np.ndarray:
        return self.centre - self.normal * self.half[:, None]

    @property
    def length(self) -> float:
        return float(self.s[-1])

    def project(self, xy, near: int | None = None, window: int = 80) -> tuple[int, float]:
        """点投影到中心线:返回 (最近点下标, 横向偏移左正)。

        near 给上一帧的下标就只在前后 window 个点里找 —— 掉头之后两段路可能只隔
        一个路宽,全局找最近点会认到隔壁那段。
        """
        lo, hi = (0, len(self.centre)) if near is None else (
            max(0, near - window), min(len(self.centre), near + window + 1))
        d = self.centre[lo:hi] - np.asarray(xy, float)
        i = lo + int(np.argmin(np.einsum('ij,ij->i', d, d)))
        rel = np.asarray(xy, float) - self.centre[i]
        n = (-math.sin(self.heading[i]), math.cos(self.heading[i]))
        return i, float(rel[0] * n[0] + rel[1] * n[1])


# ---- 生成 ----
def _elements(rng: np.random.Generator, lv: Level) -> list[dict]:
    """元素序列。每个元素:kind + 参数。首尾各一段直道(起步 / 终点区)。"""
    width = rng.uniform(*lv.width)
    out = [dict(kind='straight', length=rng.uniform(2.0, 3.0), width=width)]

    def straight():
        if rng.random() < lv.gap_chance:
            # 无边界段只放在直道上,前后各留 1m 看得见的边界
            out.append(dict(kind='straight', length=1.0, width=width))
            out.append(dict(kind='gap', length=rng.uniform(*lv.gap_len), width=width))
            out.append(dict(kind='straight', length=1.0, width=width))
        else:
            out.append(dict(kind='straight', length=rng.uniform(*lv.straight), width=width))

    def r_in():
        return rng.uniform(0.0, 0.1) if rng.random() < lv.sharp_chance else rng.uniform(*lv.r_in)

    for _ in range(int(rng.integers(lv.n_elems[0], lv.n_elems[1] + 1))):
        sign = 1 if rng.random() < 0.5 else -1
        u = rng.random()
        if u < lv.uturn_chance:
            out.append(dict(kind='arc', angle=180.0 * sign, r_in=rng.uniform(*lv.uturn_r_in),
                            width=width))
        elif u < lv.uturn_chance + lv.jog_chance:
            out.append(dict(kind='arc', angle=90.0 * sign, r_in=r_in(), width=width))
            out.append(dict(kind='straight', length=rng.uniform(1.2, 3.0), width=width))
            out.append(dict(kind='arc', angle=-90.0 * sign, r_in=r_in(), width=width))
        elif u < lv.uturn_chance + lv.jog_chance + lv.taper_chance:
            new = rng.uniform(*lv.width)
            out.append(dict(kind='taper', length=rng.uniform(1.5, 3.0), width=width, to=new))
            width = new
        else:
            out.append(dict(kind='arc', angle=float(rng.choice(lv.angles)) * sign, r_in=r_in(),
                            width=width))
        straight()
    out.append(dict(kind='straight', length=3.0, width=width))
    return out


def _materials(rng: np.random.Generator, lv: Level, elems: list[dict]) -> None:
    """每个元素两侧各选一种材质。整条路先抽一个"风格",元素之间小概率换。"""
    style = [WALL if rng.random() < lv.wall_chance else CONE for _ in range(2)]
    for e in elems:
        if e['kind'] == 'gap':
            e['mat'] = (NONE, NONE)
            continue
        side = []
        for k in range(2):
            if rng.random() < 0.15:
                style[k] = WALL if style[k] == CONE else CONE
            side.append(style[k])
        if e['kind'] == 'straight' and rng.random() < lv.one_side_none_chance:
            side[int(rng.integers(2))] = NONE
        e['mat'] = tuple(side)


def build(elems: list[dict], cone_spacing: float = 0.7, cone_radius: float = 0.14) -> Road:
    x = y = yaw = 0.0
    pts, hs, half, ml, mr = [], [], [], [], []
    for e in elems:
        w0 = e['width']
        if e['kind'] == 'arc':
            rc = e['r_in'] + w0 / 2
            a = math.radians(abs(e['angle']))
            sign = 1.0 if e['angle'] > 0 else -1.0
            n = max(2, int(math.ceil(rc * a / DS)))
            cx, cy = x - sign * rc * math.sin(yaw), y + sign * rc * math.cos(yaw)
            for k in range(n):
                t = yaw + sign * a * k / n
                pts.append((cx + sign * rc * math.sin(t), cy - sign * rc * math.cos(t)))
                hs.append(t)
                half.append(w0 / 2)
            yaw += sign * a
            x, y = cx + sign * rc * math.sin(yaw), cy - sign * rc * math.cos(yaw)
        else:
            n = max(1, int(math.ceil(e['length'] / DS)))
            w1 = e.get('to', w0)
            for k in range(n):
                f = k / n
                pts.append((x + math.cos(yaw) * e['length'] * f, y + math.sin(yaw) * e['length'] * f))
                hs.append(yaw)
                half.append((w0 + (w1 - w0) * f) / 2)
            x += math.cos(yaw) * e['length']
            y += math.sin(yaw) * e['length']
        ml += [e['mat'][0]] * n
        mr += [e['mat'][1]] * n
    pts.append((x, y))
    hs.append(yaw)
    half.append(half[-1])
    ml.append(ml[-1])
    mr.append(mr[-1])
    centre = np.array(pts)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(centre, axis=0).T))])
    road = Road(centre, np.unwrap(np.array(hs)), s, np.array(half), np.array(ml), np.array(mr),
                cone_radius=cone_radius, elems=elems)
    road.walls, road.cones = _boundary_objects(road, cone_spacing)
    return road


def _runs(mat: np.ndarray, kind: int):
    """材质数组里连续等于 kind 的 [lo, hi) 区间。"""
    on = np.r_[False, mat == kind, False]
    edges = np.flatnonzero(np.diff(on.astype(int)))
    return zip(edges[::2], edges[1::2])


def _resample(poly: np.ndarray, step: float) -> tuple[np.ndarray, np.ndarray]:
    """折线按弧长 step 重采样,返回 (点, 每点弧长)。重合点(尖内角)自然被跳过。"""
    seg = np.hypot(*np.diff(poly, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] < 1e-9:
        return poly[:1], s[:1]
    t = np.linspace(0, s[-1], max(2, int(math.ceil(s[-1] / step)) + 1))
    return np.stack([np.interp(t, s, poly[:, 0]), np.interp(t, s, poly[:, 1])], axis=1), t


def _boundary_objects(road: Road, cone_spacing: float):
    walls, cones = [], []
    for side, mat in ((road.left, road.mat_left), (road.right, road.mat_right)):
        for lo, hi in _runs(mat, WALL):
            p, _ = _resample(side[lo:hi + 1 if hi < len(side) else hi], WALL_STEP)
            seg = np.stack([p[:-1], p[1:]], axis=1)
            walls.append(seg[np.hypot(*(seg[:, 1] - seg[:, 0]).T) > 1e-6])
        for lo, hi in _runs(mat, CONE):
            p, _ = _resample(side[lo:hi], cone_spacing)
            cones.append(p)
    walls = np.concatenate(walls) if walls else np.zeros((0, 2, 2))
    cones = np.concatenate(cones) if cones else np.zeros((0, 2))
    if len(cones) > 1:          # 尖内角处两段的锥桶会叠在一起,去重
        keep = [0]
        for i in range(1, len(cones)):
            if np.min(np.hypot(*(cones[keep] - cones[i]).T)) > 0.3 * cone_spacing:
                keep.append(i)
        cones = cones[keep]
    return walls, cones


def self_overlaps(road: Road, clearance: float = 0.4) -> bool:
    """路和自己隔得太近(掉头后两段贴在一起、绕回来压到前面)就算重叠。

    弧长隔得远、欧氏距离却近于 两边半宽 + clearance 的点对就是重叠。
    """
    idx = np.arange(0, len(road.s), 5)
    c, s, h = road.centre[idx], road.s[idx], road.half[idx]
    d = np.hypot(c[:, None, 0] - c[None, :, 0], c[:, None, 1] - c[None, :, 1])
    need = h[:, None] + h[None, :] + clearance
    far = np.abs(s[:, None] - s[None, :]) > 2.0 * need
    return bool(np.any(far & (d < need)))


def generate(rng: np.random.Generator, lv: Level, tries: int = 50) -> Road:
    """按 lv 生成一条不自重叠的路。可通性另外查(road_check)。"""
    for _ in range(tries):
        elems = _elements(rng, lv)
        _materials(rng, lv, elems)
        road = build(elems, cone_spacing=rng.uniform(*lv.cone_spacing))
        if not self_overlaps(road):
            return road
    raise RuntimeError(f'{lv.name}: {tries} 次都生成了自重叠的路,参数范围有问题')
