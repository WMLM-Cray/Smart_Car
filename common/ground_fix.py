"""地面分割掩码 -> 地面点,带在线俯仰修正和脏点筛除(纯 numpy,车上 ground_seg_node 和离线评估共用)。

为什么要在线修俯仰:车停着时标定准(≤0.3°),但实拍里相机对地面的俯仰一帧一帧能差 ±1.5°(车身点头 / 支架晃),
地面投影误差 ≈ 距离²/高度 × 角度,3° 在 2 m 处就是 ~0.9 m,地面会"穿墙"(2026-10-14 runs/ground_compare_large)。

每帧:
1. 去孤岛:只留从画面底边连上来的地面(墙上反光、远处孤立小块不要)。
2. 俯仰:雷达里连成直线段的回波(墙、柜子;椅腿桌腿成不了线)当墙根,在一组候选俯仰下投到图像,
   和掩码每列地面上沿比行差,取"中位 |行差|"最小的候选(一半以内的点被矮东西挡住也不怕;
   一样好的候选取离标定值最近的)。
   点够多、横向铺得开、多数点贴合才采用;不采用时 1 s 内沿用上次的值,再久回标定值。
3. 投影:按选中的俯仰查预先算好的表;俯仰被雷达确认时信到 3 m,否则只信到 2 m(俯仰不确定时远处误差最大)。
4. 雷达截断:同一方向上比雷达回波远 0.15 m 以上的地面点删掉(墙后不可能是地面)。只删不加。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from common.ground_proj import GroundLut, ground_to_pixel
from common.params import CarParams

PITCH_OFFSETS = np.radians(np.arange(-5.0, 5.001, 0.25))   # 相对标定值的候选俯仰
WALL_RANGE = (0.6, 3.0)       # 拿来修俯仰的雷达回波距离 m
LINE_HALF = 20                # 判直线:前后各 20 根(0.216°/根,1.5 m 处 ±0.11 m)
LINE_TOL = 0.008              # 离拟合直线的均方根 m
LINE_SPAN = 0.2               # 这段至少多长 m
MIN_WALL = 30                 # 至少这么多墙点
MIN_SPREAD = 0.15             # 墙点横向至少铺开画面宽度的这么多
MAX_COST = 0.75               # 中位 |行差| 至多这么多个掩码行(72 行时一行 ≈ 0.85°)
TIE_ROWS = 0.3                # 代价和最好的差在这么多行以内算一样好,取离标定值最近的
FIT_ROWS = 1.5                # |行差| 在这么多个掩码行以内算贴合
MIN_INLIER = 0.5              # 贴合点比例
HOLD_S = 1.0                  # 不采用时沿用上次俯仰的时长 s
RANGE_SURE, RANGE_UNSURE = 3.0, 2.0
CUT_MARGIN = 0.15             # 雷达截断余量 m


def bottom_connected(mask: np.ndarray) -> np.ndarray:
    """只留和最底一行 4 连通的地面格。"""
    m = mask.astype(bool)
    keep = np.zeros_like(m)
    keep[-1] = m[-1]
    while True:
        grow = keep.copy()
        grow[:-1] |= keep[1:]
        grow[1:] |= keep[:-1]
        grow[:, :-1] |= keep[:, 1:]
        grow[:, 1:] |= keep[:, :-1]
        grow &= m
        if (grow == keep).all():
            return keep
        keep = grow


def floor_edge(mask: np.ndarray, height: int) -> np.ndarray:
    """每列从底往上连续地面的上沿,换算成原图像素行;底格就不是地面的列给 nan。"""
    rows = mask.shape[0]
    notf = ~mask[::-1]
    run = np.where(notf.any(0), notf.argmax(0), rows)      # 底下连续地面的格数
    return np.where(run > 0, (rows - run) * height / rows, np.nan)


def wall_points(ranges, angle_min: float, angle_inc: float, p: CarParams) -> np.ndarray:
    """雷达里连成直线段的回波 -> 车体系 (n,2)。"""
    r = np.asarray(ranges, float)
    a = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r) & (r > WALL_RANGE[0]) & (r < WALL_RANGE[1])
    xy = np.c_[p.lidar_x + np.where(ok, r, 0) * np.cos(a), np.where(ok, r, 0) * np.sin(a)]
    k, n = LINE_HALF, 2 * LINE_HALF + 1
    if len(r) < n:
        return np.empty((0, 2))
    win = np.lib.stride_tricks.sliding_window_view
    wx, wy, wok = win(xy[:, 0], n), win(xy[:, 1], n), win(ok, n).all(1)
    cx, cy = wx - wx.mean(1, keepdims=True), wy - wy.mean(1, keepdims=True)
    sxx, syy, sxy = (cx * cx).mean(1), (cy * cy).mean(1), (cx * cy).mean(1)
    lam = (sxx + syy) / 2 - np.sqrt(((sxx - syy) / 2) ** 2 + sxy ** 2)   # 协方差小特征值
    span = np.hypot(wx[:, -1] - wx[:, 0], wy[:, -1] - wy[:, 0])
    line = wok & (span >= LINE_SPAN) & (np.sqrt(np.maximum(lam, 0)) < LINE_TOL)
    sel = np.zeros(len(r), bool)
    sel[k:len(r) - k] = line
    return xy[sel]


@dataclass
class FixInfo:
    pitch: float          # 这帧用的俯仰 rad
    source: str           # 'lidar' 本帧雷达确认 / 'hold' 沿用 / 'calib' 标定值
    walls: int            # 墙点数
    inlier: float         # 贴合比例(没算时 0)
    cut: int              # 雷达截断删掉的点数
    points: int           # 发出去的点数


class GroundFixer:
    """rows×cols 的掩码(铺满整幅原图) -> 车体系地面点。一个节点一个实例(带俯仰保持状态)。"""

    def __init__(self, rows: int, cols: int, p: CarParams = CarParams()):
        self.p, self.rows, self.cols = p, rows, cols
        self.pitches = p.camera_pitch + PITCH_OFFSETS
        self.params = [replace(p, camera_pitch=float(v)) for v in self.pitches]
        self.luts = [GroundLut(rows, cols, q, RANGE_SURE) for q in self.params]
        self.calib = int(np.argmin(np.abs(PITCH_OFFSETS)))
        self.last_i, self.last_t = self.calib, -math.inf

    def estimate(self, edge: np.ndarray, walls: np.ndarray):
        """-> (候选下标, 贴合比例) 或 (None, 贴合比例)。"""
        if len(walls) < MIN_WALL:
            return None, 0.0
        W, H = self.p.camera_size
        row_px = H / self.rows
        costs = []
        for i, q in enumerate(self.params):
            uv = ground_to_pixel(walls, q)
            ok = np.isfinite(uv[:, 0]) & (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] > 0) & (uv[:, 1] < H)
            col = (uv[ok, 0] * self.cols / W).astype(int)
            d = uv[ok, 1] - edge[col]
            good = np.isfinite(d)
            if good.sum() < MIN_WALL or np.ptp(col[good]) < MIN_SPREAD * self.cols:
                continue
            costs.append((np.median(np.abs(d[good])), i, float(np.mean(np.abs(d[good]) < FIT_ROWS * row_px))))
        if not costs:
            return None, 0.0
        low = min(c for c, _, _ in costs)
        if low > MAX_COST * row_px:                                     # 怎么转都贴不上:墙前有矮东西 / 掩码错
            return None, 0.0
        # 掩码一行 ≈ 0.85°,代价曲线底部是平的:一样好的候选里取离标定值最近的,别被量化噪声带跑
        _, i, inlier = min((t for t in costs if t[0] <= low + TIE_ROWS * row_px), key=lambda t: abs(t[1] - self.calib))
        if inlier < MIN_INLIER or i in (0, len(self.params) - 1):     # 多数点贴不上,或顶到候选边界
            return None, inlier
        return i, inlier

    def process(self, mask: np.ndarray, now: float, scan=None) -> tuple[np.ndarray, FixInfo]:
        """scan = (ranges, angle_min, angle_increment) 或 None。返回 (地面点 (n,2), FixInfo)。"""
        floor = bottom_connected(mask)
        walls = wall_points(*scan, self.p) if scan is not None else np.empty((0, 2))
        i, inlier = self.estimate(floor_edge(floor, self.p.camera_size[1]), walls)
        if i is not None:
            self.last_i, self.last_t, source = i, now, 'lidar'
        elif now - self.last_t <= HOLD_S:
            i, source = self.last_i, 'hold'
        else:
            i, source = self.calib, 'calib'
        lut = self.luts[i]
        sel = floor.ravel() & lut.valid
        xy = lut.xy[sel]
        if source != 'lidar':
            xy = xy[np.hypot(xy[:, 0] - self.p.camera_x, xy[:, 1]) <= RANGE_UNSURE]
        cut = 0
        if scan is not None and len(xy):
            keep = self.lidar_keep(xy, *scan)
            cut, xy = int((~keep).sum()), xy[keep]
        if len(xy):
            xy = np.unique(np.round(xy / 0.05).astype(np.int32), axis=0).astype(np.float64) * 0.05
        return xy, FixInfo(float(self.pitches[i]), source, len(walls), inlier, cut, len(xy))

    def lidar_keep(self, xy, ranges, angle_min, angle_inc) -> np.ndarray:
        """比同方向雷达回波远 CUT_MARGIN 以上的点给 False;那根没有有效回波就不管。"""
        r = np.asarray(ranges, float)
        dx, dy = xy[:, 0] - self.p.lidar_x, xy[:, 1]
        k = np.round((np.arctan2(dy, dx) - angle_min) / angle_inc).astype(int) % len(r)
        rl = r[k]
        valid = np.isfinite(rl) & (rl >= self.p.lidar_min)
        return ~(valid & (np.hypot(dx, dy) > rl + CUT_MARGIN))
