"""地面分割掩码 -> 地面点:在线修俯仰/横滚 + 按方向和雷达对齐 + 脏点筛除(纯 numpy,车上 ground_seg_node 和离线评估共用)。

为什么:车停着时标定准(≤0.3°),但实拍里相机对地面的姿态一帧一帧能差 ±1.5°(车身点头 / 支架晃),
地面投影误差 ≈ 距离²/高度 × 角度,3° 在 2 m 处就是 ~0.9 m,地面会"穿墙"(2026-10-14 runs/ground_compare_large)。

每帧:
1. 去孤岛:只留从画面底边连上来的地面(墙上反光、远处孤立小块不要)。
2. 姿态:雷达里连成直线段的回波(墙、柜子;椅腿桌腿成不了线)当墙根投到图像,和掩码每列地面上沿比行差;
   行差 ≈ J_俯仰·δ俯仰 + J_横滚·δ横滚,两点一组随机抽样找多数派(一半以内的点被矮东西挡住也不怕),
   再对多数派最小二乘;墙点横向铺得不够开时只修俯仰。朝上(地面变远)只许修 1°,见 PITCH_LIM。
3. 跨帧平滑:采纳的单帧估计取最近 WINDOW_S 内的中位数,输出每帧最多变 STEP;和当前输出差太多的单帧当错估丢掉,
   连续 JUMP_CONFIRM 帧都一致地跳才认(换了停放姿态)。没有新估计时沿用 HOLD_S,再久回标定值。
4. 投影:按输出姿态查表(表按 0.1°/0.25° 量化,用到才算、缓存);姿态被雷达确认时信到 3 m,否则 2 m。
5. 按方向对齐:视野按 SECTOR 分扇区;扇区里的墙点在当前姿态下和掩码上沿贴合(两边都认为"地面一直到这面墙根",
   只差投影误差)时,这一格的地面就对齐到墙前 ALIGN_GAP:短了补(最多 FILL_MAX)、长了删。相机地面离墙还远就停了(多半是雷达看不到的
   矮东西)不补,信相机。
6. 雷达截断:同一方向上比雷达回波远 CUT_MARGIN 以上的地面点删掉(墙后不可能是地面)。
"""
from __future__ import annotations

import collections
import math
from dataclasses import dataclass, replace

import numpy as np

from common.ground_proj import GroundLut, ground_to_pixel
from common.params import CarParams

WALL_RANGE = (0.6, 3.0)       # 拿来修姿态的雷达回波距离 m
LINE_HALF = 20                # 判直线:前后各 20 根(0.216°/根,1.5 m 处 ±0.11 m)
LINE_TOL = 0.008              # 离拟合直线的均方根 m
LINE_SPAN = 0.2               # 这段至少多长 m
MIN_WALL = 20                 # 视野里至少这么多墙点
SPREAD_PITCH = 0.15           # 墙点横向铺开画面宽度的比例:够这么多才修俯仰
SPREAD_ROLL = 0.35            # 够这么多才连横滚一起修
FIT_ROWS = 1.5                # |行差| 在这么多个掩码行以内算贴合(72 行时一行 ≈ 0.85°)
MIN_INLIER = 0.5              # 贴合点比例
MAX_COST = 0.75               # 贴合点的中位 |行差| 至多这么多行
# 俯仰修正不对称:往"朝上"修会把地面投得更远(空地变多,危险方向);墙前整排矮东西在单帧里和"相机朝上"分不开,
# 所以朝上只许修一点。实测偏差都在朝下一侧(ground_snaps 相对标定 +0.45 ~ +3.45°)。
PITCH_LIM = (math.radians(-1.0), math.radians(4.0))   # (朝上, 朝下),正 = 朝下
ROLL_LIM = math.radians(3.0)
WINDOW_S = 0.7                # 跨帧中位数窗口 s(~3 Hz 时 2–3 帧)
STEP = math.radians(1.0)      # 输出每帧最多变这么多
JUMP = math.radians(1.5)      # 单帧估计和当前输出差这么多算可疑
JUMP_CONFIRM = 3              # 连续这么多帧都一致地跳才认
HOLD_S = 1.0                  # 没新估计时沿用的时长 s
RANGE_SURE, RANGE_UNSURE = 3.0, 2.0
SECTOR = math.radians(2.0)    # 对齐扇区宽
SECTOR_MIN = 5                # 扇区里至少这么多墙点才对齐
ALIGN_GAP = 0.05              # 对齐后地面停在墙前这么远 m
FILL_MAX = 0.3                # 一个扇区最多往前补这么远 m
CUT_MARGIN = 0.15             # 雷达截断余量 m
CELL = 0.05                   # 输出栅格 m
P_Q, R_Q = math.radians(0.1), math.radians(0.25)   # 查表量化


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
    roll: float           # 这帧用的横滚 rad
    source: str           # 'lidar' 本帧采纳了雷达估计 / 'hold' 沿用 / 'calib' 标定值
    walls: int            # 墙点数
    inlier: float         # 单帧估计的贴合比例(没算时 0)
    aligned: int          # 和雷达对齐的扇区数
    cut: int              # 删掉的点数(扇区对齐 + 雷达截断)
    filled: int           # 扇区对齐补上的点数
    points: int           # 发出去的点数
    why: str = ''         # 单帧没采纳的原因:few / view / inlier / cost / limit / jump


class AttitudeFilter:
    """单帧姿态估计 -> 平滑输出(见模块说明第 3 条)。角度都是相对标定值的修正量。"""

    def __init__(self):
        self.hist = collections.deque()                 # (t, dp, dr) 采纳的单帧估计
        self.pending = []                               # 被当成可疑跳变的单帧估计
        self.out, self.last_t = None, -math.inf

    def update(self, now, est):
        """est = (dp, dr) 或 None。返回 (dp, dr, source, why)。"""
        why = ''
        if est is not None:
            fresh = self.out is not None and now - self.last_t <= HOLD_S
            if fresh and abs(est[0] - self.out[0]) > JUMP:
                self.pending.append((now, *est))
                ps = np.array([q[1] for q in self.pending[-JUMP_CONFIRM:]])
                if len(self.pending) >= JUMP_CONFIRM and np.ptp(ps) < JUMP / 3:
                    self.hist = collections.deque(self.pending[-JUMP_CONFIRM:])   # 一致地跳:换姿态了
                    self.out, self.pending = None, []
                else:
                    est, why = None, 'jump'
            else:
                self.pending = []
                self.hist.append((now, *est))
        if est is not None:
            while self.hist and now - self.hist[0][0] > WINDOW_S:
                self.hist.popleft()
            target = np.median(np.array([(q[1], q[2]) for q in self.hist]), axis=0)
            if self.out is None or now - self.last_t > HOLD_S:
                self.out = target
            else:
                self.out = self.out + np.clip(target - self.out, -STEP, STEP)
            self.last_t = now
            return float(self.out[0]), float(self.out[1]), 'lidar', why
        if self.out is not None and now - self.last_t <= HOLD_S:
            return float(self.out[0]), float(self.out[1]), 'hold', why
        return 0.0, 0.0, 'calib', why


class GroundFixer:
    """rows×cols 的掩码(铺满整幅原图) -> 车体系地面点。一个节点一个实例(带姿态平滑状态)。"""

    def __init__(self, rows: int, cols: int, p: CarParams = CarParams(), fill: bool = False):
        self.p, self.rows, self.cols = p, rows, cols
        self.fill = fill                    # 补远处行间的缝(见 process)
        self.filter = AttitudeFilter()
        self.luts = {}
        self.row_px = p.camera_size[1] / rows

    def params(self, dp, dr):
        return replace(self.p, camera_pitch=self.p.camera_pitch + dp, camera_roll=self.p.camera_roll + dr)

    def lut(self, dp, dr):
        key = (round(dp / P_Q), round(dr / R_Q))
        if key not in self.luts:
            if len(self.luts) > 400:
                self.luts.clear()
            self.luts[key] = GroundLut(self.rows, self.cols, self.params(key[0] * P_Q, key[1] * R_Q), RANGE_SURE)
        return self.luts[key]

    def residuals(self, walls, edge, dp, dr):
        """墙点在姿态 (dp, dr) 下的 (视野内下标, 行差 = 预测墙根行 - 掩码上沿行, 列号)。"""
        W, H = self.p.camera_size
        uv = ground_to_pixel(walls, self.params(dp, dr))
        ok = np.isfinite(uv[:, 0]) & (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] > 0) & (uv[:, 1] < H)
        idx = np.flatnonzero(ok)
        col = (uv[idx, 0] * self.cols / W).astype(int)
        d = uv[idx, 1] - edge[col]
        good = np.isfinite(d)
        return idx[good], d[good], col[good]

    def estimate(self, edge, walls, dp0, dr0):
        """单帧姿态估计,从 (dp0, dr0) 出发。-> ((dp, dr), 贴合比例, '') 或 (None, 贴合比例, 原因)。"""
        if len(walls) < MIN_WALL:
            return None, 0.0, 'few'
        idx, d, col = self.residuals(walls, edge, dp0, dr0)
        if len(d) < MIN_WALL or np.ptp(col) < SPREAD_PITCH * self.cols:
            return None, 0.0, 'view'
        use_roll = np.ptp(col) >= SPREAD_ROLL * self.cols
        dp, dr, tol = dp0, dr0, FIT_ROWS * self.row_px
        rng = np.random.default_rng(0)
        for it in range(3):                                         # 重新线性化几轮
            idx, d, col = self.residuals(walls, edge, dp, dr)
            if len(d) < MIN_WALL:
                return None, 0.0, 'view'
            pts, eps = walls[idx], 1e-3
            v0 = ground_to_pixel(pts, self.params(dp, dr))[:, 1]
            J = np.c_[(ground_to_pixel(pts, self.params(dp + eps, dr))[:, 1] - v0) / eps,
                      (ground_to_pixel(pts, self.params(dp, dr + eps))[:, 1] - v0) / eps]
            if not use_roll:
                J = J[:, :1]
            if it == 0:                                             # 随机抽样找多数派(只在第一轮)
                best = None
                for _ in range(60):
                    s = rng.choice(len(d), J.shape[1], replace=False)
                    try:
                        x = np.linalg.solve(J[s], -d[s]) if J.shape[1] == 2 else -d[s] / J[s, 0]
                    except np.linalg.LinAlgError:
                        continue
                    n_in = int((np.abs(d + J @ np.atleast_1d(x)) < tol).sum())
                    if best is None or n_in > best[0]:
                        best = (n_in, np.atleast_1d(x))
                inl = np.abs(d + J @ best[1]) < tol
            else:
                inl = np.abs(d) < tol
            if inl.sum() < MIN_WALL // 2:
                return None, float(inl.mean()), 'inlier'
            x = np.linalg.lstsq(J[inl], -d[inl], rcond=None)[0]
            dp += float(x[0])
            dr += float(x[1]) if use_roll else 0.0
        idx, d, col = self.residuals(walls, edge, dp, dr)
        inlier = float(np.mean(np.abs(d) < tol)) if len(d) else 0.0
        if inlier < MIN_INLIER:
            return None, inlier, 'inlier'
        if np.median(np.abs(d[np.abs(d) < tol])) > MAX_COST * self.row_px:
            return None, inlier, 'cost'
        if not PITCH_LIM[0] <= dp <= PITCH_LIM[1] or abs(dr) > ROLL_LIM:
            return None, inlier, 'limit'
        return (dp, dr), inlier, ''

    def process(self, mask: np.ndarray, now: float, scan=None) -> tuple[np.ndarray, FixInfo]:
        """scan = (ranges, angle_min, angle_increment) 或 None。返回 (地面点 (n,2), FixInfo)。"""
        floor = bottom_connected(mask)
        edge = floor_edge(floor, self.p.camera_size[1])
        walls = wall_points(*scan, self.p) if scan is not None else np.empty((0, 2))
        dp0, dr0 = (self.filter.out if self.filter.out is not None else (0.0, 0.0))
        est, inlier, why = self.estimate(edge, walls, float(dp0), float(dr0))
        dp, dr, source, jwhy = self.filter.update(now, est)
        why = why or jwhy
        lut = self.lut(dp, dr)
        xy = lut.xy[floor.ravel() & lut.valid]
        if self.fill:                       # 补远处行间的缝(GroundLut.fill_rows),再走对齐 / 截断
            xy = np.vstack([xy, lut.fill_rows(floor)])
        n0 = len(xy)
        aligned, filled = 0, 0
        if len(walls):
            xy, aligned, filled = self.align(xy, walls, edge, dp, dr, sure=source == 'lidar')
        elif source != 'lidar':
            xy = xy[np.hypot(xy[:, 0] - self.p.camera_x, xy[:, 1]) <= RANGE_UNSURE]
        if scan is not None and len(xy):
            xy = xy[self.lidar_keep(xy, *scan)]
        cut = n0 + filled - len(xy)
        if len(xy):
            xy = np.unique(np.round(xy / CELL).astype(np.int32), axis=0).astype(np.float64) * CELL
        return xy, FixInfo(self.p.camera_pitch + dp, self.p.camera_roll + dr, source, len(walls), inlier,
                           aligned, cut, filled, len(xy), why)

    def align(self, xy, walls, edge, dp, dr, sure):
        """按扇区和雷达墙对齐(模块说明第 5 条);不在对齐扇区里的点按 sure 截到 3 / 2 m。-> (点, 对齐扇区数, 补的点数)。"""
        lx = self.p.lidar_x
        b = np.arctan2(xy[:, 1], xy[:, 0] - lx)
        dist = np.hypot(xy[:, 0] - lx, xy[:, 1])
        sec = np.floor(b / SECTOR).astype(int)
        far = np.hypot(xy[:, 0] - self.p.camera_x, xy[:, 1]) > (RANGE_SURE if sure else RANGE_UNSURE)
        keep = ~far
        idx, d, _ = self.residuals(walls, edge, dp, dr)
        wb = np.arctan2(walls[idx, 1], walls[idx, 0] - lx)
        wd = np.hypot(walls[idx, 0] - lx, walls[idx, 1])
        wsec = np.floor(wb / SECTOR).astype(int)
        extra, aligned = [], 0
        for k in np.unique(wsec):
            m = wsec == k
            if m.sum() < SECTOR_MIN or np.median(np.abs(d[m])) > FIT_ROWS * self.row_px:
                continue                                            # 墙点太少,或相机地面没到这面墙根
            in_sec = sec == k
            if not in_sec.any():
                continue
            aligned += 1
            stop = float(np.median(wd[m])) - ALIGN_GAP
            keep[in_sec] = dist[in_sec] <= stop                     # 长了删(也不受 3/2 m 限制,墙已确认)
            start = float(dist[in_sec & keep].max()) if (in_sec & keep).any() else stop
            start = max(start, stop - FILL_MAX)
            if stop - start > CELL:                                 # 短了补:扇区楔形里从相机地面尽头补到墙前
                rr = np.arange(start + CELL, stop, CELL)
                for r in rr:
                    bb = np.arange(k * SECTOR, (k + 1) * SECTOR, CELL / r)
                    extra.append(np.c_[lx + r * np.cos(bb), r * np.sin(bb)])
        out = xy[keep]
        add = np.concatenate(extra) if extra else np.empty((0, 2))
        return np.concatenate([out, add]), aligned, len(add)

    def lidar_keep(self, xy, ranges, angle_min, angle_inc) -> np.ndarray:
        """比同方向雷达回波远 CUT_MARGIN 以上的点给 False;那根没有有效回波就不管。"""
        r = np.asarray(ranges, float)
        dx, dy = xy[:, 0] - self.p.lidar_x, xy[:, 1]
        k = np.round((np.arctan2(dy, dx) - angle_min) / angle_inc).astype(int) % len(r)
        rl = r[k]
        valid = np.isfinite(rl) & (rl >= self.p.lidar_min)
        return ~(valid & (np.hypot(dx, dy) > rl + CUT_MARGIN))
