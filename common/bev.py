"""BEV(俯视语义栅格)—— 仿真和实车共用的唯一来源,只依赖 numpy。

为什么要它:路段策略每帧只看 48 个扇区的最近距离、叠 5 帧(0.25 s)。方角开口这类稀桶处,
单帧里只有几个孤立的点,车一转头刚经过的桶就从视野里没了;b036 剩下的失败大多是在这里
"认错路"。BEV 把雷达回波和摄像头检测按位姿累积到里程计坐标系,几秒前看到的桶还在,稀疏
边界连成线;雷达看不见的胶带由摄像头登记进来。

## 约定(改这里要走 PR,训练出来的模型和这些数字绑定)

分辨率  1 格 = 0.05 m(BevConfig.res)。锥桶在雷达高度处直径 ~0.19 m ≈ 4 格,车宽 0.35 m = 7 格。
裁剪窗  车前 6 m、车后 2 m、左右各 4 m -> 160 × 160 格(BevConfig.shape)。
坐标    位姿 (x, y, yaw) 是**后轴中心**在里程计系(odom)里的位置和朝向,和 FastSim 的 car.x/y/yaw、
        vehicle/ 的 base_footprint 一致。雷达在后轴前 CarParams.lidar_x,相机在 CarParams.camera_x。
        crop() 输出车头朝上:第 0 行在最前,第 0 列在最左。

格子类型(CellType,每格一个,多来源冲突按 PRIORITY 取):

    0 UNKNOWN        没有任何证据(没看到过)
    1 FREE           可行驶路面(摄像头地面分割;bev_v2 起登记,雷达射线扫过的空地仍不登记)
    2 OBSTACLE       雷达打中、摄像头没给类别
    3 CONE           锥桶(摄像头检测;雷达同格命中也算锥桶)
    4 TAPE           胶带/地面边线(只有摄像头看得见,例如黄 V 区)
    5 ZONE           地面区域标记(起点框、检测区黄框)
    6 TRAFFIC_LIGHT  红绿灯的位置

存储按"来源层"分开记计数,类型图是现算的:同一格既被雷达打中、又被摄像头认成锥桶,两层
都记;要判断"摄像头说是胶带但雷达也打中了"这类冲突,看层比看类型图直接。

    层 0 lidar   雷达命中次数
    层 1..5      摄像头各类别(CONE / TAPE / ZONE / TRAFFIC_LIGHT / FREE)的检测次数,顺序见 LAYERS

只存出现过证据的格子(odom 系没有边界,用稀疏存储);prune() 可删掉超过 max_age 没再更新的。

锥桶合并成"地标":摄像头每帧对同一个桶给一个带误差的落地点,逐次登记会把一个桶画成直径
20~30 cm 的一团。所以 CONE 不按格子存,而是按桶存:新检测落在已有地标 merge_r 以内就并进去
(位置取所有检测的平均),否则新建一个;至少被检到 min_hits 次的地标才画出来(滤掉偶发误检)。
画的时候每个地标是一个半径 = 锥桶底半径的实心圆。胶带、区域、红绿灯仍按格子存。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from common.params import CarParams, TrackParams

TRACK = TrackParams()


class CellType(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OBSTACLE = 2
    CONE = 3
    TAPE = 4
    ZONE = 5
    TRAFFIC_LIGHT = 6


# 来源层:层号 -> 它给出的格子类型。层 0 是雷达,其余是摄像头类别。
LAYERS = (CellType.OBSTACLE, CellType.CONE, CellType.TAPE, CellType.ZONE, CellType.TRAFFIC_LIGHT, CellType.FREE)
LIDAR_LAYER = 0
CAMERA_LAYER = {t: k for k, t in enumerate(LAYERS) if k > 0}
# 同一格多种证据时取谁:车能撞上的优先,其次是地面标记
PRIORITY = (CellType.CONE, CellType.TRAFFIC_LIGHT, CellType.OBSTACLE, CellType.TAPE, CellType.ZONE, CellType.FREE)


@dataclass(frozen=True)
class BevConfig:
    res: float = 0.05           # 格子边长 m
    ahead: float = 6.0          # 裁剪窗口:车前多远 m
    behind: float = 2.0         # 车后多远 m
    side: float = 4.0           # 左右各多宽 m
    max_range: float = 10.0     # 只用这么近的雷达回波(和路段策略的观测量程一致)
    max_age: float = 20.0       # 多久没再更新就忘掉 s
    merge_r: float = 0.25       # 锥桶检测离已有地标这么近就并进去 m(最密桶距 0.4,留余量)
    min_hits: int = 2           # 地标至少被检到这么多次才画
    cone_sigma0: float = 0.02   # [假设] 锥桶检测位置误差 m:cone_sigma0 + cone_sigma_per_m × 离光心距离
    cone_sigma_per_m: float = 0.02  # [假设] (与 FakeYolo 默认一致;实车未标定)

    @property
    def shape(self) -> tuple[int, int]:
        return (int(round((self.ahead + self.behind) / self.res)), int(round(2 * self.side / self.res)))


_SHIFT = 1 << 20                # 格子键 = (ix + 2^20) * 2^21 + (iy + 2^20);5 cm 格时 odom ±52 km 不越界


def _keys(ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
    return (ix.astype(np.int64) + _SHIFT) * (2 * _SHIFT) + (iy.astype(np.int64) + _SHIFT)


def _unkey(k: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return k // (2 * _SHIFT) - _SHIFT, k % (2 * _SHIFT) - _SHIFT


def body_to_odom(xy_body, pose) -> np.ndarray:
    """车体系(后轴原点,x 前 y 左)点 -> odom 系。"""
    p = np.asarray(xy_body, dtype=np.float64).reshape(-1, 2)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]], axis=1)


def lidar_points(ranges, angles, pose, params: CarParams = CarParams(), lidar_yaw: float = 0.0,
                 max_range: float = 10.0) -> np.ndarray:
    """一帧雷达回波 -> odom 系点 (n, 2)。无回波(inf/nan)、近于 lidar_min、远于 max_range 的丢掉。"""
    r = np.asarray(ranges, dtype=np.float64)
    a = np.asarray(angles, dtype=np.float64)
    ok = np.isfinite(r) & (r >= params.lidar_min) & (r <= max_range)
    x, y, yaw = pose
    ox, oy = x + params.lidar_x * math.cos(yaw), y + params.lidar_x * math.sin(yaw)
    world = a[ok] + yaw + lidar_yaw
    return np.stack([ox + r[ok] * np.cos(world), oy + r[ok] * np.sin(world)], axis=1)


class Bev:
    """累积 BEV。update_lidar() / update_camera() 登记证据;crop() 取以车为中心的图。"""

    def __init__(self, cfg: BevConfig = BevConfig(), params: CarParams = CarParams(),
                 lidar_yaw: float = 0.0):
        self.cfg, self.params, self.lidar_yaw = cfg, params, lidar_yaw
        self.reset()

    def reset(self):
        self.keys = np.zeros(0, np.int64)
        self.count = np.zeros((0, len(LAYERS)), np.int32)
        self.last = np.full((0, len(LAYERS)), -np.inf, np.float64)
        self.lm_xy = np.zeros((0, 2))           # 锥桶地标:位置(所有检测的平均)
        self.lm_n = np.zeros(0, np.int32)       # 被检到的次数
        self.lm_s2 = np.zeros(0, np.float64)    # 各次检测误差方差之和;平均位置的误差 = sqrt(lm_s2) / lm_n
        self.lm_last = np.zeros(0, np.float64)  # 最后一次被检到的时间

    def __len__(self):
        return len(self.keys)

    def _add(self, pts: np.ndarray, layer: int, t: float):
        if not len(pts):
            return
        res = self.cfg.res
        new, cnt = np.unique(_keys(np.floor(pts[:, 0] / res), np.floor(pts[:, 1] / res)),
                             return_counts=True)
        # keys 保持有序:二分找位置,已有的格子原地累加,没有的按序插入(不再对整张图重排)
        pos = np.searchsorted(self.keys, new)
        hit = pos < len(self.keys)
        hit[hit] = self.keys[pos[hit]] == new[hit]
        self.count[pos[hit], layer] += cnt[hit].astype(np.int32)
        self.last[pos[hit], layer] = np.maximum(self.last[pos[hit], layer], float(t))
        miss = ~hit
        add = np.zeros((int(miss.sum()), len(LAYERS)), np.int32)
        add[:, layer] = cnt[miss]
        self.keys = np.insert(self.keys, pos[miss], new[miss])
        self.count = np.insert(self.count, pos[miss], add, axis=0)
        last = np.full(add.shape, -np.inf)
        last[:, layer] = float(t)
        self.last = np.insert(self.last, pos[miss], last, axis=0)

    def update_lidar(self, ranges, angles, pose, t: float):
        """一帧雷达。pose 是**这一帧扫描时**的后轴位姿。"""
        self._add(lidar_points(ranges, angles, pose, self.params, self.lidar_yaw, self.cfg.max_range),
                  LIDAR_LAYER, t)

    def update_camera(self, kinds, xy_body, pose, t: float):
        """一批摄像头检测。kinds 是 CellType(CONE / TAPE / ZONE / TRAFFIC_LIGHT),xy_body 是检测
        落地点在车体系(后轴原点,x 前 y 左)的坐标 —— 像素到地面的投影由摄像头节点做。
        胶带这类线状目标给沿线的一串点。"""
        kinds = np.asarray(kinds, dtype=int).reshape(-1)
        pts = body_to_odom(xy_body, pose)
        rng = np.hypot(*(np.asarray(xy_body, float).reshape(-1, 2) - (self.params.camera_x, 0.0)).T)
        for kind in np.unique(kinds):
            if CellType(kind) not in CAMERA_LAYER:
                raise ValueError(f'摄像头不登记 {CellType(kind).name}')
            if kind == CellType.CONE:
                self._merge_cones(pts[kinds == kind], rng[kinds == kind], t)
            else:
                self._add(pts[kinds == kind], CAMERA_LAYER[CellType(kind)], t)

    def _merge_cones(self, pts: np.ndarray, rng: np.ndarray, t: float):
        r2 = self.cfg.merge_r ** 2
        for p, s2 in zip(pts, (self.cfg.cone_sigma0 + self.cfg.cone_sigma_per_m * rng) ** 2):
            if len(self.lm_xy):
                d2 = np.einsum('ij,ij->i', self.lm_xy - p, self.lm_xy - p)
                k = int(np.argmin(d2))
                if d2[k] <= r2:
                    n = self.lm_n[k]
                    self.lm_xy[k] = (self.lm_xy[k] * n + p) / (n + 1)
                    self.lm_n[k] = n + 1
                    self.lm_s2[k] += s2
                    self.lm_last[k] = t
                    continue
            self.lm_xy = np.vstack([self.lm_xy, p])
            self.lm_n = np.append(self.lm_n, 1).astype(np.int32)
            self.lm_s2 = np.append(self.lm_s2, s2)
            self.lm_last = np.append(self.lm_last, float(t))
        self._consolidate()

    def _consolidate(self):
        """两个地标靠到 merge_r + 两者平均位置的误差以内就并成一个。远处的检测
        误差大(5 m 处每轴 ~12 cm),几次远处检测可能另立一个偏了 ~0.28 m 的地标;之后近处的检测
        都并到更近的真地标上,它再也不会被拉回来。按误差放宽合并距离,这种只有几次远处检测的
        地标就能并进去;两个都被近处看准了的地标仍按 merge_r(最密桶距 0.4)分开。"""
        while len(self.lm_xy) > 1:
            d = self.lm_xy[:, None] - self.lm_xy[None]
            d2 = np.einsum('ijk,ijk->ij', d, d)
            s2 = self.lm_s2 / self.lm_n.astype(float) ** 2
            over = d2 - (self.cfg.merge_r + np.sqrt(s2[:, None] + s2[None])) ** 2
            np.fill_diagonal(over, np.inf)
            i, j = np.unravel_index(int(np.argmin(over)), over.shape)
            if over[i, j] > 0:
                return
            ni, nj = self.lm_n[i], self.lm_n[j]
            self.lm_xy[i] = (self.lm_xy[i] * ni + self.lm_xy[j] * nj) / (ni + nj)
            self.lm_n[i] = ni + nj
            self.lm_s2[i] += self.lm_s2[j]
            self.lm_last[i] = max(self.lm_last[i], self.lm_last[j])
            keep = np.arange(len(self.lm_xy)) != j
            self.lm_xy, self.lm_n, self.lm_s2, self.lm_last = (
                self.lm_xy[keep], self.lm_n[keep], self.lm_s2[keep], self.lm_last[keep])

    def cones(self, t: float | None = None) -> np.ndarray:
        """画得出来的锥桶地标(odom 系),(n, 2):被检到至少 min_hits 次,t 给了还要在 max_age 内。"""
        ok = self.lm_n >= self.cfg.min_hits
        if t is not None:
            ok &= t - self.lm_last <= self.cfg.max_age
        return self.lm_xy[ok]

    def prune(self, t: float):
        self.count[t - self.last > self.cfg.max_age] = 0
        keep = self.count.any(axis=1)
        self.keys, self.count, self.last = self.keys[keep], self.count[keep], self.last[keep]
        keep = t - self.lm_last <= self.cfg.max_age
        self.lm_xy, self.lm_n, self.lm_s2, self.lm_last = (
            self.lm_xy[keep], self.lm_n[keep], self.lm_s2[keep], self.lm_last[keep])

    def points(self, kind: CellType | None = None) -> np.ndarray:
        """格子中心(odom 系),(n, 2)。kind 给了就只要类型图里是这一类的格子;锥桶请用 cones()。"""
        ix, iy = _unkey(self.keys)
        pts = np.stack([(ix + 0.5) * self.cfg.res, (iy + 0.5) * self.cfg.res], axis=1)
        return pts if kind is None else pts[self.types() == kind]

    def types(self) -> np.ndarray:
        """每个已存格子的类型(CellType),按 PRIORITY 取。"""
        out = np.full(len(self.keys), int(CellType.UNKNOWN), np.int8)
        for kind in reversed(PRIORITY):
            layer = LIDAR_LAYER if kind == CellType.OBSTACLE else CAMERA_LAYER[kind]
            out[self.count[:, layer] > 0] = int(kind)
        return out

    def _to_grid(self, pts, pose):
        cfg = self.cfg
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        fx, fy = c * dx + s * dy, -s * dx + c * dy
        return np.floor((cfg.ahead - fx) / cfg.res).astype(int), np.floor((cfg.side - fy) / cfg.res).astype(int)

    def _cone_cells(self, pose, t):
        """锥桶地标画成实心圆:返回 (row, col)。"""
        cones = self.cones(t)
        if not len(cones):
            return np.zeros(0, int), np.zeros(0, int)
        rad = TRACK.cone_base_radius
        k = int(math.ceil(rad / self.cfg.res))
        off = np.array([(i, j) for i in range(-k, k + 1) for j in range(-k, k + 1)
                        if (i * i + j * j) * self.cfg.res ** 2 <= rad * rad])
        row, col = self._to_grid(cones, pose)
        row = (row[:, None] + off[None, :, 0]).ravel()
        col = (col[:, None] + off[None, :, 1]).ravel()
        h, w = self.cfg.shape
        ok = (row >= 0) & (row < h) & (col >= 0) & (col < w)
        return row[ok], col[ok]

    def _window(self, pose, t):
        cfg = self.cfg
        x, y, yaw = pose
        # 键按 ix 有序:先二分切出 ix 落在窗口外接圆内的一段,只算这一段(地图再大,每拍也只碰车附近)
        reach = math.hypot(max(cfg.ahead, cfg.behind), cfg.side) + 2 * cfg.res
        lo = np.searchsorted(self.keys, _keys(np.array([math.floor((x - reach) / cfg.res)]), np.array([-_SHIFT]))[0])
        hi = np.searchsorted(self.keys, _keys(np.array([math.floor((x + reach) / cfg.res) + 1]), np.array([-_SHIFT]))[0])
        ix, iy = _unkey(self.keys[lo:hi])
        near = np.abs((iy + 0.5) * cfg.res - y) <= reach
        if t is not None:
            near &= ((t - self.last[lo:hi] <= cfg.max_age) & (self.count[lo:hi] > 0)).any(axis=1)
        sel = lo + np.flatnonzero(near)
        pts = np.stack([(ix[near] + 0.5) * cfg.res, (iy[near] + 0.5) * cfg.res], axis=1)
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        fx, fy = c * dx + s * dy, -s * dx + c * dy        # 车体系:前、左
        h, w = cfg.shape
        row = np.floor((cfg.ahead - fx) / cfg.res).astype(int)
        col = np.floor((cfg.side - fy) / cfg.res).astype(int)
        ok = (row >= 0) & (row < h) & (col >= 0) & (col < w)
        return sel[ok], row[ok], col[ok]

    def _crop_free(self, pose, t):
        """目标格中心反查 odom 格的 FREE 证据,避免旋转正向投点的重复落格/漏格。

        只采已有源格;未知或该层过期仍为 0。稀疏障碍层继续用 _window 正向投点,
        不使用这条路径,避免最近邻采样漏掉单个障碍回波。
        """
        cfg = self.cfg
        out = np.zeros(cfg.shape, bool)
        layer = CAMERA_LAYER[CellType.FREE]
        if not len(self.keys) or not self.count[:, layer].any():
            return out
        row, col = np.indices(cfg.shape)
        xy = np.column_stack(((cfg.ahead - (row.ravel() + 0.5) * cfg.res),
                              (cfg.side - (col.ravel() + 0.5) * cfg.res)))
        world = body_to_odom(xy, pose)
        query = _keys(np.floor(world[:, 0] / cfg.res), np.floor(world[:, 1] / cfg.res))
        pos = np.searchsorted(self.keys, query)
        found = pos < len(self.keys)
        found[found] &= self.keys[pos[found]] == query[found]
        dest = np.flatnonzero(found)
        src = pos[dest]
        hit = self.count[src, layer] > 0
        if t is not None:
            hit &= t - self.last[src, layer] <= cfg.max_age
        out.ravel()[dest] = hit
        return out

    def crop(self, pose, t: float | None = None) -> np.ndarray:
        """以车(后轴)为中心、车头朝上的类型图 (H, W) int8,值是 CellType。
        t 给了就只画 max_age 以内的格子(不必先 prune)。"""
        img = np.zeros(self.cfg.shape, np.int8)
        img[self._crop_free(pose, t)] = int(CellType.FREE)
        if len(self.keys):
            idx, row, col = self._window(pose, t)
            count = self.count[idx]
            if t is not None:
                count = np.where(t - self.last[idx] <= self.cfg.max_age, count, 0)
            for kind in reversed(PRIORITY):
                if kind == CellType.FREE:
                    continue                         # FREE 已按目标格中心采样,其它类型优先覆盖
                layer = LIDAR_LAYER if kind == CellType.OBSTACLE else CAMERA_LAYER[kind]
                hit = count[:, layer] > 0
                img[row[hit], col[hit]] = int(kind)
        row, col = self._cone_cells(pose, t)
        img[row, col] = int(CellType.CONE)            # 锥桶优先级最高
        return img

    def crop_layers(self, pose, t: float | None = None) -> np.ndarray:
        """同一窗口的各来源层 (len(LAYERS), H, W) float32,值 = 有没有证据(0/1)。给网络当输入用。"""
        out = np.zeros((len(LAYERS),) + self.cfg.shape, np.float32)
        out[CAMERA_LAYER[CellType.FREE]] = self._crop_free(pose, t)
        if len(self.keys):
            idx, row, col = self._window(pose, t)
            for k in range(len(LAYERS)):
                if k == CAMERA_LAYER[CellType.FREE]:
                    continue
                hit = self.count[idx, k] > 0
                if t is not None:
                    hit &= t - self.last[idx, k] <= self.cfg.max_age
                out[k, row[hit], col[hit]] = 1.0
        row, col = self._cone_cells(pose, t)
        out[CAMERA_LAYER[CellType.CONE], row, col] = 1.0
        return out


class PoseIntegrator:
    """编码器车速 + 陀螺偏航角速度 -> odom 位姿(后轴中心)。实车没有真位姿,只能这样推。

    中点法积分:这一步的航向取前后两个航向的平均,转弯时比欧拉法少一阶误差。
    """

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def step(self, v: float, yaw_rate: float, dt: float):
        mid = self.yaw + 0.5 * yaw_rate * dt
        self.x += v * dt * math.cos(mid)
        self.y += v * dt * math.sin(mid)
        self.yaw = (self.yaw + yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
