"""锥桶地标定位(Issue RaceCar_Main#12):位姿直接由"这一帧看到的锥桶"和"地图里的锥桶"的相对几何关系求出来,
每隔几帧做一次扫描对地图配准,不靠 IMU / 编码器积分的位置。

    extract_cones(ranges, angles)   一帧雷达 -> 车体系(后轴原点,x 前 y 左)锥桶中心
    register(obs, ref, guess)       车体系锥桶对地图锥桶的最近邻 + 闭式 SE(2) 迭代配准 -> 位姿
    ConeMap                         第一圈:每次配准后把观测并进地图(匹配上的更新均值,没匹配上的加新桶)

两次配准之间只需要一个初值让最近邻关联对得上:默认按前两次配准的位姿做匀速外推;也可以传里程计增量当初值,
但输出位姿始终是配准结果。关联门限要小于半个锥桶间距(实车 realA 中位 0.9 m,仿真官方图 0.5 m、最密 0.33 m),所以初值误差要在 ~0.15 m 以内。
纯 numpy / scipy,车上、仿真、RaceCar_Main 共用。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from common.params import CarParams, TrackParams

_CP, _TP = CarParams(), TrackParams()
# 锥桶在雷达扫描高度处的半径:底 0.14、顶 0.025、高 0.70 线性收窄,0.28 m 高处 ~0.094
CONE_R_AT_LIDAR = 0.14 - (0.14 - _TP.cone_top_radius) * _CP.lidar_z / _TP.cone_height
CLUSTER_GAP = 0.12      # 相邻回波点距离大于这个就断开成两簇 m
CONE_W_MAX = 0.30       # 簇的弦长不超过这个才当锥桶(直径 ~0.19,留余量);更长的是挡板 / 墙
MIN_POINTS = 2
MAX_RANGE = 6.0         # 只用这么近的桶:远处点少、角度误差放大


def extract_cones(ranges, angles, lidar_x=_CP.lidar_x, max_range=MAX_RANGE, cone_r=CONE_R_AT_LIDAR):
    """一帧扫描 -> (k, 2) 车体系锥桶中心。按角度顺序分簇,弦长小的当锥桶;中心 = 簇质心沿视线往外推 cone_r。"""
    r = np.asarray(ranges, float)
    a = np.asarray(angles, float)
    ok = np.isfinite(r) & (r < max_range)
    if ok.sum() < MIN_POINTS:
        return np.zeros((0, 2))
    idx = np.flatnonzero(ok)
    p = np.c_[r[idx] * np.cos(a[idx]), r[idx] * np.sin(a[idx])]
    # 断点:相邻有效点距离过大,或中间夹着无效点(数组下标不连续)
    jump = (np.hypot(*np.diff(p, axis=0).T) > CLUSTER_GAP) | (np.diff(idx) > 1)
    starts = np.r_[0, np.flatnonzero(jump) + 1]
    ends = np.r_[starts[1:], len(p)]
    groups = [np.arange(s, e) for s, e in zip(starts, ends)]
    # 扫描首尾相接(-π / +π):首尾两簇连着就合并
    if len(groups) > 1 and idx[0] == 0 and idx[-1] == len(r) - 1 and \
            np.hypot(*(p[0] - p[-1])) <= CLUSTER_GAP:
        groups[0] = np.r_[groups[-1], groups[0]]
        groups.pop()
    out = []
    for g in groups:
        if len(g) < MIN_POINTS:
            continue
        q = p[g]
        if np.hypot(*(q[0] - q[-1])) > CONE_W_MAX:
            continue
        c = q.mean(0)
        d = np.hypot(*c)
        out.append(c * (d + cone_r) / max(d, 1e-6))
    if not out:
        return np.zeros((0, 2))
    return np.asarray(out) + (lidar_x, 0.0)


def to_world(pts, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    p = np.asarray(pts, float).reshape(-1, 2)
    return np.c_[x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]]


def compose(a, b):
    """位姿 a ∘ 增量 b(b 在 a 的车体系里)。"""
    x, y = to_world([b[:2]], a)[0]
    return (float(x), float(y), float((a[2] + b[2] + math.pi) % (2 * math.pi) - math.pi))


def relative(a, b):
    """b 相对 a 的增量(在 a 的车体系里),compose(a, relative(a, b)) == b。"""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, (b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi)


def _fit_se2(src, dst):
    """最小二乘刚体变换 src -> dst(2D Umeyama,无尺度)。返回 (tx, ty, θ)。"""
    ms, md = src.mean(0), dst.mean(0)
    h = (src - ms).T @ (dst - md)
    th = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, s = math.cos(th), math.sin(th)
    t = md - np.array([c * ms[0] - s * ms[1], s * ms[0] + c * ms[1]])
    return float(t[0]), float(t[1]), th


@dataclass
class Match:
    pose: tuple            # 配准后的位姿(地图系)
    n: int                 # 用上的锥桶对数
    rms: float             # 配准残差 m
    ok: bool               # 对数和残差都合格
    pairs: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), int))   # (obs 下标, 地图下标)


GATES = (0.3, 0.25, 0.2, 0.15, 0.15, 0.15)  # 逐轮收紧的关联门限 m:要小于锥桶半个间距(仿真官方图最密 0.33 m)
MIN_PAIRS = 3
MAX_RMS = 0.12


def register(obs, ref, guess, tree=None, gates=GATES, min_pairs=MIN_PAIRS, max_rms=MAX_RMS) -> Match:
    """车体系锥桶 obs 对地图锥桶 ref 配准:按当前位姿投到地图,最近邻 + 门限关联(一对一),闭式解 SE(2),迭代。"""
    obs = np.asarray(obs, float).reshape(-1, 2)
    if len(obs) < min_pairs or len(ref) < min_pairs:
        return Match(tuple(guess), 0, math.inf, False)
    tree = tree or cKDTree(ref)
    pose = tuple(guess)
    pairs = np.zeros((0, 2), int)
    for gate in gates:
        d, j = tree.query(to_world(obs, pose), distance_upper_bound=gate)
        i = np.flatnonzero(np.isfinite(d))
        if len(i):     # 一对一:同一个地图桶被多个观测抢,只留最近的
            order = np.argsort(d[i])
            _, first = np.unique(j[i][order], return_index=True)
            i = i[order][first]
        if len(i) < min_pairs:
            return Match(pose, len(i), math.inf, False)
        pose = _fit_se2(obs[i], ref[j[i]])
        pairs = np.c_[i, j[i]]
    res = np.hypot(*(to_world(obs[pairs[:, 0]], pose) - ref[pairs[:, 1]]).T)
    rms = float(np.sqrt(np.mean(res ** 2)))
    return Match(pose, len(pairs), rms, rms <= max_rms, pairs)


NEW_CONE_MIN = 0.3     # 离地图上所有桶都比这个远的观测才加成新桶 m:两个桶中心不会比底座直径 0.28 更近
CONFIRM = 2             # 被看到这么多次才参与配准(单帧误检不进地图)
# 有里程计增量当初值时,配准结果离初值超过这个就当锁到了隔壁桶(等间距成排时残差照样小):
# 里程计几帧之内的增量误差只有厘米级,跳半个桶距只能是错配
MAX_JUMP = (0.2, math.radians(5.0))


class ConeMap:
    """第一圈在线建的锥桶地图:均值 + 被看到的次数。"""

    def __init__(self):
        self.pts = np.zeros((0, 2))
        self.n = np.zeros(0, int)

    def confirmed(self):
        m = self.n >= CONFIRM
        return self.pts[m], np.flatnonzero(m)

    def add(self, obs_world, matched_ids=()):
        """matched_ids:(观测下标, 地图下标) —— 匹配上的更新均值;没匹配上、又离所有桶够远的加新桶。"""
        obs_world = np.asarray(obs_world, float).reshape(-1, 2)
        used = set()
        for oi, mi in matched_ids:
            k = self.n[mi]
            self.pts[mi] = (self.pts[mi] * k + obs_world[oi]) / (k + 1)
            self.n[mi] = k + 1
            used.add(int(oi))
        rest = [k for k in range(len(obs_world)) if k not in used]
        if not rest:
            return
        cand = obs_world[rest]
        if len(self.pts):
            d, j = cKDTree(self.pts).query(cand)
            near = d < NEW_CONE_MIN
            for c, jj in zip(cand[near], j[near]):        # 离已有桶近但没被配准选中的:当同一个桶再看一次
                k = self.n[jj]
                self.pts[jj] = (self.pts[jj] * k + c) / (k + 1)
                self.n[jj] = k + 1
            cand = cand[~near]
        if len(cand):
            # 同一帧里互相很近的新观测只加一个
            keep = []
            for c in cand:
                if all(np.hypot(*(c - q)) >= NEW_CONE_MIN for q in keep):
                    keep.append(c)
            self.pts = np.vstack([self.pts, keep])
            self.n = np.r_[self.n, np.ones(len(keep), int)]


class ConeLocalizer:
    """第一次 step 的扫描必须是在 pose0 拍的(第一圈用它定义地图系)。每 every 帧做一次配准。build=True(第一圈)边配准边建图;False(第二圈)只对固定地图配准。
    初值:有里程计增量就用它,否则按前两次配准结果匀速外推。"""

    def __init__(self, pose0, every=3, cmap: ConeMap | None = None, build=True, prev=None):
        """prev:上一次配准的位姿(交接时带过来,匀速外推才有速度;不给就当原地不动)。"""
        self.pose, self.prev, self.every, self.build = tuple(pose0), prev, every, build
        self.map = cmap or ConeMap()
        self.k, self.fails, self.last = 0, 0, None
        self._tree, self._ids = None, None
        if not build:
            self._freeze()

    def _freeze(self):
        ref, ids = self.map.confirmed() if self.build else (self.map.pts, np.arange(len(self.map.pts)))
        self._ref, self._ids = ref, ids
        self._tree = cKDTree(ref) if len(ref) else None

    def finish_map(self):
        """第一圈结束:只留确认过的桶,之后只对这张固定地图配准。位姿、速度、帧计数原样保留(交接不新建定位器,
        否则两次配准之间走的那段会丢,初值落后一个桶距就整圈锁在隔壁桶上)。"""
        keep = self.map.n >= CONFIRM
        self.map.pts, self.map.n = self.map.pts[keep], self.map.n[keep]
        self.build = False
        self._freeze()

    def guess(self, odom_delta=None):
        if odom_delta is not None:
            return compose(self.pose, odom_delta)
        if self.prev is None:
            return self.pose
        return compose(self.pose, relative(self.prev, self.pose))

    def step(self, ranges, angles, odom_delta=None):
        """一帧扫描。odom_delta = 上次配准以来的里程计增量(可选,只当初值)。返回本帧位姿和是否做了配准。"""
        self.k += 1
        if self.k % self.every:
            return self.pose, None
        obs = extract_cones(ranges, angles)
        g = self.guess(odom_delta)
        if self.build and self._tree is None and len(self.map.pts) == 0:
            self.map.add(to_world(obs, self.pose))      # 第一帧:定义地图系
            self.map.n[:] = CONFIRM
            self._freeze()
            return self.pose, None
        m = register(obs, self._ref, g, self._tree) if self._tree is not None else Match(g, 0, math.inf, False)
        if m.ok and odom_delta is not None:
            d = relative(g, m.pose)
            if math.hypot(d[0], d[1]) > MAX_JUMP[0] or abs(d[2]) > MAX_JUMP[1]:
                m = Match(m.pose, m.n, m.rms, False, m.pairs)
        self.last = m
        if m.ok:
            self.prev, self.pose = self.pose, m.pose
        else:
            self.fails += 1
            self.prev, self.pose = self.pose, g          # 配不上:先按初值走,下次再配
        if self.build:
            pairs = [(int(o), int(self._ids[j])) for o, j in m.pairs] if m.ok else []
            self.map.add(to_world(obs, self.pose), pairs)
            self._freeze()
        return self.pose, m
