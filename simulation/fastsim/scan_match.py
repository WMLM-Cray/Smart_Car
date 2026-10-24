"""离线扫描匹配:把录包里每帧雷达对齐到已经建好的图上,修正里程计的漂移,让同一面墙只留一层。

只靠里程计建图时,位姿一点点漂,同一面墙被叠成好几层、变成很厚的一条:在图上打射线总是提前撞墙,
2~4 m 误差中位 -22 cm、4~8 m -63 cm(2026-10-17,map_1015_185021)。

做法(逐帧,点到线 ICP):
  预测   上一帧修正后的位姿 + 这两帧之间的里程计增量
  匹配   这一帧回波(车体系)按预测位姿放进世界系,找图里最近的点;按图点的法向(邻点 PCA)算点到线距离,
         线性化解 (dx, dy, dyaw),迭代几次,配对距离门限从粗到细
  建图   修正后的回波按 MAP_CELL 记次数,每 REBUILD 帧重建一次 KD 树(只要被打到 >= 2 次的格,去掉零星的人和噪点)
  兜底   配上的点太少或者修正量离谱,就信里程计
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree

MAP_CELL = 0.025        # 匹配用的图点格 m(比 5 cm 的仿真格细,墙的位置更准)
REBUILD = 5             # 每这么多帧重建一次 KD 树
LOCAL = 9.0             # 只用当前位置这么远以内的图点 m(回波只取到 8 m)
GATES = (0.30, 0.20, 0.12, 0.08, 0.06, 0.05)     # 每次迭代的配对距离门限 m
MIN_PAIRS = 80
MAX_FIX = (0.25, math.radians(6.0))     # 单帧修正超过这个(位移 m, 转角 rad)就不信,用预测


def compose(a, b):
    """位姿复合 a ∘ b(b 是在 a 的系里的增量)。"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def relative(a, b):
    """b 相对 a 的增量:a ∘ relative(a, b) = b。"""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, b[2] - a[2])


def to_world(pts, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.stack([pose[0] + c * pts[:, 0] - s * pts[:, 1], pose[1] + s * pts[:, 0] + c * pts[:, 1]], 1)


class MatchMap:
    """MAP_CELL 格的命中次数(稠密数组,碰边就扩);匹配只用当前位置 LOCAL 米内、命中 >= min_hits 的格。"""

    def __init__(self):
        self.a = np.zeros((0, 0), np.uint16)
        self.i0 = self.j0 = 0
        self.tree = self.pts = self.normals = None

    def add(self, world):
        ij = np.round(world / MAP_CELL).astype(np.int64)
        lo, hi = ij.min(0), ij.max(0)
        if self.a.size == 0:
            self.i0, self.j0 = int(lo[0]) - 400, int(lo[1]) - 400
            self.a = np.zeros((int(hi[0] - lo[0]) + 801, int(hi[1] - lo[1]) + 801), np.uint16)
        p0, q0 = max(0, self.i0 - int(lo[0])), max(0, self.j0 - int(lo[1]))
        p1 = max(0, int(hi[0]) - (self.i0 + self.a.shape[0] - 1))
        q1 = max(0, int(hi[1]) - (self.j0 + self.a.shape[1] - 1))
        if p0 or q0 or p1 or q1:
            p0, q0, p1, q1 = [v + 400 if v else 0 for v in (p0, q0, p1, q1)]
            self.a = np.pad(self.a, ((p0, p1), (q0, q1)))
            self.i0, self.j0 = self.i0 - p0, self.j0 - q0
        np.add.at(self.a, (ij[:, 0] - self.i0, ij[:, 1] - self.j0), 1)

    def rebuild(self, centre, min_hits=2):
        r = int(LOCAL / MAP_CELL)
        ci, cj = int(round(centre[0] / MAP_CELL)) - self.i0, int(round(centre[1] / MAP_CELL)) - self.j0
        i_lo, j_lo = max(0, ci - r), max(0, cj - r)
        i, j = np.nonzero(self.a[i_lo:ci + r + 1, j_lo:cj + r + 1] >= min_hits)
        if len(i) < 20:
            return
        self.pts = np.stack([i + i_lo + self.i0, j + j_lo + self.j0], 1) * MAP_CELL
        self.tree = cKDTree(self.pts)
        _, nb = self.tree.query(self.pts, k=8)
        q = self.pts[nb] - self.pts[nb].mean(1, keepdims=True)
        _, vec = np.linalg.eigh(np.einsum('nki,nkj->nij', q, q))
        self.normals = vec[:, :, 0]                      # 最小特征值方向 = 墙的法向


def icp(m: MatchMap, body, pose):
    """body:这一帧回波(车体系,后轴原点);pose:预测位姿。返回 (修正后位姿, 配上的点数)。"""
    x, y, yaw = pose
    n = 0
    for gate in GATES:
        w = to_world(body, (x, y, yaw))
        d, j = m.tree.query(w, distance_upper_bound=gate)
        ok = np.isfinite(d)
        n = int(ok.sum())
        if n < MIN_PAIRS:
            return (x, y, yaw), n
        p, q, nr = w[ok], m.pts[j[ok]], m.normals[j[ok]]
        # 点到线:r = n·(p - q),对 (dx, dy, dθ) 线性化(绕车的位置转)
        rel = p - (x, y)
        J = np.stack([nr[:, 0], nr[:, 1], nr[:, 0] * -rel[:, 1] + nr[:, 1] * rel[:, 0]], 1)
        r = np.einsum('ij,ij->i', nr, p - q)
        dx, dy, dt = np.linalg.lstsq(J, -r, rcond=None)[0]
        x, y, yaw = x + dx, y + dy, yaw + dt
    return (x, y, yaw), n


def match_trajectory(body_scans, odom_poses, log_every=0):
    """body_scans:每帧回波(车体系)列表;odom_poses:每帧里程计位姿 (n, 3)。返回修正后的位姿 (n, 3) 和每帧配对数。"""
    m = MatchMap()
    out, pairs = [tuple(odom_poses[0])], [0]
    m.add(to_world(body_scans[0], out[0]))
    m.rebuild(out[0], min_hits=1)
    for k in range(1, len(body_scans)):
        pred = compose(out[-1], relative(odom_poses[k - 1], odom_poses[k]))
        pose, n = icp(m, body_scans[k], pred) if m.tree is not None else (pred, 0)
        fix = relative(pred, pose)
        if n < MIN_PAIRS or math.hypot(fix[0], fix[1]) > MAX_FIX[0] or abs(fix[2]) > MAX_FIX[1]:
            pose = pred
        out.append(pose)
        pairs.append(n)
        m.add(to_world(body_scans[k], pose))
        if k % REBUILD == 0:
            m.rebuild(pose, min_hits=2 if k > 20 else 1)
        if log_every and k % log_every == 0:
            d = relative(odom_poses[k], pose)
            print(f'  {k}/{len(body_scans)} 配对 {n},累计修正 ({d[0]:+.2f}, {d[1]:+.2f}) m {math.degrees(d[2]):+.1f}°')
    return np.array(out), np.array(pairs)
