"""对数概率占据栅格:用仿真的完整 /scan 建图,原理与车上 gmapping 的建图部分一致。

建图吃的是**整圈 1667 点**的原始扫描,与策略观测(48 维扇区)无关 —— 实车上也是
gmapping 单独订阅 /scan,策略只拿自己那份降采样。位姿直接用仿真真值:本项目要
评的是"跑一圈能不能把赛道扫全",不是 SLAM 的定位精度。
"""
from __future__ import annotations

import math

import numpy as np

FREE, UNKNOWN, OCCUPIED = 0, -1, 100


class OccupancyMap:
    def __init__(self, half_extent=12.0, resolution=0.1, hit=0.85, miss=-0.4, clamp=6.0,
                 occupied_threshold=1.0, free_threshold=-1.0):
        self.res = float(resolution)
        self.n = int(2 * half_extent / self.res)
        self.origin = -half_extent
        self.log_odds = np.zeros((self.n, self.n))
        self.hit, self.miss, self.clamp = hit, miss, clamp
        self.occupied_threshold, self.free_threshold = occupied_threshold, free_threshold

    def _cells(self, xy) -> np.ndarray:
        return np.floor((np.asarray(xy, float).reshape(-1, 2) - self.origin) / self.res).astype(int)

    def _inside(self, cells) -> np.ndarray:
        return (cells >= 0).all(axis=1) & (cells < self.n).all(axis=1)

    def integrate(self, origin_xy, angles, ranges, max_range):
        """一帧扫描:命中点 +hit,射线经过的格子 +miss(逐格采样,步长半格)。"""
        finite = np.isfinite(ranges)
        r = np.where(finite, ranges, max_range)
        ends = np.asarray(origin_xy, float) + np.stack([r * np.cos(angles), r * np.sin(angles)], axis=1)

        steps = int(max_range / (self.res * 0.5)) + 1
        t = np.linspace(0.0, 1.0, steps)[None, :, None]
        along = np.asarray(origin_xy, float) + t * (ends - origin_xy)[:, None, :]
        keep = (np.linalg.norm(along - origin_xy, axis=2)
                < (r[:, None] - self.res))                     # 命中点前一格之前都算空闲
        free_cells = self._cells(along[keep])
        free_cells = free_cells[self._inside(free_cells)]
        np.add.at(self.log_odds, (free_cells[:, 0], free_cells[:, 1]), self.miss)

        hit_cells = self._cells(ends[finite])
        hit_cells = hit_cells[self._inside(hit_cells)]
        np.add.at(self.log_odds, (hit_cells[:, 0], hit_cells[:, 1]), self.hit - self.miss)
        np.clip(self.log_odds, -self.clamp, self.clamp, out=self.log_odds)

    def grid(self) -> np.ndarray:
        """三值栅格:OCCUPIED / FREE / UNKNOWN。"""
        g = np.full((self.n, self.n), UNKNOWN, np.int8)
        g[self.log_odds >= self.occupied_threshold] = OCCUPIED
        g[self.log_odds <= self.free_threshold] = FREE
        return g

    def cone_coverage(self, cones_xy, radius) -> float:
        """被标成占据的锥桶比例 —— 建图完成度的主判据。"""
        cones = np.asarray(cones_xy, float).reshape(-1, 2)
        g = self.grid()
        reach = int(math.ceil(radius / self.res)) + 1
        found = 0
        for cone in cones:
            c = self._cells(cone)[0]
            lo = np.maximum(c - reach, 0)
            hi = np.minimum(c + reach + 1, self.n)
            if (g[lo[0]:hi[0], lo[1]:hi[1]] == OCCUPIED).any():
                found += 1
        return found / max(1, len(cones))

    def free_area(self) -> float:
        """标成空闲的面积 m²,用来看路面扫没扫开。"""
        return float((self.grid() == FREE).sum()) * self.res ** 2
