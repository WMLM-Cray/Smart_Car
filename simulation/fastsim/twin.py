"""数字孪生场景:车上建图(vehicle/bev_logbook 的雷达证据网格)当墙,在网格上打射线出雷达扫描。

真实地图两万多个障碍格,按线段求交太慢(每帧 1667 根射线 × 几万段),这里直接在 5 cm 网格上沿射线按半格步长走,
第一个落进障碍格的点就是回波。每个格子覆盖 [k - 0.5, k + 0.5) · CELL(和 bev_logbook._cells 的 round 一致)。
"""
from __future__ import annotations

import numpy as np


class TwinMap:
    def __init__(self, occ_ij: np.ndarray, cell: float = 0.05, free_ij: np.ndarray | None = None):
        """occ_ij:障碍格 (n, 2) 整数格坐标;free_ij:相机看到过的地面格(可选,给出图判断用)。"""
        occ_ij = np.asarray(occ_ij, np.int64).reshape(-1, 2)
        allc = occ_ij if free_ij is None else np.concatenate([occ_ij, np.asarray(free_ij, np.int64).reshape(-1, 2)])
        self.cell = cell
        self.i0, self.j0 = allc.min(0) - 2
        h, w = allc.max(0) - (self.i0, self.j0) + 3
        self.occ = np.zeros((h, w), bool)
        self.occ[occ_ij[:, 0] - self.i0, occ_ij[:, 1] - self.j0] = True
        self.free = np.zeros((h, w), bool)
        if free_ij is not None:
            f = np.asarray(free_ij, np.int64).reshape(-1, 2)
            self.free[f[:, 0] - self.i0, f[:, 1] - self.j0] = True

    @classmethod
    def from_seg(cls, path: str, occ_on: int | None = None) -> 'TwinMap':
        """从车上 / 回放出来的 seg.npz 建:雷达证据 >= occ_on(默认同车上画图的 OCC_ON)算墙,地面证据 > 0 算地面。"""
        from vehicle.bev_logbook import OCC_ON
        z = np.load(path)
        lk, ln = z['lidar_ij'], z['lidar_n']
        fk, fn = z['free_ij'], z['free_n']
        return cls(lk[ln >= (OCC_ON if occ_on is None else occ_on)], float(z['cell']), fk[fn > 0])

    def _lookup(self, grid, x, y):
        i = np.round(x / self.cell).astype(np.int64) - self.i0
        j = np.round(y / self.cell).astype(np.int64) - self.j0
        ok = (i >= 0) & (i < grid.shape[0]) & (j >= 0) & (j < grid.shape[1])
        out = np.zeros(np.shape(x), bool)
        out[ok] = grid[i[ok], j[ok]]
        return out, ok

    def raycast(self, ox: float, oy: float, world_angles: np.ndarray, max_range: float = 10.0,
                min_range: float = 0.0) -> np.ndarray:
        """从 (ox, oy) 沿各世界系方向打射线 -> 距离(碰不到 / 出图 = inf)。min_range 内的障碍不算(雷达盲区)。"""
        step = self.cell / 2
        s = np.arange(max(min_range, step), max_range + step, step)
        a = np.asarray(world_angles, np.float64)
        x = ox + np.cos(a)[:, None] * s[None, :]
        y = oy + np.sin(a)[:, None] * s[None, :]
        hit, _ = self._lookup(self.occ, x, y)
        first = hit.argmax(1)
        r = s[first] - step / 2                              # 落进障碍格的第一个采样点,往回半步
        r[~hit.any(1)] = np.inf
        return r

    def is_free(self, x, y) -> np.ndarray:
        """这些点是不是相机看到过的地面(出图 / 没见过 = False)。"""
        return self._lookup(self.free, np.asarray(x, np.float64), np.asarray(y, np.float64))[0]
