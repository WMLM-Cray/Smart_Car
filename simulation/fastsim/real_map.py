"""实景地图仿真:车上建图(vehicle/bev_logbook 的雷达证据网格)当墙,在网格上打射线出雷达扫描。

真实地图两万多个障碍格,按线段求交太慢(每帧 1667 根射线 × 几万段),这里直接在 5 cm 网格上沿射线按半格步长走,
第一个落进障碍格的点就是回波。每个格子覆盖 [k - 0.5, k + 0.5) · CELL(和 bev_logbook._cells 的 round 一致)。
"""
from __future__ import annotations

import numpy as np


class RealMap:
    def __init__(self, occ_ij: np.ndarray, cell: float = 0.05, free_ij: np.ndarray | None = None,
                 known_ij: np.ndarray | None = None, known_pad: float = 0.3):
        """occ_ij:障碍格 (n, 2) 整数格坐标;free_ij:相机看到过的地面格(可选,给出图判断用);
        known_ij:建图时雷达 / 相机观测过的格(可选),外扩 known_pad 米(远处射线稀,中间有缝)算"见过的范围"。"""
        occ_ij = np.asarray(occ_ij, np.int64).reshape(-1, 2)
        extra = [np.asarray(a, np.int64).reshape(-1, 2) for a in (free_ij, known_ij) if a is not None]
        allc = np.concatenate([occ_ij] + extra)
        self.cell = cell
        self.i0, self.j0 = allc.min(0) - 2
        h, w = allc.max(0) - (self.i0, self.j0) + 3
        self.occ = np.zeros((h, w), bool)
        self.occ[occ_ij[:, 0] - self.i0, occ_ij[:, 1] - self.j0] = True
        self.free = np.zeros((h, w), bool)
        if free_ij is not None:
            f = np.asarray(free_ij, np.int64).reshape(-1, 2)
            self.free[f[:, 0] - self.i0, f[:, 1] - self.j0] = True
        self.known = None
        if known_ij is not None:
            from scipy.ndimage import binary_dilation
            k = np.asarray(known_ij, np.int64).reshape(-1, 2)
            self.known = np.zeros((h, w), bool)
            self.known[k[:, 0] - self.i0, k[:, 1] - self.j0] = True
            r = int(round(known_pad / cell))
            yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
            self.known = binary_dilation(self.known, xx ** 2 + yy ** 2 <= r * r)

    @classmethod
    def from_seg(cls, path: str, occ_on: int | None = None, min_blob: int = 3) -> 'RealMap':
        """从车上 / 回放出来的 seg.npz 建:雷达证据 >= occ_on(默认同车上画图的 OCC_ON)算墙,地面证据 > 0 算地面,
        雷达 / 地面网格里出现过的格都算见过。连通(8 邻)不到 min_blob 格的零散障碍去掉:走动的人、椅子腿、噪点
        (实车 1015 seg_03 走廊正中一个 2 帧证据的孤格,实车从它旁边 25 cm 开过,仿真里每局都在它前面截停)。"""
        from vehicle.bev_logbook import OCC_ON
        z = np.load(path)
        lk, ln = z['lidar_ij'], z['lidar_n']
        fk, fn = z['free_ij'], z['free_n']
        world = cls(lk[ln >= (OCC_ON if occ_on is None else occ_on)], float(z['cell']), fk[fn > 0], np.concatenate([lk, fk]))
        world.drop_small(min_blob)
        return world

    def drop_small(self, min_blob: int):
        """去掉连通(8 邻)不到 min_blob 格的零散障碍。"""
        if min_blob > 1:
            from scipy.ndimage import label
            lab, _ = label(self.occ, np.ones((3, 3)))
            self.occ &= np.bincount(lab.ravel())[lab] >= min_blob

    def clear_swept(self, path: np.ndarray, length: float, width: float, rear: float, step: float = 0.025):
        """实车开过的地方当时没有障碍:path (n, 3) 后轴位姿上车身矩形扫过的障碍格清掉(走动的人、出发时站在车边的人)。
        清完可能剩下被切开的碎格,调用方再 drop_small 一次。"""
        bx, by = np.meshgrid(np.arange(-rear, length - rear + 1e-9, step), np.arange(-width / 2, width / 2 + 1e-9, step),
                             indexing='ij')
        bx, by = bx.ravel(), by.ravel()
        for x, y, yaw in np.asarray(path, np.float64):
            c, s = np.cos(yaw), np.sin(yaw)
            i = np.round((x + c * bx - s * by) / self.cell).astype(np.int64) - self.i0
            j = np.round((y + s * bx + c * by) / self.cell).astype(np.int64) - self.j0
            ok = (i >= 0) & (i < self.occ.shape[0]) & (j >= 0) & (j < self.occ.shape[1])
            self.occ[i[ok], j[ok]] = False

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

    def is_known(self, x, y) -> np.ndarray:
        """这些点在不在建图时见过的范围里(没给 known_ij 就都算见过)。"""
        if self.known is None:
            return np.ones(np.shape(x), bool)
        return self._lookup(self.known, np.asarray(x, np.float64), np.asarray(y, np.float64))[0]

    def is_free(self, x, y) -> np.ndarray:
        """这些点是不是相机看到过的地面(出图 / 没见过 = False)。"""
        return self._lookup(self.free, np.asarray(x, np.float64), np.asarray(y, np.float64))[0]
