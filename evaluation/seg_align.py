"""把车上 Logbook 的一段(自己的里程计系)对齐到一张参考图上(比如扫描匹配建好的录包图),求段系 -> 参考系的刚体变换。

    python -m evaluation.seg_align runs/map_1016/matched/seg_01 runs/drv_1015/logbook_1014_204311/seg_04 ...

每段从放下车那一刻起里程计归零,和参考图没有共同的原点,所以先做全局搜索再细化:
  粗   段的障碍格(证据 >= 2、去零散格)按 1° 转一圈,每个角度和参考图的似然场(到最近障碍的距离 -> 高斯)
       在 10 cm 网格上做 FFT 互相关,取得分最高的平移
  细   点到线 ICP(simulation/fastsim/scan_match.icp,参考图点的法向)
报:内点率(段的障碍点离参考图障碍 < 10 cm 的比例)、参考区内点率(只算落在参考图见过范围里的段障碍点;
段的大部分在参考图外时前者被摊薄,看后者)和"唯一性"(离最优解 > 1 m 或 > 10° 的次优粗得分 / 最优,
接近 1 说明有别的位置也差不多 —— 比如长走廊沿走廊方向滑)。
"""
from __future__ import annotations

import argparse
import math
import os

import numpy as np

RES = 0.10              # 粗搜索网格 m
SIGMA = 0.15            # 似然场宽度 m
STEP_DEG = 1.0


def occ_points(seg_dir):
    """段 / 参考图的障碍点(世界系,5 cm 格中心)。"""
    from simulation.fastsim.real_map import RealMap
    w = RealMap.from_seg(os.path.join(seg_dir, 'seg.npz'))
    return (np.argwhere(w.occ) + (w.i0, w.j0)) * w.cell


def apply(T, xy):
    c, s = math.cos(T[2]), math.sin(T[2])
    xy = np.asarray(xy, np.float64).reshape(-1, 2)
    return np.stack([T[0] + c * xy[:, 0] - s * xy[:, 1], T[1] + s * xy[:, 0] + c * xy[:, 1]], 1)


def apply_pose(T, poses):
    """段系位姿 (n, 3) -> 参考系。"""
    poses = np.asarray(poses, np.float64).reshape(-1, 3)
    out = np.empty_like(poses)
    out[:, :2] = apply(T, poses[:, :2])
    out[:, 2] = poses[:, 2] + T[2]
    return out


def _raster(xy, res):
    ij = np.floor(xy / res).astype(np.int64)
    lo = ij.min(0)
    g = np.zeros(tuple(ij.max(0) - lo + 1), np.float32)
    g[ij[:, 0] - lo[0], ij[:, 1] - lo[1]] = 1.0
    return g, lo


def align(src, dst, prior=None, win=(3.0, math.radians(15))):
    """src / dst:障碍点 (n, 2)。prior:先验变换(同一次实车的段共用一个里程计系,换段不归零,只差漂移),
    给了就只在它周围 win = (平移 m, 转角 rad) 里找。返回 (T, 内点率, 唯一性)。"""
    from scipy.ndimage import distance_transform_edt
    from scipy.signal import fftconvolve
    from scipy.spatial import cKDTree
    occ, lo_d = _raster(dst, RES)
    occ = np.pad(occ, 20)
    lo_d = lo_d - 20
    L = np.exp(-0.5 * (distance_transform_edt(occ == 0) * RES / SIGMA) ** 2).astype(np.float32)
    cands = []
    degs = np.arange(0.0, 360.0, STEP_DEG)
    if prior is not None:
        degs = math.degrees(prior[2]) + np.arange(-math.degrees(win[1]), math.degrees(win[1]) + 1e-9, STEP_DEG)
    for deg in degs:
        th = math.radians(deg)
        S, lo_s = _raster(apply((0.0, 0.0, th), src), RES)
        C = fftconvolve(L, S[::-1, ::-1], mode='full')
        if prior is not None:                   # 平移窗外的置零
            k0 = np.round(np.array(prior[:2]) / RES).astype(int) - lo_d + lo_s + (np.array(S.shape) - 1)
            r = int(win[0] / RES)
            mask = np.zeros_like(C, bool)
            mask[max(0, k0[0] - r):k0[0] + r + 1, max(0, k0[1] - r):k0[1] + r + 1] = True
            C = np.where(mask, C, -1.0)
        k = np.unravel_index(np.argmax(C), C.shape)
        shift = lo_d - lo_s + np.array(k) - (np.array(S.shape) - 1)          # 段格 + shift = 参考格
        cands.append((float(C[k]) / len(src), shift * RES, th))
    cands.sort(key=lambda c: -c[0])
    best = cands[0]
    far = [c for c in cands[1:] if np.hypot(*(c[1] - best[1])) > 1.0 or
           abs((c[2] - best[2] + math.pi) % (2 * math.pi) - math.pi) > math.radians(10)]
    unique = far[0][0] / best[0] if far else 0.0
    # 细:点到线 ICP。粗解平移是 10 cm 格对齐的,把格内的半格偏移也算进去
    from simulation.fastsim.scan_match import MatchMap, icp
    m = MatchMap()
    m.pts = dst
    m.tree = cKDTree(dst)
    _, nb = m.tree.query(dst, k=8)
    q = dst[nb] - dst[nb].mean(1, keepdims=True)
    m.normals = np.linalg.eigh(np.einsum('nki,nkj->nij', q, q))[1][:, :, 0]
    T = (best[1][0], best[1][1], best[2])
    for _ in range(3):
        T, _ = icp(m, src, T)
    d, _ = m.tree.query(apply(T, src))
    return tuple(float(v) for v in T), float(np.mean(d < 0.10)), unique


def merged_world(ref_dir, seg_dir, T):
    """参考图见过的地方用参考图(扫描匹配建的,墙准);参考图没见过的地方用这一段自己的图(变换到参考系)。返回 RealMap。
    段的格按 4 个亚格点(±1/4 格)变换后再落格,转过任意角度的墙不会出现能让射线穿过去的斜缝。
    只适合对得准的段(seg_align 报的"参考区内点" >= ~80%);对不准的段(一段里自己的里程计漂移大,
    或者大半在参考图外)直接用段自己的图。
    试过"段里离参考图障碍 > 15 cm 的也留下(那天关着的门)":段自己厚墙的外层也被当成新东西留下,
    1014 seg_03 / seg_04 一出发就截停,更糟(2026-10-17)。"""
    from simulation.fastsim.real_map import RealMap
    ref = RealMap.from_seg(os.path.join(ref_dir, 'seg.npz'))
    seg = RealMap.from_seg(os.path.join(seg_dir, 'seg.npz'))
    sub = np.array([[-0.25, -0.25], [-0.25, 0.25], [0.25, -0.25], [0.25, 0.25]])

    def cells(world, grid, outside_ref=False):
        ij = np.argwhere(grid) + (world.i0, world.j0)
        if world is ref:
            return ij
        xy = apply(T, ((ij[:, None, :] + sub[None]) * world.cell).reshape(-1, 2))
        if outside_ref:
            xy = xy[~ref.is_known(xy[:, 0], xy[:, 1])]
        return np.unique(np.round(xy / ref.cell).astype(np.int64), axis=0)

    occ = np.concatenate([cells(ref, ref.occ), cells(seg, seg.occ, True)])
    free = np.concatenate([cells(ref, ref.free), cells(seg, seg.free, True)])
    known = np.concatenate([cells(ref, ref.known), cells(seg, seg.known)])
    return RealMap(occ, ref.cell, free, known, known_pad=0.0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ref', help='参考图段目录(含 seg.npz)')
    ap.add_argument('segs', nargs='+')
    ap.add_argument('--prior', type=float, nargs=3, default=None, metavar=('X', 'Y', 'DEG'),
                    help='先验变换:只在它周围 3 m / 15° 里找(同一次实车的段)')
    a = ap.parse_args(argv)
    from scipy.spatial import cKDTree
    from simulation.fastsim.real_map import RealMap
    ref = RealMap.from_seg(os.path.join(a.ref, 'seg.npz'))
    dst = occ_points(a.ref)
    tree = cKDTree(dst)
    prior = None if a.prior is None else (a.prior[0], a.prior[1], math.radians(a.prior[2]))
    for s in a.segs:
        src = occ_points(s)
        T, inl, uniq = align(src, dst, prior)
        q = apply(T, src)
        k = ref.is_known(q[:, 0], q[:, 1])
        inl_ref = float(np.mean(tree.query(q[k])[0] < 0.10)) if k.any() else 0.0
        print(f'{s}: T = ({T[0]:+.2f}, {T[1]:+.2f}) m {math.degrees(T[2]):+.1f}°  内点 {inl:.0%}  '
              f'段在参考区 {k.mean():.0%}、其中内点 {inl_ref:.0%}  唯一性 {uniq:.2f}')


if __name__ == '__main__':
    main()
