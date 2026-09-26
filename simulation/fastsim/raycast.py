"""3D 解析射线求交:竖直圆台(锥桶)与竖直挡板(矩形面)。

赛道上只有这两种东西。需要球体/红绿灯杆时再加。
"""
from __future__ import annotations

import numpy as np


def cast_walls(origin, directions, segments, height, max_range=40.0):
    """射线与竖直挡板(俯视是线段,竖直方向 0..height)的最近交点距离;不命中为 inf。

    segments: (m,2,2) = 每块挡板俯视的两个端点。先在 XY 平面解射线与线段的交点
    (二元一次方程组),再检查交点高度落在 0..height 内。挡板数量只有几块,直接全算。
    """
    origin = np.asarray(origin, dtype=np.float64).reshape(3)
    d = np.asarray(directions, dtype=np.float64).reshape((-1, 3))
    seg = np.asarray(segments, dtype=np.float64).reshape((-1, 2, 2))
    best = np.full(len(d), np.inf)
    if not len(seg):
        return best

    a = seg[:, 0, :]                                  # 起点
    e = seg[:, 1, :] - a                              # 方向向量
    ox, oy = origin[0] - a[:, 0], origin[1] - a[:, 1]   # (m,)
    dx, dy = d[:, 0][:, None], d[:, 1][:, None]         # (n,1)
    denom = dx * e[None, :, 1] - dy * e[None, :, 0]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (ox[None, :] * e[None, :, 1] - oy[None, :] * e[None, :, 0]) / -denom   # 沿射线
        u = (oy[None, :] * dx - ox[None, :] * dy) / denom                          # 沿线段 0..1
        z = origin[2] + t * d[:, 2][:, None]
        ok = (np.abs(denom) > 1e-12) & (t > 1e-9) & (u >= 0) & (u <= 1) & (z >= 0) & (z <= height)
        hit = np.where(ok, t, np.inf).min(axis=1)
    return np.where(hit <= float(max_range), hit, np.inf)


def cast_cones(origin, directions, centres_xy, base_radius, top_radius, height,
               max_range=40.0, cull=True):
    """射线与竖直圆台(底面在 z=0)侧面的最近交点距离;不命中为 inf。

    半径随高度线性收缩 r(z) = r0 - k·z,k = (r0 - r1)/h。沿射线 p = o + t·d 代入
    (px-cx)² + (py-cy)² = r(pz)² 得关于 t 的二次方程,再要求 0 <= pz <= h。

    cull=True 时先按方位角裁剪:圆台整体在底面圆柱内,射线水平投影方位与锥桶方位
    之差超过 asin(r0/距离) 就不可能命中,只对剩下的 (射线, 锥桶) 对解方程。
    结果与 cull=False 逐位相同(测试覆盖),只是快。
    """
    origin = np.asarray(origin, dtype=np.float64).reshape(3)
    d = np.asarray(directions, dtype=np.float64).reshape((-1, 3))
    c = np.asarray(centres_xy, dtype=np.float64).reshape((-1, 2))
    best = np.full(len(d), np.inf)
    if not len(c):
        return best
    # 只算包围圆落在量程内的锥桶
    dist = np.hypot(*(c - origin[:2]).T)
    near = dist - base_radius <= max_range
    c, dist = c[near], dist[near]
    if not len(c):
        return best

    if cull:
        ray_az = np.arctan2(d[:, 1], d[:, 0])
        cone_az = np.arctan2(c[:, 1] - origin[1], c[:, 0] - origin[0])
        with np.errstate(invalid="ignore"):
            half = np.where(dist > base_radius, np.arcsin(np.minimum(1.0, base_radius / dist)), np.pi)
        diff = np.abs((ray_az[:, None] - cone_az[None, :] + np.pi) % (2 * np.pi) - np.pi)
        # 竖直射线没有方位,不能裁
        vertical = (d[:, 0] ** 2 + d[:, 1] ** 2) < 1e-18
        ri, ci = np.nonzero((diff <= half[None, :] + 1e-9) | vertical[:, None])
    else:
        ri, ci = (idx.ravel() for idx in np.meshgrid(np.arange(len(d)), np.arange(len(c)), indexing="ij"))
    if not len(ri):
        return best

    k = (base_radius - top_radius) / height
    a = base_radius - k * origin[2]                       # 射线起点高度处的半径
    dx, dy, dz = d[ri, 0], d[ri, 1], d[ri, 2]
    ox = origin[0] - c[ci, 0]
    oy = origin[1] - c[ci, 1]
    b = k * dz
    qa = dx ** 2 + dy ** 2 - b ** 2
    qb = 2 * (ox * dx + oy * dy + a * b)
    qc = ox ** 2 + oy ** 2 - a ** 2
    with np.errstate(divide="ignore", invalid="ignore"):
        disc = qb ** 2 - 4 * qa * qc
        root = np.sqrt(np.where(disc >= 0, disc, np.nan))
        t_best = np.full(len(ri), np.inf)
        for sign in (-1.0, 1.0):
            # qa≈0(光线与母线平行)时退化为一次方程
            t = np.where(np.abs(qa) > 1e-12, (-qb + sign * root) / (2 * qa), -qc / qb)
            z = origin[2] + t * dz
            ok = (t > 1e-9) & (z >= 0) & (z <= height) & (a - b * t >= 0)
            t_best = np.minimum(t_best, np.where(ok, t, np.inf))
    np.minimum.at(best, ri, t_best)
    return np.where(best <= float(max_range), best, np.inf)
