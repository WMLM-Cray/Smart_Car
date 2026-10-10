"""摄像头地面分割结果 -> 车体系地面点(后轴原点,x 前 y 左)。

只用 CarParams 里的内参、畸变、外参;纯 numpy,不依赖 cv2。畸变只在这一步处理(分割模型直接吃原图),
每个输出格子对应的地面坐标启动时算一次成查找表,每帧只查表。

外参约定(和 2026-10 标定脚本一致):车体向量 (前,左,上) 先绕上轴转 -yaw,再在前/上平面转 -pitch
(pitch 负 = 光轴朝上),再在左/上平面转 roll;相机系 x 右 y 下 z 前。
"""
from __future__ import annotations

import math

import numpy as np

from common.params import CarParams


def cam_from_body(p: CarParams) -> np.ndarray:
    """3×3:相机系向量 = M @ 车体系向量。"""
    cY, sY = math.cos(p.camera_yaw), math.sin(p.camera_yaw)
    cP, sP = math.cos(p.camera_pitch), math.sin(p.camera_pitch)
    cR, sR = math.cos(p.camera_roll), math.sin(p.camera_roll)

    def f(v):
        dF, dL, dU = v
        F1, L1 = dF * cY + dL * sY, -dF * sY + dL * cY
        Fc, Uc = F1 * cP - dU * sP, dU * cP + F1 * sP
        L2, U2 = L1 * cR - Uc * sR, L1 * sR + Uc * cR
        return np.array([-L2, -U2, Fc])
    return np.stack([f(e) for e in np.eye(3)], 1)


def distort(xn, yn, dist):
    k1, k2, p1, p2, k3 = dist
    r2 = xn * xn + yn * yn
    rad = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
    return (xn * rad + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn),
            yn * rad + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn)


def undistort(xd, yd, dist, iters=20):
    """畸变归一化坐标 -> 理想归一化坐标(定点迭代,和 cv2.undistortPoints 同法)。"""
    k1, k2, p1, p2, k3 = dist
    xn, yn = xd.copy(), yd.copy()
    for _ in range(iters):
        r2 = xn * xn + yn * yn
        rad = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
        dx = 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
        dy = p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
        xn, yn = (xd - dx) / rad, (yd - dy) / rad
    return xn, yn


def ground_to_pixel(xy_body, p: CarParams = CarParams()):
    """车体系地面点 (n,2) -> 原图像素 (n,2);相机背后的点给 nan。"""
    M = cam_from_body(p)
    d = np.c_[xy_body[:, 0] - p.camera_x, xy_body[:, 1], np.full(len(xy_body), -p.camera_z)]
    c = d @ M.T
    with np.errstate(divide='ignore', invalid='ignore'):
        xn, yn = c[:, 0] / c[:, 2], c[:, 1] / c[:, 2]
    xd, yd = distort(xn, yn, p.camera_dist)
    fx, fy, cx, cy = p.camera_K
    uv = np.c_[cx + fx * xd, cy + fy * yd]
    uv[c[:, 2] <= 0] = np.nan
    return uv


class GroundLut:
    """分割输出 (rows, cols)(铺满整幅原图)每格中心 -> 车体系地面点。

    max_range:离相机超过这么远的格子不要(俯仰 ±1° 在 3 m 处已是 ±12 cm,再远投影不可信)。
    """

    def __init__(self, rows: int, cols: int, p: CarParams = CarParams(), max_range: float = 3.0):
        W, H = p.camera_size
        fx, fy, cx, cy = p.camera_K
        v, u = np.mgrid[0:rows, 0:cols]
        u = (u.ravel() + 0.5) * W / cols
        v = (v.ravel() + 0.5) * H / rows
        xd, yd = (u - cx) / fx, (v - cy) / fy
        xn, yn = undistort(xd, yd, p.camera_dist)
        # 画面四角超出畸变模型可逆的范围(桶形 k1 很大,迭代不收敛),来回对不上的格子不要
        x2, y2 = distort(xn, yn, p.camera_dist)
        inv = np.hypot((x2 - xd) * fx, (y2 - yd) * fy) < 0.1
        ray = np.c_[xn, yn, np.ones_like(xn)] @ cam_from_body(p)       # 相机系射线 -> 车体系(M 正交)
        down = -ray[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = np.where(down > 1e-6, p.camera_z / down, np.inf)
        gx, gy = p.camera_x + t * ray[:, 0], t * ray[:, 1]
        self.valid = inv & np.isfinite(t) & (np.hypot(gx - p.camera_x, gy) <= max_range)
        self.xy = np.c_[gx, gy]
        self.shape = (rows, cols)

    def points(self, mask: np.ndarray, cell: float = 0.05) -> np.ndarray:
        """mask (rows, cols) bool -> 地面点 (n,2);按 cell 栅格去重(BEV 栅格 5 cm,多了没用)。"""
        sel = mask.ravel().astype(bool) & self.valid
        xy = self.xy[sel]
        if len(xy) == 0:
            return xy
        q = np.unique(np.round(xy / cell).astype(np.int32), axis=0)
        return q.astype(np.float64) * cell
