"""BEV 模型输入图的预览画法(视频、影子模式网页共用):白底,地面、雷达画成小点,锥桶画成按真实底座大小的
橙色实心圆,车画成黑框。只用 numpy,返回 RGB uint8。

坐标:输入图 80×80、每格 10 cm,车头朝上,后轴在第 60 行(车前 6 m、车后 2 m、左右各 4 m)。
"""
from __future__ import annotations

import math

import numpy as np

CELL = 0.1                  # 输入图每格 m
AHEAD, SIDE = 6.0, 4.0      # 车前、左右各多少 m(= BevConfig)
# 通道 -> (颜色 RGB, 点边长 px);后画的盖先画的
DOTS = [(5, (60, 150, 40), 2), (3, (60, 110, 230), 2), (0, (150, 150, 150), 3), (2, (230, 200, 40), 3),
        (4, (230, 40, 40), 3)]
CONE = (255, 140, 0)
CONE_R = 0.14               # 锥桶底座半径 m(= Road.cone_radius)


def to_px(xy_body, px):
    """车体系 (n, 2) -> 像素 (u, v) 浮点。"""
    xy = np.asarray(xy_body, np.float64).reshape(-1, 2)
    return (SIDE - xy[:, 1]) / CELL * px, (AHEAD - xy[:, 0]) / CELL * px


def render(img, cones_body=None, px: int = 6, car=(-0.15, 0.35, 0.15)) -> np.ndarray:
    """img = bev_image 的输出 (C, 80, 80);cones_body = 锥桶地标在车体系的位置 (n, 2)(不给就按锥桶通道的格子画)。
    car = (车尾, 车头, 半宽) m,后轴为原点。"""
    n = img.shape[1]
    out = np.full((n * px, n * px, 3), 255, np.uint8)
    for ch, col, d in DOTS:
        if ch >= img.shape[0]:
            continue
        r, c = np.nonzero(img[ch] > 0.5)
        o = (px - d) // 2
        for dv in range(d):
            for du in range(d):
                out[r * px + o + dv, c * px + o + du] = col
    if cones_body is None:                                  # 退化:锥桶通道每格中心
        r, c = np.nonzero(img[1] > 0.5)
        cones_body = np.c_[AHEAD - (r + 0.5) * CELL, SIDE - (c + 0.5) * CELL]
    u, v = to_px(cones_body, px)
    rad = CONE_R / CELL * px
    k = int(math.ceil(rad))
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
    disk = xx ** 2 + yy ** 2 <= rad ** 2
    for cu, cv in zip(u, v):
        iu, iv = int(round(cu)), int(round(cv))
        y0, y1, x0, x1 = iv - k, iv + k + 1, iu - k, iu + k + 1
        sy0, sx0 = max(0, -y0), max(0, -x0)
        y0, x0, y1, x1 = max(0, y0), max(0, x0), min(out.shape[0], y1), min(out.shape[1], x1)
        if y1 > y0 and x1 > x0:
            m = disk[sy0:sy0 + y1 - y0, sx0:sx0 + x1 - x0]
            out[y0:y1, x0:x1][m] = CONE
    tail, head, half = car                                  # 车:黑框,2 px
    (ul, ur), (vt, vh) = to_px([[0, half], [0, -half]], px)[0], to_px([[tail, 0], [head, 0]], px)[1]
    ul, ur, vh, vt = int(round(ul)), int(round(ur)), int(round(vh)), int(round(vt))
    out[vh:vt + 1, ul:ul + 2] = out[vh:vt + 1, ur - 1:ur + 1] = 0
    out[vh:vh + 2, ul:ur + 1] = out[vt - 1:vt + 1, ul:ur + 1] = 0
    return out


def cones_in_body(bev, pose, t) -> np.ndarray:
    """BEV 里此刻画得出来的锥桶地标(odom)-> 车体系。"""
    w = bev.cones(t)
    if not len(w):
        return np.zeros((0, 2))
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = w[:, 0] - x, w[:, 1] - y
    return np.c_[c * dx + s * dy, -s * dx + c * dy]
