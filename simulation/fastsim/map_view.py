"""仿真 / 复现出图,和车上 Logbook.render 同一个样式:白底、1 m 浅灰网格、黑点 = 雷达障碍、浅绿点 = 地面、
蓝线 = 路径、黄短线 = 模型想打的方向、三角 = 车。x 朝上、y 朝左(同车上的图)。"""
from __future__ import annotations

import math

import numpy as np

OCC_COL = (30, 30, 30)
FREE_COL = (190, 235, 190)
GRID_COL = (225, 225, 225)
PATH_COL = (200, 90, 20)
STEER_COL = (0, 200, 255)


class MapView:
    def __init__(self, bounds, cell=0.05, px=5, dot=3):
        """bounds = (x0, y0, x1, y1) 世界系 m:画布固定(视频每帧同尺寸)。"""
        self.cell, self.px, self.dot = cell, px, dot
        x0, y0, x1, y1 = bounds
        self.i0, self.j0 = int(math.floor(x0 / cell)), int(math.floor(y0 / cell))
        self.i1, self.j1 = int(math.ceil(x1 / cell)), int(math.ceil(y1 / cell))
        self.h, self.w = (self.i1 - self.i0 + 1) * px, (self.j1 - self.j0 + 1) * px

    def uv(self, xy):
        xy = np.asarray(xy, np.float64).reshape(-1, 2)
        return np.stack([(self.j1 - xy[:, 1] / self.cell) * self.px,
                         (self.i1 - xy[:, 0] / self.cell) * self.px], 1).astype(np.int32)

    def base(self, cv2, occ_ij=(), free_ij=()):
        img = np.full((self.h, self.w, 3), 255, np.uint8)
        px = self.px
        lo = (px - self.dot) // 2
        for cells, col in ((free_ij, FREE_COL), (occ_ij, OCC_COL)):
            c = np.asarray(cells, np.int64).reshape(-1, 2)
            c = c[(c[:, 0] >= self.i0) & (c[:, 0] <= self.i1) & (c[:, 1] >= self.j0) & (c[:, 1] <= self.j1)]
            for dv in range(lo, lo + self.dot):
                for du in range(lo, lo + self.dot):
                    img[(self.i1 - c[:, 0]) * px + dv, (self.j1 - c[:, 1]) * px + du] = col
        step = int(round(1.0 / self.cell))
        for i in range(self.i0 // step * step, self.i1 + 1, step):
            v = (self.i1 - i) * px
            cv2.line(img, (0, v), (self.w, v), GRID_COL, 1)
        for j in range(self.j0 // step * step, self.j1 + 1, step):
            u = (self.j1 - j) * px
            cv2.line(img, (u, 0), (u, self.h), GRID_COL, 1)
        return img

    def path(self, cv2, img, xy, color=PATH_COL, width=2):
        if len(xy) > 1:
            cv2.polylines(img, [self.uv(xy).reshape(-1, 1, 2)], False, color, width, cv2.LINE_AA)

    def steer(self, cv2, img, pose, steer, length=0.4):
        """黄短线:车头朝向 + 前轮转角(模型想往哪打)。"""
        x, y, yaw = pose
        hd = yaw + steer
        a, b = self.uv([(x, y), (x + length * math.cos(hd), y + length * math.sin(hd))])
        cv2.line(img, tuple(int(v) for v in a), tuple(int(v) for v in b), STEER_COL, 2)

    def car(self, cv2, img, pose, color=PATH_COL, size=0.3):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        tri = self.uv([(x + size * c, y + size * s), (x - 0.4 * size * s, y + 0.4 * size * c),
                       (x + 0.4 * size * s, y - 0.4 * size * c)])
        cv2.fillPoly(img, [tri], color)


def bev_tile(cv2, img, size=320):
    """模型输入的 BEV(6×80×80)画成同样配色:浅绿 = 地面层,黑 = 雷达历史,深灰 = 当前帧雷达,蓝点 = 车(后轴)。"""
    t = np.full(img.shape[1:] + (3,), 255, np.uint8)
    t[img[5] > 0] = FREE_COL
    t[img[0] > 0] = OCC_COL
    t[img[4] > 0] = (90, 90, 90)
    t = cv2.resize(t, (size, size), interpolation=cv2.INTER_NEAREST)
    cv2.circle(t, (size // 2, size * 60 // 80), 5, PATH_COL, -1)        # 车在裁图前方 6 m、后方 2 m 处
    return t
