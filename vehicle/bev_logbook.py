"""影子模式的问题记录本(evaluation/shadow_replay.py 回放时用):按"段"累积全局地图(雷达 + 摄像头地面,不过期)、路径、模型每拍想怎么走,
以及人点"标记"时的现场(摄像头画面 + BEV 输入 + 决策)。

抱车会让里程计和实际位置脱节,所以一段 = 车一直在地上;检测到被抱起来(IMU 倾斜 / 晃动 / 加速度不对)
就结束这一段,放下静止 SETTLE 秒后从零开新的一段(调用方要同时清空 BEV 运行时)。也可以手动分段。
每段存一个目录:seg.npz(网格 + 路径 + 标记)、map.png(全局图)、mark_NN.jpg(标记现场)。
"""
from __future__ import annotations

import json
import math
import os
import time

import numpy as np

CELL = 0.05             # 全局图栅格 m
PICKUP_GYRO = 0.35      # 绕横轴/纵轴角速度超过这个 rad/s(地上开车只有颠簸的尖峰)
PICKUP_TILT = 8.0       # 重力方向偏离车体竖直超过这个度
PICKUP_ACC = 1.5        # 加速度模长偏离静止值超过这个 m/s²
PICKUP_HOLD = 0.25      # 上面任一条持续这么久才算被抱起(滤掉过坎的颠簸)
SETTLE = 1.5            # 放下后静止这么久才开新段
PATH_DT = 0.2           # 路径和决策每这么久记一条
CLEAR_STOP = 0.10       # 雷达射线穿过的格子记"空",离回波点这么近的不记(斜扫墙面时别把墙自己擦掉)
CLEAR_EVERY = 4         # 只用每 4 根射线记"空"(实车一帧 ~1400 根,间隔 0.86°,3 m 处 4.5 cm < 一格;车上 ~45 -> ~12 ms)


class PickupDetector:
    def __init__(self):
        self.g0 = None                  # 静止时的加速度向量(开机时学;IMU 本身装歪 ~2°,倾斜按它算)
        self.bad_since = None
        self.quiet_since = None
        self.held = False

    def update(self, wx, wy, ax, ay, az, t):
        """返回 'pickup' / 'settled' / None。"""
        v = np.array([ax, ay, az])
        a = float(np.linalg.norm(v))
        if self.g0 is None:
            self.g0 = v
        g = float(np.linalg.norm(self.g0))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(v @ self.g0) / max(a * g, 1e-6)))))
        bad = abs(wx) > PICKUP_GYRO or abs(wy) > PICKUP_GYRO or tilt > PICKUP_TILT or abs(a - g) > PICKUP_ACC
        if bad:
            self.quiet_since = None
            self.bad_since = t if self.bad_since is None else self.bad_since
            if not self.held and t - self.bad_since >= PICKUP_HOLD:
                self.held = True
                return 'pickup'
        else:
            self.bad_since = None
            self.quiet_since = t if self.quiet_since is None else self.quiet_since
            if self.held and t - self.quiet_since >= SETTLE:
                self.held = False
                return 'settled'
        return None


class Grid:
    """按 CELL 栅格计数的稠密数组,碰到边就往外扩;接口像 {(i, j): 次数} 的 dict(只列非零格)。"""

    def __init__(self):
        self.a = np.zeros((0, 0), np.int32)
        self.i0 = self.j0 = 0

    def add(self, ij, unique_only=False):
        lo, hi = ij.min(0), ij.max(0)
        if self.a.size == 0:
            self.i0, self.j0 = int(lo[0]) - 100, int(lo[1]) - 100
            self.a = np.zeros((int(hi[0] - lo[0]) + 201, int(hi[1] - lo[1]) + 201), np.int32)
        pi0, pj0 = max(0, self.i0 - int(lo[0])), max(0, self.j0 - int(lo[1]))
        pi1 = max(0, int(hi[0]) - (self.i0 + self.a.shape[0] - 1))
        pj1 = max(0, int(hi[1]) - (self.j0 + self.a.shape[1] - 1))
        if pi0 or pj0 or pi1 or pj1:                     # 多扩 100 格(5 m),别每帧都扩
            pi0, pj0, pi1, pj1 = [v + 100 if v else 0 for v in (pi0, pj0, pi1, pj1)]
            self.a = np.pad(self.a, ((pi0, pi1), (pj0, pj1)))
            self.i0, self.j0 = self.i0 - pi0, self.j0 - pj0
        i, j = ij[:, 0] - self.i0, ij[:, 1] - self.j0
        if unique_only:
            flat = np.unique(i * self.a.shape[1] + j)
            self.a.reshape(-1)[flat] += 1
        else:
            np.add.at(self.a, (i, j), 1)

    def get(self, k, default=0):
        i, j = k[0] - self.i0, k[1] - self.j0
        if 0 <= i < self.a.shape[0] and 0 <= j < self.a.shape[1]:
            return int(self.a[i, j])
        return default

    def __getitem__(self, k):
        return self.get(k)

    def ij(self):
        i, j = np.nonzero(self.a)
        return np.stack([i + self.i0, j + self.j0], 1).astype(np.int32), self.a[i, j]

    def items(self):
        k, n = self.ij()
        return zip(map(tuple, k.tolist()), n.tolist())

    def __iter__(self):
        return iter(map(tuple, self.ij()[0].tolist()))


class Logbook:
    def __init__(self, root):
        self.root = root
        os.makedirs(root, exist_ok=True)
        self.seg = 0
        self.active = False
        self._new()

    def _new(self):
        self.seg += 1
        self.lidar, self.miss, self.free = Grid(), Grid(), Grid()
        self.path, self.marks = [], []
        self.last_path_t = -1e9
        self.t0 = time.time()
        self.active = True

    def dir(self):
        return os.path.join(self.root, f'seg_{self.seg:02d}')

    def split(self):
        """结束当前段(有内容才存)并开新段。"""
        self.save()
        self._new()

    def pause(self):
        """被抱起来:存盘,暂停记录,等 resume()。"""
        self.save()
        self.active = False

    def resume(self):
        self._new()

    @staticmethod
    def _add(grid, xy, unique_only=False):
        """unique_only:一帧里同一格只算一次(射线上采样比格密,同一格会数好几遍)。"""
        if len(xy) == 0:
            return
        grid.add(np.round(np.asarray(xy) / CELL).astype(np.int64), unique_only)

    def add_lidar(self, xy_odom, origin):
        """一帧雷达回波(odom 系)+ 雷达此刻的位置。回波格记"有",射线穿过的格记"空":
        人走过留下的一串回波,之后雷达再扫过那片地方时被"空"压过,图上就不会留成一堵墙。"""
        if not self.active or len(xy_odom) == 0:
            return
        p = np.asarray(xy_odom, np.float64)
        self._add(self.lidar, p)
        o = np.asarray(origin, np.float64)
        d = p[::CLEAR_EVERY] - o
        r = np.hypot(d[:, 0], d[:, 1])
        s = np.arange(0.0, float(r.max()), CELL * 0.7, dtype=np.float32)  # 沿射线每 0.7 格取一点
        ok = s[None, :] < (r - CLEAR_STOP)[:, None]
        u = (d / np.maximum(r, 1e-9)[:, None]).astype(np.float32)
        q = o.astype(np.float32) + u[:, None, :] * s[None, :, None]
        self._add(self.miss, q[ok], unique_only=True)

    def add_free(self, xy_odom):
        if self.active:
            self._add(self.free, xy_odom)

    def add_pose(self, t, pose, steer, target, speed):
        if self.active and t - self.last_path_t >= PATH_DT:
            self.last_path_t = t
            self.path.append((t, *pose, float('nan') if steer is None else steer, target, speed))

    def mark(self, t, pose, steer, target, speed, cam_bgr, bev_bgr, cv2):
        """记一个问题现场;返回编号。"""
        if not self.active:
            return None
        k = len(self.marks) + 1
        self.marks.append(dict(k=k, t=t, x=pose[0], y=pose[1], yaw=pose[2], steer=steer, target=target, speed=speed))
        os.makedirs(self.dir(), exist_ok=True)
        tiles = [bev_bgr] + ([cv2.resize(cam_bgr, (bev_bgr.shape[0] * 4 // 3, bev_bgr.shape[0]))] if cam_bgr is not None else [])
        cv2.imwrite(os.path.join(self.dir(), f'mark_{k:02d}.jpg'), np.hstack(tiles))
        self.save()
        return k

    def save(self):
        if not self.path:
            return
        os.makedirs(self.dir(), exist_ok=True)
        (lk, ln), (mk, mn), (fk, fn) = self.lidar.ij(), self.miss.ij(), self.free.ij()
        np.savez(os.path.join(self.dir(), 'seg.npz'), cell=CELL,
                 lidar_ij=lk, lidar_n=ln, miss_ij=mk, miss_n=mn, free_ij=fk, free_n=fn,
                 path=np.array(self.path, np.float64))        # t x y yaw steer target speed
        with open(os.path.join(self.dir(), 'marks.json'), 'w', encoding='utf-8') as f:
            json.dump(self.marks, f, ensure_ascii=False, indent=1)

    def render(self, cv2, px=2):
        """当前段的全局图(BGR):黑 = 雷达(至少 2 次回波,且回波次数多于被射线穿过的次数),浅绿 = 摄像头地面,蓝线 = 路径,
        黄短线 = 模型想打的方向(每 1 s 一根),红圈 = 标记编号,三角 = 车现在的位置。"""
        pts = [k for k in self.lidar] + [k for k in self.free] + \
              [(round(x / CELL), round(y / CELL)) for _, x, y, *_ in self.path]
        if not pts:
            return np.full((200, 300, 3), 255, np.uint8)
        a = np.array(pts)
        i0, j0 = a.min(0) - 20
        i1, j1 = a.max(0) + 20
        h, w = int((i1 - i0 + 1) * px), int((j1 - j0 + 1) * px)
        img = np.full((h, w, 3), 255, np.uint8)
        to = lambda i, j: (int((j1 - j) * px), int((i1 - i) * px))     # x 朝上、y 朝左
        for (i, j), n in self.free.items():
            u, v = to(i, j)
            img[v:v + px, u:u + px] = (190, 235, 190)
        for (i, j), n in self.lidar.items():
            if n >= 2 and n > self.miss.get((i, j), 0):
                u, v = to(i, j)
                img[v:v + px, u:u + px] = (30, 30, 30)
        m = px / CELL
        step = int(round(1.0 / CELL))
        for i in range(int(i0 // step * step), int(i1) + 1, step):    # 1 m 格
            cv2.line(img, (0, to(i, 0)[1]), (w, to(i, 0)[1]), (225, 225, 225), 1)
        for j in range(int(j0 // step * step), int(j1) + 1, step):
            cv2.line(img, (to(0, j)[0], 0), (to(0, j)[0], h), (225, 225, 225), 1)
        if len(self.path) > 1:
            P = np.array(self.path)
            q = np.array([to(x / CELL, y / CELL) for x, y in P[:, 1:3]])
            cv2.polylines(img, [q.reshape(-1, 1, 2)], False, (200, 90, 20), 2)
            for t, x, y, yaw, steer, target, sp in self.path[::5]:
                if steer == steer:
                    hd = yaw + steer
                    u, v = to(x / CELL, y / CELL)
                    cv2.line(img, (u, v), (int(u - 0.4 * m * math.sin(hd)), int(v - 0.4 * m * math.cos(hd))),
                             (0, 200, 255), 1)
            t, x, y, yaw = self.path[-1][:4]
            u, v = to(x / CELL, y / CELL)
            tri = np.array([(u - 0.3 * m * math.sin(yaw), v - 0.3 * m * math.cos(yaw)),
                            (u - 0.12 * m * math.cos(yaw), v + 0.12 * m * math.sin(yaw)),
                            (u + 0.12 * m * math.cos(yaw), v - 0.12 * m * math.sin(yaw))], np.int32)
            cv2.fillPoly(img, [tri], (200, 90, 20))
        for mk in self.marks:
            u, v = to(mk['x'] / CELL, mk['y'] / CELL)
            cv2.circle(img, (u, v), 9, (40, 40, 220), 2)
            cv2.putText(img, str(mk['k']), (u + 10, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 220), 2)
        cv2.putText(img, f'seg {self.seg}  {len(self.marks)} marks', (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
        return img
