"""影子模式(vehicle/bev_shadow.py)的问题记录本:按"段"累积全局地图(雷达 + 摄像头地面,不过期)、路径、模型每拍想怎么走,
以及人点"标记"时的现场(摄像头画面 + BEV 输入 + 决策)。

抱车会让里程计和实际位置脱节,所以一段 = 车一直在地上;检测到被抱起来(IMU 倾斜 / 晃动 / 加速度不对)
就结束这一段,放下静止 SETTLE 秒后从零开新的一段(调用方要同时清空 BEV 运行时)。也可以手动分段。
每段存一个目录:seg.npz(网格 + 路径 + 标记)、map.png(全局图)、mark_NN.jpg(标记现场);
开车时(road_car_node)另存逐帧回看:frames.npz(每 FRAME_DT 秒一帧的 BEV 输入 + 决策 + 事件)和
frames/cam_NNNN.jpg(地面分割调试图,有新的才存),用 evaluation/logbook_frames.py 画成视频。
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
FRAME_DT = 0.3          # 逐帧回看每这么久记一帧(BEV 输入 float16 6×80×80 ≈ 77 KB,压缩后一分钟几 MB)
CLEAR_STOP = 0.10       # 雷达射线穿过的格子记"空",离回波点这么近的不记(斜扫墙面时别把墙自己擦掉)
CLAMP = 6               # 证据封顶:最多 6 帧相反的观测就能把一格翻过来
OCC_ON = 2              # 雷达证据 >= 这个才画成障碍(只被扫到一次的噪点不画)
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
    """按 CELL 栅格的证据值(int8,夹在 ±CLAMP),稠密数组,碰到边就往外扩。
    每帧"看见有"的格 +1、"看见没有"的格 -1(一帧里同一格只算一次,两边都有算"有")。证据封顶,
    所以停着不动扫了几百帧的格子,也只要几帧相反的观测就翻过来 —— 扫错的点不会一直留在图上。"""

    def __init__(self):
        self.a = np.zeros((0, 0), np.int8)
        self.i0 = self.j0 = 0

    def _fit(self, ij):
        lo, hi = ij.min(0), ij.max(0)
        if self.a.size == 0:
            self.i0, self.j0 = int(lo[0]) - 100, int(lo[1]) - 100
            self.a = np.zeros((int(hi[0] - lo[0]) + 201, int(hi[1] - lo[1]) + 201), np.int8)
        pi0, pj0 = max(0, self.i0 - int(lo[0])), max(0, self.j0 - int(lo[1]))
        pi1 = max(0, int(hi[0]) - (self.i0 + self.a.shape[0] - 1))
        pj1 = max(0, int(hi[1]) - (self.j0 + self.a.shape[1] - 1))
        if pi0 or pj0 or pi1 or pj1:                     # 多扩 100 格(5 m),别每帧都扩
            pi0, pj0, pi1, pj1 = [v + 100 if v else 0 for v in (pi0, pj0, pi1, pj1)]
            self.a = np.pad(self.a, ((pi0, pi1), (pj0, pj1)))
            self.i0, self.j0 = self.i0 - pi0, self.j0 - pj0

    def update(self, yes_ij, no_ij):
        both = np.concatenate([yes_ij, no_ij])
        if len(both) == 0:
            return
        self._fit(both)
        w = self.a.shape[1]
        flat = lambda ij: np.unique((ij[:, 0] - self.i0) * w + (ij[:, 1] - self.j0))
        yes, no = flat(yes_ij), flat(no_ij)
        no = no[~np.isin(no, yes, assume_unique=True)]
        v = self.a.reshape(-1)
        v[yes] = np.minimum(v[yes] + 1, CLAMP)
        v[no] = np.maximum(v[no] - 1, -CLAMP)

    def get(self, k, default=0):
        i, j = k[0] - self.i0, k[1] - self.j0
        if 0 <= i < self.a.shape[0] and 0 <= j < self.a.shape[1]:
            return int(self.a[i, j])
        return default

    def __getitem__(self, k):
        return self.get(k)

    def ij(self, at_least=None):
        """非零格(或证据 >= at_least 的格)的 (i, j) 和证据值。"""
        i, j = np.nonzero(self.a if at_least is None else self.a >= at_least)
        return np.stack([i + self.i0, j + self.j0], 1).astype(np.int32), self.a[i, j]


def _cells(xy):
    xy = np.asarray(xy, np.float64).reshape(-1, 2)
    return np.round(xy / CELL).astype(np.int64)


class Logbook:
    def __init__(self, root, fov_body=None):
        """fov_body:摄像头地面分割管得到的地面点(车体系,(n, 2));给了才会把"在视野里但没被分成地面"的格减分。"""
        self.root = root
        self.fov = None if fov_body is None else np.asarray(fov_body, np.float64).reshape(-1, 2)
        os.makedirs(root, exist_ok=True)
        self.seg = 0
        self.active = False
        self._new()

    def _new(self):
        self.seg += 1
        self.lidar, self.free = Grid(), Grid()
        self.path, self.marks = [], []
        self.frames, self.cams, self.last_cam = [], {}, None
        self.last_path_t = self.last_frame_t = -1e9
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

    def add_lidar(self, xy_odom, origin):
        """一帧雷达回波(odom 系)+ 雷达此刻的位置。回波格 +1,射线穿过的格 -1:
        人走过留下的一串回波,之后雷达再扫过那片地方时被压回去,图上就不会留成一堵墙。"""
        if not self.active or len(xy_odom) == 0:
            return
        p = np.asarray(xy_odom, np.float64)
        o = np.asarray(origin, np.float64)
        d = p[::CLEAR_EVERY] - o
        r = np.hypot(d[:, 0], d[:, 1])
        s = np.arange(0.0, float(r.max()), CELL * 0.7, dtype=np.float32)  # 沿射线每 0.7 格取一点
        ok = s[None, :] < (r - CLEAR_STOP)[:, None]
        u = (d / np.maximum(r, 1e-9)[:, None]).astype(np.float32)
        q = o.astype(np.float32) + u[:, None, :] * s[None, :, None]
        self.lidar.update(_cells(p), _cells(q[ok]))

    def add_free(self, xy_body, pose):
        """一帧地面分割(车体系地面点 + 拍摄时刻位姿)。地面格 +1,视野里其余的格 -1。"""
        if not self.active:
            return
        from common.bev import body_to_odom
        yes = _cells(body_to_odom(xy_body, pose)) if len(xy_body) else np.zeros((0, 2), np.int64)
        no = _cells(body_to_odom(self.fov, pose)) if self.fov is not None else np.zeros((0, 2), np.int64)
        self.free.update(yes, no)

    def add_pose(self, t, pose, steer, target, speed):
        if self.active and t - self.last_path_t >= PATH_DT:
            self.last_path_t = t
            self.path.append((t, *pose, float('nan') if steer is None else steer, target, speed))

    def add_frame(self, t, pose, steer, target, speed, event, bev_img, cam_jpg=None):
        """逐帧回看:模型这一拍的 BEV 输入 + 决策 + 事件(None / stale / guard / stuck);cam_jpg = 最新一张地面分割调试图。"""
        if not self.active or bev_img is None or t - self.last_frame_t < FRAME_DT:
            return
        self.last_frame_t = t
        if cam_jpg is not None and cam_jpg is not self.last_cam:
            self.cams[len(self.frames)] = cam_jpg
            self.last_cam = cam_jpg
        self.frames.append((t, *pose, float('nan') if steer is None else steer, target, speed, event or '',
                            np.asarray(bev_img, np.float16)))

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
        (lk, ln), (fk, fn) = self.lidar.ij(), self.free.ij()
        np.savez(os.path.join(self.dir(), 'seg.npz'), cell=CELL,             # *_n = 证据值(±CLAMP)
                 lidar_ij=lk, lidar_n=ln, free_ij=fk, free_n=fn,
                 path=np.array(self.path, np.float64))        # t x y yaw steer target speed
        with open(os.path.join(self.dir(), 'marks.json'), 'w', encoding='utf-8') as f:
            json.dump(self.marks, f, ensure_ascii=False, indent=1)
        if self.frames:
            fd = os.path.join(self.dir(), 'frames')
            os.makedirs(fd, exist_ok=True)
            for k, jpg in self.cams.items():
                fn = os.path.join(fd, f'cam_{k:04d}.jpg')
                if not os.path.exists(fn):
                    with open(fn, 'wb') as f:
                        f.write(jpg)
            have = sorted(self.cams)
            cam = [max([k for k in have if k <= i], default=-1) for i in range(len(self.frames))]
            F = self.frames
            np.savez_compressed(os.path.join(self.dir(), 'frames.npz'),
                                num=np.array([f[:7] for f in F], np.float64),      # t x y yaw steer target speed
                                event=np.array([f[7] for f in F]), bev=np.stack([f[8] for f in F]),
                                cam=np.array(cam, np.int32))                         # 这一帧对应的 cam_NNNN.jpg(-1 = 没有)

    def render(self, cv2, px=2, dot=None):
        """dot:每格只画中间 dot×dot 像素的点(格子之间留白,点阵风格,px 要比 dot 大),默认整格涂满。
        当前段的全局图(BGR):黑 = 雷达障碍(证据 >= OCC_ON),浅绿 = 摄像头地面(证据 > 0),蓝线 = 路径,
        黄短线 = 模型想打的方向(每 1 s 一根),红圈 = 标记编号,三角 = 车现在的位置。"""
        occ, fr = self.lidar.ij(OCC_ON)[0], self.free.ij(1)[0]
        P = np.array([(x, y) for _, x, y, *_ in self.path]).reshape(-1, 2)
        a = np.concatenate([occ, fr, np.round(P / CELL).astype(np.int32)])
        if not len(a):
            return np.full((200, 300, 3), 255, np.uint8)
        i0, j0 = a.min(0) - 20
        i1, j1 = a.max(0) + 20
        h, w = int((i1 - i0 + 1) * px), int((j1 - j0 + 1) * px)
        img = np.full((h, w, 3), 255, np.uint8)
        to = lambda i, j: (int((j1 - j) * px), int((i1 - i) * px))     # x 朝上、y 朝左
        lo = 0 if dot is None else (px - dot) // 2
        hi = px if dot is None else lo + dot
        for cells, col in ((fr, (190, 235, 190)), (occ, (30, 30, 30))):
            for dv in range(lo, hi):
                for du in range(lo, hi):
                    img[(i1 - cells[:, 0]) * px + dv, (j1 - cells[:, 1]) * px + du] = col
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
