"""FastSim 里的"摄像头 + YOLO":按真值给出检测结果,喂给 common/bev.py 的 update_camera()。

不渲染图像。实车上摄像头节点做的是"检测框 -> 落地点 -> 车体系坐标",这里直接从真值生成
同样格式的结果,再加上实车会有的毛病:视场与有效距离、漏检、位置误差、偶尔的误检。
误差参数都是 [假设],等实车标定后改 YoloSimConfig。

视场:水平 camera_hfov、垂直 camera_vfov(队友给的 70.3° / 55.8°)。垂直视场决定车前多近的地面
拍不到:相机高 camera_z、下俯 camera_pitch(未实测时按 0)-> 相机前 ~0.45 m 内看不到桶底。

能检出的类别(common.bev.CellType):
  CONE  锥桶:road.cones 里在视场内、有效距离内的
  TAPE  胶带:边界材质是 NONE(没桶、只铺胶带)的那几段,沿线每 0.1 m 一个点
  ZONE / TRAFFIC_LIGHT:FastSim 的路里还没有这两类物体,接口已留(update_camera 认这两类)

遮挡:从相机看,每个锥桶占一段方位角(半宽 asin(r/d),r 取桶底半径);离相机更近的桶把这段
角度盖住的比例超过 1 - min_visible,就当被挡住、YOLO 认不出来。胶带贴地,不算遮挡。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from common.bev import CellType
from common.params import CarParams, TrackParams
from simulation.fastsim.road import EDGE, NONE, WALL

TAPE_STEP = 0.10                # 胶带沿线取点间距 m


@dataclass(frozen=True)
class YoloSimConfig:
    miss: float = 0.10          # [假设] 漏检概率(每个目标每帧)
    sigma0: float = 0.02        # [假设] 位置误差 m:sigma0 + sigma_per_m × 距离
    sigma_per_m: float = 0.02   # [假设]
    false_rate: float = 0.02    # [假设] 每帧冒出一个假锥桶的概率
    min_visible: float = 0.5    # [假设] 锥桶露出这么多(按方位角宽度算)YOLO 才认得出


class FakeYolo:
    def __init__(self, road, params: CarParams = CarParams(), cfg: YoloSimConfig = YoloSimConfig(),
                 seed: int = 0):
        self.road, self.p, self.cfg = road, params, cfg
        self.rng = np.random.default_rng(seed)
        tape = []
        for side, mat in ((road.left, road.mat_left), (road.right, road.mat_right)):
            pts = side[np.asarray(mat) == NONE]
            if len(pts):
                step = max(1, int(round(TAPE_STEP / max(float(np.median(np.diff(road.s))), 1e-6))))
                tape.append(pts[::step])
        self.tape = np.vstack(tape) if tape else np.zeros((0, 2))

    def _in_view(self, pts, pose):
        """odom 点 -> (视场内的掩码, 车体系坐标, 到相机的距离)。"""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        bx, by = c * dx + s * dy, -s * dx + c * dy
        cx = bx - self.p.camera_x
        d = np.hypot(cx, by)
        # 垂直视场:光轴下俯 pitch,画面下沿再往下 vfov/2;地面点比 near 还近就出了画面下沿,
        # 桶底(落地投影要用它)拍不到。pitch 未实测时按 0。
        down = self.p.camera_pitch + self.p.camera_vfov / 2
        near = self.p.camera_z / math.tan(down) if down < math.pi / 2 else 0.0
        ok = ((d <= self.p.yolo_range) & (cx >= near) & (cx > 0)
              & (np.abs(np.arctan2(by, cx)) <= self.p.camera_hfov / 2))
        return ok, np.stack([bx, by], axis=1), d

    def visible(self, pose, cones: np.ndarray, radius: float) -> np.ndarray:
        """每个锥桶从相机看露出的比例(0~1):被更近的桶挡住的方位角宽度扣掉。"""
        x, y, yaw = pose
        cx, cy = x + self.p.camera_x * math.cos(yaw), y + self.p.camera_x * math.sin(yaw)
        d = np.hypot(cones[:, 0] - cx, cones[:, 1] - cy)
        th = np.arctan2(cones[:, 1] - cy, cones[:, 0] - cx)
        half = np.arcsin(np.clip(radius / np.maximum(d, radius), 0, 1))
        out = np.ones(len(cones))
        order = np.argsort(d)
        for n, i in enumerate(order):
            lo, hi = th[i] - half[i], th[i] + half[i]
            ivs = []
            for j in order[:n]:                                     # 只有更近的桶能挡
                dj = (th[j] - th[i] + math.pi) % (2 * math.pi) - math.pi
                a, b = max(lo, th[i] + dj - half[j]), min(hi, th[i] + dj + half[j])
                if b > a:
                    ivs.append((a, b))
            if ivs:
                ivs.sort()
                cov, cur_a, cur_b = 0.0, *ivs[0]
                for a, b in ivs[1:]:
                    if a > cur_b:
                        cov += cur_b - cur_a
                        cur_a, cur_b = a, b
                    else:
                        cur_b = max(cur_b, b)
                cov += cur_b - cur_a
                out[i] = 1.0 - cov / (hi - lo)
        return out

    def detect(self, pose) -> tuple[np.ndarray, np.ndarray]:
        """返回 (kinds, xy_body):kinds 是 CellType,xy_body 是车体系(后轴原点)落地点。"""
        cfg, rng = self.cfg, self.rng
        kinds, xy = [], []
        for objs, kind, can_miss in ((self.road.cones, CellType.CONE, True),
                                     (self.tape, CellType.TAPE, False)):
            if not len(objs):
                continue
            objs = np.asarray(objs, float)
            ok, b, d = self._in_view(objs, pose)
            if can_miss:
                if ok.any():                    # 遮挡按视场内所有桶算(漏检的桶照样挡视线)
                    vis = np.zeros(len(ok))
                    vis[ok] = self.visible(pose, objs[ok], self.road.cone_radius)
                    ok &= vis >= cfg.min_visible
                ok &= rng.random(len(ok)) >= cfg.miss
            b, d = b[ok], d[ok]
            b = b + rng.normal(0, 1, b.shape) * (cfg.sigma0 + cfg.sigma_per_m * d)[:, None]
            kinds += [int(kind)] * len(b)
            xy.append(b)
        if rng.random() < cfg.false_rate:
            r = rng.uniform(0.5, self.p.yolo_range)
            a = rng.uniform(-self.p.camera_hfov / 2, self.p.camera_hfov / 2)
            kinds.append(int(CellType.CONE))
            xy.append(np.array([[self.p.camera_x + r * math.cos(a), r * math.sin(a)]]))
        if not kinds:
            return np.zeros(0, int), np.zeros((0, 2))
        return np.asarray(kinds, int), np.vstack(xy)


@dataclass(frozen=True)
class FloorSegConfig:
    """假地面分割的误差。全是 [假设],等实车 SegFormer 标定、录像后改。"""
    range: float = 3.0          # 有效距离 m:和车上 ground_seg_node --max-range 一致(俯仰 ±1° 在 3 m 外已是 ±12 cm 以上)
    step: float = 0.05          # 采样间距 m(= BEV 栅格)
    edge_sigma: float = 0.05    # 每帧边界整体偏移(外扩为正)m
    jitter: float = 0.03        # 逐点边界抖动 m
    miss: float = 0.05          # 整帧失败(什么都不给)的概率


class FakeFloorSeg:
    """FastSim 里的"摄像头 + 地面分割":给出相机视场内被识别为可行驶地面的点(车体系),
    喂给 Bev.update_camera(kinds=FREE)。实车上对应 SegFormer 的地面类别经标定投影到地面。

    语义 = 实车的通用地面分割(ADE20K SegFormer):锥桶/胶带边界内外是同一块地板,一样算可行驶;
    只有墙外和 EDGE(路面换材质,如地板/草地)外不算,并扣掉锥桶底座。锥桶赛道的路沿只能靠锥桶层和雷达认。
    (曾经一半的局"只认路内",等于给了模型实车永远拿不到的路沿信号,已删,2026-10-10。)
    """

    def __init__(self, road, params: CarParams = CarParams(), cfg: FloorSegConfig = FloorSegConfig(),
                 seed: int = 0):
        from scipy.spatial import cKDTree
        self.road, self.p, self.cfg = road, params, cfg
        self.rng = np.random.default_rng(seed)
        self.centre_tree = cKDTree(road.centre)
        self.cone_tree = cKDTree(road.cones) if len(road.cones) else None
        # 相机视场内的地面采样点(车体系,后轴原点),一次算好
        down = params.camera_pitch + params.camera_vfov / 2
        near = params.camera_z / math.tan(down) if down < math.pi / 2 else 0.0
        self.near = near
        xs = np.arange(near, cfg.range + 1e-9, cfg.step)
        ys = np.arange(-cfg.range, cfg.range + 1e-9, cfg.step)
        gx, gy = np.meshgrid(xs, ys, indexing='ij')
        g = np.stack([gx.ravel(), gy.ravel()], axis=1)
        keep = (np.hypot(g[:, 0], g[:, 1]) <= cfg.range) & (np.abs(np.arctan2(g[:, 1], g[:, 0])) <= params.camera_hfov / 2)
        self.grid_cam = g[keep]
        self.grid_body = self.grid_cam + (params.camera_x, 0.0)
        self.grid_r = np.hypot(self.grid_cam[:, 0], self.grid_cam[:, 1])
        self.grid_a = np.arctan2(self.grid_cam[:, 1], self.grid_cam[:, 0])

    def in_view(self, world, pose) -> np.ndarray:
        """世界系点 (n, 2) 是否落在地面分割的视场里(和 detect 的采样范围一致)。"""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = world[:, 0] - x, world[:, 1] - y
        bx, by = c * dx + s * dy - self.p.camera_x, -s * dx + c * dy
        return ((bx >= self.near) & (np.hypot(bx, by) <= self.cfg.range)
                & (np.abs(np.arctan2(by, bx)) <= self.p.camera_hfov / 2))

    def detect(self, pose) -> np.ndarray:
        """返回可行驶地面点 (n, 2),车体系(后轴原点,x 前 y 左)。"""
        cfg, rng, road = self.cfg, self.rng, self.road
        if rng.random() < cfg.miss:
            return np.zeros((0, 2))
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        b = self.grid_body
        w = np.stack([x + c * b[:, 0] - s * b[:, 1], y + s * b[:, 0] + c * b[:, 1]], axis=1)
        _, i = self.centre_tree.query(w)
        n = road.normal[i]
        d = np.einsum('ij,ij->i', w - road.centre[i], n)               # 左正
        m = rng.normal(0, cfg.edge_sigma) + rng.normal(0, cfg.jitter, len(d))
        hl, hr = road.half_l[i] + m, road.half_r[i] + m
        ml, mr = np.asarray(road.mat_left)[i], np.asarray(road.mat_right)[i]
        stop = (WALL, EDGE)                                            # 墙挡住地面;EDGE 外是别的材质
        free = ~(((d > hl) & np.isin(ml, stop)) | ((d < -hr) & np.isin(mr, stop)))
        if self.cone_tree is not None:
            dc, _ = self.cone_tree.query(w)
            free &= dc > road.cone_radius
            free &= ~self._shadow(x + c * self.p.camera_x, y + s * self.p.camera_x, yaw)
        if len(road.obstacles):
            free &= ~self._obstacle_shadow(x + c * self.p.camera_x, y + s * self.p.camera_x, yaw)
        return b[free]

    def _obstacle_shadow(self, cx, cy, yaw) -> np.ndarray:
        """路中障碍(箱子/板子)本身和它挡住的那一扇地面(grid_cam 上的布尔):按相机看到的四个角张成的角度范围,
        比最近的角远的都算看不到(当障碍比相机 24 cm 高)。"""
        out = np.zeros(len(self.grid_cam), bool)
        for box in self.road.obstacles:
            d = box - (cx, cy)
            dist = np.hypot(d[:, 0], d[:, 1])
            if dist.min() > self.cfg.range:
                continue
            ang = np.arctan2(d[:, 1], d[:, 0]) - yaw
            mid = math.atan2(d[:, 1].mean(), d[:, 0].mean()) - yaw
            rel = (ang - mid + np.pi) % (2 * np.pi) - np.pi              # 以箱子中心方向为零,免得跨 ±π
            da = (self.grid_a - mid + np.pi) % (2 * np.pi) - np.pi
            out |= (da >= rel.min()) & (da <= rel.max()) & (self.grid_r > dist.min())
        return out

    def _shadow(self, cx, cy, yaw) -> np.ndarray:
        """被锥桶挡住、相机看不到的地面格(grid_cam 上的布尔)。桶高 70 cm > 相机高 24 cm,桶后面那一扇
        地面一直挡到有效距离;扇形半角 = asin(底座半径 / 距离)(实车 SegFormer 把桶身分成非地面,投到地面
        就是这一扇空着)。"""
        idx = self.cone_tree.query_ball_point((cx, cy), self.cfg.range)
        if not idx:
            return np.zeros(len(self.grid_cam), bool)
        d = self.road.cones[idx] - (cx, cy)
        dist = np.hypot(d[:, 0], d[:, 1])
        ang = np.arctan2(d[:, 1], d[:, 0]) - yaw
        half = np.arcsin(np.minimum(1.0, self.road.cone_radius / np.maximum(dist, 1e-6)))
        h, ch = TrackParams().cone_height, self.p.camera_z
        end = np.full(len(dist), np.inf) if h >= ch else dist * ch / (ch - h)     # 矮桶:影子只到这么远
        da = np.abs((self.grid_a[None, :] - ang[:, None] + np.pi) % (2 * np.pi) - np.pi)
        r = self.grid_r[None, :]
        return ((da < half[:, None]) & (r > dist[:, None]) & (r < end[:, None])).any(axis=0)
