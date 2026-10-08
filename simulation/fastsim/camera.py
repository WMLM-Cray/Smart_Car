"""FastSim 里的"摄像头 + YOLO":按真值给出检测结果,喂给 common/bev.py 的 update_camera()。

不渲染图像。实车上摄像头节点做的是"检测框 -> 落地点 -> 车体系坐标",这里直接从真值生成
同样格式的结果,再加上实车会有的毛病:视场与有效距离、漏检、位置误差、偶尔的误检。
误差参数都是 [假设],等实车标定后改 YoloSimConfig。

能检出的类别(common.bev.CellType):
  CONE  锥桶:road.cones 里在视场内、有效距离内的
  TAPE  胶带:边界材质是 NONE(没桶、只铺胶带)的那几段,沿线每 0.1 m 一个点
  ZONE / TRAFFIC_LIGHT:FastSim 的路里还没有这两类物体,接口已留(update_camera 认这两类)

第一版不算遮挡(前面的桶挡住后面的桶),所以检出偏乐观。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from common.bev import CellType
from common.params import CarParams
from simulation.fastsim.road import NONE

TAPE_STEP = 0.10                # 胶带沿线取点间距 m


@dataclass(frozen=True)
class YoloSimConfig:
    miss: float = 0.10          # [假设] 漏检概率(每个目标每帧)
    sigma0: float = 0.02        # [假设] 位置误差 m:sigma0 + sigma_per_m × 距离
    sigma_per_m: float = 0.02   # [假设]
    false_rate: float = 0.02    # [假设] 每帧冒出一个假锥桶的概率


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
        ok = (d <= self.p.yolo_range) & (np.abs(np.arctan2(by, cx)) <= self.p.camera_hfov / 2) & (cx > 0)
        return ok, np.stack([bx, by], axis=1), d

    def detect(self, pose) -> tuple[np.ndarray, np.ndarray]:
        """返回 (kinds, xy_body):kinds 是 CellType,xy_body 是车体系(后轴原点)落地点。"""
        cfg, rng = self.cfg, self.rng
        kinds, xy = [], []
        for objs, kind, can_miss in ((self.road.cones, CellType.CONE, True),
                                     (self.tape, CellType.TAPE, False)):
            if not len(objs):
                continue
            ok, b, d = self._in_view(np.asarray(objs, float), pose)
            if can_miss:
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
