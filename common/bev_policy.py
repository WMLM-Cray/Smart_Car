"""bev_v1:吃 BEV 的路段策略 —— 输入规格、网络结构、纯 numpy 推理(车上 CPU 参照/备用)。

## 输入(改这里就是换模型输入,训出来的权重和它绑定)

图  5 × 80 × 80,float32,值 0/1。车头朝上、以后轴为原点:车前 6 m、车后 2 m、左右各 4 m,
    每格 0.10 m(由 common/bev.py 的 5 cm 栅格 2×2 取最大合并)。
    通道  0 雷达占用(累积记忆)
          1 锥桶地标(摄像头,合并后画成实心圆)
          2 胶带(摄像头)
          3 区域标记(摄像头;起点框、检测区)
          4 本帧雷达(只有当前这一帧的回波,和记忆区分"现在看到的")
    红绿灯不进图(是事件不是几何),以后作为单独输入。
    bev_v2 = bev_v1 再加第 5 通道"可行驶路面"(摄像头地面分割登记的 FREE 层),6 × 80 × 80;
    网络只有第一层卷积的输入通道数不同(6→16)。schema 前缀决定是哪一版。
标量 3 个:车速/scale、速度上限/scale、偏航角速度/3,裁到 ±1(scale 来自速度契约,和 road_v2.5 同一套)。
    不放上一步动作(PR #18:这个通道会让策略"抄上一步"、闭环自激)。

## 输出
[转向, 目标速度],±1,解码同 common/road_policy(Encoding / spec_of)。

## 网络(全是标准算子、输入尺寸固定,便于 ONNX -> 昇腾 NPU)
    conv 5→16  5×5 s2 + ReLU   80 → 40
    conv 16→32 3×3 s2 + ReLU   40 → 20
    conv 32→64 3×3 s2 + ReLU   20 → 10
    conv 64→64 3×3 s2 + ReLU   10 → 5
    展平 1600 + 标量 3 -> 全连接 256 tanh -> 256 tanh -> 2,输出裁到 ±1
    约 54 万参数。
"""
from __future__ import annotations

import math

import numpy as np

from common.bev import Bev, BevConfig, CAMERA_LAYER, CellType, lidar_points
from common.road_policy import Encoding, spec_of

SCHEMA = 'bev_v1'
GRID = BevConfig()                     # 5 cm 栅格:车前 6 车后 2 左右各 4 -> 160×160
POOL = 2                               # 合并成 10 cm
H = W = 80
CHANNELS = ('lidar', 'cone', 'tape', 'zone', 'lidar_now')
C = len(CHANNELS)
SCHEMA_V2 = 'bev_v2'
CHANNELS_V2 = CHANNELS + ('free',)
C2 = len(CHANNELS_V2)


def channels_of(schema: str) -> int:
    """schema 前缀 -> 输入通道数。"""
    if schema.startswith(SCHEMA_V2):
        return C2
    if schema.startswith(SCHEMA):
        return C
    raise ValueError(f'模型版本 {schema} 不是 {SCHEMA} / {SCHEMA_V2}')
N_SCALAR = 3
CONVS = ((5, 16, 5), (16, 32, 3), (32, 64, 3), (64, 64, 3))     # (输入, 输出, 核);步长都是 2
HIDDEN = 256

assert GRID.shape == (H * POOL, W * POOL)


def _pool(img: np.ndarray) -> np.ndarray:
    """(..., 160, 160) -> (..., 80, 80),2×2 取最大。"""
    s = img.shape
    return img.reshape(s[:-2] + (H, POOL, W, POOL)).max(axis=(-3, -1))


def bev_image(bev: Bev, pose, t: float, ranges_now=None, angles=None, scan_pose=None,
              free: bool = False) -> np.ndarray:
    """当前位姿下的网络输入图 (5, 80, 80) float32。ranges_now 是最近一帧雷达(没有就留空),
    scan_pose 是**扫描那一刻**的位姿(雷达比控制慢,两次扫描之间车在动;不给就当作当前位姿)。
    回波按 scan_pose 落到 odom、再按当前 pose 裁图,和记忆层(也按扫描位姿登记)对齐。
    free=True 出 bev_v2 的 6 通道(多一层可行驶路面)。"""
    layers = bev.crop_layers(pose, t)                       # (5, 160, 160):雷达/锥桶/胶带/区域/红绿灯
    out = np.zeros((C2 if free else C, H * POOL, W * POOL), np.float32)
    out[0] = layers[0]
    out[1] = layers[CAMERA_LAYER[CellType.CONE]]
    out[2] = layers[CAMERA_LAYER[CellType.TAPE]]
    out[3] = layers[CAMERA_LAYER[CellType.ZONE]]
    if free:
        out[5] = layers[CAMERA_LAYER[CellType.FREE]]
    if ranges_now is not None:
        pts = lidar_points(ranges_now, angles, pose if scan_pose is None else scan_pose, bev.params,
                           bev.lidar_yaw, bev.cfg.max_range)
        if len(pts):
            row, col = bev._to_grid(pts, pose)
            ok = (row >= 0) & (row < H * POOL) & (col >= 0) & (col < W * POOL)
            out[4, row[ok], col[ok]] = 1.0
    return _pool(out)


def scalars(speed: float, cap: float, yaw_rate: float, enc: Encoding) -> np.ndarray:
    return np.clip(np.array([speed / enc.scale, cap / enc.scale, yaw_rate / 3.0], np.float32), -1, 1)


# ---------------- 纯 numpy 推理 ----------------

def _conv_s2(x: np.ndarray, w: np.ndarray, b: np.ndarray) -> np.ndarray:
    """(Cin, H, W) 卷积,步长 2,padding = k//2。im2col 实现,和 torch Conv2d(stride=2, padding=k//2) 逐元素一致。"""
    cin, h, wd = x.shape
    cout, _, k, _ = w.shape
    p = k // 2
    xp = np.pad(x, ((0, 0), (p, p), (p, p)))
    ho, wo = (h + 2 * p - k) // 2 + 1, (wd + 2 * p - k) // 2 + 1
    cols = np.empty((cin, k, k, ho, wo), x.dtype)
    for i in range(k):
        for j in range(k):
            cols[:, i, j] = xp[:, i:i + 2 * ho:2, j:j + 2 * wo:2]
    return (w.reshape(cout, -1) @ cols.reshape(-1, ho * wo)).reshape(cout, ho, wo) + b[:, None, None]


def forward(params: dict, img: np.ndarray, sc: np.ndarray) -> np.ndarray:
    x = img.astype(np.float32)
    for k in range(len(CONVS)):
        x = np.maximum(_conv_s2(x, params[f'c{k}w'], params[f'c{k}b']), 0.0)
    h = np.concatenate([x.reshape(-1), sc.astype(np.float32)])
    for k in range(2):
        h = np.tanh(params[f'f{k}w'] @ h + params[f'f{k}b'])
    return np.clip(params['f2w'] @ h + params['f2b'], -1, 1).astype(np.float32)


class BevPolicy:
    """车上/评估用:读 npz,纯 numpy 前向。"""

    def __init__(self, path):
        with np.load(path) as z:
            self.schema = str(z['schema'])
            self.channels = channels_of(self.schema)
            self.free = self.channels == C2
            self.enc = spec_of('spica_v3' + self.schema[len(SCHEMA):])     # 速度契约后缀和 spica 同一套(两版前缀等长)
            self.params = {k: z[k].astype(np.float32) for k in z.files if k != 'schema'}

    def __call__(self, img, sc):
        return forward(self.params, img, sc)


def n_params() -> int:
    n = sum(co * ci * k * k + co for ci, co, k in CONVS)
    flat = CONVS[-1][1] * (H // 16) * (W // 16) + N_SCALAR
    return n + flat * HIDDEN + HIDDEN + HIDDEN * HIDDEN + HIDDEN + HIDDEN * 2 + 2
