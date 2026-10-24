"""bev_v1/v2 与 bev_v2.5 的 BEV 输入、网络规格和纯 NumPy 推理(车上 CPU 参照/备用)。

bev_v1/v2 主图: 80×80, 10 cm/格, 车前 6 m、车后 2 m、左右各 4 m。
通道 0 雷达记忆,1 锥桶,2 胶带,3 区域,4 本帧雷达;bev_v2 另有通道 5 FREE。
bev_v2.5 沿用完整 bev_v2 主图,并从同一张 5 cm BEV 直接裁 6×64×64 近场图
(车前 2.6 m、车后 0.6 m、左右各 1.6 m),不做 2×2 合并;两图共享 3 个标量。

旧网络(五通道/六通道主图):conv 5/6→16(5×5 s2)→32→64→64,展平特征接标量,FC 256→256→2。
bev_v2.5:主图分支不变;近场分支 conv 6→16(3×3 s2)→32→32→32,展平 512 维;
主图特征、近场特征和标量拼接后 FC 256→256→2,共 694,498 个参数。
所有 schema 通过文件内前缀分派;bev_v2 权重键名及前向路径保持不变。
"""
from __future__ import annotations

import math

import numpy as np

from common.bev import Bev, BevConfig, CAMERA_LAYER, CellType, lidar_points
from common.road_policy import Encoding, spec_of

SCHEMA = 'bev_v1'
SCHEMA_V2 = 'bev_v2'
SCHEMA_V25 = 'bev_v2.5'
GRID = BevConfig()                     # 5 cm 栅格:车前 6 车后 2 左右各 4 -> 160×160
POOL = 2                               # 合并成 10 cm
H = W = 80
CHANNELS = ('lidar', 'cone', 'tape', 'zone', 'lidar_now')
C = len(CHANNELS)
CHANNELS_V2 = CHANNELS + ('free',)
C2 = len(CHANNELS_V2)
NEAR_AHEAD, NEAR_BEHIND, NEAR_SIDE = 2.6, 0.6, 1.6
NEAR_SIZE = 64
NEAR_ROW0 = int(round((GRID.ahead - NEAR_AHEAD) / GRID.res))
NEAR_COL0 = int(round((GRID.side - NEAR_SIDE) / GRID.res))
NEAR_ROW1, NEAR_COL1 = NEAR_ROW0 + NEAR_SIZE, NEAR_COL0 + NEAR_SIZE
N_SCALAR = 3
CONVS = ((5, 16, 5), (16, 32, 3), (32, 64, 3), (64, 64, 3))     # bev_v1/v2 主分支
NEAR_CONVS = ((6, 16, 3), (16, 32, 3), (32, 32, 3), (32, 32, 3))  # bev_v2.5 近场分支
HIDDEN = 256

assert GRID.shape == (H * POOL, W * POOL)
assert (NEAR_AHEAD + NEAR_BEHIND) / GRID.res == 2 * NEAR_SIDE / GRID.res == NEAR_SIZE
assert NEAR_ROW1 <= H * POOL and NEAR_COL1 <= W * POOL


def schema_prefix(schema: str) -> str:
    """先匹配最长前缀,不把 bev_v2.5 当成 bev_v2 的速度后缀。"""
    for prefix in (SCHEMA_V25, SCHEMA_V2, SCHEMA):
        if schema == prefix or schema.startswith(prefix + '_'):
            return prefix
    raise ValueError(f'模型版本 {schema} 不是 {SCHEMA} / {SCHEMA_V2} / {SCHEMA_V25}')


def channels_of(schema: str) -> int:
    return C if schema_prefix(schema) == SCHEMA else C2


def _pool(img: np.ndarray) -> np.ndarray:
    """(..., 160, 160) -> (..., 80, 80),2×2 取最大。"""
    s = img.shape
    return img.reshape(s[:-2] + (H, POOL, W, POOL)).max(axis=(-3, -1))


def _raw_image(bev: Bev, pose, t: float, ranges_now=None, angles=None, scan_pose=None,
               free: bool = False) -> np.ndarray:
    """未合并的 5 cm 图;主图和近场使用相同的证据、裁图位姿和扫描位姿。"""
    layers = bev.crop_layers(pose, t)
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
    return out


def bev_image(bev: Bev, pose, t: float, ranges_now=None, angles=None, scan_pose=None,
              free: bool = False) -> np.ndarray:
    """bev_v1/v2 主图,沿用原来的 2×2 max 合并及扫描时刻位姿投影。"""
    return _pool(_raw_image(bev, pose, t, ranges_now, angles, scan_pose, free))


def bev_images(bev: Bev, pose, t: float, ranges_now=None, angles=None, scan_pose=None):
    """bev_v2.5 的 (主图, 近场图);一次裁原始 BEV,近场不做池化。

    近场构造集中在这里,将来添加 B_alpha' 的低分辨率归因对照时不必改动网络。
    """
    raw = _raw_image(bev, pose, t, ranges_now, angles, scan_pose, free=True)
    near = raw[:, NEAR_ROW0:NEAR_ROW1, NEAR_COL0:NEAR_COL1].copy()
    return _pool(raw), near


def bev_near_image(bev: Bev, pose, t: float, ranges_now=None, angles=None, scan_pose=None) -> np.ndarray:
    """仅取 6×64×64 近场图;闭环推理用 bev_images 避免重复裁图。"""
    raw = _raw_image(bev, pose, t, ranges_now, angles, scan_pose, free=True)
    return raw[:, NEAR_ROW0:NEAR_ROW1, NEAR_COL0:NEAR_COL1].copy()


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


def forward_v25(params: dict, img: np.ndarray, sc: np.ndarray, near: np.ndarray) -> np.ndarray:
    """新 schema 的双分支路径,旧 forward 的算子和运算顺序不变。"""
    if img.shape != (C2, H, W) or near.shape != (C2, NEAR_SIZE, NEAR_SIZE):
        raise ValueError('bev_v2.5 需要 (6,80,80) 主图和 (6,64,64) 近场图')
    x, nx = img.astype(np.float32), near.astype(np.float32)
    for k in range(len(CONVS)):
        x = np.maximum(_conv_s2(x, params[f'c{k}w'], params[f'c{k}b']), 0.0)
    for k in range(len(NEAR_CONVS)):
        nx = np.maximum(_conv_s2(nx, params[f'n{k}w'], params[f'n{k}b']), 0.0)
    h = np.concatenate([x.reshape(-1), nx.reshape(-1), sc.astype(np.float32)])
    for k in range(2):
        h = np.tanh(params[f'f{k}w'] @ h + params[f'f{k}b'])
    return np.clip(params['f2w'] @ h + params['f2b'], -1, 1).astype(np.float32)


class BevPolicy:
    """车上/评估用:读 npz,纯 numpy 前向。"""

    def __init__(self, path):
        with np.load(path) as z:
            self.schema = str(z['schema'])
            self.prefix = schema_prefix(self.schema)
            self.channels = channels_of(self.schema)
            self.free = self.prefix in (SCHEMA_V2, SCHEMA_V25)
            self.near = self.prefix == SCHEMA_V25
            suffix = self.schema[len(self.prefix):]
            self.enc = spec_of('spica_v3' + suffix)
            self.params = {k: z[k].astype(np.float32) for k in z.files if k != 'schema'}
            if self.near != ('n0w' in self.params):
                raise ValueError('模型 schema 与近场权重不匹配')

    def reset(self):
        """没有帧历史(记忆在 BEV 里),给 RoadRuntime 的接口留着。"""

    def __call__(self, img, sc, near=None):
        """旧 schema 接收 (主图, 标量);v2.5 追加第三项 5 cm 近场图。"""
        if self.near != (near is not None):
            raise ValueError(f'{self.prefix} 的输入与近场分支不匹配')
        if self.near:
            return forward_v25(self.params, img, sc, near)
        return forward(self.params, img, sc)


def n_params() -> int:
    n = sum(co * ci * k * k + co for ci, co, k in CONVS)
    flat = CONVS[-1][1] * (H // 16) * (W // 16) + N_SCALAR
    return n + flat * HIDDEN + HIDDEN + HIDDEN * HIDDEN + HIDDEN + HIDDEN * 2 + 2


def n_params_v25() -> int:
    main = sum(co * ci * k * k + co for ci, co, k in ((C2,) + CONVS[0][1:],) + CONVS[1:])
    near = sum(co * ci * k * k + co for ci, co, k in NEAR_CONVS)
    flat = CONVS[-1][1] * (H // 16) * (W // 16) + NEAR_CONVS[-1][1] * (NEAR_SIZE // 16) ** 2 + N_SCALAR
    return main + near + flat * HIDDEN + HIDDEN + HIDDEN * HIDDEN + HIDDEN + HIDDEN * 2 + 2
