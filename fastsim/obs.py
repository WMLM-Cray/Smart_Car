"""观测构造 —— 唯一来源。仿真和实车都从这里取,不许各写一份。

单独成文件的原因:实车(OrangePi)上跑推理只需要 numpy,而 env.py 要 gymnasium。
把这几个函数留在 env.py 里,车上就得为了 48 维压缩装一整套训练依赖,或者照抄一遍 ——
照抄就是迟早会分叉,而观测一旦和训练时差半度,网络看到的是它没见过的输入,
车会"莫名其妙乱开",这种问题极难查。所以宁可多一个文件。
"""
from __future__ import annotations

import math

import numpy as np


def make_sector_edges(front: int = 36, rear_each: int = 6) -> np.ndarray:
    """扇区边界:后左 rear_each 段 | 前 front 段 | 后右 rear_each 段,默认 6|36|6 = 48。

    front 加大 = 车头方向角分辨率变细(默认每 5°)。改这个会改观测维度,
    策略权重不通用,只在做观测消融时动。
    """
    return np.concatenate([
        np.linspace(-math.pi, -math.pi / 2, rear_each + 1),
        np.linspace(-math.pi / 2, math.pi / 2, front + 1)[1:],
        np.linspace(math.pi / 2, math.pi, rear_each + 1)[1:],
    ])


SECTOR_EDGES = make_sector_edges()
SECTORS = len(SECTOR_EDGES) - 1        # 48


def sector_index(angles, edges=None) -> np.ndarray:
    """每条射线落在哪个扇区。角度是车体系,0=正前,左正。"""
    edges = SECTOR_EDGES if edges is None else edges
    return np.clip(np.searchsorted(edges, angles, side="right") - 1, 0, len(edges) - 2)


def sectorize(ranges, index, obs_range: float, n_sectors: int = SECTORS) -> np.ndarray:
    """一圈测距 -> n 个扇区的最近距离 / obs_range。

    对点数不敏感:实车与本仿真 1667 点、队友 Gazebo 2000 点,过这一层都是同样的 48 维,
    所以同一个策略两边都能用(index 由 sector_index(angles) 预先算好)。
    """
    r = np.minimum(np.nan_to_num(ranges, nan=obs_range, posinf=obs_range), obs_range)
    out = np.full(n_sectors, obs_range)
    np.minimum.at(out, index, r)
    return out / obs_range


def build_obs(sectors, speed: float, yaw_rate: float, last_action, time_left: float,
              v_cmd_fwd: float) -> np.ndarray:
    """扇区 + 编码器速度 + IMU 偏航角速度 + 上一步动作 + 剩余时间比例 -> 策略输入。

    last_action 是上一步 **clip 到 ±1 之后** 的动作(见 env.step);
    time_left 是 max(0, 1 - 已用时间/时长预算)。
    """
    obs = np.concatenate([sectors, [speed / v_cmd_fwd, yaw_rate / 3.0],
                          np.asarray(last_action, np.float32), [time_left]])
    return np.clip(obs, -1, 1).astype(np.float32)
