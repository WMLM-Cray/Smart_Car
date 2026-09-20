"""策略推理 —— 不依赖 ROS,也不依赖 torch,只要 numpy。

车上(OrangePi)装不装得上 torch 是未知数,而这个网络就是三层全连接 + Tanh,
用 numpy 写前向只要六行,不值得为它在车上拖一套深度学习框架。
权重先在开发机上用 work/export_policy.py 从 .pt 导成 .npz。

ROS 那一层在 teleop/policy_node.py:这里只负责"传感器数值 -> (车速, 转角)",
好处是能在开发机上直接拿仿真数据验证,不用上车,见 fastsim/tests/test_policy.py。
"""
from __future__ import annotations

import numpy as np

from .obs import build_obs, make_sector_edges, sector_index, sectorize


class Policy:
    def __init__(self, npz_path, obs_range=10.0, budget=120.0, max_steer=0.55):
        z = np.load(npz_path)
        self.w = [z[f'w{i}'] for i in range(3)]
        self.b = [z[f'b{i}'] for i in range(3)]
        self.obs_dim, self.frames = int(z['obs_dim']), int(z['frames'])
        self.v_cmd_fwd, self.v_cmd_rev = float(z['v_cmd_fwd']), float(z['v_cmd_rev'])
        self.edges = make_sector_edges(int(z['front_sectors']))
        self.n_sectors = len(self.edges) - 1
        self.obs_range, self.budget, self.max_steer = obs_range, budget, max_steer
        self._angles_key = None
        self.reset()

    def reset(self):
        self.buf = np.zeros(self.obs_dim * self.frames, np.float32)
        self.last_action = np.zeros(2, np.float32)

    def _index(self, angles):
        """射线->扇区的映射只在雷达角度变了才重算 —— 每帧重算 1667 个点纯属浪费。

        先折回 [-π, π):扇区边界是按这个区间定的,雷达要是按 0..2π 发角度,
        不折的话后半圈会全挤进最后一个扇区,而且不会报错,只会安静地开错。
        """
        angles = (np.asarray(angles, float) + np.pi) % (2 * np.pi) - np.pi
        key = (len(angles), float(angles[0]), float(angles[-1]))
        if key != self._angles_key:
            self._idx, self._angles_key = sector_index(angles, self.edges), key
        return self._idx

    def observe(self, ranges, angles, speed, yaw_rate, time_left) -> np.ndarray:
        """当前帧的观测(未叠帧)。角度是车体系,0=正前,左正,单位 rad。"""
        return build_obs(sectorize(ranges, self._index(angles), self.obs_range, self.n_sectors),
                         speed, yaw_rate, self.last_action, time_left, self.v_cmd_fwd)

    def forward(self, obs) -> np.ndarray:
        """叠帧后过网络,返回 clip 到 ±1 的动作,并记为 last_action。"""
        self.buf = np.roll(self.buf, -self.obs_dim)
        self.buf[-self.obs_dim:] = obs
        x = self.buf
        for i in range(2):
            x = np.tanh(x @ self.w[i].T + self.b[i])
        act = np.clip(x @ self.w[2].T + self.b[2], -1, 1).astype(np.float32)
        self.last_action = act
        return act

    def act(self, ranges, angles, speed, yaw_rate, time_left) -> tuple[float, float]:
        """一步控制,返回(车速 m/s, 前轮转角 rad,左正)。"""
        a0, a1 = self.forward(self.observe(ranges, angles, speed, yaw_rate, time_left))
        return float(a0) * (self.v_cmd_fwd if a0 >= 0 else self.v_cmd_rev), float(a1) * self.max_steer
