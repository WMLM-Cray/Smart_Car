"""强化学习环境适配层:把 RoadEnv 包装成标准的 reset/step 语义。

    reset()             -> 初始观测(已叠好 5 帧历史,维度 DIM*FRAMES)
    step(action)        -> (obs, reward, terminated, truncated, info)

不改动 RoadEnv 本身 —— DAgger(road_train)、评测(road_eval / road_track)继续走老接口;
RL 训练(road_rl)和后续新实验从这层进。观测/动作解码复用 common.road_policy,
车辆/赛道参数复用 common.params,这里不重复任何维度或参数。

奖励(从 road_rl.rollout 原来的内联计算原样搬来,逐位等价):
    r = 沿路前进距离 Δs - time_cost - slip × 打滑程度;撞/出界再扣 crash
    权重由调用方注入,来源是实验配置 training/config/rl_ppo.yaml,不在代码里写死。

终止语义(给 GAE 用,见 road_rl.gae 的 terminal 参数):
    terminated = goal / collision / off_track   局真结束了,末态价值取 0
    truncated  = timeout(路预算超时或步数上限)  截断局要对末态 bootstrap
"""
from __future__ import annotations

import numpy as np

from common.road_policy import DIM, FRAMES, History
from simulation.fastsim.road_env import RoadEnv

DONE_REASONS = ('goal', 'collision', 'off_track')   # terminated
TRUNC_REASONS = ('timeout',)                        # truncated


class RlEnv:
    """一局可反复 reset 的 RL 环境。road/pl 是同一局的路与车辆参数,reset 只换噪声种子。"""

    def __init__(self, road, pl, seed, start=(0, 0.0, 0.0), noise=True, robust=False,
                 crash=50.0, time_cost=0.02, slip=0.5, max_steps=3000):
        self._road, self._pl, self._start = road, pl, start
        self._seed = int(seed)
        self._noise, self._robust = noise, robust
        self.crash, self.time_cost, self.slip = float(crash), float(time_cost), float(slip)
        self.max_steps = int(max_steps)
        self.env: RoadEnv | None = None
        self.hist = History()
        # 维度从 common 读取,不在这里再写一份
        self.obs_dim = DIM * FRAMES
        self.action_dim = 2
        self._s_prev, self._n = 0.0, 0

    # ---- 回合 ----
    def reset(self, seed=None) -> np.ndarray:
        """新开一局,返回叠好历史的初始观测。seed=None 时用构造时的种子。"""
        self.env = RoadEnv(self._road, self._pl, self._seed if seed is None else int(seed),
                           self._start, noise=self._noise, robust=self._robust)
        self.hist.reset()
        self._s_prev = self.env.road.s[self.env.idx]
        self._n = 0
        return self.hist.push(self.env.observation())

    def step(self, action):
        """执行一拍 50ms。返回 (obs, reward, terminated, truncated, info)。"""
        if self.env is None:
            raise RuntimeError('先 reset() 再 step()')
        why = self.env.step(np.clip(np.asarray(action, np.float32), -1, 1))
        self._n += 1
        s_now = self.env.road.s[self.env.idx]
        slip = float(getattr(self.env.car, 'slip', 0.0))
        r = (s_now - self._s_prev) - self.time_cost - self.slip * slip
        if why in ('collision', 'off_track'):
            r -= self.crash
        self._s_prev = s_now
        terminated = why in DONE_REASONS
        truncated = why in TRUNC_REASONS or (why is None and self._n >= self.max_steps)
        reason = why or ('timeout' if truncated else None)
        info = dict(reason=reason,           # None=继续 / goal / collision / off_track / timeout
                    progress=float(s_now),   # 沿路里程 m(评测用,不进观测)
                    min_clear=float(self.env.min_clear),   # 全程最小边界余量 m
                    time=float(self.env.t),  # 仿真耗时 s
                    cap=float(self.env.pl.cap), slip=slip)
        obs = self.hist.push(self.env.observation())
        return obs, float(r), terminated, truncated, info
