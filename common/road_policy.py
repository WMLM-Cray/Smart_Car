"""路段策略 v2 的观测与推理,只依赖 numpy(车上要跑)。

每帧 53 维:48 扇区 + 编码器速度 + 速度上限 + 偏航角速度 + 上一步转向 + 上一步速度,叠 5 帧。
输出 2 维:[转向, 速度],都是 ±1。
  转向 a -> 舵机 turn_mid + a·turn_span(和 v1 一样)
  速度 a -> V_LO + (a+1)/2·(V_HI-V_LO) m/s,交给车上的 PID 闭环;实际再和速度上限取小
"""
import numpy as np

from common.obs import sector_index, sectorize

SCHEMA = 'road_v2'
DIM, FRAMES = 53, 5
SPEED_SCALE = 1.5
OBS_RANGE = 10.0
V_LO, V_HI = 0.45, 1.0      # 速度输出范围 m/s:下限是亏电时的平台速度,不输出 0(停车归防撞和人)


def action_to_speed(a: float) -> float:
    a = min(1.0, max(-1.0, a))
    return V_LO + (a + 1.0) / 2.0 * (V_HI - V_LO)


def speed_to_action(v: float) -> float:
    return min(1.0, max(-1.0, 2.0 * (v - V_LO) / (V_HI - V_LO) - 1.0))


def observe(ranges, angles, speed, cap, yaw_rate, last_action):
    angles = (np.asarray(angles) + np.pi) % (2 * np.pi) - np.pi
    sectors = sectorize(ranges, sector_index(angles), OBS_RANGE)
    return np.clip(np.r_[sectors, speed / SPEED_SCALE, cap / SPEED_SCALE, yaw_rate / 3.,
                         np.asarray(last_action, np.float32)], -1, 1).astype(np.float32)


class History:
    def __init__(self):
        self.reset()

    def reset(self):
        self.buf = np.zeros(DIM * FRAMES, np.float32)

    def push(self, obs):
        self.buf = np.roll(self.buf, -DIM)
        self.buf[-DIM:] = obs
        return self.buf.copy()


def forward(w, b, x):
    """两层 tanh + 线性输出,clip 到 ±1。w/b 是三层的权重列表。"""
    for i in range(2):
        x = np.tanh(x @ w[i].T + b[i])
    return np.clip(x @ w[2].T + b[2], -1, 1)


class RoadPolicy:
    def __init__(self, path):
        with np.load(path) as z:
            if str(z['schema']) != SCHEMA:
                raise ValueError(f'模型版本 {z["schema"]} 不是 {SCHEMA}')
            self.w = [z[f'w{i}'].copy() for i in range(3)]
            self.b = [z[f'b{i}'].copy() for i in range(3)]
        self.history = History()

    def reset(self):
        self.history.reset()

    def __call__(self, obs) -> np.ndarray:
        return forward(self.w, self.b, self.history.push(obs)).astype(np.float32)
