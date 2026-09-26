"""Versioned, numpy-only low-speed steering observation and inference."""
import numpy as np

from common.obs import sector_index, sectorize

SCHEMA = 'low_speed_steering_v1'
DIM, FRAMES = 52, 5
SPEED_SCALE = 1.5


def observe(ranges, angles, speed, target_speed, yaw_rate, last_steer):
    angles = (np.asarray(angles) + np.pi) % (2 * np.pi) - np.pi
    sectors = sectorize(ranges, sector_index(angles), 10.)
    return np.clip(np.r_[sectors, speed / SPEED_SCALE, target_speed / SPEED_SCALE,
                         yaw_rate / 3., last_steer], -1, 1).astype(np.float32)


class History:
    def __init__(self):
        self.reset()

    def reset(self):
        self.buf = np.zeros(DIM * FRAMES, np.float32)

    def push(self, obs):
        self.buf = np.roll(self.buf, -DIM)
        self.buf[-DIM:] = obs
        return self.buf.copy()


class SteeringPolicy:
    def __init__(self, path):
        with np.load(path) as z:
            if str(z['schema']) != SCHEMA:
                raise ValueError('Wrong steering policy schema')
            self.w = [z[f'w{i}'].copy() for i in range(3)]
            self.b = [z[f'b{i}'].copy() for i in range(3)]
        self.history = History()

    def reset(self):
        self.history.reset()

    def __call__(self, obs):
        x = self.history.push(obs)
        for i in range(2):
            x = np.tanh(x @ self.w[i].T + self.b[i])
        return float(np.clip(x @ self.w[2].T + self.b[2], -1, 1).item())
