"""Versioned, numpy-only low-speed steering observation and inference.

观测每帧 52 维:48 扇区 + 编码器速度 + 速度上限 + 偏航角速度 + 上一步转向,叠 5 帧。
输出 1 维:转向 ±1。

## SPEED_SCALE 与 schema 自描述

`SPEED_SCALE` 是**训练时**观测里速度通道的归一化尺度,不是部署速度。同一个网络
在 `speed/1.5` 下训出来,拿到 `speed/3.0` 下用,输入分布就整个偏了 —— 实测干净率
从 0.9 掉到 0.0,是灾难级而不是"差一点"。所以这个值必须**跟着权重走**。

模型文件里的 `schema` 字段记录了它训练时的 scale:

    low_speed_steering_v1        -> 1.5   (历史值,实车在跑的 baseline 和 m4 都写着这个)
    low_speed_steering_v1_s2.4   -> 2.4
    low_speed_steering_v1_s3     -> 3.0

于是部署时不必再问环境变量 —— `SteeringPolicy` 自己从 schema 反解出 scale,
调用方把 `policy.scale` 传给 `observe()` 即可。旧模型(无后缀)反解为 1.5,
行为逐位不变。`schema_for()` / `scale_for_schema()` 是 schema 与 scale 的**唯一**
定义处,别再在别处拼字符串。
"""
import numpy as np

from common.obs import sector_index, sectorize

# schema 基名,以及"老" scale。scale 1.5 的 schema 就是基名本身 —— 这是历史,
# 不能改:所有 1.5 的产物(含实车在跑的 baseline 与 m4)都写着这个名字。
SCHEMA_PREFIX = 'low_speed_steering_v1'
DEFAULT_SPEED_SCALE = 1.5

SCHEMA = SCHEMA_PREFIX          # 兼容旧引用:默认 scale 对应的 schema
SPEED_SCALE = DEFAULT_SPEED_SCALE
DIM, FRAMES = 52, 5


def schema_for(scale):
    """SPEED_SCALE -> 它对应的 schema 字符串。"""
    scale = float(scale)
    return (SCHEMA_PREFIX if scale == DEFAULT_SPEED_SCALE
            else '%s_s%g' % (SCHEMA_PREFIX, scale))


def scale_for_schema(schema):
    """schema 字符串 -> 它蕴含的 SPEED_SCALE。不认识的 schema 抛 ValueError。

    模型是自描述的:它记着自己训练时的 scale,所以部署侧不用再设环境变量。
    """
    schema = str(schema)
    if schema == SCHEMA_PREFIX:
        return DEFAULT_SPEED_SCALE
    if not schema.startswith(SCHEMA_PREFIX + '_s'):
        raise ValueError('不认识的转向策略 schema: %r' % schema)
    try:
        return float(schema[len(SCHEMA_PREFIX) + 2:])
    except ValueError:
        raise ValueError('schema %r 的后缀不是数字' % schema) from None


def observe(ranges, angles, speed, target_speed, yaw_rate, last_steer, scale=None):
    """一帧观测。`scale` 不给(None)就用模块级 SPEED_SCALE —— 老调用方逐位不变。"""
    scale = SPEED_SCALE if scale is None else float(scale)
    angles = (np.asarray(angles) + np.pi) % (2 * np.pi) - np.pi
    sectors = sectorize(ranges, sector_index(angles), 10.)
    return np.clip(np.r_[sectors, speed / scale, target_speed / scale,
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
    """加载一个 .npz 转向策略。`scale` 从模型自带的 schema 反解,不必外部指定。"""

    def __init__(self, path):
        with np.load(path) as z:
            self.schema = str(z['schema'])
            self.scale = scale_for_schema(self.schema)   # 不认识的 schema 在此抛错
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
