"""spica:在 v2 架构上继续训练,同时能把 v1 的模型无损灌进来。

v1(52 维)和 v2(53 维)是**同一套通道,v2 多一个**:

    v1: [48 扇区, speed/scale, target_speed/scale, yaw_rate/3, last_steer]
    v2: [48 扇区, speed/scale, cap/scale,          yaw_rate/3, last_action[0], last_action[1]]

前 52 维逐位相同,展平缓冲是帧主序,所以 260 -> 265 是**精确的列重排**:
每帧 52 列插到 53 列块的前 52 位,新增通道的权重置 0。纯排列,浮点上无损
(实测 300 组随机输入含 History 滚动,转向输出最大差 0.000e+00)。

速度行权重置 0 后网络速度输出恒为 0.0,`action_to_speed(0.0)` = 0.725 m/s ——
是动作区间 [0.45, 1.0] 的**中点**,不是下限。所以重排后的模型起步速度是 0.725。
"""
import numpy as np

from common import low_speed_policy as v1
from common import road_policy as v2

# 顶层 spica 模型的 schema。无后缀 = 和当前的 road_v2 逐位等价。
# `_sN` 与 v1 同义(训练时速度通道的归一化尺度,不是部署速度);
# `_vA-B` 是动作速度范围,决定 action_to_speed。目前只用到无后缀这一种。
SCHEMA = 'spica_v3'
SCHEMA_PREFIX = SCHEMA

V1_DIM = v1.DIM          # 52
V1_PREFIX = v1.SCHEMA_PREFIX


def is_v1(schema) -> bool:
    return str(schema).startswith(V1_PREFIX)


def remap_v1_to_spica(path):
    """v1 模型(npz 路径) -> spica 权重 dict。转向通道逐位不变,速度通道置 0。"""
    with np.load(path) as z:
        schema = str(z['schema'])
        if not is_v1(schema):
            raise ValueError(f'{path} 不是 v1 模型(schema={schema})')
        w0 = np.zeros((256, v2.DIM * v2.FRAMES), np.float32)        # 53*5 = 265
        for j in range(v2.FRAMES):
            w0[:, j * v2.DIM: j * v2.DIM + V1_DIM] = z['w0'][:, j * V1_DIM: j * V1_DIM + V1_DIM]
        w2 = np.zeros((2, 256), np.float32)
        w2[0] = z['w2'][0]                                          # 转向行原样搬
        b2 = np.array([z['b2'][0], 0.0], np.float32)                # 速度行置 0
        return dict(schema=np.array(SCHEMA), w0=w0, w1=z['w1'].copy(), w2=w2,
                    b0=z['b0'].copy(), b1=z['b1'].copy(), b2=b2)


def load_init(path):
    """热启动权重。v1 或 road_v2/spica 都接受,返回 dict,键和 npz 一致。"""
    with np.load(path) as z:
        schema = str(z['schema'])
        if is_v1(schema):
            return remap_v1_to_spica(path)
        if schema.startswith(v2.SCHEMA) or schema.startswith(SCHEMA):
            return {k: z[k].copy() for k in ('w0', 'w1', 'w2', 'b0', 'b1', 'b2')}
        raise ValueError(f'{path} 的 schema {schema} 不认识')
