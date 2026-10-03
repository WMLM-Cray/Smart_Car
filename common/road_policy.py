"""路段策略 v2 的观测与推理,只依赖 numpy(车上要跑)。

每帧 53 维:48 扇区 + 编码器速度 + 速度上限 + 偏航角速度 + 上一步转向 + 上一步速度,叠 5 帧。
输出 2 维:[转向, 速度],都是 ±1。
  转向 a -> 舵机 turn_mid + a·turn_span(和 v1 一样)
  速度 a -> v_lo + (a+1)/2·(v_hi-v_lo) m/s,设计上交给实车 PID；当前无 road_v2 部署节点。仿真先与速度上限取小

速度契约自描述:schema 后缀决定 (scale, v_lo, v_hi),无后缀 = 下面三个默认值。
历史模型(road_v2 / spica_v3)不改名、不改码就能加载,逐位不变。
    road_v2 / spica_v3              scale 1.5, V ∈ [0.45, 1.0]
    spica_v3_s3_v0.45-3             scale 3.0, V ∈ [0.45, 3.0]
前缀之后**要么什么都不剩,要么是一个能认出的后缀** —— 其余一律报错,不回落。
`road_v2.5` 这种前缀能匹配、剩下的 `.5` 认不出的名字,静默按默认契约跑会拿错尺度
观测、拿错区间解码,而且全程不报错;打错一个字母的后缀同理。
scale 是**训练时**观测里速度通道的归一化尺度,不是部署速度(同 v1 的 SPEED_SCALE)。
V_HI 是动作空间能表达的最高目标速度 —— 它设小了,策略物理上就快不起来。
见 docs/spica方案.md §3.2。
"""
import re
from typing import NamedTuple

import numpy as np

from common.obs import sector_index, sectorize

SCHEMA = 'road_v2'
SCHEMA_SPICA = 'spica_v3'
DIM, FRAMES = 53, 5
SPEED_SCALE = 1.5
OBS_RANGE = 10.0
V_LO, V_HI = 0.45, 1.0      # 无后缀 schema 的默认速度契约:0.45 是建模下限,不输出 0(停车归防撞和人)
# spica 是同一套观测/网络的顶层层级名(无后缀 = 和 road_v2 逐位等价),见 common/spica.py
SCHEMA_PREFIXES = (SCHEMA, SCHEMA_SPICA)

# 2026-09-30:schema 尾部加这个后缀 = **网络输入层物理上不含 last_action 两列**。
# 环境照旧吐 53 维观测(不动 DIM,免得动到 v1 兼容和 rescale_spica),网络只接 255 列。
# 为什么是"删"不是"置零":置零后网络收到的输入分布是它从没见过的(训练时该列标准差
# 约 0.4,突然恒 0)= OOD,闭环全灭(SS_zl 0.000);删列则连这一列都不存在,不存在 OOD。
# 动机:该通道让策略"抄上一步",闭环里退化成自激反馈环
# (官方赛道 A 上 0/30 -> 剪断后 30/30,α≈0.4 相变)。
NOLA = '_nola'
# 每帧第 51/52 列是 last_action[0]/[1];展平缓冲是帧主序,所以帧内偏移就是 i % DIM
KEEP_COLS = np.array([i for i in range(DIM * FRAMES) if (i % DIM) not in (51, 52)], dtype=np.intp)


def drop_last_action(schema) -> bool:
    return str(schema).endswith(NOLA)


class Encoding(NamedTuple):
    """一局/一个模型的速度契约:观测里速度通道的归一化尺度 + 动作解码出的速度区间。"""
    scale: float = SPEED_SCALE
    v_lo: float = V_LO
    v_hi: float = V_HI


DEFAULT_ENC = Encoding()
_SUFFIX = re.compile(r'^_s(?P<scale>[0-9.]+)_v(?P<lo>[0-9.]+)-(?P<hi>[0-9.]+)$')


def spec_of(schema) -> Encoding:
    """schema -> Encoding。无后缀(只带前缀)返回默认,历史模型逐位不变。

    `_nola` 只描述网络输入宽度,不参与速度契约,先剥掉再匹配后缀。
    剥掉之后还有剩余又匹配不上后缀的,是打错的名字(或别人家的命名),报错而不是
    当成默认契约 —— 静默回落会让整个模型按错的尺度和区间跑,不报错。
    """
    s = str(schema)
    prefix = next((p for p in SCHEMA_PREFIXES if s.startswith(p)), None)
    if prefix is None:
        raise ValueError(f'不认识的 schema {s}')
    rest = s[len(prefix):]
    if rest.endswith(NOLA):
        rest = rest[:-len(NOLA)]
    if not rest:
        return DEFAULT_ENC
    m = _SUFFIX.match(rest)
    if m is None:
        raise ValueError(f'schema {s} 的后缀 {rest!r} 不认识'
                         f'(能认的是 {_SUFFIX.pattern!r},或什么都不带 = 默认契约)')
    return Encoding(float(m['scale']), float(m['lo']), float(m['hi']))


def suffix_of(enc: Encoding) -> str:
    """Encoding -> schema 后缀。默认契约给空串,于是老模型的名字原样保留。"""
    return '' if tuple(enc) == tuple(DEFAULT_ENC) else f'_s{enc.scale:g}_v{enc.v_lo:g}-{enc.v_hi:g}'


def action_to_speed(a: float, enc: Encoding = DEFAULT_ENC) -> float:
    a = min(1.0, max(-1.0, a))
    return enc.v_lo + (a + 1.0) / 2.0 * (enc.v_hi - enc.v_lo)


def speed_to_action(v: float, enc: Encoding = DEFAULT_ENC) -> float:
    return min(1.0, max(-1.0, 2.0 * (v - enc.v_lo) / (enc.v_hi - enc.v_lo) - 1.0))


def observe(ranges, angles, speed, cap, yaw_rate, last_action, scale=None):
    scale = SPEED_SCALE if scale is None else float(scale)
    angles = (np.asarray(angles) + np.pi) % (2 * np.pi) - np.pi
    sectors = sectorize(ranges, sector_index(angles), OBS_RANGE)
    return np.clip(np.r_[sectors, speed / scale, cap / scale, yaw_rate / 3.,
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
    """两层 tanh + 线性输出,clip 到 ±1。w/b 是三层的权重列表。

    输入宽度由 w[0] 自己决定:255 列 = 无 last_action 的模型(见 KEEP_COLS)。
    这样所有调用点都不用改,`_nola` 模型和普通模型走同一条路径。
    """
    if w[0].shape[1] != x.shape[-1]:
        x = x[..., KEEP_COLS]
    for i in range(2):
        x = np.tanh(x @ w[i].T + b[i])
    return np.clip(x @ w[2].T + b[2], -1, 1)


class RoadPolicy:
    def __init__(self, path):
        with np.load(path) as z:
            schema = str(z['schema'])
            if not schema.startswith(SCHEMA_PREFIXES):
                raise ValueError(f'模型版本 {schema} 不是 {SCHEMA} / {SCHEMA_SPICA}')
            self.enc = spec_of(schema)
            self.w = [z[f'w{i}'].copy() for i in range(3)]
            self.b = [z[f'b{i}'].copy() for i in range(3)]
        self.history = History()

    def reset(self):
        self.history.reset()

    def __call__(self, obs) -> np.ndarray:
        return forward(self.w, self.b, self.history.push(obs)).astype(np.float32)
