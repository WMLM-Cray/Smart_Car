"""把一个模型从一个速度契约**无损**迁移到另一个(53 维 road_v2 / spica 布局)。

## 为什么需要

`common/spica.load_init` 对 road_v2/spica 模型做的是**裸拷贝**,不做任何缩放。
拿 scale 1.5 的模型去热启动 `--speed-scale 3.0` 的训练,网络看到的是 `speed/3.0`,
而它训练时学的是 `speed/1.5` —— 同一物理速度的输入只有一半大,网络以为车开得
比实际慢一半,转向不足,直接冲出赛道。2026-09-29 的 fam4 就是这么从 0.817 掉到
0.315 的,而且**连最低速档都只有 0.478**(同速下原模型 0.817),正是这个特征。

v1 的 52 维布局有对应的实现(仓库历史里的 `fastsim/rescale_policy.py`),本模块是它在
53 维 road_v2 / spica 布局上的对应物。

## 迁移分两部分

**输入列**:观测里的速度通道是 `speed/scale` 和 `cap/scale`,只差这两维的线性
缩放。对同一个物理速度 `obs_new = speed/s_new = obs_old · (s_old/s_new)`,要让
网络响应不变就把输入权重列反向缩放:

    W_new[:, 48/49] = W_old[:, 48/49] · (s_new / s_old)

**速度输出头**:动作空间 `action_to_speed(a)` 的区间也换了,所以光缩放输入会让
同一个输出 `a` 对应到另一个物理速度。要"无损"就得让同一批输入产生的**物理速度**
不变。设 `k = (v_hi_o-v_lo_o)/(v_hi_n-v_lo_n)`、`c = 2(v_lo_o-v_lo_n)/(v_hi_n-v_lo_n)`,
要求 `a_new = c + k(a_old+1) - 1`,于是

    w2_new[1] = k · w2_old[1]
    b2_new[1] = k · (b2_old[1] + 1) + c - 1

转向行 `w2[0]` / `b2[0]` 原样不动 —— 转向的动作语义没变。

## 用法

    python3 -m common.rescale_spica --in runs/spica_a/candidate_003.npz \
        --out runs/init/cand003_s3.npz --speed-scale 3.0 --v-hi 3.0

    # 自检:同一物理状态下两个网络解出的速度应逐位相同
    python3 -m common.rescale_spica --in X.npz --out Y.npz --speed-scale 3.0 --v-hi 3.0 --check
"""
from __future__ import annotations

import argparse

import numpy as np

from common import spica
from common.road_policy import (DIM, FRAMES, KEEP_COLS, NOLA, SCHEMA, SCHEMA_SPICA, V_HI, V_LO,
                                Encoding, action_to_speed, drop_last_action, spec_of,
                                speed_to_action, suffix_of)

# 每帧:0-47 扇区, 48 speed, 49 cap, 50 yaw_rate, 51 last_action[0], 52 last_action[1]
# `_nola` 模型输入层是 255 列(每帧少 51/52),所以列号不能写死成 265 布局的下标 ——
# 由 w0 的实际宽度反推:下面两个函数给的是"在这个 w0 里"的下标。
_ORIG = {DIM * FRAMES: np.arange(DIM * FRAMES), len(KEEP_COLS): KEEP_COLS}


def layout_of(n_in: int) -> np.ndarray:
    """w0 的列号 -> 265 列布局里的原始列号(帧主序)。认不出宽度就报错。"""
    if n_in not in _ORIG:
        raise ValueError(f'输入层 {n_in} 列,既不是 {DIM * FRAMES} 也不是 {len(KEEP_COLS)}(_nola)')
    return _ORIG[n_in]


def speed_cols(n_in: int):
    """w0 里速度通道(speed/cap)的列号。"""
    return tuple(i for i, o in enumerate(layout_of(n_in)) if o % DIM in (48, 49))


def speed_act_cols(n_in: int):
    """w0 里 last_action[1](上一步速度动作)的列号。"""
    return tuple(i for i, o in enumerate(layout_of(n_in)) if o % DIM == 52)


SPEED_COLS = speed_cols(DIM * FRAMES)
SPEED_ACT_COLS = speed_act_cols(DIM * FRAMES)


def rescale_input(w0: np.ndarray, b0: np.ndarray, eo: Encoding, en: Encoding):
    """输入层重标定。返回 (w0, b0) 新数组。

    两类列:
      - speed / cap(帧内偏移 48/49):`v/scale` 是纯线性缩放,列乘 s_new/s_old 即可。
      - last_action[1](帧内偏移 52):它的语义是"我发出的速度动作",而动作->速度的
        仿射映射也变了。同一物理速度下 `a_new = k·a_old + (k-1)`,带常数项。
        要让线性层贡献 `w_new·x_new + b_new = w_old·x_old + b_old` 恒成立,
        解得 `w_new = w_old/k`、`b_new = b_old - (k-1)/k·w_old[:,52]`。
      - last_action[0](转向)语义不变,原样。

    列号按 w0 自己的宽度算,`_nola`(255 列)和普通(265 列)走同一条路。
    """
    sp, sa = speed_cols(w0.shape[1]), speed_act_cols(w0.shape[1])
    w0 = np.array(w0, copy=True)
    b0 = np.array(b0, copy=True)
    w0[:, list(sp)] *= en.scale / eo.scale
    k = (eo.v_hi - eo.v_lo) / (en.v_hi - en.v_lo)
    for c in sa:
        b0 -= (k - 1.0) / k * w0[:, c]
        w0[:, c] /= k
    return w0, b0


def rescale_head(w2: np.ndarray, b2: np.ndarray, eo: Encoding, en: Encoding):
    """把速度输出行重标定,使同一批输入产生的物理速度不变。返回 (w2, b2) 新数组。"""
    k = (eo.v_hi - eo.v_lo) / (en.v_hi - en.v_lo)
    c = 2.0 * (eo.v_lo - en.v_lo) / (en.v_hi - en.v_lo)
    w2 = np.array(w2, copy=True)
    b2 = np.array(b2, copy=True)
    w2[1] *= k
    b2[1] = k * (b2[1] + 1.0) + c - 1.0
    return w2, b2


def convert(src: str, enc_new: Encoding):
    """读任意 v1 / road_v2 / spica 模型,返回 (权重 dict, 源 Encoding, 源是不是 _nola)。

    `_nola` 的输入层宽度跟着权重走(255 列),转出来还是 255 列,所以标志要一路带出去
    —— 写回 schema 时少了它,模型就变成"265 列的名字配 255 列的权重"。
    """
    with np.load(src) as z:
        schema = str(z['schema'])
    nola = drop_last_action(schema)
    if spica.is_v1(schema):
        enc_old = Encoding(1.5, V_LO, V_HI)          # v1 只有 scale 1.5 这一种契约
        w = {k: v for k, v in spica.remap_v1_to_spica(src).items() if k != 'schema'}
    elif schema.startswith((SCHEMA, SCHEMA_SPICA)):
        enc_old = spec_of(schema)
        with np.load(src) as z:
            w = {k: z[k].copy() for k in ('w0', 'w1', 'w2', 'b0', 'b1', 'b2')}
    else:
        raise ValueError(f'{src} 的 schema {schema} 不认识')
    if (w['w0'].shape[1] == len(KEEP_COLS)) != nola:
        raise ValueError(f'{src}:schema 说 nola={nola},但 w0 是 {w["w0"].shape[1]} 列,对不上')
    w['w0'], w['b0'] = rescale_input(w['w0'], w['b0'], enc_old, enc_new)
    w['w2'], w['b2'] = rescale_head(w['w2'], w['b2'], enc_old, enc_new)
    return w, enc_old, nola


def _obs(base, v, cap, a_last, enc, nola: bool = False):
    """按给定契约造一段(FRAMES 帧)展平观测,每帧同值 —— 展平缓冲是帧主序。

    `base` 是共享的随机噪声帧:新旧两次调用必须喂**同一份**噪声,否则比的是
    两组不同输入,偏差没有意义。`nola=True` 时按 255 列的布局摆(每帧少了 51/52)。
    """
    f = base.copy()
    f[48] = v / enc.scale
    f[49] = cap / enc.scale
    if not nola:
        f[51] = a_last[0]
        f[52] = a_last[1]
    return np.tile(f, FRAMES)[KEEP_COLS] if nola else np.tile(f, FRAMES)


def check(src: str, w_new: dict, enc_old: Encoding, enc_new: Encoding, n: int = 128) -> float:
    """自检:同一物理状态下,新旧两个网络解出的速度(以及转向)应逐位相同。

    关键在于 `last_action[1]` 也要按各自契约换算 —— 它存的是"上一步发出的动作",
    同一物理速度在新旧契约里对应不同的动作值。输入宽度按 w0 自己来:`_nola` 的
    网络看不到 last_action,自检就得按 255 列喂,否则比的是两个不同的网络。
    """
    nola = w_new['w0'].shape[1] == len(KEEP_COLS)
    w_old, _, _ = convert(src, enc_old)
    rng = np.random.default_rng(0)
    worst_v = worst_s = 0.0
    for _ in range(n):
        v = rng.uniform(enc_old.v_lo, enc_old.v_hi)
        cap = rng.uniform(enc_old.v_lo, enc_old.v_hi)
        v_last = rng.uniform(enc_old.v_lo, enc_old.v_hi)
        a_old_last = (rng.uniform(-1, 1), float(speed_to_action(v_last, enc_old)))
        a_new_last = (a_old_last[0], float(speed_to_action(v_last, enc_new)))
        base = rng.uniform(-1, 1, DIM).astype(np.float32)
        xo = _obs(base, v, cap, a_old_last, enc_old, nola)
        xn = _obs(base, v, cap, a_new_last, enc_new, nola)
        ao, an = _fwd(w_old, xo), _fwd(w_new, xn)
        worst_s = max(worst_s, abs(ao[0] - an[0]))
        worst_v = max(worst_v, abs(action_to_speed(float(ao[1]), enc_old)
                                   - action_to_speed(float(an[1]), enc_new)))
    return worst_s, worst_v


def _fwd(w, x):
    h = x
    for i in (0, 1):
        h = np.tanh(h @ w[f'w{i}'].T + w[f'b{i}'])
    return np.clip(h @ w['w2'].T + w['b2'], -1, 1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--in', dest='src', required=True)
    ap.add_argument('--out', dest='dst', default=None)
    ap.add_argument('--speed-scale', type=float, required=True)
    ap.add_argument('--v-hi', type=float, default=None, help='默认等于 --speed-scale')
    ap.add_argument('--check', action='store_true', help='只自检,不写文件')
    a = ap.parse_args()

    enc_new = Encoding(float(a.speed_scale), V_LO, float(a.v_hi or a.speed_scale))
    if enc_new.v_hi <= enc_new.v_lo:
        raise SystemExit('--v-hi 必须大于 0.45')

    w, enc_old, nola = convert(a.src, enc_new)
    out_schema = SCHEMA_SPICA + suffix_of(enc_new) + (NOLA if nola else '')
    print('schema %s (%s) -> %s' % (str(np.load(a.src)['schema']), enc_old, out_schema))
    print('  scale %.4g -> %.4g   输入速度列 x%.4g'
          % (enc_old.scale, enc_new.scale, enc_new.scale / enc_old.scale))
    print('  动作区间 [%.2f,%.2f] -> [%.2f,%.2f]'
          % (enc_old.v_lo, enc_old.v_hi, enc_new.v_lo, enc_new.v_hi))
    print('  输入层 %d 列%s' % (w['w0'].shape[1], '(last_action 已删,不参与重标定)' if nola else ''))

    if a.check:
        ws, wv = check(a.src, w, enc_old, enc_new)
        print('  自检: 转向最大偏差 %.3e   速度最大偏差 %.3e m/s' % (ws, wv))
        return

    if not a.dst:
        raise SystemExit('要写文件就加 --out')
    np.savez(a.dst, schema=np.array(out_schema), **w)
    ws, wv = check(a.src, w, enc_old, enc_new)
    print('  自检: 转向最大偏差 %.3e   速度最大偏差 %.3e m/s' % (ws, wv))
    print('写出', a.dst)


if __name__ == '__main__':
    main()
