"""段对齐(evaluation/seg_align.py):把一张图转一个角度、挪一段后,能找回变换。"""
import math

import numpy as np

from evaluation.seg_align import align, apply


def floor_plan():
    """L 形走廊 + 几个门洞和柱子(不对称,免得有多个解),5 cm 一个点。"""
    s = np.arange(0, 12, 0.05)
    t = np.arange(0, 6, 0.05)
    pts = [np.c_[s, np.zeros_like(s)],                          # 走廊下墙
           np.c_[s[s > 1.5], np.full((s > 1.5).sum(), 1.5)],    # 走廊上墙(左头留口)
           np.c_[np.zeros_like(t), -t],                         # 往下拐的支路两侧
           np.c_[np.full((t < 4.5).sum(), 1.5), -t[t < 4.5]],
           np.c_[6.0 + np.arange(8) * 0.05, np.full(8, 0.6)]]   # 走廊里一段隔板
    return np.vstack(pts)


def test_align_recovers_transform():
    dst = floor_plan()
    T = (3.0, -2.0, math.radians(37.0))
    c, s = math.cos(T[2]), math.sin(T[2])
    inv = (-(c * T[0] + s * T[1]), s * T[0] - c * T[1], -T[2])              # 参考系 -> 段系
    src = apply(inv, dst[::2])
    est, inl, _ = align(src, dst)
    assert abs(est[0] - T[0]) < 0.05 and abs(est[1] - T[1]) < 0.05
    assert abs((est[2] - T[2] + math.pi) % (2 * math.pi) - math.pi) < math.radians(1.0)
    assert inl > 0.95
