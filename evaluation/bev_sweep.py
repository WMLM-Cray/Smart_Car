"""BEV 模型的横移扫描:14 m 全墙直走廊(几种宽度)、同一位置,车只横向平移(每步 0.5 cm,±15 cm),看模型转角是不是平滑单调。

BEV 是 5 cm 格,墙跨一格时网络输入整片挪一格。对格子量化敏感的模型在这里会锯齿状乱跳
(v3.5m5:相邻两步最大跳 10°,横移 30 cm 总共才变 3°,2026-10-15),上车就是直道左右来回修。

    python -m evaluation.bev_sweep models/xxx.npz [--widths 1.0,1.4,2.0]

每个宽度报:相邻步最大跳变、总变化(偏右 -> 偏左,合格的模型应该明显往回打)、反向步占比。
最后一行是汇总;验收看 max_jump 和 total。
"""
from __future__ import annotations

import argparse
import math

import numpy as np

OFFSETS = np.arange(-0.15, 0.1501, 0.005)


def sweep(policy, road, pl, i0, free):
    from common.params import CarParams
    from common.pwm import action_to_steer
    from training.bev_env import BevEnv
    p = CarParams()
    out = []
    for off in OFFSETS:
        b = BevEnv(road, pl, 1, (i0, float(off), 0.0), noise=False, free=free, seg_period=0.33, seg_lag=0.0)
        out.append(math.degrees(action_to_steer(float(np.clip(policy(*b.observation())[0], -1, 1)), p)))
    return np.array(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('model')
    ap.add_argument('--widths', default='1.0,1.2,1.4,1.7,2.0,2.4', help='直走廊宽度 m,每个宽度一条')
    a = ap.parse_args(argv)
    from common.bev_policy import BevPolicy
    from simulation.fastsim.road import WALL, build
    from simulation.fastsim.road_env import Plant
    pol = BevPolicy(a.model)
    pl = Plant.sample(np.random.default_rng(0))
    jumps, totals, back = [], [], []
    for k, w in enumerate(float(v) for v in a.widths.split(',')):
        # 14 m 全墙直走廊,站在 2 m 处:前方 6 m 视野里没有弯,转角只该取决于横向位置
        road = build([dict(kind='straight', length=14.0, width=w, mat=(WALL, WALL))])
        s = sweep(pol, road, pl, 40, pol.free)
        d = np.diff(s)
        # 横向偏移从右(-)到左(+):车越偏左越该往右打(转角变负),反向步 = 转角变大的步
        jumps.append(np.abs(d).max())
        totals.append(s[-1] - s[0])
        back.append(float((d > 0.5).mean()))
        print(f'width {w:.1f}: max_jump {jumps[-1]:5.1f}°  total {totals[-1]:+6.1f}°  反向步 {back[-1]:.0%}  '
              f'{np.round(s[::6], 1).tolist()}')
    print(f'ALL: max_jump 中位 {np.median(jumps):.1f}° 最大 {np.max(jumps):.1f}°  total 中位 {np.median(totals):+.1f}°  '
          f'反向步 {np.mean(back):.0%}')


if __name__ == '__main__':
    main()
