"""BEV 模型的直道平顺度验收:14 m 全墙直走廊(6 种宽度),两项。

A 横移扫描:模型自己开 2 s(BEV 记忆攒起来,和开车时一样),停在那一刻,把裁图位姿横向平移(每步 0.5 cm,±15 cm)
  重新出图,看转角是不是平滑。BEV 是 5 cm 格,墙跨一格时输入整片挪一格;对格子量化敏感的模型在这里锯齿状乱跳。
  (只扫一帧新扫描、没有记忆的图不在训练分布里,会把好模型也测成乱跳,2026-10-16 改成先开 2 s。)
B 闭环:实车节奏(地面分割 0.33 s / 0.35 s)带噪声开 12 s,每 0.3 s 取一次转角(和车上逐帧回看同一采样),
  报变化量、正负翻转比例、横向误差标准差。v3.5m5 这里翻转 0.43,实车走廊 0.41。

    python -m evaluation.bev_sweep models/xxx.npz [更多模型 ...]

验收(v4 起):A max_jump 中位 <= 2°、反向步 <= 10%;B |d| p90 明显小于 v3.5m5(8.3°)。
"""
from __future__ import annotations

import argparse
import math

import numpy as np

WIDTHS = (1.0, 1.2, 1.4, 1.7, 2.0, 2.4)
OFFSETS = np.arange(-0.15, 0.1501, 0.005)


def check(pol):
    from common.bev_policy import bev_image
    from common.params import CarParams
    from common.pwm import action_to_steer
    from simulation.fastsim.road import WALL, build
    from simulation.fastsim.road_env import Plant
    from training.bev_env import BevEnv
    p = CarParams()

    def deg(a):
        return math.degrees(action_to_steer(float(np.clip(a, -1, 1)), p))
    jump, back, flips, lat, dd = [], [], [], [], []
    for k, w in enumerate(WIDTHS):
        road = build([dict(kind='straight', length=14.0, width=w, mat=(WALL, WALL))])
        pl = Plant.sample(np.random.default_rng(k))
        # A
        b = BevEnv(road, pl, k, (0, 0.0, 0.0), noise=False, free=pol.free, seg_period=0.33, seg_lag=0.35)
        for _ in range(40):
            b.step(pol(*b.observation()))
        x, y, yaw = b.pose
        i, d0 = road.project((x, y))
        n = road.normal[i]
        _, sc = b.observation()
        s = np.array([deg(pol(bev_image(b.bev, (x + (o - d0) * n[0], y + (o - d0) * n[1], yaw), b.env.t,
                                        b.env.ranges, b.env.angles, b.scan_pose, free=pol.free), sc)[0])
                      for o in OFFSETS])
        d = np.diff(s)
        jump.append(np.abs(d).max())
        back.append(float((d > 0.5).mean()))        # 越往左越该往右打(转角变小),变大 0.5° 以上算反向
        # B
        b = BevEnv(road, pl, 100 + k, (0, 0.0, 0.0), noise=True, free=pol.free, seg_period=0.33, seg_lag=0.35)
        st, la, why = [], [], None
        while why is None and b.env.t < 12.0:
            a = pol(*b.observation())
            st.append(deg(a[0]))
            why = b.step(a)
            la.append(road.project((b.env.car.x, b.env.car.y))[1])
        st = np.array(st)[40::6]                     # 跳过起步 2 s,每 0.3 s 取一次
        dd.append(np.abs(np.diff(st)))
        sg = np.sign(st)
        flips.append(float((sg[1:] * sg[:-1] < 0).mean()))
        lat.append(float(np.std(la[40:])))
    d = np.concatenate(dd)
    return dict(jump_med=float(np.median(jump)), jump_max=float(np.max(jump)), back=float(np.mean(back)),
                d_med=float(np.median(d)), d_p90=float(np.percentile(d, 90)), flips=float(np.mean(flips)),
                lat_cm=100 * float(np.mean(lat)))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('models', nargs='+')
    a = ap.parse_args(argv)
    from common.bev_policy import BevPolicy
    for m in a.models:
        r = check(BevPolicy(m))
        print(f'{m}: A max_jump 中位 {r["jump_med"]:.1f}° 最大 {r["jump_max"]:.1f}° 反向步 {r["back"]:.0%} | '
              f'B 每 0.3 s 变化 中位 {r["d_med"]:.1f}° p90 {r["d_p90"]:.1f}° 翻转 {r["flips"]:.2f} 横向 {r["lat_cm"]:.1f} cm')


if __name__ == '__main__':
    main()
