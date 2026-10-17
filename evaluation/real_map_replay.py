"""实景地图闭环复现:拿车上 Logbook 一段的建图当场地,从实车那段的起点、按车上的条件(截停、地面分割投影 + 补缝、
无 YOLO、速度上限 0.7)让几个模型各开几局,和实车那次比开了多远、怎么结束的。

    python -m evaluation.real_map_replay runs/drv_1015b/logbook_1015_181810/seg_03 models/a.npz models/b.npz

每局换一个种子(车辆参数按 Plant.sample 抽,速度上限固定 0.7、起步等待固定 0.5 s);结束原因:
guard(车上截停,实车到这一步基本卡死)、collision、unknown(开出建图时见过的范围,后面没有图,算"没出事")、timeout。
--png 画图:车上同款点阵图 + 实车路径(蓝)+ 各模型每局路径(红 / 橙 / 紫)。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import math
from collections import Counter
from dataclasses import replace

import numpy as np

CAP = 0.7               # 车上网页设的速度上限
SEG_PERIOD, SEG_LAG = 0.33, 0.35     # 车上 M2F-T 288 地面分割的周期 / 延迟(training/bev_env 文档)
WARM = 2.0              # 出发前原地攒 BEV 的时间 s:车上放行前 BEV 早就在攒了,第一拍不是只有一帧雷达、没有地面层
                        # (不攒的话 1014 seg_04 三个模型第一拍都往右前一个近处障碍打方向、当场截停,实车那拍只打了 -0.05)
COLORS = [(40, 40, 220), (0, 140, 255), (180, 60, 160), (60, 160, 60), (120, 120, 0)]     # 模型路径(BGR):红、橙、紫、绿、青;实车是蓝


def real_path(seg, ref=None, T=None):
    """实车这一段的路径 (n, 7)(t x y yaw ...);给了参考图就把位姿变换到参考系。"""
    P = np.load(os.path.join(seg, 'seg.npz'))['path'].copy()
    if ref is not None:
        from evaluation.seg_align import apply_pose
        P[:, 1:4] = apply_pose(T, P[:, 1:4])
    return P


def load_world(seg, ref=None, T=None, lidar_floor=True):
    """段的建图(给了 ref:参考图 + 参考图没覆盖到的地方用这段自己的图,见 seg_align.merged_world),
    实车车身扫过的障碍格清掉(那时那里没东西:走动的人、出发时站在车边的人)。
    lidar_floor:雷达确认空的格也算地面(见 RealMap.from_seg;实车没拐的岔口相机没看过,不补的话那边没有地面层)。"""
    from common.params import CarParams
    from simulation.fastsim.real_map import RealMap
    p = CarParams()
    if ref is None:
        world = RealMap.from_seg(os.path.join(seg, 'seg.npz'), lidar_floor=lidar_floor)
    else:
        from evaluation.seg_align import merged_world
        world = merged_world(ref, seg, T, lidar_floor)
    world.clear_swept(real_path(seg, ref, T)[:, 1:4], p.length, p.width, p.rear_overhang)
    world.drop_small(3)                     # 人被车身扫掉一半,剩下的碎格(1014 seg_04 出发点右前 0.4 m 一个孤格,每局当场截停)
    return world


def run_one(args):
    seg, ref, T, model, start, seed, duration, lidar_floor = args
    from common.bev_policy import BevPolicy
    world = load_world(seg, ref, T, lidar_floor)
    from simulation.fastsim.road_env import Plant
    from training.bev_env import BevEnv
    pol = BevPolicy(model)
    pl = replace(Plant.sample(np.random.default_rng(seed), pol.enc), cap=CAP, launch_delay=WARM + 0.5)
    b = BevEnv(None, pl, seed, start, noise=True, free=pol.free, seg_period=SEG_PERIOD, seg_lag=SEG_LAG,
               guard=True, world=world)
    b.env.budget_s = WARM + duration
    while b.env.t < WARM - 1e-9:             # 原地攒 BEV:车还没放行,不截停(速度环在 launch_delay 内不动车)
        b.env.step(pol(*b.observation()))
        b._camera()
    why, path = None, []
    while why is None:
        why = b.step(pol(*b.observation()))
        path.append(b.pose[:2])
    return why, b.env.dist, b.env.t - WARM, np.array(path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('seg', help='Logbook 段目录(含 seg.npz)')
    ap.add_argument('models', nargs='+')
    ap.add_argument('--runs', type=int, default=8)
    ap.add_argument('--duration', type=float, default=None, help='每局最长 s(默认 = 实车这段的时长 + 10)')
    ap.add_argument('--png', default=None)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--no-lidar-floor', action='store_true', help='地面层只用相机建图时看过的(实车没拐的岔口没有地面)')
    ap.add_argument('--ref', default=None, help='参考图段目录(扫描匹配建的录包图),配 --T')
    ap.add_argument('--T', type=float, nargs=3, default=None, metavar=('X', 'Y', 'DEG'),
                    help='段系 -> 参考系的变换(evaluation.seg_align 求)')
    a = ap.parse_args(argv)
    T = None if a.T is None else (a.T[0], a.T[1], math.radians(a.T[2]))
    world = load_world(a.seg, a.ref, T, not a.no_lidar_floor)
    P = real_path(a.seg, a.ref, T)                                       # t x y yaw steer target speed
    start = tuple(P[0, 1:4])
    real_len = float(np.hypot(*np.diff(P[:, 1:3], axis=0).T).sum())
    dur = a.duration or (P[-1, 0] - P[0, 0]) + 10.0
    print(f'{a.seg}:实车 {P[-1, 0] - P[0, 0]:.1f} s、{real_len:.1f} m;每局最长 {dur:.0f} s')
    paths = {}
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(a.workers) as ex:
        res = list(ex.map(run_one, [(a.seg, a.ref, T, m, start, 1000 + k, dur, not a.no_lidar_floor) for m in a.models for k in range(a.runs)]))
    for n, m in enumerate(a.models):
        rows = res[n * a.runs:(n + 1) * a.runs]
        paths[m] = [r[3] for r in rows]
        d = [r[1] for r in rows]
        print(f'  {os.path.basename(m):16s} 结束 {dict(Counter(r[0] for r in rows))}  开了 中位 {np.median(d):.1f} m '
              f'(最少 {min(d):.1f} / 最多 {max(d):.1f})')
    if a.png:
        import cv2
        img = draw(cv2, world, P[:, 1:3], paths)
        cv2.imwrite(a.png, img)
        print('图 ->', a.png)


def draw(cv2, world, real, paths, margin=3.0):
    """车上同款样式(simulation/fastsim/map_view):白底、黑点 = 障碍、浅绿点 = 地面、1 m 网格;
    实车路径 = 蓝粗线(起点黄圈),各模型每局路径 = 细线(颜色按模型)。裁到所有路径周围 margin 米。"""
    from simulation.fastsim.map_view import MapView
    allp = np.vstack([real] + [p for ps in paths.values() for p in ps if len(p)])
    lo, hi = allp.min(0) - margin, allp.max(0) + margin
    view = MapView((lo[0], lo[1], hi[0], hi[1]), world.cell)
    img = view.base(cv2, np.argwhere(world.occ) + (world.i0, world.j0), np.argwhere(world.free) + (world.i0, world.j0))
    for k, (m, ps) in enumerate(paths.items()):
        for p in ps:
            view.path(cv2, img, p, COLORS[k % len(COLORS)], 1)
    view.path(cv2, img, real, width=3)
    cv2.circle(img, tuple(int(v) for v in view.uv(real[:1])[0]), 8, (0, 200, 255), 2)
    cv2.putText(img, 'real', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 90, 20), 1, cv2.LINE_AA)
    for k, name in enumerate(paths):
        cv2.putText(img, os.path.basename(name), (8, 40 + 20 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    COLORS[k % len(COLORS)], 1, cv2.LINE_AA)
    return img


if __name__ == '__main__':
    main()
