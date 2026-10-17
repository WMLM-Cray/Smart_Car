"""fastsim 一局做成视频:左边是车上同款全局图(累积的雷达障碍 + 地面分割,simulation/fastsim/map_view),
右边是模型那一拍的 BEV 输入。现在支持 evaluation/l_test 的 L 型拐角。

    python -m evaluation.sim_video models/a.npz --out runs/l_video/a.mp4 --width 6 --open-left --yaw 0 --seed 7000

全局图和车上一样按 Logbook 累积(雷达射线穿过的格减分、地面分割按拍摄位姿登记),所以看到的就是车"已经看到过"的东西。
视频 20 帧/s,每帧 = 仿真 0.1 s(2 倍速)。
"""
from __future__ import annotations

import argparse
import math
import os
import tempfile
from dataclasses import replace

import numpy as np


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('model')
    ap.add_argument('--out', required=True)
    ap.add_argument('--width', type=float, default=6.0)
    ap.add_argument('--open-left', action='store_true')
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--seed', type=int, default=7000)
    a = ap.parse_args(argv)
    import cv2
    from common.bev import lidar_points
    from common.bev_policy import BevPolicy
    from common.pwm import action_to_steer
    from common.road_policy import V_LO, Encoding
    from evaluation.l_test import FRONT, RIGHT, ell
    from simulation.fastsim.map_view import MapView, bev_tile
    from simulation.fastsim.road_env import Plant
    from training.bev_env import BevEnv
    from vehicle.bev_logbook import OCC_ON, Logbook

    road = ell(a.width, a.open_left)
    pl = replace(Plant.sample(np.random.default_rng(a.seed), Encoding(3.0, V_LO, 3.0)), cap=0.7)
    dist = FRONT - a.width - 0.05
    i0 = road.elems[1]['_i0'] - int(round(dist / 0.05))
    b = BevEnv(road, pl, a.seed, (i0, -a.width / 2 + RIGHT, math.radians(a.yaw)), noise=True, free=True,
               seg_period=0.33, seg_lag=0.35, yolo=False, guard=True)
    pol = BevPolicy(a.model)
    env, p = b.env, b.env.cp

    book = Logbook(tempfile.mkdtemp())
    def on_scan(pose):
        x, y, yaw = pose
        book.add_lidar(lidar_points(env.ranges, env.angles, pose, p, pl.lidar_yaw, 10.0),
                       (x + p.lidar_x * math.cos(yaw), y + p.lidar_x * math.sin(yaw)))
    b.scan_hooks.append(on_scan)
    on_scan(b.scan_pose)
    detect = b.seg.detect
    def detect_and_log(pose):
        xy = detect(pose)
        book.add_free(xy, pose)
        return xy
    b.seg.detect = detect_and_log

    w = road.walls.reshape(-1, 2)
    lo, hi = w.min(0) - 1.0, w.max(0) + 1.0
    view = MapView((lo[0], lo[1], hi[0], hi[1]))
    scale = min(1.0, 900 / view.h)
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    vw = None
    path, why, k = [], None, 0
    name = os.path.basename(a.model).replace('.npz', '')
    while True:
        img, sc = b.observation()
        act = pol(img, sc)
        steer = action_to_steer(float(np.clip(act[0], -1, 1)), p)
        path.append(b.pose[:2])
        if k % 2 == 0 or why is not None:
            m = view.base(cv2, book.lidar.ij(OCC_ON)[0], book.free.ij(1)[0])
            view.path(cv2, m, np.array(path))
            view.steer(cv2, m, b.pose, steer)
            view.car(cv2, m, b.pose)
            m = cv2.resize(m, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            right = np.full((m.shape[0], 340, 3), 255, np.uint8)
            right[10:330, 10:330] = bev_tile(cv2, img)
            lines = [name, f't = {env.t:5.1f} s', f'steer {math.degrees(steer):+5.1f} deg', f'speed {env.car.v:4.2f} m/s',
                     f'L width {a.width:g} m{" open-left" if a.open_left else ""}', f'yaw0 {a.yaw:+g} deg  seed {a.seed}']
            if why is not None:
                lines.append(f'END: {why}')
            for n, s in enumerate(lines):
                cv2.putText(right, s, (12, 360 + 26 * n), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (40, 40, 220) if s.startswith('END') else (0, 0, 0), 1, cv2.LINE_AA)
            frame = np.hstack([m, right])
            if vw is None:
                vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), 20, (frame.shape[1], frame.shape[0]))
            vw.write(frame)
            if why is not None:
                for _ in range(40):                                     # 结尾停 2 s
                    vw.write(frame)
                cv2.imwrite(a.out.replace('.mp4', '_end.png'), frame)
                break
        why = b.step(act)
        if why is None and env.t >= 30.0:
            why = 'timeout'
        k += 1
    vw.release()
    print(f'{name}: {why}  t={env.t:.1f}s -> {a.out}')


if __name__ == '__main__':
    main()
