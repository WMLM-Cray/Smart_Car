"""纯追踪跑一圈的二维可视化短视频(GIF)。

    python3 -m sim.video --track trackA --out trackA_pure_pursuit.gif
    python3 -m sim.video --track random --seed 0 --out sim_random.gif

先跑完整圈记录回放数据(位姿/轨迹/雷达帧),再逐帧渲染——比边跑边画稳,
轨迹总长不受渲染速度影响。控制器是 Pure Pursuit 参考控制器(preview 同款);
RL 策略未训练,本视频不含任何 RL 结果(如实声明,见 docs/SIM_INTEROP.md)。
"""
from __future__ import annotations

import argparse
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from .env import ConeTrackEnv
from .preview import pure_pursuit


def rect_corners(x, y, yaw, length, width, rear_overhang):
    """车体矩形四角(后轴参考系:x 前为 rear_overhang→length-rear_overhang)。"""
    c, s = math.cos(yaw), math.sin(yaw)
    lx = np.array([-rear_overhang, length - rear_overhang, length - rear_overhang, -rear_overhang])
    ly = np.array([-width / 2, -width / 2, width / 2, width / 2])
    return np.stack([x + lx * c - ly * s, y + lx * s + ly * c], axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", choices=["random", "trackA"], default="trackA")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="sim_video.gif")
    ap.add_argument("--lookahead", type=float, default=0.9)
    ap.add_argument("--throttle", type=float, default=0.35)
    ap.add_argument("--frame-every", type=float, default=0.3, help="多少仿真秒一帧")
    ap.add_argument("--fps", type=int, default=6)
    args = ap.parse_args()

    if args.track == "trackA":
        from .trackA import load_trackA
        env = ConeTrackEnv(track=load_trackA())
    else:
        env = ConeTrackEnv()
    env.reset(seed=args.seed)

    # ---- 记录回放 ----
    frames = []          # (time, car_pose, n_path_keep, scan_pts)
    path = []
    info = {}
    every = max(1, round(args.frame_every / env.control_dt))
    k = 0
    while True:
        _, _, term, trunc, info = env.step(
            pure_pursuit(env, lookahead=args.lookahead, throttle=args.throttle))
        path.append((env.car.x, env.car.y))
        if k % every == 0 or term or trunc:
            ang = env.lidar.angles + env.car.yaw
            ok = np.isfinite(env.ranges)
            lx = env.car.x + env.cp.lidar_x * math.cos(env.car.yaw) + env.ranges[ok] * np.cos(ang[ok])
            ly = env.car.y + env.cp.lidar_x * math.sin(env.car.yaw) + env.ranges[ok] * np.sin(ang[ok])
            frames.append((env.time, (env.car.x, env.car.y, env.car.yaw),
                           len(path), np.stack([lx, ly], axis=1)))
        k += 1
        if term or trunc:
            break
    path = np.asarray(path)
    print(f"回放 {len(frames)} 帧,仿真时长 {frames[-1][0]:.1f}s,结果 {info['termination_reason']},"
          f"撞锥 {info['cone_hits']},进度 {info['progress']:.1f}/{env.track.length:.1f}m")

    # ---- 渲染 ----
    fig, ax = plt.subplots(figsize=(6.4, 6.4), dpi=90)
    ax.plot(*env.track.centerline.T, color="#ccc", lw=1)
    ax.scatter(*env.track.left.T, c="#d33", s=10)
    ax.scatter(*env.track.right.T, c="#36c", s=10)
    (trail,) = ax.plot([], [], color="#1a7f4b", lw=1.5)
    (car_poly,) = ax.fill([], [], color="#1a7f4b", alpha=0.6, edgecolor="k", lw=0.8)
    scan = ax.scatter([], [], s=1.5, c="k")
    title = ax.set_title("")
    allx = np.concatenate([env.track.centerline[:, 0], path[:, 0]])
    ally = np.concatenate([env.track.centerline[:, 1], path[:, 1]])
    pad = 0.6
    ax.set_xlim(allx.min() - pad, allx.max() + pad)
    ax.set_ylim(ally.min() - pad, ally.max() + pad)
    ax.set_aspect("equal")

    p = env.cp

    def draw(i):
        t, (x, y, yaw), n_keep, pts = frames[i]
        trail.set_data(path[:n_keep, 0], path[:n_keep, 1])
        car_poly.set_xy(rect_corners(x, y, yaw, p.length, p.width, p.rear_overhang))
        off = pts - np.array([x, y])
        scan.set_offsets(pts[np.hypot(*off.T) > 0.45])      # 滤掉打车体的点
        # 标题用英文:WSL/无中文字体环境仍可复现(文档为中文说明)
        title.set_text(f"{args.track} Pure Pursuit  t={t:.1f}s  cone_hits={info['cone_hits']}")
        return trail, car_poly, scan, title

    anim = FuncAnimation(fig, draw, frames=len(frames), blit=False)
    anim.save(args.out, writer=PillowWriter(fps=args.fps))
    print(args.out, "已写出")


if __name__ == "__main__":
    main()
