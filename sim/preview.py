"""画一张 PNG:赛道锥桶 + 纯追踪跑一圈的轨迹 + 最后一帧雷达点。

    python -m sim.preview --seed 0 --out sim_preview.png          # 随机训练赛道
    python -m sim.preview --track trackA --out trackA_preview.png # 冻结 A 赛道
"""
import argparse
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .env import ConeTrackEnv


def pure_pursuit(env, lookahead=1.2, throttle=0.4):   # 动作 0.4 -> 0.4·v_cmd_fwd m/s
    """用赛道真值的参考控制器,只用于预览/测试,策略不能用。"""
    car, track = env.car, env.track
    s, _, _ = track.project((car.x, car.y))
    pts = track.centerline
    cum = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    target = pts[np.searchsorted(cum, (s + lookahead) % cum[-1]) % len(pts)]
    alpha = math.atan2(target[1] - car.y, target[0] - car.x) - car.yaw
    steer = math.atan2(2 * env.cp.wheelbase * math.sin(alpha), lookahead)
    return np.array([throttle, np.clip(steer / env.cp.max_steer, -1, 1)], np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="sim_preview.png")
    ap.add_argument("--track", choices=["random", "trackA"], default="random",
                    help="random=随机训练赛道(默认);trackA=冻结 A 赛道(不随机化)")
    ap.add_argument("--lookahead", type=float, default=1.2)
    ap.add_argument("--throttle", type=float, default=0.4)
    args = ap.parse_args()
    if args.track == "trackA":
        from .trackA import load_trackA
        env = ConeTrackEnv(track=load_trackA())
        env.reset(seed=args.seed)          # 出生位姿固定(11.0,0.75,朝+x);seed 只定噪声
    else:
        env = ConeTrackEnv()
        env.reset(seed=args.seed)
    path, info = [], {}
    while True:
        _, _, term, trunc, info = env.step(
            pure_pursuit(env, lookahead=args.lookahead, throttle=args.throttle))
        path.append((env.car.x, env.car.y))
        if term or trunc:
            break
    path = np.array(path)
    ang = env.lidar.angles + env.car.yaw
    ok = np.isfinite(env.ranges)
    lx = env.car.x + env.cp.lidar_x * math.cos(env.car.yaw) + env.ranges[ok] * np.cos(ang[ok])
    ly = env.car.y + env.cp.lidar_x * math.sin(env.car.yaw) + env.ranges[ok] * np.sin(ang[ok])

    fig, ax = plt.subplots(figsize=(8, 8))
    # 标签用英文:WSL/无中文字体环境仍可复现(文档为中文说明)
    ax.plot(*env.track.centerline.T, color="#ccc", lw=1, label="centerline")
    ax.scatter(*env.track.left.T, c="#d33", s=14, label="left cones (red)")
    ax.scatter(*env.track.right.T, c="#36c", s=14, label="right cones (blue)")
    ax.plot(*path.T, color="#1a7f4b", lw=1.5, label="pure pursuit path")
    ax.scatter(lx, ly, s=2, c="k", label="lidar (last frame)")
    ax.set_aspect("equal")
    ax.set_title(f"track={args.track} seed={args.seed} result={info['termination_reason']} "
                 f"time={info['time']:.1f}s cone_hits={info['cone_hits']}")
    ax.legend(loc="upper right", fontsize=8)
    fig.savefig(args.out, dpi=120, bbox_inches="tight")
    print(args.out, info)


if __name__ == "__main__":
    main()
