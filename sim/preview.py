"""画一张 PNG:赛道锥桶 + 纯追踪跑一圈的轨迹 + 最后一帧雷达点。

    python -m sim.preview --seed 0 --out sim_preview.png
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
    args = ap.parse_args()
    env = ConeTrackEnv()
    env.reset(seed=args.seed)
    path, info = [], {}
    while True:
        _, _, term, trunc, info = env.step(pure_pursuit(env))
        path.append((env.car.x, env.car.y))
        if term or trunc:
            break
    path = np.array(path)
    ang = env.lidar.angles + env.car.yaw
    ok = np.isfinite(env.ranges)
    lx = env.car.x + env.cp.lidar_x * math.cos(env.car.yaw) + env.ranges[ok] * np.cos(ang[ok])
    ly = env.car.y + env.cp.lidar_x * math.sin(env.car.yaw) + env.ranges[ok] * np.sin(ang[ok])

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(*env.track.centerline.T, color="#ccc", lw=1, label="中心线")
    ax.scatter(*env.track.left.T, c="#d33", s=14, label="左锥(红)")
    ax.scatter(*env.track.right.T, c="#36c", s=14, label="右锥(蓝)")
    ax.plot(*path.T, color="#1a7f4b", lw=1.5, label="轨迹")
    ax.scatter(lx, ly, s=2, c="k", label="雷达(最后一帧)")
    ax.set_aspect("equal")
    ax.set_title(f"seed={args.seed} 结果={info['termination_reason']} 用时={info['time']:.1f}s "
                 f"撞锥={info['cone_hits']}")
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    ax.legend(loc="upper right", fontsize=8)
    fig.savefig(args.out, dpi=120, bbox_inches="tight")
    print(args.out, info)


if __name__ == "__main__":
    main()
