"""画一张 PNG:左边赛道锥桶 + 轨迹 + 最后一帧雷达点,右边跑出来的占据栅格图。

    python -m fastsim.preview --seed 0 --out sim_preview.png          # 纯追踪参考控制器
    python -m fastsim.preview --model runs/s1/model.zip --stage s1    # 训练出来的策略
"""
import argparse
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .curriculum import STAGES
from .env import ConeTrackEnv
from .mapping import FREE, OCCUPIED
from .pwm import steer_to_action


def pure_pursuit(env, lookahead=1.2, throttle=0.4):   # 动作 0.4 -> 0.4·v_cmd_fwd m/s
    """用赛道真值的参考控制器,只用于预览/测试,策略不能用。"""
    car, track = env.car, env.track
    s, _, _ = track.project((car.x, car.y))
    pts = track.centerline
    cum = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    target = pts[np.searchsorted(cum, (s + lookahead) % cum[-1]) % len(pts)]
    alpha = math.atan2(target[1] - car.y, target[0] - car.x) - car.yaw
    steer = math.atan2(2 * env.cp.wheelbase * math.sin(alpha), lookahead)
    return np.array([throttle, steer_to_action(steer, env.cp)], np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="sim_preview.png")
    ap.add_argument("--model", help="训练好的 PPO 权重;不给就用纯追踪参考控制器")
    ap.add_argument("--stage", default="s2", choices=list(STAGES), help="--model 时用哪个阶段的赛道")
    args = ap.parse_args()

    if args.model:
        from stable_baselines3 import PPO           # 只有画策略时才需要装 sb3
        from .train import StackedPolicy
        env = STAGES[args.stage].make_env()
        policy = StackedPolicy(PPO.load(args.model), env.observation_space.shape[0])
    else:
        env = ConeTrackEnv(mapping=True)
        policy = lambda obs, env: pure_pursuit(env)

    obs, info = env.reset(seed=args.seed)
    path = []
    while True:
        obs, _, term, trunc, info = env.step(policy(obs, env))
        path.append((env.car.x, env.car.y))
        if term or trunc:
            break
    path = np.array(path)
    ang = env.lidar.angles + env.car.yaw
    ok = np.isfinite(env.ranges)
    lx = env.car.x + env.cp.lidar_x * math.cos(env.car.yaw) + env.ranges[ok] * np.cos(ang[ok])
    ly = env.car.y + env.cp.lidar_x * math.sin(env.car.yaw) + env.ranges[ok] * np.sin(ang[ok])

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 7.5))
    ax.plot(*env.track.centerline.T, color="#ccc", lw=1, label="中心线")
    ax.scatter(*env.track.left.T, c="#d33", s=14, label="左锥(红)")
    ax.scatter(*env.track.right.T, c="#36c", s=14, label="右锥(蓝)")
    for i, board in enumerate(env.track.boards):
        ax.plot(*board.T, color="#0b3", lw=4, solid_capstyle="butt",
                label="挡板" if i == 0 else None)
    ax.plot(*path.T, color="#1a7f4b", lw=1.5, label="轨迹")
    ax.scatter(lx, ly, s=2, c="k", label="雷达(最后一帧)")
    ax.set_aspect("equal")
    ax.set_title(f"{'策略' if args.model else '纯追踪'} seed={args.seed} "
                 f"结果={info['termination_reason']} 用时={info['time']:.1f}s "
                 f"撞锥={info['cone_hits']}")
    ax.legend(loc="upper right", fontsize=8)

    # 右图:跑这一圈建出来的占据栅格(黑=占据,白=空闲,灰=未知)
    grid = env.map.grid()
    shade = np.full(grid.shape, 0.6)
    shade[grid == FREE] = 1.0
    shade[grid == OCCUPIED] = 0.0
    half = env.map.n * env.map.res / 2
    ax2.imshow(shade.T, cmap="gray", vmin=0, vmax=1, origin="lower",
               extent=[-half, half, -half, half])
    ax2.plot(*path.T, color="#1a7f4b", lw=1.2)
    ax2.set_aspect("equal")
    ax2.set_title(f"占据栅格 10cm:锥桶覆盖率={info['cone_coverage']:.2f} "
                  f"空闲面积={info['mapped_free_area']:.0f}m²")
    fig.savefig(args.out, dpi=120, bbox_inches="tight")
    print(args.out, info)


if __name__ == "__main__":
    main()
