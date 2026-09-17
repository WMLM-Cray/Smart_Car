"""实时二维交互驾驶入口(2026-09-17 任务书五-B:可实时操作/可复现/可信 RL 接口)。

    python3 -m sim.interactive --track trackA --mode manual        # 键盘驾驶(需 GUI)
    python3 -m sim.interactive --track random --seed 0 --mode pp   # 纯追踪自动跑
    python3 -m sim.interactive --selftest                          # 无头自检(无 GUI 环境)
    python3 -m sim.interactive --demo-gif out.gif                  # 内置演示键序渲染 GIF

要点(与任务书逐条对应):
  - 参考线(中心线)只画在画布上做调试显示,**绝不进策略观测**(观测仍只含
    48 扇区雷达 + encoder/IMU + 动作回显 + 剩余时间,见 env._observation);
  - 键盘是阿克曼语义:↑↓ 加减**车速指令**(松开保持,巡航语义),←→ 打**前轮
    转角**(松开立即回中)。ω 由自行车模型 ω=v·tanδ/L 决定——倒车时同 δ 的
    ω 自动反号,不存在原地转向(v=0 则 ω≡0);
  - 失焦/关窗/重置都会清除控制状态;暂停=物理不推进(固定 control 步长不变);
  - GUI 刷新(matplotlib timer)与物理固定步(control_dt=0.05s)解耦:每个
    timer tick 恰好推 1 个 control step(默认 1× 实时),渲染掉帧不会改变
    物理步长,也不会追帧跳步;
  - 本轮不接实体手柄(任务书明确);演示 GIF 由 --demo-gif 用同一套键序
    状态机无头渲染,不是用静态图冒充实时画面。
"""
from __future__ import annotations

import argparse
import math

import numpy as np

from .env import ConeTrackEnv


class ManualDriver:
    """键盘 → 动作域 [-1,1]² 的映射,press/release 共用一条路径(键序回放同路)。

    ↑/↓: a0 ±0.1(车速指令,松开保持)   ←/→: a1 ±0.1(转角,松开回中)
    X:   a0 清零(指令级急停;物理停车还要过一阶响应 τ=0.35s)
    """

    STEP = 0.1

    def __init__(self):
        self.clear()

    def clear(self):
        self.a0 = 0.0
        self.a1 = 0.0

    def press(self, key: str):
        if key == "up":
            self.a0 = round(min(1.0, self.a0 + self.STEP), 10)
        elif key == "down":
            self.a0 = round(max(-1.0, self.a0 - self.STEP), 10)
        elif key == "left":
            self.a1 = round(min(1.0, self.a1 + self.STEP), 10)
        elif key == "right":
            self.a1 = round(max(-1.0, self.a1 - self.STEP), 10)
        elif key == "x":
            self.a0 = 0.0

    def release(self, key: str):
        if key in ("left", "right"):       # 转向键松开=回中;速度键松开=保持
            self.a1 = 0.0

    def action(self) -> np.ndarray:
        return np.array([self.a0, self.a1], np.float32)


def rect_corners(x, y, yaw, length, width, rear_overhang):
    c, s = math.cos(yaw), math.sin(yaw)
    lx = np.array([-rear_overhang, length - rear_overhang, length - rear_overhang, -rear_overhang])
    ly = np.array([-width / 2, -width / 2, width / 2, width / 2])
    return np.stack([x + lx * c - ly * s, y + lx * s + ly * c], axis=1)


# 内置演示键序(供 --demo-gif 无头渲染;与实时入口同一状态机,非静态图冒充):
# 起步直行 → 左转绕行 → 松方向回中 → 减速停 → 倒车(演示 ω 反号)→ 停。
DEMO_SCRIPT = [
    ("press", "up"), ("wait", 1.0), ("press", "left"), ("wait", 1.2),
    ("release", "left"), ("wait", 0.8), ("press", "down"), ("wait", 0.6),
    ("press", "down"), ("wait", 1.6), ("press", "x"), ("wait", 0.5),
]


def make_env(track: str, seed: int, budget: float) -> tuple[ConeTrackEnv, dict]:
    if track == "trackA":
        from .trackA import load_trackA
        env = ConeTrackEnv(track=load_trackA(), budget=budget)
    else:
        env = ConeTrackEnv(budget=budget)
    _, info = env.reset(seed=seed)          # random 模式同 seed 可复现;trackA 只定噪声
    return env, info


def status_text(env: ConeTrackEnv, info: dict, mode: str, paused: bool, drv: ManualDriver) -> str:
    state = info.get("termination_reason") or ("PAUSED" if paused else "running")
    return (f"t={info['time']:5.1f}s v={env.car.v:+.2f}m/s steer={env.car.steer:+.2f}rad "
            f"w={env.car.yaw_rate:+.2f}rad/s | cmd a0={drv.a0:+.1f} a1={drv.a1:+.1f} "
            f"| {mode} {state} hits={info['cone_hits']} prog={info['progress']:.1f}m")


# ---------- 无头路径:selftest / demo-gif(不依赖任何 GUI 后端) ----------
def run_headless(args) -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    env, info = make_env(args.track, args.seed, args.budget)
    drv = ManualDriver()
    mode = "manual(script)" if args.demo_gif else "selftest"
    path, frames = [], []

    def render_frame():
        ang = env.lidar.angles + env.car.yaw
        ok = np.isfinite(env.ranges)
        lx = env.car.x + env.cp.lidar_x * math.cos(env.car.yaw) + env.ranges[ok] * np.cos(ang[ok])
        ly = env.car.y + env.cp.lidar_x * math.sin(env.car.yaw) + env.ranges[ok] * np.sin(ang[ok])
        frames.append((env.time, (env.car.x, env.car.y, env.car.yaw), len(path),
                       np.stack([lx, ly], axis=1)))

    script = list(DEMO_SCRIPT) if args.demo_gif else []
    wait_left = 0.0
    k = 0
    while True:
        if script:
            ev = script[0]
            if wait_left <= 1e-9 and ev[0] != "wait":
                script.pop(0)
                if ev[0] == "press":
                    drv.press(ev[1])
                elif ev[0] == "release":
                    drv.release(ev[1])
            else:
                wait_left -= env.control_dt
            if script and script[0][0] == "wait":
                wait_left = script.pop(0)[1]
        _, _, term, trunc, info = env.step(drv.action())
        path.append((env.car.x, env.car.y))
        k += 1
        if (args.demo_gif and k % max(1, round(0.25 / env.control_dt)) == 0) or term or trunc:
            render_frame()
        if term or trunc or (not script and k >= (160 if args.demo_gif else 40)):
            break        # demo:脚本 5.7s(≈114 步)+2s 尾段;selftest:固定 40 步

    if args.demo_gif:
        path_arr = np.asarray(path)
        fig, ax = plt.subplots(figsize=(6.4, 6.4), dpi=90)
        ax.plot(*env.track.centerline.T, color="#ccc", lw=1, label="centerline (debug only)")
        ax.scatter(*env.track.left.T, c="#d33", s=10)
        ax.scatter(*env.track.right.T, c="#36c", s=10)
        (trail,) = ax.plot([], [], color="#1a7f4b", lw=1.5)
        (car_poly,) = ax.fill([], [], color="#1a7f4b", alpha=0.6, edgecolor="k", lw=0.8)
        scan = ax.scatter([], [], s=1.5, c="k")
        title = ax.set_title("")
        allx = np.concatenate([env.track.centerline[:, 0], path_arr[:, 0]])
        ally = np.concatenate([env.track.centerline[:, 1], path_arr[:, 1]])
        ax.set_xlim(allx.min() - 0.6, allx.max() + 0.6)
        ax.set_ylim(ally.min() - 0.6, ally.max() + 0.6)
        ax.set_aspect("equal")

        def draw(i):
            t, (x, y, yaw), n_keep, pts = frames[i]
            trail.set_data(path_arr[:n_keep, 0], path_arr[:n_keep, 1])
            car_poly.set_xy(rect_corners(x, y, yaw, env.cp.length, env.cp.width,
                                         env.cp.rear_overhang))
            off = pts - np.array([x, y])
            scan.set_offsets(pts[np.hypot(*off.T) > 0.45])
            title.set_text(f"{args.track} keyboard (scripted)  t={t:.1f}s  "
                           f"hits={info['cone_hits']}  {info['termination_reason'] or ''}")
            return trail, car_poly, scan, title

        anim = FuncAnimation(fig, draw, frames=len(frames), blit=False)
        anim.save(args.demo_gif, writer=PillowWriter(fps=4))
        plt.close(fig)
        print(f"{args.demo_gif} 已写出({len(frames)} 帧,{mode})")
    print("selftest:", status_text(env, info, mode, False, drv))
    return 0


# ---------- 实时 GUI 路径 ----------
def run_gui(args) -> int:
    # 注意顺序:preview.py 模块顶部会 matplotlib.use("Agg"),必须先 import 它,
    # 再强制切回 GUI 后端,否则窗口会被偷换成 Agg(无法交互)。
    from .preview import pure_pursuit
    import matplotlib
    matplotlib.use(args.backend, force=True)
    import matplotlib as mpl
    import matplotlib.pyplot as plt
    # 清空与驾驶键冲突的默认快捷键(left/right=导航、r=home、q=quit 等)
    for km in ("keymap.home", "keymap.back", "keymap.forward",
               "keymap.save", "keymap.quit", "keymap.fullscreen", "keymap.grid"):
        mpl.rcParams[km] = []

    env, info = make_env(args.track, args.seed, args.budget)
    drv = ManualDriver()
    mode = args.mode
    paused, step_once, closed = False, False, False
    path: list[tuple[float, float]] = []

    fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=100)
    ax.plot(*env.track.centerline.T, color="#ccc", lw=1, label="centerline (debug only)")
    ax.scatter(*env.track.left.T, c="#d33", s=12, label="left cones (red)")
    ax.scatter(*env.track.right.T, c="#36c", s=12, label="right cones (blue)")
    (trail,) = ax.plot([], [], color="#1a7f4b", lw=1.5, label="path")
    (car_poly,) = ax.fill([], [], color="#1a7f4b", alpha=0.6, edgecolor="k", lw=0.8)
    scan = ax.scatter([], [], s=1.5, c="k", label="lidar (last frame)")
    title = ax.set_title("")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)

    def refit():
        pts = np.asarray(path) if len(path) else np.zeros((1, 2))
        allx = np.concatenate([env.track.centerline[:, 0], pts[:, 0]])
        ally = np.concatenate([env.track.centerline[:, 1], pts[:, 1]])
        ax.set_xlim(allx.min() - 0.6, allx.max() + 0.6)
        ax.set_ylim(ally.min() - 0.6, ally.max() + 0.6)

    def draw():
        car_poly.set_xy(rect_corners(env.car.x, env.car.y, env.car.yaw, env.cp.length,
                                     env.cp.width, env.cp.rear_overhang))
        trail.set_data(*(np.asarray(path).T if len(path) else np.array([[], []])))
        ang = env.lidar.angles + env.car.yaw
        ok = np.isfinite(env.ranges)
        lx = env.car.x + env.cp.lidar_x * math.cos(env.car.yaw) + env.ranges[ok] * np.cos(ang[ok])
        ly = env.car.y + env.cp.lidar_x * math.sin(env.car.yaw) + env.ranges[ok] * np.sin(ang[ok])
        keep = np.hypot(lx - env.car.x, ly - env.car.y) > 0.45
        scan.set_offsets(np.stack([lx[keep], ly[keep]], axis=1))
        title.set_text(status_text(env, info, mode, paused, drv))
        fig.canvas.draw_idle()

    def on_key(event):
        nonlocal paused, step_once
        k = event.key
        if k in ("q", "escape"):
            fig.canvas.stop_event_loop()
            plt.close(fig)
            return
        if k == " ":
            paused = not paused
        elif k == ".":
            paused, step_once = True, True
        elif k == "r":
            nonlocal_seed_reset()
        else:
            drv.press(k)

    def nonlocal_seed_reset():
        nonlocal path, info
        _, info = env.reset(seed=args.seed)   # 同 seed 重来(可复现);控制状态一并清
        drv.clear()
        path = []
        refit()

    def on_release(event):
        drv.release(event.key)

    def on_close(_event):
        nonlocal closed
        closed = True
        drv.clear()                         # 失焦/关窗清控制状态

    def tick():
        nonlocal step_once, info
        if closed:
            return
        if paused and not step_once:
            return
        if step_once:
            step_once = False
        action = pure_pursuit(env) if mode == "pp" else drv.action()
        _, _, term, trunc, info = env.step(action)
        path.append((env.car.x, env.car.y))
        draw()
        if term or trunc:                   # 局终:物理停,窗口保留看轨迹
            title.set_text(status_text(env, info, mode, paused, drv) + "  [ended, R=restart]")

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("key_release_event", on_release)
    fig.canvas.mpl_connect("close_event", on_close)
    # matplotlib 版本兼容:3.8+ 用 fig.new_timer,旧版在 canvas 上
    new_timer = getattr(fig, "new_timer", None) or fig.canvas.new_timer
    timer = new_timer(interval=round(env.control_dt * 1000))   # 1× 实时:50ms/步
    timer.add_callback(tick)

    if args.shot:
        # 窗口级自动凭据:真 TkAgg 窗口里注入合成 KeyEvent(走 on_key 同一条
        # matplotlib 事件分发路径),起步+转向驾驶 ~3s 后截画布图退出。
        from matplotlib.backend_bases import KeyEvent

        def send(channel: str, key: str):
            fig.canvas.callbacks.process(channel, KeyEvent(channel, fig.canvas, key))

        plt.pause(0.5)                      # 先让窗口完成首帧绘制
        send("key_press_event", "up")
        for _ in range(30):                 # 30 步 = 1.5s 直行
            _, _, term, trunc, info = env.step(drv.action())
            path.append((env.car.x, env.car.y))
            if term or trunc:
                break
        send("key_press_event", "left")
        for _ in range(30):                 # 1.5s 左转
            _, _, term, trunc, info = env.step(drv.action())
            path.append((env.car.x, env.car.y))
            if term or trunc:
                break
        send("key_release_event", "left")   # 松键回中,再推 1 步
        _, _, term, trunc, info = env.step(drv.action())
        path.append((env.car.x, env.car.y))
        refit()
        draw()
        fig.savefig(args.shot, dpi=110)
        print("shot:", status_text(env, info, mode, paused, drv))
        print(f"{args.shot} 已写出(窗口级 KeyEvent 注入驾驶)")
        drv.clear()
        plt.close(fig)
        return 0

    refit()
    draw()
    timer.start()
    print("键位: ↑↓=车速±(松开保持) ←→=转向(松开回中) X=速度清零 空格=暂停 "
          ". =单步 R=重置 Q/Esc=退出 | PP 模式无键盘驾驶")
    plt.show()
    drv.clear()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", choices=["random", "trackA"], default="trackA")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--mode", choices=["manual", "pp"], default="manual")
    ap.add_argument("--budget", type=float, default=120.0)
    ap.add_argument("--backend", default="TkAgg")
    ap.add_argument("--selftest", action="store_true", help="无头自检(无 GUI)")
    ap.add_argument("--demo-gif", default="", metavar="OUT.gif",
                    help="内置演示键序无头渲染 GIF")
    ap.add_argument("--shot", default="", metavar="OUT.png",
                    help="真窗口 KeyEvent 注入驾驶后截画布图(自动凭据)")
    args = ap.parse_args()
    if args.selftest or args.demo_gif:
        raise SystemExit(run_headless(args))
    raise SystemExit(run_gui(args))


if __name__ == "__main__":
    main()
