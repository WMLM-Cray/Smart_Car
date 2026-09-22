# -*- coding: utf-8 -*-
"""二维仿真——实车约束敏感性分析 (2026-09-21)。

不改 sim/params.py、sim/vehicle.py(原模型保持原样)。独立运行器，对"车辆几何参数固定、
只有两个因素变化"做同条件对照：速度上限 + 执行器死区。居间量(轴距/外轮廓/转角/传感器)
都从 sim/params.py 单源取，二维三维已对齐(见 PARAM_COMPARISON)。

三个变体(命名=敏感性变体, 不是已标定的实车模型):
  A 原模型              = CarParams() 默认, 速度上限 1.5/0.5 m/s, 无执行器死区
  B 0.10m/s限幅变体     = 速度上限限到 0.10/0.05 m/s(安全层 max_linear 保守[配置]), 无死区
  C 假设0.48m/s死区变体 = B 之上再加速度域对称死区 v_dz=0.48 m/s(假设)

死区模型形式(写清施加位置 / 输入单位 / 模型形式, 不据此建议提高限速):
  施加位置 = 速度命令 v_cmd, 在 min(v_max) 限幅之后、车辆动力学 car.step() 之前;
  输入单位 = m/s(速度域);
  模型形式 = 对称死区 |v_cmd| < v_dz -> 0(v_dz=0.48 m/s);
  定性     = "485mm/s≈1588µs"这一死区口径在速度域的折算(口径①[待裁决]), 是敏感性假设,
             不是已标定的实车最低可控地面速度; 真实电调死区在 PWM 域(µs), 仅在
             线速度↔PWM 映射逐点成立时才能折算为速度域。

三场景 (几何见 SCENARIOS_HANDOFF):
  S1 直行单点        (0,0,0)->(2,0)
  S2 含转弯两点      (0,0,0)->(1.5,0)->(1.5,1.5)   [两点分别判定]
  S3 给定绕障参考路径的跟踪与净空验证 (0,0,0)->(2,0), 障碍(1,0,r0.14),
     参考路径=上半弧 R=0.6(显式给定, 非自主避障规划、非 Nav2 规划)

统一成功条件(逐维独立判定, 不放宽凑通过):
  位置  pos_err <= 0.10 m  (终点后轴中心到终点欧氏距离)
  朝向  head_err <= 0.35 rad (车头朝向 vs 参考路径终段方向)
  停车  v_final <= 0.02 m/s (到达后刹停)
  时限  t < 120 s 且已到达
  净空(仅S3) 车体外轮廓到障碍外轮廓最近距离 >= 0(不入   侵), 碰撞=外轮廓相交

导出几何量来源: 前悬 = length - rear_overhang = 0.56 - 0.13 = 0.43 m;
最小转弯半径 R_min = wheelbase / tan(max_steer) = 0.36 / tan(0.55) = 0.587 m。

[方法修正记录] 首版 S3 用势场式反应控制器(斥力+切向), A/B 均撞障碍
(min_clearance=-0.14, collided=True), 判定"高速+紧障碍下反应式绕不开"不可靠;
改成本文件"给定参考路径 Pure Pursuit 跟踪"(复用 sim/preview.py 纯追踪思想, 折线订阅版),
A/B 用同一参考路径/控制器/成功判据比较跟踪与净空。

运行: python tools/sim2real/run_compare.py [--out-dir docs/sim2real/out]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from sim.params import CarParams                      # noqa: E402
from sim.vehicle import Car                           # noqa: E402

DEFAULT_OUT = REPO / "docs/sim2real/out"

DT = 0.01
BUDGET = 120.0
LOOKAHEAD = 0.35          # m  Pure Pursuit 前视距离
K_SPEED = 0.8             # 接近终点减速比例 v = K_SPEED * d_remain
ARRIVE_TOL = 0.05         # m  沿路径投影到终点的到达触发
POS_TOL = 0.10            # m  统一位置误差阈值
HEAD_TOL = 0.35           # rad 统一朝向误差阈值
V_STOP = 0.02             # m/s 统一停车阈值

# ---- 参数变体 ----
VARIANTS = [
    dict(key="orig",     label_zh="原模型",                label_en="A orig 1.5",
         v_max=1.5, v_rev=0.5, v_dz=0.0),
    dict(key="limit",    label_zh="0.10m/s限幅变体",       label_en="B limit 0.10",
         v_max=0.10, v_rev=0.05, v_dz=0.0),
    dict(key="limit+dz", label_zh="假设0.48m/s死区变体",    label_en="C limit+dz 0.48",
         v_max=0.10, v_rev=0.05, v_dz=0.48),
]
COLORS = {"orig": "#1a7f4b", "limit": "#c44e52", "limit+dz": "#555"}


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def rect_point_dist(car: Car, pt: tuple[float, float]) -> float:
    p = car.p
    c, s = math.cos(car.yaw), math.sin(car.yaw)
    rx, ry = pt[0] - car.x, pt[1] - car.y
    lx = rx * c + ry * s
    ly = -rx * s + ry * c
    x_lo, x_hi = -p.rear_overhang, p.length - p.rear_overhang
    dx = max(x_lo - lx, 0.0, lx - x_hi)
    dy = max(abs(ly) - p.width / 2, 0.0)
    return math.hypot(dx, dy)


def apply_deadzone(v: float, v_dz: float) -> float:
    if v > 1e-9 and v < v_dz:
        return 0.0
    if v < -1e-9 and v > -v_dz:
        return 0.0
    return v


def make_path(pts):
    pts = np.asarray(pts, dtype=float)
    seg = np.hypot(*np.diff(pts, axis=0).T)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    return pts, cum


def project_s(pts, cum, x, y):
    """点到折线的正交投影 arc-length(连续, 非最近点阶跃)。"""
    best_s, best_d2 = 0.0, float("inf")
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        abx, aby = bx - ax, by - ay
        ab2 = abx * abx + aby * aby
        if ab2 < 1e-12:
            t = 0.0
        else:
            t = ((x - ax) * abx + (y - ay) * aby) / ab2
            t = max(0.0, min(1.0, t))
        px, py = ax + t * abx, ay + t * aby
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_s = cum[i] + t * (cum[i + 1] - cum[i])
    return best_s


def arc(cx, cy, r, a0, a1, n=60):
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / n),
             cy + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]


def build_detour():
    """S3: 上半弧绕障碍 (1,0,r0.14), 弧心 (1,0) 半径 0.6(> R_min 0.587 可跟踪)。

    注: 该半圆弧端点切线分别朝 +y(进) / -y(出), 与终点直行段(+x)存在 90° 切线不连续,
    是"给定参考路径"的几何特性; 若需终点朝向对齐, 须延长终点直行段或加调头段(本轮不含)。
    """
    return ([(0.0, 0.0), (0.4, 0.0)]
            + arc(1.0, 0.0, 0.6, math.pi, 0.0, n=40)
            + [(1.6, 0.0), (2.0, 0.0)])


SCENARIOS = {
    "s1": dict(start=(0.0, 0.0, 0.0), path=[(0.0, 0.0), (2.0, 0.0)],
               goals=[(2.0, 0.0)], ob=None, title_en="S1 straight single-point"),
    "s2": dict(start=(0.0, 0.0, 0.0), path=[(0.0, 0.0), (1.5, 0.0), (1.5, 1.5)],
               goals=[(1.5, 0.0), (1.5, 1.5)], ob=None,
               title_en="S2 two-point with 90deg turn"),
    "s3": dict(start=(0.0, 0.0, 0.0), path=build_detour(), goals=[(2.0, 0.0)],
               ob=(1.0, 0.0, 0.14), title_en="S3 reference-path tracking & clearance"),
}


def run(variant: dict, sc: dict) -> dict:
    p = CarParams()
    car = Car(p)
    car.reset(*sc["start"])
    pts, cum = make_path(sc["path"])
    s_end = cum[-1]
    ob = sc["ob"]

    goals = sc["goals"]
    goal_arcs = []
    for (gx, gy) in goals:
        d = np.hypot(pts[:, 0] - gx, pts[:, 1] - gy)
        goal_arcs.append(cum[int(d.argmin())])

    traj = [(car.x, car.y)]
    clears, collided = [], False
    way = []
    gi = 0
    reached = False
    stopping = False
    t = 0.0
    while t < BUDGET - 1e-9:
        s = project_s(pts, cum, car.x, car.y)
        d_remain = s_end - s
        # 中间任务点判定(逐个, 排除终点)
        while gi < len(goals) - 1 and s >= goal_arcs[gi]:
            gx, gy = goals[gi]
            perr = math.hypot(gx - car.x, gy - car.y)
            way.append(dict(label=f"wp{gi + 1}", time=round(t, 2),
                            pos_err=round(perr, 4), pass_pos=perr <= POS_TOL))
            gi += 1
        if d_remain <= ARRIVE_TOL or stopping:
            stopping = True
            if car.v < V_STOP and d_remain <= ARRIVE_TOL:
                reached = True
                break
            v = 0.0
            steer = 0.0
        else:
            target = min(s + LOOKAHEAD, s_end)
            ti = min(int(np.searchsorted(cum, target)), len(pts) - 1)
            tx, ty = pts[ti]
            alpha = math.atan2(ty - car.y, tx - car.x) - car.yaw
            steer = math.atan2(2 * p.wheelbase * math.sin(alpha), LOOKAHEAD)
            steer = max(-p.max_steer, min(p.max_steer, steer))
            v = K_SPEED * d_remain
            v = max(0.0, min(variant["v_max"], v))
            v = apply_deadzone(v, variant["v_dz"])
        car.step(v, steer, DT)
        t += DT
        traj.append((car.x, car.y))
        if ob is not None:
            cl = rect_point_dist(car, (ob[0], ob[1])) - ob[2]
            clears.append(cl)
            if cl < 0.0:
                collided = True

    gx, gy = goals[-1]
    pos_err = float(math.hypot(gx - car.x, gy - car.y))
    wp = sc["path"]
    px, py = wp[-2]
    goal_yaw = math.atan2(gy - py, gx - px)
    head_err = float(abs(wrap(float(car.yaw) - goal_yaw)))
    v_final = float(car.v)
    return dict(
        reached=bool(reached), time=round(t, 2), timed_out=bool(not reached and t >= BUDGET - 1e-9),
        pos_err=round(pos_err, 4), head_err=round(head_err, 4), v_final=round(v_final, 4),
        pass_pos=bool(pos_err <= POS_TOL),
        # 朝向/停车判据仅在"已到达"时才有意义; 未达(如死区抑制)标 None=不适用
        pass_head=(bool(head_err <= HEAD_TOL) if reached else None),
        pass_speed=(bool(v_final <= V_STOP) if reached else None),
        min_clearance=(round(float(min(clears)), 4) if clears else None),
        collided=bool(collided), waypoints=way, traj=np.array(traj, dtype=float),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = ap.parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    metrics = {}
    for sid, sc in SCENARIOS.items():
        for v in VARIANTS:
            r = run(v, sc)
            metrics.setdefault(sid, {})[v["key"]] = {
                "label": v["label_zh"], "reached": r["reached"], "time": r["time"],
                "timed_out": r["timed_out"], "pos_err": r["pos_err"],
                "head_err": r["head_err"], "v_final": r["v_final"],
                "pass_pos": r["pass_pos"], "pass_head": r["pass_head"],
                "pass_speed": r["pass_speed"], "min_clearance": r["min_clearance"],
                "collided": r["collided"], "waypoints": r["waypoints"],
                "traj": r["traj"].tolist(),
            }
    (out / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=1),
                                      encoding="utf-8")
    print("metrics ->", out / "metrics.json")
    plot(metrics, out)


def plot(metrics, out: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon

    for sid, sc in SCENARIOS.items():
        fig, ax = plt.subplots(figsize=(6.4, 5.0))
        p = CarParams()
        for v in VARIANTS:
            key = v["key"]
            m = metrics[sid][key]
            traj = np.array(m["traj"])
            if m["reached"]:
                lab = f"{v['label_en']}  t={m['time']:.1f}s pos={m['pos_err']:.2f}m"
            else:
                lab = f"{v['label_en']}  NOT reached (pos={m['pos_err']:.2f}m)"
            if m["min_clearance"] is not None:
                lab += f" clr={m['min_clearance']:.2f}m"
            ax.plot(traj[:, 0], traj[:, 1], lw=1.6, color=COLORS[key], label=lab)
        rp = np.array(sc["path"])
        ax.plot(rp[:, 0], rp[:, 1], ls=":", lw=1.2, color="#bbb", label="reference path")
        ax.scatter([sc["start"][0]], [sc["start"][1]], marker="s", s=60, c="k", zorder=5, label="start")
        for (gx, gy) in sc["goals"]:
            ax.scatter([gx], [gy], marker="X", s=90, c="#ff7f0e", zorder=5)
        ax.scatter([], [], marker="X", s=90, c="#ff7f0e", zorder=5, label="goal")
        if sc["ob"]:
            ox, oy, or_ = sc["ob"]
            ax.add_patch(plt.Circle((ox, oy), or_, color="#888", alpha=0.55, zorder=4,
                                    label=f"obstacle r={or_}"))
        for v in VARIANTS:
            traj = np.array(metrics[sid][v["key"]]["traj"])
            if len(traj) < 2:
                continue
            x, y = traj[-1]
            yaw = math.atan2(traj[-1][1] - traj[-2][1], traj[-1][0] - traj[-2][0])
            c, s = math.cos(yaw), math.sin(yaw)
            lx = np.array([-p.rear_overhang, p.length - p.rear_overhang,
                           p.length - p.rear_overhang, -p.rear_overhang])
            ly = np.array([-p.width / 2, -p.width / 2, p.width / 2, p.width / 2])
            poly = np.stack([x + lx * c - ly * s, y + lx * s + ly * c], axis=1)
            ax.add_patch(MplPolygon(poly, closed=True, fill=False, ec=COLORS[v["key"]],
                                    lw=1.0, alpha=0.8))
        ax.set_aspect("equal")
        ax.set_title(sc["title_en"] + "   (A orig vs B limit vs C limit+deadzone)")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.legend(fontsize=7, loc="best")
        fig.tight_layout()
        fig.savefig(out / f"scenario_{sid}.png", dpi=140, bbox_inches="tight")
        plt.close(fig)
        print("png ->", out / f"scenario_{sid}.png")


if __name__ == "__main__":
    main()