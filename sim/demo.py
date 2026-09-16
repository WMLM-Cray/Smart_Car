"""四工况集成演示(2026-09-16 任务书第 5 项):直行/定舵圆周/倒车/撞锥。

    python3 -m sim.demo [--json out.json]

前三个工况在**纯车辆模型**(sim.vehicle.Car)上做,不进赛道环境——对账对象
是运动学自行车模型本身,不掺入出界/撞锥终止;撞锥工况用冻结 A 赛道环境
(ConeTrackEnv(track=load_trackA()))验证碰撞检测链路。

圆周对账:δ=0.3rad、v→0.6m/s 稳态,理论 R = L/tanδ(阿克曼运动学),
      ω = v·tanδ/L;取加速收敛后(t>5s)的轨迹做最小二乘圆拟合,
      半径与航向角速度双对账,阈值 2%。
来源声明:本演示全部为运动学模型/规则判定的确定性输出,
      不含 Pure Pursuit,也不含 RL(未训练)。
"""
from __future__ import annotations

import argparse
import json
import math

import numpy as np

from .params import CarParams
from .vehicle import Car


def fit_circle(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    """最小二乘圆拟合(Kasa 代数法):返回 (cx, cy, R)。"""
    a = np.stack([x, y, np.ones_like(x)], axis=1)
    b = x ** 2 + y ** 2
    (aa, bb, cc), *_ = np.linalg.lstsq(a, b, rcond=None)
    cx, cy = aa / 2, bb / 2
    return float(cx), float(cy), float(math.sqrt(cc + cx ** 2 + cy ** 2))


def run_straight(car: Car, v: float, seconds: float, dt: float = 0.01):
    x0, y0 = car.x, car.y
    t = 0.0
    while t < seconds:
        car.step(v, 0.0, dt)
        t += dt
    return dict(dx=car.x - x0, dy=car.y - y0, dyaw=car.yaw)


def run_circle(car: Car, v: float, steer: float, seconds: float, dt: float = 0.01):
    xs, ys, yaws, t = [], [], [], 0.0
    while t < seconds:
        car.step(v, steer, dt)
        t += dt
        if t > 5.0:                       # 避开起步加速段,只取稳态
            xs.append(car.x); ys.append(car.y); yaws.append(car.yaw)
    cx, cy, r_fit = fit_circle(np.asarray(xs), np.asarray(ys))
    # 航向角速度:相邻采样 dyaw(已 wrap 到 ±π,采样密不跨分支)
    yaws = np.unwrap(np.asarray(yaws))
    omega = float((yaws[-1] - yaws[0]) / ((len(yaws) - 1) * dt))
    return dict(r_fit=r_fit, cx=cx, cy=cy, omega=omega, n_pts=len(xs))


def demo_collision() -> dict:
    from .env import ConeTrackEnv
    from .trackA import load_trackA
    env = ConeTrackEnv(track=load_trackA())
    env.reset(seed=0)
    env.car.reset(11.0, 0.75, math.pi / 2)      # 底道中线原地朝 +y(内圈红锥方向)
    env.last_s = env.track.project((11.0, 0.75))[0]
    env._sense()
    for _ in range(200):                        # 200×0.05=10s 上限
        _, _, term, trunc, info = env.step([0.4, 0.0])
        if term or trunc:
            break
    return dict(reason=info["termination_reason"], cone_hits=info["cone_hits"],
                time=info["time"], speed=info["speed"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default="", help="结果另存 JSON(可选)")
    args = ap.parse_args()
    p = CarParams()
    rows, ok_all = [], True

    # ① 直行:v=0.6, δ=0, 3s → 纵向位移≈稳态积分值(一阶响应),横向不漂
    car = Car(p); car.reset(0.0, 0.0, 0.0)
    r = run_straight(car, v=0.6, seconds=3.0)
    ok = abs(r["dy"]) < 0.01 and abs(r["dyaw"]) < 1e-6 and 1.5 < r["dx"] < 1.8
    ok_all &= ok
    rows.append(("直行 v=0.6 δ=0 3s", ok,
                 f"Δx={r['dx']:+.3f}m Δy={r['dy']:+.5f}m Δyaw={r['dyaw']:+.1e}rad"))

    # ② 定舵圆周:v=0.6, δ=0.3 → R=L/tanδ, ω=v·tanδ/L
    r_theory = p.wheelbase / math.tan(0.3)
    w_theory = 0.6 * math.tan(0.3) / p.wheelbase
    car = Car(p); car.reset(0.0, 0.0, 0.0)
    c = run_circle(car, v=0.6, steer=0.3, seconds=30.0)
    err_r = abs(c["r_fit"] - r_theory) / r_theory
    err_w = abs(c["omega"] - w_theory) / w_theory
    ok = err_r < 0.02 and err_w < 0.02
    ok_all &= ok
    rows.append(("定舵圆周 v=0.6 δ=0.3", ok,
                 f"R拟合={c['r_fit']:.4f} vs 理论 {r_theory:.4f}({err_r*100:.2f}%) "
                 f"ω={c['omega']:.4f} vs {w_theory:.4f}({err_w*100:.2f}%) L={p.wheelbase}"))

    # ③ 倒车:v=-0.3, δ=0, 3s → 沿 -x,横向不漂
    car = Car(p); car.reset(0.0, 0.0, 0.0)
    r = run_straight(car, v=-0.3, seconds=3.0)
    ok = abs(r["dy"]) < 0.01 and r["dx"] < -0.7
    ok_all &= ok
    rows.append(("倒车 v=-0.3 δ=0 3s", ok,
                 f"Δx={r['dx']:+.3f}m Δy={r['dy']:+.5f}m(一阶加速,3s 位移<0.9)"))

    # ④ 撞锥:trackA 底道朝内圈红锥直开 → cone 命中即终止(规则撞锥+5s 在奖励侧)
    col = demo_collision()
    ok = col["reason"] == "cone" and col["cone_hits"] >= 1
    ok_all &= ok
    rows.append(("撞锥(trackA 朝红锥直开)", ok,
                 f"reason={col['reason']} hits={col['cone_hits']} t={col['time']:.2f}s"))

    print(f"{'工况':<22}{'结果':<6}测量")
    print("-" * 78)
    for name, ok, detail in rows:
        print(f"{name:<22}{'PASS' if ok else 'FAIL':<6}{detail}")
    print("-" * 78)
    print(f"demo_overall={'PASS' if ok_all else 'FAIL'}(轴距 L={p.wheelbase}m,对账 R=L/tanδ)")

    if args.json:
        with open(args.json, "w") as f:
            json.dump({"rows": [dict(name=n, ok=bool(o), detail=d) for n, o, d in rows],
                       "overall": "PASS" if ok_all else "FAIL",
                       "wheelbase": p.wheelbase,
                       "r_theory": r_theory, "omega_theory": w_theory,
                       "circle_fit": c, "collision": col}, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
