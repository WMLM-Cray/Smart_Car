"""与队友 Gazebo 框架(hwcart_sim)的互通层。

两边的分工:本仿真跑得快,用来训练;hwcart_sim 是 Gazebo,用来做贴近实车的验证。
要让"一套策略两边都能跑",只需要口径对齐,不需要合并代码:

1. 控制口径 —— 他们的 `/cmd_vel` 是**物理单位** Twist(linear.x=m/s, angular.z=rad/s),
   本仿真的动作是(车速, 前轮转角),两者用同一个自行车模型 ω = v·tanδ/L 互换。
   实车 `/car_cmd_vel` 收的是 PWM 刻度,那一层在 pwm.py,不要和这里混。
2. 赛道口径 —— export_track() 把本仿真随机生成的赛道写成 track_generator 认的
   cones.csv + gaps.json,他们 Gazebo 侧就能生成同一条赛道的 world。
3. 观测口径 —— 他们 `/scan` 是 2000 点,实车/本仿真是 1667 点;
   观测用的 48 扇区压缩见 env.sectorize(),对任意点数都给同一套 48 维,不用改策略。
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np

from .params import CarParams
from .track import Track

# track_generator/generate.py 里挡板候选位的判据:0.8m 挡板要塞得下同色相邻两锥桶之间
GAP_MIN, GAP_MAX = 1.08, 1.5


# ---- 控制口径 ----
def to_cmd_vel(v: float, steer: float, p: CarParams = CarParams()) -> tuple[float, float]:
    """(车速 m/s, 前轮转角 rad) -> 队友 /cmd_vel 的 (linear.x, angular.z)。"""
    return float(v), float(v * math.tan(steer) / p.wheelbase)


def from_cmd_vel(linear_x: float, angular_z: float, p: CarParams = CarParams()) -> tuple[float, float]:
    """/cmd_vel -> (车速, 前轮转角)。原地转向在阿克曼车上不存在,v=0 时转角取 0。"""
    if abs(linear_x) < 1e-6:
        return float(linear_x), 0.0
    steer = math.atan(angular_z * p.wheelbase / linear_x)
    return float(linear_x), max(-p.max_steer, min(p.max_steer, steer))


# ---- 赛道口径 ----
def _align_to_start(track: Track) -> tuple[np.ndarray, np.ndarray]:
    """把赛道搬到"起点在原点、车头朝 +x",对齐他们 launch 里车的出生点。"""
    start = track.centerline[0]
    heading = track.project(start)[2]
    c, s = math.cos(-heading), math.sin(-heading)
    rot = np.array([[c, -s], [s, c]])
    return (track.left - start) @ rot.T, (track.right - start) @ rot.T


def export_track(track: Track, cones_path, gaps_path) -> tuple[int, int]:
    """导出 cones.csv + gaps.json 供 track_generator 生成 Gazebo world。

    左侧红(r0,r1,…)、右侧蓝(b0,b1,…),沿行进方向编号;gaps 取同色相邻、
    且间距在挡板判据内的所有位置(他们的 generate.py 会从中随机选 3 处放挡板)。
    返回 (锥桶数, 候选间隙数)。
    """
    left, right = _align_to_start(track)
    rows, gaps = [], []
    for prefix, colour, pts in (("r", "red", left), ("b", "blue", right)):
        ids = [f"{prefix}{i}" for i in range(len(pts))]
        rows += [(i, colour, f"{x:.4f}", f"{y:.4f}") for i, (x, y) in zip(ids, pts)]
        for i in range(len(pts)):                       # 闭环,最后一个接回第一个
            j = (i + 1) % len(pts)
            if GAP_MIN <= math.dist(pts[i], pts[j]) < GAP_MAX:
                gaps.append([ids[i], ids[j]])
    if len(gaps) < 3:
        raise ValueError(f"候选间隙只有 {len(gaps)} 处,不足 track_generator 要求的 3 处")

    cones_path, gaps_path = Path(cones_path), Path(gaps_path)
    cones_path.parent.mkdir(parents=True, exist_ok=True)
    with open(cones_path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "color", "x", "y"])
        w.writerows(rows)
    gaps_path.write_text(json.dumps(gaps, indent=2), encoding="utf-8")
    return len(rows), len(gaps)
