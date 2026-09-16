"""冻结 A 赛道(docs/track/trackA_cones.csv)的加载适配器。

A 赛道是比赛图 fig9 尺寸链重建的冻结几何(阶段2 v5 六弧模型,63 锥),
**不允许重新随机化**:本模块只读 CSV 与冻结参考线常量,不做任何随机采样。

语义对照(与随机赛道 sim.track.make_track 同一约定):
    行进方向逆时针;左侧红锥(内圈)、右侧蓝锥(外圈)。
    出生位姿固定 (11.0, 0.75, 0)——与 Gazebo 验收基线 sim_bringup
    的 spawn_x/y/yaw 一致,坐标系同为赛道世界系(y0=底边,x0=左竖边)。
黄 V 无锥区:E3/E4/I3/I4 段 CSV 本身无锥(黄 V 是隔离带不是锥桶区),
    该语义随 CSV 原样保留;黄 V 挡板不参与本仿真的碰撞判定(判罚物=锥桶),
    车道参考线走 V 谷折线(相邻锥对中点)已避开挡板。

参考线常量与 tools/trackA/test_driver.py 同源(冻结几何推导记录见
docs/track/trackA_阶段3_2_测试驾驶器.md)。本文件不 import test_driver
(它依赖 rclpy),正确性由 sim/tests/test_trackA.py 独立对账保证:
整圈长度 35.46m、63 锥=37蓝+26红、出生位姿、两次加载逐位一致。
"""
from __future__ import annotations

import csv
import math
import re
from dataclasses import replace
from pathlib import Path

import numpy as np

from .params import TrackParams
from .track import Track

DEFAULT_CSV = Path(__file__).resolve().parent.parent / "docs/track/trackA_cones.csv"

# ---- 冻结几何常量(逐项与 tools/trackA/test_driver.py 同源,勿单边改动) ----
BOT_Y, TOP_Y = 0.75, 4.75            # 底道/顶道中线 y
RX, RY, R = 14.03, 2.75, 2.0         # 右U弧(内环R1.25/外环R2.75 的同心中线)
TL_C, TL_R = (1.75, 3.75), 1.0       # 左上切角小弧
BL_C, BL_R = (2.75, 2.75), 2.0       # 左下大弧
V_NODES = [                          # 锥桶V段中线(相邻锥对中点折线)
    (8.410, 4.750), (7.4385, 4.4625), (6.5465, 3.8665), (5.578, 3.578),
    (4.6105, 3.8665), (3.718, 4.4625), (2.750, 4.750),
]
START = (11.0, 0.75)                 # 出生点(与 Gazebo 验收基线一致)
SPAWN = (11.0, 0.75, 0.0)            # 后轴中心 + 朝向(+x)

# A 赛道冻结参数:道宽 1.5 为阶段2定案(黄 V 区法向宽 1.386 例外,见
# docs/track/trackA_尺寸残差表.md);锥尺寸=官方 280mm 底径/630mm 高。
TRACKA_PARAMS = replace(
    TrackParams(),
    width=1.5,
    cone_base_radius=0.14,
    cone_top_radius=0.025,
    cone_height=0.63,
    cone_spacing=0.0,       # 冻结赛道锥距不均(0.92~1.25m),此字段不适用
)

_ID_RE = re.compile(r"^[br]\d{2,}$")
N_BLUE, N_RED = 37, 26              # 冻结基线锥数(蓝外红内)


def _seg_line(p0, p1, step=0.05):
    n = max(2, int(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / step) + 1)
    return [(p0[0] + (p1[0] - p0[0]) * i / (n - 1),
             p0[1] + (p1[1] - p0[1]) * i / (n - 1)) for i in range(n)]


def _seg_arc(cx, cy, r, a0, a1, step=0.05):
    n = max(2, int(abs(a1 - a0) * r / step) + 1)
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / (n - 1)),
             cy + r * math.sin(a0 + (a1 - a0) * i / (n - 1))) for i in range(n)]


def build_centerline() -> np.ndarray:
    """整圈车道参考线(起点=出生点,逆时针,0.05m 步长,不闭合重复首点)。

    与 test_driver.build_route("full") 同公式:底道直 → 右U弧 → 顶右直 →
    V 谷折线 → 左U复合弯(切角弧+竖直+左下大弧)→ 底道回起点。
    """
    pts = (_seg_line(START, (RX, BOT_Y))[:-1]
           + _seg_arc(RX, RY, R, -math.pi / 2, math.pi / 2)[1:-1]
           + _seg_line((RX, TOP_Y), V_NODES[0])[1:-1])
    for a, b in zip(V_NODES, V_NODES[1:]):
        pts += _seg_line(a, b)[1:-1]
    pts += (_seg_line(V_NODES[-1], (TL_C[0], TOP_Y))[1:-1]
            + _seg_arc(TL_C[0], TL_C[1], TL_R, math.pi / 2, math.pi)[1:-1]
            + _seg_line((0.75, TL_C[1]), (0.75, BL_C[1]))[1:-1]
            + _seg_arc(BL_C[0], BL_C[1], BL_R, math.pi, 3 * math.pi / 2)[1:-1]
            + _seg_line((BL_C[0], BOT_Y), START)[1:])
    if math.dist(pts[-1], pts[0]) < 1e-9:      # 去掉与首点重合的尾点
        pts = pts[:-1]
    return np.asarray(pts)


def load_trackA(csv_path=DEFAULT_CSV) -> Track:
    """读冻结 CSV → Track。schema 不符立即报错,绝不静默换坐标/换色。

    CSV 格式(A 赛道冻结版,与 export_track 的训练版是两种格式,见
    docs/SIM_INTEROP.md 对照表):id,color,x,y,segment;
    id=b01../r01..(b=blue 外圈,r=red 内圈);坐标=赛道世界系米制。
    """
    path = Path(csv_path)
    if not path.exists():
        raise FileNotFoundError(f"冻结赛道 CSV 不存在:{path}(A 赛道不做随机替代)")
    blue, red = [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None or not {"id", "color", "x", "y"} <= set(reader.fieldnames):
            raise ValueError(f"{path}: 表头须含 id,color,x,y(实际 {reader.fieldnames})")
        for ln, row in enumerate(reader, start=2):
            cid = (row["id"] or "").strip()
            if not _ID_RE.match(cid):
                raise ValueError(f"{path}:{ln}: id {cid!r} 不匹配 ^[br]\\d{{2,}}$(A 赛道格式)")
            colour = (row["color"] or "").strip().lower()
            try:
                x, y = float(row["x"]), float(row["y"])
            except (TypeError, ValueError):
                raise ValueError(f"{path}:{ln}: x/y 不是数字({row['x']!r},{row['y']!r})") from None
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError(f"{path}:{ln}: x/y 非有限值")
            if colour == "blue":
                blue.append((x, y))
            elif colour == "red":
                red.append((x, y))
            else:
                raise ValueError(f"{path}:{ln}: color {colour!r} 须为 blue/red")
    if len(blue) != N_BLUE or len(red) != N_RED:
        raise ValueError(f"{path}: 冻结基线应为 {N_BLUE}蓝+{N_RED}红,实得 {len(blue)}蓝+{len(red)}红")
    # 逆时针行驶:左=红(内圈),右=蓝(外圈)——与 make_track 的 left/right 语义一致
    return Track(centerline=build_centerline(),
                 left=np.asarray(red), right=np.asarray(blue),
                 params=TRACKA_PARAMS, spawn=SPAWN, name="trackA")
