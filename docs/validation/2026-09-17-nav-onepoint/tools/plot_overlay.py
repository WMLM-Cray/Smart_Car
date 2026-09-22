# 本地渲染: 对齐候选位姿 + 激光点 叠加到地图 (验证 initpose 候选用)
# 用法: python plot_overlay.py <scan.json> <x> <y> <yaw_deg> <out.png>
import json
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

scan_path, PX, PY, YAW, out = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), sys.argv[5]

MAP = r"D:/Claude/projects/hwcart_amcl_stage2/map.pgm"
RES = 0.05
OX, OY = -2.5002234304301752497, -2.0682804995853985
LASER_DX = 0.07

with open(MAP, "rb") as f:
    assert f.readline().strip() == b"P5"
    line = f.readline()
    while line.startswith(b"#"):
        line = f.readline()
    W, H = map(int, line.split())
    assert int(f.readline().strip()) == 255
    data = np.frombuffer(f.read(), dtype=np.uint8).reshape(H, W)

d = json.load(open(scan_path))
r = np.array([v if v is not None and v > 0.15 else np.nan for v in d["ranges"]])
a = d["angle_min"] + np.arange(len(r)) * d["angle_increment"]
yaw = math.radians(YAW)
c, s = math.cos(yaw), math.sin(yaw)
# 车体系波束 -> 世界系 (位姿为车体原点, 雷达前移 LASER_DX)
bx = LASER_DX + r * np.cos(a)
by = r * np.sin(a)
wx = PX + c * bx - s * by
wy = PY + s * bx + c * by

fig, ax = plt.subplots(figsize=(8, 7))
ax.imshow(data, cmap="gray", extent=[OX, OX + W * RES, OY, OY + H * RES], origin="upper")
ax.plot(wx, wy, "r.", markersize=1.5, alpha=0.7)
# 车体三角
th = np.linspace(yaw + 2.6, yaw - 2.6, 20)
tri_x = np.r_[PX + 0.25 * np.cos(yaw), PX + 0.22 * np.cos(th)]
tri_y = np.r_[PY + 0.25 * np.sin(yaw), PY + 0.22 * np.sin(th)]
ax.plot(tri_x, tri_y, "g-", linewidth=2)
ax.plot(PX, PY, "g^", markersize=10)
ax.set_title(f"pose=({PX:.2f},{PY:.2f},{YAW:.1f}deg)  beams={np.isfinite(r).sum()}")
ax.set_xlabel("x [m]")
fig.savefig(out, dpi=110, bbox_inches="tight")
print("saved", out)
