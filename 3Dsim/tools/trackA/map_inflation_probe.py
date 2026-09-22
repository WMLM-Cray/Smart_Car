# [Nav2调试] G4规划亚毫秒秒拒 → 解析存档地图+膨胀,定位被封锁位姿/走廊
# 原理:global_costmap=static_layer+inflation(0.45),无障碍层;Smac瞬间失败
#       ⇒ 起点/终点位姿落在 INSCRIBED(253)带(距障碍≤足印内切半径0.175m)
#       或必经走廊被该带封死。本脚本:PGM→占据栅格→多源距离场→逐一判定。
import sys
import numpy as np
import yaml

MAP_DIR = ("/home/lzx/hwcart_sim/hwcart_sim/install/racecar_bringup_sim"
           "/share/racecar_bringup_sim/maps")
INSCRIBED_M = 0.175   # 足印内切半径(0.35/2),INSCRIBED 带宽
RES = 0.05

# ---- 读 yaml/pgm ----
with open(f"{MAP_DIR}/trackA_map_full.yaml") as f:
    y = yaml.safe_load(f)
res = float(y["resolution"])   # 0.05
ox, oy = y["origin"][0], y["origin"][1]
raw = open(f"{MAP_DIR}/trackA_map_full.pgm", "rb").read()
# P5 头: magic\n width height\n maxval\n
tok, idx = [], 0
while len(tok) < 4:
    while raw[idx : idx + 1].isspace():
        idx += 1
    if raw[idx : idx + 1] == b"#":
        while raw[idx : idx + 1] != b"\n":
            idx += 1
        continue
    s = idx
    while not raw[idx : idx + 1].isspace():
        idx += 1
    tok.append(raw[s:idx])
idx += 1
w, h = int(tok[1]), int(tok[2])
img = np.frombuffer(raw[idx : idx + w * h], dtype=np.uint8).reshape(h, w)
occ = img == 0            # 占据(墙/锥)
unknown = img == 205
print(f"地图 {w}x{h}@{res}  origin=({ox:.2f},{oy:.2f})  "
      f"占据{occ.sum()}格 未知{unknown.sum()}格")

# ---- 多源欧氏距离场(BFS 按 8 邻接近似,栅格级精度足够) ----
INF = 1e9
dist = np.full((h, w), INF)
dist[occ] = 0.0
frontier = list(zip(*np.nonzero(occ)))
step = ((1, 0, res), (-1, 0, res), (0, 1, res), (0, -1, res),
        (1, 1, res * 1.414), (1, -1, res * 1.414),
        (-1, 1, res * 1.414), (-1, -1, res * 1.414))
while frontier:
    nxt = []
    for r, c in frontier:
        for dr, dc, cost in step:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w:
                nd = dist[r, c] + cost
                if nd < dist[nr, nc]:
                    dist[nr, nc] = nd
                    nxt.append((nr, nc))
    frontier = nxt

def cell(x, y):   # 世界→栅格(PGM 第0行=最高y)
    return int(round((y - oy) / res - 0.5)), int(round((x - ox) / res - 0.5))

def d_at(x, y):
    r, c = cell(x, y)
    return dist[r, c]

print("\n== 关键位姿的距障碍距离(需 >0.175 才不在 INSCRIBED 带) ==")
for name, x, yy in [("G4起点_顶道左端", -1.89, 4.74), ("G4终点_出生点", 0.0, 0.0),
                    ("G1终点_短直道", 2.5, 0.0), ("G3终点", -2.0, 4.75)]:
    print(f"  {name} ({x:.2f},{yy:.2f}): {d_at(x, yy):.3f}m")

# ---- 连通性:从起点只走 dist>阈值的格,能否到终点 ----
print("\n== 走廊连通性(避开 INSCRIBED 带,需通过左U) ==")
sr, sc = cell(-1.89, 4.74)
gr, gc = cell(0.0, 0.0)
for thr_m in (INSCRIBED_M, 0.19, 0.25, 0.30):
    ok = dist[sr, sc] > thr_m and dist[gr, gc] > thr_m
    seen = np.zeros((h, w), bool)
    stack = [(sr, sc)]
    seen[sr, sc] = True
    while stack:
        r, c = stack.pop()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                nr, nc = r + dr, c + dc
                if (0 <= nr < h and 0 <= nc < w and not seen[nr, nc]
                        and dist[nr, nc] > thr_m):
                    seen[nr, nc] = True
                    stack.append((nr, nc))
    print(f"  阈值{thr_m:.3f}m: 起点干净={dist[sr,sc]>thr_m} "
          f"终点干净={dist[gr,gc]>thr_m} 连通={seen[gr,gc]}")

# ---- 左U必经段走廊最窄处(顶道左端 x∈[-2.9,-1.8], y∈[3.9,4.75] 及以下弧段) ----
print("\n== 地图最窄走廊扫描(每列最小自由距离) ==")
worst = []
for c in range(0, w):
    col = dist[:, c]
    free = col[~occ[:, c]]
    if free.size:
        worst.append((float(free.min()), ox + (c + 0.5) * res))
worst.sort()
for mn, x in worst[:8]:
    print(f"  x={x:6.2f}m  该列最窄自由距离={mn:.3f}m")
print("\n(供对照:车道自由宽1.22m → 中心距锥圆~0.61m;距障碍<0.175 即 INSCRIBED)")
