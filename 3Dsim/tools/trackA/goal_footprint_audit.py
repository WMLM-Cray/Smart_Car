# [Nav2离线终审] 左U目标不可规划之谜:合成代价地图 + 矩形足印碰撞测试
# 原理:global_costmap=StaticLayer(PGM)+InflationLayer(0.45/8.0/内切0.175),
#       Smac 目标位姿判定=足印多边形覆盖格 max cost>=253 即碰撞(255未知可通)。
#       离线复刻该判定,对失败目标 A/B/E/F/G/H/P1 与成功目标 C/D/P4/P2 做全
#       朝向扫描,分辨"位置被封"还是"朝向被封"。
import numpy as np
import yaml

MD = ("/home/lzx/hwcart_sim/hwcart_sim/install/racecar_bringup_sim"
      "/share/racecar_bringup_sim/maps")
RES = 0.05
INFL = 0.50
SCALE = 8.0
PAD = 0.08             # footprint_padding(第六轮 0.06→0.08:右U残余0.086<0.10)
INSCR = 0.175 + PAD    # 垫后足印内切半径(costmap 253 带由此决定)
# 顶点必须逆时针(叉积内点测试按逆时针写;顺时针会把内部全判在外面)
# 足印=冻结矩形外扩 PAD(nav2 padding 语义:多边形各边外推)
FP = np.array([[-0.28-PAD, -0.175-PAD], [0.28+PAD, -0.175-PAD],
               [0.28+PAD, 0.175+PAD], [-0.28-PAD, 0.175+PAD]])

y = yaml.safe_load(open(f"{MD}/trackA_map_full.yaml"))
res = float(y["resolution"]); ox, oy = y["origin"][0], y["origin"][1]
raw = open(f"{MD}/trackA_map_full.pgm", "rb").read()
tok, idx = [], 0
while len(tok) < 4:
    while raw[idx:idx+1].isspace():
        idx += 1
    if raw[idx:idx+1] == b"#":
        while raw[idx:idx+1] != b"\n":
            idx += 1
        continue
    s = idx
    while not raw[idx:idx+1].isspace():
        idx += 1
    tok.append(raw[s:idx])
idx += 1
w, h = int(tok[1]), int(tok[2])
img = np.frombuffer(raw[idx:idx+w*h], dtype=np.uint8).reshape(h, w)
occ = img == 0
unknown = img == 205

# 距离场(到占据格):优先 scipy 精确欧氏,chamfer 3-4 兜底(略高估→膨胀带偏窄)
try:
    from scipy.ndimage import distance_transform_edt
    dist = distance_transform_edt(~occ) * res
    print("(scipy 精确欧氏距离场)")
except ImportError:
    INF = 1e9
    dist = np.full((h, w), INF); dist[occ] = 0.0
    fr = list(zip(*np.nonzero(occ)))
    st = ((1, 0, res), (-1, 0, res), (0, 1, res), (0, -1, res),
          (1, 1, res*1.414), (1, -1, res*1.414), (-1, 1, res*1.414), (-1, -1, res*1.414))
    while fr:
        nx = []
        for r, c in fr:
            for dr, dc, cs in st:
                nr, nc = r+dr, c+dc
                if 0 <= nr < h and 0 <= nc < w and dist[r, c]+cs < dist[nr, nc]:
                    dist[nr, nc] = dist[r, c]+cs; nx.append((nr, nc))
        fr = nx

# 合成 costmap(nav2 公式)
cost = np.zeros((h, w), np.int32)
cost[occ] = 254
m_infl = (~occ) & (dist <= INFL)
cost[m_infl] = np.clip((252*np.exp(-SCALE*(dist[m_infl]-INSCR))).astype(int), 1, 252)
cost[(~occ) & (dist <= INSCR)] = 253
cost[unknown] = 255   # 未知可通(allow_unknown=true)

def cell(x, yy):
    j = int((yy-oy)/res); i = int((x-ox)/res)
    return h-1-j, i

def fp_max_cost(x, yy, yaw):
    """矩形足印旋转 yaw 后覆盖格的最大 cost(255 视作 0=可通未知)"""
    c, s = np.cos(yaw), np.sin(yaw)
    # 行向量形式的 +yaw 旋转:FP @ [[c,-s],[s,c]] 实为 −yaw(矩阵取转置效
    # 应),0/90/180° 目标因矩形镜像对称不暴露,240° 斜朝向首踩(2026-09-16
    # 发卡拐角误判 253,现场格 (−9.775,3.462) 距心 0.375>横向支撑 0.255)
    rot = FP @ np.array([[c, s], [-s, c]])
    pts = rot + np.array([x, yy])
    r0, r1 = cell(pts[:, 0].min(), pts[:, 1].max()), cell(pts[:, 0].max(), pts[:, 1].min())
    rmin, rmax = min(r0[0], r1[0])-1, max(r0[0], r1[0])+1
    cmin, cmax = min(r0[1], r1[1])-1, max(r0[1], r1[1])+1
    rmin, rmax = max(0, rmin), min(h-1, rmax)
    cmin, cmax = max(0, cmin), min(w-1, cmax)
    best = 0
    poly = pts
    xs_min, ys_min = poly[:, 0].min(), poly[:, 1].min()
    # 逐外接盒格心做点在多边形内测试
    for rr in range(rmin, rmax+1):
        for cc in range(cmin, cmax+1):
            gx = ox + (cc+0.5)*res
            gy = oy + (h-1-rr+0.5)*res
            inside = True
            for k in range(4):
                x1, y1 = poly[k]; x2, y2 = poly[(k+1) % 4]
                if (x2-x1)*(gy-y1)-(y2-y1)*(gx-x1) < -1e-9:
                    inside = False; break
            if inside:
                v = cost[rr, cc]
                if v == 255:
                    v = 0    # 未知可通
                best = max(best, v)
    return best

GOALS = [("A失败", -2.60, 3.00, 90), ("B失败", -2.58, 3.60, 90),
         ("E失败", -2.32, 4.55, 10), ("P1失败", -2.30, 4.75, 0),
         ("P4成功", -2.60, 2.00, 270), ("P2成功", 2.00, 4.75, 180),
         # ---- v2 目标集(63锥真值几何:顶道中心4.0,S弯,远左发卡) ----
         ("v2_G1_短直道", 2.5, 0.0, 0), ("v2_G2_右U出口", 2.0, 4.0, 180),
         ("v2_G3_顶道左行", -2.0, 4.0, 180), ("v2_G4a_S弯中", -5.4, 2.85, 180),
         ("v2_G4b_发卡中段", -10.25, 2.6, 270), ("v2_G4c_回出生点", 0.0, 0.0, 0),
         # 发卡两个直角的过弯参考位姿
         ("v2_角1_左上入口", -10.25, 3.9, 200), ("v2_角2_左下出口", -10.0, 1.0, 300),
         ("v2_S弯西口", -7.4, 3.6, 190),
         # ---- v3 增补(S 弯拆段中间目标) ----
         ("v3_S弯东口", -3.6, 3.7, 200), ("v3_顶道西段", -8.0, 4.0, 185),
         ("v3_发卡入口", -10.0, 4.0, 235),
         # ---- v4 增补(右U中段压中线) ----
         ("v4_右U中段", 5.0, 2.0, 90), ("v4_S弯中点改", -5.4, 2.85, 200),
         # ---- v5 增补(发卡拐角提前压外线,拆 85° 急转为 55°+30°) ----
         ("v5_发卡拐角", -10.1, 3.65, 240),
         # ---- v7 增补(右U上口:出口弧切 r16 (3.05,3.25),第八轮 0.046;
         # 候选 (3.0,3.9,210)/(3.05,3.95,210) 压 253 拒,(2.9,4.0,200) 净) ----
         ("v7_右U上口", 2.9, 4.0, 200),
         # ---- v12 增补(右U出口 yaw 对齐锚点带 200°,防短跳终点绕圈) ----
         ("v12_右U出口200", 2.0, 4.0, 200),
         # ---- v6 教训存档(发卡出口:b36-I10 斜缝不放目标) ----
         # (−9.35,0.3,0°) 位姿本身 maxcost 78 干净,但北线弧中段位姿垫后
         # 足印对 b36 像素净距仅 0.02m 压 253 带 → Smac 只能南绕 b36 蹭锥
         # 中止(第七轮实测 ABORTED);(−9.6/−9.55,0.4) 则目标位姿本身被拒。
         # 结论:5cm 像格把斜缝地板钉在 ~0.076m,不放出口中间目标。
         ]
print("== 目标位姿足印测试(>=253=规划器必拒;252-128=高代价但可通) ==")
for nm, gx, gy, gyaw in GOALS:
    mc = fp_max_cost(gx, gy, np.radians(gyaw))
    verdict = "拒绝" if mc >= 253 else ("高费" if mc >= 150 else "干净")
    print(f"  {nm} ({gx:.2f},{gy:.2f},{gyaw}°): maxcost={mc:3d} → {verdict}")

print("\n== 失败位置全朝向扫描(找可用朝向) ==")
for nm, gx, gy, gyaw in GOALS:
    if "失败" not in nm:
        continue
    oks = []
    for yaw in range(0, 360, 15):
        mc = fp_max_cost(gx, gy, np.radians(yaw))
        if mc < 253:
            oks.append(f"{yaw}°({mc})")
    print(f"  {nm} ({gx:.2f},{gy:.2f}): 可用朝向 = {', '.join(oks) if oks else '无!全部朝向被拒'}")
