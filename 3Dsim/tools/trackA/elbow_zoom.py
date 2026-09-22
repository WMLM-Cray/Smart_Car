# [左U肘弯放大检查] G4规划失败链最后一环:肘弯区(TL弧→顶道)原始占据栅格
# ASCII 渲染 x∈[-3.3,-1.3] y∈[3.7,5.35],统计走廊内部孤立占据格(SLAM噪声)。
import numpy as np
import yaml
from scipy.ndimage import convolve

MD = ("/home/lzx/hwcart_sim/hwcart_sim/install/racecar_bringup_sim"
      "/share/racecar_bringup_sim/maps")
y = yaml.safe_load(open(f"{MD}/trackA_map_full.yaml"))
res = float(y["resolution"])
ox, oy = y["origin"][0], y["origin"][1]
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
occ = (img == 0)
unk = (img == 205)


def c0(x):
    return int((x-ox)/res)


def r0(yy):
    return h-1-int((yy-oy)/res)


print("图例 #=占据 ?=未知 .=自由  (每字符=0.1m×0.1m,聚合2×2格)")
xs = np.arange(-3.3, -1.29, 0.1)
hdr = "      "
for x in xs:
    hdr += str(int(abs(x*10) % 10)) if abs(x*10) % 1 < 0.01 else " "
print(hdr)
yy = 5.35
while yy >= 3.7:
    row = ""
    for x in xs:
        rr, cc = r0(yy), c0(x)
        blk_occ = occ[max(0, rr-1):rr+1, max(0, cc-1):cc+1]
        blk_unk = unk[max(0, rr-1):rr+1, max(0, cc-1):cc+1]
        if blk_occ.any():
            row += "#"
        elif blk_unk.any():
            row += "?"
        else:
            row += "."
    print(f"y={yy:5.2f} {row}")
    yy -= 0.1

k = np.ones((3, 3), int)
k[1, 1] = 0
nb = convolve(occ.astype(int), k, mode="constant")
print(f"\n全图占据格={occ.sum()}  孤立0邻={int((occ&(nb==0)).sum())}  仅1邻={int((occ&(nb==1)).sum())}")

x0, x1, y0, y1 = -3.0, -1.6, 4.2, 5.2
sub = occ[r0(y1):r0(y0)+1, c0(x0):c0(x1)+1]
snb = convolve(sub.astype(int), k, mode="constant")
print(f"肘弯窗[{x0},{x1}]x[{y0},{y1}] 占据={int(sub.sum())} "
      f"0邻={int((sub&(snb==0)).sum())} 1邻={int((sub&(snb==1)).sum())} "
      f">=2邻={int((sub&(snb>=2)).sum())}")

# 顶道干净段对照(x∈[0,1.4])
ref = occ[r0(5.2):r0(4.2)+1, c0(0.0):c0(1.4)+1]
rnb = convolve(ref.astype(int), k, mode="constant")
print(f"顶道对照窗[0,1.4]x[4.2,5.2] 占据={int(ref.sum())} "
      f"0邻={int((ref&(rnb==0)).sum())} 1邻={int((ref&(rnb==1)).sum())}")
