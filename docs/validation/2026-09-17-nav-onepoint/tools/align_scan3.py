# 离线激光-地图对齐 #3: 单点导航轮 (2026-09-17 深夜)
# 车被重新摆位(位置未知), 全房域粗搜: 中心=房间中心(-0.4,-0.25), 域 ±1.8, 步 0.12, yaw 2° 分3块
import json
import math

import numpy as np

MAP = r"D:/Claude/projects/hwcart_amcl_stage2/map.pgm"
SCAN = r"D:/Claude/projects/hwcart_amcl_stage2/scan_nav_clean.json"
RES = 0.05
OX, OY = -2.5002234304301752497, -2.0682804995853985
LASER_DX = 0.07
CX, CY = -0.4, -0.25   # 房间中心(地图 x∈[-2.5,1.65] y∈[-2.07,1.53])


def load_pgm(path):
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P5"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        assert int(f.readline().strip()) == 255
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    return data


def dilate3(b):
    out = b.copy()
    for ax in (0, 1):
        for sh in (-1, 1):
            out |= np.roll(b, sh, axis=ax)
    return out


def main():
    img = load_pgm(MAP)
    occ = img < 100
    hit = dilate3(occ)
    H, W = occ.shape

    d = json.load(open(SCAN))
    r = np.array([v if isinstance(v, (int, float)) and math.isfinite(v) and v > 0.15
                  else np.nan for v in d["ranges"]])
    a = d["angle_min"] + np.arange(len(r)) * d["angle_increment"]
    m = np.isfinite(r)
    r, a = r[m], a[m]
    r, a = r[::4], a[::4]
    pb = np.stack([LASER_DX + r * np.cos(a), r * np.sin(a)], axis=1)
    print(f"有效波束(抽稀后): {len(pb)}")

    def score(cands):
        x = cands[:, 0:1]
        y = cands[:, 1:2]
        yaw = cands[:, 2:3]
        c, s = np.cos(yaw), np.sin(yaw)
        wx = x + c * pb[None, :, 0] - s * pb[None, :, 1]
        wy = y + s * pb[None, :, 0] + c * pb[None, :, 1]
        col = np.floor((wx - OX) / RES).astype(np.int64)
        row = np.floor((OY + H * RES - wy) / RES).astype(np.int64)
        ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
        col = np.clip(col, 0, W - 1)
        row = np.clip(row, 0, H - 1)
        hits = hit[row, col] & ok
        return hits.mean(axis=1)

    # 粗搜: 全房域
    yaws = np.deg2rad(np.arange(0, 360, 2.0))
    xs = np.arange(CX - 1.8, CX + 1.801, 0.12)
    ys = np.arange(CY - 1.8, CY + 1.801, 0.12)
    best = []
    for yaw_chunk in np.array_split(yaws, 3):
        XX, YY, YYA = np.meshgrid(xs, ys, yaw_chunk, indexing="ij")
        coarse = np.stack([XX.ravel(), YY.ravel(), YYA.ravel()], axis=1)
        sc = score(coarse)
        order = np.argsort(sc)[::-1][:200]
        best.extend(zip(coarse[order], sc[order]))
    best.sort(key=lambda t: -t[1])
    print("== 粗搜 top5 ==")
    for (x, y, yaw), s in best[:5]:
        print(f"  x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):7.1f}°  score={s:.4f}")

    (bx, by, byaw), _ = best[0]
    fx = np.arange(bx - 0.15, bx + 0.151, 0.02)
    fy = np.arange(by - 0.15, by + 0.151, 0.02)
    fyaw = np.arange(byaw - np.deg2rad(3), byaw + np.deg2rad(3.001), np.deg2rad(0.5))
    XX, YY, YYA = np.meshgrid(fx, fy, fyaw, indexing="ij")
    fine = np.stack([XX.ravel(), YY.ravel(), YYA.ravel()], axis=1)
    sf = score(fine)
    o2 = np.argsort(sf)[::-1]
    print("== 精搜 top5 ==")
    for i in o2[:5]:
        x, y, yaw = fine[i]
        print(f"  x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):7.2f}° score={sf[i]:.4f}")
    gx, gy, gyaw = fine[o2[0]]
    print(f"== 最优: x={gx:+.3f} y={gy:+.3f} yaw={math.degrees(gyaw):+.2f}° score={sf[o2[0]]:.4f}")
    print(f"建议: ./amcl_initpose.sh {gx:.3f} {gy:.3f} {math.degrees(gyaw):.1f}")


if __name__ == "__main__":
    main()
