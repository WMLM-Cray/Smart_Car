# 离线激光-地图对齐: 推算 AMCL 初始位姿依据 (2026-09-17 阶段2)
# 输入: scan_clean.json(一帧/scan, laser_link系) + map.pgm(83x72@0.05, origin已知)
# 模型: 车静止于建图终点附近(用户确认未挪动), odom 从 Run_car 启动归零
#       => 求 map 系位姿 (x,y,yaw): p_map = (x,y) + R(yaw)·((0.07,0) + r·(cosθ,sinθ))
#       (laser_link 在 base 前方 0.07m, 厂家 TF; z 高度差对 2D 无影响)
# 评分: 点落入 [占据∪膨胀1格] 的比例; 粗搜全周2°+±0.6m步0.1 -> 精搜0.5°/0.02m
import json
import math

import numpy as np

MAP = r"D:/Claude/projects/hwcart_amcl_stage2/map.pgm"
SCAN = r"D:/Claude/projects/hwcart_amcl_stage2/scan_clean.json"
RES = 0.05
OX, OY = -2.5002234301752497, -2.0682804995853985
LASER_DX = 0.07  # laser_link 相对 base 前向偏移(厂家静态TF)


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


def dilate3(b):  # 3x3 膨胀(=占据∪邻域), 手写避免scipy依赖
    out = b.copy()
    for ax in (0, 1):
        for sh in (-1, 1):
            out |= np.roll(b, sh, axis=ax)
    return out


def main():
    img = load_pgm(MAP)          # 行0=图像顶部=世界 y 最大
    occ = img < 100              # 黑=占据(slam_toolbox 保存口径)
    hit = dilate3(occ)
    H, W = occ.shape

    d = json.load(open(SCAN))
    r = np.array([v if isinstance(v, (int, float)) and math.isfinite(v) and v > 0.15
                  else np.nan for v in d["ranges"]])
    a = d["angle_min"] + np.arange(len(r)) * d["angle_increment"]
    m = np.isfinite(r)
    r, a = r[m], a[m]
    # 下采样: 每4点取1, ~350点, 保持覆盖
    r, a = r[::4], a[::4]
    # laser 系 -> base 系
    pb = np.stack([LASER_DX + r * np.cos(a), r * np.sin(a)], axis=1)  # (N,2)

    def score(cands):
        # cands: (M,3) x,y,yaw -> (M,) 命中率
        x = cands[:, 0:1]
        y = cands[:, 1:2]
        yaw = cands[:, 2:3]
        c, s = np.cos(yaw), np.sin(yaw)
        wx = x + c * pb[None, :, 0] - s * pb[None, :, 1]   # (M,N)
        wy = y + s * pb[None, :, 0] + c * pb[None, :, 1]
        col = np.floor((wx - OX) / RES).astype(np.int64)
        row = np.floor((OY + H * RES - wy) / RES).astype(np.int64)  # 行0=y最大
        ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
        col = np.clip(col, 0, W - 1)
        row = np.clip(row, 0, H - 1)
        hits = hit[row, col] & ok
        return hits.mean(axis=1)

    # 粗搜
    yaws = np.deg2rad(np.arange(0, 360, 2.0))
    xs = np.arange(-0.6, 0.61, 0.1)
    ys = np.arange(-0.6, 0.61, 0.1)
    XX, YY, YYA = np.meshgrid(xs, ys, yaws, indexing="ij")
    coarse = np.stack([XX.ravel(), YY.ravel(), YYA.ravel()], axis=1)
    sc = score(coarse)
    order = np.argsort(sc)[::-1]
    print("== 粗搜 top5 ==")
    for i in order[:5]:
        x, y, yaw = coarse[i]
        print(f"  x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):7.1f}°  score={sc[i]:.4f}")

    # 精搜(围绕粗最优 ±0.15m / ±3°)
    bx, by, byaw = coarse[order[0]]
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
        print(f"  x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):7.2f}°  score={sf[i]:.4f}")
    gx, gy, gyaw = fine[o2[0]]
    gap = sf[o2[0]] - sf[o2[1]]
    print(f"== 最优: x={gx:+.3f} y={gy:+.3f} yaw={math.degrees(gyaw):+.2f}° "
          f"score={sf[o2[0]]:.4f} (次优差距 {gap:.4f})")
    print(f"建议: ./amcl_initpose.sh {gx:.3f} {gy:.3f} {math.degrees(gyaw):.1f}")


if __name__ == "__main__":
    main()
