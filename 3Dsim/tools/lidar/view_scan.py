#!/usr/bin/env python3
"""把 /scan 数据画出来看。三种视角一次出图。

用法:
    python view_scan.py                          # 用 scan.npz,出 scan_overview.png
    python view_scan.py scan.npz --frame 100     # 指定看哪一帧
    python view_scan.py scan.npz --show          # 直接弹窗口(而不是只存文件)
    python view_scan.py carscan_bag/carscan_bag_0.db3   # 也能直接吃 .db3

图里四个面板:
  左上 单帧极坐标   —— 这一帧雷达"看到"的形状,最直观
  右上 单帧笛卡尔   —— 同一帧摊平到 XY,能看朝向和距离
  左下 时间-角度瀑布 —— 横轴角度、纵轴时间,一眼看出扫到什么、什么时候
  右下 诊断         —— 有效点数随时间 + 距离分布 + 帧间隔抖动
"""
from __future__ import annotations

import argparse
import sys

import matplotlib
import numpy as np

from parse_bag import load_scans


def load(path: str):
    if path.endswith(".db3"):
        scans = load_scans(path)
        R = np.stack([s["ranges"] for s in scans])
        meta = scans[0]
        t = np.array([s["bag_time"] for s in scans])
        t = t - t[0]
    else:
        z = np.load(path)
        R, meta = z["ranges"], {k: z[k] for k in z.files}
        t = z["bag_time"] - z["bag_time"][0]
    amin = float(meta["angle_min"])
    ainc = float(meta["angle_increment"])
    n = R.shape[1]
    angles = amin + np.arange(n) * ainc
    return R, angles, t, meta


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="scan.npz")
    ap.add_argument("--frame", type=int, default=None,
                    help="看第几帧(默认挑有效点最多的一帧)")
    ap.add_argument("--out", default="scan_overview.png")
    ap.add_argument("--show", action="store_true", help="弹窗显示")
    ap.add_argument("--dpi", type=int, default=130)
    a = ap.parse_args()

    try:
        R, angles, t, meta = load(a.path)
    except Exception as exc:                       # noqa: BLE001
        print(f"读不了 {a.path}: {exc}", file=sys.stderr)
        return 1

    finite = np.isfinite(R)
    valid_per_frame = finite.sum(axis=1)
    k = a.frame if a.frame is not None else int(np.argmax(valid_per_frame))
    if not 0 <= k < len(R):
        print(f"--frame {k} 超出范围(共 {len(R)} 帧)", file=sys.stderr)
        return 1

    # -π 和 +π 是同一个物理方向,画极坐标时会重复;只取一半来做散点
    half = angles < np.pi - 1e-9

    fig = plt.figure(figsize=(15, 11))
    fig.suptitle(
        f"LiDAR /scan  —  {len(R)} frames, {R.shape[1]} pts, "
        f"{(t[-1]-t[0]) if len(t) > 1 else 0:.1f} s",
        fontsize=13)

    # ---- 左上:单帧极坐标 ----
    ax = fig.add_subplot(2, 2, 1, projection="polar")
    r = R[k]
    ok = np.isfinite(r)
    ax.scatter(angles[ok], r[ok], s=2, c="tab:blue")
    ax.set_theta_zero_location("N")       # 0° 朝上
    ax.set_theta_direction(-1)            # 顺时针 = ROS 惯例(左正)
    ax.set_title(f"frame {k}  (polar, {valid_per_frame[k]} pts)", pad=14)
    ax.set_rmax(float(np.nanmax(r[ok])) * 1.05 if ok.any() else 1.0)
    ax.tick_params(labelsize=8)

    # ---- 右上:单帧笛卡尔 ----
    ax = fig.add_subplot(2, 2, 2)
    m = ok & half
    x, y = r[m] * np.cos(angles[m]), r[m] * np.sin(angles[m])
    ax.scatter(x, y, s=3, c="tab:blue")
    ax.plot(0, 0, "r^", ms=10, label="lidar")
    ax.annotate("+x (front)", xy=(0.5, 0.1), xycoords="axes fraction",
                color="gray", fontsize=9)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.axhline(0, color="gray", lw=0.5)
    ax.axvline(0, color="gray", lw=0.5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"frame {k}  (cartesian)")
    ax.legend(loc="upper right", fontsize=8)

    # ---- 左下:时间-角度瀑布图 ----
    ax = fig.add_subplot(2, 2, 3)
    M = np.where(finite, R, np.nan)
    im = ax.imshow(M, aspect="auto", origin="lower", cmap="viridis",
                   extent=[np.degrees(angles[0]), np.degrees(angles[-1]),
                           t[0], t[-1]], vmin=0,
                   vmax=float(np.nanpercentile(M, 98)) if finite.any() else 1)
    ax.set_xlabel("angle (deg)   0 = front, + = left")
    ax.set_ylabel("time (s)")
    ax.set_title("range vs angle over time  (dark = no return)")
    fig.colorbar(im, ax=ax, label="range (m)", fraction=0.046)

    # ---- 右下:诊断 ----
    ax = fig.add_subplot(2, 2, 4)
    ax.plot(t, valid_per_frame, lw=1, color="tab:green")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("valid pts / frame", color="tab:green")
    ax.tick_params(axis="y", labelcolor="tab:green")
    ax.set_ylim(0, R.shape[1])
    ax.grid(alpha=0.3)
    ax2 = ax.twinx()
    dt = np.diff(t) * 1000
    if len(dt):
        ax2.plot(t[1:], dt, lw=0.8, color="tab:orange", alpha=0.7)
        ax2.set_ylabel("frame interval (ms)", color="tab:orange")
        ax2.tick_params(axis="y", labelcolor="tab:orange")
        ax2.axhline(np.median(dt), color="tab:orange", ls="--", lw=0.8)
    ax.set_title(f"valid pts (median {int(np.median(valid_per_frame))})  "
                 f"& interval (median {np.median(dt) if len(dt) else 0:.1f} ms)")

    fig.tight_layout()
    fig.savefig(a.out, dpi=a.dpi, bbox_inches="tight")
    print(f"已写出 {a.out}")
    if a.show:
        plt.show()
    return 0


if __name__ == "__main__":
    # 在非交互后端下也要能存图;--show 时再切回默认后端
    _argv = sys.argv[1:]
    if "--show" not in _argv:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt                # noqa: E402

    sys.exit(main())
