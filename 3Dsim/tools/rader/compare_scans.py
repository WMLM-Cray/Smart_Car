#!/usr/bin/env python3
"""把多次录制的 /scan 放一起对比。

用法:
    python compare_scans.py scan_*.npz scan2.npz scan3.npz
    python compare_scans.py --dir scan_20260917_*      # 或直接给目录
    python compare_scans.py scan*.npz --out cmp.png --show

对比三件事:
  1. 关键指标表(帧数/频率/噪声/有效点)—— 看一致性
  2. 逐帧有效点数曲线叠加       —— 看每份的稳定性
  3. 帧间隔分布(箱线图)         —— 看录制质量,抓 bag_time 抖动

重要:间隔同时用 bag_time 和 header.stamp 算。两者不一致 = 录制端卡顿,
不是雷达丢帧(这个区别在 scan3 上真实发生过,别误判成雷达故障)。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
import numpy as np

from view_scan import load


def stats_of(path: str) -> dict:
    R, angles, t, meta = load(path)
    finite = np.isfinite(R)
    valid = finite.sum(axis=1)
    dt_bag = np.diff(t) * 1000
    st = np.asarray(meta["stamp"], dtype=float) if "stamp" in meta else None
    dt_hdr = np.diff(st) * 1000 if st is not None else None
    v = R[finite]
    per_angle = np.nanstd(np.where(finite, R, np.nan), axis=0)
    ok = np.isfinite(per_angle)
    return dict(
        name=Path(path).parent.name if Path(path).name == "scan.npz" else Path(path).stem,
        path=path,
        frames=len(R),
        dur=t[-1] - t[0] if len(t) > 1 else 0.0,
        hz=len(R) / (t[-1] - t[0]) if len(t) > 1 else float("nan"),
        valid_frac=finite.mean(),
        valid_med=float(np.median(valid)),
        dist_mean=float(v.mean()) if v.size else float("nan"),
        dist_max=float(v.max()) if v.size else float("nan"),
        noise_mm=float(np.median(per_angle[ok]) * 1000) if ok.any() else float("nan"),
        dt_bag=dt_bag, dt_hdr=dt_hdr, valid=valid, t=t,
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="*")
    ap.add_argument("--dir", nargs="*", help="目录(取其下的 scan.npz)")
    ap.add_argument("--out", default="compare_scans.png")
    ap.add_argument("--show", action="store_true")
    a = ap.parse_args()

    files = list(a.paths)
    for d in (a.dir or []):
        p = Path(d) / "scan.npz"
        if p.is_file():
            files.append(str(p))
        else:
            print(f"跳过 {d}:没有 scan.npz", file=sys.stderr)
    if not files:
        ap.print_help()
        return 2

    S = [stats_of(f) for f in files]

    # ---- 表 ----
    hdr = (f"{'名称':<18}{'帧数':>6}{'时长s':>8}{'频率Hz':>9}{'有效点%':>9}"
           f"{'距离均值':>10}{'最远':>8}{'底噪mm':>9}{'bag抖动ms':>11}{'hdr抖动ms':>11}")
    print(hdr)
    print("-" * len(hdr))
    for s in S:
        b = s["dt_bag"].std() if s["dt_bag"] is not None and len(s["dt_bag"]) else float("nan")
        h = s["dt_hdr"].std() if s["dt_hdr"] is not None and len(s["dt_hdr"]) else float("nan")
        print(f"{s['name']:<18}{s['frames']:>6}{s['dur']:>8.1f}{s['hz']:>9.3f}"
              f"{s['valid_frac']*100:>9.1f}{s['dist_mean']:>10.2f}{s['dist_max']:>8.2f}"
              f"{s['noise_mm']:>9.2f}{b:>11.2f}{h:>11.2f}")

    # ---- 图 ----
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    ax = axes[0]
    for s in S:
        ax.plot(s["t"], s["valid"], lw=0.9, label=s["name"])
    ax.set_xlabel("time (s)")
    ax.set_ylabel("valid pts / frame")
    ax.set_title("valid points per frame")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1]
    data = [s["dt_bag"][s["dt_bag"] < 200] for s in S]     # 截掉极端值,否则箱线图压扁
    ax.boxplot(data, tick_labels=[s["name"] for s in S], showfliers=True,
               flierprops=dict(marker=".", ms=3))
    ax.set_ylabel("frame interval, bag_time (ms)")
    ax.set_title("bag_time interval  (outliers = recorder hiccups)")
    ax.grid(alpha=0.3, axis="y")
    ax.tick_params(axis="x", labelsize=8)

    ax = axes[2]
    w = 0.35
    idx = np.arange(len(S))
    bstd = [s["dt_bag"].std() if len(s["dt_bag"]) else 0 for s in S]
    hstd = [s["dt_hdr"].std() if s["dt_hdr"] is not None and len(s["dt_hdr"]) else 0
            for s in S]
    ax.bar(idx - w / 2, bstd, w, label="bag_time", color="tab:orange")
    ax.bar(idx + w / 2, hstd, w, label="header.stamp", color="tab:blue")
    ax.set_xticks(idx)
    ax.set_xticklabels([s["name"] for s in S], fontsize=8, rotation=20)
    ax.set_ylabel("interval std (ms)")
    ax.set_title("jitter: recorder vs lidar\n(bar gap = recording artifact, not dropped frames)")
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(a.out, dpi=130, bbox_inches="tight")
    print(f"\n已写出 {a.out}")
    if a.show:
        plt.show()
    return 0


if __name__ == "__main__":
    if "--show" not in sys.argv[1:]:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt                 # noqa: E402

    sys.exit(main())
