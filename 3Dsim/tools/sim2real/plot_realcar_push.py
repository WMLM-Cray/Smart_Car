# -*- coding: utf-8 -*-
"""实车推车历史数据对照图 (sim2real 首轮交付物③)。

只读复用既有取证脚本的解析结论, 不重推协议、不改 segment 窗口:
  位姿寄存器 = resp_d 帧的 b[3]<<8|b[4] (16位), 每 1024 计数回绕一次轮;
  4 段推车窗口(相对首帧 t0, 秒): 前进1 31.9-37.3 / 前进2 42.0-47.3 /
  前进3 50.74-58.97(异常段, 真实窗口含 55.4-56.9s 停顿) / 后退1 64.1-71.4,
  卷尺距离 100/100/80/100 cm。

口径说明(诚实):
  - 推车 = 无动力人工推, 只能证明 编码器计数↔卷尺距离 的行程积分一致性;
    不能证明 动态响应/制动/最小可控速度。
  - mm/计数 = 卷尺距离(mm) / |Δ计数|(误差分母); 代码基准值 = 现役速度公式完整系数链
    `3.57×0.25×1.03 × 227.45/1000 / 1024` = 0.919275×0.22745/1024 = **0.2042** mm/计数,
    **非** C/1024=0.2221(漏 0.919275 系数为误算, 见 STAGE_A_TAP_PROTOCOL.md §6 纠错)。
    三段(前进1/2/后退1)实测 0.2076/0.2064/0.2018 vs 0.2042 → +1.7%/+1.1%/-1.2% 吻合。
  - 前进3 保留异常段: 真实窗口 50.74-58.97s Δ=4242 → 800/4242=0.1886 mm/计数, -7.6%,
    即编码器计数对应 86.6cm > 卷尺 80cm 约 6.6cm(系统性多计), 根因离线无法判定,
    列为下次现场独立复测项, 不参与三段比例吻合依据。

运行: python tools/sim2real/plot_realcar_push.py [--csv <push.csv> --out <png>]
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[2]
DEFAULT_CSV = REPO / "docs/sim2real/data/tap_push_20260918.csv"
DEFAULT_OUT = REPO / "docs/sim2real/out/realcar_push_compare.png"

# 既证段窗口: (名, 起秒, 止秒, 卷尺cm, 颜色)
# 前进3 用真实原始窗口 50.74~58.97s(含 55.4~56.9s 停顿), 非硬编码 51.1~59.4s;
# 见 docs/validation/2026-09-18-nav-onepoint-round2/STAGE_A_TAP_PROTOCOL.md §6。
SEGS = [
    ("FWD1", 31.9, 37.3, 100.0, "#1a7f4b"),
    ("FWD2", 42.0, 47.3, 100.0, "#1f77b4"),
    ("FWD3*", 50.74, 58.97, 80.0, "#c44e52"),   # 异常段(真实窗口 Δ=4242)
    ("BACK1", 64.1, 71.4, 100.0, "#9467bd"),
]
# 现役代码 mm/计数(完整系数链, 非 C/1024):
#   Vel = 3.57 × counts × (227.45/1000) / 1024 / 0.02 × k × 0.25 × 1.03   [m/s]
#   每计数距离 = 0.919275 × (227.45/1024) = 0.919275 × 0.2221 mm = 0.2042 mm/计数
_CODE_FACTOR = 3.57 * 0.25 * 1.03  # 0.919275
CODE_MM_PER_CNT = _CODE_FACTOR * 227.45 / 1024.0  # mm/计数 ≈ 0.2042


def parse(path: str):
    t = []
    pos16 = []
    with open(path, newline="") as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            t.append(float(row[0]))
            p = None
            try:
                b = bytes.fromhex(row[4])  # resp_d 位姿帧
                if len(b) >= 5:
                    p = (b[3] << 8) | b[4]
            except Exception:
                p = None
            pos16.append(p)
    t = np.array(t)
    # 解回绕(1024/轮, 与 analyze_push3 一致)
    unwrap = []
    prev = None
    offset = 0
    for p in pos16:
        if p is None:
            unwrap.append(np.nan)
            continue
        if prev is not None:
            d = p - prev
            if d > 512:
                offset -= 1024
            elif d < -512:
                offset += 1024
        unwrap.append(p + offset)
        prev = p
    return t, np.array(unwrap)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(DEFAULT_CSV))
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    args = ap.parse_args()

    t, u = parse(args.csv)
    t0 = t[0]
    tr = t - t0

    fig, ax = plt.subplots(figsize=(11, 4.6))
    ax.plot(tr, u, lw=0.9, color="#333", label="unwrapped encoder counts (raw push)")

    for name, ts, te, cm, color in SEGS:
        ax.axvspan(ts, te, color=color, alpha=0.14, zorder=0)
        m = (tr >= ts) & (tr <= te) & ~np.isnan(u)
        us = u[m]
        if len(us) < 2:
            continue
        du = us[-1] - us[0]
        mm_per_cnt = cm * 10.0 / abs(du) if du else np.nan
        ax.annotate(
            f"{name}: du={int(du):+d} cnt, tape={cm:.0f}cm, "
            f"mm/cnt={mm_per_cnt:.4f}",
            xy=(0.5 * (ts + te), u[m].min()),
            xytext=(0.5 * (ts + te), u[m].min() - 2200),
            fontsize=7, color=color, ha="center",
            arrowprops=dict(arrowstyle="->", color=color, lw=0.6),
        )

    ax.axhline(0, color="#999", lw=0.6)
    ax.set_xlabel("time rel t0 (s)")
    ax.set_ylabel("unwrapped position counts (1024 / rev)")
    ax.set_title(
        "Real-car push test 2026-09-18: encoder counts vs tape distance  "
        f"(code mm/cnt={CODE_MM_PER_CNT:.4f} = 0.919275*227.45/1024; "
        "FWD3* = anomaly, excluded from +/-2% fit)"
    )
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print("png ->", out)


if __name__ == "__main__":
    main()