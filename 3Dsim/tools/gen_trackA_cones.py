#!/usr/bin/env python3
"""按给定间距沿赛道 A 的边界重新排锥桶,输出可直接喂 track_generator 的 CSV。

替代原来的 fill_yellow_v.py(那个只补黄 V 那四段)。几何单源是
trackA_layout.yaml 的 `boundaries` —— 十段外边界 + 十段内边界,直线给端点,
圆弧给圆心/半径/起止角。锥桶按弧长等分排,不做拟合。

## 间距为什么可配

layout 里 `cone_spacing: {value: 1.000, source: 'asmp: 无官方定值,可配置'}` ——
作者自己标了这是假设,不是官方值。`fastsim/params.py` 那边又写 1.15
(`[图1量得]`)。两处差 15%,都没有一手依据。

2026-09-22 用户看比赛录像:没有挡板的地方锥桶几乎是贴着的,要求改到 0.5m 左右,
弯里还要更密。所以这里默认 --spacing 0.5 / --curve-spacing 0.4,但值仍然是待核实
的,真依据见 issue「Verify cone spacing against the real track」。

## 黄 V 段

E3/E4/I3/I4 是黄 V,实际比赛铺**黄色胶带**不放锥桶(layout 的
`yellow_v_band.no_cone_segments` 明确列了)。默认照规矩留空;训练时想让边界连续
可以 --fill-v 把它也排上,但那样撞锥判定会比真实赛道严 —— 真车压胶带不算撞。

用法:
    python3 3Dsim/tools/gen_trackA_cones.py trackA_layout.yaml -o cones.csv
    python3 3Dsim/tools/gen_trackA_cones.py trackA_layout.yaml -o cones.csv \
        --spacing 0.5 --curve-spacing 0.4 --fill-v
"""
import argparse
import csv
import math
from pathlib import Path

import yaml

V_SEGMENTS = ('E3', 'E4', 'I3', 'I4')
SAME_POINT = 0.02          # 相邻段共用端点,离这么近就当同一个桶


def arc_span(a0, a1, ccw):
    """起止角(度)按给定方向的跨度,弧度,恒正。"""
    d = math.radians(a1 - a0) if ccw else math.radians(a0 - a1)
    return d % (2 * math.pi) or 2 * math.pi


def segment_points(item, spacing):
    """一段边界上等距排点,含两个端点。"""
    if 'line' in item:
        x0, y0, x1, y1 = item['line']
        length = math.hypot(x1 - x0, y1 - y0)
        n = max(1, round(length / spacing))
        return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)], length
    arc = item['arc']
    cx, cy = arc['center']
    r = arc['radius']
    a0, a1 = arc['angles_deg']
    ccw = bool(arc['ccw'])
    span = arc_span(a0, a1, ccw)
    length = r * span
    n = max(1, round(length / spacing))
    out = []
    for i in range(n + 1):
        a = math.radians(a0) + (span * i / n) * (1 if ccw else -1)
        out.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return out, length


def boundary_cones(items, spacing, curve_spacing, fill_v):
    placed = []
    for item in items:
        seg = item['seg']
        if seg in V_SEGMENTS and not fill_v:
            continue
        step = curve_spacing if 'arc' in item else spacing
        pts, _ = segment_points(item, step)
        for x, y in pts:
            # 相邻段共用端点;黄V留空时两个肩点也别被隔壁段重复放
            if any(math.hypot(x - px, y - py) < SAME_POINT for px, py in placed):
                continue
            placed.append((x, y))
            yield seg, x, y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('layout', type=Path)
    ap.add_argument('-o', '--output', required=True, type=Path)
    ap.add_argument('--spacing', type=float, default=0.5, help='直线段间距 m')
    ap.add_argument('--curve-spacing', type=float, default=0.4, help='圆弧段间距 m')
    ap.add_argument('--fill-v', action='store_true', help='黄V段也排锥桶(默认留空)')
    args = ap.parse_args()

    layout = yaml.safe_load(args.layout.read_text(encoding='utf-8'))
    rows = []
    for side, color, prefix in (('outer', 'blue', 'b'), ('inner', 'red', 'r')):
        n = 0
        for seg, x, y in boundary_cones(layout['boundaries'][side], args.spacing,
                                        args.curve_spacing, args.fill_v):
            n += 1
            rows.append({'id': f'{prefix}{n:02d}', 'color': color,
                         'x': f'{x:.6f}', 'y': f'{y:.6f}', 'segment': seg})

    with args.output.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=['id', 'color', 'x', 'y', 'segment'])
        w.writeheader()
        w.writerows(rows)

    blue = sum(1 for r in rows if r['color'] == 'blue')
    gaps = []
    for color in ('blue', 'red'):
        p = [r for r in rows if r['color'] == color]
        gaps += [math.hypot(float(a['x']) - float(b['x']), float(a['y']) - float(b['y']))
                 for a, b in zip(p, p[1:])]
    print(f'{args.layout} -> {args.output}')
    print(f'  {len(rows)} 个锥桶(蓝 {blue} / 红 {len(rows) - blue}),'
          f'黄V {"已排" if args.fill_v else "留空"}')
    print(f'  相邻间距 {min(gaps):.3f}~{max(gaps):.3f} m '
          f'(目标 直线 {args.spacing} / 弧 {args.curve_spacing})')


if __name__ == '__main__':
    main()
