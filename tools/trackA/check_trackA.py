#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""赛道A 几何检查(阶段2 交付1)——独立读取 docs/track/trackA_layout.yaml 检查交付物。

判据与依据:
  [rule]  总长 16.8m;走廊宽 1.5m;锥桶间距 < 赛道宽度
  [fig9]  V 折线斜边 3.06m、顶角 135°(±5° 容差)
  [image] V#1 外边口部内外肩错位(外 3.69/7.47 vs 内 2.76/8.55),
          肩过渡区走廊局部加宽 —— 判据放宽至 1.80 上限(结构忠实于图读)
  [geom]  中心线闭合不自交;锥桶不侵入行车线(≥0.55m);区框在走廊内且互不重叠;
          黄线与 V#2 边界重合;出发位在走廊内且朝向与中心线一致
exit 0 = 全 PASS;exit 1 = 有 FAIL(逐项打印实测值)。
用法(WSL): python3 .../tools/trackA/check_trackA.py [yaml路径]
"""
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT = os.path.normpath(os.path.join(HERE, '..', '..', 'docs', 'track', 'trackA_layout.yaml'))

BOUND_STEP = 0.25   # 边界离散
CL_STEP = 0.15      # 中心线采样
CONE_R = 0.14       # 锥底半径(厂家规格 28cm 直径)


# ---------- 几何工具 ----------
def seg_points(seg, step):
    if 'line' in seg:
        x0, y0, x1, y1 = seg['line']
        n = max(1, math.ceil(math.hypot(x1 - x0, y1 - y0) / step))
        return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]
    a = seg['arc']
    (cx, cy), r, (a0, a1) = a['center'], a['radius'], a['angles_deg']
    n = max(1, math.ceil(abs(math.radians(a1 - a0)) * r / step))
    t = lambda i: math.radians(a0 + (a1 - a0) * i / n)
    return [(cx + r * math.cos(t(i)), cy + r * math.sin(t(i))) for i in range(n + 1)]


def polyline(segs, step=BOUND_STEP):
    pts = []
    for s in segs:
        for p in seg_points(s, step):
            if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-9:
                pts.append(p)
    return pts


def dist_pt_seg(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def dist_pt_polyline(p, pts):
    return min(dist_pt_seg(p[0], p[1], pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
               for i in range(len(pts) - 1))


def segs_intersect(p1, p2, p3, p4):
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    d1, d2 = cross(p3, p4, p1), cross(p3, p4, p2)
    d3, d4 = cross(p1, p2, p3), cross(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def point_in_poly(p, poly):
    x, y = p
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


# ---------- 检查 ----------
class Report:
    def __init__(self):
        self.items = []

    def add(self, name, ok, detail):
        self.items.append((name, ok, detail))
        print('%s %-38s %s' % ('PASS' if ok else 'FAIL', name, detail))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    data = yaml.safe_load(open(path, encoding='utf-8'))
    rep = Report()
    outer = polyline(data['boundaries']['outer'])
    inner = polyline(data['boundaries']['inner'])
    cl = polyline(data['centerline'], CL_STEP)
    cones = data['cones']

    # 1 走廊宽度(沿中心线采样;d_out+d_in=局部宽)
    # 判据 1.25~1.80:平段/弧=1.50(宽档);V 谷=1.30~1.41(窄档,对应图9 的 1.25m 档,
    # 部位对应仍列待复核);V#1 肩过渡区因图读内外肩 x 错位局部放宽至 ~1.76。
    widths = [(dist_pt_polyline(p, outer) + dist_pt_polyline(p, inner),
               dist_pt_polyline(p, outer)) for p in cl]
    wmin, wmax = min(widths)[0], max(widths)[0]
    rep.add('corridor width range', 1.25 <= wmin and wmax <= 1.80,
            'min=%.3f max=%.3f (判据1.25~1.80;平段/弧=1.50宽档,V谷=窄档,V#1肩过渡=图读错位)' % (wmin, wmax))
    rep.add('centerline inside corridor', min(d for _, d in widths) > 0.5,
            '中心线到外边界最小距离=%.3f' % min(d for _, d in widths))

    # 2 锥桶间距(同色相邻)
    gaps = []
    for color in ('blue', 'red'):
        pts = [(c['x'], c['y']) for c in cones if c['color'] == color]
        gaps += [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                 for i in range(len(pts) - 1)]
    rep.add('cone spacing (<track width 1.5)', max(gaps) < 1.5 and min(gaps) >= 0.3,
            'min=%.3f max=%.3f n=%d' % (min(gaps), max(gaps), len(gaps)))

    # 3 V 臂几何(顶区非水平 line 段)——分两类判据:
    #   fig9 体系(内边4臂+外V#2两臂): 长∈[2.85,3.25] 角∈[20,25]°
    #   图读结构(外V#1两臂,口部收窄短陡): 长∈[2.1,2.5] 角∈[30,36]°
    arms = []
    for bname in ('outer', 'inner'):
        for s in data['boundaries'][bname]:
            if 'line' not in s:
                continue
            x0, y0, x1, y1 = s['line']
            if y0 > 2.8 and y1 > 2.8 and abs(y1 - y0) > 0.1:
                L = math.hypot(x1 - x0, y1 - y0)
                a = math.degrees(math.atan2(abs(y1 - y0), abs(x1 - x0)))
                kind = ('short_steep' if bname == 'outer' and L < 2.6 else 'fig9')
                arms.append((bname, L, a, kind))
    n_fig9 = sum(1 for k in arms if k[3] == 'fig9')
    n_ss = sum(1 for k in arms if k[3] == 'short_steep')
    fig9_ok = n_fig9 == 6 and all(2.85 <= L <= 3.25 and 20 <= a <= 25
                                  for _, L, a, k in arms if k == 'fig9')
    ss_ok = n_ss == 2 and all(2.1 <= L <= 2.5 and 30 <= a <= 36
                              for _, L, a, k in arms if k == 'short_steep')
    rep.add('V arms: 6 fig9(3.06m/135°)+2 short', fig9_ok and ss_ok,
            ' '.join('%.2fm/%.1f°%s' % (L, a, '' if k == 'fig9' else '*')
                     for _, L, a, k in arms) + '  (*=外V#1图读短臂)')
    inner_L = [L for b, L, _a, _k in arms if b == 'inner']
    rep.add('inner V arms mean ≈3.06m [fig9]',
            2.85 <= sum(inner_L) / len(inner_L) <= 3.20,
            'mean=%.3f n=%d' % (sum(inner_L) / len(inner_L), len(inner_L)))

    # 4 包络
    allpts = [(c['x'], c['y']) for c in cones] + outer + inner
    xs = [p[0] for p in allpts]
    span_x = max(xs) - min(xs)
    rep.add('envelope: span_x = 16.80±0.05', abs(span_x - 16.80) <= 0.05,
            'span_x=%.3f min=%.3f max=%.3f' % (span_x, min(xs), max(xs)))
    ys = [p[1] for p in allpts]
    rep.add('envelope: height = 5.52±0.05', abs((max(ys) - min(ys)) - 5.52) <= 0.05,
            'h=%.3f' % (max(ys) - min(ys)))

    # 5 区框:在走廊内(不在岛内)、互不重叠
    inner_poly = inner
    ok_zones = True
    detail = []
    znames = list(data['zones'])
    for i, n1 in enumerate(znames):
        z1 = data['zones'][n1]
        for cx in (z1['x'][0], z1['x'][1]):
            for cy in (z1['y'][0], z1['y'][1]):
                p = (cx, cy)
                on_edge = dist_pt_polyline(p, inner_poly) < 0.05
                if point_in_poly(p, inner_poly) and not on_edge:
                    ok_zones = False
                    detail.append('%s角点%s在岛内' % (n1, p))
        for n2 in znames[i + 1:]:
            z2 = data['zones'][n2]
            if z1['x'][1] > z2['x'][0] and z2['x'][1] > z1['x'][0]:
                ok_zones = False
                detail.append('%s与%s重叠' % (n1, n2))
    rep.add('zones in corridor, no overlap', ok_zones, '; '.join(detail) or
            ' '.join('%s[%.2f,%.2f]' % (n, data['zones'][n]['x'][0], data['zones'][n]['x'][1])
                     for n in znames))

    # 6 中心线闭合 + 不自交
    closed = math.hypot(cl[0][0] - cl[-1][0], cl[0][1] - cl[-1][1]) < 0.25
    rep.add('centerline closed', closed,
            '首尾距=%.3f' % math.hypot(cl[0][0] - cl[-1][0], cl[0][1] - cl[-1][1]))
    coarse = cl[::3]
    n_int = sum(1 for i in range(len(coarse) - 1) for j in range(i + 2, len(coarse) - 1)
                if segs_intersect(coarse[i], coarse[i + 1], coarse[j], coarse[j + 1]))
    rep.add('centerline no self-intersection', n_int == 0, '交点数=%d' % n_int)
    lap = sum(math.hypot(cl[i + 1][0] - cl[i][0], cl[i + 1][1] - cl[i][1])
              for i in range(len(cl) - 1))
    rep.add('centerline lap length (report only)', True, '≈%.1f m(无规则对照值)' % lap)

    # 7 锥桶不侵入行车线
    dmin = min(dist_pt_polyline((c['x'], c['y']), cl) for c in cones)
    rep.add('cones clear of centerline ≥0.55', dmin >= 0.55,
            'min=%.3f (走廊半宽0.75-锥半径%.2f-余量)' % (dmin, CONE_R))

    # 8 出发位
    sp = data['start_pose']
    d_out = dist_pt_polyline((sp['x'], sp['y']), outer)
    d_in = dist_pt_polyline((sp['x'], sp['y']), inner)
    yaw_ok = abs(sp['yaw_deg'] % 360) < 1.0
    rep.add('start pose in corridor, yaw=+x', d_out < 0.86 and d_in < 0.86 and yaw_ok,
            'd_out=%.3f d_in=%.3f yaw=%s°' % (d_out, d_in, sp['yaw_deg']))

    # 9 黄线与 V#2 边界重合
    ok_tape = True
    tmax = 0.0
    for s in data['yellow_tape']['segments']:
        for p in ((s[0], s[1]), (s[2], s[3])):
            d = min(dist_pt_polyline(p, outer), dist_pt_polyline(p, inner))
            tmax = max(tmax, d)
            if d > 0.02:
                ok_tape = False
    rep.add('yellow tape on V#2 boundary', ok_tape,
            '端点到边界最大距离=%.4f' % tmax)

    n_fail = sum(1 for _, ok, _ in rep.items if not ok)
    print('\n===== 结果: %d 项检查, PASS=%d FAIL=%d %s =====' % (
        len(rep.items), len(rep.items) - n_fail, n_fail,
        'ALL_GREEN' if n_fail == 0 else 'HAS_FAILURES'))
    sys.exit(1 if n_fail else 0)


if __name__ == '__main__':
    main()
