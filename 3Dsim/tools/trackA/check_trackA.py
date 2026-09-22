#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_trackA.py v2 —— 检查"生成的文件"(trackA_cones.csv + trackA_layout.yaml)。

与旧版本质区别:旧版检查生成器内存状态;本版只读生成后的两个文件,从 YAML
独立重建边界折线(不 import 生成器),验证冻结几何被正确转换成了布局。

检查项(任务表逐条):
  1. CSV 结构:表头/有限坐标/id 格式 ^[A-Za-z][A-Za-z0-9_]*$/color∈{red,blue}
     (与 hwcart_sim track_generator/generate.py 的消费校验一致)
  2. 唯一编号(重复 id → FAIL)
  3. 重复位置(任意两锥距 < 0.05 m → FAIL;含同色与异色)
  4. 颜色与边界对应 + 边界偏离:锥到本应属色边界 ≤ 0.02 m;
     到另一色边界 > 0.10 m(挂错环当场暴露);segment 标注诚实
     (锥确在其声明段上,≤ 0.02 m)
  5. 禁放区(黄V):segment ∈ {E3,E4,I3,I4} → FAIL;锥落在黄V臂线上
     (非四肩点)→ FAIL;锥位于黄V带内部(两臂之间,离臂 > 0.03 m)→ FAIL
  5b. 黄V polygon:必须六点(外左肩→外谷→外右肩→内右肩→内谷→内左肩),
     与 shoulders/valley 字段 1e-6 对账,外/内三点各贴重建的 E3∪E4/I3∪I4
     臂线(≤0.02 m)——防四点矩形漏谷点(首版 bug)及字段系统性同错
  6. 肩点归属:四肩点各有一锥且 segment 归属相邻圆弧段(YAML shoulders 表)
  7. YAML-CSV 一致性(数量/id/坐标 1e-6/颜色/segment 全对账)
  8. 几何环自检(从 YAML 重建):20 接缝 gap(含环回绕)、有向切角(>170°
     折返 FAIL;肩22.5°/谷45°/直角90°为预期信息项)、环间不交叉、
     内环采样点全在外环内 —— 确认文件里的几何=冻结基线
  9. 间距统计:同色相邻锥(文件序=环序,黄V断口对单独列出)实际间距
     min/max/mean;<0.3 或 >2.0 视为异常 FAIL

--negtest:两个针对性负例,定向断言对应检查项失败(仅总数>0不算抓住):
  N1 黄V中段插一个锥(谷中点,segment 标 E2)→ "yellow-V no-cone zone clean" 须 FAIL
  N2 复制一个已有锥的坐标(新 id)→ "no duplicate positions" 须 FAIL
任何 FAIL 非零退出;负例未被抓住或抓错了检查项也非零退出。
"""
import argparse
import csv
import math
import os
import re
import sys
import tempfile
from math import radians, degrees, cos, sin, hypot, atan2

ID_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')
TOL_ON = 0.02      # 锥到应属边界/声明段的最大距
TOL_OTHER = 0.10   # 锥到另一色边界的最小距
TOL_DUP = 0.05     # 两锥最小距(重复位置)
TOL_SHO = 0.06     # 肩点识别半径
BAND_MARGIN = 0.03 # 带内判定离臂余量

class Rep:
    def __init__(self):
        self.fails = 0
        self.failed_names = set()   # 失败检查项名集合(负例定向断言用)
    def add(self, name, ok, detail=""):
        tag = "OK  " if ok else "FAIL"
        if not ok:
            self.fails += 1
            self.failed_names.add(name)
        print("  [%s] %-34s %s" % (tag, name, detail))
        return ok

# ---------- 几何重建(只依赖 YAML) ----------
def sample_seg(entry, step=0.05):
    """entry = {seg, line:[x0,y0,x1,y1]} 或 {seg, arc:{center,radius,angles_deg,ccw}}
    返回 (segname, polyline points)。"""
    if "line" in entry:
        x0, y0, x1, y1 = entry["line"]
        L = hypot(x1-x0, y1-y0)
        n = max(int(L/step), 1)
        return entry["seg"], [(x0+(x1-x0)*k/n, y0+(y1-y0)*k/n) for k in range(n+1)]
    a = entry["arc"]
    (cx, cy), R = a["center"], a["radius"]
    d0, d1 = a["angles_deg"]
    ccw = a.get("ccw", True)
    t0 = radians(d0)
    da = radians((d1-d0) % 360 if ccw else (d0-d1) % 360)
    n = max(int(da*R/step), 4)
    sgn = 1.0 if ccw else -1.0
    return entry["seg"], [(cx+R*cos(t0+sgn*da*k/n), cy+R*sin(t0+sgn*da*k/n))
                          for k in range(n+1)]

def seg_tangents(entry):
    """(起切向, 终切向) 单位向量。"""
    if "line" in entry:
        x0, y0, x1, y1 = entry["line"]
        L = hypot(x1-x0, y1-y0)
        return ((x1-x0)/L, (y1-y0)/L), ((x1-x0)/L, (y1-y0)/L)
    a = entry["arc"]
    d0, d1 = a["angles_deg"]
    ccw = a.get("ccw", True)
    s = 1.0 if ccw else -1.0
    t0, t1 = radians(d0), radians(d1)
    return ((-s*sin(t0), s*cos(t0)), (-s*sin(t1), s*cos(t1)))

def dist_pt_seg(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    px, py = p
    dx, dy = bx-ax, by-ay
    L2 = dx*dx+dy*dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px-ax)*dx+(py-ay)*dy)/L2))
    return hypot(px-(ax+t*dx), py-(ay+t*dy))

def dist_pt_polyline(p, pts):
    return min(dist_pt_seg(p, pts[i], pts[i+1]) for i in range(len(pts)-1))

def point_in_poly(p, poly):
    """Ray casting;poly 为闭合折线(首尾不重复)。"""
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i+1) % n]
        if (y0 > y) != (y1 > y):
            xint = (x1-x0)*(y-y0)/(y1-y0)+x0
            if x < xint:
                inside = not inside
    return inside

def seg_crossings(A, B):
    """采样折线线段交叉计数(非严格 straddle,排除全共线)——与冻结检查器同判据。"""
    def orient(p1, p2, q):
        return (p2[0]-p1[0])*(q[1]-p1[1])-(p2[1]-p1[1])*(q[0]-p1[0])
    cnt = 0
    for i in range(len(A)-1):
        a0, a1 = A[i], A[i+1]
        for j in range(len(B)-1):
            b0, b1 = B[j], B[j+1]
            s1 = orient(a0, a1, b0); s2 = orient(a0, a1, b1)
            s3 = orient(b0, b1, a0); s4 = orient(b0, b1, a1)
            if ((s1*s2 <= 0) and (s3*s4 <= 0) and not
                    (s1 == 0 and s2 == 0 and s3 == 0 and s4 == 0)):
                cnt += 1
    return cnt

def wrap180(a):
    while a <= -180:
        a += 360
    while a > 180:
        a -= 360
    return a

# ---------- 主检查 ----------
def run_checks(csv_path, yaml_path, label=""):
    rep = Rep()
    print("="*100)
    print("check_trackA v2 %s" % ("[%s]" % label if label else ""))
    print("  csv : %s" % csv_path)
    print("  yaml: %s" % yaml_path)

    # --- 载入 ---
    import yaml
    with open(yaml_path, encoding='utf-8') as f:
        Y = yaml.safe_load(f)
    with open(csv_path, encoding='utf-8', newline='') as f:
        rows = list(csv.DictReader(f))
    cones = []
    parse_err = []
    for i, r in enumerate(rows, start=2):
        try:
            cones.append({"id": r["id"].strip(), "color": r["color"].strip(),
                          "x": float(r["x"]), "y": float(r["y"]),
                          "segment": (r.get("segment") or "").strip()})
        except Exception as e:
            parse_err.append("line %d: %s" % (i, e))

    # 1 结构
    print("-- 1) CSV 结构(有限坐标/id正则/color)")
    ok = rep.add("rows parseable", not parse_err, "; ".join(parse_err[:3]))
    bad = [c["id"] for c in cones if not (math.isfinite(c["x"]) and math.isfinite(c["y"]))]
    rep.add("finite coordinates", not bad, "non-finite: %s" % bad[:5])
    bad = [c["id"] for c in cones if not ID_RE.match(c["id"])]
    rep.add("id format ^[A-Za-z][A-Za-z0-9_]*$", not bad, "bad: %s" % bad[:5])
    bad = sorted({c["color"] for c in cones} - {"red", "blue"})
    rep.add("color in {red,blue}", not bad, "bad: %s" % bad)

    # 2 唯一编号
    print("-- 2) 唯一编号")
    ids = [c["id"] for c in cones]
    dups = sorted({i for i in ids if ids.count(i) > 1})
    rep.add("unique ids (%d cones)" % len(cones), not dups, "duplicated: %s" % dups)

    # 3 重复位置
    print("-- 3) 重复位置(任意两锥距 < %.2f m)" % TOL_DUP)
    dup_pairs = []
    for i in range(len(cones)):
        for j in range(i+1, len(cones)):
            d = hypot(cones[i]["x"]-cones[j]["x"], cones[i]["y"]-cones[j]["y"])
            if d < TOL_DUP:
                dup_pairs.append("%s~%s d=%.6f" % (cones[i]["id"], cones[j]["id"], d))
    rep.add("no duplicate positions", not dup_pairs, "; ".join(dup_pairs[:4]))

    # --- 边界重建 ---
    BO = [sample_seg(e) for e in Y["boundaries"]["outer"]]
    BI = [sample_seg(e) for e in Y["boundaries"]["inner"]]
    EO = [e for e in Y["boundaries"]["outer"]]
    EI = [e for e in Y["boundaries"]["inner"]]
    ring = {"blue": (BO, EO, "outer"), "red": (BI, EI, "inner")}
    flat = {c: [p for _, pts in ring[c][0] for p in pts] for c in ("blue", "red")}
    segmap = {c: {name: pts for name, pts in ring[c][0]} for c in ("blue", "red")}

    # 4 颜色对应 + 偏离 + segment 诚实
    print("-- 4) 颜色与边界对应 / 边界偏离(本色<=%.2f m,他色>%.2f m)/segment 诚实" %
          (TOL_ON, TOL_OTHER))
    dev_bad, seg_bad = [], []
    for c in cones:
        own = flat[c["color"]]
        other = flat["blue" if c["color"] == "red" else "red"]
        d_own = dist_pt_polyline((c["x"], c["y"]), own)
        d_other = dist_pt_polyline((c["x"], c["y"]), other)
        if d_own > TOL_ON:
            dev_bad.append("%s d_own=%.3f" % (c["id"], d_own))
        if d_other <= TOL_OTHER:
            dev_bad.append("%s on-wrong-ring d_other=%.3f" % (c["id"], d_other))
        sp = segmap[c["color"]].get(c["segment"])
        if sp is None:
            seg_bad.append("%s seg '%s' unknown" % (c["id"], c["segment"]))
        elif dist_pt_polyline((c["x"], c["y"]), sp) > TOL_ON:
            seg_bad.append("%s not on declared seg %s" % (c["id"], c["segment"]))
    rep.add("cone on its own-color boundary", not dev_bad, "; ".join(dev_bad[:4]))
    rep.add("segment attribute honest", not seg_bad, "; ".join(seg_bad[:4]))

    # 5 禁放区(黄V)
    print("-- 5) 禁放区:黄V四段无锥(段标注/臂线上/带内部)")
    band = Y["yellow_v_band"]
    no_segs = set(band["no_cone_segments"])
    shs = {k: tuple(v) for k, v in band["shoulders"].items()}
    shoulder_pts = list(shs.values())
    # 四条臂各自独立求距——绝不能拼接成一条折线:段间会拼出贯穿带内的虚构弦,
    # 注入点可能恰好落上(首版即此 bug:FAIL 结论对、理由错,探针复算抓出)
    arm_polys = [segmap["blue" if s.startswith("E") else "red"][s]
                 for s in sorted(no_segs)]
    # 带解析上下沿(由肩点与谷点插值;臂为直线段)
    xL = min(p[0] for p in shoulder_pts)
    xR = max(p[0] for p in shoulder_pts)
    vo = tuple(band["valley_outer"]); vi = tuple(band["valley_inner"])
    def arm_y(sh_y, val_y, x):
        xv = (xL+xR)/2
        t = min(max((x-xL)/(xv-xL), 0.0), 1.0) if x <= xv else \
            min(max((xR-x)/(xR-xv), 0.0), 1.0)
        return sh_y+(val_y-sh_y)*t
    zone_bad = []
    for c in cones:
        p = (c["x"], c["y"])
        if c["segment"] in no_segs:
            zone_bad.append("%s segment=%s (黄V段)" % (c["id"], c["segment"]))
            continue
        d_arm = min(dist_pt_polyline(p, poly) for poly in arm_polys)
        near_sh = any(hypot(p[0]-s[0], p[1]-s[1]) <= TOL_SHO for s in shoulder_pts)
        if d_arm <= TOL_ON and not near_sh:
            zone_bad.append("%s on V-arm d=%.3f" % (c["id"], d_arm))
            continue
        if xL < p[0] < xR:
            y_hi = arm_y(shs["E2"][1], vo[1], p[0])       # 外臂(蓝肩高)
            y_lo = arm_y(shs["I2"][1], vi[1], p[0])       # 内臂(红肩高)
            if y_lo+BAND_MARGIN <= p[1] <= y_hi-BAND_MARGIN:
                zone_bad.append("%s inside V-band (%.3f,%.3f)" % (c["id"], p[0], p[1]))
    rep.add("yellow-V no-cone zone clean", not zone_bad, "; ".join(zone_bad[:4]))

    # 5b 黄V polygon 与四条臂一致(首版四点矩形漏谷点 bug 的回归检查)
    # 六点序:外左肩→外谷→外右肩→内右肩→内谷→内左肩(闭合)。
    # 三重判据:点数=6;与 shoulders/valley 字段 1e-6 对账(字段间自洽);
    # 外/内三点各自贴从 boundaries 独立重建的 E3∪E4 / I3∪I4 臂线(≤TOL_ON,
    # 抓"polygon 与 shoulders 一起错"的系统性偏移——字段对账抓不住这种情况)
    print("-- 5b) yellow-V polygon 六点与四条臂/肩谷字段一致")
    poly_bad = []
    poly = band.get("polygon") or []
    if len(poly) != 6:
        poly_bad.append("polygon has %d pts, expect 6 "
                        "(外左肩→外谷→外右肩→内右肩→内谷→内左肩)" % len(poly))
    else:
        pts6 = [tuple(p) for p in poly]
        for nm, got, ref in (("外左肩=E5肩", pts6[0], shs.get("E5")),
                             ("外谷=valley_outer", pts6[1], vo),
                             ("外右肩=E2肩", pts6[2], shs.get("E2")),
                             ("内右肩=I2肩", pts6[3], shs.get("I2")),
                             ("内谷=valley_inner", pts6[4], vi),
                             ("内左肩=I5肩", pts6[5], shs.get("I5"))):
            if ref is None or hypot(got[0]-ref[0], got[1]-ref[1]) > 1e-6:
                poly_bad.append("%s: polygon %s != field %s" % (nm, got, ref))
        arms_o = [segmap["blue"][s] for s in ("E3", "E4")]
        arms_i = [segmap["red"][s] for s in ("I3", "I4")]
        for nm, p, arms in (("外左肩", pts6[0], arms_o), ("外谷", pts6[1], arms_o),
                            ("外右肩", pts6[2], arms_o), ("内右肩", pts6[3], arms_i),
                            ("内谷", pts6[4], arms_i), ("内左肩", pts6[5], arms_i)):
            dmin = min(dist_pt_polyline(p, pl) for pl in arms)
            if dmin > TOL_ON:
                poly_bad.append("%s off arm %.6f m" % (nm, dmin))
    rep.add("yellow-V polygon matches arms", not poly_bad, "; ".join(poly_bad[:4]))

    # 6 肩点归属
    print("-- 6) 肩点锥归属相邻圆弧段")
    sho_bad = []
    for owner, pt in shs.items():
        hit = [c for c in cones if hypot(c["x"]-pt[0], c["y"]-pt[1]) <= TOL_SHO]
        if not any(c["segment"] == owner for c in hit):
            sho_bad.append("%s: no cone owned by %s (found %s)" %
                           (tuple(round(v, 3) for v in pt), owner,
                            [c["id"]+"/"+c["segment"] for c in hit] or "none"))
    rep.add("shoulder cones owned by arcs", not sho_bad, "; ".join(sho_bad[:4]))

    # 7 YAML-CSV 一致
    print("-- 7) YAML-CSV 一致性")
    yc = Y.get("cones", [])
    mismatch = []
    if len(yc) != len(cones):
        mismatch.append("count yaml=%d csv=%d" % (len(yc), len(cones)))
    for a, b in zip(yc, cones):
        if (a["id"] != b["id"] or a["color"] != b["color"] or
                abs(a["x"]-b["x"]) > 1e-6 or abs(a["y"]-b["y"]) > 1e-6 or
                a.get("segment") != b["segment"]):
            mismatch.append("row %s/%s" % (a["id"], b["id"]))
    rep.add("yaml cones == csv cones", not mismatch, "; ".join(mismatch[:4]))

    # 8 几何环自检(从 YAML 重建)
    print("-- 8) 几何环自检(YAML 重建;确认=冻结基线)")
    for label, entries, polys in (("outer", EO, BO), ("inner", EI, BI)):
        n = len(entries)
        starts = [pts[0] for _, pts in polys]            # 各段首点(含回绕)
        gap_bad = []
        for i in range(n):
            j = (i+1) % n
            end_i = polys[i][1][-1]                      # 段 i 末点
            g = hypot(end_i[0]-starts[j][0], end_i[1]-starts[j][1])
            if g > 0.05:
                gap_bad.append("%s->%s %.4f" % (entries[i]["seg"], entries[j]["seg"], g))
        rep.add("%s ring: %d joints gap<=0.05" % (label, n), not gap_bad,
                "; ".join(gap_bad[:3]))
        breaks = []
        for i in range(n):
            nxt = entries[(i+1) % n]
            t1 = seg_tangents(entries[i])[1]
            t2 = seg_tangents(nxt)[0]
            a = abs(wrap180(degrees(atan2(t2[1], t2[0])-atan2(t1[1], t1[0]))))
            if a > 170:
                breaks.append("%s->%s %.1f BACKTRACK" % (entries[i]["seg"], nxt["seg"], a))
        rep.add("%s ring: no backtrack (>170deg)" % label, not breaks,
                "; ".join(breaks[:3]))
    cross_bad = []
    def bbox(pts):
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        return min(xs), min(ys), max(xs), max(ys)
    bbo = [bbox(pts) for _, pts in BO]
    bbi = [bbox(pts) for _, pts in BI]
    for i, (_, A) in enumerate(BO):
        for j, (_, B) in enumerate(BI):
            ax0, ay0, ax1, ay1 = bbo[i]
            bx0, by0, bx1, by1 = bbi[j]
            if ax1 < bx0 or bx1 < ax0 or ay1 < by0 or by1 < ay0:
                continue          # 包围盒不相交,必无交叉
            k = seg_crossings(A, B)
            if k:
                cross_bad.append("outer %s x inner %s x%d" % (BO[i][0], BI[j][0], k))
    rep.add("rings do not cross", not cross_bad, "; ".join(cross_bad[:3]))
    outer_poly = [p for _, pts in BO for p in pts]
    inner_pts = [p for _, pts in BI for p in pts][::4]
    nout = sum(0 if point_in_poly(p, outer_poly) else 1 for p in inner_pts)
    rep.add("inner ring inside outer", nout == 0,
            "%d/%d sampled points outside" % (nout, len(inner_pts)))

    # 9 间距统计
    print("-- 9) 间距统计(仿真假设值无官方定值;黄V断口单列)")
    full_order = {"blue": [e["seg"] for e in EO], "red": [e["seg"] for e in EI]}
    for color in ("blue", "red"):
        pts = [c for c in cones if c["color"] == color]
        idx = {s: k for k, s in enumerate(full_order[color])}
        nring = len(idx)
        spac, vbreak = [], []
        seq = pts + [pts[0]]     # 末位=首锥,凑出环回绕对
        for k in range(len(seq)-1):
            a, b = seq[k], seq[k+1]
            d = hypot(b["x"]-a["x"], b["y"]-a["y"])
            adjacent = (b["segment"] == a["segment"] or
                        idx[b["segment"]] == (idx[a["segment"]]+1) % nring)
            if k == len(seq)-2 or adjacent:      # 环回绕对按间距计
                spac.append(d)
            else:
                vbreak.append("%s(%s)->%s(%s) %.3f m" %
                              (a["id"], a["segment"], b["id"], b["segment"], d))
        if spac:
            mn, mx = min(spac), max(spac)
            mean = sum(spac)/len(spac)
            rep.add("%s spacing %.3f..%.3f mean %.3f (n=%d)" %
                    (color, mn, mx, mean, len(spac)),
                    mn >= 0.3 and mx <= 2.0, "out of sane range 0.3-2.0")
        rep.add("%s yellow-V break: exactly 1" % color, len(vbreak) == 1,
                "; ".join(vbreak) if vbreak else "missing?")

    print("--\nTOTAL FAIL(s): %d" % rep.fails)
    return rep.fails, rep.failed_names

# ---------- 负例 ----------
def negtest(csv_path, yaml_path):
    with open(csv_path, encoding='utf-8', newline='') as f:
        raw = f.read()
    lines = raw.splitlines()
    hdr, body = lines[0], lines[1:]
    tmpdir = tempfile.mkdtemp(prefix="trackA_neg_")
    misses = 0
    # N1: 黄V中段(谷中点,两臂之间)插锥——段标注故意给非黄V段 E2,
    # 迫使"带内部"纯几何判据(而非仅段标注自曝)参与抓捕。
    # 定向断言:必须恰是"禁放区"检查项失败(仅靠别项失败凑数不算抓住)
    p1 = os.path.join(tmpdir, "neg1.csv")
    with open(p1, 'w', encoding='utf-8', newline='') as f:
        f.write("\n".join([hdr]+body+["bx01,blue,11.228427,3.581266,E2"])+"\n")
    f1, n1 = run_checks(p1, yaml_path, label="NEG-1 cone in yellow-V middle")
    caught1 = f1 > 0 and "yellow-V no-cone zone clean" in n1
    print("  NEG-1 %s (fails=%d, 'yellow-V no-cone zone clean' failed: %s)\n" %
          ("CAUGHT" if caught1 else "MISSED!!", f1,
           "yes" if "yellow-V no-cone zone clean" in n1 else "no"))
    misses += (1 if not caught1 else 0)
    # N2: 复制已有锥坐标(取第一个蓝锥),新 id。定向断言:"重复位置"检查项失败
    first = body[0].split(',')
    p2 = os.path.join(tmpdir, "neg2.csv")
    with open(p2, 'w', encoding='utf-8', newline='') as f:
        f.write("\n".join([hdr] + body +
                          ["bx02,%s,%s,%s,%s" % (first[1], first[2], first[3], first[4])]) + "\n")
    f2, n2 = run_checks(p2, yaml_path, label="NEG-2 duplicated cone coords")
    caught2 = f2 > 0 and "no duplicate positions" in n2
    print("  NEG-2 %s (fails=%d, 'no duplicate positions' failed: %s)\n" %
          ("CAUGHT" if caught2 else "MISSED!!", f2,
           "yes" if "no duplicate positions" in n2 else "no"))
    misses += (1 if not caught2 else 0)
    return misses

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=None)
    ap.add_argument("--yaml", default=None)
    ap.add_argument("--negtest", action="store_true",
                    help="注入两个负例(黄V插锥/复制坐标)并断言被抓")
    a = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    docs = os.path.normpath(os.path.join(here, '..', '..', 'docs', 'track'))
    csv_path = a.csv or os.path.join(docs, "trackA_cones.csv")
    yaml_path = a.yaml or os.path.join(docs, "trackA_layout.yaml")
    fails, _names = run_checks(csv_path, yaml_path)
    extra = 0
    if a.negtest:
        extra = negtest(csv_path, yaml_path)
    if fails > 0 or extra > 0:
        print("EXIT: nonzero (FAIL present)" if fails else
              "EXIT: nonzero (negative case MISSED)")
        sys.exit(1)
    print("EXIT: 0")
    sys.exit(0)

if __name__ == "__main__":
    main()
