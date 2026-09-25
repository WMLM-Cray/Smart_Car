#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""赛道 A 数据生成器：默认逐个读取 2026 规则图 9A 的锥桶中心。

下面的边界段来自旧仿真几何，尚未与规则图逐桶坐标校准；不可据此
宣称赛道已通过物理尺寸或可通行性验收。--source uniform 仅作合成对照。

冻结来源:docs/track/fig9A_rebuild5.py.txt(commit 7ba1e08,验收 FAIL=0/exit 0)。
本轮任务 = 把冻结几何移入正式代码,统一参数来源;CSV、YAML、预览图全部由
本文件的同一份几何生成,不再有第二处坐标。

冻结几何要点(全部与 rebuild5 一致,勿改,改=重新走验收):
  外环/内环各 10 段,段序均为逆时针(自左下弧下切点;NOTE: 早期版本注释
  误写 clockwise,已更正——底边向右、右端向上、顶部向左,数学上逆时针)。
  右端完整 180° 半圆(外R2.75/内R1.25,圆心 14.05,2.75);底边单段。
  顶部六段相切圆弧链:LL(2.75,2.75)/V/W 三圆心,极角 90→45 / 225→315 / 135→90;
  V 处半径翻转(蓝1.25/红2.75),三圆心径向道宽均 1.5。
  顶线高度推导:蓝 5.50 / 红 4.00(与 LL 圆顶点相切)。
  黄V:肩点直接引用圆弧顶点(无占位短线),臂生成值 3.054048(标称3.06,
  残差 -0.005952 入 trackA_尺寸残差表.md);135° 顶角。

锥桶布局规则(本轮决策):
  蓝沿外环、红沿内环;黄V四段(E3/E4/I3/I4)不放锥桶;
  肩点锥桶归属相邻圆弧段(E2/I2 提供 (14.05,5.50)/(14.05,4.00),
  E5/I5 提供 (8.406854,5.50)/(8.406854,4.00));
  相邻段共享端点去重(保留首见段归属);数量不固定(默认间距下 ≈40蓝+31红)。
  正式锥桶位置来自 2026 规则图9A逐桶读图;可选的 uniform 重排只供
  对照/压力测试,不得覆盖正式赛道图的摆位。

来源标注: rule=规则文字 / dim=尺寸链定案 / frozen=冻结基线决策 /
asmp=仿真假设 / envchk=包络偏差保留核查。

输出(docs/track/):
  trackA_layout.yaml   坐标清单(params+boundaries+yellow_v_band+cones+zones)
  trackA_cones.csv     id,color,x,y,segment(前三列兼容 track_generator)
  trackA_topview.png   俯视图(锥桶+编号+黄V禁放带)

用法(WSL):
  python3 tools/trackA/trackA_geometry.py
  python3 tools/trackA/trackA_geometry.py --source uniform --spacing 0.5 --curve-spacing 0.4
"""
import argparse
import csv
import math
import os
from math import sqrt, cos, sin, pi, radians, degrees, atan2, hypot

HERE = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'docs', 'track'))

# ---------------- 冻结参数(单源;每项含来源) ----------------
LL_C = (2.75, 2.75)     # dim: 左下弧圆心(链线起点)
RE_C = (14.05, 2.75)    # dim: 右端半圆圆心
U45 = 1.0/sqrt(2)
V_C = (LL_C[0]+4.0*U45, LL_C[1]+4.0*U45)          # dim: 90°顶点=凹弧圆心(4m链精确)
W_C = (V_C[0]+4.0*U45, V_C[1]-4.0*U45)            # dim: 45°顶点=右过渡圆心(4m斜线)
R_LL_O, R_LL_I = 2.75, 1.25                        # dim: 左下同心弧对
R_RE_O, R_RE_I = 2.75, 1.25                        # dim: 右端同心弧对
R_V_O,  R_V_I  = 1.25, 2.75                        # frozen: V处半径翻转(蓝1.25/红2.75)
R_W_O,  R_W_I  = 2.75, 1.25                        # dim
TOP_O, TOP_I = 5.50, 4.00                          # frozen: 顶高推导(切LL圆顶点)
X_OUT = 16.80                                       # rule: 总宽(左右最外侧)
AA = radians(22.5)                                  # dim: 黄V臂坡(135°顶角)
SH_L_O = (W_C[0], TOP_O)                            # frozen: 蓝左肩=W圆顶点
SH_R_O = (RE_C[0], TOP_O)                           # frozen: 蓝右肩=RE半圆顶点
SH_L_I = (W_C[0], TOP_I)
SH_R_I = (RE_C[0], TOP_I)
ARM_NOM = 3.06                                      # dim: 标称臂长
ARM_GEN = (SH_R_O[0]-SH_L_O[0])/2.0/cos(AA)         # frozen: 生成臂长 3.054048
VAL_O = ((SH_L_O[0]+SH_R_O[0])/2, TOP_O-ARM_GEN*sin(AA))
VAL_I = ((SH_L_I[0]+SH_R_I[0])/2, TOP_I-ARM_GEN*sin(AA))
DV_VERT = TOP_O-TOP_I                               # 1.50 结构竖直差
DV_NORM = DV_VERT*cos(AA)                           # asmp: 法向1.386,与规则1.5关系待确认
W_DEV_NOTE = "来源待核查(不足以断言读数噪声)"          # 修正措辞: W点0.211m偏差

# 功能区按图 9A 像素横坐标估读；仅供可视化，尚无独立尺寸验收。
def fig_x(px):
    return (px - 14) * X_OUT / 1321


ZONES = {
    'zone_light_example': ((fig_x(231), fig_x(408)), 'image', '图示红绿灯区;实赛位置可变'),
    'zone_a':      ((fig_x(496), fig_x(585)), 'image', 'A 检测区黄框'),
    'zone_ab':     ((fig_x(674), fig_x(762)), 'image', 'A/B 双格黄框'),
    'zone_start':  ((fig_x(851), fig_x(940)), 'image', '发车区黄框'),
}
START_POSE = (11.0, 0.75, 0.0)   # asmp: 发车位(底道中央,朝+x,逆时针起跑)

# ---------------- 冻结段表(与 rebuild5 逐段一致) ----------------
def pol(c, R, deg):
    t = radians(deg)
    return (c[0]+R*cos(t), c[1]+R*sin(t))

def line_samples(a, b, step=0.05):
    L = hypot(b[0]-a[0], b[1]-a[1]); n = max(int(L/step), 1)
    return [tuple(a[i]+(b[i]-a[i])*k/n for i in (0, 1)) for k in range(n+1)]

def arc_samples(c, R, p0, p1, ccw=True, step=0.05):
    a0 = atan2(p0[1]-c[1], p0[0]-c[0]); a1 = atan2(p1[1]-c[1], p1[0]-c[0])
    da = ((a1-a0) % (2*pi)) if ccw else ((a0-a1) % (2*pi))
    n = max(int(da*R/step), 4)
    ts = [a0+da*k/n for k in range(n+1)] if ccw else [a0-da*k/n for k in range(n+1)]
    return [(c[0]+R*cos(t), c[1]+R*sin(t)) for t in ts]

# 外环:逆时针自左下弧下切点(clockwise 注释系早期笔误,实际逆时针)
OUTER = [
    ("E1",  "line", (LL_C[0], 0.0), (RE_C[0], 0.0), None),
    ("E2",  "arc",  (RE_C[0], 0.0), (RE_C[0], TOP_O), (RE_C, R_RE_O, -90, 90, True)),
    ("E3",  "line", SH_R_O, VAL_O, None),                      # 黄V臂:不放锥
    ("E4",  "line", VAL_O, SH_L_O, None),                      # 黄V臂:不放锥
    ("E5",  "arc",  SH_L_O, pol(W_C, R_W_O, 135), (W_C, R_W_O, 90, 135, True)),
    ("E6",  "arc",  pol(V_C, R_V_O, 315), pol(V_C, R_V_O, 225), (V_C, R_V_O, 315, 225, False)),
    ("E7",  "arc",  pol(LL_C, R_LL_O, 45), (LL_C[0], TOP_O), (LL_C, R_LL_O, 45, 90, True)),
    ("E8",  "line", (LL_C[0], TOP_O), (0.0, TOP_O), None),
    ("E9",  "line", (0.0, TOP_O), (0.0, LL_C[1]), None),
    ("E10", "arc",  (0.0, LL_C[1]), (LL_C[0], 0.0), (LL_C, R_LL_O, 180, 270, True)),
]
# 内环:逆时针自左下内弧下切点
INNER = [
    ("I1",  "line", (LL_C[0], 1.5), (RE_C[0], 1.5), None),
    ("I2",  "arc",  (RE_C[0], 1.5), (RE_C[0], TOP_I), (RE_C, R_RE_I, -90, 90, True)),
    ("I3",  "line", SH_R_I, VAL_I, None),                      # 黄V臂:不放锥
    ("I4",  "line", VAL_I, SH_L_I, None),                      # 黄V臂:不放锥
    ("I5",  "arc",  SH_L_I, pol(W_C, R_W_I, 135), (W_C, R_W_I, 90, 135, True)),
    ("I6",  "arc",  pol(V_C, R_V_I, 315), pol(V_C, R_V_I, 225), (V_C, R_V_I, 315, 225, False)),
    ("I7",  "arc",  pol(LL_C, R_LL_I, 45), (LL_C[0], TOP_I), (LL_C, R_LL_I, 45, 90, True)),
    ("I8",  "line", (LL_C[0], TOP_I), (1.5, TOP_I), None),
    ("I9",  "line", (1.5, TOP_I), (1.5, LL_C[1]), None),
    ("I10", "arc",  (1.5, LL_C[1]), (LL_C[0], 1.5), (LL_C, R_LL_I, 180, 270, True)),
]
NO_CONE_SEGS = {"E3", "E4", "I3", "I4"}

def seg_samples(seg, step=0.05):
    name, kind, p0, p1, aux = seg
    if kind == "line":
        return line_samples(p0, p1, step)
    c, R, d0, d1, ccw = aux
    return arc_samples(c, R, pol(c, R, d0), pol(c, R, d1), ccw, step)

def seg_length(seg):
    name, kind, p0, p1, aux = seg
    if kind == "line":
        return hypot(p1[0]-p0[0], p1[1]-p0[1])
    c, R, d0, d1, ccw = aux
    da = ((d1-d0) % 360) if ccw else ((d0-d1) % 360)
    return radians(da)*R

# ---------------- 锥桶生成 ----------------
def seg_points(seg, n):
    """n 个沿段均布点(含两端),直线/弧参数化直铺——不走采样步长,避免整型截断。
    弧扫角符号:幅值 = (d1-d0)%360(ccw) / (d0-d1)%360(cw),方向 ccw 为 +。
    (首版在此对已是负数的 d1-d0 再取反,E6/I6 锥被放到凹弧错误一侧,被检查器抓出)"""
    _, kind, p0, p1, aux = seg
    if kind == "line":
        return [tuple(p0[i] + (p1[i]-p0[i])*k/(n-1) for i in (0, 1)) for k in range(n)]
    c, R, d0, d1, ccw = aux
    a0 = radians(d0)
    sweep = radians((d1-d0) % 360 if ccw else (d0-d1) % 360)
    sgn = 1.0 if ccw else -1.0
    return [pol(c, R, degrees(a0 + sgn*sweep*k/(n-1))) for k in range(n)]

def gen_cones(spacing, curve_spacing=None):
    """每段 n = ceil(L/spacing)+1 个含两端均布点;黄V四段整段跳过(其肩端点由
    相邻圆弧段 E2/E5/I2/I5 提供);共享端点按 1e-6 键去重,首见段持有该锥。"""
    if curve_spacing is None:
        curve_spacing = spacing
    cones = []
    seen = {}
    for color, ring, pref in (("blue", OUTER, "b"), ("red", INNER, "r")):
        cnt = 0
        for seg in ring:
            name = seg[0]
            if name in NO_CONE_SEGS:
                continue
            L = seg_length(seg)
            step = curve_spacing if seg[1] == "arc" else spacing
            n = max(2, math.ceil(L/step) + 1)
            for p in seg_points(seg, n):
                key = (round(p[0], 6), round(p[1], 6))
                if key in seen:
                    continue
                seen[key] = name
                cnt += 1
                cones.append({"id": "%s%02d" % (pref, cnt), "color": color,
                              "x": p[0], "y": p[1], "segment": name})
    return cones

def spacing_stats(cones):
    """同色相邻锥实际间距(文件序=环序)。黄V断口对(段序跨过被跳过段)单独列出,
    不混入常规间距统计。返回 {color: (n, min, max, mean, sd, [break_strs])}。"""
    order = {"blue": [s[0] for s in OUTER], "red": [s[0] for s in INNER]}
    out = {}
    for color in ("blue", "red"):
        pts = [c for c in cones if c["color"] == color]
        idx = {s: k for k, s in enumerate(order[color])}
        nring = len(idx)
        d, breaks = [], []
        seq = pts + [pts[0]]
        for k in range(len(seq)-1):
            a, b = seq[k], seq[k+1]
            dist = hypot(b["x"]-a["x"], b["y"]-a["y"])
            adjacent = (b["segment"] == a["segment"] or
                        idx[b["segment"]] == (idx[a["segment"]]+1) % nring)
            if k == len(seq)-2 or adjacent:
                d.append(dist)
            else:
                breaks.append("%s(%s)->%s(%s) %.3f" %
                              (a["id"], a["segment"], b["id"], b["segment"], dist))
        mean = sum(d)/len(d)
        out[color] = (len(pts), min(d), max(d), mean,
                      sqrt(sum((x-mean)**2 for x in d)/len(d)), breaks)
    return out

# ---------------- 输出 ----------------
def write_yaml(path, cones, spacing, curve_spacing, source="fig9"):
    P = lambda p: "[%.6f, %.6f]" % (p[0], p[1])
    y = []
    y.append("# 赛道A 图9A逐桶坐标；边界段为未校准的旧仿真模型")
    y.append("# 锥桶坐标来自 fig9a_cones.py 对原图逐桶读点;旧边界段为未校准的模型。")
    y.append("# 坐标系: 原点=外边界左下角, x向右, y向上, 单位m, 行车逆时针")
    y.append("# 假设声明: 黄V按图示肩点与角度连接;局部法向宽度约1.386m,")
    y.append("#           与规则统一道宽1.5m的关系待确认。不宣称全程严格1.5m。")
    y.append("track: A")
    y.append("baseline: {name: legacy-sim-boundaries, commit: 7ba1e08, geometry_validation: pending}")
    y.append("frame:")
    y.append("  origin: outer_boundary_lower_left")
    y.append("  axes: {x: right, y: up}")
    y.append("  unit: m")
    y.append("  ring_order: counterclockwise")
    y.append("params:")
    y.append("  total_length:   {value: %.2f,  source: rule}" % X_OUT)
    y.append("  corridor_width: {value: 1.50,  source: dim}")
    y.append("  ll_center:      {value: %s, source: dim}" % P(LL_C))
    y.append("  re_center:      {value: %s, source: dim}" % P(RE_C))
    y.append("  v_center:       {value: %s, source: dim}" % P(V_C))
    y.append("  w_center:       {value: %s, source: dim}" % P(W_C))
    y.append("  radii_ll:  {value: [2.75, 1.25], source: dim}")
    y.append("  radii_re:  {value: [2.75, 1.25], source: dim}")
    y.append("  radii_v:   {value: [1.25, 2.75], source: 'frozen: V处翻转'}")
    y.append("  radii_w:   {value: [2.75, 1.25], source: dim}")
    y.append("  top_heights:    {value: [5.50, 4.00], source: 'frozen: 推导(切LL圆顶)'}")
    y.append("  v_arm_nominal: {value: 3.06, source: dim}")
    y.append("  v_arm_generated: {value: %.6f, source: 'frozen: 保肩点+135°,残差-0.005952'}" % ARM_GEN)
    y.append("  v_band_width: {value: {vertical: 1.50, normal: %.3f}, source: 'asmp: 待确认'}" % DV_NORM)
    y.append("  w_point_dev: {value: 0.211, source: 'envchk: %s'}" % W_DEV_NOTE)
    if source == "fig9":
        y.append("  cone_placement: {value: individual_pixel_centres, source: '2026规则图9A;fig9A_source.jpg'}")
        y.append("  cone_scale: {value: 78.630952, source: '图示外蓝锥水平跨度1321px对应16.8m'}")
    else:
        y.append("  cone_spacing: {value: %.3f, source: 'asmp: 实赛录像观察,规则仅规定<1.5m'}" % spacing)
        y.append("  cone_spacing_arc: {value: %.3f, source: 'asmp: 弯道加密,规则仅规定<1.5m'}" % curve_spacing)
    y.append("boundaries:")
    for label, ring in (("outer", OUTER), ("inner", INNER)):
        y.append("  %s:  # 逆时针,自左下弧下切点" % label)
        for name, kind, p0, p1, aux in ring:
            if kind == "line":
                y.append("    - {seg: %s, line: [%.6f, %.6f, %.6f, %.6f]}" %
                         (name, p0[0], p0[1], p1[0], p1[1]))
            else:
                c, R, d0, d1, ccw = aux
                y.append("    - {seg: %s, arc: {center: [%.6f, %.6f], radius: %.6f, "
                         "angles_deg: [%g, %g], ccw: %s}}" % (name, c[0], c[1], R, d0, d1, ccw))
    y.append("yellow_v_band:  # 禁放区(黄V中段不放锥桶)")
    # 六点带:外左肩→外谷→外右肩→内右肩→内谷→内左肩(闭合)。
    # 首版误写四肩点矩形,漏两谷点,下游直接消费 polygon 会得到错误区域(已修)。
    y.append("  polygon: [%s, %s, %s, %s, %s, %s]" %
             (P(SH_L_O), P(VAL_O), P(SH_R_O), P(SH_R_I), P(VAL_I), P(SH_L_I)))
    y.append("  no_cone_segments: [E3, E4, I3, I4]")
    y.append("  shoulders: {E2: %s, E5: %s, I2: %s, I5: %s}" %
             (P(SH_R_O), P(SH_L_O), P(SH_R_I), P(SH_L_I)))
    y.append("  valley_outer: %s" % P(VAL_O))
    y.append("  valley_inner: %s" % P(VAL_I))
    y.append("zones:")
    for n, ((x0, x1), src, desc) in ZONES.items():
        y.append("  %s: {x: [%.6f, %.6f], source: %s, note: '%s'}" % (n, x0, x1, src, desc))
    y.append("start_pose: {xy: [%.2f, %.2f], yaw_deg: %g, source: asmp}" %
             (START_POSE[0], START_POSE[1], math.degrees(START_POSE[2])))
    y.append("cones:  # %d blue + %d red,沿边界逆时针;黄V段无锥" %
             (sum(1 for c in cones if c['color'] == 'blue'),
              sum(1 for c in cones if c['color'] == 'red')))
    for c in cones:
        y.append("  - {id: %s, color: %s, x: %.6f, y: %.6f, segment: %s}" %
                 (c['id'], c['color'], c['x'], c['y'], c['segment']))
    with open(path, 'w', encoding='utf-8') as f:
        f.write("\n".join(y) + "\n")

def write_csv(path, cones):
    with open(path, 'w', encoding='utf-8', newline='') as f:
        w = csv.writer(f, lineterminator="\n")   # 仓库既有 CSV 为 LF,勿引入 CRLF
        w.writerow(["id", "color", "x", "y", "segment"])
        for c in cones:
            w.writerow([c['id'], c['color'], "%.6f" % c['x'], "%.6f" % c['y'], c['segment']])

def write_topview(path, cones):
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        print("PIL missing, skip topview")
        return
    S = 110  # px per m
    mx, my = 70, 50
    W = int(17.6*S)+2*mx
    H = int(6.6*S)+2*my
    im = Image.new("RGB", (W, H), (24, 24, 24))
    d = ImageDraw.Draw(im)
    try:
        fs = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 11)
        fb = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 14)
    except Exception:
        fs = fb = ImageFont.load_default()
    def P(x, y):
        return (mx + x*S, H - my - y*S)
    # yellow-V no-cone band(六点带,与 YAML polygon 同序:外左肩→外谷→外右肩→内右肩→内谷→内左肩)
    band = [P(*SH_L_O), P(*VAL_O), P(*SH_R_O), P(*SH_R_I), P(*VAL_I), P(*SH_L_I)]
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    od = ImageDraw.Draw(ov)
    od.polygon(band, fill=(255, 220, 60, 60))
    im = Image.alpha_composite(im.convert("RGBA"), ov).convert("RGB")
    d = ImageDraw.Draw(im)
    d.polygon(band, outline=(255, 220, 60), width=2)
    # boundaries
    for ring, col in ((OUTER, (70, 130, 220)), (INNER, (220, 70, 70))):
        for seg in ring:
            pts = [P(*p) for p in seg_samples(seg, 0.05)]
            d.line(pts, fill=col, width=2)
    # cones + ids
    for c in cones:
        X, Y = P(c['x'], c['y'])
        col = (80, 150, 255) if c['color'] == 'blue' else (255, 80, 80)
        d.ellipse([X-4, Y-4, X+4, Y+4], fill=col, outline=(255, 255, 255))
        d.text((X+5, Y-5), c['id'], fill=(235, 235, 235), font=fs)
    d.text((mx, 8), "TrackA synthetic boundary model - cones %d blue / %d red - "
            "yellow-V band = NO-CONE zone" %
            (sum(1 for c in cones if c['color'] == 'blue'),
             sum(1 for c in cones if c['color'] == 'red')), fill=(240, 240, 240), font=fb)
    im.save(path)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spacing", type=float, default=0.5,
                    help="straight cone spacing; simulation assumption, rule only says <1.5 m")
    ap.add_argument("--curve-spacing", type=float, default=0.4,
                    help="arc cone spacing; simulation assumption, rule only says <1.5 m")
    ap.add_argument("--source", choices=("fig9", "uniform"), default="fig9",
                    help="fig9 preserves each illustrated cone; uniform is a synthetic stress test")
    a = ap.parse_args()
    if not (0.3 <= a.spacing < 1.5 and 0.3 <= a.curve_spacing < 1.5):
        raise SystemExit("spacing and curve-spacing must be in [0.3, 1.5) m")
    if a.source == "fig9":
        import importlib.util
        path = os.path.join(HERE, "fig9a_cones.py")
        spec = importlib.util.spec_from_file_location("fig9a_cones", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cones = list(module.rows())
    else:
        cones = gen_cones(a.spacing, a.curve_spacing)
    os.makedirs(OUTDIR, exist_ok=True)
    write_csv(os.path.join(OUTDIR, "trackA_cones.csv"), cones)
    write_yaml(os.path.join(OUTDIR, "trackA_layout.yaml"), cones, a.spacing, a.curve_spacing, a.source)
    if a.source == "fig9":
        module.write_overlay(os.path.join(OUTDIR, "trackA_topview.png"))
        module.write_overlay(os.path.join(OUTDIR, "trackA_fig9_cone_overlay.png"))
    else:
        write_topview(os.path.join(OUTDIR, "trackA_topview.png"), cones)
    st = spacing_stats(cones)
    print("cones: %d total" % len(cones))
    for color, (n, mn, mx_, mean, sd, breaks) in st.items():
        print("  %-5s n=%3d  spacing actual min=%.3f max=%.3f mean=%.3f sd=%.3f"
              % (color, n, mn, mx_, mean, sd))
        for b in breaks:
            print("         yellow-V break: %s m" % b)
    print("source: %s; rule only says spacing < 1.5 m" % a.source)
    print("outputs -> %s : trackA_cones.csv / trackA_layout.yaml / trackA_topview.png" % OUTDIR)

if __name__ == "__main__":
    main()
