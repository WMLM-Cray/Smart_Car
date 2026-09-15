#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""赛道A 几何解析与坐标生成(阶段2 交付1:道具摆在哪里)。

结构定案(规则 PDF 图3-赛道A 嵌入图 656×231px 颜色分割 + 图9 尺寸体系交叉):
  体育场环形: 两端 180° 半圆回头弯,同心圆心 (2.76,2.76)/(14.04,2.76),
    外半径 2.76(=16.80-14.04,纵向总高 5.52=2×2.76 自洽),
    内半径 1.26(右内弧最右实测 15.30-14.04;图9 的"1.25m"即内弯半径),
    走廊宽 2.76-1.26=1.50 与规则走廊宽互证。
  顶边双 V 折线(均 ≈3.06m 斜边@135° 顶角):
    V#1 锥桶线  内肩 x[2.76,8.55] 谷(5.655,2.85);外肩 x[3.69,7.47] 谷(5.63,4.26)
    V#2 黄线    内肩 x[8.55,14.04] 谷(11.295,2.85);外肩同 x 谷(11.295,4.26)
  底边长直道功能区(左→右): 红绿灯区青框→A区→A/B区→发车区。逆时针行车。

坐标系: 原点=外边界左下角, x 向右, y 向上, 单位 m。
来源标注: rule=规则文字 / fig9=图9标注 / image=赛道A图读(1px≈26.33mm,
16.8m 总长标定) / asmp=假设(待复核清单已列)。

输出(仓库 docs/track/):
  trackA_layout.yaml  坐标清单(锥桶/黄线/区框/中心线/出发位)
  trackA_cones.csv    id,color,x,y(兼容 hwcart_sim/src/track_generator)
  trackA_topview.png  带尺寸标注顶视图
用法(WSL): python3 /mnt/f/.../07-仿真系统/tools/trackA/trackA_geometry.py
"""
import csv
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.normpath(os.path.join(HERE, '..', '..', 'docs', 'track'))

# ---------------- 参数(值, 来源, 依据) ----------------
R_OUT = 2.76          # image: 16.80-14.04;纵向 5.52=2x2.76 自洽
R_IN = 1.26           # image+fig9: 右内弧最右 15.30;图9 "1.25"
C_L = (2.76, 2.76)    # image: 左弧心(外弧左缘切 x=0)
C_R = (14.04, 2.76)   # image: 右弧心 = 15.30-1.26
Y_TO, Y_TI = 5.52, 4.02   # 顶外/内基线(image;差=走廊宽1.50)
Y_BO, Y_BI = 0.00, 1.50   # 底外/内基线(image)
V_DEPTH = 3.06 * math.sin(math.radians(22.5))  # fig9: 3.06*sin(135°/2 补角)=1.171
CONES_STEP = 1.0     # asmp: 锥间距,图上目测1~1.25m,规则<1.5m
# 顶边 V 折线拐点(image;外V#1肩图读5.29~5.36规整至基线5.52,差<0.23m)
V1_OUT = [(3.69, Y_TO), (5.655, 4.26), (7.47, Y_TO)]  # 谷x图读5.63,与内谷对齐(差0.025m在精度内)
V1_IN = [(2.76, Y_TI), (5.655, Y_TI - V_DEPTH), (8.55, Y_TI)]
V2_OUT = [(8.55, Y_TO), (11.295, 4.26), (14.04, Y_TO)]
V2_IN = [(8.55, Y_TI), (11.295, Y_TI - V_DEPTH), (14.04, Y_TI)]
# 底直道功能区 x 范围(image: 青框15.9~31%/A框36~44%/AB框50~56%/发车62~70%)
ZONES = {
    'traffic_light_zone': ((2.52, 5.14), 'image', '红绿灯停车区(青虚框);红绿灯当天随机'),
    'zone_a':             ((6.00, 7.38), 'image', 'A 检测/停车区黄框'),
    'zone_ab':            ((8.41, 9.44), 'image', 'A/B 双格黄框(上下格语义待复核)'),
    'start_zone':         ((10.49, 11.87), 'image', '发车区黄框(图内竖排红字)'),
}
START_POSE = (11.18, 0.75, 0.0)   # asmp: 发车区中心,朝+x(逆时针首段方向)
TAPE_W = 0.10                     # rule: 黄胶带宽10cm


def seg_line(p0, p1):
    return ('line', p0, p1)


def seg_arc(c, r, a0, a1):
    """角度制,a0→a1 逆时针为正;参数化 (c+r·cosθ, c+r·sinθ)。"""
    return ('arc', c, r, a0, a1)


def sample_segments(segs, step):
    """沿段序列等弧长采样(每段 ceil(len/step) 均匀分段,含端点,去相邻重复)。"""
    pts = []
    for seg in segs:
        if seg[0] == 'line':
            _, (x0, y0), (x1, y1) = seg
            n = max(1, math.ceil(math.hypot(x1 - x0, y1 - y0) / step))
            for i in range(n + 1):
                t = i / n
                pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        else:
            _, (cx, cy), r, a0, a1 = seg
            n = max(1, math.ceil(abs(math.radians(a1 - a0)) * r / step))
            for i in range(n + 1):
                t = math.radians(a0 + (a1 - a0) * i / n)
                pts.append((cx + r * math.cos(t), cy + r * math.sin(t)))
    out = []
    for p in pts:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 1e-9:
            out.append(p)
    return out


def build_model():
    """返回 dict: boundaries/cones/yellow_tape/zones/start_pose/centerline。"""
    # 外边界环(逆时针,自左弧底 (2.76,0) 起)
    outer = [
        seg_line((C_L[0], Y_BO), (C_R[0], Y_BO)),          # 底直道外
        seg_arc(C_R, R_OUT, -90, 90),                       # 右外回头弯
        seg_line(V2_OUT[2], V2_OUT[1]), seg_line(V2_OUT[1], V2_OUT[0]),  # V2 外(右→左)
        seg_line(V2_OUT[0], V1_OUT[2]),                     # 肩间平段 1.08m
        seg_line(V1_OUT[2], V1_OUT[1]), seg_line(V1_OUT[1], V1_OUT[0]),  # V1 外
        seg_line(V1_OUT[0], (C_L[0], Y_TO)),                # 进左弧前平段 0.93m
        seg_arc(C_L, R_OUT, 90, 270),                       # 左外回头弯
    ]
    # 内边界环(逆时针,自左弧底 (2.76,1.50) 起)
    inner = [
        seg_line((C_L[0], Y_BI), (C_R[0], Y_BI)),
        seg_arc(C_R, R_IN, -90, 90),
        seg_line(V2_IN[2], V2_IN[1]), seg_line(V2_IN[1], V2_IN[0]),
        seg_line(V2_IN[0], V1_IN[2]),                       # V1/V2 共享肩(8.55,4.02)
        seg_line(V1_IN[2], V1_IN[1]), seg_line(V1_IN[1], V1_IN[0]),
        seg_arc(C_L, R_IN, 90, 270),
    ]
    # 中心线(逆时针;顶段谷点=内外谷中点,V1 中线肩=内外肩中点 x)
    mid_v1 = ((V1_OUT[0][0] + V1_IN[0][0]) / 2, (Y_TO + Y_TI) / 2)
    mid_v1r = ((V1_OUT[2][0] + V1_IN[2][0]) / 2, (Y_TO + Y_TI) / 2)
    r_c = (R_OUT + R_IN) / 2
    y_mid = Y_BI / 2 + 0  # 底中线 y=0.75
    centerline = [
        seg_line((C_L[0], y_mid), (C_R[0], y_mid)),
        seg_arc(C_R, r_c, -90, 90),
        seg_line((C_R[0], (Y_TO + Y_TI) / 2), (V2_IN[1][0], (V2_IN[1][1] + V2_OUT[1][1]) / 2)),
        seg_line((V2_IN[1][0], (V2_IN[1][1] + V2_OUT[1][1]) / 2), (8.55, (Y_TO + Y_TI) / 2)),
        seg_line((8.55, (Y_TO + Y_TI) / 2), mid_v1r),
        seg_line(mid_v1r, (V1_IN[1][0], (V1_IN[1][1] + V1_OUT[1][1]) / 2)),
        seg_line((V1_IN[1][0], (V1_IN[1][1] + V1_OUT[1][1]) / 2), mid_v1),
        seg_line(mid_v1, (C_L[0], (Y_TO + Y_TI) / 2)),
        seg_arc(C_L, r_c, 90, 270),
    ]
    cones = []
    for i, p in enumerate(sample_segments(outer, CONES_STEP), 1):
        cones.append({'id': 'b%02d' % i, 'color': 'blue', 'boundary': 'outer', 'x': p[0], 'y': p[1]})
    for i, p in enumerate(sample_segments(inner, CONES_STEP), 1):
        cones.append({'id': 'r%02d' % i, 'color': 'red', 'boundary': 'inner', 'x': p[0], 'y': p[1]})
    def arm(a, b):
        return [a[0], a[1], b[0], b[1]]

    yellow_tape = {
        'width_m': TAPE_W,
        'note': 'asmp: 黄线=折线区(V#2)地面标线,锥桶仍沿内外边界布置',
        'segments': [arm(V2_OUT[0], V2_OUT[1]), arm(V2_OUT[1], V2_OUT[2]),
                     arm(V2_IN[0], V2_IN[1]), arm(V2_IN[1], V2_IN[2])],
    }
    return {
        'outer': outer, 'inner': inner, 'centerline': centerline,
        'cones': cones, 'yellow_tape': yellow_tape,
        'zones': ZONES, 'start_pose': START_POSE,
    }


# ---------------- 输出 ----------------

def fmt(v):
    return round(v, 4)


def seg_yaml(seg):
    if seg[0] == 'line':
        return 'line: [%s, %s, %s, %s]' % tuple(fmt(v) for p in seg[1:] for v in p)
    _, c, r, a0, a1 = seg
    return 'arc: {center: [%s, %s], radius: %s, angles_deg: [%s, %s]}' % (
        fmt(c[0]), fmt(c[1]), fmt(r), a0, a1)


def write_yaml(model, path):
    L = []
    L.append('# 赛道A 坐标清单(阶段2 交付1)——几何依据见 trackA_尺寸依据.md')
    L.append('# 坐标系: 原点=外边界左下角, x向右, y向上, 单位m, 行车逆时针')
    L.append('track: A')
    L.append('frame:')
    L.append('  origin: outer_boundary_lower_left')
    L.append('  axes: {x: right, y: up}')
    L.append('  unit: m')
    L.append('params:')
    L.append('  total_length: {value: 16.80, source: rule}')
    L.append('  corridor_width: {value: 1.50, source: "rule+fig9"}')
    L.append('  hairpin_outer_radius: {value: %s, source: image}' % R_OUT)
    L.append('  hairpin_inner_radius: {value: %s, source: "image+fig9(1.25)"}' % R_IN)
    L.append('  hairpin_centers: {value: [[%s, %s], [%s, %s]], source: image}' % (
        C_L[0], C_L[1], C_R[0], C_R[1]))
    L.append('  v_side_length: {value: 3.06, source: fig9}')
    L.append('  v_apex_angle_deg: {value: 135, source: fig9}')
    L.append('  cone_spacing: {value: %s, source: asmp}' % CONES_STEP)
    L.append('boundaries:')
    L.append('  outer:  # 蓝锥连线,逆时针,自左弧底')
    for s in model['outer']:
        L.append('    - %s' % seg_yaml(s))
    L.append('  inner:  # 红锥连线(岛边界),逆时针,自左弧底')
    for s in model['inner']:
        L.append('    - %s' % seg_yaml(s))
    L.append('cones:  # %d blue + %d red,顺序沿边界逆时针' % (
        sum(1 for c in model['cones'] if c['color'] == 'blue'),
        sum(1 for c in model['cones'] if c['color'] == 'red')))
    for c in model['cones']:
        L.append('  - {id: %s, color: %s, x: %s, y: %s}' % (
            c['id'], c['color'], fmt(c['x']), fmt(c['y'])))
    yt = model['yellow_tape']
    L.append('yellow_tape:')
    L.append('  width_m: %s  # %s' % (yt['width_m'], yt['note']))
    L.append('  segments:  # 线段 [x0, y0, x1, y1]')
    for s in yt['segments']:
        L.append('    - [%s]' % ', '.join(str(fmt(v)) for v in s))
    L.append('zones:  # 均在底直道走廊 y∈[0, 1.50] 内;x范围图读')
    for name, ((x0, x1), src, note) in model['zones'].items():
        L.append('  %s: {x: [%s, %s], y: [0.0, 1.5], source: %s, note: "%s"}' % (
            name, x0, x1, src, note))
    sp = model['start_pose']
    L.append('start_pose: {x: %s, y: %s, yaw_deg: %s}  # asmp: 发车区中心,朝+x' % (
        fmt(sp[0]), fmt(sp[1]), sp[2]))
    L.append('centerline:  # 行车参考线,逆时针(弧半径=(2.76+1.26)/2=2.01)')
    for s in model['centerline']:
        L.append('  - %s' % seg_yaml(s))
    L.append('random_day_elements:  # 比赛当天随机,基础场景不摆')
    L.append('  boards: []  # rule: 挡板3块80x50cm,位置随机')
    L.append('  traffic_light: null  # rule: 红绿灯位置随机(默认见尺寸依据.md待复核)')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')


def write_csv(model, path):
    with open(path, 'w', encoding='utf-8', newline='') as f:
        w = csv.writer(f)
        w.writerow(['id', 'color', 'x', 'y'])
        for c in model['cones']:
            w.writerow([c['id'], c['color'], fmt(c['x']), fmt(c['y'])])


def write_png(model, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Arc as MArc, Rectangle

    fig, ax = plt.subplots(figsize=(20, 8), dpi=110)

    def draw(segs, **kw):
        pts = sample_segments(segs, 0.5)
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        ax.plot(xs, ys, **kw)

    draw(model['outer'], color='tab:blue', lw=2, label='outer boundary (blue cones)')
    draw(model['inner'], color='tab:red', lw=2, label='inner boundary (red cones)')
    cl = sample_segments(model['centerline'], 0.2)
    ax.plot([p[0] for p in cl], [p[1] for p in cl], 'g--', lw=1.2,
            alpha=0.8, label='centerline (CCW)')
    for s in model['yellow_tape']['segments']:
        ax.plot([s[0], s[2]], [s[1], s[3]], color='gold', lw=4, alpha=0.9,
                solid_capstyle='butt', label='_yellow tape 10cm')
    blues = [c for c in model['cones'] if c['color'] == 'blue']
    reds = [c for c in model['cones'] if c['color'] == 'red']
    ax.scatter([c['x'] for c in blues], [c['y'] for c in blues], s=14, c='blue',
               zorder=5, label='blue cones x%d' % len(blues))
    ax.scatter([c['x'] for c in reds], [c['y'] for c in reds], s=14, c='red',
               zorder=5, label='red cones x%d' % len(reds))
    zc = {'traffic_light_zone': ('cyan', 'traffic-light zone\n(random-day light)'),
          'zone_a': ('orange', 'zone A'), 'zone_ab': ('orange', 'zone A/B'),
          'start_zone': ('green', 'START')}
    for name, ((x0, x1), src, note) in model['zones'].items():
        col, lab = zc[name]
        ax.add_patch(Rectangle((x0, 0), x1 - x0, Y_BI, fill=False, ec=col,
                               lw=1.8, ls='--' if name == 'traffic_light_zone' else '-'))
        ax.text((x0 + x1) / 2, 1.62, lab, ha='center', va='bottom',
                fontsize=9, color=col)
    sp = model['start_pose']
    ax.annotate('', xy=(sp[0] + 0.7, sp[1]), xytext=(sp[0], sp[1]),
                arrowprops=dict(arrowstyle='-|>', color='green', lw=2))
    # 尺寸标注
    ax.annotate('', xy=(16.80, -0.75), xytext=(0, -0.75),
                arrowprops=dict(arrowstyle='<->', color='k'))
    ax.text(8.4, -1.05, '16.80 m (rule: total length)', ha='center', fontsize=10)
    ax.annotate('', xy=(-0.75, 5.52), xytext=(-0.75, 0),
                arrowprops=dict(arrowstyle='<->', color='k'))
    ax.text(-1.05, 2.76, '5.52 = 2xR_out', rotation=90, va='center', ha='right', fontsize=9)
    ax.text(16.95, 2.76, 'R_out 2.76\nR_in 1.26\n(corridor 1.50)', fontsize=9, va='center')
    ax.annotate('corridor 1.50 m', xy=(6.6, 0.75), xytext=(5.0, -0.55),
                fontsize=9, arrowprops=dict(arrowstyle='-', color='gray'))
    ax.annotate('V: side 3.06 m, apex 135 deg (fig9)', xy=(4.6, 4.9),
                xytext=(2.2, 6.15), fontsize=9,
                arrowprops=dict(arrowstyle='-', color='gray'))
    ax.set_xlim(-1.6, 18.6)
    ax.set_ylim(-1.5, 6.8)
    ax.set_aspect('equal')
    ax.grid(alpha=0.15, which='both')
    ax.set_xticks(range(0, 18))
    ax.set_yticks(range(0, 7))
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Track A - parametric layout (source-tagged; evidence in docs/track/)')
    ax.legend(loc='lower right', fontsize=9, ncol=2)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    m = build_model()
    yml = os.path.join(OUTDIR, 'trackA_layout.yaml')
    csvp = os.path.join(OUTDIR, 'trackA_cones.csv')
    png = os.path.join(OUTDIR, 'trackA_topview.png')
    write_yaml(m, yml)
    write_csv(m, csvp)
    write_png(m, png)
    nb = sum(1 for c in m['cones'] if c['color'] == 'blue')
    nr = sum(1 for c in m['cones'] if c['color'] == 'red')
    print('cones: %d blue + %d red = %d' % (nb, nr, nb + nr))
    print('wrote:', yml)
    print('wrote:', csvp)
    print('wrote:', png)


if __name__ == '__main__':
    main()
