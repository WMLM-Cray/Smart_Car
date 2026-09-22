#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""赛道A 图9(规则PDF第11页嵌入图 fig11-000.jpg)线迹提取 + 结构重建 + 叠加对照。

产出: fig9A_overlay_draft.png 三联图(官方线迹 / 重建结构 / 叠加)
证据链: 像素分割只做定位; 尺寸以图9标注为准(1.25/1.5/3.06/135°/4m/45°/16.8)。
"""
import math
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Arc as MArc, Rectangle

import os as _os
HERE = _os.path.dirname(_os.path.abspath(__file__))
IM = _os.path.join(HERE, "..", "..", "docs", "track", "fig9A_source.jpg")

im = Image.open(IM).convert("RGB")
W, H = im.size
a = np.asarray(im).astype(np.int16)
R, G, B = a[..., 0], a[..., 1], a[..., 2]
blue = (B > 100) & (B - R > 40) & (B - G > 25)
red  = (R > 130) & (R - G > 50) & (R - B > 50)
yel  = (R > 150) & (G > 120) & (B < 110) & (R - B > 60) & (G - B > 40)
cyan = (G > 120) & (B > 120) & (R < 110) & (abs(G - B) < 60) & (G - R > 40)

bx = np.nonzero(blue)[1]
s = 16.8 / (bx.max() - bx.min())
YM0 = 522  # 底部蓝直道中心线
def XM(x): return (x - bx.min()) * s
def YM(y): return (YM0 - y) * s

def pts_m(mask):
    ys, xs = np.nonzero(mask)
    return np.array([XM(xs), YM(ys)]).T

pb, pr, py_, pc = pts_m(blue), pts_m(red), pts_m(yel), pts_m(cyan)

# ---------------- 重建几何(参数=图9标注优先, 像素轨迹定位) ----------------
W_CORR = 1.50          # fig9: 走廊宽 1.5(两端 1.5+1.25 尺寸链之第一段)
R_IN = 1.25            # fig9: 内弧半径(右端 1.25 标注; 拟合 1.22)
R_OUT = R_IN + W_CORR  # = 2.75(16.8-右弧心14.05; 左端拟合同量级)
C_L_IN = (2.66, 2.87)   # image: 左端内弧心(红弧拟合 2.66,3.07; 与顶/底切线自洽)
C_L_OUT = (3.20, 3.40)  # image: 左端外大弧心(轨迹拟合: 顶(0,3.4)/底(3.2,0.1))
R_L_OUT = 3.30          # image: 左外弧半径(五点验算 3.24-3.43)
LEFT_VX = 0.0           # image: 左上直角竖直边 x=0(顶水平线在 (0,5.57) 90° 下折)
C_R = (14.05, 2.82)    # image: 右弧心(红弧拟合 14.0,2.87; 16.8-R_OUT)
Y_BO, Y_BI = 0.20, 1.65    # image: 底直道蓝/红中心线
Y_TO, Y_TI = 5.57, 4.10    # image: 顶直道蓝/红中心线
# 顶部锥桶V弯(蓝/红平行下凹, 谷深1.15; image)
V1_B = [(3.00, Y_TO), (5.50, 4.42), (7.60, Y_TO)]
V1_R = [(3.50, Y_TI), (5.60, 2.95), (7.40, Y_TI)]
# 黄胶带V区(等臂3.06@135°; fig9 3.06/135°; 顶点由 4m@45° 定位线锁定)
DV = 3.06 * math.sin(math.radians(22.5))   # =1.171
YU_L, YU_R = 8.50, 13.90    # image: 黄区左/右接直道处
X_APEX = (YU_L + YU_R) / 2  # 11.20
YU_TO = YU_L + 0.0          # 上带左肩 y 取 5.55(带左端延伸接蓝线终点)
V2_U = [(YU_L, 5.55), (X_APEX, 5.55 - DV - 0.0), (YU_R, 5.55)]
V2_D = [(YU_L, 4.07), (X_APEX, 4.07 - DV + 0.03), (YU_R, 4.07)]
# 功能区(image: cyan掩码范围/黄格轨迹)
ZONES = {'traffic_light(cyan)': ((2.65, 5.19), (0.01, 1.84)),
         'zone A (grid)':      ((6.20, 7.30), (0.18, 1.66)),
         'zone A/B up/dn':     ((8.40, 9.50), (0.18, 1.66)),
         'start zone':         ((10.70, 11.70), (0.18, 1.66))}

# ---------------- 画图 ----------------
fig, axes = plt.subplots(1, 3, figsize=(24, 8.5), dpi=100)
titles = ['(1) Official fig11-000 traced (16.8m calibrated)',
          '(2) Rebuilt structure (fig9 dims + trace fit)',
          '(3) Overlay: rebuilt lines over official trace']

for ax in axes:
    ax.set_aspect('equal'); ax.grid(alpha=.15)
    ax.set_xlim(-1.2, 18.0); ax.set_ylim(-1.2, 7.2)
    ax.set_xlabel('x [m]')

# (1) 官方线迹
ax = axes[0]
ax.scatter(pb[:, 0], pb[:, 1], s=1.5, c='tab:blue', label='blue trace')
ax.scatter(pr[:, 0], pr[:, 1], s=1.5, c='tab:red', label='red trace')
ax.scatter(py_[:, 0], py_[:, 1], s=1.5, c='gold', label='yellow trace')
ax.scatter(pc[:, 0], pc[:, 1], s=1.5, c='tab:cyan', label='cyan box')
ax.legend(loc='lower right', fontsize=8, markerscale=6)
ax.set_title(titles[0], fontsize=11)
ax.annotate('16.8 m span (blue)', xy=(8.4, -0.7), fontsize=9, ha='center')

# (2) 重建
ax = axes[1]
def poly(ax, pts, **kw):
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    ax.plot(xs, ys, **kw)
# 外环(蓝): 底直道-右弧-顶右直道-黄上带-短直道-V1蓝-顶左直道-左弧
ax.add_patch(MArc(C_R, R_OUT*2, R_OUT*2, theta1=-90, theta2=90, color='tab:blue', lw=2))
poly(ax, [(LEFT_VX, Y_TO), (LEFT_VX, 3.40)], color='tab:blue', lw=2)  # 90°直角竖直边
ax.add_patch(MArc(C_L_OUT, R_L_OUT*2, R_L_OUT*2, theta1=180, theta2=270, color='tab:blue', lw=2))
poly(ax, [(C_L_OUT[0], Y_BO), (C_R[0], Y_BO)], color='tab:blue', lw=2)
poly(ax, [(C_R[0], Y_TO), (YU_R, 5.55)], color='tab:blue', lw=2)  # 顶右直道(弧顶切点→黄区右肩)
poly(ax, V2_U, color='gold', lw=4, solid_capstyle='butt')
poly(ax, [(YU_L, 5.55), (V1_B[0][0], Y_TO)], color='tab:blue', lw=2)
poly(ax, V1_B, color='tab:blue', lw=2)
poly(ax, [(V1_B[2][0], Y_TO), (LEFT_VX, Y_TO)], color='tab:blue', lw=2)
# 内环(红)
ax.add_patch(MArc(C_R, R_IN*2, R_IN*2, theta1=-90, theta2=90, color='tab:red', lw=2))
ax.add_patch(MArc(C_L_IN, R_IN*2, R_IN*2, theta1=90, theta2=270, color='tab:red', lw=2))
poly(ax, [(C_L_IN[0], Y_BI), (C_R[0], Y_BI)], color='tab:red', lw=2)
poly(ax, [(C_R[0], Y_TI), (YU_R, 4.07)], color='tab:red', lw=2)
poly(ax, V2_D, color='gold', lw=4, solid_capstyle='butt')
poly(ax, [(YU_L, 4.07), (V1_R[0][0], Y_TI)], color='tab:red', lw=2)
poly(ax, V1_R, color='tab:red', lw=2)
poly(ax, [(V1_R[2][0], Y_TI), (C_L_IN[0], Y_TI)], color='tab:red', lw=2)
# 4m@45° 定位线(黑虚)
poly(ax, [(YU_L, Y_TO), (X_APEX, 2.97)], color='k', lw=1, ls='--')
ax.text(9.3, 4.6, '4 m @ 45°\n(locates apex)', fontsize=8, rotation=-40)
for name, ((x0, x1), (y0, y1)) in ZONES.items():
    col = 'tab:cyan' if 'cyan' in name else 'orange'
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec=col,
                           lw=1.6, ls='--' if 'cyan' in name else '-'))
    ax.text((x0 + x1) / 2, y1 + 0.12, name, ha='center', fontsize=7.5, color=col)
ax.set_title(titles[1], fontsize=11)
ax.annotate('R2.75 / R1.25', xy=(15.3, 2.8), fontsize=8)
ax.annotate('3.06 m @135°', xy=(10.3, 5.9), fontsize=8, ha='center')
ax.annotate('corridor 1.5', xy=(4.0, 1.0), fontsize=8)
ax.annotate('90 deg corner + R3.3 outer arc (outer square/inner circle)',
            xy=(0.2, 1.6), fontsize=7.5)

# (3) 叠加
ax = axes[2]
ax.scatter(pb[:, 0], pb[:, 1], s=1.2, c='tab:blue', alpha=.35)
ax.scatter(pr[:, 0], pr[:, 1], s=1.2, c='tab:red', alpha=.35)
ax.scatter(py_[:, 0], py_[:, 1], s=1.2, c='gold', alpha=.5)
ax.scatter(pc[:, 0], pc[:, 1], s=1.2, c='tab:cyan', alpha=.5)
ax.add_patch(MArc(C_R, R_OUT*2, R_OUT*2, theta1=-90, theta2=90, color='k', lw=1.2, ls='--'))
poly(ax, [(LEFT_VX, Y_TO), (LEFT_VX, 3.40)], color='k', lw=1.2, ls='--')
ax.add_patch(MArc(C_L_OUT, R_L_OUT*2, R_L_OUT*2, theta1=180, theta2=270, color='k', lw=1.2, ls='--'))
ax.add_patch(MArc(C_R, R_IN*2, R_IN*2, theta1=-90, theta2=90, color='k', lw=1.2, ls='--'))
ax.add_patch(MArc(C_L_IN, R_IN*2, R_IN*2, theta1=90, theta2=270, color='k', lw=1.2, ls='--'))
poly(ax, [(C_L_OUT[0], Y_BO), (C_R[0], Y_BO)], color='k', lw=1.2, ls='--')
poly(ax, [(C_R[0], Y_TO), (YU_R, 5.55)], color='k', lw=1.2, ls='--')
poly(ax, [(YU_L, 5.55), (V1_B[0][0], Y_TO)], color='k', lw=1.2, ls='--')
poly(ax, V1_B, color='k', lw=1.2, ls='--')
poly(ax, [(V1_B[2][0], Y_TO), (LEFT_VX, Y_TO)], color='k', lw=1.2, ls='--')
poly(ax, [(C_L_IN[0], Y_BI), (C_R[0], Y_BI)], color='k', lw=1.2, ls='--')
poly(ax, [(C_R[0], Y_TI), (YU_R, 4.07)], color='k', lw=1.2, ls='--')
poly(ax, [(YU_L, 4.07), (V1_R[0][0], Y_TI)], color='k', lw=1.2, ls='--')
poly(ax, V1_R, color='k', lw=1.2, ls='--')
poly(ax, [(V1_R[2][0], Y_TI), (C_L_IN[0], Y_TI)], color='k', lw=1.2, ls='--')
ax.set_title(titles[2], fontsize=11)

fig.tight_layout()
out = _os.path.normpath(_os.path.join(HERE, "..", "..", "docs", "track", "trackA_fig9_overlay.png"))
fig.savefig(out)
print("wrote", out)

# 残差报告: 重建线到官方轨迹的吻合度(采样重建线, 找最近官方点距离)
def seg_pts(p0, p1, n=40):
    return [(p0[0] + (p1[0] - p0[0]) * i / n, p0[1] + (p1[1] - p0[1]) * i / n)
            for i in range(n + 1)]
def arc_pts(c, r, a0, a1, n=120):
    return [(c[0] + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             c[1] + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]
outer = ([(C_L_OUT[0], Y_BO), (C_R[0], Y_BO)] + arc_pts(C_R, R_OUT, -90, 90)
         + [(C_R[0], Y_TO), (YU_R, 5.55)] + V2_U + [(YU_L, 5.55), (V1_B[0][0], Y_TO)]
         + V1_B + [(V1_B[2][0], Y_TO), (LEFT_VX, Y_TO), (LEFT_VX, 3.40)]
         + arc_pts(C_L_OUT, R_L_OUT, 180, 270))
inner = ([(C_L_IN[0], Y_BI), (C_R[0], Y_BI)] + arc_pts(C_R, R_IN, -90, 90)
         + [(C_R[0], Y_TI), (YU_R, 4.07)] + V2_D + [(YU_L, 4.07), (V1_R[0][0], Y_TI)]
         + V1_R + [(V1_R[2][0], Y_TI), (C_L_IN[0], Y_TI)] + arc_pts(C_L_IN, R_IN, 90, 270))
from matplotlib.path import Path
def near_dist(p, arr):
    return np.min(np.hypot(arr[:, 0] - p[0], arr[:, 1] - p[1]))
blue_off = [near_dist(p, pb) for p in outer if p not in V2_U]
red_off = [near_dist(p, pr) for p in inner if p not in V2_D]
print("outer(blue incl V1) median off = %.3f m, p90 = %.3f" % (
    np.median(blue_off), np.percentile(blue_off, 90)))
print("inner(red incl V1)  median off = %.3f m, p90 = %.3f" % (
    np.median(red_off), np.percentile(red_off, 90)))
# 黄带单独
yu = seg_pts((YU_L, 5.55), (X_APEX, 5.55 - DV)) + seg_pts((X_APEX, 5.55 - DV), (YU_R, 5.55))
yd = seg_pts((YU_L, 4.07), (X_APEX, 4.07 - DV)) + seg_pts((X_APEX, 4.07 - DV), (YU_R, 4.07))
pyl = py_[(py_[:, 0] > 8.3) & (py_[:, 0] < 14.1) & (py_[:, 1] > 2.5)]
print("yellow V zone pts=%d; sample up-band y@x=11.2: %.2f vs rebuilt %.2f" % (
    len(pyl), pyl[np.argmin(np.abs(pyl[:, 0] - 11.2)), 1], 5.55 - DV))
