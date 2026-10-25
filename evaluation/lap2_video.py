"""第二圈走线动画:二维俯视地图上回放 record 局的轨迹,速度着色+实时时间/圈数/速度。

    python -m evaluation.lap2_video --model runs/road_v2.5_td3/candidate_004.npz \
        --tracks A B C D --cap2 1.8 --out-dir runs/eval_final/videos

每赛道跑一局(record),生成 trackX_capY.gif:
  - 锥桶(红/蓝)、内外边界、START 框
  - 轨迹按速度着色(turbo 色标),车用三角标记
  - 标题实时显示 t= / LAP / v=;结束定格显示两圈成绩与停车结果
  - 3 倍速回放(每 0.3s 一帧,fps=10)
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['AR PL UKai CN', 'Noto Serif CJK SC', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection

from evaluation.road_lap2 import run
from evaluation.road_track import _boundaries, _cones, official

V_MAX_DISPLAY = 2.5          # 色标上限 m/s(plan 模式可达 2.4)
STEP = 3                     # 每 3 个 trace 点一帧(3 倍速)
FPS = 10


def _start_box(name):
    """START 框 (x0, x1, y_lo, y_hi):x 来自 layout;y 取中心线在起点处的宽度。"""
    road, _ = official(name, True, laps=1)
    _, _, _, (x0, x1) = _boundaries(name)
    y_lo = float(road.centre[0][1] - road.half[0])
    y_hi = float(road.centre[0][1] + road.half[0])
    return float(x0), float(x1), y_lo, y_hi


def make_video(track: str, ep: dict, out: Path):
    t = np.array([p[0] for p in ep['trace']])
    xy = np.array([[p[1], p[2]] for p in ep['trace']])
    v = np.array([p[3] for p in ep['trace']])
    lap = np.array([p[5] for p in ep['trace']])

    outer, inner, _, _ = _boundaries(track)
    rows = _cones(track)

    fig, ax = plt.subplots(figsize=(8, 6), dpi=85)
    ax.plot(outer[:, 0], outer[:, 1], '-', color='#9aa0a6', lw=1.0)
    ax.plot(inner[:, 0], inner[:, 1], '-', color='#9aa0a6', lw=1.0)
    cone_c = {'red': '#e53935', 'blue': '#1e88e5', 'yellow': '#fdd835'}
    for c in set(r['color'] for r in rows):
        pts = np.array([[float(r['x']), float(r['y'])] for r in rows if r['color'] == c])
        ax.scatter(pts[:, 0], pts[:, 1], s=18 if c != 'yellow' else 30,
                   color=cone_c.get(c, 'gray'), zorder=3,
                   marker='^' if c != 'yellow' else 's')
    try:
        x0, x1, y_lo, y_hi = _start_box(track)
        ax.add_patch(plt.Rectangle((x0, y_lo), x1 - x0, y_hi - y_lo,
                                   fill=False, ec='#f9a825', lw=1.6, ls='--', zorder=3))
        ax.annotate('START/停车框', (x0, y_hi), fontsize=8, color='#f9a825',
                    xytext=(2, 3), textcoords='offset points')
    except Exception:
        pass

    all_pts = np.vstack([outer, inner, xy])
    pad = 0.6
    ax.set_xlim(all_pts[:, 0].min() - pad, all_pts[:, 0].max() + pad)
    ax.set_ylim(all_pts[:, 1].min() - pad, all_pts[:, 1].max() + pad)
    ax.set_aspect('equal')
    ax.set_xticks([]), ax.set_yticks([])
    ax.set_facecolor('#111418')
    fig.patch.set_facecolor('#111418')

    cmap = plt.get_cmap('turbo')
    lc = LineCollection([], linewidths=2.2, cmap=cmap, norm=plt.Normalize(0, V_MAX_DISPLAY))
    ax.add_collection(lc)
    cbar = fig.colorbar(lc, ax=ax, fraction=0.035, pad=0.02)
    cbar.set_label('速度 m/s', color='#eeeeee', fontsize=9)
    cbar.ax.tick_params(colors='#eeeeee', labelsize=8)
    cbar.outline.set_edgecolor('#555')

    car_mk, = ax.plot([], [], '^', color='white', ms=9, mec='black', zorder=5)
    title = ax.set_title('', color='#eeeeee', fontsize=11)

    laps = ep['laps']
    n_end = len(t)
    end_txt = (f"LAP1 {laps[0]:.1f}s   LAP2 {laps[1]:.1f}s   "
               f"STOP {'OK框内' if ep.get('in_box') else 'X出框'} "
               f"(along{ep['stop_along']:+.2f}m lat{ep['stop_lat']:+.2f}m)")

    def draw(i0):
        i0 = min(i0, n_end - 1)
        if i0 > 0:
            seg = np.stack([xy[:-1], xy[1:]], axis=1)[:i0]
            lc.set_segments(seg)
            lc.set_array(v[:i0])
        car_mk.set_data([xy[i0, 0]], [xy[i0, 1]])
        if i0 >= n_end - 1:
            title.set_text(f'赛道 {track}  cap={ep["cap2"]}   {end_txt}')
        else:
            title.set_text(f'赛道 {track}  cap={ep["cap2"]}   t={t[i0]:6.1f}s   '
                           f'LAP {min(int(lap[i0]) + 1, 2)}   v={v[i0]:.2f} m/s')

    frames = list(range(0, n_end, STEP)) + [n_end - 1] * 12
    ani = FuncAnimation(fig, draw, frames=frames, interval=1000 / FPS)
    ani.save(out, writer=PillowWriter(fps=FPS))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', required=True)
    ap.add_argument('--tracks', nargs='+', default=['A', 'B', 'C', 'D'])
    ap.add_argument('--cap1', type=float, default=0.5)
    ap.add_argument('--cap2', type=float, default=1.0)
    ap.add_argument('--a-lat-sup', default='1.0')
    ap.add_argument('--v-source', choices=['model', 'plan'], default='model')
    ap.add_argument('--a-acc', type=float, default=0.4)
    ap.add_argument('--a-dec', type=float, default=0.45)
    ap.add_argument('--a-lat-low', type=float, default=None)
    ap.add_argument('--kappa-split', type=float, default=0.55)
    ap.add_argument('--seed', type=int, default=96000000)
    ap.add_argument('--out-dir', default='runs/eval_final/videos')
    a = ap.parse_args()
    lat = None if str(a.a_lat_sup).lower() == 'none' else float(a.a_lat_sup)
    out_dir = Path(a.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = 'plan' if a.v_source == 'plan' else 'model'
    for k, tr in enumerate(a.tracks):
        ep = run((a.model, tr, a.seed + k, a.cap1, a.cap2, lat, 0.42, True,
                  a.v_source, a.a_acc, a.a_dec, a.a_lat_low, a.kappa_split))
        if 'trace' not in ep:
            print(f'赛道 {tr} 未完成({ep["reason"]}),跳过')
            continue
        out = out_dir / f'track{tr}_cap{a.cap2}_{tag}.gif'
        make_video(tr, ep, out)
        print(f'{tr}: lap1={ep["laps"][0]:.1f}s lap2={ep["laps"][1]:.1f}s '
              f'in_box={ep["in_box"]} -> {out}')


if __name__ == '__main__':
    main()
