"""实车逐帧回看(road_car_node 开车时记的 seg_NN/frames.npz,~3 Hz 的模型 BEV 输入 + 位姿 + 决策)出点阵视频:
左 = 实车锥桶图 + 车轨迹,右 = 车上模型当时吃的 BEV 输入(common/bev_view 点阵,灰 雷达记忆 / 红 本帧雷达)。

    python -m evaluation.logbook_replay runs/drive_1024c/logbook_1024_131728/seg_11 --map maps/realA_1023.yaml
    python -m evaluation.logbook_replay LOGBOOK/seg_* --map maps/realA_1023.yaml --out-dir runs/replay

位姿:车上推算位姿先按每帧锥桶配准纠偏(common/cone_slam.ConeLocalizer),再整段对地图锥桶 ICP 对齐;
不给 --map 就画推算位姿 + 这一段自己的轨迹。锥桶取自每帧"本帧雷达"层(10 cm 格),车身框
CarParams.self_box(车尾天线)里的点不参与配准。按记录间隔实时播放。2026-10-24 从 runs/drive_1024c 整理入库。
"""
from __future__ import annotations

import argparse
import math
import subprocess
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy.spatial import cKDTree

from common import bev_view
from common.cone_slam import ConeLocalizer, _fit_se2, relative, to_world
from common.params import CarParams
from evaluation.bev_video import S, font

CELL, AHEAD, SIDE = bev_view.CELL, bev_view.AHEAD, bev_view.SIDE
FPS = 20


def frame_points(bev):
    """一帧 BEV 的本帧雷达层 -> 车体系点(后轴系),去掉车身框里的点。"""
    r, c = np.nonzero(bev[4] > 0.5)
    p = np.c_[AHEAD - (r + 0.5) * CELL, SIDE - (c + 0.5) * CELL]
    x0, x1, hy = CarParams().self_box
    return p[~((p[:, 0] >= x0) & (p[:, 0] <= x1) & (np.abs(p[:, 1]) <= hy))]


def cones_from_points(p):
    """格点分簇(距离 <= 0.15 m 连通),外接尺寸 <= 0.3 m、离车 5 m 内的当锥桶,取质心。"""
    if not len(p):
        return np.zeros((0, 2))
    t = cKDTree(p)
    lab = -np.ones(len(p), int)
    k = 0
    for i in range(len(p)):
        if lab[i] >= 0:
            continue
        st, lab[i] = [i], k
        while st:
            j = st.pop()
            for n in t.query_ball_point(p[j], 0.15):
                if lab[n] < 0:
                    lab[n] = k
                    st.append(n)
        k += 1
    out = [g.mean(0) for g in (p[lab == q] for q in range(k))
           if np.ptp(g[:, 0]) <= 0.3 and np.ptp(g[:, 1]) <= 0.3 and np.hypot(*g.mean(0)) < 5.0]
    return np.asarray(out).reshape(-1, 2)


def corrected_poses(num, bev):
    """锥桶配准纠偏:初值 = 上一帧纠偏位姿 + 推算位姿增量;配上了用配准结果。返回 (位姿 (n, 3), 配准成功率)。"""
    import common.cone_slam as C
    poses = [tuple(num[0, 1:4])]
    loc = ConeLocalizer(poses[0], every=1, build=True)
    ok, orig = 0, C.extract_cones
    try:
        for k in range(len(num)):
            cones = cones_from_points(frame_points(bev[k]))
            C.extract_cones = lambda *_a, **_k: cones          # 本帧锥桶已经提好
            delta = relative(tuple(num[k - 1, 1:4]), tuple(num[k, 1:4])) if k else None
            p, m = loc.step(None, None, delta)
            ok += bool(m is not None and m.ok)
            if k:
                poses.append(p)
    finally:
        C.extract_cones = orig
    return np.array(poses), ok / len(num)


def align_to_map(num, bev, poses, cones_map):
    """整段每隔一帧的锥桶(纠偏位姿下)对地图锥桶 ICP;初值穷举朝向(每 10°)。返回 (对地图内点率, (tx, ty, th))。"""
    tree = cKDTree(cones_map)
    obs = np.vstack([to_world(cones_from_points(frame_points(bev[k])), poses[k]) for k in range(0, len(num), 2)])
    best = (-1.0, (0.0, 0.0, 0.0))
    c_obs, c_map = obs.mean(0), cones_map.mean(0)
    for th0 in np.radians(np.arange(0, 360, 10)):
        c, s = math.cos(th0), math.sin(th0)
        t0 = c_map - np.array([c * c_obs[0] - s * c_obs[1], s * c_obs[0] + c * c_obs[1]])
        T = (t0[0], t0[1], th0)
        for gate in (1.0, 0.6, 0.4, 0.25, 0.15, 0.15):
            d, j = tree.query(to_world(obs, T), distance_upper_bound=gate)
            i = np.isfinite(d)
            if i.sum() < 20:
                break
            T = _fit_se2(obs[i], cones_map[j[i]])
        d, _ = tree.query(to_world(obs, T))
        if np.mean(d < 0.15) > best[0]:
            best = (float(np.mean(d < 0.15)), T)
    return best


def world_panel(cones, trail, pose, view):
    """同 evaluation/bev_video.world_panel 的画法,赛道是锥桶图(实车没有路沿真值)。"""
    cx, cy, r = view
    k = S / (2 * r)
    to = lambda p: ((p[..., 0] - cx) * k + S / 2, S / 2 - (p[..., 1] - cy) * k)
    im = Image.new('RGB', (S, S), (250, 250, 250))
    d = ImageDraw.Draw(im)
    cr = bev_view.CONE_R * k
    for u, v in zip(*to(cones)):
        d.ellipse([u - cr, v - cr, u + cr, v + cr], fill=(255, 140, 0))
    if len(trail) > 1:
        u, v = to(np.asarray(trail))
        d.line(list(zip(u, v)), fill=(30, 90, 220), width=2)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    body = np.array([[-0.15, -0.15], [0.35, -0.15], [0.35, 0.15], [-0.15, 0.15]])
    u, v = to(np.c_[x + body[:, 0] * c - body[:, 1] * s, y + body[:, 0] * s + body[:, 1] * c])
    d.polygon(list(zip(u, v)), fill=(20, 20, 20))
    return im


def render(seg: Path, out: Path, cones_map=None):
    z = np.load(seg / 'frames.npz')
    num, bev, ev = z['num'], z['bev'].astype(np.float32), z['event']
    if cones_map is not None:
        poses, ok = corrected_poses(num, bev)
        score, T = align_to_map(num, bev, poses, cones_map)
        P = np.c_[to_world(poses[:, :2], T), poses[:, 2] + T[2]]
        cones, note = cones_map, f'锥桶配准 {ok:.0%},对地图内点 {score:.0%}'
    else:
        P, cones, note = num[:, 1:4], np.zeros((0, 2)), '推算位姿(未对齐地图)'
    pts = np.vstack([cones, P[:, :2]]) if len(cones) else P[:, :2]
    lo, hi = pts.min(0), pts.max(0)
    view = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, max(hi - lo) / 2 + 0.5)
    ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                           '-s', f'{2 * S}x{S + 40}', '-r', str(FPS), '-i', '-',
                           '-pix_fmt', 'yuv420p', '-vcodec', 'libx264', '-crf', '20', str(out)], stdin=subprocess.PIPE)
    f_big, f_small = font(16), font(13)
    t0 = num[0, 0]
    for k in range(len(num)):
        t, _, _, _, steer, target, v = num[k]
        frame = Image.new('RGB', (2 * S, S + 40), (255, 255, 255))
        frame.paste(world_panel(cones, P[:k + 1, :2], P[k], view), (0, 40))
        frame.paste(Image.fromarray(bev_view.render(bev[k], np.zeros((0, 2)), px=S // bev.shape[2])), (S, 40))
        d = ImageDraw.Draw(frame)
        d.text((8, 4), f'{seg.parent.name}/{seg.name}  t {t - t0:5.1f}s  v {v:4.2f}  '
                       f'要 {"刹车" if target < 0 else f"{target:4.2f}"}  转角 {math.degrees(steer):+5.1f}°  {ev[k] or ""}',
                 fill=(0, 0, 0), font=f_big)
        d.text((8, 24), note, fill=(80, 80, 80), font=f_small)
        d.text((S + 8, 24), '车上模型输入:灰点 雷达记忆  红点 本帧雷达', fill=(80, 80, 80), font=f_small)
        dt = num[k + 1, 0] - t if k + 1 < len(num) else 1.5
        for _ in range(max(1, round(dt * FPS))):
            ff.stdin.write(np.asarray(frame).tobytes())
    ff.stdin.close()
    ff.wait()
    return len(num), num[-1, 0] - t0, float(num[:, 6].max()), note


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('segs', nargs='+', type=Path, help='logbook 段目录(含 frames.npz)')
    ap.add_argument('--map', help='实车锥桶图 maps/<name>.yaml(inner / outer);给了就对齐到地图画')
    ap.add_argument('--out-dir', type=Path, help='视频目录,默认放在各段目录里')
    ap.add_argument('--min-frames', type=int, default=20, help='帧数少于这个的段跳过')
    a = ap.parse_args()
    cones = None
    if a.map:
        mp = yaml.safe_load(open(a.map, encoding='utf-8'))
        cones = np.array(mp['inner'] + mp['outer'], float)
    for seg in a.segs:
        if not (seg / 'frames.npz').exists() or len(np.load(seg / 'frames.npz')['num']) < a.min_frames:
            continue
        out_dir = a.out_dir or seg
        out_dir.mkdir(parents=True, exist_ok=True)
        out = out_dir / f'{seg.parent.name}_{seg.name}_replay.mp4'
        n, span, vmax, note = render(seg, out, cones)
        print(f'{out}: {n} 帧 {span:.1f}s 最高 {vmax:.2f} m/s  {note}')


if __name__ == '__main__':
    main()
