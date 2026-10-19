"""BEV 模型跑一局,录成视频:左 = 俯视真值(路、锥桶、墙、车和轨迹),右 = 模型此刻吃的输入图。

    python -m evaluation.bev_video --model runs/bev_v2_a/candidate_011.npz --track A --cap 1.8 --out runs/video_A.mp4
    python -m evaluation.bev_video --model ... --level L6f --seed 93000001 --out runs/video_L6f.mp4

右图(common/bev_view):灰点 = 雷达记忆,红点 = 本帧雷达,橙圆 = 锥桶,黄点 = 胶带,蓝点 = 区域,绿点 = 可行驶地面(bev_v2)。
控制 20 Hz,每步一帧,20 fps = 实时。需要 PATH 里有 ffmpeg。
"""
from __future__ import annotations

import argparse
import subprocess
from dataclasses import replace

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from common import bev_view
from common.bev_policy import BevPolicy
from simulation.fastsim.road_env import CONTROL_DT, Plant
from training.bev_env import BevEnv

S = 480                                           # 每个面板边长(像素)


def font(size):
    try:
        return ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', size)
    except OSError:
        return ImageFont.load_default()


def world_panel(road, trail, pose, view):
    """view = (cx, cy, 半边长 m):以 (cx,cy) 为中心的俯视窗口。"""
    cx, cy, r = view
    k = S / (2 * r)
    to = lambda p: ((p[..., 0] - cx) * k + S / 2, S / 2 - (p[..., 1] - cy) * k)
    im = Image.new('RGB', (S, S), (250, 250, 250))
    d = ImageDraw.Draw(im)
    for side in (1, -1):                                       # 路沿(按左右半宽)
        half = road.half_l if side > 0 else road.half_r
        e = road.centre + side * road.normal * half[:, None]
        u, v = to(e)
        d.line(list(zip(u, v)), fill=(200, 200, 200), width=1)
    for w in road.walls:
        u, v = to(np.asarray(w))
        d.line(list(zip(u, v)), fill=(60, 60, 60), width=3)
    cr = road.cone_radius * k
    for u, v in zip(*to(road.cones)):
        d.ellipse([u - cr, v - cr, u + cr, v + cr], fill=(255, 140, 0))
    if len(trail) > 1:
        u, v = to(np.asarray(trail))
        d.line(list(zip(u, v)), fill=(30, 90, 220), width=2)
    x, y, yaw = pose                                           # 车:后轴为原点,长 0.5 宽 0.3
    c, s = np.cos(yaw), np.sin(yaw)
    body = np.array([[-0.15, -0.15], [0.35, -0.15], [0.35, 0.15], [-0.15, 0.15]])
    w = np.c_[x + body[:, 0] * c - body[:, 1] * s, y + body[:, 0] * s + body[:, 1] * c]
    u, v = to(w)
    d.polygon(list(zip(u, v)), fill=(20, 20, 20))
    return im


def bev_panel(img, cones_body):
    return Image.fromarray(bev_view.render(img, cones_body, px=S // img.shape[1]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', required=True)
    ap.add_argument('--track', help='官方赛道 A-D')
    ap.add_argument('--level', help='随机路段级别,如 L6f')
    ap.add_argument('--seed', type=int, default=95_000_000)
    ap.add_argument('--cap', type=float, default=1.8)
    ap.add_argument('--max-t', type=float, default=60.0)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    pol = BevPolicy(a.model)
    rng = np.random.default_rng(a.seed)
    start = (0, 0.0, 0.0)
    if a.track:
        from evaluation.road_track import official
        road, _ = official(a.track, True)
        pl = replace(Plant.sample(rng, pol.enc), cap=a.cap)
        title = f'赛道 {a.track}'
    else:
        from training.road_check import sample_episode
        road, pl, start, _, _ = sample_episode(a.seed, a.level, enc=pol.enc, cap_max=a.cap)
        title = f'随机路段 {a.level} seed {a.seed}'
    benv = BevEnv(road, pl, a.seed, start, noise=True, free=pol.free, near=pol.near)

    lo, hi = road.centre.min(0), road.centre.max(0)
    whole = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, max(hi - lo) / 2 + 1.0)
    ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                           '-s', f'{2 * S}x{S + 40}', '-r', str(round(1 / CONTROL_DT)), '-i', '-',
                           '-pix_fmt', 'yuv420p', '-vcodec', 'libx264', '-crf', '23', a.out], stdin=subprocess.PIPE)
    f_big, f_small = font(16), font(13)
    trail, why = [], None
    while why is None and benv.env.t < a.max_t:
        obs = benv.observation()
        img = obs[0]
        act = pol(*obs)
        x, y, yaw = benv.pose
        trail.append((x, y))
        frame = Image.new('RGB', (2 * S, S + 40), (255, 255, 255))
        frame.paste(world_panel(road, trail, benv.pose, whole), (0, 40))
        frame.paste(bev_panel(img, bev_view.cones_in_body(benv.bev, benv.pose, benv.env.t)), (S, 40))
        d = ImageDraw.Draw(frame)
        d.text((8, 4), f'{title}  {pol.schema if hasattr(pol, "schema") else ""}  t {benv.env.t:5.1f}s  '
                       f'v {benv.env.car.v:4.2f} m/s  限速 {pl.cap:.1f}  转向 {act[0]:+.2f}', fill=(0, 0, 0), font=f_big)
        d.text((S + 8, 24), '模型输入:灰点 雷达记忆  红点 本帧雷达  橙圆 锥桶  绿点 可行驶地面', fill=(80, 80, 80), font=f_small)
        ff.stdin.write(np.asarray(frame).tobytes())
        why = benv.step(act)
    ff.stdin.close()
    ff.wait()
    print(f'{a.out}: {why or "到时"}  t={benv.env.t:.1f}s')


if __name__ == '__main__':
    main()
