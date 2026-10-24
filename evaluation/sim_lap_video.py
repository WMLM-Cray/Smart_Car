"""仿真按车上条件跑整圈出点阵视频(画法同 evaluation/bev_video):左 = 全局赛道真值 + 轨迹,右 = 模型输入 BEV,
只画模型真能看到的雷达层(灰 记忆 / 红 本帧),不画真值锥桶。和实车回看 evaluation/logbook_replay 对照着看。

    python -m evaluation.sim_lap_video --model runs/lidar_esc_c005.npz --track realA_1023 --cap 2.8 --seed 95000000
    python -m evaluation.sim_lap_video --model ... --track A --cap 1.8 --laps 2 --guard

车上条件同 evaluation/bev_track --car --no-seg(纯雷达、无 YOLO),默认不截停(--guard 打开);
官方赛道 A-D 或 maps/<name>.yaml 实车赛道。跑满 --laps 圈或撞桶 / 出界为止。2026-10-24 从 runs/lap1_video 整理入库。
"""
from __future__ import annotations

import argparse
import subprocess
from dataclasses import replace

import numpy as np
from PIL import Image, ImageDraw

from common import bev_view
from common.bev_policy import BevPolicy
from common.road_policy import BRAKE_CMD, action_to_speed
from evaluation.bev_track import CAR
from evaluation.bev_video import S, font, world_panel
from evaluation.road_track import official, real_track
from simulation.fastsim.road_env import CONTROL_DT, Plant
from training.bev_env import BevEnv


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', required=True)
    ap.add_argument('--track', default='realA_1023', help='A-D 或 maps/<name>.yaml 的实车赛道名')
    ap.add_argument('--cap', type=float, default=1.8)
    ap.add_argument('--seed', type=int, default=95_000_000)
    ap.add_argument('--laps', type=int, default=1)
    ap.add_argument('--guard', action='store_true', help='截停(车上默认 --no-guard 关)')
    ap.add_argument('--out', help='默认 runs/sim_lap_<track>_cap<cap>_s<seed>.mp4')
    a = ap.parse_args()

    pol = BevPolicy(a.model)
    road, lap_len = official(a.track, True) if a.track in 'ABCD' else real_track(a.track)
    pl = replace(Plant.sample(np.random.default_rng(a.seed), pol.enc), cap=a.cap)
    benv = BevEnv(road, pl, a.seed, noise=True, free=pol.free, near=pol.near,
                  **{**CAR, 'guard': a.guard, 'no_seg': True})
    lo, hi = road.cones.min(0), road.cones.max(0)             # 按锥桶取景(centre 是多圈拼接)
    whole = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, max(hi - lo) / 2 + 0.5)
    out = a.out or f'runs/sim_lap_{a.track}_cap{a.cap}_s{a.seed}.mp4'
    ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                           '-s', f'{2 * S}x{S + 40}', '-r', str(round(1 / CONTROL_DT)), '-i', '-',
                           '-pix_fmt', 'yuv420p', '-vcodec', 'libx264', '-crf', '20', out], stdin=subprocess.PIPE)
    f_big, f_small = font(16), font(13)
    trail, laps, why, frame = [], [], None, None
    no_cones = np.zeros((0, 2))
    while why is None:
        obs = benv.observation()
        img = obs[0].copy()
        img[1:4] = 0                                            # 纯雷达:锥桶 / 胶带 / 区域层本来就空
        if img.shape[0] > 5:
            img[5] = 0                                          # FREE 层(--no-seg 恒空)
        act = pol(*obs)
        trail.append(benv.pose[:2])
        tv = action_to_speed(float(act[1]), pol.enc)
        frame = Image.new('RGB', (2 * S, S + 40), (255, 255, 255))
        frame.paste(world_panel(road, trail, benv.pose, whole), (0, 40))
        frame.paste(Image.fromarray(bev_view.render(img, no_cones, px=S // img.shape[1])), (S, 40))
        d = ImageDraw.Draw(frame)
        d.text((8, 4), f'{a.track} 第 {len(laps) + 1} 圈  t {benv.env.t:5.1f}s  v {benv.env.car.v:4.2f}  '
                       f'要 {"刹车" if tv == BRAKE_CMD else f"{tv:4.2f}"}  限速 {a.cap:.1f}  转向 {act[0]:+.2f}  '
                       f'打滑 {getattr(benv.env.car, "slip", 0.0):.0%}', fill=(0, 0, 0), font=f_big)
        d.text((S + 8, 24), '模型视角:灰点 雷达记忆  红点 本帧雷达(无真值)', fill=(80, 80, 80), font=f_small)
        ff.stdin.write(np.asarray(frame).tobytes())
        why = benv.step(act)
        if road.s[benv.env.idx] >= lap_len * (len(laps) + 1):
            laps.append(benv.env.t)
            if why is None and len(laps) >= a.laps:
                why = f'跑完 {a.laps} 圈'
    lap_t = [round(float(x), 1) for x in np.diff([0.0] + laps)]
    d = ImageDraw.Draw(frame)
    d.text((8, S + 10), f'结果:{why}  t={benv.env.t:.1f}s  圈时 {lap_t}', fill=(220, 0, 0), font=font(22))
    for _ in range(30):                                         # 末帧停 1.5 s
        ff.stdin.write(np.asarray(frame).tobytes())
    ff.stdin.close()
    ff.wait()
    print(f'{out}: {why}  t={benv.env.t:.1f}s  圈时 {lap_t}')


if __name__ == '__main__':
    main()
