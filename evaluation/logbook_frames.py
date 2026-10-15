"""把 road_car_node 开车时记的逐帧回看(段目录里的 frames.npz + frames/cam_NNNN.jpg)画成视频:
左 = 模型这一拍的 BEV 输入(和影子模式网页同一画法,黄线 = 模型想走的 1.5 m)、右 = 最近一张地面分割调试图
(左上角是在线姿态状态),顶栏 = 时间、事件(guard 截停 / stale 过期)。

    python -m evaluation.logbook_frames runs/drv_1015/logbook_1014_201802/seg_02 [--out seg_02.mp4] [--sheet seg_02.jpg]

--sheet 另存一张缩略总览(每隔几帧取一帧)。实时速度播放(每帧 = 记录间隔)。
"""
from __future__ import annotations

import argparse
import math
import os

import numpy as np


def frames(seg_dir):
    z = np.load(os.path.join(seg_dir, 'frames.npz'))
    return z['num'], z['event'], z['bev'].astype(np.float32), z['cam']


def compose(cv2, i, num, event, bev, cam, seg_dir, p):
    from vehicle.bev_shadow import render
    t, x, y, yaw, steer, target, speed = num[i]
    v = render(cv2, bev[i], None, None if math.isnan(steer) else steer, target, speed,
               [event[i]] if event[i] == 'stale' else [], int((bev[i][5] > 0.5).sum()) if bev.shape[1] > 5 else 0, p)
    h = v.shape[0]
    img = None
    if cam[i] >= 0:
        img = cv2.imread(os.path.join(seg_dir, 'frames', f'cam_{int(cam[i]):04d}.jpg'))
    right = np.full((h, h * 4 // 3, 3), 40, np.uint8) if img is None else cv2.resize(img, (h * 4 // 3, h))
    out = np.hstack([v, right])
    bar = f't {t - num[0, 0]:5.1f}s  {event[i] or "drive"}'
    col = (0, 0, 255) if event[i] in ('guard', 'stuck', 'stale') else (0, 160, 0)
    cv2.rectangle(out, (v.shape[1], 0), (out.shape[1], 30), (0, 0, 0), -1)
    cv2.putText(out, bar, (v.shape[1] + 8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('seg_dir')
    ap.add_argument('--out', default=None, help='视频路径,默认 段目录/frames.mp4')
    ap.add_argument('--sheet', default=None, help='缩略总览 jpg')
    a = ap.parse_args(argv)
    import cv2
    from common.params import CarParams
    p = CarParams()
    num, event, bev, cam = frames(a.seg_dir)
    imgs = [compose(cv2, i, num, event, bev, cam, a.seg_dir, p) for i in range(len(num))]
    out = a.out or os.path.join(a.seg_dir, 'frames.mp4')
    fps = 1.0 / max(1e-3, float(np.median(np.diff(num[:, 0])))) if len(num) > 1 else 3.0
    h, w = imgs[0].shape[:2]
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    for im in imgs:
        vw.write(im)
    vw.release()
    print(f'{len(imgs)} 帧 -> {out}({fps:.1f} fps)')
    if a.sheet:
        step = max(1, len(imgs) // 12)
        tiles = [cv2.resize(im, (w // 2, h // 2)) for im in imgs[::step]]
        while len(tiles) % 3:
            tiles.append(np.zeros_like(tiles[0]))
        cv2.imwrite(a.sheet, np.vstack([np.hstack(tiles[k:k + 3]) for k in range(0, len(tiles), 3)]),
                    [cv2.IMWRITE_JPEG_QUALITY, 85])
        print('总览 ->', a.sheet)


if __name__ == '__main__':
    main()
