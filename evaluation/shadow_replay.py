"""影子模式(电脑上那一半):把车上 vehicle/shadow_record.py 录的原始话题按时间顺序喂给 BevRuntime(和
road_car_node 同一套),每 CONTROL_PERIOD 出一次决策 —— 得到"模型当时会怎么开",车上不用跑模型。

    python -m evaluation.shadow_replay runs/drives/shadow_1010_160000 models/bev_v2/v2m_013.npz --speed-cap 0.7
    ... --video          # 另存 replay.mp4:BEV 输入 + 模型想走的弧线 + 摄像头画面,10 fps

输出到 <录制目录>/replay/:记录本(vehicle/bev_logbook.py)每段 seg_NN/(seg.npz、map.png、mark_NN.jpg)。
分段:录下的 IMU 判断抱车(PickupDetector,放下静止后开新段,BEV 清空)+ 网页上点的"新段"。
"""
from __future__ import annotations

import argparse
import csv
import glob
import heapq
import math
import os

import numpy as np

from common.bev import body_to_odom, lidar_points
from common.bev_policy import bev_image
from common.bev_runtime import BevRuntime
from common.params import CarParams
from vehicle.bev_logbook import Logbook, PickupDetector
from vehicle.road_car_node import CONTROL_PERIOD, ENABLE_TIMEOUT, SENSOR_TIMEOUT, load_policy

S = 8                                   # 每个 BEV 格子画成 S×S 像素
LAYERS = [(5, (60, 150, 40)), (0, (150, 150, 150)), (3, (230, 110, 60)), (2, (40, 210, 230)),
          (1, (0, 140, 255)), (4, (40, 40, 230))]           # BGR;后画的盖先画的


def render(cv2, img, steer, target, speed, stale, free_n, p):
    """BEV 输入图:车头朝上,后轴在下方 1/4 处,每格 10 cm。灰 = 雷达记忆,红 = 本帧雷达,橙 = 锥桶,
    绿 = 可行驶地面;黄线 = 模型想打的方向(按转角画 1.5 m 弧);灰圈 = 离后轴 1/2/3/4 m。"""
    n = img.shape[1]
    rgb = np.full((n, n, 3), 255, np.uint8)
    for ch, col in LAYERS:
        if ch < img.shape[0]:
            rgb[img[ch] > 0.5] = col
    v = cv2.resize(rgb, (n * S, n * S), interpolation=cv2.INTER_NEAREST)
    ax, ay = n * S // 2, int(n * S * 6 / 8)                 # 后轴
    m = S / 0.1                                             # 像素 / m
    cv2.rectangle(v, (int(ax - 0.15 * m), int(ay - 0.42 * m)), (int(ax + 0.15 * m), int(ay + 0.12 * m)), (0, 0, 0), 2)
    if steer is not None:                                   # 按自行车模型画 1.5 m 的轨迹
        k = math.tan(steer) / p.wheelbase
        pts = []
        for s in np.linspace(0, 1.5, 30):
            x, y = (s, 0.0) if abs(k) < 1e-6 else (math.sin(k * s) / k, (1 - math.cos(k * s)) / k)
            pts.append((int(ax - y * m), int(ay - x * m)))
        cv2.polylines(v, [np.array(pts)], False, (0, 220, 255), 3)
    for r in (1, 2, 3, 4):                                  # 距离圈(以后轴为圆心)
        cv2.circle(v, (ax, ay), int(r * m), (200, 200, 200), 1)
        cv2.putText(v, f'{r}m', (ax + 4, int(ay - r * m) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 120), 1)
    y0 = n * S - 112                                        # 图例
    for k, (name, col) in enumerate((('lidar now', (40, 40, 230)), ('lidar memory 20s', (150, 150, 150)),
                                     ('camera floor', (60, 150, 40)), ('model path', (0, 220, 255)))):
        cv2.rectangle(v, (8, y0 + 22 * k), (24, y0 + 22 * k + 14), col, -1)
        cv2.putText(v, name, (30, y0 + 22 * k + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.rectangle(v, (0, 0), (n * S, 30), (0, 0, 0), -1)
    txt = ('STALE ' + ','.join(stale)) if stale or steer is None else \
        f'steer {math.degrees(steer):+5.1f} deg  target {target:.2f} m/s  speed {speed:.2f}  free {free_n}'
    cv2.putText(v, txt, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return v


def load(root):
    """所有 chunk 拼起来;free 按条拆回 (t, stamp, xy)。"""
    ch = [np.load(f) for f in sorted(glob.glob(os.path.join(root, 'chunk_*.npz')))]
    if not ch:
        raise SystemExit(f'{root} 里没有 chunk_*.npz')
    cat = lambda k: np.concatenate([c[k] for c in ch])
    free = []
    for c in ch:
        xy = np.split(c['free_xy'].astype(np.float64), np.cumsum(c['free_n'])[:-1]) if len(c['free_n']) else []
        free += list(zip(c['free_t'], c['free_stamp'], xy))
    return dict(angle_min=float(ch[0]['angle_min']), angle_increment=float(ch[0]['angle_increment']),
                scan_t=cat('scan_t'), scans=cat('scans'), enc_t=cat('enc_t'), enc_v=cat('enc_v'),
                imu=cat('imu'), free=free)


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding='utf-8') as f:
        return [r for r in csv.reader(f) if r]


def events(d, splits, marks, t0, t1):
    """所有输入按时刻排成一串;同一时刻按 传感器 -> 网页事件 -> 决策 的顺序。"""
    tick = [(t, 3, 'tick', None) for t in np.arange(t0, t1, CONTROL_PERIOD)]
    srcs = [[(t, 0, 'scan', i) for i, t in enumerate(d['scan_t'])],
            [(t, 0, 'enc', i) for i, t in enumerate(d['enc_t'])],
            [(r[0], 0, 'imu', i) for i, r in enumerate(d['imu'])],
            [(f[0], 0, 'free', i) for i, f in enumerate(d['free'])],
            [(t, 1, 'split', None) for t in splits], [(t, 2, 'mark', None) for t in marks], tick]
    return heapq.merge(*[sorted(s, key=lambda e: (e[0], e[1])) for s in srcs], key=lambda e: (e[0], e[1]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('root', help='shadow_record 录的目录')
    ap.add_argument('model')
    ap.add_argument('--speed-cap', type=float, required=True)
    ap.add_argument('--video', action='store_true')
    a = ap.parse_args(argv)
    import cv2

    d = load(a.root)
    ev = read_csv(os.path.join(a.root, 'events.csv'))
    splits = [float(t) for t, k in ev if k == 'split']
    marks = [float(t) for t, k in ev if k == 'mark']
    cams = [(float(t), os.path.join(a.root, 'cam', f'{int(i):05d}.jpg')) for i, t in read_csv(os.path.join(a.root, 'cam.csv'))]
    cam_t = np.array([t for t, _ in cams])

    def cam_at(t):                                          # 不晚于 t 的最近一张
        k = int(np.searchsorted(cam_t, t, 'right')) - 1
        return cv2.imread(cams[k][1]) if k >= 0 else None

    pol = load_policy(a.model)
    p = CarParams()
    rt = BevRuntime(pol, a.speed_cap, SENSOR_TIMEOUT, ENABLE_TIMEOUT, p)
    out = os.path.join(a.root, 'replay')
    book, pick = Logbook(out), PickupDetector()
    am, ai = d['angle_min'], d['angle_increment']
    t0, t1 = float(d['scan_t'][0]), float(d['scan_t'][-1])
    vw, frame_k, free_n = None, 0, 0
    last = dict(steer=None, target=0.0, img=None, stale=[])

    def close(t):
        """当前段画全局图、报一行;接着调用方 pause / split 存盘。"""
        if book.active and book.path:
            os.makedirs(book.dir(), exist_ok=True)
            cv2.imwrite(os.path.join(book.dir(), 'map.png'), book.render(cv2))
            P = np.array(book.path)
            dist = float(np.hypot(*np.diff(P[:, 1:3], axis=0).T).sum()) if len(P) > 1 else 0.0
            print(f'{t - t0:7.1f} s  第 {book.seg} 段结束:{P[-1, 0] - P[0, 0]:4.0f} s,走了 {dist:4.1f} m,'
                  f'标记 {len(book.marks)}')

    def bev_now(t):
        img = bev_image(rt.bev, rt.odom.pose, t, rt.scan, rt.angles, rt.scan_pose, free=pol.free)
        return render(cv2, img, last['steer'], last['target'], rt.speed, last['stale'], free_n, p)

    for t, _, kind, i in events(d, splits, marks, t0, t1):
        t = float(t)
        if kind == 'scan':
            rt.on_scan(d['scans'][i], am, ai, t)
            x, y, yaw = rt.scan_pose
            book.add_lidar(lidar_points(rt.scan, rt.angles, rt.scan_pose, p, rt.bev.lidar_yaw, 8.0),
                           (x + p.lidar_x * math.cos(yaw), y + p.lidar_x * math.sin(yaw)))
        elif kind == 'enc':
            rt.on_odometry(float(d['enc_v'][i]), t)
        elif kind == 'imu':
            _, wx, wy, wz, ax_, ay_, az_ = d['imu'][i]
            rt.on_imu(float(wz), t)
            e = pick.update(wx, wy, ax_, ay_, az_, t)
            if e == 'pickup':
                close(t)
                book.pause()
            elif e == 'settled':
                rt.reset()                                  # 新位置:BEV 和积分起点都清零
                book.resume()
        elif kind == 'free':
            _, stamp, xy = d['free'][i]
            free_n = len(xy)
            rt.on_free(xy, float(stamp), t)
            if len(xy):
                book.add_free(body_to_odom(xy, rt.pose_at(float(stamp))))
        elif kind == 'split':
            close(t)
            book.split()
            rt.reset()
        elif kind == 'mark':
            k = book.mark(t, rt.odom.pose, last['steer'], last['target'], rt.speed, cam_at(t), bev_now(t), cv2)
            print(f'{t - t0:7.1f} s  标记 {k}(第 {book.seg} 段)' if k else f'{t - t0:7.1f} s  标记时车被抱着,没记')
        else:                                               # tick:内部心跳,只为让运行时出决策
            rt.set_enabled(True, t)
            last['stale'] = rt.stale_sensors(t)
            dd = rt.decide(t)
            last['steer'], last['target'] = dd if isinstance(dd, tuple) else (None, 0.0)
            book.add_pose(t, rt.odom.pose, last['steer'], last['target'], rt.speed)
            frame_k += 1
            if a.video and frame_k % 2 == 0 and rt.scan is not None:
                v = bev_now(t)
                cam = cam_at(t)                             # 还没画面时补黑,视频每帧要一样大
                cam = np.zeros((v.shape[0], v.shape[0] * 4 // 3, 3), np.uint8) if cam is None else                     cv2.resize(cam, (v.shape[0] * 4 // 3, v.shape[0]))
                v = np.hstack([v, cam])
                cv2.putText(v, f'{t - t0:6.1f} s  seg {book.seg}{"" if book.active else " (held)"}',
                            (8, v.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 60, 0), 1, cv2.LINE_AA)
                if vw is None:
                    vw = cv2.VideoWriter(os.path.join(out, 'replay.mp4'), cv2.VideoWriter_fourcc(*'mp4v'),
                                         0.5 / CONTROL_PERIOD, (v.shape[1], v.shape[0]))
                vw.write(v)
    close(t1)
    book.save()
    if vw is not None:
        vw.release()
    print(f'输出在 {out}')


if __name__ == '__main__':
    main()
