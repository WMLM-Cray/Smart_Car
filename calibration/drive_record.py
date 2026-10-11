"""人开车时一直录:摄像头帧 + 雷达 + 编码器车速 + 陀螺偏航角速度,全部带同一时钟的时间戳。
只订阅、只读摄像头,不发任何控制。离线用途:相机外参标定(雷达墙线 vs 图像墙根)、地面分割 / BEV 回放。

    source /opt/ros/humble/setup.bash
    python3 -m calibration.drive_record ~/drives/lap1          # Ctrl-C 结束

目录里:frames/NNNNN.jpg(JPEG 质量 95,640×480,默认 5 Hz)、frames.csv(序号, 时刻 s)、
sensors.npz(scan_t, scans[n, 1667], angle_min, angle_increment, enc_t, enc_v, imu_t, imu_wz, imu_acc[n, 3])。
imu_acc = 线加速度 x y z(静止时就是重力方向,用来看车身俯仰 / 横滚)。
每 30 s 存一次 sensors.npz,中途断电最多丢 30 s 传感器数据。
"""
from __future__ import annotations

import argparse
import os
import signal
import threading
import time

import numpy as np


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out')
    ap.add_argument('--fps', type=float, default=5.0)
    a = ap.parse_args(argv)

    import cv2
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, LaserScan

    os.makedirs(os.path.join(a.out, 'frames'), exist_ok=True)
    rclpy.init(args=[])
    node = Node('drive_record')
    now = lambda: node.get_clock().now().nanoseconds * 1e-9
    lock = threading.Lock()
    S = dict(scan_t=[], scans=[], enc_t=[], enc_v=[], imu_t=[], imu_wz=[], imu_acc=[], meta=None)

    def on_scan(m):
        with lock:
            S['scan_t'].append(now())
            S['scans'].append(np.asarray(m.ranges, np.float32))
            S['meta'] = (m.angle_min, m.angle_increment)

    def on_enc(m):
        with lock:
            S['enc_t'].append(now())
            S['enc_v'].append(m.twist.twist.linear.x)

    def on_imu(m):
        with lock:
            S['imu_t'].append(now())
            S['imu_wz'].append(m.angular_velocity.z)
            a = m.linear_acceleration
            S['imu_acc'].append((a.x, a.y, a.z))

    node.create_subscription(LaserScan, '/scan', on_scan, 5)
    node.create_subscription(Odometry, '/encoder', on_enc, 20)
    node.create_subscription(Imu, '/IMU_data', on_imu, 20)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def save():
        with lock:
            if not S['scans']:
                return
            d = {k: np.asarray(v) for k, v in S.items() if k != 'meta'}
            am, ai = S['meta']
        tmp = os.path.join(a.out, 'sensors.tmp.npz')
        np.savez(tmp, angle_min=am, angle_increment=ai, **d)
        os.replace(tmp, os.path.join(a.out, 'sensors.npz'))

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    csv = open(os.path.join(a.out, 'frames.csv'), 'a', buffering=1)
    i, last_save, next_t = 0, time.monotonic(), 0.0
    print(f'录到 {a.out},Ctrl-C 结束', flush=True)
    while not stop.is_set():
        ok, f = cap.read()                       # 一直读,不让缓冲攒旧帧
        t = now()
        if not ok or t < next_t:
            continue
        next_t = t + 1.0 / a.fps
        i += 1
        cv2.imwrite(os.path.join(a.out, 'frames', f'{i:05d}.jpg'), f, [cv2.IMWRITE_JPEG_QUALITY, 95])
        csv.write(f'{i},{t:.4f}\n')
        if time.monotonic() - last_save > 30:
            save()
            last_save = time.monotonic()
            print(f'{i} 帧,雷达 {len(S["scans"])} 帧', flush=True)
    save()
    print(f'结束:{i} 帧,雷达 {len(S["scans"])} 帧,编码器 {len(S["enc_t"])},IMU {len(S["imu_t"])}', flush=True)
    os._exit(0)


if __name__ == '__main__':
    main()
