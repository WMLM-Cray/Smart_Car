"""BEV 模型影子模式:和 road_car_node 同一套运行时(BevRuntime),吃同样的话题,但**不发任何控制**。
网页 http://<车IP>:8090/ 实时看模型的输入图(BEV)和它想打的方向、想开的速度。人推车 / 遥控开,
看模型判断对不对,再决定让它真开。

    source /opt/ros/humble/setup.bash
    python3 -m vehicle.bev_shadow MODEL.npz --speed-cap 0.7

bev_v2 模型要同时开 vehicle/ground_seg_node.py(发 /ground_free)。
图:车头朝上,后轴在下方 1/4 处,每格 10 cm(车前 6 m、车后 2 m、左右各 4 m)。
灰 = 雷达记忆,红 = 本帧雷达,橙 = 锥桶,绿 = 可行驶地面。黄线 = 模型想打的方向(按转角画 1.5 m 弧),
灰圈 = 离后轴 1/2/3/4 m。右边并排是摄像头画面叠地面分割(ground_seg_node 的 /ground_debug,2 Hz)。
"""
from __future__ import annotations

import argparse
import http.server
import math
import threading

import numpy as np

from common.params import CarParams
from vehicle.road_car_node import CONTROL_PERIOD, ENABLE_TIMEOUT, ENC_TOPIC, FREE_TOPIC, SENSOR_TIMEOUT, load_policy

PORT = 8090
S = 8                                   # 每个 BEV 格子画成 S×S 像素
LAYERS = [(5, (60, 150, 40)), (0, (150, 150, 150)), (3, (230, 110, 60)), (2, (40, 210, 230)),
          (1, (0, 140, 255)), (4, (40, 40, 230))]           # BGR;后画的盖先画的


def render(cv2, img, steer, target, speed, stale, free_n, p):
    n = img.shape[1]
    rgb = np.full((n, n, 3), 255, np.uint8)
    for ch, col in LAYERS:
        if ch < img.shape[0]:
            rgb[img[ch] > 0.5] = col
    v = cv2.resize(rgb, (n * S, n * S), interpolation=cv2.INTER_NEAREST)
    ax, ay = n * S // 2, int(n * S * 6 / 8)                 # 后轴
    m = S / 0.1                                             # 像素 / m
    cv2.rectangle(v, (int(ax - 0.15 * m), int(ay - 0.42 * m)), (int(ax + 0.15 * m), int(ay + 0.12 * m)), (0, 0, 0), 2)
    if steer is not None:                                   # 按自行车模型画 1 m 的轨迹
        k = math.tan(steer) / p.wheelbase
        pts = []
        for s in np.linspace(0, 1.5, 30):
            if abs(k) < 1e-6:
                x, y = s, 0.0
            else:
                x, y = math.sin(k * s) / k, (1 - math.cos(k * s)) / k
            pts.append((int(ax - y * m), int(ay - x * m)))
        cv2.polylines(v, [np.array(pts)], False, (0, 220, 255), 3)
    for r in (1, 2, 3, 4):                                  # 距离圈(以后轴为圆心)
        cv2.circle(v, (ax, ay), int(r * m), (200, 200, 200), 1)
        cv2.putText(v, f'{r}m', (ax + 4, int(ay - r * m) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 120), 1)
    y0 = n * S - 92                                         # 图例
    for k, (name, col) in enumerate((('lidar now', (40, 40, 230)), ('lidar memory 20s', (150, 150, 150)),
                                     ('camera floor', (60, 150, 40)), ('model path', (0, 220, 255)))):
        cv2.rectangle(v, (8, y0 + 22 * k), (24, y0 + 22 * k + 14), col, -1)
        cv2.putText(v, name, (30, y0 + 22 * k + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.rectangle(v, (0, 0), (n * S, 30), (0, 0, 0), -1)
    txt = ('传感器过期: ' + ','.join(stale)) if stale else \
        f'steer {math.degrees(steer):+5.1f} deg  target {target:.2f} m/s  speed {speed:.2f}  free {free_n}'
    cv2.putText(v, txt if not stale else 'STALE ' + ','.join(stale), (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 255), 1, cv2.LINE_AA)
    return v


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('model')
    ap.add_argument('--speed-cap', type=float, required=True)
    a = ap.parse_args(argv)

    import cv2
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, Imu, LaserScan, PointCloud2

    from vehicle.ground_seg_node import DEBUG_TOPIC

    from common.bev_policy import bev_image
    from common.bev_runtime import BevRuntime

    pol = load_policy(a.model)
    p = CarParams()
    rt = BevRuntime(pol, a.speed_cap, SENSOR_TIMEOUT, ENABLE_TIMEOUT, p)
    rclpy.init(args=[])
    node = Node('bev_shadow')
    now = lambda: node.get_clock().now().nanoseconds * 1e-9
    lock = threading.Lock()
    st = {'jpg': None, 'free_n': 0, 'cam': None}
    cond = threading.Condition()

    def locked(f):
        def g(m):
            with lock:
                f(m)
        return g

    node.create_subscription(LaserScan, '/scan', locked(lambda m: rt.on_scan(m.ranges, m.angle_min, m.angle_increment, now())), 1)
    node.create_subscription(Odometry, ENC_TOPIC, locked(lambda m: rt.on_odometry(m.twist.twist.linear.x, now())), 10)
    node.create_subscription(Imu, '/IMU_data', locked(lambda m: rt.on_imu(m.angular_velocity.z, now())), 10)

    def on_free(m):
        xy = np.frombuffer(bytes(m.data), np.float32).reshape(-1, 2).astype(np.float64)
        st['free_n'] = len(xy)
        rt.on_free(xy, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, now())
    if pol.free:
        node.create_subscription(PointCloud2, FREE_TOPIC, locked(on_free), 2)

    def on_cam(m):
        st['cam'] = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
    node.create_subscription(CompressedImage, DEBUG_TOPIC, on_cam, 1)

    def tick():
        t = now()
        with lock:
            rt.set_enabled(True, t)                         # 内部心跳:只为让运行时出决策,什么都不发
            stale = rt.stale_sensors(t)
            d = rt.decide(t)
            img = bev_image(rt.bev, rt.odom.pose, t, rt.scan, rt.angles, rt.scan_pose, free=pol.free) \
                if rt.scan is not None else np.zeros((6, 80, 80), np.float32)
            speed = rt.speed
        steer, target = (d if isinstance(d, tuple) else (None, 0.0))
        v = render(cv2, img, steer, target, speed, stale, st['free_n'], p)
        cam = st['cam']
        if cam is not None:                                 # 右边并排:摄像头画面 + 地面分割(2 Hz)
            cam = cv2.resize(cam, (v.shape[0] * 4 // 3, v.shape[0]))
            v = np.hstack([v, cam])
        jpg = cv2.imencode('.jpg', v, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        with cond:
            st['jpg'] = jpg
            cond.notify_all()

    node.create_timer(CONTROL_PERIOD, tick)

    class Hd(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            self.send_response(200)
            if self.path != '/stream':
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.end_headers()
                self.wfile.write("<html><head><meta name='viewport' content='width=device-width'></head>"
                                 "<body style='margin:0;background:#222'><img src='/stream' style='width:100%;max-width:1500px'>"
                                 "</body></html>".encode())
                return
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=f')
            self.end_headers()
            try:
                while True:
                    with cond:
                        cond.wait()
                        j = st['jpg']
                    self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n\r\n' + j + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass

    srv = http.server.ThreadingHTTPServer(('0.0.0.0', PORT), Hd)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(f'影子模式 {a.model}(不发控制),http://<车IP>:{PORT}/')
    rclpy.spin(node)


if __name__ == '__main__':
    main()
