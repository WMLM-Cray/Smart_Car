"""BEV 模型影子模式:和 road_car_node 同一套运行时(BevRuntime),吃同样的话题,但**不发任何控制**。
网页 http://<车IP>:8090/ 实时看模型的输入图(BEV)和它想打的方向、想开的速度。人推车 / 遥控开,
看模型判断对不对,再决定让它真开。

    source /opt/ros/humble/setup.bash
    source ~/shadow_feed_ws/install/setup.bash && ros2 run shadow_feed shadow_feed &     # 先开(C++,见下)
    python3 -m vehicle.bev_shadow MODEL.npz --speed-cap 0.7

bev_v2 模型要同时开 vehicle/ground_seg_node.py(发 /ground_free)。
图:车头朝上,后轴在下方 1/4 处,每格 10 cm(车前 6 m、车后 2 m、左右各 4 m)。
灰点 = 雷达记忆,红点 = 本帧雷达,橙圆 = 锥桶,绿点 = 可行驶地面。黄线 = 模型想打的方向(按转角画 1.5 m 弧),
灰圈 = 离后轴 1/2/3/4 m。右边并排是摄像头画面叠地面分割(ground_seg_node 的 /ground_debug,2 Hz)。

问题记录本(vehicle/bev_logbook.py):网页上"标记问题"存下这一刻的现场,"本段全局地图"看这一段的全局图;
抱起车自动分段(放下静止 1.5 s 开新段,BEV 一起清空),"新段"手动分。存到 --log 目录。

省 CPU 的做法(原来车上实测占 60-86% 一个核):
- IMU(100 Hz)、编码器(50 Hz)不在 Python 里逐条收(rclpy 每条 ~1 ms),由 C++ 的 vehicle/shadow_feed 攒成
  20 Hz 一批;所有输入按接收时刻排好序、晚 LAG 秒再喂给运行时,和逐条收的处理顺序一样。
- 模型每 DECIDE_DT 出一次决策(BEV 模型拍与拍之间没有状态,少算几拍不改变每拍的结果)。
- 网页有人看时才画图、编 JPEG(每 VIEW_DT 一张),BEV 图直接用决策时算好的那张。
"""
from __future__ import annotations

import argparse
import http.server
import math
import os
import threading
import time

import numpy as np

from common import bev_view
from common.params import CarParams
from vehicle.road_car_node import ENABLE_TIMEOUT, FREE_TOPIC, SENSOR_TIMEOUT, load_policy

PORT = 8090
LAG = 0.12              # 输入攒这么久再按时间顺序处理(各路消息到 Python 的先后不一定按接收时刻)
DECIDE_DT = 0.1         # 决策周期 s
VIEW_DT = 0.2           # 网页画面周期 s
S = 8                                   # 每个 BEV 格子(10 cm)画成 S×S 像素
PAGE = """<html><head><meta name='viewport' content='width=device-width'><meta charset='utf-8'></head>
<body style='margin:0;background:#222;color:#eee;font-family:sans-serif'>
<img src='/stream' style='width:100%;max-width:800px;display:block'>
<div style='padding:8px'>
<button onclick="post('/mark')" style='font-size:22px;padding:10px 28px'>标记问题</button>
<button onclick="post('/split')" style='font-size:22px;padding:10px 28px'>新段</button>
<a href='/map' target='_blank' style='font-size:22px;color:#9cf;margin-left:16px'>本段全局地图</a>
<span id='s' style='margin-left:16px;font-size:18px'></span></div>
<script>function post(u){fetch(u,{method:'POST'}).then(r=>r.text()).then(t=>{document.getElementById('s').innerText=t})}</script>
</body></html>""".encode()


def render(cv2, img, cones_body, steer, target, speed, stale, free_n, p):
    """底图用 common/bev_view(和 evaluation/bev_video 同一画法),再叠决策弧线、距离圈、图例、状态栏。"""
    n = img.shape[1]
    v = np.ascontiguousarray(bev_view.render(img, cones_body, px=S)[..., ::-1])     # RGB -> BGR
    ax, ay = n * S // 2, int(n * S * 6 / 8)                 # 后轴
    m = S / 0.1                                             # 像素 / m
    for r in (1, 2, 3, 4):                                  # 距离圈(以后轴为圆心)
        cv2.circle(v, (ax, ay), int(r * m), (225, 225, 225), 1)
        cv2.putText(v, f'{r}m', (ax + 4, int(ay - r * m) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
    if steer is not None:                                   # 按自行车模型画 1.5 m 的轨迹
        k = math.tan(steer) / p.wheelbase
        pts = []
        for s in np.linspace(0, 1.5, 30):
            x, y = (s, 0.0) if abs(k) < 1e-6 else (math.sin(k * s) / k, (1 - math.cos(k * s)) / k)
            pts.append((int(ax - y * m), int(ay - x * m)))
        cv2.polylines(v, [np.array(pts)], False, (0, 200, 255), 3)
    y0 = n * S - 124                                        # 图例
    for k, (name, col) in enumerate((('lidar now', (40, 40, 230)), ('lidar memory 20s', (150, 150, 150)),
                                     ('camera floor', (40, 150, 60)), ('cone', (0, 140, 255)),
                                     ('model path', (0, 200, 255)))):
        cv2.circle(v, (16, y0 + 22 * k + 7), 6, col, -1)
        cv2.putText(v, name, (30, y0 + 22 * k + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.rectangle(v, (0, 0), (n * S, 30), (0, 0, 0), -1)
    txt = ('STALE ' + ','.join(stale)) if stale or steer is None else         f'steer {math.degrees(steer):+5.1f} deg  target {target:.2f} m/s  speed {speed:.2f}  free {free_n}'
    cv2.putText(v, txt, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return v


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('model')
    ap.add_argument('--speed-cap', type=float, required=True)
    ap.add_argument('--log', default=None, help='记录本目录;默认 ~/drives/logbook_<时间>')
    ap.add_argument('--ground-size', type=int, nargs=2, default=(512, 512), metavar=('H', 'W'),
                    help='地面分割模型的输入尺寸(算摄像头视野用,和 ground_seg_node 一致)')
    a = ap.parse_args(argv)

    import cv2
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, LaserScan, PointCloud2
    from std_msgs.msg import Float64MultiArray

    from common.bev import lidar_points
    from common.bev_runtime import BevRuntime
    from common.ground_proj import GroundLut
    from vehicle.bev_logbook import Logbook, PickupDetector
    from vehicle.ground_seg_node import DEBUG_TOPIC

    pol = load_policy(a.model)
    p = CarParams()
    rt = BevRuntime(pol, a.speed_cap, SENSOR_TIMEOUT, ENABLE_TIMEOUT, p)
    lut = GroundLut(a.ground_size[0] // 4, a.ground_size[1] // 4, p, 3.0)
    fov = lut.points(np.ones(lut.valid.shape, bool))      # 摄像头视野里的全部地面格(车体系)
    book = Logbook(a.log or os.path.expanduser(time.strftime('~/drives/logbook_%m%d_%H%M%S')), fov_body=fov)
    pick = PickupDetector()
    rclpy.init(args=[])
    node = Node('bev_shadow')
    log = node.get_logger()
    now = lambda: node.get_clock().now().nanoseconds * 1e-9
    lock = threading.Lock()
    pending = []                                            # (接收时刻, 类型, 数据),攒过 LAG 再处理
    st = {'jpg': None, 'free_n': 0, 'cam': None, 'd': (None, 0.0), 'pose': (0.0, 0.0, 0.0),
          'speed': 0.0, 'stale': [], 'msg': '', 'viewers': 0, 'next_decide': 0.0, 'next_view': 0.0, 'feed_t': 0.0}
    cond = threading.Condition()

    def on_scan(m):
        r = np.asarray(m.ranges, np.float32)
        with lock:
            pending.append((now(), 'scan', (r, m.angle_min, m.angle_increment)))

    def on_free(m):
        xy = np.frombuffer(bytes(m.data), np.float32).reshape(-1, 2).astype(np.float64)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with lock:
            pending.append((now(), 'free', (xy, stamp)))

    def on_cam(m):
        st['cam'] = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)   # 2 Hz,标记时要用

    def handle(t, kind, x):
        if kind == 'scan':
            rt.on_scan(x[0], x[1], x[2], t)
            sx, sy, yaw = rt.scan_pose
            book.add_lidar(lidar_points(rt.scan, rt.angles, rt.scan_pose, p, rt.bev.lidar_yaw, 8.0),
                           (sx + p.lidar_x * math.cos(yaw), sy + p.lidar_x * math.sin(yaw)))
        elif kind == 'enc':
            rt.on_odometry(x, t)
        elif kind == 'imu':
            wx, wy, wz, ax, ay, az = x
            rt.on_imu(wz, t)
            ev = pick.update(wx, wy, ax, ay, az, t)
            if ev == 'pickup':
                book.pause()
                st['msg'] = f'picked up, seg {book.seg} saved'
                log.info(f'车被抱起,第 {book.seg} 段已存,放下静止后开新段')
            elif ev == 'settled':
                rt.reset()                                  # 新位置:BEV 和积分起点都清零
                book.resume()
                st['msg'] = f'seg {book.seg} started'
                log.info(f'放下了,开第 {book.seg} 段')
        elif kind == 'free':
            xy, stamp = x
            st['free_n'] = len(xy)
            rt.on_free(xy, stamp, t)
            book.add_free(xy, rt.pose_at(stamp))
        if t >= st['next_decide']:
            st['next_decide'] = max(st['next_decide'] + DECIDE_DT, t)
            rt.set_enabled(True, t)                         # 内部心跳:只为让运行时出决策,什么都不发
            st['stale'] = rt.stale_sensors(t)
            d = rt.decide(t)
            steer, target = d if isinstance(d, tuple) else (None, 0.0)
            book.add_pose(t, rt.odom.pose, steer, target, rt.speed)
            st['d'], st['pose'], st['speed'] = (steer, target), rt.odom.pose, rt.speed

    def view_frame():
        """当前 BEV 输入图 + 决策(调用方持锁)。还没出过决策时返回 None。"""
        img = getattr(rt, 'last_img', None)
        if img is None:
            return None
        steer, target = st['d']
        v = render(cv2, img, bev_view.cones_in_body(rt.bev, rt.odom.pose, rt.pose_t or 0.0), steer, target,
                   st['speed'], st['stale'], st['free_n'], p)
        head = f"seg {book.seg}{'' if book.active else ' (held)'}  marks {len(book.marks)}  {st['msg']}"
        cv2.putText(v, head, (8, v.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 60, 0), 1, cv2.LINE_AA)
        return v

    def on_feed(m):
        d = np.asarray(m.data)
        ni = int(d[0])
        imu, enc = d[2:2 + 7 * ni].reshape(-1, 7), d[2 + 7 * ni:].reshape(-1, 2)
        with lock:
            pending.extend((r[0], 'imu', tuple(r[1:])) for r in imu.tolist())
            pending.extend((r[0], 'enc', r[1]) for r in enc.tolist())
            st['feed_t'] = now()
            cut = st['feed_t'] - LAG
            pending.sort(key=lambda e: e[0])
            k = 0
            while k < len(pending) and pending[k][0] <= cut:
                handle(*pending[k])
                k += 1
            del pending[:k]
            v = None
            if st['viewers'] and cut >= st['next_view']:
                st['next_view'] = cut + VIEW_DT
                v = view_frame()
        if v is not None:                                   # 编码在锁外
            cam = st['cam']
            if cam is not None:                             # 下面接着:摄像头画面 + 地面分割(2 Hz)
                v = np.vstack([v, cv2.resize(cam, (v.shape[1], v.shape[1] * 3 // 4))])
            jpg = cv2.imencode('.jpg', v, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
            with cond:
                st['jpg'] = jpg
                cond.notify_all()

    node.create_subscription(LaserScan, '/scan', on_scan, 5)
    node.create_subscription(Float64MultiArray, '/shadow_feed', on_feed, 10)
    node.create_subscription(CompressedImage, DEBUG_TOPIC, on_cam, 1)
    if pol.free:
        node.create_subscription(PointCloud2, FREE_TOPIC, on_free, 5)

    class Hd(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, body, ctype='text/plain; charset=utf-8'):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path == '/mark':
                with lock:
                    v = view_frame()
                    steer, target = st['d']
                    k = book.mark(now(), st['pose'], steer, target, st['speed'], st['cam'],
                                  v if v is not None else np.zeros((640, 640, 3), np.uint8), cv2)
                    seg = book.seg
                self.reply((f'已标记 {k}(第 {seg} 段)' if k else '车被抱着,没记').encode())
            elif self.path == '/split':
                with lock:
                    book.split()
                    rt.reset()
                    seg = book.seg
                self.reply(f'开第 {seg} 段'.encode())

        def do_GET(self):
            if self.path.startswith('/map'):
                with lock:
                    png = cv2.imencode('.png', book.render(cv2))[1].tobytes()
                return self.reply(png, 'image/png')
            if self.path != '/stream':
                return self.reply(PAGE, 'text/html; charset=utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=f')
            self.end_headers()
            st['viewers'] += 1
            try:
                while True:
                    with cond:
                        cond.wait()
                        j = st['jpg']
                    self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n\r\n' + j + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                st['viewers'] -= 1

    srv = http.server.ThreadingHTTPServer(('0.0.0.0', PORT), Hd)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    def autosave():
        while True:
            time.sleep(10)
            if now() - st['feed_t'] > 2:
                log.warn('收不到 /shadow_feed(IMU、编码器):先开 ros2 run shadow_feed shadow_feed')
            with lock:
                if book.path:
                    book.save()
                    cv2.imwrite(os.path.join(book.dir(), 'map.png'), book.render(cv2))
    threading.Thread(target=autosave, daemon=True).start()
    log.info(f'影子模式 {a.model}(不发控制),http://<车IP>:{PORT}/,记录到 {book.root}')
    rclpy.spin(node)


if __name__ == '__main__':
    main()
