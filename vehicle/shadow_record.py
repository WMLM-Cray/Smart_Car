"""影子模式(车上那一半):只录 BEV 模型要吃的原始话题,不跑模型、不画图、不发控制。录制交给
`ros2 bag record`(C++,不反序列化,几乎不占 CPU);这里只开个网页记"标记问题" / "新段"的时刻。
模型怎么判断、全局地图、问题现场都在电脑上用 evaluation/shadow_replay.py 回放出来(和车上跑同一套
BevRuntime,输入一样,决策就一样)。

    source /opt/ros/humble/setup.bash
    python3 -m vehicle.shadow_record              # 默认录到 ~/drives/shadow_<时间>,Ctrl-C 结束后自动转换
    python3 -m vehicle.shadow_record --convert DIR    # 断电没转换的,手动转

bev_v2 模型回放要 FREE 层:同时开 vehicle/ground_seg_node.py(它发的 /ground_free、/ground_debug 也录下)。
网页 http://<车IP>:8090/:两个按钮 + 录了多久、bag 多大。抱车分段在回放时按录下的 IMU 判断。

目录里:bag/(ros2 bag 原始数据)、events.csv(时刻, mark|split);转换后多出 chunk_0001.npz
(scan_t, scans, enc_t, enc_v, imu[t wx wy wz ax ay az], free_t, free_stamp, free_n, free_xy)、
cam/NNNNN.jpg + cam.csv(序号, 拍摄时刻)。时刻都是 bag 的接收时刻(系统时钟秒),和 road_car_node 一样。
"""
from __future__ import annotations

import argparse
import http.server
import json
import os
import signal
import subprocess
import threading
import time

import numpy as np

PORT = 8090
TOPICS = ('/scan', '/encoder', '/IMU_data', '/ground_free', '/ground_debug')
PAGE = """<html><head><meta name='viewport' content='width=device-width'><meta charset='utf-8'></head>
<body style='margin:0;background:#222;color:#eee;font-family:sans-serif;padding:12px'>
<button onclick="post('/mark')" style='font-size:26px;padding:16px 36px'>标记问题</button>
<button onclick="post('/split')" style='font-size:26px;padding:16px 36px'>新段</button>
<p id='s' style='font-size:20px'></p><pre id='st' style='font-size:18px'></pre>
<script>
function post(u){fetch(u,{method:'POST'}).then(r=>r.text()).then(t=>{document.getElementById('s').innerText=t})}
setInterval(()=>fetch('/status').then(r=>r.text()).then(t=>{document.getElementById('st').innerText=t}),1000)
</script></body></html>""".encode()


def convert(out):
    """bag -> chunk_0001.npz + cam/。在车上录完后跑(要 rclpy / rosbag2_py)。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=os.path.join(out, 'bag'), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    S = {k: [] for k in ('scan_t', 'scans', 'enc_t', 'enc_v', 'imu', 'free_t', 'free_stamp', 'free_n', 'free_xy')}
    meta, n_cam = (0.0, 0.0), 0
    os.makedirs(os.path.join(out, 'cam'), exist_ok=True)
    with open(os.path.join(out, 'cam.csv'), 'w') as cam_csv:
        while r.has_next():
            topic, data, t_ns = r.read_next()
            m, t = deserialize_message(data, types[topic]), t_ns * 1e-9
            if topic == '/scan':
                S['scan_t'].append(t)
                S['scans'].append(np.asarray(m.ranges, np.float32))
                meta = (m.angle_min, m.angle_increment)
            elif topic == '/encoder':
                S['enc_t'].append(t)
                S['enc_v'].append(m.twist.twist.linear.x)
            elif topic == '/IMU_data':
                w, c = m.angular_velocity, m.linear_acceleration
                S['imu'].append((t, w.x, w.y, w.z, c.x, c.y, c.z))
            elif topic == '/ground_free':
                xy = np.frombuffer(bytes(m.data), np.float32).reshape(-1, 2)
                S['free_t'].append(t)
                S['free_stamp'].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
                S['free_n'].append(len(xy))
                S['free_xy'].append(xy)
            elif topic == '/ground_debug':
                n_cam += 1
                with open(os.path.join(out, 'cam', f'{n_cam:05d}.jpg'), 'wb') as f:
                    f.write(bytes(m.data))
                cam_csv.write(f'{n_cam},{m.header.stamp.sec + m.header.stamp.nanosec * 1e-9:.4f}\n')
    np.savez(os.path.join(out, 'chunk_0001.npz'), angle_min=meta[0], angle_increment=meta[1],
             scan_t=np.array(S['scan_t']), scans=np.array(S['scans'], np.float32).reshape(len(S['scan_t']), -1),
             enc_t=np.array(S['enc_t']), enc_v=np.array(S['enc_v']),
             imu=np.array(S['imu'], np.float64).reshape(-1, 7),
             free_t=np.array(S['free_t']), free_stamp=np.array(S['free_stamp']),
             free_n=np.array(S['free_n'], np.int32),
             free_xy=np.concatenate(S['free_xy']) if S['free_xy'] else np.zeros((0, 2), np.float32))
    print(f'转换完:雷达 {len(S["scan_t"])} 帧,地面 {len(S["free_t"])} 帧,画面 {n_cam} 张 -> {out}', flush=True)


def du(path):
    return sum(os.path.getsize(os.path.join(d, f)) for d, _, fs in os.walk(path) for f in fs)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', nargs='?', default=None)
    ap.add_argument('--convert', metavar='DIR', help='只把 DIR 里录好的 bag 转成 npz')
    a = ap.parse_args(argv)
    if a.convert:
        return convert(a.convert)
    out = a.out or os.path.expanduser(time.strftime('~/drives/shadow_%m%d_%H%M%S'))
    os.makedirs(out, exist_ok=True)
    ev_csv = open(os.path.join(out, 'events.csv'), 'a', buffering=1)
    t0, n_ev = time.time(), [0]

    class Hd(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, body, ctype='text/plain; charset=utf-8'):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path in ('/mark', '/split'):
                ev_csv.write(f'{time.time():.4f},{self.path[1:]}\n')     # 系统时钟,和 bag 的接收时刻同一个钟
                n_ev[0] += 1
                self.reply(f'{"已标记" if self.path == "/mark" else "已分段"} {time.strftime("%H:%M:%S")}'.encode())

        def do_GET(self):
            if self.path.startswith('/status'):
                return self.reply(json.dumps(dict(录了=f'{time.time() - t0:.0f} s', bag=f'{du(out) / 1e6:.0f} MB',
                                                  标记分段=n_ev[0], 录制=('在录' if bag.poll() is None else '已停!')),
                                             ensure_ascii=False).encode())
            return self.reply(PAGE, 'text/html; charset=utf-8')

    srv = http.server.ThreadingHTTPServer(('0.0.0.0', PORT), Hd)     # 先占端口:占不到就别开 bag
    srv.daemon_threads = True
    bag = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', os.path.join(out, 'bag'), *TOPICS])
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    print(f'影子录制(不跑模型、不发控制)到 {out},http://<车IP>:{PORT}/', flush=True)
    stop.wait()
    bag.send_signal(signal.SIGINT)
    bag.wait(timeout=20)
    srv.shutdown()
    os.nice(19)                                         # 转换是录完之后的事,让着别的进程
    convert(out)


if __name__ == '__main__':
    main()
