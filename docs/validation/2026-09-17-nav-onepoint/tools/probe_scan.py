import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
n = Node('probe_scan')
got = {}


def seg(m):
    # 各距离段统计 + 近距点的角度(车体系, 雷达0°=车头前)
    out = []
    for i, v in enumerate(m.ranges):
        if v != v or v == float('inf'):
            continue
        if v <= 0.60:
            ang = math.degrees(m.angle_min + i * m.angle_increment)
            out.append((round(v, 3), round(ang, 1)))
    return out


def mk(topic, key):
    def cb(m):
        if key in got:
            return
        r = m.ranges
        fin = [v for v in r if v == v and v != float('inf')]
        stat = (len(r),
                sum(1 for v in fin if v <= 0.25),
                sum(1 for v in fin if 0.25 < v <= 0.40),
                sum(1 for v in fin if 0.40 < v <= 0.60),
                min(fin) if fin else None)
        got[key] = (stat, seg(m))
    return n.create_subscription(LaserScan, topic, cb, qos_profile_sensor_data)


mk('/scan', 'raw')
mk('/scan_filtered', 'flt')
t0 = time.time()
while len(got) < 2 and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
for k in ('raw', 'flt'):
    if k in got:
        (ln, a, b, c, mn), pts = got[k]
        print(f'{k}: len={ln} <=0.25m:{a}  0.25-0.40m:{b}  0.40-0.60m:{c}  min={mn}')
        for v, ang in pts[:20]:
            side = '前' if abs(ang) < 60 else ('后' if abs(ang) > 120 else '侧')
            print(f'   {v:.3f}m @ {ang:+7.1f}° ({side})')
    else:
        print(k, ': 未收到')
