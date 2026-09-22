# 采一帧 /scan_filtered 存 JSON (绕开 echo 的 YAML/假象坑)
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
n = Node('save_scan')
got = {}
n.create_subscription(LaserScan, '/scan_filtered',
                      lambda m: got.setdefault('s', m), qos_profile_sensor_data)
t0 = time.time()
while 's' not in got and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
m = got['s']
out = {
    'angle_min': m.angle_min,
    'angle_increment': m.angle_increment,
    'ranges': [None if (v != v or v == float('inf')) else v for v in m.ranges],
}
with open('/tmp/scan_now.json', 'w') as f:
    json.dump(out, f)
ok = sum(1 for v in out['ranges'] if v and v > 0.15)
print(f'saved /tmp/scan_now.json beams={len(out["ranges"])} valid={ok} min={min(v for v in out["ranges"] if v and v > 0.15):.3f}')
