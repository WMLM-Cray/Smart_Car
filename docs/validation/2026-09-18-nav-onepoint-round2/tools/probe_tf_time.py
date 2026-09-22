# 探针: dump TF 各关键边最新时间戳 vs 节点当前钟, 验证"缓存未来戳"污染
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

rclpy.init()
n = Node('probe_tf_time')
b = Buffer()
TransformListener(b, n, spin_thread=False)

t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.2)

print(f'epoch now      : {time.time():.3f}  ({time.strftime("%F %T")})')
print(f'node clock now : {n.get_clock().now().nanoseconds / 1e9:.3f}')
for src, dst in [('odom_combined', 'base_footprint'), ('map', 'odom_combined'),
                 ('base_footprint', 'laser_link')]:
    try:
        t = b.lookup_transform(src, dst, rclpy.time.Time())
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        age = time.time() - stamp
        print(f'{src}->{dst:16s} latest stamp: {stamp:.3f}  (age {age:+.1f}s)')
    except Exception as e:
        print(f'{src}->{dst:16s} FAIL: {type(e).__name__}: {e}')
