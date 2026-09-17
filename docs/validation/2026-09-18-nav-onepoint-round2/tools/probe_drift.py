# 漂移探针: 静止时连续采样 odom TF 位移与 amcl_pose 更新, 定位虚假运动源
# 判读: odom(x,y) 持续变化 = encoder/IMU/EKF 虚漂; odom 恒定但 amcl_pose 有新帧 = 激光绑架
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

rclpy.init()
n = Node('probe_drift')
tfb = Buffer()
TransformListener(tfb, n, spin_thread=False)
poses = []


def cb_pose(m):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    p = m.pose.pose.position
    poses.append((t, p.x, p.y))


n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', cb_pose, 10)

t0 = time.time()
last_tick = -1
n0 = len(poses)
while time.time() - t0 < 16:
    rclpy.spin_once(n, timeout_sec=0.2)
    tick = int((time.time() - t0) * 2)
    if tick != last_tick:
        last_tick = tick
        try:
            t = tfb.lookup_transform('odom_combined', 'base_footprint', rclpy.time.Time())
            tr = t.transform.translation
            ap = f'({poses[-1][1]:+.3f},{poses[-1][2]:+.3f})' if poses else 'none'
            print(f't={time.time() - t0:5.1f}s odom=({tr.x:+.3f},{tr.y:+.3f}) '
                  f'amcl_pose={ap} 新pose帧={len(poses)}')
        except Exception as e:
            print(f't={time.time() - t0:5.1f}s TF fail: {e}')
