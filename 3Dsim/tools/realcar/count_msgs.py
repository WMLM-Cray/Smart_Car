#!/usr/bin/env python3
# 话题消息计数器+首末时间戳 (阶段2 四.7 证据采集, 2026-09-17)
# 用法: python3 count_msgs.py <窗口秒> <话题1> [话题2 ...]
# 目的: 推车前后各跑一遍, 证明 /scan 与里程计在持续发布、AMCL 是否被移动触发更新
# QoS: 统一 best_effort+volatile 订阅(对 reliable 发布者兼容, 只计数不取 latch)
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

TYPES = {
    '/scan': LaserScan,
    '/odometry/filtered': Odometry,
    '/odom': Odometry,
    '/odom_combined': Odometry,   # 实车 EKF 的里程计话题名(2026-09-17 实测 topic list)
    '/amcl_pose': PoseWithCovarianceStamped,
}


def main():
    if len(sys.argv) < 3:
        print("用法: count_msgs.py <窗口秒> <话题...>")
        sys.exit(1)
    dur = float(sys.argv[1])
    topics = sys.argv[2:]
    rclpy.init()
    node = Node('count_msgs')
    stats = {t: {'n': 0, 'first': None, 'last': None} for t in topics}
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)

    def mk(t):
        def cb(m):
            s = stats[t]
            s['n'] += 1
            st = getattr(getattr(m, 'header', None), 'stamp', None)
            ts = f"{st.sec}.{st.nanosec:09d}" if st is not None else 'NA'
            if s['first'] is None:
                s['first'] = ts
            s['last'] = ts
        return cb

    for t in topics:
        if t not in TYPES:
            print(f"未支持话题类型: {t} (支持: {' '.join(sorted(TYPES))})")
            sys.exit(2)
        node.create_subscription(TYPES[t], t, mk(t), qos)

    end = time.time() + dur
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.3)

    print(f"计数窗口 {dur:.0f}s, 车机墙钟 {time.strftime('%H:%M:%S')}")
    for t in topics:
        s = stats[t]
        print(f"{t}: n={s['n']} first_stamp={s['first']} last_stamp={s['last']}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
