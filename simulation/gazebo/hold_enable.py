#!/usr/bin/env python3
"""按住不放:以 20Hz 持续发 /policy_enable,松手(Ctrl+C)即停。

policy_node 把 /policy_enable 当心跳,超过 ENABLE_TIMEOUT 没收到就回中位。
所以 `ros2 topic pub --once` 只能让车动一下,要跑就得有人一直发 —— 实车上
由网页摇杆发(推着才发),仿真里用这个脚本代替。

退出时补发一次 false,不靠超时,停得更干脆。

用法:python3 simulation/gazebo/hold_enable.py [秒数]   不给秒数就一直发到 Ctrl+C
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class Holder(Node):
    def __init__(self, limit):
        super().__init__('hold_enable')
        self.pub = self.create_publisher(Bool, '/policy_enable', 1)
        self.limit = limit
        self.n = 0
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'按住中({"%.0fs" % limit if limit else "Ctrl+C 松手"})')

    def tick(self):
        self.pub.publish(Bool(data=True))
        self.n += 1
        if self.limit and self.n * 0.05 >= self.limit:
            raise KeyboardInterrupt


def main():
    limit = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    rclpy.init()
    node = Holder(limit)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Bool(data=False))      # 松手,不等超时
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
