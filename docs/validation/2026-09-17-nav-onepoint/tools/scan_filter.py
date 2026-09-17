#!/usr/bin/env python3
# scan_filter.py — /scan -> /scan_filtered (2026-09-17, team 层, 不碰厂家)
# 目的: 滤除车载天线贴身回波(车正后方 ~0.15m 等距双点, 随车固定)
# 行为: ranges 中 <=MIN_R 的有限值改 inf; nan/inf 原样; 消息其余字段不动
# QoS: SensorDataQoS (best_effort, 与 /scan 一致)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

MIN_R = 0.25


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb,
                                            qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered',
                                         qos_profile_sensor_data)
        self.n_in = 0
        self.n_out = 0

    def cb(self, msg):
        self.n_in += 1
        r = list(msg.ranges)
        n = 0
        for i, v in enumerate(r):
            if v != v or v == float('inf'):
                continue
            if v <= MIN_R:
                r[i] = float('inf')
                n += 1
        self.n_out += n
        msg.ranges = r
        self.pub.publish(msg)
        if self.n_in % 600 == 0:
            self.get_logger().info(
                f'frames={self.n_in} filtered_pts_total={self.n_out} '
                f'(MIN_R={MIN_R})')


def main():
    rclpy.init()
    node = ScanFilter()
    # rclpy 默认 SIGINT handler 先 shutdown context -> finally 报错, 自接管(同 cmd_vel_safety)
    try:
        from rclpy.signals import uninstall_signal_handlers
        uninstall_signal_handlers()
    except Exception as e:
        node.get_logger().warn(f"卸载 rclpy 信号处理失败: {e}")
    import signal
    stop = {'f': False}

    def _sig(_s, _f):
        stop['f'] = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    while not stop['f']:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
