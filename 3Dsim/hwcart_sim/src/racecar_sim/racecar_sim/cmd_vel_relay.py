#!/usr/bin/env python3
# =============================================================================
# cmd_vel 中继节点:/cmd_vel(Twist,对齐实车契约) → ackermann 控制器输入
# 背景:ackermann_steering_controller 2.53.3 基于 steering_controllers_library,
#   use_stamped_vel=false 时订阅 ~/reference_unstamped(Twist),不直接收 /cmd_vel;
#   gzserver 又不解析 --ros-args 无法用 remap 接线(2026-09-14 实测),
#   故用本节点做同类型转发。topic_tools 的 relay 亦可,但本包零额外依赖。
# =============================================================================
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(
            Twist, '/ackermann_steering_controller/reference_unstamped', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.get_logger().info('cmd_vel relay: /cmd_vel -> '
                               '/ackermann_steering_controller/reference_unstamped')

    def on_cmd(self, msg: Twist):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
