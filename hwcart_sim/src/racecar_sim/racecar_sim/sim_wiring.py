#!/usr/bin/env python3
# =============================================================================
# 仿真接线箱:补齐 Gazebo Classic 仿真与实车契约之间的两根"线"
#   1) /cmd_vel(Twist,实车契约话题) → ackermann 控制器 ~/reference_unstamped
#      背景:gzserver 不解析 --ros-args,无法用 remap 接线(2026-09-14 实测);
#      ackermann_steering_controller 2.53.3 订阅 ~/reference_unstamped(Twist)
#   2) 控制器 ~/tf_odometry(TFMessage) → /tf
#      背景:steering_controllers_library 的 enable_odom_tf 只往专属话题发
#      TFMessage,不广播 /tf;odom→base_footprint 是导航/验收的必需 TF
# =============================================================================
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class SimWiring(Node):

    def __init__(self):
        super().__init__('sim_wiring')

        # 1) cmd_vel → 控制器速度指令
        self.vel_pub = self.create_publisher(
            Twist, '/ackermann_steering_controller/reference_unstamped', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

        # 2) 控制器 TFMessage → /tf
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.create_subscription(
            TFMessage, '/ackermann_steering_controller/tf_odometry',
            self.on_tf_odometry, 10)

        self.get_logger().info(
            'sim_wiring: /cmd_vel -> reference_unstamped, '
            'tf_odometry -> /tf')

    def on_cmd_vel(self, msg: Twist):
        self.vel_pub.publish(msg)

    def on_tf_odometry(self, msg: TFMessage):
        self.tf_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimWiring()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
