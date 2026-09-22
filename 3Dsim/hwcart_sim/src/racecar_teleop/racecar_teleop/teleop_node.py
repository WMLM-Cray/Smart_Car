"""rclpy node for racecar_teleop.

Subscribes to ``/joy``, drives the pure ``TeleopStateMachine``, and publishes
``geometry_msgs/Twist`` on ``/teleop/cmd_vel`` at ``publish_rate``.

Safety notes:
  * The input timeout uses ``time.monotonic()`` (wall clock), so a paused
    simulation can never freeze the safety timer.
  * This node does NOT enable ``use_sim_time``; its publish timer therefore
    keeps firing on wall clock during a simulation pause.
  * On shutdown it makes a best-effort zero-command publish.  This is a
    "request to stop", not a brake — the vehicle must have its own command
    timeout for the case where this node crashes or the network drops.

Requires ROS 2 Humble.  NOT yet built/run on this dev machine (see README).
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

from racecar_teleop.mapping import Parameters
from racecar_teleop.state import TeleopStateMachine


def _load_params(node):
    """Build validated Parameters from ROS parameters (fail fast on bad input)."""
    p = Parameters(
        speed_axis=node.declare_parameter('speed_axis', 1).value,
        steering_axis=node.declare_parameter('steering_axis', 3).value,
        speed_axis_inverted=node.declare_parameter('speed_axis_inverted', False).value,
        steering_axis_inverted=node.declare_parameter('steering_axis_inverted', False).value,
        enable_button=node.declare_parameter('enable_button', 5).value,
        low_speed_button=node.declare_parameter('low_speed_button', 4).value,
        deadzone=node.declare_parameter('deadzone', 0.08).value,
        max_speed_forward=node.declare_parameter('max_speed_forward', 0.5).value,
        max_speed_reverse=node.declare_parameter('max_speed_reverse', 0.3).value,
        max_steering_angle=node.declare_parameter('max_steering_angle', 0.35).value,
        wheelbase=node.declare_parameter('wheelbase', 0.36).value,
        low_speed_scale=node.declare_parameter('low_speed_scale', 0.4).value,
        publish_rate=node.declare_parameter('publish_rate', 20.0).value,
        input_timeout=node.declare_parameter('input_timeout', 0.3).value,
    )
    p.validate()  # raises ValueError -> node fails to start with a clear reason
    return p


class TeleopNode(Node):
    def __init__(self):
        super().__init__('racecar_teleop')
        self.params = _load_params(self)
        self.sm = TeleopStateMachine(self.params)

        self.pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)

        period = 1.0 / self.params.publish_rate
        self.create_timer(period, self._on_tick)
        self.get_logger().info(
            'racecar_teleop started: /joy -> /teleop/cmd_vel at '
            '%.1f Hz, input_timeout %.2f s'
            % (self.params.publish_rate, self.params.input_timeout))

    def _on_joy(self, msg):
        self.sm.on_joy(time.monotonic(), msg.axes, msg.buttons)

    def _on_tick(self):
        v, w = self.sm.on_tick(time.monotonic())
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort stop request on shutdown (not a brake).
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
