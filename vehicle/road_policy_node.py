#!/usr/bin/env python3
"""ROS 2 simulation-only inference node for the dual-output road_v2 model.

Run in the same ROS_DOMAIN_ID as run_sim.sh (default 60; never the car's 42),
with a Gazebo /clock publisher:
  python3 -m vehicle.road_policy_node models/road_v2/candidate_167.npz --speed-cap 0.8
Inputs: /scan, /ackermann_steering_controller/odometry, /IMU_data,
        /policy_enable (Bool heartbeat)
Output: /cmd_vel (Twist; linear.x m/s, angular.z rad/s)
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.road_policy import RoadPolicy, spec_of
from common.road_runtime import RoadRuntime


def _schema(path):
    with np.load(path) as z:
        return str(z['schema'])


CMD_TOPIC = '/cmd_vel'
SCAN_TOPIC = '/scan'
ODOM_TOPIC = '/ackermann_steering_controller/odometry'
IMU_TOPIC = '/IMU_data'
ENABLE_TOPIC = '/policy_enable'
CONTROL_PERIOD = 0.05
CAR_DOMAIN = '42'           # vehicle/ros_env.sh


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('model', help='road_v2 .npz checkpoint')
    p.add_argument('--speed-cap', type=float, default=None,
                   help='maximum commanded speed in m/s (must be within the model\'s own '
                        'training range, see its schema suffix; default: the model\'s v_hi)')
    p.add_argument('--sensor-timeout', type=float, default=0.3,
                   help='maximum age of each scan/odometry/IMU sample in sim seconds')
    p.add_argument('--enable-timeout', type=float, default=0.3,
                   help='maximum age of /policy_enable heartbeat in sim seconds')
    return p


def create_node_class():
    """Import ROS only when starting the node; pure logic and --help need no ROS."""
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool

    class RoadPolicyNode(Node):
        def __init__(self, model, speed_cap, sensor_timeout, enable_timeout):
            super().__init__('road_policy_node', parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', value=True)])
            self.policy = RoadPolicy(model)
            self.runtime = RoadRuntime(self.policy, speed_cap, sensor_timeout, enable_timeout)
            self.pub = self.create_publisher(Twist, CMD_TOPIC, 5)
            self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, 1)
            self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
            self.create_subscription(Imu, IMU_TOPIC, self.on_imu, 10)
            self.create_subscription(Bool, ENABLE_TOPIC, self.on_enable, 1)
            self._last_stale_log = None
            self.create_timer(CONTROL_PERIOD, self.tick)
            self.get_logger().info(
                f'road_v2 loaded; simulation clock required; {CMD_TOPIC} output, '
                f'speed cap {speed_cap:.2f} m/s; waiting for /policy_enable heartbeat')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def publish(self, command):
            msg = Twist()
            msg.linear.x, msg.angular.z = command
            self.pub.publish(msg)

        def neutral(self):
            self.publish((0.0, 0.0))

        def on_scan(self, msg):
            self.runtime.on_scan(msg.ranges, msg.angle_min, msg.angle_increment, self.now())

        def on_odom(self, msg):
            self.runtime.on_odometry(msg.twist.twist.linear.x, self.now())

        def on_imu(self, msg):
            self.runtime.on_imu(msg.angular_velocity.z, self.now())

        def on_enable(self, msg):
            was_enabled = self.runtime.enabled
            if self.runtime.set_enabled(msg.data, self.now()):
                self.neutral()
                self.get_logger().info('policy disabled; zero command sent')
            elif msg.data and not was_enabled:
                self.get_logger().info('policy enabled')

        def tick(self):
            now = self.now()
            if now <= 0.0:
                if self.runtime.enabled:
                    self.neutral()
                return
            command = self.runtime.step(now)
            if command is not None:
                self.publish(command)
                stale = command == (0.0, 0.0)
                if stale and self._last_stale_log != 'stale':
                    self.get_logger().warn('sensor or enable heartbeat stale; zero command sent')
                    self._last_stale_log = 'stale'
                elif not stale:
                    self._last_stale_log = None

    return RoadPolicyNode


def main(argv=None):
    args = parser().parse_args(argv)
    domain = os.environ.get('ROS_DOMAIN_ID')
    if domain is None or domain == CAR_DOMAIN:
        raise SystemExit(f'simulation only: set ROS_DOMAIN_ID to the sim domain (run_sim.sh default 60), '
                         f'not unset and not the car domain {CAR_DOMAIN}')
    if args.sensor_timeout <= 0.0 or args.enable_timeout <= 0.0:
        raise SystemExit('timeouts must be positive')
    if not Path(args.model).is_file():
        raise SystemExit(f'model file not found: {args.model}')

    enc = spec_of(_schema(args.model))      # 速度区间是模型自描述的,不再写死
    if args.speed_cap is None:
        args.speed_cap = enc.v_hi
    if not enc.v_lo <= args.speed_cap <= enc.v_hi:
        raise SystemExit(f'--speed-cap {args.speed_cap} must be within this model\'s training '
                         f'range {enc.v_lo}..{enc.v_hi} m/s (from its schema)')

    import rclpy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = create_node_class()(args.model, args.speed_cap,
                                   args.sensor_timeout, args.enable_timeout)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            for _ in range(5):
                node.neutral()
                time.sleep(0.02)
            node.get_logger().info('shutdown: zero commands sent')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
