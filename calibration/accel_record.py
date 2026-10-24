"""直道加速 / 减速实测用的轻量记录:只订阅,不发控制。手动(网页)和策略开都录。

    source /opt/ros/humble/setup.bash
    nohup python3 -m calibration.accel_record ~/drives/accel_1023 > /tmp/accel_record.log 2>&1 &   # kill 结束

out.npz:enc_t / enc_v(/encoder 车速,~34 Hz)、cmd_t / cmd_pwm / cmd_servo(/teleop_cmd_vel 发给底盘的 PWM 和舵机刻度)、
imu_t / imu_ax / imu_wz(/IMU_data 纵向加速度、偏航角速度)。时间戳同一时钟;每 5 s 落盘一次,断电最多丢 5 s。
"""
from __future__ import annotations

import os
import signal
import sys
import threading

import numpy as np


def main(argv=None):
    out = (argv or sys.argv[1:])[0]
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    os.makedirs(out, exist_ok=True)
    rclpy.init(args=[])
    node = Node('accel_record')
    now = lambda: node.get_clock().now().nanoseconds * 1e-9
    lock = threading.Lock()
    S = {k: [] for k in ('enc_t', 'enc_v', 'cmd_t', 'cmd_pwm', 'cmd_servo', 'imu_t', 'imu_ax', 'imu_wz')}

    def add(**kw):
        with lock:
            for k, v in kw.items():
                S[k].append(v)

    node.create_subscription(Odometry, '/encoder', lambda m: add(enc_t=now(), enc_v=m.twist.twist.linear.x), 50)
    node.create_subscription(Twist, '/teleop_cmd_vel', lambda m: add(cmd_t=now(), cmd_pwm=m.linear.x, cmd_servo=m.angular.z), 50)
    node.create_subscription(Imu, '/IMU_data', lambda m: add(imu_t=now(), imu_ax=m.linear_acceleration.x,
                                                              imu_wz=m.angular_velocity.z), 50)

    def save():
        with lock:
            d = {k: np.asarray(v, float) for k, v in S.items()}
        tmp = os.path.join(out, 'out.tmp.npz')
        np.savez(tmp, **d)
        os.replace(tmp, os.path.join(out, 'out.npz'))

    node.create_timer(5.0, save)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    while rclpy.ok() and not stop.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
