#!/usr/bin/env python3
"""低速转向策略上车:订 /scan + /encoder + /IMU_data,20Hz 发 /teleop_cmd_vel。

和 policy_node.py 的区别:那个跑的是速度+转向双输出的 fastsim.policy.Policy,
这个跑 low_speed_policy.SteeringPolicy —— **只出转向**,油门是固定 PWM。
两者观测维度不同(52x5 vs 53x3),权重不通用,npz 里有 schema 会直接报错。

速度为什么是固定 PWM 而不是闭环:
  * 训练时速度就是常数(phase A 全程 target_speed=0.6,每局 mean_speed≈0.597),
    老师只规划转向,所以车上照着给一个固定油门就是训练时的样子。
  * 电调在中位附近是自由滑行不是刹车(2026-09-21 实测 PWM 1515~1560 全零、
    松开后车滑行减速),但闭环要压 PWM 降速这件事本身在这台车上没验过,
    先不引入。用户明确偏好开环。

PWM 上限硬限在 1620(PWM_HARD_MAX),这是安全线不是标定值:2026-09-21 架空
试跑时车冲得太快,要求限速。别为了"跑够 0.6m/s"把它往上改 —— 太快先减速再说。
注意这和当天的扫描结果冲突:那次 1640 及以下都读 0(破静死区),1650 才
491mm/s。也就是说 1620 以下可能根本带不动车,或者只能跑到 0.6m/s 以下。
先落地实测 /encoder,跑不动就用占空比降速,不要抬上限。

用法:python3 teleop/steer_policy_node.py runs/policy_s3_steer.npz [--pwm 1600] [--speed 0.6]

和 policy_node.py 一样靠 /policy_enable 心跳放行,松手即停;同一时间只能跑一个,
两个都发 /teleop_cmd_vel 会互相冲掉(2026-09-21 教训:pwm_sweep 被没停的
policy_node 冲成一张全 0 的表,不报错)。
"""
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool

from fastsim.low_speed_policy import SteeringPolicy, observe
from fastsim.params import CarParams
from fastsim.pwm import steer_to_turn
from scan_guard import ScanGuard

SCAN_TIMEOUT = 0.3      # 雷达这么久没数就回中位 —— 观测里 48 维全是它
GUARD_STOP = 0.25       # 保险杠前净空小于这个就回中位(米)
ENABLE_TIMEOUT = 0.3    # /policy_enable 是心跳不是开关
PWM_HARD_MAX = 1620     # --pwm 的硬上限。2026-09-21 架空试跑差点冲出去,限死在这


def main():
    argv = [a for a in sys.argv[1:] if not a.startswith('--')]
    pwm = target = None
    for i, a in enumerate(sys.argv):
        if a == '--pwm':
            pwm = float(sys.argv[i + 1])
        elif a == '--speed':
            target = float(sys.argv[i + 1])
    pwm = 1600.0 if pwm is None else pwm
    target = 0.6 if target is None else target
    if not CarParams().pwm_mid < pwm <= PWM_HARD_MAX:
        sys.exit(f'--pwm 要在 ({CarParams().pwm_mid}, {PWM_HARD_MAX}] 之间,给的是 {pwm}')

    # 自己接信号:rclpy 默认的处理器会先关掉 context,等 finally 里再发中位时
    # publisher 已经失效(RCLError: publisher's context is invalid),等于
    # systemctl stop 停节点时车收不到中位。2026-09-21 实测。watchdog 能兜住,
    # 但不该指望兜底。和 pwm_sweep.py 同一套做法。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SteerPolicyNode(argv[0], pwm, target)

    def stop(*_):
        for _ in range(5):
            node.neutral()
            time.sleep(0.02)
        node.get_logger().info('已回中位')
        os._exit(0)             # sys.exit 会和 spin 抢,C++ 层直接 abort
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    rclpy.spin(node)


class SteerPolicyNode(Node):
    def __init__(self, npz, pwm, target_speed):
        super().__init__('steer_policy_node')
        self.p = CarParams()
        self.policy = SteeringPolicy(npz)
        self.pwm, self.target_speed = float(pwm), float(target_speed)
        self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 1)
        self.create_subscription(Odometry, '/encoder', self.on_enc, 10)
        self.create_subscription(Imu, '/IMU_data', self.on_imu, 10)
        self.create_subscription(Bool, '/policy_enable', self.on_enable, 1)
        self.scan = self.angles = None
        self.scan_t = 0.0
        self.speed = self.yaw_rate = 0.0
        self.last_steer = 0.0           # 网络上一步的原始输出(±1),不是转角
        self.enabled = False
        self.enable_t = 0.0
        self.guard = ScanGuard()
        self.create_timer(0.05, self.tick)          # 20Hz,和训练的 control_dt 一致
        self.get_logger().info(
            f'转向策略已载入,目标 {self.target_speed} m/s,固定 PWM {self.pwm:.0f} '
            f'-> /teleop_cmd_vel,等 /policy_enable')

    def on_scan(self, m):
        n = len(m.ranges)
        if self.angles is None or len(self.angles) != n:
            self.angles = m.angle_min + np.arange(n) * m.angle_increment
        self.scan = np.asarray(m.ranges, float)
        self.scan_t = self.now()
        self.guard.update(m)

    def on_enc(self, m):
        self.speed = m.twist.twist.linear.x

    def on_imu(self, m):
        self.yaw_rate = m.angular_velocity.z

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_enable(self, m):
        if not m.data:
            if self.enabled:
                self.get_logger().info('停车')
            self.enabled = False
            return
        if not self.enabled:
            self.policy.reset()         # 叠帧缓冲清零,别拿上一次的历史开车
            self.last_steer = 0.0
            self.get_logger().info('使能')
        self.enabled = True
        self.enable_t = self.now()

    def send(self, pwm, steer):
        tw = Twist()
        tw.linear.x = float(pwm)
        tw.angular.z = float(steer_to_turn(steer, self.p))
        self.pub.publish(tw)

    def neutral(self):
        self.send(self.p.pwm_mid, 0.0)

    def tick(self):
        if not self.enabled:
            return                      # 没使能一个字都不发,不跟网页遥控抢
        if self.now() - self.enable_t > ENABLE_TIMEOUT:
            self.enabled = False
            self.get_logger().info('心跳断开,停车')
            return self.neutral()
        if self.scan is None or self.now() - self.scan_t > SCAN_TIMEOUT:
            self.get_logger().warn('雷达无数据,回中位', throttle_duration_sec=1.0)
            return self.neutral()

        # 逐位对齐 low_speed_train.episode():target_speed 是那个常数,speed 是
        # 编码器实测值,last 是网络上一步 clip 后的原始输出。搞混了车会乱开。
        obs = observe(self.scan, self.angles, self.speed, self.target_speed,
                      self.yaw_rate, self.last_steer)
        action = self.policy(obs)
        self.last_steer = action
        steer = action * self.p.max_steer

        gap = self.guard.gap()
        if gap is not None and gap < GUARD_STOP:
            self.get_logger().warn(f'前方 {gap:.2f}m,截停', throttle_duration_sec=1.0)
            return self.send(self.p.pwm_mid, steer)
        self.send(self.pwm, steer)


if __name__ == '__main__':
    main()
