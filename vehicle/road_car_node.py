#!/usr/bin/env python3
"""road_v2 实车节点:转向 + 目标速度双输出模型,20Hz。

和 steer_policy_node(v1:只出转向、目标速度恒定)只差"大脑":观测、推理、转角与目标速度
都走 common.road_runtime,和仿真节点 road_policy_node、训练环境是同一份代码。
安全外壳照搬 v1 在车上验证过的做法:
  /policy_enable 是心跳(0.3s),松手立刻发中位;雷达/编码器/IMU 任一过期回中位;
  沿当前转角扫掠路径的防撞截停(0.25m)+ 连续截停 3s 报卡死;
  SpeedLoop 编码器闭环出 PWM(爬升 20 PWM/s、硬上限 1620);SIGINT/SIGTERM 时连发中位。
模型给的目标速度每拍都在变(0.45 ~ 速度上限)。SpeedLoop 的 floor 自标定按"当前目标"判塌速,
目标往下调(弯前减速)不会被当成塌速。按变化目标跟随的表现还没在车上测过,先架空再低速落地。

运行(车上,先 source vehicle/ros_env.sh):
  python3 -m vehicle.road_car_node models/road_v2/candidate_167.npz --speed-cap 0.5
输出 /teleop_cmd_vel:linear.x = PWM(电调刻度),angular.z = 舵机刻度。仿真用 road_policy_node。
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

from common.params import CarParams
from common.pwm import steer_to_turn
from common.road_policy import V_LO, RoadPolicy, spec_of
from common.road_runtime import STOP, RoadRuntime
from vehicle.scan_guard import ScanGuard
from vehicle.speed_loop import SpeedLoop, SpeedLoopConfig

CMD_TOPIC = '/teleop_cmd_vel'
ENC_TOPIC = '/encoder'
CONTROL_PERIOD = 0.05       # 20Hz,和训练的 CONTROL_DT 一致
SENSOR_TIMEOUT = 0.3        # 实测 /encoder 34Hz、/scan 12Hz、/IMU_data 55Hz
ENABLE_TIMEOUT = 0.3        # /policy_enable 是心跳不是开关
GUARD_STOP = 0.25           # 同 steer_policy_node:保险杠前净空小于这个就回中位(米)
STUCK_SECONDS = 3.0         # 同 steer_policy_node:连续截停这么久就当卡死
# 起步从中位爬(20 PWM/s)。2026-09-29 试过第一拍跳到 1575(当时以为在死区内,破静 1594),
# 当天下午实测曲线整体下移 ~40:破静 1552、1575 已是 ~0.8 m/s,跳过去等于猛踩一脚,已撤回。
# 死区位置会漂 40 个 PWM,不能写死一个"死区内"的跳点。
LAUNCH_FROM = 1540


class RoadCarController:
    """纯逻辑,不依赖 ROS;节点只管收发。

    tick() 返回 None(没使能,一个字都不发,不跟网页遥控抢)或 (PWM, 舵机刻度)。
    event 给节点打日志:None / 'stale'(心跳或传感器过期)/ 'guard'(截停)/ 'stuck'(截停超过 3s)。
    """

    def __init__(self, policy, speed_cap, params=None, loop=None, guard=None):
        self.p = params or CarParams()
        self.rt = RoadRuntime(policy, speed_cap, SENSOR_TIMEOUT, ENABLE_TIMEOUT, self.p)
        self.loop = loop or SpeedLoop(SpeedLoopConfig(launch_from=LAUNCH_FROM))
        self.guard = guard or ScanGuard()
        self.block_t = None         # 连续截停的起点
        self.event = None
        self.gap = None

    def neutral(self):
        """中位:油门回 1540、方向回正,速度环从中位重新爬(学到的 floor 保留)。"""
        self.loop.reset()
        return self.p.pwm_mid, steer_to_turn(0.0, self.p)

    def on_scan(self, msg, now):
        self.rt.on_scan(msg.ranges, msg.angle_min, msg.angle_increment, now)
        self.guard.update(msg)

    def on_encoder(self, speed, now):
        self.rt.on_odometry(speed, now)

    def on_imu(self, yaw_rate, now):
        self.rt.on_imu(yaw_rate, now)

    def on_enable(self, enabled, now):
        """返回 True = 刚松手,要马上发一次中位(不靠看门狗补,2026-09-24 松手失控的教训)。"""
        was = self.rt.enabled
        release = self.rt.set_enabled(enabled, now)
        if enabled and not was:
            self.loop.reset()       # 重新使能不沿用上一次的油门状态
            self.block_t = None
        return release

    def tick(self, now):
        d = self.rt.decide(now)
        if d is None:
            self.event = None
            return None
        if d is STOP:
            self.event, self.block_t = 'stale', None
            return self.neutral()
        steer, target = d
        servo = steer_to_turn(steer, self.p)
        # 防撞按**当前要打的转角**算扫掠路径。截停时方向照发、油门回中位;策略不出倒车,
        # 所以只报卡死、不自动倒车(雷达在车头,背后看不见)
        gap = self.guard.gap(steer)
        if gap is not None and gap < GUARD_STOP:
            if self.block_t is None:
                self.block_t = now
            self.event = 'stuck' if now - self.block_t > STUCK_SECONDS else 'guard'
            self.gap = gap
            self.loop.reset()
            return self.p.pwm_mid, servo
        self.event, self.block_t = None, None
        return round(self.loop.update(target, self.rt.speed, CONTROL_PERIOD)), servo


def parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('model', help='road_v2 / road_v2.5 / spica_v3_* .npz')
    p.add_argument('--speed-cap', type=float, required=True,
                   help=f'速度上限 m/s,要在模型自己的训练范围内(见 schema);首次上车用 {V_LO}~0.5')
    return p


def create_node_class():
    """ROS 只在真正启动节点时 import;纯逻辑、测试和 --help 不需要 ROS。"""
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool

    class RoadCarNode(Node):
        def __init__(self, model, speed_cap):
            super().__init__('road_car_node')
            self.ctl = RoadCarController(RoadPolicy(model), speed_cap)
            self.pub = self.create_publisher(Twist, CMD_TOPIC, 5)
            self.create_subscription(LaserScan, '/scan', self.on_scan, 1)
            self.create_subscription(Odometry, ENC_TOPIC, self.on_enc, 10)
            self.create_subscription(Imu, '/IMU_data', self.on_imu, 10)
            self.create_subscription(Bool, '/policy_enable', self.on_enable, 1)
            self.last_event = None
            self.last_floor = self.ctl.loop.floor
            self.create_timer(CONTROL_PERIOD, self.tick)
            self.get_logger().info(
                f'road_v2 已载入 {model},速度上限 {speed_cap:.2f} m/s,编码器闭环 -> {CMD_TOPIC},'
                f'等 /policy_enable')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def send(self, cmd):
            tw = Twist()
            tw.linear.x, tw.angular.z = float(cmd[0]), float(cmd[1])
            self.pub.publish(tw)

        def neutral(self):
            self.send(self.ctl.neutral())

        def on_scan(self, m):
            self.ctl.on_scan(m, self.now())

        def on_enc(self, m):
            self.ctl.on_encoder(m.twist.twist.linear.x, self.now())

        def on_imu(self, m):
            self.ctl.on_imu(m.angular_velocity.z, self.now())

        def on_enable(self, m):
            was = self.ctl.rt.enabled
            if self.ctl.on_enable(m.data, self.now()):
                self.neutral()
                self.get_logger().info('松手,已发中位')
            elif m.data and not was:
                self.get_logger().info('使能')

        def tick(self):
            now = self.now()
            cmd = self.ctl.tick(now)
            if cmd is None:
                self.last_event = None
                return
            self.send(cmd)
            ev, log = self.ctl.event, self.get_logger()
            if abs(self.ctl.loop.floor - self.last_floor) >= 1.0:
                # floor 自标定只该在电量变化时慢慢动;频繁变说明被弯道减速之类骗了(2026-09-29)
                log.info(f'速度环 floor {self.last_floor:.0f} -> {self.ctl.loop.floor:.0f},'
                         f'车速 {self.ctl.rt.speed:.2f} m/s')
                self.last_floor = self.ctl.loop.floor
            if ev == 'stale':
                log.warn(f'{self.ctl.rt.stale_sensors(now) or ["心跳"]} 过期,回中位',
                         throttle_duration_sec=1.0)
            elif ev == 'guard':
                log.warn(f'前方 {self.ctl.gap:.2f}m,截停', throttle_duration_sec=1.0)
            elif ev == 'stuck' and self.last_event != 'stuck':
                log.error(f'已连续截停 {STUCK_SECONDS:.0f}s 以上,车卡住了 —— 前方 {self.ctl.gap:.2f}m,'
                          f'策略不会倒车,需要人工干预')
            elif ev is None and self.last_event in ('guard', 'stuck'):
                log.info('脱困,恢复行驶')
            self.last_event = ev

    return RoadCarNode


def main(argv=None):
    args = parser().parse_args(argv)
    if not Path(args.model).is_file():
        raise SystemExit(f'模型文件不存在: {args.model}')
    with np.load(args.model) as z:
        enc = spec_of(str(z['schema']))        # 速度范围跟模型走(#19)
    if not enc.v_lo <= args.speed_cap <= enc.v_hi:
        raise SystemExit(f'--speed-cap 要在训练范围 {enc.v_lo}~{enc.v_hi} m/s 内,给的是 {args.speed_cap}')

    import rclpy
    from rclpy.signals import SignalHandlerOptions

    # 自己接信号:rclpy 默认处理器会先关掉 context,finally 里再发中位时 publisher 已失效,
    # systemctl stop 时车收不到中位(steer_policy_node 2026-09-21 实测)。
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = create_node_class()(args.model, args.speed_cap)

    def stop(*_):
        for _ in range(5):
            node.neutral()
            time.sleep(0.02)
        node.get_logger().info('已回中位')
        os._exit(0)             # sys.exit 会和 spin 抢,C++ 层直接 abort
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
