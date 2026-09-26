#!/usr/bin/env python3
"""现有单转向模型的 ROS 2 执行节点：订阅雷达、编码器与 IMU，20Hz 输出。

使用 common.low_speed_policy（52 维 × 5 帧），不接受 road_v2（53 维 × 5 帧、双输出）。
--speed-loop 使用编码器 PI 跟踪 --speed；未启用时保留固定 PWM 开环。
systemd 模板使用闭环、目标 0.7 m/s。直线已标定，转弯和低电量仍需实车验收。
PWM 硬上限 1620；达不到目标速度也不能提高上限。

运行：python3 -m vehicle.steer_policy_node models/steering_v1/candidate_060.npz --speed-loop --speed 0.7
--sim 输出 /cmd_vel 的 m/s 与 rad/s，通过 common.control.to_cmd_vel 换算。
实车 /teleop_cmd_vel 使用 PWM 与舵机刻度。策略必须收到放行心跳才运行。
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

from common.control import to_cmd_vel
from common.low_speed_policy import SteeringPolicy, observe
from common.params import CarParams
from common.pwm import action_to_steer, steer_to_turn
from vehicle.scan_guard import ScanGuard
from vehicle.speed_loop import SpeedLoop

# 实车 / 仿真的接口差异只有这两行,和 policy_node.py 同一套。
CAR = ('/teleop_cmd_vel', '/encoder')
SIM = ('/cmd_vel', '/ackermann_steering_controller/odometry')

SCAN_TIMEOUT = 0.3      # 雷达这么久没数就回中位 —— 观测里 48 维全是它
GUARD_STOP = 0.25       # 保险杠前净空小于这个就回中位(米)
ENABLE_TIMEOUT = 0.3    # /policy_enable 是心跳不是开关
STUCK_SECONDS = 3.0     # 连续截停这么久就当卡死,报 error —— 见 tick() 注释
PWM_HARD_MAX = 1620     # --pwm 的硬上限。2026-09-21 架空试跑差点冲出去,限死在这

# 油门每个 tick 最多加这么多。20Hz * 1 = 20 PWM/s,和所有测速脚本里 ramp_to()
# 用的 +2/0.1s 是同一个速率,2026-09-22 架空实测这个速率能起转。
#
# 上下都不能省:
#   * 一步从中位跳到目标可能突然起步;2026-09-23 落地单步到 1600
#     曾让车走出约 0.3m,不能把"电调拒绝跳变"当作保护。
#   * 爬得太慢也起不来 -> 同一天实测 +2/1s(2 PWM/s)从 1550 爬到 1650 全程不转,
#     换成 +2/0.1s 立刻转。所以这个值不是"越保守越安全",调小了车就不走了。
# 减速方向不受这个限制:截停要立刻生效,见 tick()。
PWM_RAMP_UP = 1.0


def main():
    argv = [a for a in sys.argv[1:] if not a.startswith('--')]
    pwm = target = guard = None
    for i, a in enumerate(sys.argv):
        if a == '--pwm':
            pwm = float(sys.argv[i + 1])
        elif a == '--speed':
            target = float(sys.argv[i + 1])
        elif a == '--guard':
            guard = float(sys.argv[i + 1])
    sim = '--sim' in sys.argv
    speed_loop = '--speed-loop' in sys.argv
    if speed_loop and (sim or pwm is not None):
        sys.exit('--speed-loop 是实车闭环油门:不能和 --sim 或 --pwm 一起用')
    if speed_loop and not 0.0 < (target or 0.0) <= 1.0:
        sys.exit(f'--speed-loop 要配 --speed,且在 (0, 1.0] m/s,给的是 {target}')
    pwm = 1600.0 if pwm is None else pwm
    target = 0.6 if target is None else target
    if not sim and not CarParams().pwm_mid < pwm <= PWM_HARD_MAX:
        sys.exit(f'--pwm 要在 ({CarParams().pwm_mid}, {PWM_HARD_MAX}] 之间,给的是 {pwm}')
    # --guard 只在仿真里认。实车的 0.25m 是保险杠不撞上去的最后一道,不给命令行调。
    if guard is not None and not sim:
        sys.exit('--guard 只能配 --sim 用;实车的防撞阈值不从命令行改')
    guard_stop = GUARD_STOP if guard is None else guard

    # 自己接信号:rclpy 默认的处理器会先关掉 context,等 finally 里再发中位时
    # publisher 已经失效(RCLError: publisher's context is invalid),等于
    # systemctl stop 停节点时车收不到中位。2026-09-21 实测。watchdog 能兜住,
    # 但不该指望兜底。和 pwm_sweep.py 同一套做法。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SteerPolicyNode(argv[0], pwm, target, sim, guard_stop, speed_loop)

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
    def __init__(self, npz, pwm, target_speed, sim=False, guard_stop=GUARD_STOP,
                 speed_loop=False):
        super().__init__('steer_policy_node')
        self.p = CarParams()
        self.policy = SteeringPolicy(npz)
        self.sim, self.guard_stop = sim, float(guard_stop)
        self.pwm, self.target_speed = float(pwm), float(target_speed)
        cmd_topic, odom_topic = SIM if sim else CAR
        self.pub = self.create_publisher(Twist, cmd_topic, 5)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 1)
        self.create_subscription(Odometry, odom_topic, self.on_enc, 10)
        self.create_subscription(Imu, '/IMU_data', self.on_imu, 10)
        self.create_subscription(Bool, '/policy_enable', self.on_enable, 1)
        self.scan = self.angles = None
        self.scan_t = 0.0
        self.speed = self.yaw_rate = 0.0
        self.last_steer = 0.0           # 网络上一步的原始输出(±1),不是转角
        self.enabled = False
        self.enable_t = 0.0
        self.stuck = False
        self.guard = ScanGuard()
        self.block_t = None         # 连续截停的起点,None = 没在截停
        self.cur_pwm = float(self.p.pwm_mid)    # 实际发出去的油门,爬升用
        self.loop = SpeedLoop() if speed_loop else None     # None = 固定 PWM 开环
        self.create_timer(0.05, self.tick)          # 20Hz,和训练的 control_dt 一致
        throttle = ('物理量 m/s' if sim else '编码器速度闭环' if self.loop
                    else f'固定 PWM {self.pwm:.0f}')
        self.get_logger().info(
            f'转向策略已载入,目标 {self.target_speed} m/s,{throttle},'
            f'{"仿真" if sim else "实车"}模式 -> {cmd_topic},等 /policy_enable')

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
                # 松手就自己发中位,不靠看门狗补。2026-09-24 实车:闭环输出贴着平台
                # (159x)时松手,这里原来一个字不发,驱动保持 159x,看门狗又只认 >=1600
                # 才算"会动",车以 ~0.6 m/s 一直往前走,摇杆和按钮都管不住。
                self.neutral()
            self.enabled = False
            return
        if not self.enabled:
            self.policy.reset()         # 叠帧缓冲清零,别拿上一次的历史开车
            # 重新使能时也清油门状态,避免沿用上一次的高 PWM。
            self.cur_pwm = float(self.p.pwm_mid)
            if self.loop is not None:
                self.loop.reset()
            self.last_steer = 0.0
            self.block_t, self.stuck = None, False
            self.get_logger().info('使能')
        self.enabled = True
        self.enable_t = self.now()

    def send(self, pwm, steer):
        """pwm 是油门:实车是电调刻度,仿真里当成 m/s(中位 -> 0)。"""
        # 任何一条回中位的路径(心跳断、雷达丢、截停)都要把爬升状态归零,
        # 不然下次恢复时 cur_pwm 还停在高位,发出去就是一个跳变。
        if pwm == self.p.pwm_mid:
            self.cur_pwm = float(self.p.pwm_mid)
            if self.loop is not None:
                self.loop.reset()       # 闭环也从中位重新爬,学到的 floor 保留
        tw = Twist()
        if self.sim:
            v = 0.0 if pwm == self.p.pwm_mid else self.target_speed
            tw.linear.x, tw.angular.z = to_cmd_vel(v, steer, self.p)
        else:
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
        steer = action_to_steer(action, self.p)

        # 防撞按**当前要打的转角**算扫掠路径,不是正前方一条直走廊。
        gap = self.guard.gap(steer)
        if gap is not None and gap < self.guard_stop:
            # 截停是死局:油门归零,策略只出转向不会倒车,场景永远不变,车再也
            # 起不来。这里不自动倒车(雷达在车头,背后看不见,盲倒更危险),只保证
            # 卡死能被看见 —— 2026-09-22 赛道 A 上三次跑圈全是这么结束的,而日志
            # 里只有一行 throttle 过的 warn,看不出车已经死了。
            now = self.now()
            if self.block_t is None:
                self.block_t = now
            elif now - self.block_t > STUCK_SECONDS and not self.stuck:
                self.stuck = True
                self.get_logger().error(
                    f'已连续截停 {now - self.block_t:.1f}s,车卡住了 —— '
                    f'前方 {gap:.2f}m,策略不会倒车,需要人工干预')
            self.get_logger().warn(f'前方 {gap:.2f}m,截停', throttle_duration_sec=1.0)
            return self.send(self.p.pwm_mid, steer)
        if self.stuck:
            self.get_logger().info('脱困,恢复行驶')
        self.block_t, self.stuck = None, False
        # 往上只能爬。脱困之后也走这条路:截停时 cur_pwm 已经被压回中位,
        # 这里重新爬上去,否则那一下就是从中位到目标的跳变,电调不认,车再也起不来。
        if self.loop is not None:
            # 闭环自己管爬升(up_rate 20 PWM/s,和 PWM_RAMP_UP 同一个速率)和上限 1620
            return self.send(round(self.loop.update(self.target_speed, self.speed, 0.05)),
                             steer)
        self.cur_pwm = min(self.pwm, self.cur_pwm + PWM_RAMP_UP)
        self.send(self.cur_pwm, steer)


if __name__ == '__main__':
    main()
