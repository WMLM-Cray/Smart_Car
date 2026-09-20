#!/usr/bin/env python3
"""策略上车:订 /scan + /encoder + /IMU_data,20Hz 发 /teleop_cmd_vel。

只做 ROS 管线,推理在 fastsim/policy.py(纯 numpy,车上不用装 torch)。
观测口径与训练时逐位一致,由 fastsim/tests/test_policy.py 保证 —— 那个测试不过就别上车。

发 /teleop_cmd_vel 而不是直接发 /car_cmd_vel,是为了白蹭已经在跑的两层保护:
watchdog.py(停发就回中位)和网页遥控的急停。代价是网页遥控同时开着会抢话题,
所以本节点默认不发车,要显式使能:

    ros2 run 之后另开一个终端
    ros2 topic pub --once /policy_enable std_msgs/Bool "{data: true}"

用法:python3 teleop/policy_node.py runs/policy_s3.npz [--sim]

--sim 接队友的 racecar_sim(3Dsim/hwcart_sim)。那边 /cmd_vel 收的是物理量 Twist,
不能再过 pwm.py —— 不然 speed_to_pwm 算出来的 1800 会被当成 1800 m/s;而且
angular.z 是偏航角速度不是前轮转角,换算走 fastsim/interop.py 的 to_cmd_vel。
话题名的差异见 SIM/CAR 两张表。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool

import numpy as np

from fastsim.interop import to_cmd_vel
from fastsim.params import CarParams
from fastsim.policy import Policy
from fastsim.pwm import speed_to_pwm, steer_to_turn
from scan_guard import ScanGuard

BUDGET = 120.0          # 训练时的时长预算,观测里的"剩余时间"按它归一化
SCAN_TIMEOUT = 0.3      # 雷达这么久没数就回中位 —— 观测里 48 维全是它,断了就是瞎开
GUARD_STOP = 0.25       # 保险杠前净空小于这个就不许再往前(米),和网页遥控同一个值
ENABLE_TIMEOUT = 0.3    # /policy_enable 是心跳不是开关:这么久没收到就回中位
PWM_CAP = 1610          # 实车 PWM 硬上限。v_fwd_full 还没实测,这一条保证最坏情况也只是慢速。
                        # 1650 架空实测太快(2026-09-20),收到 1610。注意电调在 1600
                        # 有个台阶(1599->0,1600->470mm/s),所以 1600~1610 这一段速度
                        # 基本一样,再往下就进死区完全不动 —— 这已经是能动的最慢挡。

# 实车 / 仿真的接口差异,只有这两行:(下发话题, 里程计话题)
# 仿真指队友的 racecar_sim(3Dsim/hwcart_sim);它的 /encoder 和 /odom_combined
# 还没接出来,车速只能从 ackermann 控制器自己的里程计取。
CAR = ('/teleop_cmd_vel', '/encoder')
SIM = ('/cmd_vel', '/ackermann_steering_controller/odometry')


def to_cmd(v, steer, sim, p, pwm_cap=PWM_CAP):
    """(m/s, 前轮转角 rad) -> Twist 的 (linear.x, angular.z)。

    实车那两个字段是电调 PWM 和舵机刻度。仿真那边 angular.z 是**偏航角速度**
    而不是前轮转角 —— 直接把转角发过去会被当成 rad/s,车原地乱转。

    实车这一路再夹一次 pwm_cap。注意这只是安全网,不是"等效于慢速训练":
    策略命令 1.5m/s 却只跑到 0.4m/s,观测里那一维车速会一直偏低,是训练时
    没见过的输入。所以限速跑得好不代表策略对,跑不好也不代表策略错。
    """
    if sim:
        return to_cmd_vel(v, steer, p)
    return float(min(speed_to_pwm(v, p), pwm_cap)), float(steer_to_turn(steer, p))


class PolicyNode(Node):
    def __init__(self, npz, sim=False):
        # 两边都用墙钟。仿真里本来该跟 /clock 走(实时系数不是 1 时墙钟会偏,而
        # "剩余时间"是观测里的一维),但 gazebo_ros 的 /clock 只发 10Hz 且改不动
        # (publish_rate 无论从命令行还是运行时设都不生效),用仿真时间的定时器
        # 就快不过 10Hz —— 而策略是按 20Hz 训的。两害相权,控制频率对齐更要紧。
        # 前提是实时系数≈1:跑之前拿 gz stats -p 确认第一列是 1.00。
        super().__init__('policy_node', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        p = CarParams()
        self.policy = Policy(npz, max_steer=p.max_steer)
        self.p = p
        self.sim = sim
        cmd_topic, odom_topic = SIM if sim else CAR
        self.pub = self.create_publisher(Twist, cmd_topic, 5)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 1)
        self.create_subscription(Odometry, odom_topic, self.on_enc, 10)
        self.create_subscription(Imu, '/IMU_data', self.on_imu, 10)
        self.create_subscription(Bool, '/policy_enable', self.on_enable, 1)
        self.scan = self.angles = None
        self.scan_t = 0.0
        self.speed = self.yaw_rate = 0.0
        self.enabled = False
        self.start_t = self.enable_t = 0.0
        self.guard = ScanGuard()
        self.create_timer(0.05, self.tick)      # 20Hz,和训练时的 control_dt 一致
        self.get_logger().info(
            f'策略已载入({self.policy.obs_dim} 维 x {self.policy.frames} 帧,'
            f'上限 {self.policy.v_cmd_fwd} m/s),{"仿真" if sim else "实车"}模式 -> '
            f'{cmd_topic},等 /policy_enable')

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

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_enable(self, m):
        """/policy_enable 是心跳,不是开关 —— 要持续发 true 才算"手一直推着"。

        实车上只发一次就跑是危险的:发命令那个进程崩了、网页关了、Wi-Fi 断了,
        车都会继续按最后一条指令开下去。改成心跳后松手即停,和网页遥控的
        DEADMAN 是同一套逻辑。仿真里用 hold_enable.py 持续发。
        """
        if not m.data:
            if self.enabled:
                self.get_logger().info('停车')
            self.enabled = False
            return
        if not self.enabled:
            self.policy.reset()             # 叠帧缓冲清零,别拿上一次的历史开车
            self.start_t = self.now()
            self.get_logger().info('使能')
        self.enabled = True
        self.enable_t = self.now()

    def neutral(self):
        tw = Twist()
        tw.linear.x, tw.angular.z = to_cmd(0.0, 0.0, self.sim, self.p)
        self.pub.publish(tw)

    def tick(self):
        if not self.enabled:
            return                          # 没使能就一个字都不发,不跟网页遥控抢
        if self.now() - self.enable_t > ENABLE_TIMEOUT:
            self.enabled = False            # 心跳断了 = 松手,补一次中位就闭嘴
            self.get_logger().info('心跳断开,停车')
            return self.neutral()
        if self.scan is None or self.now() - self.scan_t > SCAN_TIMEOUT:
            self.get_logger().warn('雷达无数据,回中位', throttle_duration_sec=1.0)
            return self.neutral()

        left = max(0.0, 1.0 - (self.now() - self.start_t) / BUDGET)
        v, steer = self.policy.act(self.scan, self.angles, self.speed, self.yaw_rate, left)

        gap = self.guard.gap()
        if v > 0 and gap is not None and gap < GUARD_STOP:
            self.get_logger().warn(f'前方 {gap:.2f}m,截停', throttle_duration_sec=1.0)
            v = 0.0

        tw = Twist()
        tw.linear.x, tw.angular.z = to_cmd(v, steer, self.sim, self.p)
        self.pub.publish(tw)


def main():
    rclpy.init()
    args = [a for a in sys.argv[1:] if not a.startswith('-')]
    node = PolicyNode(args[0], sim='--sim' in sys.argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.neutral()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
