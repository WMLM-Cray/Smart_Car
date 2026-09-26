#!/usr/bin/env python3
"""找舵机真中位:低速跑一段直线,用 IMU 积分偏航角判断往哪偏。逐个舵机值试。

`turn_center.py` 是静态的 —— 保持一个值看前轮正不正。那个靠肉眼,而且站在车头
前面看时左右是反的(2026-09-22 就因为这个出过一次对不上:用户报"偏右",量出来
却是 84 -> 83,方向正好相反)。

这里改成**动态判**:低速直线跑一段,对 `/IMU_data` 的 angular_velocity.z 积分。
车走直线时偏航角不变,积分应该是 0。ROS 右手系,z 正 = 逆时针 = **往左**。
所以:

    积分为正 -> 车在往左偏 -> 舵机值偏大了,往小调
    积分为负 -> 车在往右偏 -> 舵机值偏小了,往大调

报出来的是**每米多少度**,这样不同长度的跑次能直接比。

## 低速长距离

场地短就把速度压低、把时间拉长:同样的偏置,跑得越远积分越明显,而低速下
一段路占的场地更小。默认 --pwm 就在破静点附近,--sec 3 秒。场地允许就加大
--sec,信噪比更好。

用法(车上,落地):
    python3 straight_trim.py --turns 81 82 83 84 85
    python3 straight_trim.py --turns 83 --sec 5 --pwm 1575   # 单个值跑长一点
    python3 straight_trim.py --turns 82 83 84 --repeat 3     # 每个值重复取中位数

**跑之前先停 racecar-policy**,不然它往同一个话题发中位冲掉指令。
"""
import argparse
import math
import os
import signal
import statistics
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))          # scan_guard
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # fastsim

from common.params import CarParams
from vehicle.scan_guard import ScanGuard

PWM_MID = CarParams().pwm_mid
MOVING = 0.05           # m/s,超过这个算在动
ABORT_SPEED = 1.5       # m/s
ABORT_GAP = 0.6         # 保险杠前净空(米)
GYRO_BIAS_SEC = 2.0     # 静止采这么久标定陀螺零漂 —— 不减掉的话积分全是漂移


class Trim(Node):
    def __init__(self):
        super().__init__('straight_trim')
        self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
        self.create_subscription(Odometry, '/encoder', self.on_enc, 20)
        self.create_subscription(Imu, '/IMU_data', self.on_imu, 50)
        self.guard = ScanGuard()
        self.create_subscription(LaserScan, '/scan', lambda m: self.guard.update(m), 1)
        self.lock = threading.Lock()
        self.speed = 0.0
        self.have_enc = False
        self.gyro = 0.0
        self.bias = 0.0
        self.reset()

    def reset(self):
        with self.lock:
            self.yaw = 0.0          # 积分出来的偏航角(rad)
            self.dist = 0.0
            self.t_imu = None
            self.t_enc = None
            self.gyros = []

    def on_imu(self, m):
        now = time.time()
        with self.lock:
            self.gyro = m.angular_velocity.z
            self.gyros.append(self.gyro)
            if self.t_imu is not None:
                self.yaw += (self.gyro - self.bias) * (now - self.t_imu)
            self.t_imu = now

    def on_enc(self, m):
        now = time.time()
        with self.lock:
            self.have_enc = True
            self.speed = m.twist.twist.linear.x
            if self.t_enc is not None:
                self.dist += abs(self.speed) * (now - self.t_enc)
            self.t_enc = now

    def take(self):
        with self.lock:
            return self.yaw, self.dist

    def send(self, pwm, turn):
        tw = Twist()
        tw.linear.x, tw.angular.z = float(pwm), float(turn)
        self.pub.publish(tw)


class Blocked(Exception):
    pass


def hold(node, pwm, turn, seconds, no_guard):
    end = time.time() + seconds
    while time.time() < end:
        if not no_guard:
            gap = node.guard.gap()
            if gap is None or gap < ABORT_GAP:
                raise Blocked('雷达无数据' if gap is None else f'前方只剩 {gap:.2f}m')
        if abs(node.speed) > ABORT_SPEED:
            raise Blocked(f'速度 {node.speed:.2f} m/s 超限')
        node.send(pwm, turn)
        time.sleep(0.05)


def ramp_to(node, target, no_guard, turn):
    """从中位缓慢爬到 target,每 0.1s +2。

    使用已测的 20 PWM/s 爬升。突加油门可能突然起步，不能假定电调会拒绝。
    """
    pwm = PWM_MID
    while pwm < target:
        pwm = min(target, pwm + 2)
        hold(node, pwm, turn, 0.1, no_guard)


def main():
    p = CarParams()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--turns', type=int, nargs='+', required=True, help='要试的舵机值')
    ap.add_argument('--pwm', type=int, default=1575, help='油门,低速跑')
    ap.add_argument('--kick', type=int, default=1610, help='起步用,带起来就降到 --pwm')
    ap.add_argument('--kick-sec', type=float, default=0.5)
    ap.add_argument('--sec', type=float, default=3.0, help='每次直线跑多久')
    ap.add_argument('--repeat', type=int, default=1, help='每个舵机值重复几次取中位数')
    ap.add_argument('--pause', type=float, default=6.0, help='跑完回中位多久(够你搬车)')
    ap.add_argument('--no-guard', action='store_true')
    args = ap.parse_args()
    if args.pwm > 1620:
        ap.error('--pwm 不要超过 1620(和策略节点同一条安全线)')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Trim()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def neutral_exit(*_):
        for _ in range(5):
            node.send(PWM_MID, p.turn_mid)
            time.sleep(0.05)
        print('已回中位', flush=True)
        sys.stdout.flush()
        os._exit(0)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, neutral_exit)

    print(f'等传感器…', flush=True)
    t0 = time.time()
    while time.time() - t0 < 10:
        node.send(PWM_MID, p.turn_mid)
        time.sleep(0.1)
        if node.have_enc and node.t_imu and (args.no_guard or node.guard.gap() is not None):
            break
    if not node.have_enc or node.t_imu is None:
        print('等不到 /encoder 或 /IMU_data —— 底盘在跑吗?ROS_DOMAIN_ID 对吗?')
        neutral_exit()
    if not args.no_guard and node.guard.gap() is None:
        print('等不到 /scan;修雷达,或者加 --no-guard 自己盯着')
        neutral_exit()

    # 陀螺零漂:静止时 z 轴本该是 0,实际有个常量偏置。不减掉的话跑 3 秒积出来
    # 的角度全是漂移,和舵机没关系。
    print(f'车别动,标定陀螺零漂 {GYRO_BIAS_SEC:.0f}s…', flush=True)
    node.reset()
    t0 = time.time()
    while time.time() - t0 < GYRO_BIAS_SEC:
        node.send(PWM_MID, p.turn_mid)
        time.sleep(0.05)
    with node.lock:
        node.bias = statistics.mean(node.gyros) if node.gyros else 0.0
    print(f'  零漂 {node.bias * 57.3:+.3f} °/s\n', flush=True)

    print(f'油门 {args.pwm}(起步 kick {args.kick}),每次 {args.sec}s,'
          f'重复 {args.repeat} 次。z 正 = 往左。', flush=True)
    print(f'{"舵机":>5} {"次":>3} {"偏航°":>8} {"距离m":>7} {"°/m":>8}  判断', flush=True)

    result = {}
    try:
        for turn in args.turns:
            per_m = []
            for k in range(args.repeat):
                ramp_to(node, args.kick, args.no_guard, turn)
                hold(node, args.kick, turn, args.kick_sec, args.no_guard)
                if node.speed < MOVING:
                    print(f'kick {args.kick} 没动起来,提高 --kick', flush=True)
                    neutral_exit()
                node.reset()
                hold(node, args.pwm, turn, args.sec, args.no_guard)
                yaw, dist = node.take()
                deg = yaw * 57.3
                rate = deg / dist if dist > 0.2 else float('nan')
                per_m.append(rate)
                verdict = ('距离太短' if math.isnan(rate)
                           else '往左偏' if rate > 1.0
                           else '往右偏' if rate < -1.0 else '基本直')
                print(f'{turn:>5} {k+1:>3} {deg:>8.1f} {dist:>7.2f} {rate:>8.1f}  {verdict}',
                      flush=True)
                for _ in range(int(args.pause / 0.05)):
                    node.send(PWM_MID, p.turn_mid)
                    time.sleep(0.05)
            good = [r for r in per_m if not math.isnan(r)]
            if good:
                result[turn] = statistics.median(good)
    except Blocked as e:
        print(f'\n中止:{e}', flush=True)

    if result:
        best = min(result, key=lambda t: abs(result[t]))
        print('\n汇总(每米偏多少度,越接近 0 越正):', flush=True)
        for t in sorted(result):
            print(f'  {t:>5} {result[t]:>8.1f} °/m', flush=True)
        print(f'\n>>> 最正的是 {best}({result[best]:+.1f} °/m)', flush=True)
        if abs(result[best]) > 1.0:
            step = '调小' if result[best] > 0 else '调大'
            print(f'    还差得远,继续往{step}的方向试', flush=True)
        print(f'填进 common/params.py 的 turn_mid(现在是 {p.turn_mid})', flush=True)
    neutral_exit()


if __name__ == '__main__':
    main()
