#!/usr/bin/env python3
"""编码器标定 + 中位查找。用法(车上,要先 source ROS 和 export ROS_DOMAIN_ID=42):

    python3 encoder_check.py wall         # 对着墙手推,用雷达当尺子(不用量距离)
    python3 encoder_check.py push 2.0     # 手推 2.00m,测 /encoder 的刻度对不对
    python3 encoder_check.py rest         # 车静止 5s,看读数是不是 0
    python3 encoder_check.py neutral      # 找真中位,从 1500 扫到 1560

起因(2026-09-21):
  * pwm_sweep 在 PWM 1600 读到 1753mm/s,而记录里这一档是 470mm/s,差 3.7 倍。
  * 脚本退出后车还在以约 0.4m/s 往前爬 —— 退出时发的是 SPEED_MID=1550,
    watchdog 兜底发的也是 1550。两个都发了却停不下来,说明 1550 不是真中位。
    params.py 里这个值标的是 [代码],从厂家源码抄来,从没实测过。

push 和 wall 都不动电机,纯手推,所以先做它们。它们能把"刻度错了"和"车真的
跑这么快"分开 —— 这两种情况在 pwm_sweep 的输出里长得一模一样。

wall 模式用雷达当尺子:墙不动,车头到墙的距离变化就是真实位移,不用量尺。
雷达是这车上唯一被地图验证过尺度的距离基准,比卷尺还省事。
"""
import math
import statistics
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.params import CarParams
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

FRONT_HALF = math.radians(4)     # 取正前方 ±4° 的中位数,单点噪声不算数

# 舵机中位/满舵幅度从 fastsim/params.py 读,别在这里写死 —— 2026-09-22 之前
# 四个脚本各抄一份 84,改中位时漏改一个就会两套行为(实测真中位是 83 不是 84)。
SPEED_MID = CarParams().pwm_mid
TURN_MID = CarParams().turn_mid


class Enc(Node):
    def __init__(self):
        super().__init__('encoder_check')
        self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
        self.create_subscription(Odometry, '/encoder', self.on_enc, 20)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.lock = threading.Lock()
        self.samples = []
        self.distance = 0.0
        self.last = None
        self.first_msg = None
        self.front = None

    def on_scan(self, m):
        # 角度折回 [-π, π):雷达按 0..2π 发的话,正前方会被切成数组两端各一半
        good = []
        for i, r in enumerate(m.ranges):
            if not (m.range_min < r < m.range_max):
                continue
            ang = (m.angle_min + i * m.angle_increment + math.pi) % (2 * math.pi) - math.pi
            if abs(ang) <= FRONT_HALF:
                good.append(r * math.cos(ang))      # 投影到车头方向
        with self.lock:
            self.front = statistics.median(good) if good else None

    def on_enc(self, m):
        now = time.time()
        with self.lock:
            if self.first_msg is None:
                self.first_msg = m
            v = m.twist.twist.linear.x
            if self.last is not None:
                self.distance += abs(v) * (now - self.last)
            self.last = now
            self.samples.append(v)

    def reset(self):
        with self.lock:
            self.samples, self.distance, self.last = [], 0.0, None

    def take(self):
        with self.lock:
            s, d, self.samples, self.distance, self.last = (
                self.samples, self.distance, [], 0.0, None)
        return s, d

    def send(self, pwm):
        tw = Twist()
        tw.linear.x, tw.angular.z = float(pwm), float(TURN_MID)
        self.pub.publish(tw)


def wait_for_data(node, seconds=10.0):
    end = time.time() + seconds
    while node.first_msg is None and time.time() < end:
        time.sleep(0.1)
    if node.first_msg is None:
        print('等不到 /encoder —— 底盘在跑吗?ROS_DOMAIN_ID 对吗?')
        return False
    return True


def mode_rest(node):
    print('别碰车,采 5 秒…')
    node.reset()
    time.sleep(5)
    s, d = node.take()
    if not s:
        print('没收到数据')
        return
    print(f'{len(s)} 条  均值 {statistics.mean(s):.4f}  '
          f'最大绝对值 {max(abs(v) for v in s):.4f}  积分距离 {d:.3f}')
    print('静止时应该接近 0。明显不为 0 就是零漂,所有速度读数都要先减掉它。')


def mode_push(node, truth):
    m = node.first_msg
    print(f'/encoder 第一帧:frame_id={m.header.frame_id!r} '
          f'child={m.child_frame_id!r} linear.x={m.twist.twist.linear.x}')
    print(f'\n把车摆在起点。回车后开始采,匀速手推 {truth:.2f}m,到终点再按一次回车。')
    input()
    node.reset()
    input('推完按回车…')
    s, d = node.take()
    if not s:
        print('没收到数据')
        return
    print(f'\n采到 {len(s)} 条,积分距离 = {d:.3f}(单位取决于 linear.x 的单位)')
    print(f'实际推了 {truth:.2f} m')
    if d > 0:
        print(f'比值 实际/积分 = {truth / d:.4f}')
        print('  接近 1.00 -> 刻度是对的,linear.x 就是 m/s,1753mm/s 是真的')
        print('  接近 0.27 -> 积分大了约 3.7 倍,正好是 1753/470,刻度错了')


def mode_wall(node):
    """墙当基准:墙不动,车头到墙的距离变化 = 真实位移。不用量尺。"""
    end = time.time() + 10
    while node.front is None and time.time() < end:
        time.sleep(0.1)
    if node.front is None:
        print('看不到 /scan,雷达没起来?')
        return
    print(f'车头对准墙。当前前方 {node.front:.3f} m')
    input('摆好按回车…')
    d0 = node.front
    node.reset()
    print(f'起点 {d0:.3f} m —— 直线往前推,推 1m 左右就够(最少 0.2m)')
    input('推完按回车…')
    s, enc = node.take()
    d1 = node.front
    moved = d0 - d1
    print(f'\n雷达:{d0:.3f} -> {d1:.3f},位移 {moved:.3f} m')
    print(f'编码器积分:{enc:.3f}({len(s)} 条样本)')
    if enc > 0.05 and moved > 0.2:
        k = moved / enc
        print(f'\n刻度系数 = 雷达/编码器 = {k:.4f}')
        print(f'  1.0 附近 -> 编码器是对的,PWM1600 真的跑 {1.753 * k:.2f} m/s')
        print(f'  0.27 附近 -> 编码器读数大了约 3.7 倍,记录里的 470mm/s 才对')
        print(f'  按这个系数,占空 20% 实际是 {0.735 * k:.3f} m/s')
    else:
        print('位移太小,推远一点重来(雷达位移要 >0.2m)')


def mode_neutral(node):
    print('找真中位:1500 -> 1560,每档 1.5s。车会小幅移动,前面留空,随时 Ctrl+C。')
    print(f'{"PWM":>5} {"均值":>9} {"最大":>9}')
    try:
        for pwm in range(1500, 1565, 5):
            end = time.time() + 1.5
            while time.time() < end:
                node.send(pwm)
                time.sleep(0.05)
            s, _ = node.take()
            if s:
                print(f'{pwm:>5} {statistics.mean(s):>9.4f} '
                      f'{max(abs(v) for v in s):>9.4f}', flush=True)
    finally:
        for _ in range(10):
            node.send(SPEED_MID)
            time.sleep(0.05)
        print('已发 1550(注意:这个值可能本身就不是中位,车若还在动就物理断电)')


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'rest'
    truth = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    rclpy.init()
    node = Enc()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        if wait_for_data(node):
            {'rest': lambda: mode_rest(node),
             'push': lambda: mode_push(node, truth),
             'wall': lambda: mode_wall(node),
             'neutral': lambda: mode_neutral(node)}[mode]()
    finally:
        sys.stdout.flush()
        # rclpy.shutdown() 会和还在跑的 spin 线程抢,C++ 层直接 abort
        # ("terminate called without an active exception")。和 pwm_sweep 一样硬退。
        os._exit(0)


if __name__ == '__main__':
    main()
