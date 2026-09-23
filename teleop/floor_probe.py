#!/usr/bin/env python3
"""测两个数:维持点(pwm_floor)和破静点(pwm_breakaway)。落地跑,编码器读速度。

speed_loop.py 的闭环全靠这两个数:
  * **维持点** —— 车已经在动时,能继续走的最低 PWM。闭环的输出下限,
    低于它车会停,而停了要更高的破静 PWM 才起得来,于是一冲一停。
  * **破静点** —— 从静止能起步的最低 PWM。闭环起步时直接给这个值。

## 为什么不用 pwm_sweep 的降序扫描

降序连续扫要一口气跑完所有档位:从 1610(实测约 2m/s)降到 1550,十几档乘每档
一秒多,累计十几米 —— 室内没这么长的直线。

这里改成**一次一个候选值**:先用 kick PWM 把车带起来,立刻掉到候选值,看它
维持还是刹停。每次约 2 秒、1~2 米,档与档之间回中位,你把车搬回起点再继续。

## 两个模式

    floor      默认。从高往低试,第一个"掉下去就停"的档位 = 维持点
    breakaway  从静止逐档往上,第一个能动起来的档位 = 破静点

用法(车上,落地,前方 3 米以上直线):
    python3 floor_probe.py floor      --from 1600 --to 1550 --step 5
    python3 floor_probe.py breakaway  --from 1550 --to 1620 --step 5

**扫之前先停 racecar-policy**,不然它往同一个话题发中位,把结果冲成全 0
(2026-09-21 踩过)。两个数都随电量变,电量变了要重测。
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
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))          # scan_guard
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # fastsim

from fastsim.params import CarParams
from scan_guard import ScanGuard

# 舵机中位/满舵幅度从 fastsim/params.py 读,别在这里写死 —— 2026-09-22 之前
# 四个脚本各抄一份 84,改中位时漏改一个就会两套行为(实测真中位是 83 不是 84)。
PWM_MID = CarParams().pwm_mid
TURN_MID = CarParams().turn_mid
MOVING = 0.05           # m/s,超过这个算"在动"
ABORT_SPEED = 2.5       # m/s,超过就中止
ABORT_GAP = 0.6         # 保险杠前净空小于这个就中止(米)


class Probe(Node):
    def __init__(self):
        super().__init__('floor_probe')
        self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
        self.create_subscription(Odometry, '/encoder', self.on_enc, 20)
        self.guard = ScanGuard()
        self.create_subscription(LaserScan, '/scan', lambda m: self.guard.update(m), 1)
        self.lock = threading.Lock()
        self.samples = []
        self.distance = 0.0
        self.last_t = None
        self.speed = 0.0

    def on_enc(self, m):
        now = time.time()
        with self.lock:
            v = m.twist.twist.linear.x
            if self.last_t is not None:
                self.distance += abs(v) * (now - self.last_t)
            self.last_t = now
            self.speed = v
            self.samples.append(v)

    def take(self):
        with self.lock:
            s, self.samples = self.samples, []
        return s

    def send(self, pwm):
        tw = Twist()
        tw.linear.x, tw.angular.z = float(pwm), float(TURN_MID)
        self.pub.publish(tw)


class Blocked(Exception):
    pass


def hold(node, pwm, seconds, no_guard):
    end = time.time() + seconds
    while time.time() < end:
        if not no_guard:
            gap = node.guard.gap()
            if gap is None or gap < ABORT_GAP:
                raise Blocked('雷达无数据' if gap is None else f'前方只剩 {gap:.2f}m')
        if abs(node.speed) > ABORT_SPEED:
            raise Blocked(f'速度 {node.speed:.2f} m/s 超限')
        node.send(pwm)
        time.sleep(0.05)


def ramp_to(node, target, no_guard):
    """从中位缓慢爬到 target,每 0.1s +2。

    电调不吃从中位直接跳到高油门这种突变,按突加油门保护处理,根本不启动
    (2026-09-22 实测:升序扫 1600 起转,降序扫直接给 1620 一点反应都没有)。
    所有"先把车带起来"的地方都要走这里,不能一步到位。
    """
    pwm = PWM_MID
    while pwm < target:
        pwm = min(target, pwm + 2)
        hold(node, pwm, 0.1, no_guard)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=['floor', 'breakaway'])
    ap.add_argument('--from', dest='start', type=int, default=None)
    ap.add_argument('--to', dest='stop', type=int, default=None)
    ap.add_argument('--step', type=int, default=5, help='正数,方向由模式决定')
    ap.add_argument('--kick', type=int, default=1610, help='floor 模式起步用的 PWM')
    ap.add_argument('--kick-sec', type=float, default=0.6)
    ap.add_argument('--hold', type=float, default=1.5, help='每个候选值保持多久')
    ap.add_argument('--pause', type=float, default=5.0, help='档间回中位多久(够你搬车)')
    ap.add_argument('--max-dist', type=float, default=10.0)
    ap.add_argument('--no-guard', action='store_true', help='场地窄时关雷达防撞,自己盯着')
    args = ap.parse_args()

    down = args.mode == 'floor'
    start = args.start if args.start is not None else (1600 if down else 1550)
    stop = args.stop if args.stop is not None else (1550 if down else 1620)
    step = -abs(args.step) if down else abs(args.step)
    if args.step <= 0:
        ap.error('--step 给正数,方向由模式决定')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Probe()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def neutral_exit(*_):
        for _ in range(5):
            node.send(PWM_MID)
            time.sleep(0.05)
        print('已回中位', flush=True)
        sys.stdout.flush()
        os._exit(0)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, neutral_exit)

    print('中位 2s(等电调解锁 + 等第一帧雷达)…', flush=True)
    t0 = time.time()
    while time.time() - t0 < 10:
        node.send(PWM_MID)
        time.sleep(0.1)
        if node.last_t and (args.no_guard or node.guard.gap() is not None):
            break
    if node.last_t is None:
        print('等不到 /encoder —— 底盘在跑吗?ROS_DOMAIN_ID 对吗?')
        neutral_exit()
    if not args.no_guard and node.guard.gap() is None:
        print('等不到 /scan —— 雷达没起来?要么修雷达,要么加 --no-guard 自己盯着')
        neutral_exit()

    print(f'\n模式 {args.mode}:{start} -> {stop} 步长 {step},每档 {args.hold}s'
          + (f',起步 kick {args.kick} 持续 {args.kick_sec}s' if down else ''), flush=True)
    print(f'{"PWM":>5} {"平均mm/s":>9} {"末段mm/s":>9} {"在动":>5} {"累计m":>6}', flush=True)

    answer = None
    try:
        pwm = start
        while (pwm >= stop) if down else (pwm <= stop):
            if down:
                # 先带起来再掉到候选值 —— 测的是"维持",不是"起步"。
                # 必须爬上去,不能从中位直接跳到 kick(见 ramp_to 的注释)。
                ramp_to(node, args.kick, args.no_guard)
                hold(node, args.kick, args.kick_sec, args.no_guard)
                if node.speed < MOVING:
                    print(f'kick {args.kick} 都没动起来,提高 --kick', flush=True)
                    break
            node.take()
            hold(node, pwm, args.hold, args.no_guard)
            s = [v * 1000 for v in node.take()]
            if not s:
                print(f'{pwm:>5} 没有编码器数据', flush=True)
                break
            tail = s[-max(1, len(s) // 3):]          # 末段才是稳态
            moving = statistics.mean(tail) > MOVING * 1000
            print(f'{pwm:>5} {statistics.mean(s):>9.0f} {statistics.mean(tail):>9.0f} '
                  f'{"是" if moving else "否":>5} {node.distance:>6.1f}', flush=True)

            if down and not moving:
                answer = pwm
                print(f'\n>>> 维持点:{pwm} 掉下去停了,上一档 {pwm - step} 还在动', flush=True)
                break
            if not down and moving:
                answer = pwm
                print(f'\n>>> 破静点:{pwm} 能从静止起步', flush=True)
                break

            for _ in range(int(args.pause / 0.05)):
                node.send(PWM_MID)
                time.sleep(0.05)
            node.take()
            if node.distance > args.max_dist:
                print(f'已跑出 {node.distance:.1f}m,到设定上限,中止', flush=True)
                break
            pwm += step
        else:
            print(f'\n扫到 {stop} 也没出结果,把范围放宽', flush=True)
    except Blocked as e:
        print(f'\n中止:{e}', flush=True)

    if answer is not None:
        key = 'pwm_floor' if down else 'pwm_breakaway'
        print(f'填进 teleop/speed_loop.py 的 SpeedLoopConfig:{key} = {answer}', flush=True)
    neutral_exit()


if __name__ == '__main__':
    main()
