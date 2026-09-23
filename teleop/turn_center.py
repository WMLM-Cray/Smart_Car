#!/usr/bin/env python3
"""找舵机真中位:保持一个舵机值,看前轮正不正。油门全程中位,车不会走。

`fastsim/params.py` 里 `turn_mid = 84` 标的是 `[代码]`,从厂家源码抄的,没有在
这台车上对过 —— 舵机装配角度、拉杆长度、舵盘齿位都会让真中位偏开几度。
2026-09-22 用户反映"转向初始稍微偏右",就是这个。

偏了的后果不只是走不直:`turn_mid` 是转角的零点,偏 n 个单位等于**每一次**转向
输出都带上 n×0.52° 的常量偏置(turn_span=60 对应 max_steer=0.55rad≈31°,
所以 1 单位 ≈ 0.52°)。策略在仿真里学的是零偏置,上车就一直往一边拽。

**数值增大 = 往左**(84 中位,144 左满舵,见 fastsim/pwm.py)。

用法(车上,架空或落地都行,油门不动):
    python3 turn_center.py            # 从 84 开始,按回车逐步调
    python3 turn_center.py 87         # 直接保持 87

调到前轮目测笔直,把那个值填进 fastsim/params.py 的 turn_mid,并记下是哪台车
哪次装配 —— 换过舵机或拉杆就要重测。
"""
import os
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from fastsim.params import CarParams

PWM_MID = 1550          # 油门全程中位,这个工具不让车走


class Turn(Node):
    def __init__(self):
        super().__init__('turn_center')
        self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
        self.turn = float(CarParams().turn_mid)

    def send(self):
        tw = Twist()
        tw.linear.x, tw.angular.z = float(PWM_MID), float(self.turn)
        self.pub.publish(tw)


def main():
    p = CarParams()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Turn()
    if len(sys.argv) > 1:
        node.turn = float(sys.argv[1])

    stop = threading.Event()

    def pump():
        while not stop.is_set():        # 20Hz 持续发,不然 watchdog 0.5s 就接管
            node.send()
            time.sleep(0.05)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=pump, daemon=True).start()

    def bye(*_):
        stop.set()
        node.turn = float(p.turn_mid)
        for _ in range(5):
            node.send()
            time.sleep(0.02)
        print('\n已回 turn_mid', flush=True)
        sys.stdout.flush()
        os._exit(0)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, bye)

    span, mx = p.turn_span, p.max_steer
    print(f'当前 turn_mid={p.turn_mid},满舵 ±{span}(对应 ±{mx:.2f} rad ≈ ±{mx*57.3:.0f}°)')
    print(f'1 个单位 ≈ {mx / span * 57.3:.2f}°。数值**增大 = 往左**。')
    print('输入新值回车;直接回车 +1(往左);- 回车 -1(往右);q 退出。\n')
    while True:
        try:
            line = input(f'  当前 {node.turn:.0f} > ').strip()
        except EOFError:
            break
        if line in ('q', 'quit'):
            break
        if line == '':
            node.turn += 1
        elif line == '-':
            node.turn -= 1
        else:
            try:
                node.turn = float(line)
            except ValueError:
                print('  给个数,或者回车/-/q')
                continue
        node.turn = max(p.turn_mid - span, min(p.turn_mid + span, node.turn))
        off = (node.turn - p.turn_mid) * mx / span
        print(f'  -> {node.turn:.0f}  偏移 {node.turn - p.turn_mid:+.0f} 单位 '
              f'= {off * 57.3:+.2f}°')
    bye()


if __name__ == '__main__':
    main()
