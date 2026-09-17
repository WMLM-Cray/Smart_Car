#!/usr/bin/env python3
"""PWM 扫描:从中位往上逐档给油门,用编码器记录每档速度,找最低能稳定跑的 PWM。
用法(车上):python3 pwm_sweep.py [起始 终止 步长 每档秒数],默认 1555 1620 5 2
"""
import os, signal, statistics, sys, threading, time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SPEED_MID, TURN_MID = 1550, 84
ABORT_SPEED = 1.5            # m/s,超过就中止


class Sweep(Node):
    def __init__(self):
        super().__init__('pwm_sweep')
        self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
        self.create_subscription(Odometry, '/encoder', self.on_enc, 10)
        self.samples = []
        self.lock = threading.Lock()

    def on_enc(self, m):
        with self.lock:
            self.samples.append(m.twist.twist.linear.x)

    def send(self, pwm):
        tw = Twist(); tw.linear.x = float(pwm); tw.angular.z = float(TURN_MID)
        self.pub.publish(tw)

    def take(self):
        with self.lock:
            s, self.samples = self.samples, []
        return s


def main():
    start, stop, step, hold = (list(map(float, sys.argv[1:5])) + [1555, 1620, 5, 2][len(sys.argv) - 1:])[:4]
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Sweep()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def neutral_exit(*_):
        for _ in range(5):
            node.send(SPEED_MID); time.sleep(0.05)
        print('已回中位', flush=True)
        os._exit(0)          # sys.exit 会和 rclpy 的 spin 线程冲突,C++ 层直接 abort
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, neutral_exit)

    def hold_pwm(pwm, sec):
        end = time.time() + sec
        while time.time() < end:
            node.send(pwm); time.sleep(0.05)

    try:
        print('中位 2s(让电调解锁)...', flush=True)
        hold_pwm(SPEED_MID, 2)
        node.take()
        print(f'{"PWM":>5} {"平均mm/s":>9} {"最小":>6} {"最大":>6} {"波动(std)":>9}', flush=True)
        pwm = start
        while pwm <= stop:
            hold_pwm(pwm, hold / 2)          # 前一半等速度稳定
            node.take()
            hold_pwm(pwm, hold / 2)          # 后一半采样
            s = [v * 1000 for v in node.take()]
            if s:
                print(f'{pwm:>5.0f} {statistics.mean(s):>9.0f} {min(s):>6.0f} {max(s):>6.0f} '
                      f'{statistics.pstdev(s):>9.0f}', flush=True)
                if max(abs(v) for v in s) > ABORT_SPEED * 1000:
                    print('速度超限,中止'); break
            else:
                print(f'{pwm:>5.0f}  没有编码器数据(car.sh 在跑吗?)', flush=True); break
            pwm += step
    finally:
        neutral_exit()


if __name__ == '__main__':
    main()
