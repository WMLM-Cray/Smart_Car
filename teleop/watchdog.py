#!/usr/bin/env python3
"""底盘看门狗:发指令的一停,就把电调拉回中位。

厂商的 racecar_driver_node.cpp(83 行,纯回调)收到 Twist 才写串口,没收到就什么也不做 ——
所以串口上最后一个 PWM 会一直保持。上游崩了、断网了、被 kill 了,车都按当前油门一直跑。
2026-09-17 车自己往前冲就是这么来的:racecar-web 一直在发油门,car.sh 一起来,
racecar_driver 上线就把它写进了电调。

这个节点不碰厂商代码,只在同一个话题上补一条中位。厂商发新压缩包也冲不掉。

只管"发指令的停了",管不了"发指令的卡住了" —— 后者要在遥控端解决(浏览器侧)。
"""
from __future__ import annotations

import time

SPEED_MID = 1550           # [代码] 电调中位
FWD_START = 1600           # [实测] >= 这个值车往前走(1599->0,1600->488mm/s,中间没有值)
REV_START = 1500           # [代码] <= 这个值车往后走
# 超时必须大于占空比周期(server.py 的 DUTY_TICKS x 50ms = 0.3s)。低速爬行时"会让车动"的
# 指令本来就是稀疏的,超时比周期短的话,看门狗的中位会插进两个爬行脉冲之间把它们抵消掉。
TIMEOUT = 0.5
ASSERT_FOR = 0.5           # 触发后按 20Hz 补这么久的中位,确保厂商驱动至少收到几条


def moves(pwm: float) -> bool:
    """这条指令会不会让车动。中位附近那段是电调死区,不动。"""
    return pwm >= FWD_START or pwm <= REV_START


class Watchdog:
    """喂 on_cmd(),每拍问 tick():该不该发中位。

    只有"会让车动"的指令才武装看门狗 —— 自己发出去的中位也会被自己收到,
    不能让它给自己续命。
    """

    def __init__(self, timeout: float = TIMEOUT, assert_for: float = ASSERT_FOR):
        self.timeout, self.assert_for = timeout, assert_for
        self.armed = False
        self.last_move = 0.0
        self.assert_until = 0.0

    def on_cmd(self, pwm: float, now: float | None = None):
        now = time.time() if now is None else now
        if moves(pwm):
            self.armed = True
            self.last_move = now
            self.assert_until = 0.0        # 车又动起来了,不用再补中位

    def tick(self, now: float | None = None) -> bool:
        now = time.time() if now is None else now
        if self.armed and now - self.last_move > self.timeout:
            self.armed = False             # 补完这一轮就休眠,等下一条会动的指令再武装
            self.assert_until = now + self.assert_for
        return now < self.assert_until


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    TOPIC = '/teleop_cmd_vel'
    TURN_MID = 84

    class WatchdogNode(Node):
        def __init__(self):
            super().__init__('teleop_watchdog')
            self.dog = Watchdog()
            self.pub = self.create_publisher(Twist, TOPIC, 5)
            self.create_subscription(Twist, TOPIC, self.on_cmd, 5)
            self.create_timer(0.05, self.tick)
            self.get_logger().info(
                f'看门狗就位:{TOPIC} 停发 {TIMEOUT}s 就补中位 {SPEED_MID}')

        def on_cmd(self, msg):
            self.dog.on_cmd(msg.linear.x)

        def tick(self):
            if not self.dog.tick():
                return
            tw = Twist()
            tw.linear.x, tw.angular.z = float(SPEED_MID), float(TURN_MID)
            self.pub.publish(tw)

    rclpy.init()
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                              # Ctrl+C / systemd stop,正常收摊
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
