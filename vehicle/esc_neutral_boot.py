#!/usr/bin/env python3
"""底盘驱动一起来就持续发中位 1540,确认驱动订阅上之后再发 HOLD_AFTER 秒退出。

电调上电那一刻收到什么油门,就把什么锁成中位(2026-09-30 架空实证:上电时发 1600 → 1600 静止、
1540 反转;当天早上车开机后中位锁在 ~1500,整条曲线低 40)。厂家 racecar_driver 只在收到指令时才往
下位机发 PWM、之后保持最后一个值,所以驱动一起来先喂一段 1540,之后电调开关什么时候打开都锁在 1540。
驱动起来之前(开机 1~2.5 分钟)电调收到的是下位机固件的默认值 —— 这段时间不能开电调,本脚本管不到。

由 racecar-neutral.service 在底盘服务起来后运行(底盘重启时跟着重跑)。
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.params import CarParams

CMD_TOPIC = '/teleop_cmd_vel'
PERIOD = 0.05
HOLD_AFTER = 5.0        # 看到驱动订阅之后再发这么久(DDS 刚连上的头几帧可能丢)
GIVE_UP = 180.0         # 这么久还没看到驱动就退出(服务会记日志)


def main():
    import rclpy
    from geometry_msgs.msg import Twist

    p = CarParams()
    rclpy.init()
    node = rclpy.create_node('esc_neutral_boot')
    pub = node.create_publisher(Twist, CMD_TOPIC, 5)
    tw = Twist()
    tw.linear.x, tw.angular.z = float(p.pwm_mid), float(p.turn_mid)
    log = node.get_logger()
    t0, seen = time.time(), None
    while time.time() - t0 < GIVE_UP:
        pub.publish(tw)
        if seen is None and 'racecar_driver' in {
                i.node_name for i in node.get_subscriptions_info_by_topic(CMD_TOPIC)}:
            seen = time.time()
            log.info(f'底盘驱动已订阅 {CMD_TOPIC}(开机后 {seen - t0:.1f}s),再发 {HOLD_AFTER:.0f}s 中位')
        if seen is not None and time.time() - seen >= HOLD_AFTER:
            log.info(f'中位 {p.pwm_mid} 已送到底盘驱动,之后驱动保持它;现在可以打开电调')
            return
        time.sleep(PERIOD)
    log.error(f'{GIVE_UP:.0f}s 没看到 racecar_driver 订阅 {CMD_TOPIC} —— 底盘没起来,先别开电调')
    sys.exit(1)


if __name__ == '__main__':
    main()
