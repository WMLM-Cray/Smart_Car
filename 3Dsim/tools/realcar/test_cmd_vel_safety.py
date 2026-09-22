#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_cmd_vel_safety.py —— cmd_vel_safety 下游输出语义隔离测试（需 ROS2，WSL/车机执行）。

不连接真实串口，只验 /cmd_vel → [CmdVelSafety] → /car_cmd_vel 的发布语义，捕获三种下游输出：
  A. 非零指令 → 下游收到限幅后非零
  B. 非零后显式全零 → 下游立即收到全零（on_cmd 无条件 publish，含零；**非"停发"，是显式发零**）
  C. 非零后断流 → 超时(cmd_timeout=0.5s)后按 zero_repeat(0.5s) 周期重发零
  D. 退出零(publish_zero) → 下游收到全零

注意：CmdVelSafety 构造后 last_cmd_t=构造时刻，若 0.5s 内无指令即进入超时发零（设计行为）。
      故每个场景前先 clear 残留，再触发目标输入。

用法（WSL/车机，需 source 后执行）：
  python3 test_cmd_vel_safety.py
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from cmd_vel_safety import CmdVelSafety


def main():
    rclpy.init()
    probe = Node("cmd_vel_safety_probe")
    received = []  # (linear.x, angular.z, monotonic)

    def on_car(msg):
        received.append((msg.linear.x, msg.angular.z, time.monotonic()))

    probe.create_subscription(Twist, "/car_cmd_vel", on_car, 10)
    pub_cmd = probe.create_publisher(Twist, "/cmd_vel", 10)
    safety = CmdVelSafety()

    def spin(t):
        end = time.time() + t
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(probe, timeout_sec=0.02)
            rclpy.spin_once(safety, timeout_sec=0.02)

    def pub(v, w):
        m = Twist()
        m.linear.x = v
        m.angular.z = w
        pub_cmd.publish(m)

    results = []
    spin(0.5)          # 起节点 + 订阅对齐（期间可能已有启动超时零，下面清掉）
    received.clear()

    # A. 非零 → 下游收到非零
    pub(0.25, 0.0)
    spin(0.3)
    a = any(v != 0.0 for (v, w, _) in received)
    results.append(("A 非零指令→下游收到非零", a))
    received.clear()

    # B. 非零后显式全零 → 立即收到全零（不是停发）
    pub(0.25, 0.0)
    spin(0.15)
    pre_nonzero = any(v != 0.0 for (v, w, _) in received)
    received.clear()
    pub(0.0, 0.0)
    spin(0.3)
    b = pre_nonzero and any(v == 0.0 and w == 0.0 for (v, w, _) in received)
    results.append(("B 非零后显式全零→立即发零", b))
    received.clear()

    # C. 非零后断流 → 超时后周期重发零
    pub(0.25, 0.0)
    spin(0.15)
    received.clear()   # 清掉非零，此后断流
    spin(1.4)          # > cmd_timeout(0.5s) + 一个 zero_repeat 窗口
    c = any(v == 0.0 and w == 0.0 for (v, w, _) in received)
    results.append(("C 断流超时后周期重发零", c))
    received.clear()

    # D. 退出零 publish_zero → 下游收到全零
    safety.publish_zero("测试退出")
    spin(0.3)
    d = any(v == 0.0 and w == 0.0 for (v, w, _) in received)
    results.append(("D 退出零→下游收到全零", d))

    safety.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()

    print("=" * 60)
    ok = True
    for name, passed in results:
        print(("PASS " if passed else "FAIL ") + name)
        ok = ok and passed
    print("=" * 60)
    print("cmd_vel_safety 隔离测试: %s" % ("全部通过" if ok else "存在失败"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
