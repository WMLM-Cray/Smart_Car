#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_encoder_gate_v2_ros.py —— encoder_gate_v2 ROS 订阅/转发/诊断层集成测试（需 ROS2，WSL/车机执行）。

不连接真实串口。验证 ROS 壳的「订阅 /encoder → gate_check → 转发 /encoder_gate + 1Hz /encoder_gate_diag」链路。

实现方式说明（重要，勿误读为"复现即通过"）：
- gate_check / new_state / diag_line / 阈值常量 **import 自真实 encoder_gate_v2**（决策逻辑是真实代码，非复刻）；
- ROS 壳（订阅/转发/诊断话题）按 encoder_gate_v2.main() 的 cb/diag_tick **逐行复现**，原因是
  WSL2 跨进程 FastDDS UDP multicast 不通（RMW 未配置、无 loopback multicast），subprocess 拉起真实 main()
  时 probe 订不到 /encoder_gate_diag（已实测：ros2 topic echo --once 无输出，而 gate 日志每秒在发）。
  车机原生 Linux 无此限制；本脚本单进程多节点验证 ROS 层组合逻辑，跨进程/record-dir 分支未覆盖（非核心）。

覆盖：
  1. 正常帧 → /encoder_gate 收到原帧（转发）
  2. abs 异常帧(|v|>1.2) → 不转发
  3. dv 跳变帧(|Δv|>0.35/帧) → 不转发
  4. 连续 3 帧一致 dv 拒 → 第 3 帧重锁转发（防 B1 死锁）
  5. 重锁后恢复正常帧 → 转发
  6. /encoder_gate_diag 1Hz 有消息，含 drop_abs/drop_dv/relocks 计数

用法（WSL/车机，需 source 后执行）：
  python3 test_encoder_gate_v2_ros.py
"""
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import encoder_gate_v2 as gv


def _odom(v):
    m = Odometry()
    m.twist.twist.linear.x = v
    m.header.stamp.sec = 1
    return m


def main():
    rclpy.init()

    # ---- gate 节点：逐行复现 encoder_gate_v2.main() 的 ROS 壳（去掉 record-dir 分支）----
    gate_node = Node('encoder_gate_v2')
    pub_gate = gate_node.create_publisher(Odometry, '/encoder_gate', 100)
    diag_pub = gate_node.create_publisher(String, '/encoder_gate_diag', 10)
    st = gv.new_state()

    def cb(m):
        v = m.twist.twist.linear.x
        now = time.time()
        st['last_input_t'] = now
        st['last_input_v'] = round(v, 4)
        accept, reason = gv.gate_check(v, st)
        if accept:
            pub_gate.publish(m)                 # 原消息原戳,不改不造
            st['last_output_t'] = now
            st['last_output_v'] = round(v, 4)
        elif reason in ('abs', 'dv'):
            gate_node.get_logger().warn(f'拒帧({reason}): v={v:.3f} {gv.diag_line(st, now)}')

    gate_node.create_subscription(Odometry, '/encoder', cb, 100)

    def diag_tick():
        msg = String()
        msg.data = gv.diag_line(st, time.time())
        diag_pub.publish(msg)

    gate_node.create_timer(1.0, diag_tick)

    # ---- probe 节点：订转发/诊断，发输入 ----
    probe = Node('encoder_gate_v2_probe')
    fwd = []
    diag = []

    def on_fwd(m):
        fwd.append(round(m.twist.twist.linear.x, 2))

    def on_diag(m):
        diag.append(m.data)

    probe.create_subscription(Odometry, '/encoder_gate', on_fwd, 10)
    probe.create_subscription(String, '/encoder_gate_diag', on_diag, 10)
    pub_enc = probe.create_publisher(Odometry, '/encoder', 10)

    def spin(t):
        end = time.time() + t
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(probe, timeout_sec=0.02)
            rclpy.spin_once(gate_node, timeout_sec=0.02)

    spin(1.0)          # 起节点 + discovery 稳定

    # 正式序列（每帧后 spin 0.3s 等转发）
    for v in (0.5, 0.6, 1.5, 1.0, 1.0, 1.02, 1.1):
        pub_enc.publish(_odom(v))
        spin(0.3)
    spin(1.5)          # 排空 + 等一条新 diag（含计数）

    gate_node.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()

    fwd_set = set(fwd)
    diag_ok = len(diag) > 0 and any("drop_abs=" in d for d in diag)

    results = [
        ("正常帧转发 0.5", 0.5 in fwd_set),
        ("正常帧转发 0.6", 0.6 in fwd_set),
        ("abs 异常不转发 1.5", 1.5 not in fwd_set),
        ("dv 跳变不转发 1.0", 1.0 not in fwd_set),
        ("重锁第3帧转发 1.02", 1.02 in fwd_set),
        ("重锁后恢复转发 1.1", 1.1 in fwd_set),
        ("diag 1Hz 有消息含计数", diag_ok),
    ]

    print("=" * 60)
    print("收到 /encoder_gate 的 v 值: %s" % sorted(fwd))
    print("diag 样例: %s" % (diag[-1] if diag else "(无)"))
    ok = True
    for name, passed in results:
        print(("PASS " if passed else "FAIL ") + name)
        ok = ok and passed
    print("=" * 60)
    print("encoder_gate_v2 ROS 集成测试: %s" % ("全部通过" if ok else "存在失败"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
