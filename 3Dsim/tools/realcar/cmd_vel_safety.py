#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel_safety:实车安全中继层(2026-09-16 控制链审查轮补齐项)

链路:nav2(/cmd_vel) ─▶ [本节点:检查/限幅/超时停车] ─▶ /car_cmd_vel ─▶ racecar_driver_node
                                                                     └▶ /dev/car(公式 ×100+1540)

背景(REALCAR_CONTROL_AUDIT.md §2/§8):driver 零防护——无输入检查、无限幅、无 watchdog、
退出不发停车;下位机失联不可感知(T10)。本节点补齐上位机侧防护:
  1. 输入检查:linear.x/angular.z 非有限值(NaN/inf)按 0 处理并告警;
  2. 限幅:|linear.x|<=max_linear(默认 0.10 m/s,任务书阶段4 上限)、
           |angular.z|<=max_angular(默认 0.35,对应舵机约 ±455µs 内);
  3. 超时停车:cmd_timeout(默认 0.5s)无新指令 → 每 zero_repeat(默认 0.5s)重发一次零
     (v=0,z=0 → motor=1540/servo=1500,字节级已联调验证),直到新指令到达;
  4. 启动默认不发送任何消息(publish_start_zero:=true 才发一条初始零);
  5. 退出(收到 SIGINT 后)尽力发一条零再退出(下游 driver 退出行为另见 T7,不依赖)。

不改变"速度→PWM 只在 driver 一处"的约定;/teleop_cmd_vel 直通通道不经本层(仅标定/应急用)。

用法(车机):
  python3 cmd_vel_safety.py [--ros-args -p max_linear:=0.10 -p max_angular:=0.35 ...]
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelSafety(Node):
    def __init__(self):
        super().__init__("cmd_vel_safety")
        self.declare_parameter("max_linear", 0.10)      # m/s,任务书阶段4 上限
        self.declare_parameter("max_angular", 0.35)     # driver 系数下 ±0.385 打满舵机,留余量
        self.declare_parameter("cmd_timeout", 0.5)      # s,无指令超时
        self.declare_parameter("zero_repeat", 0.5)      # s,超时期间零速重发周期
        self.declare_parameter("publish_start_zero", False)

        p = lambda n: self.get_parameter(n).value
        self.max_linear = float(p("max_linear"))
        self.max_angular = float(p("max_angular"))
        self.cmd_timeout = float(p("cmd_timeout"))
        self.zero_repeat = float(p("zero_repeat"))

        self.sub = self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 5)
        self.pub = self.create_publisher(Twist, "/car_cmd_vel", 5)
        self.last_cmd_t = self.get_clock().now()
        self.last_zero_t = None
        self.in_timeout = False

        self.timer = self.create_timer(0.05, self.on_tick)  # 20Hz 检查
        self.get_logger().info(
            f"安全中继就绪: max_linear={self.max_linear} max_angular={self.max_angular} "
            f"timeout={self.cmd_timeout}s zero_repeat={self.zero_repeat}s "
            f"start_zero={bool(p('publish_start_zero'))}")
        if p("publish_start_zero"):
            self.publish_zero("启动零指令(参数要求)")

    def publish_zero(self, why: str):
        t = Twist()  # 全零字段
        self.pub.publish(t)
        self.last_zero_t = self.get_clock().now()
        self.get_logger().info(f"发零速: {why}")

    def on_cmd(self, msg: Twist):
        v, w = msg.linear.x, msg.angular.z
        ov = ow = 0.0
        if not math.isfinite(v):
            self.get_logger().warn(f"输入 linear.x={v} 非有限值,按 0 处理")
            v = 0.0
        if not math.isfinite(w):
            self.get_logger().warn(f"输入 angular.z={w} 非有限值,按 0 处理")
            w = 0.0
        if v > self.max_linear:
            ov, v = v, self.max_linear
        elif v < -self.max_linear:
            ov, v = v, -self.max_linear
        if w > self.max_angular:
            ow, w = w, self.max_angular
        elif w < -self.max_angular:
            ow, w = w, -self.max_angular
        if ov:
            self.get_logger().warn(f"限幅 linear.x {ov:.3f}->{v:.3f}")
        if ow:
            self.get_logger().warn(f"限幅 angular.z {ow:.3f}->{w:.3f}")

        out = Twist()
        out.linear.x = v
        out.angular.z = w
        self.pub.publish(out)
        self.last_cmd_t = self.get_clock().now()
        if self.in_timeout:
            self.get_logger().info("指令恢复,退出超时零速状态")
            self.in_timeout = False

    def on_tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_t).nanoseconds * 1e-9
        if dt <= self.cmd_timeout:
            return
        # 已超时:按 zero_repeat 周期持续重发零(下位机无 watchdog,重发防丢帧)
        if (not self.in_timeout) or (
                self.last_zero_t is None or
                (now - self.last_zero_t).nanoseconds * 1e-9 >= self.zero_repeat):
            if not self.in_timeout:
                self.get_logger().warn(f"超时 {dt:.2f}s 无指令 → 零速(此后每 {self.zero_repeat}s 重发)")
                self.in_timeout = True
            self.publish_zero("超时保持")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafety()
    # rclpy.init 默认装的 SIGINT handler 会先 context.shutdown(),导致 finally 里
    # 退出零必败(publisher's context is invalid,2026-09-16 T16 实测)。
    # 这里卸掉它,自己接管信号 → 置 flag → 退出循环时 context 仍有效,退出零真正发出。
    try:
        from rclpy.signals import uninstall_signal_handlers
        uninstall_signal_handlers()
    except Exception as e:
        node.get_logger().warn(f"卸载 rclpy 信号处理失败(退出发零可能无效): {e}")
    import signal as _sig
    stop = {"flag": False}

    def _on_sig(signum, frame):
        stop["flag"] = True

    _sig.signal(_sig.SIGINT, _on_sig)
    _sig.signal(_sig.SIGTERM, _on_sig)
    try:
        while not stop["flag"] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_zero("节点退出")
        except Exception as e:  # 尽力而为
            node.get_logger().warn(f"退出发零失败: {e}")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # context 可能已被外部关闭


if __name__ == "__main__":
    main()
