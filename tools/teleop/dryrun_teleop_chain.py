#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手动链全链虚拟验证:模拟 /joy → racecar_teleop →(remap)/cmd_vel → cmd_vel_safety → /car_cmd_vel
(2026-09-17 手柄接入轮·任务书A"虚拟串口测试"的链路部分)

覆盖场景(任务书A:方向、限幅、松使能、断流、节点退出、重连锁定——实体手柄未到场,
先用 20Hz 模拟 /joy 注入代替 joy_node,实体手柄到场后同场景重跑):
  S1 启动锁定:上电即"按住使能"不运动(判据1)
  S2 松开→按住→满油门:/car_cmd_vel.linear.x 收敛 0.10(限幅对齐),angular≈0
  S3 松使能:立即零
  S4 满舵转向:angular.z≈+0.101(v·tan(δ)/L, 0.10·tan(0.35)/0.36)
  S5 断流 1.0s:teleop 0.3s 锁定零 → safety 0.5s 超时零速重发,/car_cmd_vel 恒零
  S6 判据3b实链:恢复发送(仍按住使能+满油门)→ 保持零(需 e2113b0 补丁)
  S7 锁定恢复(油门门 2026-09-17):杆仍推着时松开重按使能不得起步;
     油门轴回死区一次后才恢复运动(防"重连后仍推着摇杆,一按使能突然起步")
  S8 kill teleop:safety 超时零速,/car_cmd_vel 恒零

串口帧层(→/dev/car)不在本脚本:该层已由 6bb97d8 无动力联调对真实 driver 字节级验证
(帧 00 AA m_lo m_hi s_lo s_hi ck 55,motor=v×100+1540/servo=1500−ω×1300);
本脚本断言到 /car_cmd_vel,帧值按公式离线并列打印供对账。
车上如需带真 driver 的帧级全链:--serial /tmp/fakecar(须电调断开,复用 dryrun 先例)。

用法(WSL,source 过 ROS 与 teleop 工作区后):
  python3 dryrun_teleop_chain.py [teleop参数yaml] [--serial /tmp/fakecar]
输出: stdout 场景 PASS/FAIL + 汇总;退出码 0=全过
"""
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# 实车口径期望值(与 realcar_teleop.yaml / cmd_vel_safety.py 默认一致)
import math  # noqa: E402

V_MAX = 0.10                                          # 满油门=安全层上限
W_EXPECT = round(V_MAX * math.tan(0.35) / 0.36, 6)    # ω=v·tan(δ)/L ≈0.1013(左满舵)
MOTOR_VMAX = 1540 + round(V_MAX * 100)                # =1550(帧值对账打印用)
SERVO_WEXPECT = 1500 - round(W_EXPECT * 1300)         # 左满舵期望舵机值

PARAMS_DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "realcar_teleop.yaml")

AX_SPEED, AX_STEER = 1, 3
BTN_ENABLE, BTN_LOW = 5, 4


def joy_msg(throttle=0.0, steer=0.0, enable=False):
    m = Joy()
    m.axes = [0.0, throttle, 0.0, steer]
    m.buttons = [0] * 6
    m.buttons[BTN_ENABLE] = 1 if enable else 0
    return m


class ChainChecker(Node):
    """发 /joy(20Hz 由外部节拍驱动)+ 收 /car_cmd_vel 与 /cmd_vel。"""

    def __init__(self):
        super().__init__("dryrun_teleop_chain")
        self.pub_joy = self.create_publisher(Joy, "/joy", 10)
        self.sub_cv = self.create_subscription(Twist, "/car_cmd_vel", self.on_cv, 20)
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 20)
        self.cv_events = []   # (t, v, w)
        self.cmd_events = []

    def on_cv(self, msg):
        self.cv_events.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def on_cmd(self, msg):
        self.cmd_events.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def send_joy(self, throttle=0.0, steer=0.0, enable=False, seconds=0.05, hz=20.0):
        """以 hz 持续发指定 joy 状态 seconds 秒(模拟 autorepeat),同时 spin 收集。"""
        period = 1.0 / hz
        n = max(1, int(seconds * hz))
        for _ in range(n):
            self.pub_joy.publish(joy_msg(throttle, steer, enable))
            end = time.monotonic() + period
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.005)

    def idle(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def tail(self, window=0.4):
        """最近 window 秒内的 /car_cmd_vel 样本。"""
        t0 = time.monotonic() - window
        return [(v, w) for (t, v, w) in self.cv_events if t >= t0]

    def tail_cmd(self, window=0.4):
        t0 = time.monotonic() - window
        return [(v, w) for (t, v, w) in self.cmd_events if t >= t0]


def main():
    params_file = PARAMS_DEFAULT
    serial = None
    args = sys.argv[1:]
    if args and not args[0].startswith("-"):
        params_file = os.path.abspath(args[0])
    if "--serial" in args:
        serial = args[args.index("--serial") + 1]
    if not os.path.isfile(params_file):
        print("缺少参数文件:", params_file); return 2

    procs = {}

    def start(key, cmd):
        procs[key] = subprocess.Popen(
            cmd, shell=True,
            stdout=open("/tmp/dryrun_%s.log" % key, "w"),
            stderr=subprocess.STDOUT)
        print("  起 %s(pid=%s): %s" % (key, procs[key].pid, cmd))

    def stop(key):
        p = procs.pop(key, None)
        if p:
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()

    results = []

    def check(sid, desc, ok, detail=""):
        results.append((sid, desc, ok))
        print("%s [%s] %s%s" % ("PASS" if ok else "FAIL", sid, desc,
                                ("  -> " + detail) if detail else ""))

    # 依赖自查 -----------------------------------------------------------
    import ament_index_python
    try:
        ament_index_python.packages.get_package_prefix("racecar_teleop")
    except Exception:
        print("!! racecar_teleop 未安装(source teleop_ws/install/setup.bash?)"); return 2
    safety_py = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             os.pardir, "realcar", "cmd_vel_safety.py")
    if not os.path.isfile(safety_py):
        print("!! 找不到 cmd_vel_safety.py:", safety_py); return 2

    # 起链:safety → (可选 driver+pty) → teleop(remap /cmd_vel) ---------
    start("safety", "python3 %s" % safety_py)
    if serial:
        start("driver", "ros2 run racecar_driver racecar_driver_node "
              "--ros-args -p serial_port:=%s" % serial)
    start("teleop", "ros2 run racecar_teleop teleop_node --ros-args "
          "--params-file %s -r /teleop/cmd_vel:=/cmd_vel" % params_file)
    time.sleep(2.0)

    rclpy.init()
    node = ChainChecker()
    try:
        # S1 启动锁定:上电即按住使能(未先松开),不运动
        node.send_joy(throttle=1.0, enable=True, seconds=0.6)
        tail = node.tail(0.4)
        ok = tail and all(abs(v) < 1e-6 and abs(w) < 1e-6 for v, w in tail)
        check("S1", "上电即按住使能不运动(判据1)", ok,
              "最近样本=%s" % (tail[-3:] if tail else "无"))

        # S2 松开→按住→满油门直行
        node.send_joy(enable=False, seconds=0.2)            # 松开=解锁
        node.send_joy(throttle=1.0, enable=True, seconds=0.8)
        tail = node.tail(0.4)
        v_ok = any(abs(v - V_MAX) < 0.005 for v, _w in tail)
        w_ok = all(abs(w) < 1e-6 for _v, w in tail if abs(_v) > 0.05)
        check("S2", "解锁后满油门: linear.x→0.10 限幅, angular≈0",
              v_ok and w_ok, "样本=%s" % (tail[-3:] if tail else "无"))

        # S3 松使能立即零
        node.send_joy(throttle=1.0, enable=False, seconds=0.5)
        tail = node.tail(0.3)
        ok = tail and all(abs(v) < 1e-6 for v, _w in tail)
        check("S3", "松使能立即零速", ok, "样本=%s" % (tail[-3:] if tail else "无"))

        # S4 满舵转向(左)
        node.send_joy(enable=False, seconds=0.2)
        node.send_joy(throttle=1.0, steer=1.0, enable=True, seconds=0.8)
        tail = node.tail(0.4)
        w_ok = any(abs(w - W_EXPECT) < 0.005 for _v, w in tail)
        check("S4", "满舵转向 angular.z≈%.4f(实测样本含 %.4f)" % (W_EXPECT,
              tail[-1][1] if tail else -1), w_ok,
              "帧值对账: motor=%d servo=%d(公式×100+1540/1500−ω×1300)"
              % (MOTOR_VMAX, SERVO_WEXPECT))

        # S5 断流 1.0s(不再发 /joy):teleop 0.3s 锁定→零;安全层 0.5s 超时零
        node.idle(1.2)
        tail = node.tail(0.5)
        ok = tail and all(abs(v) < 1e-6 and abs(w) < 1e-6 for v, w in tail)
        check("S5", "断流后 /car_cmd_vel 恒零(teleop锁定+safety超时零速)", ok,
              "样本=%s" % (tail[-3:] if tail else "无"))

        # S6 判据3b实链:恢复发送,仍按住使能+满油门 → 应保持零
        node.send_joy(throttle=1.0, steer=0.0, enable=True, seconds=0.8)
        tail = node.tail(0.4)
        ok = tail and all(abs(v) < 1e-6 and abs(w) < 1e-6 for v, w in tail)
        check("S6", "断流恢复(仍按使能)不自行运动(判据3b,e2113b0补丁)", ok,
              "样本=%s" % (tail[-3:] if tail else "无"))

        # S7 锁定恢复:杆仍推着时松开重按不得起步(油门门);回死区一次后恢复
        node.send_joy(throttle=0.9, enable=False, seconds=0.2)   # 松使能,杆在死区外
        node.send_joy(throttle=0.9, enable=True, seconds=0.5)    # 重按使能,杆仍推着
        tail_hold = node.tail(0.2)
        ok_hold = tail_hold and all(abs(v) < 1e-6 for v, _w in tail_hold)
        node.send_joy(throttle=0.0, enable=True, seconds=0.3)    # 杆回死区一次(使能保持)
        node.send_joy(throttle=1.0, enable=True, seconds=0.6)    # 再推杆
        tail_move = node.tail(0.3)
        ok_move = any(abs(v - V_MAX) < 0.005 for v, _w in tail_move)
        check("S7", "锁定恢复:杆仍推着重按使能不得起步,回死区一次后恢复(判据4+7)",
              ok_hold and ok_move,
              "按使能(杆0.9)样本=%s 回中+推杆后样本=%s" %
              (tail_hold[-3:] if tail_hold else "无",
               tail_move[-3:] if tail_move else "无"))

        # S8 teleop 进程退出 → safety 0.5s 超时零速
        stop("teleop")
        node.idle(1.2)
        tail = node.tail(0.5)
        ok = tail and all(abs(v) < 1e-6 and abs(w) < 1e-6 for v, w in tail)
        check("S8", "teleop 退出后 safety 超时零速", ok,
              "样本=%s" % (tail[-3:] if tail else "无"))
    finally:
        for k in list(procs):
            stop(k)
        node.destroy_node()
        rclpy.shutdown()

    n_fail = sum(1 for r in results if not r[2])
    print("\n==== 全链复验汇总: S1-S8, PASS=%d FAIL=%d ====" %
          (len(results) - n_fail, n_fail))
    print("帧值对账(未实测串口,引用6bb97d8字节级联调结论): "
          "满油门 motor=%d servo=1500 | 左满舵 servo=%d" % (MOTOR_VMAX, SERVO_WEXPECT))
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
