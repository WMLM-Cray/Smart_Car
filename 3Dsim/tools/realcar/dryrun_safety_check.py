#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
安全层验证(联调 T11-T16):cmd_vel_safety 的 检查/限幅/超时 行为,纯话题级,不碰任何串口。
v2:固定时间窗收集+any 断言(v1 的 wait_for(1) 被安全层周期零速重发打乱时序,全部错位一拍)。
用法(车机,source 后): python3 dryrun_safety_check.py [输出目录]
"""
import json
import os
import subprocess
import sys
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Checker(Node):
    def __init__(self):
        super().__init__("safety_checker")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 5)
        self.received = []          # (t, v, z)
        self.sub = self.create_subscription(Twist, "/car_cmd_vel", self.on_msg, 10)

    def on_msg(self, m):
        self.received.append((time.time(), m.linear.x, m.angular.z))

    def send(self, v, z):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(z)
        self.pub.publish(t)
        time.sleep(0.15)
        self.pub.publish(t)

    def collect(self, dur=0.9):
        """固定时长窗口内持续 spin 收全部消息(不清空中间状态)"""
        deadline = time.time() + dur
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        out = list(self.received)
        self.received.clear()
        return out


def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        "~/safety_check_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(outdir, exist_ok=True)
    log = open(os.path.join(outdir, "report.txt"), "w", encoding="utf-8")

    def say(msg=""):
        print(msg); log.write(msg + "\n"); log.flush()

    say(f"# 安全层验证 v2 {datetime.now().isoformat()}(纯话题级,不碰串口)")

    team = os.path.expanduser("~/racecar_team")
    proc = subprocess.Popen(["python3", os.path.join(team, "cmd_vel_safety.py")],
                            stdout=open(os.path.join(outdir, "safety_node.log"), "w"),
                            stderr=subprocess.STDOUT)
    rclpy.init()
    chk = Checker()
    time.sleep(2.0)  # 发现匹配
    results = {}

    def case(tag, desc, inputs, predicate):
        for (v, z) in inputs:
            chk.send(v, z)
        got = chk.collect(1.0)
        ok, detail = predicate(got)
        results[tag] = {"desc": desc, "ok": ok, "detail": detail,
                        "received": [(round(v, 4), round(z, 4)) for _, v, z in got]}
        say(f"== {tag}: {desc}")
        say(f"   窗口内收到 {len(got)} 条: {[(round(v,3), round(z,3)) for _, v, z in got]}")
        say(f"   判定: {'PASS' if ok else 'FAIL'} ({detail})")

    case("T11", "正常值透传 v=0.05 z=0.10", [(0.05, 0.10)],
         lambda g: (any(abs(v-0.05) < 1e-6 and abs(z-0.10) < 1e-6 for _, v, z in g), "含 0.05/0.10"))

    case("T12", "正向限幅 v=0.50 z=0.90 → 0.10/0.35", [(0.50, 0.90)],
         lambda g: (any(abs(v-0.10) < 1e-6 and abs(z-0.35) < 1e-6 for _, v, z in g), "钳到 0.10/0.35"))

    case("T15", "负向限幅 v=-0.50 → -0.10", [(-0.50, 0.0)],
         lambda g: (any(abs(v+0.10) < 1e-6 for _, v, z in g), "钳到 -0.10"))

    case("T13", "非有限值 NaN/inf → 0", [(float("nan"), float("inf"))],
         lambda g: (any(v == 0.0 and z == 0.0 for _, v, z in g), "NaN/inf 按 0 发"))

    # T14 超时:发一条正常值(应透传),静默 1.5s,再开窗 1.1s,应捕获周期零速
    chk.send(0.05, 0.0)
    first = chk.collect(0.8)
    passed = any(abs(v-0.05) < 1e-6 for _, v, z in first)
    time.sleep(1.5)
    got = chk.collect(1.1)
    zeros = [g for g in got if g[1] == 0.0 and g[2] == 0.0]
    ok14 = passed and len(zeros) >= 1
    results["T14"] = {"desc": "超时 0.5s 无指令 → 周期零速", "ok": ok14,
                      "detail": f"透传{'✓' if passed else '✗'};静默后窗口零速 {len(zeros)} 条",
                      "received": [(round(v, 4), round(z, 4)) for _, v, z in got]}
    say(f"== T14: 超时零速(透传窗口{'OK' if passed else '丢失'};静默 1.5s 后窗口收 {len(got)} 条,零速 {len(zeros)} 条)")
    say(f"   判定: {'PASS' if ok14 else 'FAIL'}")

    # T16 退出行为(v2 危险场景):非零指令透传中(未超时)SIGINT,窗口应含 0.05 透传+0/0 退出零
    chk.send(0.05, 0.0)
    proc.send_signal(2)  # 趁 0.5s 超时未到就打断
    got = chk.collect(1.5)
    zero_on_exit = any(g[1] == 0.0 and g[2] == 0.0 for g in got)
    nonzero_seen = any(abs(g[1] - 0.05) < 1e-6 for g in got)
    ok16 = zero_on_exit and nonzero_seen
    results["T16"] = {"desc": "非零透传中 SIGINT → 退出零(不让非零成为遗言)", "ok": ok16,
                      "detail": f"窗口收 {len(got)} 条;透传 0.05 {'✓' if nonzero_seen else '✗'};退出零 {'✓' if zero_on_exit else '✗'}"}
    say(f"== T16: 非零透传中 SIGINT(窗口收 {len(got)} 条: {[(round(v,3), round(z,3)) for _, v, z in got]})")
    say(f"   判定: {'PASS' if ok16 else 'FAIL'}(需同时见 0.05 与 0/0)")

    proc.wait(timeout=5)
    chk.destroy_node()
    rclpy.shutdown()
    fails = [k for k, v in results.items() if not v["ok"]]
    say(f"\n汇总: {'全部 PASS' if not fails else 'FAIL: ' + str(fails)}")
    with open(os.path.join(outdir, "raw.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=1)
    log.close()
    print(f"完成: {outdir}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
