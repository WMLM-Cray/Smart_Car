#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无动力联调:虚拟串口隔离验证 racecar_driver 的 话题→PWM 转换(2026-09-16 控制链审查轮·任务三)

原理:
  1. os.openpty() 造一对虚拟串口;从端 symlink 到 /tmp/fakecar;
  2. 以参数 serial_port:=/tmp/fakecar 启动真实 driver 二进制(racecar_driver_node / _one)
     —— 真实 /dev/car 全程不被打开;
  3. 向 /car_cmd_vel、/teleop_cmd_vel 发布测试 Twist;
  4. 从 pty 主端读取 driver 实际写出的原始字节(hex 记录),同时收集 driver stdout 的
     "the send buff is" hex 打印(静态库 print_send_buff),双路证据;
  5. 覆盖矩阵:启动无指令/零指令/正反向/限幅端点/越界回绕/断流/teleop 直通/退出/重启。

安全边界(缺一不跑):
  * 电调开关物理断开(脚本开头交互确认);
  * 只通过 /tmp/fakecar,绝不 open /dev/car;
  * T1 安全阀:发零指令后 1.5s 内 /tmp/fakecar 收不到字节 → 立即 SIGINT driver 并中止
    (防参数未生效导致 driver 连到真实串口);
  * 本脚本不碰厂家工作区、不写入任何系统配置。

用法(车机上,source 过 ROS 后):
  python3 dryrun_driver_test.py [输出目录]
默认输出 ~/dryrun_时间戳/:report.txt(人读)+ raw.json(字节证据)+ driver_stdout.log
"""

import json
import os
import select
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist

FAKE_DEV = "/tmp/fakecar"
BAUD_DEFAULT = 38400

# 期望值(车上 1540 版公式;若驱动为 1500 版,报告会并列给出对照)
MOTOR_ZERO = 1540   # 0.0*100+1540
MOTOR_025 = 1565    # 0.25*100+1540
MOTOR_100 = 1640    # 1.0*100+1540
MOTOR_NEG025 = 1290
MOTOR_WRAP = 65076  # -20*100+1540=-460 -> uint16 回绕
SERVO_ZERO = 1500
SERVO_LEFT_FULL = 1000   # z=+0.385 -> 1500-500.5 (取 z=5/13 恰 500)
SERVO_RIGHT_FULL = 2000  # z=-5/13
TELEOP_MID_MOTOR = 1540  # 直通:linear.x 原样
TELEOP_TURN84_SERVO = 1566  # 2500-84*2000/180=1566.67 -> uint16


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b) if b else "(无字节)"


def u16(v: float) -> int:
    """复现 driver 的 static_cast<uint16_t>"""
    return int(v) & 0xFFFF  # C++ 截断语义:负数按补码回绕


class DriverProc:
    """driver 子进程 + stdout 收集线程 + pty 字节读取"""

    def __init__(self, exe: str, outdir: str, tag: str):
        self.exe = exe
        self.tag = tag
        self.stdout_lines = []
        self.proc = None
        self.logf = open(os.path.join(outdir, f"driver_stdout_{tag}.log"), "w", encoding="utf-8")

    def start(self):
        cmd = ["ros2", "run", "racecar_driver", self.exe, "--ros-args",
               "-p", f"serial_port:={FAKE_DEV}",
               "-p", f"baud_rate:={BAUD_DEFAULT}"]
        print(f"[{self.tag}] 启动: {' '.join(cmd)}")
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, bufsize=1)
        t = threading.Thread(target=self._pump, daemon=True)
        t.start()
        time.sleep(2.5)  # 等 ROS 发现+串口打开

    def _pump(self):
        for line in self.proc.stdout:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            rec = f"{ts} {line.rstrip()}"
            self.stdout_lines.append(rec)
            self.logf.write(rec + "\n")
            self.logf.flush()

    def send_lines_with(self, needle: str):
        return [l for l in self.stdout_lines if needle in l]

    def sigint_and_wait(self, timeout=8):
        if self.proc and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        return self.proc.returncode if self.proc else None

    def close(self):
        self.sigint_and_wait()
        self.logf.close()


class PtyHub:
    def __init__(self):
        self.master_fd, self.slave_fd = os.openpty()
        self.slave_name = os.ttyname(self.slave_fd)
        os.symlink(self.slave_name, FAKE_DEV)
        os.set_blocking(self.master_fd, False)

    def drain(self, wait_s=1.5, quiet_s=0.4):
        """收集 wait_s 窗口内的字节;若 quiet_s 内无新字节可提前返回"""
        buf = b""
        deadline = time.time() + wait_s
        last = time.time()
        while time.time() < deadline:
            r, _, _ = select.select([self.master_fd], [], [], 0.1)
            if r:
                try:
                    chunk = os.read(self.master_fd, 4096)
                    if chunk:
                        buf += chunk
                        last = time.time()
                except BlockingIOError:
                    pass
            elif buf and time.time() - last > quiet_s:
                break
        return buf

    def cleanup(self):
        try:
            os.unlink(FAKE_DEV)
        except FileNotFoundError:
            pass
        for fd in (self.master_fd, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass

    def drop_master(self):
        """T10 用:只关主端,模拟下位机 USB 拔出(从端 driver 仍持有,write 将得 EIO)"""
        try:
            os.close(self.master_fd)
            self.master_fd = -1
        except OSError:
            pass


def pub_twist(pub, lx, az):
    t = Twist()
    t.linear.x = float(lx)
    t.angular.z = float(az)
    pub.publish(t)
    time.sleep(0.15)   # 首条可能因 QoS 匹配未建而丢,补发一条
    pub.publish(t)


def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        "~/dryrun_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(outdir, exist_ok=True)
    report = open(os.path.join(outdir, "report.txt"), "w", encoding="utf-8")
    raw = {"meta": {"fake_dev": FAKE_DEV, "baud": BAUD_DEFAULT,
                    "utc_assumption": "车上公式=×100+1540(2026-09-16 src)"},
           "cases": []}

    def say(msg=""):
        print(msg)
        report.write(msg + "\n")
        report.flush()

    say(f"# 无动力联调报告 {datetime.now().isoformat()}")
    say(f"输出目录: {outdir}")

    # ---- 前置自检 ----
    r = subprocess.run(["pgrep", "-af", "racecar_driver_node"],
                       capture_output=True, text=True)
    if r.stdout.strip():
        say("!! 已有 racecar_driver 进程在跑,为避免双写串口/话题干扰,中止:")
        say(r.stdout)
        sys.exit(2)
    say("[自检] 无 racecar_driver 残留进程 ✓")
    if os.path.exists(FAKE_DEV) and not os.path.islink(FAKE_DEV):
        say(f"!! {FAKE_DEV} 已存在且不是符号链接,拒绝覆盖,中止")
        sys.exit(2)
    say(f"[自检] /dev/car 存在性(仅记录,不打开): "
        f"{os.path.exists('/dev/car')}")
    ans = input("[安全确认] 电调开关已物理断开?(回车继续 / Ctrl+C 取消):")
    say(f"[安全确认] 用户回车继续(ans={ans!r})")

    hub = PtyHub()
    say(f"[隔离] pty 从端 {hub.slave_name} -> {FAKE_DEV}")

    rclpy.init()
    node = rclpy.create_node("dryrun_tester")
    pub_car = node.create_publisher(Twist, "/car_cmd_vel", 5)
    pub_teleop = node.create_publisher(Twist, "/teleop_cmd_vel", 5)
    time.sleep(1.0)  # publisher 发现

    def case(tag, desc, expect, action, expect_bytes=True, collect_s=1.5):
        """执行一个工况并记录。expect=预期 PWM 语义描述"""
        n_before = len(driver.send_lines_with("send buff"))
        action()
        b = hub.drain(collect_s)
        n_buff = len(driver.send_lines_with("send buff")) - n_before
        verdict = "PASS" if ((len(b) > 0) == expect_bytes) else "FAIL"
        say(f"\n== {tag}: {desc}")
        say(f"   输入后 {collect_s}s 内 pty 字节({len(b)}B): {hexs(b)}")
        say(f"   driver stdout 新增 send-buff 打印: {n_buff} 条")
        say(f"   预期: {expect}")
        say(f"   判定: {verdict}(字节{'出现' if expect_bytes else '应无'})")
        raw["cases"].append({"tag": tag, "desc": desc, "expect": expect,
                             "bytes_hex": b.hex(), "n_bytes": len(b),
                             "send_buff_lines": n_buff, "verdict": verdict})
        return b

    results = {}

    # ================= 第一轮:racecar_driver_node(/car_cmd_vel)=================
    driver = DriverProc("racecar_driver_node", outdir, "node")
    driver.start()

    case("T0", "启动后 3s 无指令(应不发任何字节,无初始化报文)",
         "无字节", lambda: time.sleep(3), expect_bytes=False, collect_s=3.0)

    b_t1 = case("T1", "零指令 v=0 z=0", f"motor={MOTOR_ZERO} servo={SERVO_ZERO}",
                lambda: pub_twist(pub_car, 0, 0))
    if not b_t1:
        say("!! 安全阀触发:发指令后 /tmp/fakecar 无字节——driver 可能没有连到虚拟串口!")
        say("!! 立即 SIGINT 并中止(未向任何真实串口发送内容)")
        driver.sigint_and_wait()
        hub.cleanup()
        sys.exit(3)

    case("T2", "正向小指令 v=0.25(=nav2 desired_linear_vel)",
         f"motor={MOTOR_025}", lambda: pub_twist(pub_car, 0.25, 0))
    case("T2b", "正向 v=1.0", f"motor={MOTOR_100}",
         lambda: pub_twist(pub_car, 1.0, 0))
    case("T3", "反向小指令 v=-0.25", f"motor={MOTOR_NEG025}",
         lambda: pub_twist(pub_car, -0.25, 0))
    case("T4a", "满舵一端 z=+5/13", f"servo={SERVO_LEFT_FULL}",
         lambda: pub_twist(pub_car, 0, 5.0/13.0))
    case("T4b", "满舵另一端 z=-5/13", f"servo={SERVO_RIGHT_FULL}",
         lambda: pub_twist(pub_car, 0, -5.0/13.0))
    case("T5", "越界回绕 v=-20(=uint16 回绕演示,只进虚拟口)",
         f"motor=uint16(-460)={MOTOR_WRAP}", lambda: pub_twist(pub_car, -20, 0))
    case("T8", "teleop 直通零位 x=1540 z=90(=install 版 teleop 停止键语义)",
         f"motor={TELEOP_MID_MOTOR} servo={SERVO_ZERO}",
         lambda: pub_twist(pub_teleop, 1540, 90))

    # T6 断流:先发一条,再静默 20s
    say("\n== T6: 断流测试(发一条后静默 20s,应无自动停车字节——无 watchdog)")
    pub_twist(pub_car, 0.0, 0)
    hub.drain(1.0)
    b = hub.drain(20.0)
    verdict = "PASS" if len(b) == 0 else "FAIL(出现自动发送!)"
    say(f"   静默期字节: {hexs(b)}")
    say(f"   预期: 无字节(下位机将保持最后指令)  判定: {verdict}")
    raw["cases"].append({"tag": "T6", "bytes_hex": b.hex(), "verdict": verdict})

    # T7 退出:SIGINT 后观察是否发停车字节
    say("\n== T7: 退出测试(SIGINT driver,退出过程不应发送任何字节)")
    rc = driver.sigint_and_wait()
    time.sleep(1.0)
    b = hub.drain(2.0)
    verdict = "PASS" if len(b) == 0 else "FAIL(退出时发送了字节!)"
    say(f"   driver 退出码: {rc};退出后字节: {hexs(b)};判定: {verdict}")
    raw["cases"].append({"tag": "T7", "returncode": rc,
                         "bytes_hex": b.hex(), "verdict": verdict})

    # ================= 第二轮:racecar_driver_node_one(/cmd_vel)=================
    say("\n========== 第二轮:racecar_driver_node_one(桥接决策采纳的 /cmd_vel 版)==========")
    driver2 = DriverProc("racecar_driver_node_one", outdir, "node_one")
    driver = driver2
    driver.start()
    pub_cmd = node.create_publisher(Twist, "/cmd_vel", 5)
    time.sleep(1.0)

    case("T9a", "node_one 零指令 v=0 z=0(/cmd_vel)",
         f"motor={MOTOR_ZERO} servo={SERVO_ZERO}",
         lambda: pub_twist(pub_cmd, 0, 0))
    b_t9b = case("T9b", "node_one v=0.25(/cmd_vel,重启可重复性)",
                 f"motor={MOTOR_025}", lambda: pub_twist(pub_cmd, 0.25, 0))
    case("T9c", "node_one teleop 直通 x=1550 z=84(=src 版 teleop 停止语义,未编译也先验证通道)",
         f"motor=1550 servo={TELEOP_TURN84_SERVO}",
         lambda: pub_twist(pub_teleop, 1550, 84))
    rc2 = driver2.sigint_and_wait()
    say(f"\n[node_one] 退出码 {rc2}")

    # ================= 第三轮 T10:下位机失联模拟(仅虚拟链路)=================
    say("\n========== 第三轮 T10:下位机失联模拟(关闭 pty 主端 = 模拟 car 板 USB 拔出)==========")
    driver3 = DriverProc("racecar_driver_node", outdir, "node_drop")
    driver = driver3
    driver.start()
    case("T10a", "失联前基准:发 v=0.25 应有字节",
         "motor=1565(字节正常)", lambda: pub_twist(pub_car, 0.25, 0))
    say("   -- 关闭 pty 主端(模拟拔线),2s 后再发指令观察 driver 存活 --")
    hub.drop_master()
    time.sleep(2.0)
    alive_before = driver.proc.poll() is None
    say(f"   拔线后 driver 存活: {alive_before}")
    pub_twist(pub_car, 0.5, 0)
    time.sleep(2.0)
    alive_after = driver.proc.poll() is None
    rc3 = driver3.sigint_and_wait()
    say(f"   拔线后再发指令 driver 存活: {alive_after};SIGINT 退出码: {rc3}")
    err_lines = [l for l in driver3.stdout_lines[-15:] if "rror" in l or "serial" in l.lower()]
    say(f"   末尾 stdout 相关行: {err_lines[-3:] if err_lines else '(无 error/serial 字样)'}")
    verdict = "记录(行为观察,无 PASS/FAIL)"
    raw["cases"].append({"tag": "T10", "alive_before": alive_before,
                         "alive_after": alive_after, "returncode": rc3,
                         "verdict": verdict})
    say(f"   判定: {verdict}")

    # ---- 收尾 ----
    node.destroy_node()
    rclpy.shutdown()
    hub.cleanup()

    say("\n========== 汇总 ==========")
    fails = [c["tag"] for c in raw["cases"] if str(c.get("verdict", "")).startswith("FAIL")]
    say("结果: " + ("全部 PASS" if not fails else f"FAIL 工况: {fails}"))
    say("\n字节差分提示(帧格式逆向素材,人工分析用):")
    say("  T1 与 T2 的字节差异应恰为 motor 低字节 +0x19(1540->1565);")
    say("  T4a/T4b 与 T1 的差异应在 servo 字节;详见 raw.json 各工况 bytes_hex。")
    with open(os.path.join(outdir, "raw.json"), "w", encoding="utf-8") as f:
        json.dump(raw, f, ensure_ascii=False, indent=1)
    report.close()
    print(f"\n完成: {outdir}(report.txt / raw.json / driver_stdout_*.log)")
    print("拖回电脑归档: docs/validation/2026-09-16-control/")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
