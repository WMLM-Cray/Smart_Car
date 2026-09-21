#!/usr/bin/env python3
"""PWM 扫描:从中位往上逐档给油门,用编码器记录每档速度,找最低能稳定跑的 PWM。

用法(车上):python3 pwm_sweep.py [起始 终止 步长 每档秒数] [--pause 秒] [--max-dist 米]
默认 1555 1620 5 2,每档之间回中位停 2.5 秒,累计跑出 4 米就中止。

加 --duty-at <PWM> 就改成扫占空比:起始/终止/步长按百分比算,在该 PWM 和中位之间
按比例通断。电调最低就是 470mm/s,比这慢只能靠占空比,这个模式就是用来测那条曲线的。

编码器是落地从动轮,架空测不到速度 —— 必须落地跑。每档之间会回中位让车停下,
方便你把车拖回起点;--max-dist 是硬保险:走够距离就回中位退出,别指望场地够长。
"""
import os, signal, statistics, sys, threading, time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from scan_guard import ScanGuard

SPEED_MID, TURN_MID = 1550, 84
ABORT_SPEED = 1.5            # m/s,超过就中止
ABORT_GAP = 0.6              # 保险杠前净空小于这个就中止(米)
DUTY_TICKS = 6               # 占空比周期 = 6 x 50ms = 0.3s,和 server.py 保持一致


class Sweep(Node):
    def __init__(self):
        super().__init__('pwm_sweep')
        self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
        self.create_subscription(Odometry, '/encoder', self.on_enc, 10)
        self.scan = ScanGuard()
        self.create_subscription(LaserScan, '/scan', lambda m: self.scan.update(m), 1)
        self.samples = []
        self.distance = 0.0
        self.last_stamp = None
        self.lock = threading.Lock()

    def on_enc(self, m):
        now = time.time()
        with self.lock:
            v = m.twist.twist.linear.x
            if self.last_stamp is not None:                 # 速度对时间积分 = 跑了多远
                self.distance += abs(v) * (now - self.last_stamp)
            self.last_stamp = now
            self.samples.append(v)

    def send(self, pwm):
        tw = Twist(); tw.linear.x = float(pwm); tw.angular.z = float(TURN_MID)
        self.pub.publish(tw)

    def take(self):
        with self.lock:
            s, self.samples = self.samples, []
        return s


def main():
    def opt(argv, name, default):
        if name not in argv:
            return default, argv
        i = argv.index(name)
        return float(argv[i + 1]), argv[:i] + argv[i + 2:]

    argv = sys.argv[1:]
    no_guard = '--no-guard' in argv
    argv = [a for a in argv if a != '--no-guard']       # 场地窄/雷达没起来时用,自己看着车
    pause, argv = opt(argv, '--pause', 2.5)
    max_dist, argv = opt(argv, '--max-dist', 4.0)
    duty_at, argv = opt(argv, '--duty-at', 0.0)
    start, stop, step, hold = (list(map(float, argv)) + [1555, 1620, 5, 2][len(argv):])[:4]
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

    class Blocked(Exception):
        pass

    def hold_pwm(pwm, sec, watch=False, duty=None):
        """duty 给了就按占空比在 duty_at 和中位之间通断,否则一直发 pwm。"""
        end = time.time() + sec
        tick = 0
        while time.time() < end:
            if watch and not no_guard:                 # 前进时盯着雷达,不够净空就停
                gap = node.scan.gap()
                if gap is None or gap < ABORT_GAP:
                    raise Blocked('雷达无数据' if gap is None else f'前方只剩 {gap:.2f}m')
            if duty is None:
                node.send(pwm)
            else:
                node.send(duty_at if tick % DUTY_TICKS < round(duty / 100 * DUTY_TICKS) else SPEED_MID)
            tick += 1
            time.sleep(0.05)

    try:
        print('中位 2s(让电调解锁)...', flush=True)
        hold_pwm(SPEED_MID, 2)
        if no_guard:
            print('⚠ 已关闭雷达防撞 —— 自己盯着车,随时 Ctrl+C', flush=True)
        else:
            # 等第一帧雷达:ROS 发现发布者要 1-3s,不等就会把"还没收到"误判成"雷达掉线"
            wait_until = time.time() + 10
            while node.scan.gap() is None and time.time() < wait_until:
                node.send(SPEED_MID); time.sleep(0.1)
            if node.scan.gap() is None:
                print('等了 10s 还是没有 /scan —— 雷达节点没起来?(car.sh)', flush=True)
                return
        node.take()
        if duty_at:
            print(f'占空比模式:在 {duty_at:.0f} 和中位之间通断,周期 {DUTY_TICKS * 0.05:.2f}s', flush=True)
        print(f'每档 {hold:.1f}s,档间回中位 {pause:.1f}s,累计 {max_dist:.1f}m 中止', flush=True)
        print(f'{"占空%" if duty_at else "PWM":>5} {"平均mm/s":>9} {"最小":>6} {"最大":>6} {"波动(std)":>9} {"累计m":>6}', flush=True)
        pwm = start
        # step 给负数就往下扫:配 --pause 0 用,先用高 PWM 把车带动再逐档降,
        # 测"维持速度"而不是"起步速度"。电调的启动死区比维持死区高,
        # 每档都从静止起步的话,低档永远是 0,看不出它其实能维持慢速。
        while (pwm <= stop if step > 0 else pwm >= stop):
            duty = pwm if duty_at else None
            try:
                hold_pwm(pwm, hold / 2, watch=True, duty=duty)   # 前一半等速度稳定
                node.take()
                hold_pwm(pwm, hold / 2, watch=True, duty=duty)   # 后一半采样
            except Blocked as e:
                print(f'{pwm:>5.0f}  前方挡住了({e}),中止', flush=True); break
            s = [v * 1000 for v in node.take()]
            if s:
                print(f'{pwm:>5.0f} {statistics.mean(s):>9.0f} {min(s):>6.0f} {max(s):>6.0f} '
                      f'{statistics.pstdev(s):>9.0f} {node.distance:>6.1f}', flush=True)
                if max(abs(v) for v in s) > ABORT_SPEED * 1000:
                    print('速度超限,中止'); break
            else:
                print(f'{pwm:>5.0f}  没有编码器数据(car.sh 在跑吗?)', flush=True); break
            hold_pwm(SPEED_MID, pause)                   # 回中位停一下:好把车拖回起点,也让电调散热
            node.take()
            if node.distance > max_dist:
                print(f'已跑出 {node.distance:.1f}m,到设定上限,中止'); break
            pwm += step
    finally:
        neutral_exit()


if __name__ == '__main__':
    main()
