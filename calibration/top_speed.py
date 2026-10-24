#!/usr/bin/env python3
"""测 1620 以上单个 PWM 的稳态车速(Roaming / MAD MAX 档位和策略 PWM 上限用)。

ramp_curve 测不了这段:车速 2.0 m/s 封顶、中止只松油滑行(2.5 m/s 要滑 4 m+)、从中位慢爬太费距离。
这里每次只测一个 PWM:跳到起步点(不动就阶梯加,同 SpeedLoop)-> 按 --slew 缓升到 --pwm -> 保持 --hold 秒,
取后一半的平均车速 -> 刹车(brake_pwm,车速低于 brake_release_speed 回中位)。雷达净空不够刹停或超速也立刻刹车。

    python3 -m calibration.top_speed --pwm 1630 --csv /tmp/top_1630.csv

每次跑完把车推回起点再测下一个 PWM。直道要 12 m 以上;前方要有雷达看得到的墙或障碍,截停才有依据。
"""
import argparse
import os
import signal
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.params import CarParams

P = CarParams()
LAUNCH_FROM, LAUNCH_MAX, LAUNCH_STEP = 1593, 1610, 4    # 同 road_car_node / SpeedLoopConfig 的阶梯起步
LAUNCH_HOLD, LAUNCH_GAP = 0.8, 0.3
MOVING = 0.05           # m/s
MARGIN = 0.5            # 刹停后保险杠前还要剩的净空 m
BRAKE_MAX = 3.0         # 刹车最多发这么久 s
PWM_LIMIT = 1700        # 这个脚本允许的最高 PWM(老网页 MAD MAX 的值)


def stop_dist(v):
    """以车速 v 刹停要的保险杠前净空:延迟 + 刹车 + 余量。"""
    return MARGIN + P.guard_latency * v + v * v / (2 * P.guard_brake_decel)


class TopRun:
    """纯逻辑:每拍喂 (now, 车速, 前方净空),返回要发的 PWM;done 后不再调用。"""

    def __init__(self, target, slew=40.0, hold=1.2, abort_speed=3.0):
        self.target, self.slew, self.hold, self.abort_speed = target, slew, hold, abort_speed
        self.phase, self.t0, self.pwm = 'launch', None, float(LAUNCH_FROM)
        self.gap_left = 0.0
        self.samples, self.why, self.done = [], None, False
        self.last = None

    def _enter(self, phase, now):
        self.phase, self.t0 = phase, now

    def step(self, now, v, gap):
        dt = 0.0 if self.last is None else now - self.last
        self.last = now
        if self.t0 is None:
            self.t0 = now
        if self.phase != 'brake':
            if gap is None or gap < stop_dist(max(v, 0.0)):
                self.why = '雷达无数据' if gap is None else f'前方 {gap:.2f} m < 刹停要 {stop_dist(v):.2f} m'
                self._enter('brake', now)
            elif v > self.abort_speed:
                self.why = f'车速 {v:.2f} > {self.abort_speed} m/s'
                self._enter('brake', now)
        if self.phase == 'launch':
            if v > MOVING:
                self._enter('ramp', now)
            elif self.gap_left > 0:
                self.gap_left -= dt
                if self.gap_left <= 0:
                    self.pwm = min(LAUNCH_MAX, self.pwm + LAUNCH_STEP)
                    self.t0 = now
                return P.pwm_mid
            elif now - self.t0 > LAUNCH_HOLD:
                if self.pwm >= LAUNCH_MAX:
                    self.why = f'跳到 {LAUNCH_MAX} 还不动(电调开了吗?中位锁对了吗?)'
                    self._enter('brake', now)
                else:
                    self.gap_left = LAUNCH_GAP
                    return P.pwm_mid
            if self.phase == 'launch':
                return round(self.pwm)
        if self.phase == 'ramp':
            self.pwm = min(float(self.target), self.pwm + self.slew * dt)
            if self.pwm >= self.target:
                self._enter('hold', now)
            return round(self.pwm)
        if self.phase == 'hold':
            if now - self.t0 >= self.hold / 2:
                self.samples.append(v)
            if now - self.t0 >= self.hold:
                self._enter('brake', now)
            else:
                return self.target
        # brake
        if v > P.brake_release_speed and now - self.t0 < BRAKE_MAX:
            return P.brake_pwm
        self.done = True
        return P.pwm_mid

    def result(self):
        return statistics.mean(self.samples) if self.samples else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pwm', type=int, required=True, help='要测的 PWM(1620~1700)')
    ap.add_argument('--slew', type=float, default=40.0, help='起步后每秒最多加多少 PWM(同网页手动)')
    ap.add_argument('--hold', type=float, default=1.2, help='到目标 PWM 后保持多久 s,取后一半平均')
    ap.add_argument('--abort-speed', type=float, default=3.0)
    ap.add_argument('--csv', default=None, help='每拍写一行 t,pwm,v,gap')
    a = ap.parse_args()
    if not LAUNCH_MAX < a.pwm <= PWM_LIMIT:
        sys.exit(f'--pwm 要在 {LAUNCH_MAX + 1}~{PWM_LIMIT}')

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan
    from vehicle.scan_guard import ScanGuard

    class Top(Node):
        def __init__(self):
            super().__init__('top_speed')
            self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
            self.guard = ScanGuard()
            self.v, self.enc_t, self.dist = 0.0, None, 0.0
            self.create_subscription(Odometry, '/encoder', self.on_enc, 20)
            self.create_subscription(LaserScan, '/scan', lambda m: self.guard.update(m), 1)

        def on_enc(self, m):
            now = time.time()
            v = m.twist.twist.linear.x
            if self.enc_t is not None:
                self.dist += abs(v) * (now - self.enc_t)
            self.v, self.enc_t = v, now

        def send(self, pwm):
            tw = Twist()
            tw.linear.x, tw.angular.z = float(pwm), float(P.turn_mid)
            self.pub.publish(tw)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Top()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def neutral_exit(*_):
        for _ in range(10):
            node.send(P.pwm_mid)
            time.sleep(0.05)
        print('已回中位', flush=True)
        os._exit(0)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, neutral_exit)

    t0 = time.time()
    while time.time() - t0 < 10 and (node.enc_t is None or node.guard.gap() is None):
        node.send(P.pwm_mid)
        time.sleep(0.1)
    if node.enc_t is None or node.guard.gap() is None:
        print('等不到 /encoder 或 /scan')
        neutral_exit()
    print(f'前方净空 {node.guard.gap():.2f} m;目标 {a.pwm},缓升 {a.slew:.0f} PWM/s,保持 {a.hold} s', flush=True)
    csv = open(a.csv, 'w', buffering=1) if a.csv else None
    if csv:
        csv.write('t,pwm,v,gap\n')
    run = TopRun(a.pwm, a.slew, a.hold, a.abort_speed)
    while not run.done:
        now = time.time()
        v = node.v if node.enc_t and now - node.enc_t < 0.3 else 0.0
        if run.phase != 'brake' and now - node.enc_t > 0.3:
            run.why = '编码器过期'
            run._enter('brake', now)
        gap = node.guard.gap()
        pwm = run.step(now, v, gap)
        node.send(pwm)
        if csv:
            csv.write(f'{now:.3f},{pwm},{v:.3f},{-1 if gap is None else gap:.2f}\n')
        time.sleep(0.05)
    v = run.result()
    print(f'\nPWM {a.pwm}: 稳态 {"没测到(没保持够)" if v is None else f"{v:.2f} m/s"},共跑 {node.dist:.1f} m'
          + (f';提前刹车:{run.why}' if run.why else ''), flush=True)
    neutral_exit()


if __name__ == '__main__':
    main()
