#!/usr/bin/env python3
"""架空空转放电:测亏电时的闭环用。**车必须架空**(轮子离地)。

    python3 drain.py --minutes 10                 # 默认 1615
    python3 drain.py --minutes 10 --pwm 1610

按 20 PWM/s 从中位爬到 --pwm,保持 --minutes 分钟,结束回中位。
编码器是落地从动轮,架空恒读 0 —— **读到车速就说明车在地上**,立刻回中位退出。
"""
import argparse
import os
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PWM_CAP = 1620          # 和速度环同一个硬上限
UP_RATE = 20.0          # PWM/s,实测电调接受
ON_GROUND = 0.1         # m/s,编码器读到这么快就是落地了
GROUND_HITS = 4         # 连续这么多拍(0.2s)才算:静止时编码器偶尔蹦 ±0.2 的单帧
DT = 0.05


def next_pwm(current, target, dt=DT):
    """按 UP_RATE 往 target 爬一步。"""
    return min(float(target), current + UP_RATE * dt)


def ground_hits(hits, speed):
    """连续超阈值的拍数。返回值 >= GROUND_HITS 就是落地了。"""
    return hits + 1 if abs(speed) > ON_GROUND else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--minutes', type=float, required=True)
    ap.add_argument('--pwm', type=int, default=1615)
    args = ap.parse_args()
    if not 1590 <= args.pwm <= PWM_CAP:
        sys.exit(f'--pwm 要在 [1590, {PWM_CAP}]')
    if not 0 < args.minutes <= 60:
        sys.exit('--minutes 要在 (0, 60]')

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from ramp_curve import PWM_MID, Ramp

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Ramp()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def neutral_exit(*_):
        for _ in range(5):
            node.send(PWM_MID)
            time.sleep(0.05)
        print('已回中位', flush=True)
        os._exit(0)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, neutral_exit)

    t0 = time.time()
    while time.time() - t0 < 3:
        node.send(PWM_MID)
        time.sleep(0.1)
    if not node.have_enc:
        print('等不到 /encoder —— 先跑 carready.py(落地检测靠它)')
        neutral_exit()

    print(f'爬到 {args.pwm},保持 {args.minutes} 分钟。Ctrl+C 随时停', flush=True)
    pwm, t0, last_min, hits = float(PWM_MID), time.time(), -1, 0
    while time.time() - t0 < args.minutes * 60:
        hits = ground_hits(hits, node.speed)
        if hits >= GROUND_HITS:
            print(f'编码器读到 {node.speed:.2f} m/s —— 车在地上,停', flush=True)
            neutral_exit()
        pwm = next_pwm(pwm, args.pwm)
        node.send(round(pwm))
        m = int((time.time() - t0) // 60)
        if m != last_min:
            print(f'  {m} 分钟', flush=True)
            last_min = m
        time.sleep(DT)
    print('时间到', flush=True)
    neutral_exit()


if __name__ == '__main__':
    main()
