#!/usr/bin/env python3
"""架空找起转点:一档一档试,每档单独从中位爬上去、停住让你看轮子、再回中位。

为什么不是"一路慢慢爬":电调对爬升速率两头卡(2026-09-22 架空实测),
2 PWM/s 从 1550 爬到 1650 全程不转,20 PWM/s 立刻转。所以"爬得慢一点好看清"
这条路走不通 —— 爬慢了它根本不转,测出来的是速率问题不是起转点。

这里每一档都是:
    中位 --(20 PWM/s 爬)--> 候选值 X --(停住 --hold 秒,你看轮子)--> 回中位 --(歇 --rest 秒)
爬升速率默认是实测能起转的 20 PWM/s(`--rate` 可改,低于 10 会警告),变的只有终点 X。X 从低往高试,
**第一个让轮子转起来的 X 就是起转点**。看到转了按 Ctrl-C,屏幕上最后一个
"停在 X"就是答案(脚本会先回中位再退)。

落地跑的话编码器能看见,连续 3 帧在动会自动停并报数;架空编码器恒读 0,只能靠眼睛。

用法(车上,架空):
    python3 stair_scan.py                          # 1560 -> 1620,每档 +2,停 2s
    python3 stair_scan.py --start 1580 --step 1    # 缩小范围、逐个 PWM 看
"""
import argparse
import os
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.params import CarParams

PWM_MID = CarParams().pwm_mid
TURN_MID = CarParams().turn_mid
SAFE_TOP = 1620         # 和 steer_policy_node.PWM_HARD_MAX 同一条安全线
RATE = 20.0             # PWM/s,实测能起转的爬升速率,别改慢
TICK = 0.05
MOVING = 0.05
MOVING_HOLD = 3


def ramp(frm: float, to: float, rate: float = RATE, dt: float = TICK) -> list[float]:
    """从 frm 按 rate 爬到 to 的每一拍 PWM(含终点)。"""
    out, pwm = [], float(frm)
    while pwm < to:
        pwm = min(float(to), pwm + rate * dt)
        out.append(pwm)
    return out


def candidates(start: int, stop: int, step: int) -> list[int]:
    return list(range(start, stop + 1, step))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--start', type=int, default=1560)
    ap.add_argument('--stop', type=int, default=SAFE_TOP)
    ap.add_argument('--step', type=int, default=2)
    ap.add_argument('--hold', type=float, default=2.0, help='每档停住多久给你看')
    ap.add_argument('--rest', type=float, default=1.5, help='回中位后歇多久,让轮子停稳')
    ap.add_argument('--rate', type=float, default=RATE,
                    help=f'从中位爬到每档的速率 PWM/s,默认 {RATE:.0f}。实测 2 不起转、20 起转,'
                         f'10 以下没验证过 —— 调慢了不转,要先怀疑速率')
    args = ap.parse_args()
    if args.rate <= 0:
        sys.exit('--rate 要大于 0')
    if args.rate < 10:
        print(f'注意:--rate {args.rate} 低于 10 PWM/s,没验证过能起转。'
              f'这一轮不转的话,不能说明起转点更高。\n', flush=True)
    if args.stop > SAFE_TOP:
        sys.exit(f'--stop 不能超过安全线 {SAFE_TOP}')
    if not PWM_MID < args.start <= args.stop:
        sys.exit(f'要满足 {PWM_MID} < --start <= --stop')

    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    class Scan(Node):
        def __init__(self):
            super().__init__('stair_scan')
            self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
            self.create_subscription(Odometry, '/encoder', self.on_enc, 20)
            self.speed, self.hits, self.have_enc = 0.0, 0, False

        def on_enc(self, m):
            self.have_enc = True
            self.speed = m.twist.twist.linear.x
            self.hits = self.hits + 1 if abs(self.speed) > MOVING else 0

        def send(self, pwm):
            tw = Twist()
            tw.linear.x, tw.angular.z = float(pwm), float(TURN_MID)
            self.pub.publish(tw)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Scan()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def neutral_exit(*_):
        for _ in range(5):
            node.send(PWM_MID)
            time.sleep(0.05)
        print('\n已回中位', flush=True)
        os._exit(0)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, neutral_exit)

    def hold(pwm, seconds):
        end = time.time() + seconds
        while time.time() < end:
            node.send(pwm)
            time.sleep(TICK)

    print('发中位,等 /encoder(最多 45s)…', flush=True)
    t0 = time.time()
    while time.time() - t0 < 45 and not node.have_enc:
        hold(PWM_MID, 0.5)
    hold(PWM_MID, 2.0)
    print(f'每档:{PWM_MID} 按 {args.rate:g} PWM/s 爬到 X,停 {args.hold}s,回中位歇 {args.rest}s。'
          f'\n轮子一转就 Ctrl-C,最后一行的 X 就是起转点。\n', flush=True)

    for x in candidates(args.start, args.stop, args.step):
        for pwm in ramp(PWM_MID, x, args.rate):
            node.send(pwm)
            time.sleep(TICK)
        print(f'>>> 停在 {x}   看轮子', end='', flush=True)
        node.hits = 0
        end = time.time() + args.hold
        while time.time() < end:
            node.send(x)
            time.sleep(TICK)
            if node.hits >= MOVING_HOLD:
                print(f'   编码器看到在动({node.speed * 1000:.0f} mm/s):起转点 {x}', flush=True)
                neutral_exit()
        print('   (编码器没动)', flush=True)
        hold(PWM_MID, args.rest)

    print(f'\n试到 {args.stop} 都没见转。', flush=True)
    neutral_exit()


if __name__ == '__main__':
    main()
