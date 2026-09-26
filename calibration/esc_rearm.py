#!/usr/bin/env python3
"""电调不出力时的重新解锁流程:全程发中位 -> 你断电再上电 -> 直接爬油门看编码器。

## 什么时候用

现象是**转向正常、油门完全不出力**:舵机能打(说明电调的 BEC 有电),但从中位
一路爬到 1620 电机一点反应都没有,编码器全 0。这不是电量不足 —— 亏电的表现是
"要更高的 PWM 才动",不是"完全不动"(见 memory esc-deadband-measured)。
也不是突变保护 —— 那条只管一步到位的跳变,缓慢爬升不触发(见
esc-rejects-throttle-jumps)。剩下的就是**电调没解锁**:上电那一刻它没看到干净的
中位信号,就一直锁着不输出。

## 为什么要脚本来做

电调解锁要求上电瞬间油门就在中位。手动断电再上电时如果没人发指令,
`/teleop_cmd_vel` 上一个包都没有(实测空闲时发布者为 0),电调看到的是"无信号",
解锁不了。所以这里在断电前就开始发中位,**断电、上电、解锁全程中位流不断**。

用法(车上):
    python3 esc_rearm.py                        # 断电重解锁 + 爬到 1620 验证
    python3 esc_rearm.py --no-cycle             # 刚重启过,不用再断电,直接扫
    python3 esc_rearm.py --no-cycle --top 1650  # 扫到 1650,查"到底还动不动"

按提示在倒计时里把动力电池开关关掉再打开,听到自检音之后脚本自动爬油门验证。
车最好架空,或者你按得住 —— 解锁成功它就会走。

**--top 超过 1620 只用来做诊断**。1620 是 `steer_policy_node.PWM_HARD_MAX`,
是 2026-09-21 架空试跑差点冲出去之后定下的安全线,不是标定值。这里允许你手动
扫过去,是因为"完全不出力"时需要知道再往上有没有反应;**扫出来的数不要回填到
策略的 --pwm 里**。爬升一见动就停,不会真的在 1650 上跑。
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
MOVING = 0.05
MOVING_HOLD = 3         # 连续这么多档超阈值才算真动(躲编码器毛刺)
SAFE_TOP = 1620        # 和 steer_policy_node.PWM_HARD_MAX 同一条安全线
NEUTRAL_BEFORE = 12.0   # 断电窗口:这段时间里你去关电、上电
NEUTRAL_AFTER = 5.0     # 上电后再喂一段干净中位,让电调走完自检
DISCOVERY_WAIT = 45.0   # 等第一帧 /encoder 的上限,见下面的注释


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--top', type=int, default=SAFE_TOP, help='爬到这里为止')
    ap.add_argument('--no-cycle', action='store_true',
                    help='跳过断电窗口(刚重启过,电调已经重新上过电)')
    ap.add_argument('--hold', type=float, default=0.1,
                    help='每档保持多久。架空时编码器读 0,靠眼睛看轮子 —— '
                         '把这个拉到 1.0,轮子一转你按 Ctrl-C,最后一行就是起转 PWM')
    args = ap.parse_args()
    if args.top > SAFE_TOP:
        print(f'注意:--top {args.top} 超过安全线 {SAFE_TOP}。这只是诊断,'
              f'一见动就停,别把这个数回填到策略里。\n', flush=True)

    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    class Rearm(Node):
        def __init__(self):
            super().__init__('esc_rearm')
            self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
            self.create_subscription(Odometry, '/encoder', self.on_enc, 20)
            self.speed = 0.0
            self.have_enc = False

        def on_enc(self, m):
            self.have_enc = True
            self.speed = m.twist.twist.linear.x

        def send(self, pwm):
            tw = Twist()
            tw.linear.x, tw.angular.z = float(pwm), float(TURN_MID)
            self.pub.publish(tw)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Rearm()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def neutral_exit(*_):
        for _ in range(5):
            node.send(PWM_MID)
            time.sleep(0.05)
        print('已回中位', flush=True)
        sys.stdout.flush()
        os._exit(0)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, neutral_exit)

    def hold(pwm, seconds):
        end = time.time() + seconds
        while time.time() < end:
            node.send(pwm)
            time.sleep(0.05)

    # DDS 发现可能很慢:2026-09-22 换到实验室 10 段网络之后,车上有 4 个网络接口
    # (10.x / 192.168.5.x / docker0 / IPv6),`ros2 node list` 要二十几秒才列全。
    # 这里等到 DISCOVERY_WAIT 为止,别用固定的两三秒 —— 那会把"发现慢"误报成
    # "底盘没跑"(当天就误报过一次)。
    print(f'开始发中位,等 /encoder(最多 {DISCOVERY_WAIT:.0f}s)…', flush=True)
    t0 = time.time()
    while time.time() - t0 < DISCOVERY_WAIT and not node.have_enc:
        hold(PWM_MID, 0.5)
    if not node.have_enc:
        print(f'等了 {DISCOVERY_WAIT:.0f}s 还是收不到 /encoder。'
              f'先确认:ros2 topic hz /encoder', flush=True)
        neutral_exit()
    print(f'  收到 /encoder,用了 {time.time() - t0:.1f}s', flush=True)
    hold(PWM_MID, 2.0)

    if not args.no_cycle:
        print(f'\n>>> 现在把动力电池开关【关掉】,数三秒再【打开】。'
              f'中位信号我一直发着。', flush=True)
        t0 = time.time()
        while time.time() - t0 < NEUTRAL_BEFORE:
            left = NEUTRAL_BEFORE - (time.time() - t0)
            print(f'\r    还剩 {left:4.1f}s ', end='', flush=True)
            hold(PWM_MID, 0.2)
        print('\n上电后再喂一段干净中位…', flush=True)
        hold(PWM_MID, NEUTRAL_AFTER)

    print(f'\n爬油门({PWM_MID} -> {args.top},每 0.1s +2):', flush=True)
    print(f'{"PWM":>5} {"mm/s":>8}', flush=True)
    pwm = PWM_MID
    moved = None
    hits = 0
    while pwm < args.top:
        pwm = min(args.top, pwm + 2)
        hold(pwm, args.hold)
        # 每档都打。架空时编码器恒为 0,这一列没用,但 PWM 那一列是眼睛看轮子时
        # 唯一的对照 —— 轮子一转按 Ctrl-C,最后一行就是起转值。
        print(f'{pwm:>5} {node.speed*1000:>8.0f}', flush=True)
        # 连续 MOVING_HOLD 档都超阈值才算动。静止时编码器有 ±10mm/s 的噪声,
        # 偶尔蹦一个上百的毛刺 —— 只看一帧的话 2026-09-22 实测会误判成 1542 起转,
        # 而下一次同样条件测出来是 1592。
        hits = hits + 1 if abs(node.speed) > MOVING else 0
        if hits >= MOVING_HOLD:
            moved = pwm - (MOVING_HOLD - 1) * 2      # 第一次超阈值那一档
            print(f'      ^^^ 连续 {MOVING_HOLD} 档在动,起转 {moved}', flush=True)
            break
    hold(PWM_MID, 0.5)

    if moved:
        print(f'\n解锁成功,{moved} 起转。接着可以跑 ramp_curve.py / coast_test.py',
              flush=True)
    else:
        print(f'\n爬到 {args.top} 还是没动。编码器是落地从动轮,架空读 0 —— '
              f'先确认车是落地的;确认了还不动,就是电调或电机那一路的硬件问题,'
              f'不要靠抬 PWM 硬怼。', flush=True)
    neutral_exit()


if __name__ == '__main__':
    main()
