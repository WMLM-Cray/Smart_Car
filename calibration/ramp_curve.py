#!/usr/bin/env python3
"""一次跑出三样东西:破静点、刹停点、整条 PWM→速度曲线。落地跑,编码器读速度。

## 为什么不用 floor_probe 的分档法

`floor_probe.py floor` 每档都是"爬到 kick -> 保持 -> **直接掉到候选值**"。那个
下跳本身就会让电调断输出,于是每个候选值都读到"停了" —— 测的是"能不能承受
下跳",不是"能不能维持"。2026-09-22 实测确认测不出来。

这里改成**连续缓慢升降,全程不跳变**:

    中位 --(每 0.1s +2)--> 一直爬到车动起来        = 破静点
         --(继续爬)------> 到 --top 或够快了
         --(每 --down-sec 降 1)--> 直到车停下       = 刹停点

降的时候每一档都记速度,顺带就是整条 PWM→速度曲线 —— `speed_loop.py` 的前馈
要的就是它。全程不跳变还有个好处:电调拒绝突变(见 memory
esc-rejects-throttle-jumps),分档法本来就容易误判成"带不动"。

三个数都随电量漂,**记一下测的时候电池什么状态**。

用法(车上,落地,前方留够 --max-dist):
    python3 ramp_curve.py
    python3 ramp_curve.py --down-sec 0.4                 # 降得更慢,分辨率更高
    python3 ramp_curve.py --no-guard                     # 场地窄,自己盯着

分段测(一段一趟,每趟 10 米以内,跑完把车拿回起点):
    python3 ramp_curve.py --top 1600 --seg-hi 1590 --bottom 1580 --down-sec 1.0 --csv /tmp/seg1.csv
    python3 ramp_curve.py --top 1600 --seg-hi 1580 --bottom 1570 --down-sec 1.0 --csv /tmp/seg2.csv
起步照常爬过破静点,然后每 0.1s 快速降到 --seg-hi,从那里每档停 --down-sec 慢测。
前方净空按当前车速留滑停距离(电调不刹车),见 stop_gap()。
"""
import argparse
import os
import signal
import statistics
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))          # scan_guard
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # fastsim

from common.params import CarParams
from vehicle.scan_guard import ScanGuard

PWM_MID = CarParams().pwm_mid
TURN_MID = CarParams().turn_mid
COAST_DECEL = CarParams().coast_decel
MOVING = 0.05           # m/s,超过这个算在动
STALL_HOLD = 3          # 连续这么多档都低于 MOVING 才算真停(躲开单点噪声)
ABORT_SPEED = 2.0       # m/s
ABORT_GAP = 0.6         # 滑停之后保险杠前还要剩的净空(米)
FAST_SEC = 0.1          # 降到 --seg-hi 之前每档只停这么久(不记曲线)


def stop_gap(v: float) -> float:
    """前方净空小于这个就得松油。电调不刹车(实测 0.73 m/s² 自由滑行),
    所以要留出滑停距离 v²/2a,再加 ABORT_GAP。原来固定 0.6m:1.5 m/s 时
    光滑行就要 1.5m,会撞上去。"""
    return ABORT_GAP + v * v / (2 * COAST_DECEL)


def settled_mean(samples):
    """一档的稳态速度:只取后一半样本。前一半是上一档过来的过渡段。"""
    tail = samples[len(samples) // 2:]
    return statistics.mean(tail) if tail else 0.0


class Ramp(Node):
    def __init__(self):
        super().__init__('ramp_curve')
        self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
        self.create_subscription(Odometry, '/encoder', self.on_enc, 20)
        self.guard = ScanGuard()
        self.create_subscription(LaserScan, '/scan', lambda m: self.guard.update(m), 1)
        self.lock = threading.Lock()
        self.samples = []
        self.dist = 0.0
        self.last_t = None
        self.speed = 0.0
        self.have_enc = False
        self.cmd = float(PWM_MID)
        self.csv = None         # 打开的话每帧编码器写一行 t,pwm,v —— 拟合 τ/θ 用

    def on_enc(self, m):
        now = time.time()
        with self.lock:
            self.have_enc = True
            v = m.twist.twist.linear.x
            if self.last_t is not None:
                self.dist += abs(v) * (now - self.last_t)
            self.last_t = now
            self.speed = v
            self.samples.append(v)
            if self.csv:
                self.csv.write(f'{now:.4f},{self.cmd:.0f},{v:.4f}\n')

    def take(self):
        with self.lock:
            s, self.samples = self.samples, []
        return s

    def send(self, pwm):
        self.cmd = float(pwm)
        tw = Twist()
        tw.linear.x, tw.angular.z = float(pwm), float(TURN_MID)
        self.pub.publish(tw)


class Blocked(Exception):
    pass


def hold(node, pwm, seconds, no_guard):
    """保持一个 PWM,返回这段时间的速度样本。"""
    end = time.time() + seconds
    while time.time() < end:
        if not no_guard:
            gap = node.guard.gap()
            if gap is None or gap < stop_gap(node.speed):
                raise Blocked('雷达无数据' if gap is None else
                              f'前方只剩 {gap:.2f}m,{node.speed:.2f} m/s 滑停要 '
                              f'{stop_gap(node.speed):.2f}m')
        if abs(node.speed) > ABORT_SPEED:
            raise Blocked(f'速度 {node.speed:.2f} m/s 超限')
        node.send(pwm)
        time.sleep(0.05)
    return node.take()


def mean_speed(samples):
    return statistics.mean(samples) if samples else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--top', type=int, default=1620, help='最高爬到这里就开始降(1620 是安全线)')
    ap.add_argument('--up-step', type=int, default=2, help='上升每步 +多少')
    ap.add_argument('--up-sec', type=float, default=0.15, help='上升每步保持多久')
    ap.add_argument('--down-step', type=int, default=1, help='下降每步 -多少')
    ap.add_argument('--down-sec', type=float, default=0.3, help='下降每步保持多久')
    ap.add_argument('--hold-top', type=float, default=1.0, help='顶点保持多久再降')
    ap.add_argument('--bottom', type=int, default=PWM_MID,
                    help='下降段降到哪里为止。默认到中位;2026-09-22 实测换电之后'
                         '车在 1550 上还以 561mm/s 爬,说明真中位更低 —— 那种情况'
                         '要把这个调到 1550 以下才找得到真正的停车点')
    ap.add_argument('--seg-hi', type=int, default=None,
                    help='分段测:降到这里之前每档只停 0.1s、不记曲线,从这里往下才按 '
                         '--down-sec 慢测。起步必须先过破静点(~1592),要测拐点那段'
                         '(1565~1575)就得先快速降下来,不然 10 米全耗在高速段')
    ap.add_argument('--csv', default=None,
                    help='每帧编码器写一行 t,pwm,v 到这个文件,拟合响应时间用')
    ap.add_argument('--max-dist', type=float, default=10.0)
    ap.add_argument('--no-guard', action='store_true')
    args = ap.parse_args()
    seg_hi = args.top if args.seg_hi is None else args.seg_hi
    if not args.bottom < seg_hi <= args.top:
        sys.exit(f'要满足 --bottom < --seg-hi <= --top,给的是 '
                 f'{args.bottom} / {seg_hi} / {args.top}')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Ramp()
    if args.csv:
        # 行缓冲:退出走 os._exit,不会替我们 flush
        node.csv = open(args.csv, 'w', buffering=1)
        node.csv.write('t,pwm,v\n')
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

    print('中位 2s,等传感器…', flush=True)
    t0 = time.time()
    while time.time() - t0 < 10:
        node.send(PWM_MID)
        time.sleep(0.1)
        if node.have_enc and (args.no_guard or node.guard.gap() is not None):
            break
    if not node.have_enc:
        print('等不到 /encoder —— 底盘在跑吗?ROS_DOMAIN_ID 对吗?')
        neutral_exit()
    if not args.no_guard and node.guard.gap() is None:
        print('等不到 /scan;修雷达,或者加 --no-guard 自己盯着')
        neutral_exit()

    breakaway = stall = None
    up_curve, down_curve = [], []
    try:
        # --- 上升:找破静点 ---
        print(f'\n缓慢爬升({PWM_MID} -> {args.top},每 {args.up_sec}s '
              f'+{args.up_step})', flush=True)
        print(f'{"PWM":>5} {"mm/s":>8}', flush=True)
        pwm = PWM_MID
        up_hits = 0
        node.take()
        while pwm < args.top:
            pwm = min(args.top, pwm + args.up_step)
            v = mean_speed(hold(node, pwm, args.up_sec, args.no_guard))
            up_curve.append((pwm, v))
            # 连续 STALL_HOLD 档都超阈值才算起转。只看一帧的话会被编码器毛刺骗:
            # 2026-09-22 晚实测,静止时偶尔蹦一个 ±200mm/s 的单帧,同一台车同样
            # 条件下误判出 1554,连测四次的真值是 1592。
            up_hits = up_hits + 1 if v > MOVING else 0
            if breakaway is None and up_hits >= STALL_HOLD:
                breakaway = pwm - (STALL_HOLD - 1) * args.up_step
                print(f'{pwm:>5} {v*1000:>8.0f}   <<< 连续 {STALL_HOLD} 档在动,'
                      f'破静点 {breakaway}', flush=True)
            elif breakaway is not None:
                print(f'{pwm:>5} {v*1000:>8.0f}', flush=True)
        if breakaway is None:
            print(f'爬到 {args.top} 都没动 —— 提高 --top,或者检查电量/电调', flush=True)
            neutral_exit()

        hold(node, args.top, args.hold_top, args.no_guard)

        # --- 下降:找刹停点 + 记曲线 ---
        if seg_hi < args.top:
            print(f'\n快速降到 {seg_hi}(每 {FAST_SEC}s -{args.down_step},不记)', flush=True)
        print(f'\n缓慢下降(每 {args.down_sec}s -{args.down_step},取每档后一半的平均),'
              f'速度塌下去就是刹停点', flush=True)
        print(f'{"PWM":>5} {"mm/s":>8}', flush=True)
        low = 0
        pwm = args.top
        while pwm > args.bottom:
            pwm -= args.down_step
            if pwm > seg_hi:
                hold(node, pwm, FAST_SEC, args.no_guard)
                if node.dist > args.max_dist:
                    print(f'还没降到 {seg_hi} 就跑出 {node.dist:.1f}m —— 把 --top '
                          f'压低一点,或者场地要更长', flush=True)
                    break
                continue
            v = settled_mean(hold(node, pwm, args.down_sec, args.no_guard))
            down_curve.append((pwm, v))
            print(f'{pwm:>5} {v*1000:>8.0f}   (已跑 {node.dist:.1f}m)', flush=True)
            low = low + 1 if v <= MOVING else 0
            if low >= STALL_HOLD:
                stall = pwm + (STALL_HOLD - 1) * args.down_step   # 第一次掉下去那档
                break
            if node.dist > args.max_dist:
                print(f'已跑出 {node.dist:.1f}m,到上限,中止', flush=True)
                break
    except Blocked as e:
        print(f'\n中止:{e}', flush=True)

    print('\n==== 结果 ====', flush=True)
    print(f'破静点(从静止能动起来)  {breakaway}', flush=True)
    print(f'刹停点(降到这里停住)    {stall}', flush=True)
    if breakaway and stall:
        print(f'滞回                      {breakaway - stall} 个 PWM', flush=True)
        print(f'\n填 teleop/speed_loop.py 的 SpeedLoopConfig:', flush=True)
        print(f'    pwm_floor     = {stall + 2}      # 刹停点 {stall} 往上留 2 的余量',
              flush=True)
        print(f'    pwm_breakaway = {breakaway}', flush=True)
    moving = [(p, v) for p, v in down_curve if v > MOVING]
    if moving:
        print(f'\n下降段曲线(闭环前馈用,{len(moving)} 个点):', flush=True)
        for p, v in moving:
            print(f'    {p} -> {v*1000:.0f} mm/s', flush=True)
        lo, hi = moving[-1], moving[0]
        if hi[0] != lo[0]:
            k = (hi[1] - lo[1]) / (hi[0] - lo[0])
            print(f'\n斜率 ≈ {k*1000:.0f} mm/s 每 PWM;'
                  f'0.6 m/s 大概在 {lo[0] + (0.6 - lo[1]) / k:.0f}', flush=True)
    neutral_exit()


if __name__ == '__main__':
    main()
