#!/usr/bin/env python3
"""测试 PWM 下跳后的速度响应：持续驱动、自由滑行或更强减速。

爬升至 --kick，稳定后降至候选 PWM，记录速度、停止耗时和距离。
已测松油减速度约 0.73–0.78 m/s²；下跳 10、20、25 PWM 曾仍持续驱动。
尚不能把所有下跳都视为断输出，也不能只凭较大减速度断言电调主动刹车。
雷达在加速及记录阶段预留滑停距离；触发防撞时切中位，截断结果不能当作候选 PWM 的完整响应。
候选值可以低于中位(测刹车):电调可能刹车也可能直接倒车,车速降到 BRAKE_RELEASE 就回中位,
不等停稳(停住后接着发就是倒车,雷达只看前方)。所以停车耗时/距离含最后一小段滑行。

运行：python3 -m calibration.coast_test --help
"""
import argparse
import os
import signal
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))          # scan_guard
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # fastsim

from common.params import CarParams
from vehicle.scan_guard import ScanGuard

PWM_MID = CarParams().pwm_mid
TURN_MID = CarParams().turn_mid
MOVING = 0.05           # m/s,低于这个算停了
STOP_HOLD = 3           # 连续这么多帧低于 MOVING 才算真停
ABORT_SPEED = 2.0       # m/s,加速段超过就中止
ABORT_GAP = 0.6         # 加速段:滑停之后保险杠前还要剩的净空(米)
COAST_DECEL = CarParams().coast_decel


def stop_gap(v: float) -> float:
    """加速段前方净空小于这个就得松油:滑停距离 v²/2a 加 ABORT_GAP。电调不刹车,
    原来固定 0.6m 在 1 m/s 时来不及(光滑行就要 0.7m)。同 ramp_curve.stop_gap。"""
    return ABORT_GAP + v * v / (2 * COAST_DECEL)


COAST_MAX = 8.0         # 滑行段最多记这么久(秒),再不停就是没停
BRAKE_RELEASE = 0.15    # m/s,低于中位的候选值:降到这个速度就回中位

# 减速度分档。滚阻+传动阻力大概 0.3~0.8 m/s²;电调刹车是量级上的区别。
COAST_MAX_DECEL = 1.0   # 小于这个算自由滑行
BRAKE_MIN_DECEL = 2.0   # 大于这个算明确在刹车


def summarize(samples):
    """samples: [(t, v)],从松油那一刻开始。返回 (v0, 停车耗时, 滑行距离, 平均减速度)。

    停车耗时为 None = 记到 COAST_MAX 还没停,这时减速度按整段算,是个下界。
    """
    if len(samples) < 4:
        return None
    t0 = samples[0][0]
    v0 = statistics.mean([abs(v) for _, v in samples[:3]])
    dist = 0.0
    prev_t = t0
    low = 0
    stop_t = None
    for t, v in samples:
        dist += abs(v) * (t - prev_t)
        prev_t = t
        low = low + 1 if abs(v) < MOVING else 0
        if low >= STOP_HOLD:
            stop_t = t - t0
            break
    span = stop_t if stop_t is not None else (samples[-1][0] - t0)
    decel = v0 / span if span > 0 else float('nan')
    return v0, stop_t, dist, decel


def speed_at(samples, after, half=0.2):
    """松油后 after 秒附近(±half)的平均速度;那段没数据返回 None。

    用来分"下跳后电调接着出力、落到平台速度继续跑"和"下跳让电调断输出、滑停"。
    前者 1s 后还在 ~0.47 m/s(11.24V 平台),后者按 0.78 m/s² 滑,从 0.9 起 1s 后只剩 ~0.1。
    """
    if not samples:
        return None
    t0 = samples[0][0]
    vs = [v for t, v in samples if abs(t - t0 - after) <= half]
    return statistics.mean(vs) if vs else None


def verdict(decel, stopped):
    if not stopped:
        return '观察结束仍在运动，需结合目标 PWM 判断是否持续驱动'
    if decel < COAST_MAX_DECEL:
        return '自由滑行'
    if decel > BRAKE_MIN_DECEL:
        return '减速度较大，需排查主动制动、碰撞或测量误差'
    return '介于两者之间'


def coast_cmd(cmd: int, speed: float) -> int:
    """滑行段这一拍发什么。低于中位(刹车/倒车)时车快停就回中位,之后一直中位(cmd 被换掉)。"""
    if cmd < PWM_MID and abs(speed) < BRAKE_RELEASE:
        return PWM_MID
    return cmd


class Blocked(Exception):
    pass


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--coast', type=int, nargs='+', required=True,
                    help='要试的松油 PWM(降到这里看怎么停)')
    ap.add_argument('--kick', type=int, default=1610, help='加速段油门,实测平台在 1605~1612')
    ap.add_argument('--kick-sec', type=float, default=1.5, help='加速段保持多久等速度稳')
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--min-gap', type=float, default=4.0, help='开跑前要求的前方净空(米)')
    ap.add_argument('--max-dist', type=float, default=10.0,
                    help='整次运行累计跑这么远就收油回中位。注意滑行段收了油也停不住,'
                         '这是限位不是刹车')
    ap.add_argument('--pause', type=float, default=8.0, help='每次跑完回中位多久(够你搬车)')
    ap.add_argument('--no-guard', action='store_true')
    args = ap.parse_args()
    if args.kick > 1620:
        ap.error('--kick 不要超过 1620(和策略节点同一条安全线)')
    for c in args.coast:
        if c > args.kick:
            ap.error(f'--coast {c} 比 --kick {args.kick} 还高,这不是松油')

    # rclpy 放在这里导入,不放模块顶上 —— 这样 summarize()/verdict() 能在没有 ROS
    # 的开发机上直接 import 进来测(teleop/test_coast_test.py 就是这么跑的)。
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    class Coast(Node):
        def __init__(self):
            super().__init__('coast_test')
            self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
            self.create_subscription(Odometry, '/encoder', self.on_enc, 20)
            self.guard = ScanGuard()
            self.create_subscription(LaserScan, '/scan', lambda m: self.guard.update(m), 1)
            self.lock = threading.Lock()
            self.speed = 0.0
            self.have_enc = False
            self.rec = None
            self.total = 0.0        # 整次运行累计走了多远,限位用
            self.last_t = None

        def on_enc(self, m):
            now = time.time()
            with self.lock:
                self.have_enc = True
                self.speed = m.twist.twist.linear.x
                if self.last_t is not None:
                    self.total += abs(self.speed) * (now - self.last_t)
                self.last_t = now
                if self.rec is not None:
                    self.rec.append((now, self.speed))

        def start_rec(self):
            with self.lock:
                self.rec = []

        def stop_rec(self):
            with self.lock:
                r, self.rec = self.rec, None
            return r or []

        def send(self, pwm):
            tw = Twist()
            tw.linear.x, tw.angular.z = float(pwm), float(TURN_MID)
            self.pub.publish(tw)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Coast()
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

    def hold(pwm, seconds, guard=True):
        end = time.time() + seconds
        while time.time() < end:
            if node.total > args.max_dist:
                raise Blocked(f'已累计跑出 {node.total:.1f}m,到 --max-dist 限位')
            if guard and not args.no_guard:
                gap = node.guard.gap()
                if gap is None or gap < stop_gap(node.speed):
                    raise Blocked('雷达无数据' if gap is None else f'前方只剩 {gap:.2f}m')
            if abs(node.speed) > ABORT_SPEED:
                raise Blocked(f'速度 {node.speed:.2f} m/s 超限')
            node.send(pwm)
            time.sleep(0.05)

    def ramp_to(target):
        """从中位每 0.1s +2 爬上去。电调不吃从中位直跳高油门,见 memory
        esc-rejects-throttle-jumps。"""
        pwm = PWM_MID
        while pwm < target:
            pwm = min(target, pwm + 2)
            hold(pwm, 0.1)

    print('中位,等传感器…', flush=True)
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

    print(f'\n加速到 {args.kick} 保持 {args.kick_sec}s,然后一步降到候选值。', flush=True)
    print(f'{"松油":>5} {"次":>3} {"松油时m/s":>10} {"停车s":>7} {"滑行m":>7} '
          f'{"减速度m/s²":>11}  判断', flush=True)

    results = {}
    try:
        for coast in args.coast:
            rows = []
            for k in range(args.repeat):
                if not args.no_guard:
                    gap = node.guard.gap()
                    if gap is None or gap < args.min_gap:
                        raise Blocked(f'开跑前前方只有 {gap}m,要求 {args.min_gap}m '
                                      f'—— 须留足滑停距离；防撞只切中位，不主动刹车')
                ramp_to(args.kick)
                hold(args.kick, args.kick_sec)
                if abs(node.speed) < MOVING:
                    print(f'kick {args.kick} 没动起来,提高 --kick', flush=True)
                    neutral_exit()

                # 滑行段不抛异常中止(会丢数据),但**要接雷达**:候选值高于刹停点时
                # 下跳后电调可能接着出力,车还在自己跑 —— 那时切中位是停得住的。
                # 前方不够滑停就切中位,继续记录,这次的数据标成"防撞截断"。
                node.start_rec()
                end = time.time() + COAST_MAX
                cmd, cut = coast, False
                while time.time() < end:
                    if node.total > args.max_dist:
                        print(f'  到 {args.max_dist}m 限位,收油', flush=True)
                        break
                    if not args.no_guard and not cut:
                        gap = node.guard.gap()
                        if gap is None or gap < stop_gap(node.speed):
                            cmd, cut = PWM_MID, True
                            print(f'  防撞:前方 {gap}m,切中位', flush=True)
                    cmd = coast_cmd(cmd, node.speed)
                    node.send(cmd)
                    time.sleep(0.05)
                    with node.lock:
                        r = node.rec
                    if r and len(r) >= STOP_HOLD and all(
                            abs(v) < MOVING for _, v in r[-STOP_HOLD:]):
                        break
                samples = node.stop_rec()
                s = summarize(samples)
                if s is None:
                    print(f'{coast:>5} {k+1:>3}   编码器数据太少', flush=True)
                    continue
                v0, stop_t, dist, decel = s
                rows.append((v0, stop_t, dist, decel))
                print(f'{coast:>5} {k+1:>3} {v0:>10.2f} '
                      f'{("%.1f" % stop_t) if stop_t is not None else "没停":>7} '
                      f'{dist:>7.2f} {decel:>11.2f}  {verdict(decel, stop_t is not None)}',
                      flush=True)
                v1, v2 = speed_at(samples, 1.0), speed_at(samples, 2.0)
                fmt = lambda v: '—' if v is None else f'{v:.2f}'
                print(f'        松油后 1s {fmt(v1)} m/s,2s {fmt(v2)} m/s'
                      f'{"  (防撞截断)" if cut else ""}', flush=True)

                for _ in range(int(args.pause / 0.05)):
                    node.send(PWM_MID)
                    time.sleep(0.05)
            if rows:
                results[coast] = rows
    except Blocked as e:
        print(f'\n中止:{e}', flush=True)

    if results:
        print('\n汇总(减速度越小越接近自由滑行):', flush=True)
        for c in sorted(results):
            d = statistics.median([r[3] for r in results[c]])
            dist = statistics.median([r[2] for r in results[c]])
            stopped = all(r[1] is not None for r in results[c])
            print(f'  {c:>5} {d:>6.2f} m/s²  滑行 {dist:>5.2f}m  {verdict(d, stopped)}',
                  flush=True)
        best = min(results, key=lambda c: statistics.median([r[3] for r in results[c]]))
        print(f'\n>>> 最接近自由滑行的是 {best}', flush=True)
    neutral_exit()


if __name__ == '__main__':
    main()
