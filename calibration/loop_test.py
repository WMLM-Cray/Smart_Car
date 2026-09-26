#!/usr/bin/env python3
"""直线跑 speed_loop 闭环:按目标速度序列跑,看跟不跟得上、会不会振、会不会停。落地跑。

    python3 loop_test.py                                  # 默认 0.75:2,0.9:2,0.7:2
    python3 loop_test.py --plan 0.8:3 --csv /tmp/l1.csv
    python3 loop_test.py --kp 5 --ki 45                   # 临时换参数对比
    python3 loop_test.py --up-rate 30                     # 起步爬快一点(电调认不认要实测)
    python3 loop_test.py --launch-from 1580               # 起步先跳进死区再爬(先架空测)

方向固定在 turn_mid 走直线。雷达防撞、超速、跑够 --max-dist 都会松油回中位(电调不刹车,
靠滑停,防撞按滑停距离留净空,和 ramp_curve 同一套)。
每帧编码器写 t,target,pwm,v 到 --csv。
"""
import argparse
import os
import signal
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vehicle.speed_loop import SpeedLoop, SpeedLoopConfig  # noqa: E402

DT = 0.05       # 20Hz,和策略节点的 control_dt 一致


def parse_plan(text):
    """'0.75:2,0.9:2' -> [(0.75, 2.0), (0.9, 2.0)]。起步那段不算在秒数里。"""
    plan = []
    for part in text.split(','):
        v, sec = part.split(':')
        v, sec = float(v), float(sec)
        if not 0.0 < v <= 1.2 or sec <= 0:
            raise ValueError(f'{part}: 速度要在 (0, 1.2]、秒数要 > 0')
        plan.append((v, sec))
    return plan


def summarize(rows):
    """一段的后一半:平均速度、速度峰峰值、PWM 范围。rows = [(pwm, v)]"""
    tail = rows[len(rows) // 2:]
    if not tail:
        return None
    vs = [v for _, v in tail]
    ps = [p for p, _ in tail]
    return statistics.mean(vs), max(vs) - min(vs), min(ps), max(ps)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plan', default='0.75:2,0.9:2,0.7:2',
                    help='目标速度:秒数,逗号分隔。第一段从静止起步,起步时间另算')
    ap.add_argument('--start-limit', type=float, default=6.0,
                    help='起步最多等这么久还没到第一段目标的 80%% 就放弃')
    ap.add_argument('--kp', type=float, default=None)
    ap.add_argument('--ki', type=float, default=None)
    ap.add_argument('--up-rate', type=float, default=None,
                    help='起步/加速的爬升速率 PWM/s,只许 [10, 40](SpeedLoopConfig 会拒绝别的)')
    ap.add_argument('--launch-from', type=int, default=None,
                    help='起步第一拍跳到这个 PWM 再爬(死区里,< 破静点)。电调认不认先架空测')
    ap.add_argument('--target-rate', type=float, default=None,
                    help='目标速度斜坡 m/s²,0 = 直接跟目标')
    ap.add_argument('--csv', default=None)
    ap.add_argument('--max-dist', type=float, default=7.0)
    args = ap.parse_args()
    plan = parse_plan(args.plan)

    # ROS 的导入放这里:纯逻辑(parse_plan/summarize)的测试不需要 ROS
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from calibration.ramp_curve import ABORT_SPEED, PWM_MID, Ramp, stop_gap

    class LoopNode(Ramp):
        def __init__(self):
            super().__init__()
            self.target = 0.0

        def on_enc(self, m):
            # 父类写 t,pwm,v;这里多一列目标速度
            csv, self.csv = self.csv, None
            super().on_enc(m)
            self.csv = csv
            if csv:
                csv.write(f'{time.time():.4f},{self.target:.2f},{self.cmd:.0f},'
                          f'{m.twist.twist.linear.x:.4f}\n')

    over = {k: v for k, v in (('kp', args.kp), ('ki', args.ki), ('up_rate', args.up_rate),
                                 ('target_rate', args.target_rate),
                                 ('launch_from', args.launch_from))
            if v is not None}
    cfg = SpeedLoopConfig(**over)
    loop = SpeedLoop(cfg)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LoopNode()
    if args.csv:
        node.csv = open(args.csv, 'w', buffering=1)
        node.csv.write('t,target,pwm,v\n')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def neutral_exit(*_):
        node.target = 0.0
        for _ in range(5):
            node.send(PWM_MID)
            time.sleep(0.05)
        print('已回中位', flush=True)
        os._exit(0)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, neutral_exit)

    print('中位 2s,等传感器…', flush=True)
    t0 = time.time()
    while time.time() - t0 < 10:
        node.send(PWM_MID)
        time.sleep(0.1)
        if node.have_enc and node.guard.gap() is not None and time.time() - t0 > 2:
            break
    if not node.have_enc or node.guard.gap() is None:
        print('等不到 /encoder 或 /scan —— 先跑 carready.py')
        neutral_exit()

    print(f'kp={cfg.kp} ki={cfg.ki} up_rate={cfg.up_rate} target_rate={cfg.target_rate} i_limit={cfg.i_limit} floor={cfg.pwm_floor} '
          f'breakaway={cfg.pwm_breakaway}', flush=True)
    node.dist = 0.0
    results, why = [], None
    try:
        for i, (target, sec) in enumerate(plan):
            node.target = target
            rows, t_seg = [], time.time()
            starting = i == 0
            while True:
                now = time.time()
                if starting:
                    if node.speed >= 0.8 * target:
                        print(f'起步用了 {now - t_seg:.1f}s,{node.dist:.1f}m', flush=True)
                        starting, t_seg, rows = False, now, []
                    elif now - t_seg > args.start_limit:
                        raise RuntimeError(f'{args.start_limit}s 没跑到 {0.8 * target:.2f} m/s,'
                                           f'PWM 停在 {loop.last_pwm:.0f}')
                elif now - t_seg >= sec:
                    break
                gap = node.guard.gap()
                if gap is None or gap < stop_gap(node.speed):
                    raise RuntimeError('雷达无数据' if gap is None else
                                       f'前方只剩 {gap:.2f}m')
                if abs(node.speed) > ABORT_SPEED:
                    raise RuntimeError(f'速度 {node.speed:.2f} 超限')
                if node.dist > args.max_dist:
                    raise RuntimeError(f'跑满 {args.max_dist}m')
                pwm = loop.update(target, node.speed, DT)
                node.send(round(pwm))
                if not starting:
                    rows.append((round(pwm), node.speed))
                time.sleep(DT)
            results.append((target, summarize(rows), loop.floor))
    except RuntimeError as e:
        why = str(e)
        if rows and not starting:
            results.append((node.target, summarize(rows), loop.floor))
    node.target = 0.0
    for _ in range(5):
        node.send(PWM_MID)
        time.sleep(0.05)

    print(f'\n{"停下原因:" + why if why else "序列跑完"}  共 {node.dist:.1f}m')
    print(' 目标   实际均值  速度峰峰  PWM 范围     floor')
    for target, s, floor in results:
        if s:
            mean, pp, lo, hi = s
            print(f' {target:.2f}   {mean:.3f}    {pp:.3f}    {lo}~{hi}    {floor:.0f}')
    print('(每段只看后一半)')
    neutral_exit()


if __name__ == '__main__':
    main()
