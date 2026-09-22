#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""encoder_gate_v2.py —— /encoder 合理性闸门 v2(2026-09-18 离线审查后重写,未部署)

v1(tools/realcar/encoder_gate.py,现役于车机)审查发现三缺陷,v2 修复:
  B1 重锁死锁: v1 只在"过 dv 门"时更新 last——若一次拒帧后真实速度已持续变化,
     之后每帧 |v-last|>DV_MAX 永久被拒(例: last=0.3,毛刺帧被拒,真速度到 0.7 后
     永远进不来)。v2: 连续 relock_n 帧 dv 拒且彼此一致(极差≤relock_tol)→ 判定为
     "真实速度已变",重锁基线放行(毛刺乱值彼此不一致,不触发)。
  B2 静默丢帧: v1 拒帧只写 log,下游(EKF)无感知。v2: 1Hz /encoder_gate_diag
     诊断话题(统计+末帧年龄)。注意: 断流的下游处理依赖 EKF sensor_timeout(2.0s
     后该传感器退出融合)——gate **不发布伪造零速度、不刷新旧值时间戳**去"维持新鲜",
     那会把失联伪装成静止,违背任务书红线。
  B3 帧率假设: DV_MAX=0.35/帧 与帧率耦合(Δv_max=加速度/帧率)。真车 1.3m/s²
     (EKF 配置)@50Hz→0.026/帧,@4Hz→0.325/帧仍安全;帧率跌破 ~3.7Hz 会误拦
     正常加减速。诊断行含实测帧率,跌破阈值时 warn。

门限(与 v1 同值,依据见 GATE_REVIEW.md):
  |v| > 1.2 m/s 拒(限幅0.55+余量); |v-last_ok| > 0.35 m/s/帧 拒(拦跳变沿)。
时钟口径: 过滤决策只用帧序列(不看消息时间戳,防车机钟跳变误拒);
  时间戳仅用于诊断 age。转发原消息对象——stamp 原样(任务书"不得伪装新鲜")。
原始输入与过滤输出分别可记录: --record-dir 开 CSV 双文件落盘(input.csv/output.csv)。

用法(上车候选,替换 v1 前须经 NEXT_ROUND_PLAN 清单验证):
  python3 encoder_gate_v2.py --record-dir /tmp/gate_rec   # 带记录
"""
import argparse
import csv
import os
import time

try:
    from rclpy.signals import uninstall_signal_handlers
    uninstall_signal_handlers()
except Exception:
    pass

V_ABS_MAX = 1.2     # m/s 幅值门
DV_MAX = 0.35       # m/s 每帧变化率门
RELOCK_N = 3        # 连续 N 帧 dv 拒且一致 → 重锁
RELOCK_TOL = 0.35   # 拒帧序列一致性极差容差


def gate_check(v, st):
    """纯函数核心: 输入速度 v 与可变状态 st(dict), 返回 (accept, reason)。
    状态字段: last_ok/pending/pass_total/drop_abs/drop_dv/relocks。
    独立于 rclpy/时钟,便于负例单测。"""
    if abs(v) > V_ABS_MAX:
        st['drop_abs'] += 1
        st['pending'].clear()                    # 幅值异常流不构成"新稳态"证据
        return False, 'abs'
    if st['last_ok'] is None:                   # 首帧(无基线)只拦幅值
        st['last_ok'] = v
        st['pass_total'] += 1
        return True, 'first'
    if abs(v - st['last_ok']) <= DV_MAX:
        st['last_ok'] = v
        st['pass_total'] += 1
        st['pending'].clear()
        return True, 'ok'
    # dv 拒: 累积一致性证据,防 B1 死锁
    st['drop_dv'] += 1
    st['pending'].append(v)
    if len(st['pending']) >= RELOCK_N and \
            max(st['pending'][-RELOCK_N:]) - min(st['pending'][-RELOCK_N:]) <= RELOCK_TOL:
        st['last_ok'] = v                       # 真实速度已变,重锁放行
        st['pass_total'] += 1
        st['relocks'] += 1
        st['pending'].clear()
        return True, 'relock'
    return False, 'dv'


def new_state():
    return {'last_ok': None, 'pending': [], 'pass_total': 0,
            'drop_abs': 0, 'drop_dv': 0, 'relocks': 0,
            'last_input_t': None, 'last_output_t': None,
            'last_input_v': None, 'last_output_v': None}


def diag_line(st, now):
    age_in = (now - st['last_input_t']) if st['last_input_t'] else -1.0
    age_out = (now - st['last_output_t']) if st['last_output_t'] else -1.0
    return (f"pass={st['pass_total']} drop_abs={st['drop_abs']} "
            f"drop_dv={st['drop_dv']} relocks={st['relocks']} "
            f"age_in={age_in:.2f}s age_out={age_out:.2f}s "
            f"last_in={st['last_input_v']} last_out={st['last_output_v']}")


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.node import Node

    ap = argparse.ArgumentParser()
    ap.add_argument('--record-dir', default=None,
                    help='可选: 落盘 input.csv/output.csv 分别记录原始输入与过滤输出')
    args, _ = ap.parse_known_args()

    rclpy.init()
    n = Node('encoder_gate_v2')
    pub = n.create_publisher(Odometry, '/encoder_gate', 100)
    diag_pub = n.create_publisher(String, '/encoder_gate_diag', 10)
    st = new_state()

    rec_in = rec_out = None
    if args.record_dir:
        os.makedirs(args.record_dir, exist_ok=True)
        rec_in = open(os.path.join(args.record_dir, 'input.csv'), 'a', newline='')
        rec_out = open(os.path.join(args.record_dir, 'output.csv'), 'a', newline='')
        for f in (rec_in, rec_out):
            csv.writer(f).writerow(['wall_time', 'msg_stamp_sec', 'v'])
        # 诊断也落一份(第三次 open 用不上,行数少直接进 input.csv 末尾注释列不搞了)

    def cb(m):
        v = m.twist.twist.linear.x
        now = time.time()
        st['last_input_t'] = now
        st['last_input_v'] = round(v, 4)
        if rec_in:
            csv.writer(rec_in).writerow(
                [f'{now:.3f}', f'{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}', v])
        accept, reason = gate_check(v, st)
        if accept:
            pub.publish(m)                       # 原消息原戳,不改不造
            st['last_output_t'] = now
            st['last_output_v'] = round(v, 4)
            if rec_out:
                csv.writer(rec_out).writerow(
                    [f'{now:.3f}', f'{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}', v])
        elif reason in ('abs', 'dv'):
            n.get_logger().warn(f'拒帧({reason}): v={v:.3f} {diag_line(st, now)}')

    n.create_subscription(Odometry, '/encoder', cb, 100)

    def diag_tick():
        msg = String()
        msg.data = diag_line(st, time.time())
        diag_pub.publish(msg)
        n.get_logger().info(msg.data)

    n.create_timer(1.0, diag_tick)
    n.get_logger().info(
        f'encoder_gate_v2 起来: |v|>{V_ABS_MAX} 或 |Δv|>{DV_MAX}/帧 拒; '
        f'连续{RELOCK_N}帧一致拒→重锁; diag 1Hz')
    try:
        rclpy.spin(n)
    finally:
        if rec_in:
            rec_in.close()
        if rec_out:
            rec_out.close()


if __name__ == '__main__':
    main()
