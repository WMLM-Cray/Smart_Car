#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""encoder_tap.py —— /dev/encoder 唯一读取者: 原始字节记录+新旧解析器同流对照
(2026-09-18 早,用户任务书 8 条约束落实;离线写,未上车)

设计约束(逐条对应任务书):
  [第2条 单读者] 本进程独占 /dev/encoder——**厂家 encoder 节点必须先停**
    (kill 其进程释放串口,交接间隙 EKF sensor_timeout=2.0s 内无碍,推车前静止完成)。
    绝不允许 cat/xxd 等旁路程序与任何节点同时读该串口。
  [第2条 记录] 每周期记录: 单调时钟 time.monotonic()(不受车机系统钟跳变影响)
    + 所发请求帧 hex + 收到的原始响应字节块 hex → CSV 逐行落盘。
  [第2条 同流对照] 同一份响应字节分别喂:
      旧 = DueVelData+厂家公式(逐字保留) → **发布 /encoder(顶替厂家语义,EKF 输入不变)**
      新 = encoder_proto CRC 提取器 → 只旁路: 落盘 + 发布 /encoder_fixed_obs(**不供给 EKF**)
  [第3条 协议实证前] 新解析输出只做旁路观察;响应协议(含异常响应帧)由落盘字节离线裁决。
  [第7条 失效可见性] --stop-publish-after N: N 秒后停止发布 /encoder 但继续读串口记录
    (隔离实验: 观察 EKF/AMCL 对断流的真实反应,证据进报告)。

厂家节奏保持: 每 0.02s 发两条请求(速度 0x0003 + 圈数 0x0000)各读 7 字节,
与现役 encoder_node.py 相同(不为对照改变下位机被询方式)。

已知厂家缺陷(调用处防护,DueVelData 本体逐字保留不改):
  DueVelData 的 range(len-1) 循环在特定字节模式(如 7 字节帧尾 3 字节=01 03 02)下
  会 IndexError 越界——tap 捕获后记 note=legacy_IndexError,该周期不发布(如实留痕)。

用法(上车,见 NEXT_ROUND_PLAN 阶段 A):
  # 1) 停厂家 encoder 节点(仅进程,不动安装目录)
  # 2) python3 encoder_tap.py --out-dir /tmp/tap_rec --duration 300
  Ctrl+C 或 --duration 到时安全退出;CSV 每 5s flush。
"""
import argparse
import csv
import os
import time

from encoder_proto import (
    ModbusRtuExtractor, decode_first_register, to_signed16,
    counts_to_velocity, REQUEST_VEL, REQUEST_DIR,
)
from encoder_node_fixed import DueVelData  # 厂家旧解析器逐字保留

try:
    from rclpy.signals import uninstall_signal_handlers
    uninstall_signal_handlers()
except Exception:
    pass


def legacy_velocity(datahex, k=1.0):
    """厂家公式原样(与 encoder_node_fixed 同式,独立复写防交叉耦合)。"""
    angle_v = DueVelData(datahex)
    C = 227.45 / 1000
    return 3.57 * angle_v * C / 1024.0 / 0.02 * k * 0.25 * 1.03


class TapWorker:
    """单周期读取+双解析对照的纯逻辑核(不依赖 rclpy/serial,离线可测)。"""

    def __init__(self, k=1.0):
        self.k = k
        self.ex = ModbusRtuExtractor()
        self.n_ok = self.n_legacy_err = self.n_fixed_reject = 0
        self.last_fixed_vel = None

    def cycle_once(self, ser, t_mono):
        """一个完整厂家周期(双请求)。返回 record dict(始终返回,坏周期也记)。"""
        rec = {'t': t_mono, 'req_v': REQUEST_VEL.hex(), 'resp_v': '',
               'req_d': REQUEST_DIR.hex(), 'resp_d': '',
               'legacy_vel': None, 'fixed_vel': None, 'note': ''}

        ser.write(REQUEST_VEL)
        resp_v = ser.read(7)
        rec['resp_v'] = resp_v.hex() if resp_v else ''

        ser.write(REQUEST_DIR)
        resp_d = ser.read(7)
        rec['resp_d'] = resp_d.hex() if resp_d else ''

        # 旧解析(厂家语义): 异常捕获留痕,不崩溃。
        # ⚠️ 空响应(超时)绝不当 0.0 发布——DueVelData(b'')数学上返回0,但"无数据≠零速",
        # 厂家节点因 read 永阻塞从不遇此情形;tap 带 timeout,必须显式不发布(任务书第7条)。
        if resp_v:
            try:
                rec['legacy_vel'] = legacy_velocity(resp_v, k=self.k)
                self.n_ok += 1
            except Exception as e:                 # noqa: BLE001
                rec['note'] = f'legacy_{type(e).__name__}'
                self.n_legacy_err += 1
        else:
            rec['note'] = 'timeout'

        # 新解析(旁路): 同一份字节喂 CRC 提取器
        self.ex.feed(resp_v)
        frames = self.ex.extract()
        if frames:
            raw = decode_first_register(frames[0])
            rec['fixed_vel'] = counts_to_velocity(
                to_signed16(raw), cycle_s=0.02, k=self.k)
            rec['note'] += f' raw=0x{raw:04X}'
            self.last_fixed_vel = rec['fixed_vel']
        else:
            if resp_v:
                self.n_fixed_reject += 1
                rec['note'] += ' fixed_reject'
        return rec


CSV_HEADER = ['monotonic_s', 'req_v', 'resp_v', 'req_d', 'resp_d',
              'legacy_vel', 'fixed_vel', 'note']


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    import serial

    ap = argparse.ArgumentParser()
    ap.add_argument('--serial-port', default='/dev/encoder')
    ap.add_argument('--baud', type=int, default=57600)
    ap.add_argument('--out-dir', required=True, help='CSV 落盘目录')
    ap.add_argument('--duration', type=float, default=0,
                    help='秒数,0=直到 Ctrl+C')
    ap.add_argument('--cycle', type=float, default=0.02)
    ap.add_argument('--k', type=float, default=1.0)
    ap.add_argument('--stop-publish-after', type=float, default=0,
                    help='N 秒后停止发布 /encoder(数据失效可见性隔离实验),0=不启用')
    ap.add_argument('--publish', choices=['legacy', 'fixed'], default='legacy',
                    help='/encoder 发布用哪个解析的值(默认 legacy=厂家语义顶替; '
                         'fixed=阶段C解析器切换环节,一次只改这一个环节)')
    ap.add_argument('--ros-args', action='store_true', help=argparse.SUPPRESS)
    args, _ = ap.parse_known_args()

    os.makedirs(args.out_dir, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(args.out_dir, f'tap_{stamp}.csv')
    fout = open(csv_path, 'w', newline='')
    w_csv = csv.writer(fout)
    w_csv.writerow(CSV_HEADER)

    rclpy.init()
    n = Node('encoder_tap')
    pub_legacy = n.create_publisher(Odometry, '/encoder', 50)        # 厂家语义顶替
    pub_fixed = n.create_publisher(Odometry, '/encoder_fixed_obs', 50)  # 旁路,不供 EKF
    w = TapWorker(k=args.k)
    n.get_logger().info(
        f'encoder_tap 起来: port={args.serial_port} csv={csv_path} '
        f'duration={args.duration or "ctrl+c"} stop_pub={args.stop_publish_after}')

    ser = serial.Serial(args.serial_port, args.baud, timeout=args.cycle)
    msg = Odometry()
    msg.header.frame_id = ''
    t_start = time.monotonic()
    next_flush = t_start + 5.0
    publishing = True
    n_rec = 0
    try:
        while rclpy.ok():
            t0 = time.monotonic()
            if args.stop_publish_after and publishing and \
                    t0 - t_start >= args.stop_publish_after:
                publishing = False
                n.get_logger().warn(
                    f'--stop-publish-after 生效: 停止发布 /encoder(继续读串口记录)。'
                    f'此刻观察 EKF/AMCL 反应(隔离实验)。')
            rec = w.cycle_once(ser, t0 - t_start)
            w_csv.writerow([f"{rec['t']:.6f}", rec['req_v'], rec['resp_v'],
                            rec['req_d'], rec['resp_d'],
                            '' if rec['legacy_vel'] is None else f"{rec['legacy_vel']:.6f}",
                            '' if rec['fixed_vel'] is None else f"{rec['fixed_vel']:.6f}",
                            rec['note']])
            n_rec += 1
            if publishing and rec['legacy_vel'] is not None:
                msg.header.stamp = n.get_clock().now().to_msg()
                msg.twist.twist.linear.x = rec['legacy_vel']
                if args.publish == 'legacy':
                    pub_legacy.publish(msg)
            if publishing and args.publish == 'fixed' and rec['fixed_vel'] is not None:
                msg.header.stamp = n.get_clock().now().to_msg()
                msg.twist.twist.linear.x = rec['fixed_vel']
                pub_legacy.publish(msg)
            if rec['fixed_vel'] is not None:
                msg.header.stamp = n.get_clock().now().to_msg()
                msg.twist.twist.linear.x = rec['fixed_vel']
                pub_fixed.publish(msg)
            if t0 >= next_flush:
                fout.flush()
                next_flush = t0 + 5.0
                n.get_logger().info(
                    f"rec={n_rec} ok={w.n_ok} legacy_err={w.n_legacy_err} "
                    f"fixed_reject={w.n_fixed_reject}")
            if args.duration and t0 - t_start >= args.duration:
                break
            dt = args.cycle - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)
    finally:
        fout.flush()
        fout.close()
        ser.close()
        n.get_logger().info(
            f'退出: rec={n_rec} ok={w.n_ok} legacy_err={w.n_legacy_err} '
            f'fixed_reject={w.n_fixed_reject} csv={csv_path}')


if __name__ == '__main__':
    main()
