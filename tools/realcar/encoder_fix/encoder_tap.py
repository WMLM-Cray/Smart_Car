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
  [补充口径(0918用户)] CSV 五项分别记录,防"解析失败的零/鬼值"被当正常数据:
    resp_v/resp_d = 原始字节 hex;
    frame_valid   = 帧有效性(CRC 客观判定): valid=正常数据帧 / exc=合法设备异常
                    / bad=坏CRC·垃圾字节 / empty=无响应;
    legacy_vel    = 旧解析值——**坏CRC字节经旧解析得到的 0.0 仅作 legacy 对照值,
                    不算有效静止测量**(分析时以 frame_valid 同行辨别);
    fixed_vel     = 新解析值;
    published     = /encoder 实际发布: legacy/fixed/none/stopped(stopped=
                    --stop-publish-after 失效实验期;旁路 /encoder_fixed_obs 不在此列)。
  [第2条 同流对照] 同一份响应字节分别喂:
      旧 = DueVelData+厂家公式(逐字保留) → 发布 /encoder(**近似顶替**,差异见下,EKF 输入话题不变)
      新 = encoder_proto CRC 提取器 → 只旁路: 落盘 + 发布 /encoder_fixed_obs(**不供给 EKF**)
  [第3条 异常响应分类] 0x83 异常帧不进速度输出;保留原始帧+校验 CRC+记异常码;
      "合法设备异常响应(CRC 过)"与"坏CRC/垃圾字节"分开统计。
      频繁合法异常 → 先查请求地址/寄存器/长度/时序/设备状态,不直接定性下位机故障。
  [第7条 失效可见性] --stop-publish-after N: N 秒后停止发布 /encoder 但继续读串口记录。
      **仅在无动力、无导航运动源条件下执行**;观察 EKF 是否继续预测/AMCL 表现,
      不预设 EKF 必然停止发布(如实记录)。

请求节奏保持厂家: 每 0.02s 发两条请求(速度 0x0003 + 圈数 0x0000)各读 7 字节,
不为对照改变下位机被询方式(实际周期是否吻合入阶段 A 实测表)。

⚠️ 与厂家节点的已知行为差异(不得宣称完全等价;阶段 A 须实测六项后再说"近似"):
  D1 读取超时:   tap timeout=0.02(可返回部分/空字节); 厂家无 timeout,read 永阻塞等满 7 字节
  D2 异常处理:   tap 捕获 IndexError 继续运行并记录; 厂家同情形节点抛异常退出
  D3 空响应:     tap 不发布(无数据≠零速,用户裁定保留); 厂家不遇此情形(阻塞无输出)
  D4 异常响应帧: tap 合法 0x83 帧不发布(记码); 厂家 A 版 DueVelData 找不到 01 03 02 → 返回 None
                 → `3.57*None` 抛 TypeError → 厂家节点崩溃(无 try/except,tap 捕获留痕)。
                 坏 CRC 的 0x83 形态字节归垃圾类,走正常双解析: legacy 同样 None→TypeError 留痕,
                 fixed 按 bad 拒帧——两路都不造数(与 B 版"静默 0.0"不同)。
  D5 发布时间:   tap 循环自由跑+sleep 补齐 vs 厂家 rclpy timer 0.02s 回调——发布频率待实测
  D6 字节拼接:   两者都不 flush 输入缓冲(残留拼接行为保留,便于复现与取证)
  阶段 A 实测比较项(入表): 请求周期/响应拼接/单位/符号/比例/发布频率。

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
    counts_to_velocity, detect_exception_frame, REQUEST_VEL, REQUEST_DIR,
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
        self.n_exc = 0            # 合法设备异常响应(CRC 过的 01 83 帧)
        self.last_exc_code = None
        self.last_fixed_vel = None

    def cycle_once(self, ser, t_mono):
        """一个完整厂家周期(双请求)。返回 record dict(始终返回,坏周期也记)。"""
        rec = {'t': t_mono, 'req_v': REQUEST_VEL.hex(), 'resp_v': '',
               'req_d': REQUEST_DIR.hex(), 'resp_d': '',
               'frame_valid': 'empty', 'legacy_vel': None, 'fixed_vel': None,
               'published': 'none', 'note': ''}

        ser.write(REQUEST_VEL)
        resp_v = ser.read(7)
        rec['resp_v'] = resp_v.hex() if resp_v else ''

        ser.write(REQUEST_DIR)
        resp_d = ser.read(7)
        rec['resp_d'] = resp_d.hex() if resp_d else ''

        # 异常响应分类(任务书第3条): 合法 0x83 帧不进速度输出,记码不发布(D4 差异)
        is_exc, exc_code, exc_crc_ok = detect_exception_frame(resp_v)
        if is_exc and exc_crc_ok:
            self.n_exc += 1
            self.last_exc_code = exc_code
            rec['frame_valid'] = 'exc'             # 帧CRC有效但是设备异常响应
            rec['note'] = f'exc=0x{exc_code:02X}'
            return rec
        if is_exc:
            rec['note'] = 'exc_badcrc'             # 0x83 形态但 CRC 不过 → 走正常双解析

        # 旧解析(厂家语义): 异常捕获留痕,不崩溃。
        # ⚠️ 空响应(超时)绝不当零速发布——A 版 DueVelData(b'') 返回 None(非 0.0),
        # legacy_velocity 对 None 抛 TypeError 留痕;厂家节点因 read 永阻塞从不遇此情形;
        # tap 带 timeout,此处 `if resp_v:` 显式不发布(任务书第7条)。
        if resp_v:
            try:
                rec['legacy_vel'] = legacy_velocity(resp_v, k=self.k)
                self.n_ok += 1
            except Exception as e:                 # noqa: BLE001
                rec['note'] += f'legacy_{type(e).__name__}'
                self.n_legacy_err += 1
        else:
            rec['note'] += 'timeout'

        # 新解析(旁路): 同一份字节喂 CRC 提取器
        self.ex.feed(resp_v)
        frames = self.ex.extract()
        if frames:
            raw = decode_first_register(frames[0])
            rec['frame_valid'] = 'valid'           # CRC 通过的正常数据帧
            rec['fixed_vel'] = counts_to_velocity(
                to_signed16(raw), cycle_s=0.02, k=self.k)
            rec['note'] += f' raw=0x{raw:04X}'
            self.last_fixed_vel = rec['fixed_vel']
        else:
            if resp_v:
                rec['frame_valid'] = 'bad'         # 有字节但CRC不过=坏帧/垃圾
                self.n_fixed_reject += 1
                rec['note'] += ' fixed_reject'
            # resp_v 为空保持 'empty'
        return rec


CSV_HEADER = ['monotonic_s', 'req_v', 'resp_v', 'req_d', 'resp_d',
              'frame_valid', 'legacy_vel', 'fixed_vel', 'published', 'note']


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
            # 实际是否发布 /encoder(补充口径: 与解析值分列,分析时可辨别)
            if not publishing:
                pub_kind = 'stopped'
            elif args.publish == 'legacy' and rec['legacy_vel'] is not None:
                pub_kind = 'legacy'
            elif args.publish == 'fixed' and rec['fixed_vel'] is not None:
                pub_kind = 'fixed'
            else:
                pub_kind = 'none'
            rec['published'] = pub_kind
            w_csv.writerow([f"{rec['t']:.6f}", rec['req_v'], rec['resp_v'],
                            rec['req_d'], rec['resp_d'], rec['frame_valid'],
                            '' if rec['legacy_vel'] is None else f"{rec['legacy_vel']:.6f}",
                            '' if rec['fixed_vel'] is None else f"{rec['fixed_vel']:.6f}",
                            pub_kind, rec['note']])
            n_rec += 1
            if pub_kind == 'legacy':
                msg.header.stamp = n.get_clock().now().to_msg()
                msg.twist.twist.linear.x = rec['legacy_vel']
                pub_legacy.publish(msg)
            elif pub_kind == 'fixed':
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
                    f"fixed_reject={w.n_fixed_reject} exc={w.n_exc}"
                    + (f" last_exc=0x{w.last_exc_code:02X}" if w.last_exc_code is not None else ''))
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
            f'fixed_reject={w.n_fixed_reject} exc={w.n_exc} csv={csv_path}')


if __name__ == '__main__':
    main()
