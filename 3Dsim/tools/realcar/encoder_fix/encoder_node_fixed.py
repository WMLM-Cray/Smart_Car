#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""encoder_node_fixed.py —— 编码器节点·帧校验修复版(2026-09-18 离线写,默认不部署)

背景: 0918 发车碰撞根案=厂家 encoder_node.py 的 DueVelData 无校验三字节状态机,
串口错位时解出任意速度→EKF 假里程→AMCL 位姿偏→路径撞实物。本文件是厂家节点的
对照替代实现: 协议解析换成 encoder_proto.py(CRC16 校验+滑窗重同步),其余语义
(话题消息/周期/公式常数)与现役逐字对齐,可独立运行做 A/B 对照。

⚠️ 部署边界(用户红线):
  - 默认话题 /encoder_fixed —— 不占用厂家 /encoder,不与厂家节点并跑同一输出;
  - 切换=停厂家 encoder 节点后 remap 本节点→/encoder(或 EKF odom0 指向 /encoder_fixed),
    回滚=反向一步,厂家 ~/racecar 一字未动;
  - 本文件未上车验证,上车前须先按 PROTOCOL_BASIS.md §四采原始字节终验。

与现役的四处差异(全部可回退):
  1. serial 加 timeout=0.02(现役无 timeout,read 永阻);
  2. 每周期 write 前 reset_input_buffer()(现役不冲缓冲,残留错位是嫌疑根因);
     代价=迟到响应被弃(本就是上一周期的旧值,弃更干净),偶发超时=EKF 少一次观测;
  3. 只认 CRC 通过的帧才发布(现役无校验直接解);
  4. 默认只读速度寄存器 0x0003(现役每周期还读 0x0000 圈数但返回值弃用=死代码);
     --read-direction 可开圈数读取进诊断(用于上车验证符号语义)。

周期除数口径: 速度公式分母保持厂家固定 0.02s(0917 实车 1m 标定 1.042/1.035 基于
该语义,换实测 dt 会改标度);实测周期进诊断供分析,参数 use_measured_dt 默认关。
"""
import sys
import time
import argparse

from encoder_proto import (
    ModbusRtuExtractor, decode_first_register, to_signed16,
    counts_to_velocity, REQUEST_VEL, REQUEST_DIR,
)

try:  # 车机长期运行防 SIGINT 自接管(同 cmd_vel_safety/gate 模式)
    from rclpy.signals import uninstall_signal_handlers
    uninstall_signal_handlers()
except Exception:  # 离线环境无 rclpy,纯逻辑照常可用
    pass


class EncoderWorker:
    """串口周期读取+CRC 解析的纯逻辑核(不依赖 rclpy/serial,可离线单测)。
    ser 只需提供 write/read/reset_input_buffer 三接口(serial.Serial 或 Fake 均可)。
    """

    def __init__(self, cycle_s=0.02, k=1.0, use_measured_dt=False,
                 read_direction=False):
        self.cycle_s = cycle_s
        self.k = k
        self.use_measured_dt = use_measured_dt
        self.read_direction = read_direction
        self.ex = ModbusRtuExtractor()
        self.stats = {'ok': 0, 'timeout': 0, 'crc_drop': 0, 'dir_ok': 0,
                      'last_raw': None, 'last_dir_raw': None, 'vel': None}

    def read_velocity_once(self, ser):
        """一个请求-响应周期。返回 (vel or None)。坏响应一律 None,绝不造数。"""
        try:
            ser.reset_input_buffer()
        except Exception:                       # 假串口/权限异常不致命
            pass
        ser.write(REQUEST_VEL)
        data = ser.read(7)
        if not data:
            self.stats['timeout'] += 1
            return None
        self.ex.feed(data)
        frames = self.ex.extract()
        if not frames:
            self.stats['crc_drop'] += 1        # 有字节但无 CRC 合法帧
            return None
        raw = decode_first_register(frames[0])
        self.stats['last_raw'] = raw
        dt = time.time() - self._t_cycle if self.use_measured_dt else self.cycle_s
        vel = counts_to_velocity(to_signed16(raw), cycle_s=dt, k=self.k)
        self.stats['ok'] += 1
        self.stats['vel'] = vel
        return vel

    def read_direction_once(self, ser):
        """读圈数寄存器(诊断用,不进速度公式)。返回原始计数 or None。"""
        ser.write(REQUEST_DIR)
        data = ser.read(7)
        if not data:
            return None
        self.ex.feed(data)
        frames = self.ex.extract()
        if not frames:
            return None
        raw = decode_first_register(frames[0])
        self.stats['dir_ok'] += 1
        self.stats['last_dir_raw'] = raw
        return raw

    _t_cycle = None                             # 主循环里刷新,供 use_measured_dt

    def diag_line(self):
        s = self.stats
        return (f"ok={s['ok']} timeout={s['timeout']} crc_drop={s['crc_drop']} "
                f"dropped_bytes={self.ex.dropped_bytes} rejected={self.ex.rejected_frames} "
                f"last_raw={s['last_raw']} vel={s['vel']}")


# ---- 厂家旧解析器逐字保留(legacy 对照用;勿改其语义) ----
# 源 = 厂商包 racecar_05_27.tar.gz → src/encoder/encoder/encoder_node.py 的 DueVelData
# (与 install/、历史 AMCL 副本、阶段0现役车端、车端当时运行安装文件逐字节一致,
#   md5 30b19137… / sha256 999242c3…;见 PARSER_SOURCE_PROVENANCE.md)。
# ⚠️ 2026-09-19 修复: 前版(索引搜索版)对单字节 data_high 判 >=32768 恒假,
#   负速度被解成正大数,已隔离为下方 DueVelData_index_buggy。本 DueVelData 恢复为
#   厂家原版逐字节遍历状态机: 对拼接 16 位 Angle_vel 判符号,符号正确;
#   无完整 01 03 02 序列时隐式返回 None(厂家原版即如此,非 0.0)。

Bytenum_vel = 0  # 厂家原版全局状态机(跨调用保留,是 0918 错位污染的根因之一,勿"修掉")


def DueVelData(inputdata):  # noqa: N802 (厂家原名)
    """厂家原版逐字节遍历状态机,仅作 legacy 对照,不用于发布。
    符号判断对拼接 16 位 Angle_vel(正确);无匹配返回 None(非 0.0)。"""
    global Bytenum_vel

    for data in inputdata:
        if data == 0x01 and Bytenum_vel == 0:
            Bytenum_vel = 1
            continue
        if data == 0x03 and Bytenum_vel == 1:
            Bytenum_vel = 2
            continue
        if data == 0x02 and Bytenum_vel == 2:
            Bytenum_vel = 3
            continue
        if Bytenum_vel == 3:
            data_high = data
            Bytenum_vel = 4
            continue
        if Bytenum_vel == 4:
            data_low = data
            Bytenum_vel = 0
            Angle_vel = data_high * 256 + data_low
            if Angle_vel >= 32768:
                Angle_vel -= 65536
            return float(Angle_vel)
    return None  # 厂家原版隐式返回 None(无 return);显式写出仅为了清晰,语义一致


def DueVelData_index_buggy(inputdata):  # noqa: N802
    """[已隔离,勿用于 legacy 对照] 前版"索引搜索"实现——符号缺陷:
    `if data_high >= 32768` 对单字节(0~255)恒假,负速度被解成正大数(如 0xffff→65535)。
    无匹配返回 0.0(把"找不到帧"静默伪造成零速)。保留仅供历史对照/复现
    2026-09-18 tap_push CSV 后退段 legacy 列符号错误。"""
    data_high = 0
    data_low = 0
    for i in range(len(inputdata) - 1):
        if inputdata[i] == 1 and inputdata[i + 1] == 3 and inputdata[i + 2] == 2:
            data_high = inputdata[i + 3]
            data_low = inputdata[i + 4]
            break
    if data_high >= 32768:
        data_high -= 65536
    Angle_vel = data_high * 256 + data_low
    return float(Angle_vel)


def legacy_velocity(datahex, k=1.0):
    """厂家公式原样(周期固定 0.02): 3.57*v*C/1024/0.02*k*0.25*1.03。"""
    angle_v = DueVelData(datahex)
    C = 227.45 / 1000
    return 3.57 * angle_v * C / 1024.0 / 0.02 * k * 0.25 * 1.03


def main():
    import rclpy                                     # 延迟导入:离线分析不炸
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    import serial                                    # pyserial(车机已装,厂家同款)

    ap = argparse.ArgumentParser()
    ap.add_argument('--ros-args', action='store_true', help=argparse.SUPPRESS)
    ap.add_argument('--serial-port', default='/dev/encoder')
    ap.add_argument('--baud', type=int, default=57600)
    ap.add_argument('-r', '--ros-args-remainder', dest='remainder', nargs='*')
    ns, unknown = ap.parse_known_args()

    rclpy.init()
    n = Node('encoder_node_fixed')
    topic = n.declare_parameter('out_topic', '/encoder_fixed').value
    pub = n.create_publisher(Odometry, topic, 50)
    k = n.declare_parameter('k', 1.0).value
    cycle = n.declare_parameter('cycle', 0.02).value
    use_dt = n.declare_parameter('use_measured_dt', False).value
    read_dir = n.declare_parameter('read_direction', False).value

    ser = serial.Serial(ns.serial_port, ns.baud, timeout=cycle)
    w = EncoderWorker(cycle_s=cycle, k=k, use_measured_dt=use_dt,
                      read_direction=read_dir)
    n.get_logger().info(
        f'encoder_node_fixed 起来: port={ns.serial_port} out={topic} '
        f'k={k} cycle={cycle} use_dt={use_dt} read_dir={read_dir}')

    pub_msg = Odometry()
    pub_msg.header.frame_id = ''                     # 与厂家一致(EKF 不看 frame)
    next_diag = time.time() + 10.0
    try:
        while rclpy.ok():
            w._t_cycle = time.time()
            vel = w.read_velocity_once(ser)
            if read_dir:
                w.read_direction_once(ser)
            if vel is not None:
                pub_msg.header.stamp = n.get_clock().now().to_msg()
                pub_msg.twist.twist.linear.x = vel
                pub.publish(pub_msg)
            if time.time() >= next_diag:
                n.get_logger().info(w.diag_line())
                next_diag = time.time() + 10.0
            dt_sleep = cycle - (time.time() - w._t_cycle)
            if dt_sleep > 0:
                time.sleep(dt_sleep)
    finally:
        n.get_logger().info(f'退出: {w.diag_line()}')
        ser.close()


if __name__ == '__main__':
    main()
