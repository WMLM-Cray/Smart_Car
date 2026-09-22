# -*- coding: utf-8 -*-
"""encoder_proto.py —— Modbus RTU 编码器帧解析库(纯函数,2026-09-18 离线修复)

协议依据(逆向证据链,详见同目录 PROTOCOL_BASIS.md):
  现役 encoder_node.py 周期发送两条硬编码请求:
    01 03 00 03 00 01 74 0A   读保持寄存器 0x0003(速度计数)
    01 03 00 00 00 01 84 0A   读保持寄存器 0x0000(累计圈数)
  帧尾 74 0A / 84 0A 经标准 Modbus CRC16(多项式 0xA001,初值 0xFFFF,低字节在前)
  复算完全吻合 → 下位机为 Modbus RTU 从站(地址 0x01)。
  响应帧结构: 01 03 <byte_count> <data 大端...> <crc_lo> <crc_hi>
  byte_count=2(单寄存器)时全长 7 字节,与现役 ser.read(7) 互证。
  ⚠️ 响应帧含 CRC 为协议标准推断(请求侧 CRC 已实证),响应原始字节尚未采集,
     上车切换前须先采原始字节终验(PROTOCOL_BASIS.md §四)。

与现役 DueVelData 的差别(为何修):
  现役为无校验三字节状态机(01 03 02+2 字节),CRC 两字节被无视、也不保证帧对齐;
  串口缓冲残留/错位时可从垃圾流中"解出"任意速度值 → EKF 假里程(0918 三次跳变的
  嫌疑链,时间相关性成立、字节级证据待采,见 EVIDENCE_TIMELINE.md)。
本库: 只输出 CRC 校验通过的完整帧;坏帧滑窗重同步;全程计数可诊断。
"""


def crc16_modbus(data: bytes) -> int:
    """标准 Modbus CRC16。返回值低字节在前发送(crc_lo = &0xFF, crc_hi = >>8)。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame_crc_valid(frame: bytes) -> bool:
    """frame = 地址..数据 + crc_lo + crc_hi(全长>=4)。校验尾部 CRC 是否吻合前文。"""
    if len(frame) < 4:
        return False
    body, lo, hi = frame[:-2], frame[-2], frame[-1]
    return crc16_modbus(body) == (lo | (hi << 8))


def make_response(slave: int, byte_count: int, data: bytes) -> bytes:
    """构造一条合法响应帧(测试用): slave 03 bc data... + CRC。"""
    body = bytes([slave, 0x03, byte_count]) + data
    c = crc16_modbus(body)
    return body + bytes([c & 0xFF, c >> 8])


REQUEST_VEL = bytes.fromhex('01 03 00 03 00 01 74 0A')   # 读速度寄存器(与现役一致)
REQUEST_DIR = bytes.fromhex('01 03 00 00 00 01 84 0A')   # 读圈数寄存器(现役死代码,备用)


class ModbusRtuExtractor:
    """滑窗帧提取器: feed() 喂原始字节流, extract() 吐出 CRC 通过的完整帧。

    重同步策略: 从缓冲头逐字节滑窗找候选帧头(slave,03,合理 byte_count),
    长度够则验 CRC——过则出帧并消费,不过则丢缓冲首字节继续滑。
    任意垃圾流最多浪费 len 字节即可重新锁定真帧;错位组合恰好通过 CRC 的
    概率为 1/65536(且下一帧仍会校验,持续错位输出需连续命中,概率平方衰减)。
    """

    def __init__(self, max_buffer: int = 512, slave: int = 0x01):
        self.max_buffer = max_buffer
        self.slave = slave
        self.buf = bytearray()
        self.dropped_bytes = 0   # 被滑窗丢弃的字节数(诊断:错位/垃圾量)
        self.rejected_frames = 0  # 帧长凑齐但 CRC 失败次数

    def feed(self, data: bytes) -> None:
        self.buf.extend(data)
        if len(self.buf) > self.max_buffer:            # 防无主垃圾无限增长
            self.dropped_bytes += len(self.buf) - self.max_buffer
            del self.buf[:-self.max_buffer]

    def extract(self) -> list:
        """返回本次可解出的完整合法帧列表(含 CRC 尾),按到达顺序。"""
        out = []
        while True:
            frame = self._try_one()
            if frame is None:
                break
            out.append(frame)
        return out

    def _try_one(self):
        i = 0
        while i < len(self.buf):
            if self.buf[i] != self.slave:               # 不是候选帧头
                i += 1
                continue
            if i + 2 >= len(self.buf):                  # 帧头未到齐,保留等待(勿清!)
                self.dropped_bytes += i                  # 前面的都是垃圾,丢掉
                del self.buf[:i]
                return None
            if self.buf[i + 1] != 0x03:
                i += 1
                continue
            bc = self.buf[i + 2]
            if bc < 2 or bc > 250 or bc % 2:            # 寄存器数据为偶数字节
                i += 1
                continue
            need = 3 + bc + 2
            if i + need > len(self.buf):                # 候选帧未凑齐,保留等待
                self.dropped_bytes += i
                del self.buf[:i]
                return None
            cand = bytes(self.buf[i:i + need])
            if frame_crc_valid(cand):
                del self.buf[:i + need]
                if i > 0:
                    self.dropped_bytes += i
                return cand
            self.rejected_frames += 1                   # CRC 失败,滑一字节重同步
            i += 1
        self.dropped_bytes += len(self.buf)             # 窗内已无 slave 字节,清空
        self.buf.clear()
        return None


def detect_exception_frame(data: bytes):
    """识别字节块开头的 Modbus 异常响应帧(用户任务书 0918: 异常响应分类)。

    正常读响应=01 03 bc ...;设备异常响应=01 83 <ex_code> <crc_lo> <crc_hi> 共 5 字节。
    返回 (is_exc, ex_code, crc_ok):
      is_exc=True 且 crc_ok=True  → 合法设备异常响应(设备在报错,不是线路坏)
      is_exc=True 且 crc_ok=False → 01 83 形态但 CRC 不过 → 归坏CRC/垃圾类
      is_exc=False                → 非异常帧形态
    频繁合法异常时先查请求地址/寄存器/长度/时序/设备状态,不直接定性下位机故障。
    """
    if len(data) >= 5 and data[0] == 0x01 and data[1] == 0x83:
        return True, data[2], frame_crc_valid(bytes(data[:5]))
    return False, None, False


def decode_first_register(frame: bytes) -> int:
    """合法帧 -> 第一个 16 位寄存器原始值(大端,与现役 data_high*256+data_low 一致)。"""
    bc = frame[2]
    if bc < 2:
        raise ValueError('byte_count < 2')
    return (frame[3] << 8) | frame[4]


def to_signed16(raw: int) -> int:
    """保持现役 DueVelData 语义: >=32768 视为负(补码)。
    ⚠️ 寄存器真实符号语义(有符号计数 vs 无符号+方向寄存器)未验证,上车采原始字节
    时需倒车采样核对(PROTOCOL_BASIS.md §四-3);若实为无符号,符号应由 0x0000 圈数
    增量判定,此处只需换 decode 策略,提取器/CRC 逻辑不变。"""
    return raw - 0x10000 if raw >= 0x8000 else raw


# ---- 速度换算: 常数保持现役 encoder_node.py 逐字一致(未标定,勿动) ----
_WHEEL_C = 227.45 / 1000     # C, 假定轮周长相关
_VEL_CONST = 3.57 * 0.25 * 1.03


def counts_to_velocity(signed_counts: float, cycle_s: float = 0.02, k: float = 1.0) -> float:
    """现役公式: 3.57 * v * C / 1024 / cycle * k * 0.25 * 1.03 (单位 m/s)。
    cycle_s 参数化以支持不同有效采样间隔(现役固定 0.02)。"""
    return _VEL_CONST * signed_counts * _WHEEL_C / 1024.0 / cycle_s * k
