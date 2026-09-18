#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_encoder_proto.py —— 解析库全场景单测(2026-09-18)

跑法(零依赖,任何 python3): python3 test_encoder_proto.py
覆盖任务书点名场景: 分段到达/连续多帧/前置垃圾;错长度/坏校验/丢字节/插入字节;
重同步/串口重连/计数复位和回绕;正常前进/倒车及不同有效采样间隔;
坏数据不产生虚假速度或位姿增量。
全过输出 ALL PASS + 用例数;任何失败 assert 直接抛栈并退出非零。
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from encoder_proto import (  # noqa: E402
    ModbusRtuExtractor, crc16_modbus, frame_crc_valid, make_response,
    REQUEST_VEL, REQUEST_DIR, decode_first_register, to_signed16,
    counts_to_velocity,
)

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def vel_frame(value_raw):
    """构造一条速度寄存器响应帧(值=16位原始计数)。"""
    return make_response(0x01, 2, bytes([(value_raw >> 8) & 0xFF, value_raw & 0xFF]))


# ---------- 一、协议基线(黄金向量:逆向实证的请求帧 CRC) ----------

@case
def crc_golden_requests():
    # 现役 encoder_node.py 硬编码请求帧,帧尾应精确复算为 74 0A / 84 0A
    assert REQUEST_VEL == bytes.fromhex('01 03 00 03 00 01 74 0A'), REQUEST_VEL.hex()
    assert REQUEST_DIR == bytes.fromhex('01 03 00 00 00 01 84 0A'), REQUEST_DIR.hex()
    assert frame_crc_valid(REQUEST_VEL) and frame_crc_valid(REQUEST_DIR)


@case
def crc_golden_response():
    # 已知向量: 响应(值=100) = 01 03 02 00 64 B9 AF
    f = make_response(0x01, 2, bytes([0x00, 0x64]))
    assert f == bytes.fromhex('01 03 02 00 64 B9 AF'), f.hex()
    assert frame_crc_valid(f)


@case
def crc_reference_value():
    # CRC16-MODBUS 标准测试向量 "123456789" = 0x4B37
    assert crc16_modbus(b'123456789') == 0x4B37


# ---------- 二、提取器: 正常流 ----------

@case
def single_frame():
    ex = ModbusRtuExtractor()
    ex.feed(vel_frame(100))
    out = ex.extract()
    assert len(out) == 1 and decode_first_register(out[0]) == 100


@case
def segmented_arrival():
    """分段到达: 帧拆成 1+3+3 字节分三次喂,仍要完整解出。"""
    f = vel_frame(0x1234)
    ex = ModbusRtuExtractor()
    ex.feed(f[:1]); assert ex.extract() == []
    ex.feed(f[1:4]); assert ex.extract() == []
    ex.feed(f[4:])
    out = ex.extract()
    assert len(out) == 1 and decode_first_register(out[0]) == 0x1234


@case
def segmented_mid_header_boundary():
    """分段到达(帧头边界): 只到 01 / 01 03 / 01 03 02 三段,均不得丢部分帧头。"""
    f = vel_frame(200)
    ex = ModbusRtuExtractor()
    ex.feed(f[:1]); assert ex.extract() == [] and len(ex.buf) == 1
    ex.feed(f[1:2]); assert ex.extract() == [] and len(ex.buf) == 2
    ex.feed(f[2:3]); assert ex.extract() == [] and len(ex.buf) == 3
    ex.feed(f[3:])
    out = ex.extract()
    assert len(out) == 1 and decode_first_register(out[0]) == 200


@case
def consecutive_multi_frame():
    """连续多帧: 两帧背靠背一次喂入,按序各出一帧。"""
    ex = ModbusRtuExtractor()
    ex.feed(vel_frame(10) + vel_frame(20))
    out = ex.extract()
    assert len(out) == 2
    assert decode_first_register(out[0]) == 10 and decode_first_register(out[1]) == 20


@case
def multi_register_frame():
    """byte_count=4(两寄存器)合法帧: 全长 9 字节,取第一个寄存器。"""
    f = make_response(0x01, 4, bytes([0x03, 0xE8, 0x00, 0x64]))
    ex = ModbusRtuExtractor()
    ex.feed(f)
    out = ex.extract()
    assert len(out) == 1 and len(out[0]) == 9 and decode_first_register(out[0]) == 1000


# ---------- 三、提取器: 坏流(核心安全场景) ----------

@case
def leading_garbage():
    """前置垃圾: 帧前混入噪声,帧仍要解出,垃圾计入 dropped_bytes。"""
    ex = ModbusRtuExtractor()
    ex.feed(b'\x00\xff\x55\x01' + vel_frame(50))
    out = ex.extract()
    assert len(out) == 1 and decode_first_register(out[0]) == 50
    assert ex.dropped_bytes == 4, ex.dropped_bytes


@case
def bad_crc_one_bit():
    """坏校验: 数据区翻 1 bit → 该帧被拒。"""
    f = bytearray(vel_frame(100))
    f[4] ^= 0x01                       # data_low 翻最低位
    ex = ModbusRtuExtractor()
    ex.feed(bytes(f))
    assert ex.extract() == []
    assert ex.rejected_frames >= 1


@case
def bad_crc_flip_crc_byte():
    """坏校验: 直接改 CRC 尾字节 → 被拒。"""
    f = bytearray(vel_frame(100))
    f[-1] ^= 0xFF
    ex = ModbusRtuExtractor()
    ex.feed(bytes(f))
    assert ex.extract() == []


@case
def lost_byte_in_frame():
    """丢字节: 帧中删 1 字节 → 原 7 字节帧无法整帧通过(CRC 错位)。
    关键断言=不得输出"原帧位置的假帧"(值可能恰好等于真值的情形不存在)。"""
    f = vel_frame(0xABCD)
    ex = ModbusRtuExtractor()
    ex.feed(f[:3] + f[4:])             # 删第4字节(data_hi)
    out = ex.extract()
    for fr in out:
        assert not (decode_first_register(fr) == 0xABCD and len(fr) == 7)


@case
def inserted_byte_in_frame():
    """插入字节: 帧中插 1 字节 → 原帧被破坏,只能重同步后再解后续帧。"""
    f = vel_frame(300)
    ex = ModbusRtuExtractor()
    ex.feed(f[:4] + b'\x77' + f[4:])
    out = ex.extract()
    for fr in out:
        # 任何恰好"通过 CRC 的错位组合"都不应等于真值 300 且帧长 7
        assert not (len(fr) == 7 and decode_first_register(fr) == 300)


@case
def wrong_byte_count():
    """错长度: bc=0 / bc=奇数 / bc=252(>250) 均不是候选帧头,不崩、滑窗跳过。"""
    for bc in (0x00, 0x03, 0xFC):
        ex = ModbusRtuExtractor()
        ex.feed(bytes([0x01, 0x03, bc]) + vel_frame(100))
        out = ex.extract()
        assert len(out) == 1 and decode_first_register(out[0]) == 100, f'bc={bc}'


@case
def resync_after_garbage_burst():
    """重同步: 30 字节随机垃圾后恢复正确帧流,第一真帧即可解出。"""
    ex = ModbusRtuExtractor()
    ex.feed(b'\xde\xad\xbe\xef' * 8)
    ex.feed(vel_frame(42) + vel_frame(43))
    out = ex.extract()
    assert [decode_first_register(x) for x in out] == [42, 43]
    assert ex.dropped_bytes == 32


@case
def resync_after_corrupt_stream():
    """重同步(损坏流变体): 半截帧+垃圾+新帧。"""
    ex = ModbusRtuExtractor()
    ex.feed(vel_frame(1)[:5])          # 半截帧
    ex.feed(b'\x01\x03\x02')           # 假帧头(将误导长度)
    ex.feed(vel_frame(7))
    out = ex.extract()
    assert [decode_first_register(x) for x in out] == [7]


@case
def reconnect_fresh_extractor():
    """串口重连: 重连=新提取器实例(缓冲清零),历史垃圾不污染新会话。"""
    ex1 = ModbusRtuExtractor()
    ex1.feed(b'\xff' * 10)
    ex2 = ModbusRtuExtractor()          # 模拟重连后重建
    ex2.feed(vel_frame(88))
    out = ex2.extract()
    assert len(out) == 1 and decode_first_register(out[0]) == 88
    assert ex2.dropped_bytes == 0 and ex2.buf == bytearray()


@case
def buffer_cap():
    """缓冲上限: 纯垃圾流不得无限积压,超限裁尾。"""
    ex = ModbusRtuExtractor(max_buffer=64)
    ex.feed(b'\x00' * 200)
    ex.extract()
    assert len(ex.buf) <= 64
    assert ex.dropped_bytes >= 136


@case
def pure_garbage_no_output():
    """坏数据不产生虚假速度: 纯噪声 1KB 输出零帧(无虚假速度→无位姿增量)。"""
    ex = ModbusRtuExtractor()
    ex.feed(bytes(range(256)) * 4)     # 含 0x01/0x03 字节但 CRC 全不过
    assert ex.extract() == []


@case
def all_ones_stream():
    """坏数据变体: 全 0xFF / 全 0x00 流零输出。"""
    for blob in (b'\xff' * 512, b'\x00' * 512):
        ex = ModbusRtuExtractor()
        ex.feed(blob)
        assert ex.extract() == []


@case
def phantom_speed_rate():
    """虚假速度概率论证(统计): 10^4 随机 7 字节窗口通过 CRC 的比率≈1/65536。
    用固定种子保证可复现;只验证<<1%,不追求精确。"""
    import random
    rnd = random.Random(20260918)
    ex = ModbusRtuExtractor()
    hits = 0
    blob = bytes(rnd.getrandbits(8) for _ in range(7 * 10000))
    ex.feed(blob)
    hits = len(ex.extract())
    assert hits < 100, hits            # 期望~1.5, 宽松上限 1%


# ---------- 四、解码与换算 ----------

@case
def decode_big_endian():
    f = make_response(0x01, 2, bytes([0x12, 0x34]))
    assert decode_first_register(f) == 0x1234   # 大端(与现役 hi*256+lo 一致)


@case
def signed16_wraparound():
    """计数回绕: 补码边界 0x7FFF/0x8000/0xFFFF 与复位 0x0000。"""
    assert to_signed16(0x0000) == 0
    assert to_signed16(0x0001) == 1
    assert to_signed16(0x7FFF) == 32767
    assert to_signed16(0x8000) == -32768
    assert to_signed16(0xFFFF) == -1


@case
def counter_reset_recovers():
    """计数复位: 大计数→复位到 0,解码立即反映(帧层无状态,天然恢复)。"""
    ex = ModbusRtuExtractor()
    ex.feed(vel_frame(3000) + vel_frame(0))
    out = ex.extract()
    assert [decode_first_register(x) for x in out] == [3000, 0]


@case
def velocity_forward_reverse():
    """正常前进/倒车: 正负计数 → 正负速度,数值与现役公式一致。"""
    v_fwd = counts_to_velocity(to_signed16(60))      # 60 计数/0.02s
    v_rev = counts_to_velocity(to_signed16(0xFFC4))  # -60
    # 现役公式直算对照: 3.57*60*0.22745/1024/0.02*1.0*0.25*1.03
    expect = 3.57 * 60 * (227.45 / 1000) / 1024.0 / 0.02 * 1.0 * 0.25 * 1.03
    assert abs(v_fwd - expect) < 1e-12, (v_fwd, expect)
    assert abs(v_rev + expect) < 1e-12


@case
def velocity_sampling_intervals():
    """不同有效采样间隔: cycle_s 减半 → 速度加倍(积分不变,位姿不自爆)。"""
    v1 = counts_to_velocity(60, cycle_s=0.02)
    v2 = counts_to_velocity(60, cycle_s=0.04)
    assert abs(v2 * 2 - v1 * 1) < 1e-12
    # 同一物理速度,周期减半则每周期计数也应减半 → 速度读数一致
    assert abs(counts_to_velocity(30, cycle_s=0.01) - v1) < 1e-12


@case
def velocity_scale_k():
    """标定系数 k: 线性作用于速度(留标定接口,默认 1.0=现役)。"""
    assert abs(counts_to_velocity(60, k=1.04) - counts_to_velocity(60) * 1.04) < 1e-12


# ---------- 五、节点逻辑核(EncoderWorker,无 rclpy 依赖) ----------

from encoder_node_fixed import EncoderWorker, DueVelData, legacy_velocity  # noqa: E402


class FakeSerial:
    """离线假串口: 脚本化 read() 返回序列,可注入垃圾/超时/坏帧。"""

    def __init__(self, responses):
        self.responses = list(responses)      # 每次 read() 弹一条(bytes, b''=超时)
        self.written = []

    def write(self, data):
        self.written.append(bytes(data))

    def read(self, n):
        return self.responses.pop(0) if self.responses else b''

    def reset_input_buffer(self):
        pass


@case
def worker_clean_stream_matches_legacy():
    """A/B 对照(干净流): 新旧解析器对合法帧输出一致(修复不改变正常语义)。"""
    resp = vel_frame(60)
    fs = FakeSerial([resp])
    w = EncoderWorker()
    v_new = w.read_velocity_once(fs)
    assert v_new is not None
    assert abs(v_new - legacy_velocity(resp)) < 1e-12, (v_new, legacy_velocity(resp))


@case
def worker_timeout_returns_none():
    """串口无响应: read 得 b'' → None + timeout 计数,绝不造数。"""
    fs = FakeSerial([b'', b'', b''])
    w = EncoderWorker()
    assert w.read_velocity_once(fs) is None
    assert w.read_velocity_once(fs) is None
    assert w.stats['timeout'] == 2 and w.stats['ok'] == 0


@case
def worker_bad_crc_returns_none():
    """坏 CRC 响应: 有字节但帧不合法 → None(旧解析器此处会出鬼值)。"""
    bad = bytearray(vel_frame(100))
    bad[3] ^= 0x02                               # data_hi 翻位
    fs = FakeSerial([bytes(bad)])
    w = EncoderWorker()
    assert w.read_velocity_once(fs) is None
    assert w.stats['crc_drop'] == 1 and w.stats['ok'] == 0


@case
def worker_phantom_suppression_vs_legacy():
    """0918 碰撞场景实验室复现: 同一段毒字节,
    旧 DueVelData 解出 ~5m/s 鬼速度;新 worker 输出 None(不发布→EKF 无假里程)。"""
    # 垃圾里嵌 01 03 02 + 大计数: 旧状态机会命中并解出 10000 计数
    poison = bytes.fromhex('FF 01 03 02 27 10 FF')
    legacy_v = legacy_velocity(poison)
    assert legacy_v > 5.0, legacy_v               # 旧解析器确实产生鬼速度(>5m/s)
    fs = FakeSerial([poison])
    w = EncoderWorker()
    assert w.read_velocity_once(fs) is None       # 新核拒绝同一段字节
    assert w.stats['ok'] == 0


@case
def worker_recovers_after_poison():
    """毒字节后恢复: 下一周期合法帧照常解出(重不影响后续)。"""
    poison_then_good = [bytes.fromhex('FF 01 03 02 27 10 FF'), vel_frame(60)]
    fs = FakeSerial(poison_then_good)
    w = EncoderWorker()
    assert w.read_velocity_once(fs) is None
    v = w.read_velocity_once(fs)
    assert v is not None and abs(v - legacy_velocity(vel_frame(60))) < 1e-12
    assert w.stats['ok'] == 1


@case
def worker_residual_then_frame():
    """残留+新帧同窗: 上一周期迟到垃圾与本周期真帧同到,只出真帧。"""
    fs = FakeSerial([bytes.fromhex('AA BB CC') + vel_frame(90)])
    w = EncoderWorker()
    v = w.read_velocity_once(fs)
    assert v is not None
    assert w.stats['last_raw'] == 90 and w.ex.dropped_bytes == 3


@case
def worker_direction_register():
    """圈数寄存器读取(诊断开关): 可读出且不进速度公式。"""
    fs = FakeSerial([vel_frame(10), make_response(0x01, 2, bytes([0x30, 0x39]))])
    w = EncoderWorker()
    v_before = w.read_velocity_once(fs)
    raw_dir = w.read_direction_once(fs)
    assert raw_dir == 12345                       # 0x3039
    assert w.stats['vel'] == v_before             # 读圈数不干扰速度状态


@case
def worker_measured_dt_off_by_default():
    """周期口径: 默认固定 0.02(保 0917 标定),开 use_measured_dt 才用实测。"""
    import time as _t
    fs = FakeSerial([vel_frame(60)])
    w_fixed = EncoderWorker()
    v_fixed = w_fixed.read_velocity_once(fs)
    fs2 = FakeSerial([vel_frame(60)])
    w_dt = EncoderWorker(use_measured_dt=True)
    w_dt._t_cycle = _t.time() - 0.04              # 假装实测周期 40ms
    v_dt = w_dt.read_velocity_once(fs2)
    assert abs(v_dt * 2 - v_fixed) < 1e-3         # dt 翻倍→速度减半(容实时抖动)


# ---------- 六、tap 唯一读取者(TapWorker,无 rclpy 依赖) ----------

from encoder_tap import TapWorker, CSV_HEADER  # noqa: E402


def tap_responses(vel_raw, dir_raw=0x3039):
    """一次 tap 周期 = 速度响应+圈数响应两条(FakeSerial 按序弹)。"""
    return [vel_frame(vel_raw), make_response(0x01, 2, bytes(
        [(dir_raw >> 8) & 0xFF, dir_raw & 0xFF]))]


@case
def tap_clean_cycle_dual_parser_agree():
    """干净流: legacy 与 fixed 同一份字节解出同值(顶替语义等价性)。"""
    fs = FakeSerial(tap_responses(60))
    w = TapWorker()
    rec = w.cycle_once(fs, t_mono=1.0)
    assert rec['legacy_vel'] is not None and rec['fixed_vel'] is not None
    assert abs(rec['legacy_vel'] - rec['fixed_vel']) < 1e-12
    assert 'raw=0x003C' in rec['note']
    assert rec['req_v'] == '010300030001740a'      # 请求原样入档


@case
def tap_poison_cycle_matrix():
    """毒字节周期(0918 场景): legacy 出鬼值(照记), fixed 拒(fixed_reject)——
    A/B 证据矩阵在同一行 CSV 里同时可见。"""
    fs = FakeSerial([bytes.fromhex('FF 01 03 02 27 10 FF'), vel_frame(0)])
    w = TapWorker()
    rec = w.cycle_once(fs, t_mono=2.0)
    assert rec['legacy_vel'] > 5.0                  # 鬼值被记录(不是被隐藏)
    assert rec['fixed_vel'] is None and 'fixed_reject' in rec['note']
    assert w.n_fixed_reject == 1


@case
def tap_legacy_indexerror_contained():
    """厂家 DueVelData 越界缺陷防护: 触发字节模式下捕获 IndexError 不崩溃,
    note 留痕 legacy_IndexError,该周期不发布。"""
    # 触发构造: 位置3起 01 03 02 → data_high=[6] 可读, data_low=[7] 越界
    fs = FakeSerial([bytes.fromhex('00 00 00 01 03 02 FF'), vel_frame(0)])
    w = TapWorker()
    rec = w.cycle_once(fs, t_mono=3.0)
    assert 'legacy_IndexError' in rec['note'], rec['note']
    assert rec['legacy_vel'] is None
    assert w.n_legacy_err == 1


@case
def tap_timeout_cycle_recorded():
    """无响应周期: 空字节照记(resp 空), 双解析 None 不发布——
    特别验证 DueVelData(b'')=0.0 的坑被防住(无数据≠新鲜零速度)。"""
    fs = FakeSerial([b'', b''])
    w = TapWorker()
    rec = w.cycle_once(fs, t_mono=4.0)
    assert rec['resp_v'] == '' and rec['resp_d'] == ''
    assert rec['legacy_vel'] is None and rec['fixed_vel'] is None
    assert 'timeout' in rec['note']
    assert w.n_fixed_reject == 0                    # 空≠坏(不计拒帧)
    # DueVelData(b'') 数学上返回 0.0(厂家无 timeout 永不触发此路径,tap 须防)
    assert DueVelData(b'') == 0.0


@case
def tap_mixed_stream_stats():
    """混合流统计: 好/毒/超时混合周期,计数器三分离(ok/legacy_err/fixed_reject)。"""
    fs = FakeSerial(tap_responses(10)
                    + [bytes.fromhex('FF 01 03 02 27 10 FF'), vel_frame(0)]
                    + [b'', b'']
                    + tap_responses(20))
    w = TapWorker()
    recs = [w.cycle_once(fs, t_mono=float(i)) for i in range(3)]
    assert len(recs) == 3
    assert w.n_ok == 2 and w.n_fixed_reject == 1
    vals = [(r['legacy_vel'], r['fixed_vel']) for r in recs]
    assert vals[0][0] is not None and vals[1][0] > 5.0 and vals[2][0] is None


def main():
    failed = 0
    for fn in CASES:
        try:
            fn()
            print(f'  PASS  {fn.__name__}')
        except AssertionError as e:
            failed += 1
            print(f'  FAIL  {fn.__name__}: {e!r}')
        except Exception as e:                     # noqa: BLE001
            failed += 1
            print(f'  ERROR {fn.__name__}: {type(e).__name__}: {e}')
    print(f'\n{len(CASES) - failed}/{len(CASES)} PASS' + ('  ** ALL PASS **' if failed == 0 else '  ** 有失败 **'))
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
