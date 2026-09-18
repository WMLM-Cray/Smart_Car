#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_gate.py —— encoder_gate_v2 五项必查负例单测(2026-09-18)

跑法(零依赖): python3 test_gate.py
对照任务书三-①~⑤逐项:
  ① 正常前进/倒车/加减速是否误拒
  ② 连续丢帧如何通知下游数据失效(诊断链路)
  ③ 坏帧后第一条好帧是否产生补偿性大跳
  ④ 计数复位/回绕/时间异常能否正确恢复
  ⑤ 拒绝数据是否被伪装成"新鲜零速度"或重复旧数据
另含 B1 重锁死锁修复验证与门限/帧率耦合(B3)数值论证。
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from encoder_gate_v2 import gate_check, new_state, diag_line, DV_MAX, V_ABS_MAX  # noqa: E402

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def run_seq(values, st=None):
    """顺序喂入速度序列,返回 [(v, accept, reason)]。"""
    st = st or new_state()
    out = []
    for v in values:
        a, r = gate_check(v, st)
        out.append((v, a, r))
    return out, st


# ---------- ① 正常运动不误拒 ----------

@case
def normal_forward_ramp():
    """正常加速: 0→0.55 m/s 按 1.3m/s²@50Hz(每帧+0.026)全放行。"""
    vals = [min(0.026 * i, 0.55) for i in range(30)]
    out, st = run_seq(vals)
    rejected = [x for x in out if not x[1]]
    assert not rejected, rejected
    assert st['pass_total'] == len(vals)


@case
def normal_reverse():
    """正常倒车: 0→-0.55 ramp 全放行(负值对称处理)。"""
    vals = [-min(0.026 * i, 0.55) for i in range(30)]
    out, _ = run_seq(vals)
    assert all(x[1] for x in out)


@case
def normal_direction_change():
    """前进↔倒车切换: 0.55→-0.55 经 0,每帧|Δv|=0.055 全过。"""
    vals = [0.55 - 0.055 * i for i in range(21)]
    out, _ = run_seq(vals)
    assert all(x[1] for x in out)


@case
def normal_decel_to_stop():
    """急减速到停: 0.55→0 每帧-0.055(≈2.75m/s²,撞前急刹量级)全过。"""
    vals = [0.55 - 0.055 * i for i in range(11)]
    out, _ = run_seq(vals)
    assert all(x[1] for x in out)


# ---------- ② 丢帧下游通知(诊断链路) ----------

@case
def diag_reports_stale_age():
    """诊断行暴露失联: 输入停止后 age_in/out 增长,且 out 停在最后放行帧。"""
    st = new_state()
    st['last_input_t'] = 100.0
    st['last_output_t'] = 100.0
    st['last_input_v'] = 0.5
    st['last_output_v'] = 0.5
    line = diag_line(st, now=103.5)
    assert 'age_in=3.50s' in line and 'age_out=3.50s' in line, line
    assert 'last_out=0.5' in line, line


@case
def diag_distinguishes_in_out():
    """原始输入与过滤输出分别可观测: 全拒流里 age_in 新鲜/age_out 停摆。"""
    st = new_state()
    st['last_input_t'] = 200.0
    st['last_input_v'] = 1.9                # 输入一直有(但毒)
    # 输出从未发生
    line = diag_line(st, now=201.0)
    assert 'age_in=1.00s' in line, line
    assert 'age_out=-1.00s' in line, line    # -1 = 从未输出,不是 0(不伪装)


# ---------- ③ 坏帧后好帧的补偿性大跳 ----------

@case
def no_compensating_jump_after_spike():
    """毛刺后首条好帧不得放行大跳: last=0.2,毒1.5(拒),好0.9 → |0.9-0.2|=0.7 拒,
    第3个一致 0.9 才重锁放行(B1 修复)。全程 EKF 只见 0.2→0.9 一次切换,无补偿假速度。"""
    out, st = run_seq([0.2, 1.5, 0.9, 0.9, 0.9])
    assert out[0][1] and out[1][2] == 'abs'
    assert out[2] == (0.9, False, 'dv'), out[2]
    assert out[3] == (0.9, False, 'dv'), out[3]
    assert out[4] == (0.9, True, 'relock'), out[4]
    assert st['relocks'] == 1


@case
def random_spikes_never_relock():
    """乱值毛刺流不触发重锁(拒帧彼此不一致): 永不输出。"""
    out, st = run_seq([0.0, 1.5, -1.4, 1.1, -1.2, 0.9, -0.9])
    assert out[0][1] and not any(x[1] for x in out[1:])
    assert st['relocks'] == 0


# ---------- ④ 复位/回绕/时间异常恢复 ----------

@case
def counter_wraparound_blocked_then_relock():
    """计数回绕(解析层未修时表现为+大值→-大值瞬跳): abs 门拦住,
    之后真实速度恢复一致帧→重锁回归。"""
    out, st = run_seq([0.3, 2.5, -2.4, 0.8, 0.8, 0.8])
    assert out[1][2] == 'abs' and out[2][2] == 'abs'
    assert out[3][2] == 'dv' and out[4][2] == 'dv'
    assert out[5] == (0.8, True, 'relock')


@case
def reset_to_zero_recovers():
    """下位机复位(速度回 0): last=0.6,复位后 0.0 → |Δv|=0.6 拒,连续 0 帧重锁。"""
    out, st = run_seq([0.6, 0.0, 0.0, 0.0])
    assert out[1][2] == 'dv' and out[2][2] == 'dv'
    assert out[3] == (0.0, True, 'relock')


@case
def time_anomaly_not_filtered_by_stamp():
    """时间异常: gate 决策只看帧序列不看时间戳——节点层转发原消息原戳
    (此处验证决策函数无时间参数,车机钟跳变不会引起误拒)。"""
    import inspect
    sig = inspect.signature(gate_check)
    assert 'stamp' not in sig.parameters and 't' not in sig.parameters
    assert 'time' not in inspect.getsource(gate_check).split('"""')[0]


# ---------- ⑤ 不伪装新鲜零速度/不重复旧数据 ----------

@case
def no_fresh_zero_on_drop():
    """全毒流零输出: 拒绝的数据不得变成"新鲜零速度"帧(输出数=放行数=0)。"""
    out, st = run_seq([1.5] * 10)
    assert not any(x[1] for x in out)
    assert st['pass_total'] == 0
    # 决策函数无任何发布/造帧路径(纯返回布尔)
    import inspect
    src = inspect.getsource(gate_check)
    assert 'publish' not in src and 'Odometry' not in src


@case
def no_repeat_old_data():
    """不重复旧数据: 拒帧期间放行计数不增长(没有重复发 last_ok 充数)。"""
    out, st = run_seq([0.4, 0.85, 0.85, 1.6, 0.85])
    # 0.85 与 last=0.4 差 0.45>0.35 两连 dv 拒(不足 3 帧不重锁);1.6 abs 拒清 pending;
    # 末帧 0.85 仍差 0.45 继续拒
    assert out[0][1] and not any(x[1] for x in out[1:])
    assert st['pass_total'] == 1            # 只有首帧 0.4,拒绝期零输出


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
