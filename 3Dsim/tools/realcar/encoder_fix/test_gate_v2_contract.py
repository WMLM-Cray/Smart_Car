#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_gate_v2_contract.py —— v1↔v2 选择性接入契约测试(2026-09-19,零依赖静态)

不 import rclpy(v1 顶层 `import rclpy` 无法离线导入),改静态源码核对 v2 相对 v1 的
话题/门限/消息类型 drop-in 兼容性,证明"换文件名不改 EKF 订阅"不是空口。
v2 的 B1/B2/B3 恢复行为另由 test_gate.py(13/13,纯函数)覆盖。
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, '..', 'encoder_gate.py'), encoding='utf-8') as f:
    V1 = f.read()
with open(os.path.join(HERE, 'encoder_gate_v2.py'), encoding='utf-8') as f:
    V2 = f.read()

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def extract(src, name):
    m = re.search(rf'^{name}\s*=\s*([0-9.]+)', src, re.M)
    return float(m.group(1)) if m else None


# ---------- 门限一致(换版本不改 EKF 输入语义) ----------

@case
def thresholds_match():
    for name in ('V_ABS_MAX', 'DV_MAX'):
        a, b = extract(V1, name), extract(V2, name)
        assert a is not None and b is not None, (name, a, b)
        assert a == b, f'{name}: v1={a} v2={b}'


# ---------- 话题/消息类型 drop-in 兼容 ----------

@case
def same_topic_contract():
    # 订阅 /encoder、发布 /encoder_gate、消息类型 Odometry 三者一致(ekf.yaml odom0 不因换版改动)
    for s in ("'/encoder'", "'/encoder_gate'", 'Odometry'):
        assert s in V1 and s in V2, s


# ---------- v2 只新增诊断,不改 EKF 可见输出 ----------

@case
def v2_extra_diag_only():
    # v2 新增诊断话题,且不新增对 EKF 可见的输出话题(/encoder_gate 仍是唯一输出)
    assert "'/encoder_gate_diag'" in V2 and "'/encoder_gate_diag'" not in V1
    assert V2.count("create_publisher(Odometry") == V1.count("create_publisher(Odometry") == 1


# ---------- v2 提供可离线测的纯函数(v1 缺,故 v1 无法离线单测) ----------

@case
def v2_has_pure_gate_core():
    assert 'def gate_check(' in V2 and 'def gate_check(' not in V1


def main():
    failed = 0
    for fn in CASES:
        try:
            fn()
            print(f'  PASS  {fn.__name__}')
        except AssertionError as e:
            failed += 1
            print(f'  FAIL  {fn.__name__}: {e!r}')
    print(f'\n{len(CASES) - failed}/{len(CASES)} PASS'
          + ('  ** ALL PASS **' if failed == 0 else '  ** 有失败 **'))
    raise SystemExit(1 if failed else 0)


if __name__ == '__main__':
    main()
