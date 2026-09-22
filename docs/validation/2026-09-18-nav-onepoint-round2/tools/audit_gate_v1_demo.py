#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""audit_gate_v1_demo.py —— v1 闸门(318b3d8 tools/realcar/encoder_gate.py)行为离线演示
主线集成审核用(2026-09-18,纯离线,不连车机)。

本脚本把 v1 的 cb() 过滤核心逐字复制为纯函数 v1_check(v, st),对审核清单 10 项场景
逐条跑,目的只有一个:**证明 v1 对给定输入序列的行为**(尤其 B1 重锁死锁 / B3 帧率耦合),
不据此证明 0918 实车毛刺的真实物理根因(那需要原始串口字节,见 EVIDENCE_TIMELINE)。

结论先行: v1 是"只裁大跳变"的最小闸门,正确拦截孤立毛刺;但在"拒帧后真实速度已
移动到距 last_ok 超过 0.35 的位置"时会**永久拒帧(B1 死锁)**,且合法加减速在帧率
<~3.7Hz 时会被**误杀(B3)**。这两点已被 encoder_gate_v2.py(relock + diag + 帧率观测)
修复,13/13 单测覆盖。本脚本只做 v1 的实证演示。
"""
import sys

try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass

V_ABS_MAX = 1.2     # 与 v1 源码一致
DV_MAX = 0.35       # 与 v1 源码一致


def v1_check(v, st):
    """v1 encoder_gate.py cb() 的纯函数等价(逐字语义)。返回是否放行。"""
    if abs(v) > V_ABS_MAX:
        st['drop_abs'] += 1
        return False
    if st['last'] is not None and abs(v - st['last']) > DV_MAX:
        st['drop_dv'] += 1
        return False
    st['last'] = v
    st['pass'] += 1
    return True


def new_state():
    return {'last': None, 'drop_abs': 0, 'drop_dv': 0, 'pass': 0}


def run(name, seq, note=""):
    st = new_state()
    out = []
    for v in seq:
        out.append(v1_check(v, st))
    passed = sum(1 for x in out if x)
    print(f"[{name}] 输入{len(seq)}帧 放行{passed} 拒_幅值{st['drop_abs']} 拒_跳变{st['drop_dv']} "
          f"last={st['last']}  | {note}")
    return out


def main():
    print("=== v1 闸门(318b3d8)行为演示 ===\n")

    # ① 正常前进 0→0.55 @1.3m/s²(50Hz 每帧 +0.026)——应全放行
    ramp = [i * 0.026 for i in range(1, 23)]  # 0.026..0.572
    run("① 正常前进ramp", ramp, "全放行=不误杀")

    # ② 停止 0.55→0 @≤1.3m/s²(每帧 -0.026)——应全放行
    dec = [0.55 - i * 0.026 for i in range(1, 23)]
    run("② 正常减速停止", dec, "全放行=不误杀")

    # ③ 倒车 0→-0.55 —— 负值对称,全放行
    rev = [-i * 0.026 for i in range(1, 23)]
    run("③ 正常倒车", rev, "全放行=负值对称")

    # ⑤ 单个异常帧(幅值门)后恢复
    run("⑤ 单帧毛刺(幅值)后恢复", [0.3, 1.5, 0.3, 0.3, 0.3],
        "毛刺1.5被幅值门拒,后续0.3在0.3±0.35内→恢复")

    # ⑥ 连续异常帧后恢复
    run("⑥ 连续毛刺(幅值)后恢复", [0.3, 1.5, 2.0, 1.8, 0.3, 0.3],
        "连续毛刺全拒,真实0.3仍在last±0.35内→恢复")

    # ⑨ B1 重锁死锁: last=0.3 → 一帧 0.8 被跳变门拒 → 真实速度真到 0.7 → 永久拒
    run("⑨ B1重锁死锁", [0.3, 0.3, 0.3, 0.8, 0.7, 0.7, 0.7, 0.7, 0.7],
        "0.8被dv门拒,真实0.7与last=0.3差0.4>0.35→之后每帧永久拒(B1死锁,闸门静默断流)")

    # ⑩ B3 帧率耦合: 帧率塌陷到 2Hz 时,合法 1.3m/s² 加速 Δv=0.65/帧 > 0.35 → 误杀
    run("⑩ B3帧率耦合(2Hz合法加速被误杀)", [0.0, 0.65, 0.65, 0.65, 0.65],
        "2Hz下合法Δv=0.65>0.35,首帧后每帧误杀;50Hz下Δv=0.026<0.35不误杀")

    # 对照: 50Hz 下同样加速,每帧 +0.026,不误杀
    run("⑩对照 50Hz合法加速", [i * 0.026 for i in range(1, 23)], "50Hz下全放行")

    # ⑪ 首帧无基线(起步) —— 只拦幅值,不拦 dv(起步 0.55 首帧放行)
    run("⑪ 首帧无基线起步0.55", [0.55], "last=None只拦幅值,0.55<1.2放行")

    # 静止后第一帧(有基线 last=0) —— 起步 0.55 首帧 |0.55-0|>0.35 被拒
    st = new_state()
    st['last'] = 0.0  # 静止基线
    out = [v1_check(0.55, st) for _ in range(4)]
    print(f"[静止后起步0.55] 静止基线last=0,起步首帧0.55:|0.55-0|>0.35→"
          f"4帧全拒(真实加速若为阶跃即死锁;若1.3m/s²渐变则0.026/帧可过) 放行={sum(out)}")

    print("\n=== 结论 ===")
    print("v1 正确拦截孤立毛刺(⑤⑥),但存在 B1(⑨ 拒帧后真实速度移动>0.35 则永久静默断流)")
    print("与 B3(⑩ 帧率<3.7Hz 时合法加减速被误杀)。两者已由 encoder_gate_v2.py 修复并")
    print("经 test_gate.py 13/13 覆盖。v1 现役于车机(318b3d8 部署),动态效果未验证。")
    return 0


if __name__ == '__main__':
    sys.exit(main())
