"""按目标速度选模型的**投资组合**解析器 —— 不碰实车控制接口。

## 为什么需要它

2026-09-26 的 n=150 同种子矩阵证明:多速训练(一个网络扛整个速度域)是零和的,
而**单速专化模型在各自速度上普遍碾压多速**:

    速度   m4      rung1   rung2   rung3   rung4   rung5   ms4(多速)
    0.8    0.987   0.387   0.000   0.000   0.000   0.000   0.827
    1.2    0.440   0.800   0.120   0.000   0.000   0.000   0.753
    2.0    0.000   0.033   0.113   0.613   0.200   0.000   0.373
    2.5    0.000   0.000   0.000   0.020   0.547   0.153   0.153
    3.0    0.000   0.000   0.000   0.000   0.020   0.520   0.040

曾经唯一的例外是 1.6 m/s(多速 quad 以 0.813 赢过所有单速)。**2026-09-26 晚这个
例外消失了**:rung2 补训到 500k 后单速在 1.6 上拿到 0.913,Wilson 区间
[0.857, 0.949] 与 quad 的 [0.743, 0.868] 不重叠。所以准确的说法是**1.6 那一档的
单速模型当时还没训够**,不是多速更强。**现在这张表是纯单速的。**

「向下兼容 + 跑到赛道上限」的答案是**投资组合**:按目标速度换模型。本模块就是这个
映射,单独一份,谁也不用改:

    python3 -m common.policy_portfolio --speed 2.0
    -> models/ladder_500k/rung3_sp2.0/candidate_010.npz

    python3 -m vehicle.steer_policy_node $(python3 -m common.policy_portfolio --speed 2.0) --speed 2.0

`steer_policy_node.py` 接的还是一个 .npz 路径,一行不用改。

## 为什么按"档位"而不是插值

每个模型是**单速专化**的,档位之间没有连续过渡 —— 插值出来的权重没有任何一级训过。
实测:选错一档干净率从 0.9 掉到 0.0,所以这里只做**最近档选择**。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# 档位表:目标速度 -> (模型路径, 干净率, 标签)。
# 干净率是 n=150、种子基 71000000 的 fastsim 实测(无 guard),是这份表唯一的
# "为什么选它"的证据。换模型后必须同步更新这里的数字。
#
# 2026-09-26:全表换成 500k 长跑终值(candidate_010)。每一档都做了同种子 n=150
# 配对 McNemar:
#
#     档位   100k    500k    delta   配对(500k赢/100k赢)   p
#     1.2    0.800   0.873   +0.073  22 / 11               8.0e-02
#     1.6    0.593   0.913   +0.320  50 / 2                6.1e-13
#     2.0    0.613   0.887   +0.273  43 / 2                5.9e-11
#     2.5    0.547   0.880   +0.333  52 / 2                1.7e-13
#     3.0    0.520   0.827   +0.307  52 / 6                3.2e-10
#
# **1.2 那一档 p=0.080,不显著** —— 点估计更好、配对 22:11 也同向,但没到 0.05。
# 它跟其它四档同一配方、同一方向,所以一并换成 500k,但**不要声称 1.2 的提升
# 已被证明**。其它四档 p 都 < 1e-9。
LADDER = [
    (0.6, 'models/steering_v1/candidate_002.npz',                  0.913, 'm4'),
    (0.8, 'models/steering_v1/candidate_002.npz',                  0.987, 'm4'),
    (1.0, 'models/steering_v1/candidate_002.npz',                  0.913, 'm4'),
    (1.2, 'models/ladder_500k/rung1_sp1.2/candidate_010.npz',      0.873, 'rung1-500k'),
    (1.6, 'models/ladder_500k/rung2_sp1.6/candidate_010.npz',      0.913, 'rung2-500k'),
    (2.0, 'models/ladder_500k/rung3_sp2.0/candidate_010.npz',      0.887, 'rung3-500k'),
    (2.5, 'models/ladder_500k/rung4_sp2.5/candidate_010.npz',      0.880, 'rung4-500k'),
    (3.0, 'models/ladder_500k/rung5_sp3.0/candidate_010.npz',      0.827, 'rung5-500k'),
]

# 实车**已标定**的速度区间:来自 docs/实测/2026-09-24-pid 的 12.5V 曲线,
# 实测点到 1618 -> 1133 mm/s 为止。下限是电调停车区(物理),上限只是标定边界。
# 区间外不是不能跑,是**没有实测的 PWM↔速度映射**可用 —— 要跑先补曲线。
# (另有纯代码闸:steer_policy_node.py 的目标速度校验和 PWM_HARD_MAX=1620,
#  那两个归项目所有者,不是物理限制。)
REALCAR_MEASURED_FLOOR = 0.66
REALCAR_MEASURED_TOP = 1.13


def select(speed: float) -> tuple[str, float, str]:
    """给定目标速度,返回 (模型路径, 干净率, 标签)。

    按**相对偏离**选最近的档,不做插值。观测里 `speed/scale` 是归一化的,所以决定
    模型能不能跑好的是**相对**偏离。5 个相邻档各自细扫(n=150/点)测出真实交叉点:

        档位边界        实测交叉点   绝对中点   相对偏离分界
        1.0 | 1.2          1.045      1.100      1.0909
        1.2 | 1.6          1.315      1.400      1.3714
        1.6 | 2.0          1.784      1.800      1.7778
        2.0 | 2.5          2.228      2.250      2.2222
        2.5 | 3.0          2.691      2.750      2.7273
        与实测的偏差之和               0.237      0.151

    相对偏离在 **5/5** 个边界上都更接近实测,所以 key 用 `|v - v_train| / v_train`
    (它给出的分界就是两档的**调和中点** `2ab/(a+b)`)。

    **最要紧的一处是 1.1 m/s**:绝对规则下 1.1 距 1.0 和 1.2 各 0.1 是平局,取低速
    -> m4(0.720);相对规则下 1.1 距 1.2 只有 8.3%、距 1.0 有 10%,选 rung1
    (**0.873**)。而 1.1 m/s 正好落在实车**已标定**区间(0.66~1.13)内。

    平局判定先把比值量化到 1e-9 再比:否则调和中点附近会因浮点抖动改变结果。
    量化后比值相同的按档位速度升序取,也就是**打平取较低速**。
    """
    speed = float(speed)
    _, path, rate, label = min(
        LADDER, key=lambda row: (round(abs(row[0] - speed) / row[0], 9), row[0]))
    return path, rate, label


def calibrated(speed: float) -> bool:
    """这个目标速度有**实车实测的速度曲线**吗?

    注意:这不是"允不允许",是"标定过没有"。上限本身是**代码闸**不是物理墙 ——
    `vehicle/steer_policy_node.py` 的目标速度校验和 `PWM_HARD_MAX = 1620` 都是纯代码
    的安全钳位,归项目所有者,不是不可动的约束。

    **真正没解决的是标定**:12.5V 曲线到 1618 -> 1133 mm/s(~1.13 m/s)为止,再往上
    没有实测的 PWM↔速度映射。所以 1.13 以上是**未测区**,不是禁区 —— 要跑就得先补
    一条速度曲线。下限是**物理**的:电调停车区,12.5V 下 PWM 1588~1602 是平台
    ~0.66 m/s,踩不死,`pwm_floor` 写死不下探。这个没人能绕开。
    """
    return REALCAR_MEASURED_FLOOR <= float(speed) <= REALCAR_MEASURED_TOP


def select_path(speed: float, root: str | Path = '.') -> str:
    """只要路径,且**校验文件真的在** —— 部署时路径错了要立刻报,不要静默。"""
    path, rate, label = select(speed)
    full = Path(root) / path
    if not full.is_file():
        raise FileNotFoundError(
            'portfolio 表里 %s (%.3f m/s, 干净率 %.3f) 指向 %s,但文件不存在'
            % (label, speed, rate, full))
    return str(full)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--speed', type=float, default=None, help='目标速度 m/s')
    ap.add_argument('--root', default='.', help='仓库根(默认当前目录)')
    ap.add_argument('--table', action='store_true', help='打印整张档位表')
    args = ap.parse_args()
    if args.table:
        print('  %-7s %-9s %-11s %-8s %s' % ('速度', '干净率', '模型', '实车', '路径'))
        for sp, path, rate, label in LADDER:
            mark = '已标定' if calibrated(sp) else '未标定'
            print('  %-7s %-9.3f %-11s %-8s %s' % (sp, rate, label, mark, path))
        print()
        print('  实车已标定区间 %.2f~%.2f m/s(下限=电调停车区,物理;'
              '上限=曲线测到哪)。' % (REALCAR_MEASURED_FLOOR, REALCAR_MEASURED_TOP))
        print('  1.2 以上的档位模型能跑,但**实车没标定过** —— 上实车之前先补一条')
        print('  该速度的 PWM↔速度曲线,别拿 0.8 的标定外推。')
        return
    if args.speed is None:
        ap.error('要么给 --speed,要么给 --table')
    if not calibrated(args.speed):
        print('提示: %.2f m/s 在实车已标定区间 %.2f~%.2f 之外,'
              '模型能选,但车上没有实测的 PWM↔速度映射'
              % (args.speed, REALCAR_MEASURED_FLOOR, REALCAR_MEASURED_TOP),
              file=sys.stderr)
    print(select_path(args.speed, args.root))


if __name__ == '__main__':
    main()
