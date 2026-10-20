#!/usr/bin/env python3
"""走廊实验臂 —— 在策略节点出口外挂一层规则。

生产件零改动：`vehicle/road_policy_node.py` / `vehicle/road_car_node.py` /
`vehicle/scan_guard.py` / `common/*` 都只在运行时被猴补，源码一个字节不动。
用法与 `arms/arm_host.py`（宿主）相同，只多读一个环境变量 `CORRIDOR_ARM`：

    CORRIDOR_ARM=A  什么都不做（对照臂，等价于原节点）
    CORRIDOR_ARM=B  只让方向会找空档（**仅在被 guard 拦停的那一刻**）
    CORRIDOR_ARM=C  只让速度会看净空（gap 反解速度上限）
    CORRIDOR_ARM=D  B + C
    CORRIDOR_ARM=E  **常驻走廊跟随**：每拍用 common/corridor.py 解出的走廊中心 c
                    直接定转向；速度同时受走廊宽度与前方净空约束；被拦停时也按
                    走廊走，而不是在七条离散弧里挑。
    CORRIDOR_ARM=F  E + B（走廊这一帧无效时，退化成 B 的弧道扫描）
    CORRIDOR_ARM=G  **提前换弧**：不等被 guard 拦停，只要策略要打的那条弧"即将"不够宽
                    就先换成离策略原意最近的可行弧，同时把速度压到"在自己的提前量内
                    还能停住"的上限；被拦停时仍然保留 B 的弧道扫描兜底。
    CORRIDOR_ARM=H  G 但提前换弧时用走廊中心 c 做同侧性检查（见 _corridor_side_ok）
    CORRIDOR_ARM=Y  **不是新策略**：就是 W 的层配置（见 ARM_LAYERS），但宿主换成仓内
                    `arms/arm_host.py`、几何读 `common.params.CarParams`。W 认证时用的
                    是仓外 `teleop/road_policy_node.py`，那个模块在 main 上已经不存在。

三个挂载点，职责不同：

* `on_scan()` —— 每来一帧新雷达就解一次走廊，缓存成 `self._corr`。雷达实际约
  10 Hz，而 tick 是 20 Hz，所以一帧最多解一次，不做重复计算。
* `publish()` —— 正常行驶时改转向 / 限速。**E/F 的转向层在这里是常驻的**，不等被拦；
  这才是"让车知道路在哪"和"被堵住了再想办法"的区别。
* `neutral()` —— 宿主的硬闸一拦就直接调 `neutral()`（`arms/arm_host.py::_step`，
  旧件对应 `teleop/road_policy_node.py:258`），**根本不经过 `publish()`**，
  所以真正卡死的那一刻只能挂在这里救。

为什么不能用 guard 的 gap 当"路在哪"的代理（armB r1 的教训，勿删）
--------------------------------------------------------------------
2026-10-05 的 armB r1 把方向层挂在 `publish()` 里、用 `score(a) = min(gap(a),1.5) - |a|`
选方向，结果**把车掰直开出了赛道**：净空 >= 1.5 m 时七个候选同分，`-|a|` 独占决定权，
于是开阔区一律直行。赛道本身宽 1.25~1.5 m，"前方 1.5 m 内没锥"是常态，这层几乎一直在工作。
更根本的是：**"这个方向净空大"不等于"这个方向是路"** —— A 臂 r3 sim 10.5~15.2 实测
`gap(+0.0) = 11.95~11.99`，而车必须左转。判"路在哪"必须用两侧墙的走廊估计（c/w），
这正是 E/F 臂成立的前提：走廊中心是**两侧障碍的中点**，开阔区的直行不再是"没得选"。
"""
import hashlib
import logging
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import numpy as np                                                # noqa: E402

import arms.arm_host as RPN                                       # noqa: E402
from common import corridor                                      # noqa: E402
from common.params import CarParams                               # noqa: E402
from common.pwm import action_to_steer                            # noqa: E402

_ARM_RAW = (os.environ.get('CORRIDOR_ARM') or 'D').strip().upper()
# Y 不是新策略：它就是 W 的层配置，只换了宿主（仓外 teleop → 仓内 arms/arm_host.py）
# 与几何来源（写死的 0.33 → common.params.CarParams）。这里用别名让下面所有
# _use_*() 判定**一字不改**，也保证 Y 与 W 的层选择永远同步、不靠人工复制 tuple。
ARM_LAYERS = {'Y': 'W'}
ARM = ARM_LAYERS.get(_ARM_RAW, _ARM_RAW)
ARM_TAG = _ARM_RAW      # 版本串/日志里记原始字母；run 目录靠 ${ARM} 天然区分 W/Y

# ---- B 臂：动作域候选扫描 ------------------------------------------------------
# 在**动作**域采样而不是转角域:action_to_steer 是非线性舵机曲线(近满舵 15 刻度只多
# 2~3 度),动作域等距取样正好在两端更密,而那正是需要分辨力的地方。
ACTS = (-1.0, -0.6, -0.3, 0.0, 0.3, 0.6, 1.0)
# 这两个数由 runs/arm_score_sweep.py 在 r1 那 83 帧上扫出来。要点:inf(这个方向没有
# 回波)只能当 SCORE_CAP=1.5 m 算,不能当"开阔"。原来按 3.0 算时,一个无回波方向
# (3.0-0.25=2.75) 会稳定压过一条实打实的通路(1.4-0=1.4),于是候选扫描反复去挑那条
# 0.2 秒后就只剩 0.07 m 的满舵擦边弧 —— 那不是"选得不好",是评分把"没看见"当成了"很空"。
# STEER_COST 从 0.25 提到 1.0 是同一件事的另一半:同样净空优先不打方向,切换从 47 次降到 13 次。
STEER_COST = 1.0           # 每单位动作的惩罚:同样净空时优先不打方向
DEADBAND = 0.10            # 改进不到这个数就不动,防抖(20 Hz 下 0.1 m 的净空差是实的)
SCORE_CAP = 1.5            # inf(该方向无回波)折算成多少米;不许吃掉所有候选

# ---- 被拦时改走(猴补 neutral) --------------------------------------------------
# 为什么必须有它:卡死期间实测 speed med = 0.000、|steer| med = 0.379(策略满舵饱和)、
# blocked = 95%。策略想满舵右转 -> 右边有锥 -> guard 拦 -> neutral() -> 车不动 ->
# 观测不变 -> 策略还是满舵 -> 永远拦。**guard 一拦就直接 neutral(),根本不经过
# publish()**,所以只挂 publish 的 B 臂在真正要命的那一段一次都不会被调用。
# 而同一批帧上直行方向全程 gap(0) = 1.0..4.0 m 是通的 —— 车不是被路堵住,是被
# "当前这个转角会被拦"堵住。所以拦下来的那一刻要问一句:别的方向能不能走。
ESCAPE_MARGIN = 0.15       # 改走的方向至少留 guard_stop + 这个数,否则宁可老实停
ESCAPE_V = 0.45            # 改走时的速度:制动距离 d(0.45)+SCAN_PERIOD*0.45 = 0.0825 m
# 闸门:不是"被拦"就改走,而是"被拦 + 车停住 + 停住够久"。健康的一局里 blocked 占
# 64~85% 的 tick,但车在动、观测在变,下一拍就自己解开了;照搬 road_policy_node.py:250
# 的判据会把正常行驶也一起接管(armB r1 就是这么把车掰直开出赛道的)。
ARM_ESCAPE_DELAY = 0.30    # 车停住多久才算"卡住"。A 臂三局的停住段 p50 2.5~2.9s,取 0.30
                           # 是刻意早于 STUCK_SECONDS=3.0 —— 赶在倒车脱困之前动手
ESCAPE_REARM = 0.60        # 我们自己在改走期间不再要求"车停住",否则会在 0.45 和 0 之间抖
STOP_EPS = 0.05            # |v| 小于它算停
# 倒车脱困:连续截停满 STUCK_SECONDS=3.0 之后那一下
#   "盲倒 1.5s、倒不动就换向"是本项目自写的**实验宿主**行为(旧 teleop/road_policy_node.py:204-217;
#   搬仓后落在 arms/arm_host.py)。实车节点 vehicle/road_car_node.py:102-103 故意不倒车
#   (「雷达在车头,背后看不见」)⇒ 仿真**默认只停不倒**(宿主 `REVERSE_ENABLED` 默认 False),
#   要复现旧 W/Y 面板读数置 `ARM_HOST_REVERSE=1`。K 臂给这一下封顶(REV_ABS=1.5),
#   但盲倒关时 `_capped_escape_reverse` 直接落到宿主出口(只停),不再自己发负速度。

# ---- C 臂:gap 反解速度上限 ----------------------------------------------------
L_DECEL = 0.0536           # 制动拟合 d(v) = L_DECEL*v + U_DECEL*v^2 (RMS 24 mm)
U_DECEL = 0.1031
SCAN_PERIOD = 1.0 / CarParams().lidar_hz   # 雷达帧周期,单源 params.lidar_hz(12 Hz,[实测] 11.99)。
#   旧写死 0.18(注释自称"实测 5.5 Hz")全仓查无出处,按实测 12 Hz 改。
#   这一段是"看见之前还在跑"的距离;改后 MOVE_GAP_MIN 0.176 → 0.133,会动 W/Y 读数口径。
V_MIN = 0.45               # fastsim 的速度下限:策略本身不出 0,停车归 guard 和人
GAP_HOLD_N = 3             # C 臂看最近 N 个雷达帧里**最坏**的那一眼(见 _recent_min_gap)

# ---- E 臂:常驻走廊跟随 -------------------------------------------------------
# 站位优先序。为什么是 1.0 打头:runs/lateral_geometry_trace.py 与子智能体的
# corridor_truth_check 都测过,x0>=0.8 才够稳;x0=0.6 的 p90 误差能到 1.07 m
# (切片里真值墙只剩 <=3 条光线,甚至零回波),x0=0.4 的切片 |x-0.4|<=0.10 正好落在
# 车体自遮挡足迹 (BODY_REAR..BODY_FRONT = -0.065..0.415,单源 common/params.CarParams)
# 里,直接被 self_filter 吃掉。
# 1.6/2.0 的两侧可见率只有 34%/4%,所以只做 1.2 / 0.8 两个退路。
CORRIDOR_X0_CHAIN = (1.0, 1.2, 0.8)
# 纯追踪:把"走廊中心在 x0 处、横向偏了 c"换算成前轮转角。弯道上走廊中心自己会偏,
# 所以这不是"把车拽回中线",而是"顺着路走"。
# 轴距：几何单一来源 = common/params.CarParams（2026-09-30 实测 0.35 m）。
# 旧值 0.33 是从旧 teleop/scan_guard.py 抄来的，仓内 vehicle/scan_guard.py:34 已改实测值。
WHEELBASE = CarParams().wheelbase
# ⚠️ 待定（几何两套值）：仿真世界已按用户 2026-10-08 的决定换成**仓内实测包**
#   （`simulation/gazebo/src/racecar_sim/config/params.yaml`：wheel_base 0.35 /
#   laser_x 0.145 / laser_z 0.28；`evaluation/run_gazebo_supervised_smoke.sh` 最后
#   source 它，覆盖队友仓里的旧值）⇒ 控制器与仿真世界现在是同一把尺，本节旧注释里
#   "偏航少 5.7 %、净空乐观约 9 cm"那两个混杂因素都已消掉。
#   仍然对不上的是**车体框**：URDF 车体 0.56 m 长、`base_footprint` 在两轴中点，
#   而实车实测车长 0.48 m、原点在后轴 ⇒ 同一个 gap，仿真比实车多留 0.135 m 净空。
#   要不要把仿真车体搬到"后轴原点 + 0.48"会改 racecar.xacro 与所有已认证世界的
#   world_sha256、必须整轮重跑，所以标待定、交组里定。
CORRIDOR_GAIN = float(os.environ.get('CORRIDOR_GAIN') or 1.0)   # 0=整个不用走廊, 1=全用
# 走廊宽度限速:窄到 CORRIDOR_W_MIN 就给 V_MIN,宽到 CORRIDOR_W_FULL 就不管。
# narrow_03 的走廊实测 1.43~1.63 m,所以这条平时不工作,只在真的收窄时生效。
CORRIDOR_W_FULL = 1.10
CORRIDOR_W_MIN = 0.45
# 切片宽度。默认 HALF_SLICE=0.10 在 narrow_03 上只有 42%/51%(x0=0.8/1.0)的帧能看到
# 两侧,因为 0.2 m 宽的切片常常整条落在两颗锥桶之间。runs/arm_e_offline.py 扫过
# 0.10/0.18/0.25:0.25 把两侧可见率抬到 83%/75%、链上可用率 77%->83%,而 |c| p90
# 只从 0.186 涨到 0.305(仍在一米四走廊的尺度内)。所以取 0.25。
CORRIDOR_HALF_SLICE = 0.25
# 宽度合理性上限,缺了它 E 层会犯错:车头正对口袋时前方"真的开阔",逐帧实测会有
# 23~25% 的帧报出 w 到 3.3~4.7 m(在一米四的赛道上)、|c|>0.3 m、动作打到 ±0.7。
# 卡在 2.2 m 后 |c| p90 掉到 0.31。宁可不改,也不拿"路宽 4.7 m"去开车。
CORRIDOR_W_MAX = 2.2

# ---- L 臂:左右净空不对称 -> 有界侧向偏置(v3.4_L2) ------------------------------
# 证据(实测诊断):把口径钉在同一个弯
# (narrow_01 s*=25.5、R=1.11)后,策略在弯心的 4 轴 SAT 净空只有 +0.020…+0.113 m,
# 而车在那一刻离中心线 9–21 cm —— 锥列本身给的是约 ±0.31 m 的余量
# (half 0.625 − 车半宽 0.175 − 锥半径 0.14)。**净空不是赛道极限,是走线偏差吃掉的。**
# 而结局不由净空决定(紧线上 0/1/5 圈都有),决定结局的是 guard 的二值判决,它判的
# 正是那条线压着的阈值 ⇒ 目标是把"贴锥那几拍"的横向偏差让回来。
#
# v3.3_L 的两条否证(同一份实测诊断,不是推测):
#   ① **闸门不能用 guard 的 gap**:v3.3 用 `gap < 0.85` 当闸门,narrow_01 cap1.8 r1
#      全程 2643 拍里只有 **1.2 %** 满足 ⇒ 偏置只动了 6 拍(0.2 %),那一版等价于 A 臂。
#      而同一份诊断里走廊几何的可用率是 41–59 %(窄 ∧ |c|≥0.10)。**gap 只在车已经
#      贴住时才叫,那时任何"提前"都晚了**;G 臂"可用窗 0.6 %"是同一个病。
#   ② **方向不能用 c**:车在弯心整段贴在左侧(e=+0.29 m),前视切片上的走廊中心 c
#      却从进弯的 −0.09 翻到弯心的 **+0.27**(x0=1.0、R=1.11 时曲率项 x0²/2R ≈ +0.45
#      是主导项)⇒ `sign(c)` 会把车继续往弯里推,方向是反的。这同时解释了 E 臂当年
#      "追 c"却卡进新口袋、以及 `_corridor_side_ok` 里"c 不是可用转向目标"那句话。
#
# 所以 v3.4 改成问 guard **同一件事的两个方向**:把策略自己的动作往左、往右各挪
# L2_DELTA,比较两条弧的净空 `gL`/`gR`,往**更空的那一侧**给一个有界偏置。
# 这是"车现在这个位置,左右哪边堵"的直接测量:不含前视曲率项,不需要地图,
# 也不需要任何一侧"已经报警"才动手。
L2_DELTA = float(os.environ.get('L2_DELTA') or 0.20)          # 探测动作:策略动作左右各挪这么多
L2_MAX_SWING = float(os.environ.get('L2_MAX_SWING') or 0.25)  # 偏置上限,动作单位(≈7.2°)
L2_ASYM_MIN = float(os.environ.get('L2_ASYM_MIN') or 0.08)    # 左右净空差小于它就别动(防抖)
L2_ASYM_FULL = float(os.environ.get('L2_ASYM_FULL') or 0.30)  # 差到 0.30 m 就给满偏置
L2_W_MAX = float(os.environ.get('L2_W_MAX') or 1.30)          # 走廊宽于此不触发(宽道自限)
# gap 报 inf = "这条弧 4 m 内什么都没有"。要比较左右就得给个有限值,统一封到 4 m
# (=MAX_LOOKAHEAD);两侧都 inf 时差为 0,自然落进"别动"。
L2_GAP_CAP = float(os.environ.get('L2_GAP_CAP') or 4.0)
# ---------------------------------------------------------------------------
# v3.5_L3:信号消失时"保持"上一拍偏置,而不是撤回中立。
#
# 依据(不是猜测):cap1.9 上 L2 唯一输掉的那局(`narrow_02 r8`,0 圈/静止 116.9 s/里程
# 37.7 m,坏盆地)的 `.b` 诊断里 `asym` 的**中位恰为 0.000**,而干净局的中位是 ±米级。
# asym = 0 的含义是"左右两次探测读到同一个数" —— 正是车被锥桶夹住、两侧一样堵的时刻。
# v3.4_L2 在这个时刻会撤回偏置、把方向决定权交还给策略的动作抖动,于是车留在原地;
# v3.5_L3 改为**沿用上一拍的偏置**(含方向与幅度),最多 L3_HOLD_TICKS 拍,超时就回中立,
# 且一旦走廊估计失效或走廊变宽(> L2_W_MAX)就立刻清掉保持,免得陈旧方向在开阔处推车。
L3_HOLD_TICKS = int(os.environ.get('L3_HOLD_TICKS') or 20)   # 保持多少拍(≈1 s @20 Hz)


# ---- G 臂:提前换弧 + 边距限速 --------------------------------------------------
# 为什么不是"等 gap 逼近 guard_stop 再换"(runs/preempt_probe.py 的判决,勿删)
# --------------------------------------------------------------------------
# 探针把 narrow_03 两局逐拍摊开:提前窗口 `guard_stop <= gap < guard_stop+m` 的拍数,
# A r1(2901 拍)在 m=0.10/0.20/0.30/0.40/0.55 下只有 16/19/22/29/35 拍(0.6%~1.2%),
# A r3(2877 拍)只有 15/17/21/25/44 拍。**车从"没事"到"被拦"只用 1~4 拍**,所以
# "等净空掉下来"这个触发量本身几乎没有提前量可言。
# 真正提前量在**几何**里:runs/junction_shape.py 量出 narrow_03 的瓶颈是 s=19.76~21.61
# 一段 **R=1.045 m 的恒定半径左弯**(54.8 deg/m),反向再来一段(23.01~24.77),两腿相距
# 1.734 m、全程最近锥只有 0.625~0.762 m。车的最小转弯半径 0.881 m ⇒ **这条路要求
# 持续打 81% 满舵**(atan(0.33/1.045)=17.5° vs MAX_STEER 21.7°),而走廊只有约 1.26 m
# 宽。所有臂的失败局都停在 s=20.0~21.3 —— 正是刚进这个圆弧一米的地方。
# 结论:G 的触发量仍然只能是 gap(策略输出的那条弧的净空),但**判据要提前一档**,
# 而且换出来的方向要"离策略原意最近",这样在开阔区它根本不会触发(见下面的自限性)。
PREEMPT_MARGIN = float(os.environ.get('PREEMPT_MARGIN') or 0.20)
PREEMPT_SPEED_MARGIN = float(os.environ.get('PREEMPT_SPEED_MARGIN') or 0.0)
# 换弧时要求新方向至少留这么多净空(比"被拦后"的 ESCAPE_MARGIN 一样,不额外苛刻)
PREEMPT_CLEAR_MARGIN = float(os.environ.get('PREEMPT_CLEAR_MARGIN') or ESCAPE_MARGIN)
# **最小修正**:只允许把策略原意挪动这么多动作(=约 17 度舵角)。超过就不动。
# runs/arm_g_smoke.py 实测:策略原意被堵时,清开它需要的动作 p50 就有 0.58,
# 所以这个上限不是"把修正变小"的旋钮,而是**硬性回答"这层绝不许变成另一个规划器"**。
PREEMPT_MAX_SWING = float(os.environ.get('PREEMPT_MAX_SWING') or 0.60)
PREEMPT_STEP = 0.05           # 从原意朝可行方向线性试探的步长(最多 12 次 gap 查询)

# ---- N 臂(v3.6):净空反解限速,但**预留量与 guard 硬闸线解耦** -------------------
# 病根不是"被拦",是"拦不下来"。A–D 上同一个 R36:cap1.8 干净 4/4 = 100 %、cap1.9 掉到
# 60~67 %、cap2.0 66.7 %,而这两个 cap 之间**只有速度不同**。差异可以用 guard 的停车
# 预算一次算完:guard 在 gap(从保险杠算的沿路净空)跌到 GUARD_STOP=0.25 时才硬截停,
# 也就是它只给了 NOSE_AHEAD+0.25 的"看见→停住"预算(当时 NOSE_AHEAD 写死 0.43 ⇒ 0.68 m;
# 2026-09-30 实测车头 0.415 ⇒ 0.665 m);而解析刹车
# 距离 d(v)=0.0536v+0.1031v^2 加上雷达周期 0.18 s 的反应距离:
#   v=1.8 → 0.754 m(用掉 84 %? 见 runs/speed_probe.py 的口径)  v=1.9 → 97.6 %
#   v=2.0 → 105.8 %
# ⚠️ 订正(2026-10-08):上面的 0.18 s 是当时写死的 SCAN_PERIOD,全仓查无出处;按实测雷达
#   12 Hz 改后 SCAN_PERIOD=1/12,同一预算改成 v=1.8 → 0.581 m(87.3 %)、1.9 → 0.632(95.1 %)、
#   2.0 → 0.686(103.2 %),预算仍为 NOSE_AHEAD 0.415 + GUARD_STOP 0.25 = 0.665 m。cap1.9 起
#   "看见→停住"不够这条结论不变(1.9 仍用掉 95 %),但绝对值变。见 SCAN_PERIOD 定义处。
# ⇒ cap1.9 起车在"看见"和"停住"之间已经把锥桶吃进车头,评分器记一次接触。这正是
# hit_inspect 量到的那次:最接近时车到被撞锥只有 0.444~0.485 m(锥桶到后轴),而车头
# 在前 0.43 m(旧写死值;实测 0.415)、车半宽 0.175 + 锥半宽 0.14 = 0.315 m ⇒ 是**车头角**
# 擦上去的。
#
# C/D/E/F 那版限速层用的预留量就是 node.guard_stop(0.25),等于"要求比 guard 自己更早
# 停住"。用 A–D cap1.9 已经跑完的 policy.csv 离线重算(runs/speed_probe.py,不跑仿真):
# 这个口径要花 4.8~6.6 % 的圈速;把预留量收到 0.10 只花 2.8~4.4 %。
# N 把预留量抽成独立常数 SAFE_RESERVE,默认 0.10:只要求"按解析刹车能停在离锥 0.10 m
# 处",剩下那 0.25 m 硬闸线仍然留给 guard —— **不拆安全网,只是更早开始收油**。
#
# 性质:N 只压速度、永不产生转向,所以走线与 A 逐线相同,只改"以多快的速度走上去";
# 它也不试图解开"停→3 s→倒车"那个环(那是 I/J 的地盘),它保证的是**环发生时不再撞上**。
SAFE_RESERVE = float(os.environ.get('CORRIDOR_RESERVE') or 0.10)
# P 臂 = v3.6_N 的换档重跑:同一层限速,但预留量取 **0.25**(= guard 自己的硬闸线)。
# 起因:cap2.2 上 N(预留 0.10)在 6 局里仍有 1 次擦锥(`track_A r3` 末圈 54.9 s
# 卡停+倒车),而离线 `runs/speed_probe.py` 量到把预留从 0.10 抬到 0.25 只多花
# 0.9~2.9 % 圈速(cap2.2 实测稳态 16.6 s ⇒ 约 17.0 s,仍低于 R36 干净档 17.80 s)。
# 预留量在 **import 时**决定,所以必须换一个臂字母,免得同一 arm 名下混两种口径。
#
# ---- T 臂(v4.0)= S(v3.9_SP)+ 净空反解限速层(N 那一层) ---------------------------
# 起因:把 S 抬到 cap1.9/2.0/2.2 后出现**新形态**的长判负段(`runs/seg_probe.py` 打在
# S cap1.9 `track_A_r3`,那局 88.9 s 卡停圈):
#   窗口 35.6–49.4 s(271 拍全判负):gap p50 **+0.056**,而策略**原意** v_cmd p50 1.893、
#   原意 steer p50 **0.000**;实测 speed p10/p50/p90 = −0.281/**−0.107**/0.000,
#   判负拍里 113/271 拍实测 |speed|<0.10,|yaw_rate| p50 0.001 ⇒ **车顶着锥原地磨**,
#   不是 v1.8 时代那种"停车→观测冻结"(那段里 `v_cmd==0` 的拍数 = 0)。
# 为什么顶着:guard 自己的解析刹车距离 `need(v)=0.0536v+0.1031v²+0.18v`(0.18 = 当时 SCAN_PERIOD;
#   2026-10-08 改 1/12 后为 `+0.0833v`,同一口径 need(1.89) 由 0.81 降到 0.63 m),
#   **need(1.89)=0.81 m**,而进判负前只剩 0.129~0.226 m(`r_approach_one.sh`)⇒ 停不下来。
#   R36 在 cap1.8 同样不满足 need(v)(0.75 m),但 1.8 速度低 + S 提前层能把弧挪开。
# 修法:到锥前就按**净空反解**收油 —— 这层 C/D/E/F(`_use_speed`)与 N/P
#   (`_use_speed_reserve`)早就各有自己的口径,**唯独 R/S 两个元组里都没有**。
# T 三个元组全带上(`_use_project`、`_use_prearm`、`_use_speed_reserve`),即 **T ⊃ S**:
#   只增不减(若 T 与 S 读数一致,就说明限速层一次都没触发,属于接线错误而不是"没用")。
# 预留量走 SAFE_RESERVE(0.10):只要求"按解析刹车能停在离锥 0.10 m 处",guard 自己的
#   0.25 m 硬闸线原封不动 —— 不拆安全网,只是更早收油。
#
# ---- U 臂(v4.1)= T + "闸门判负但净空为正时,按 guard 自己的刹车律极低速继续走" ----
# 起因:T 抬到 cap1.9/2.2 后**撞锥归零**(23 局 0 撞锥 0 出界,对照 R36@2.2 撞 4、S@2.2 撞 3),
# 但剩下的长判负段换了形态:`runs/pin_report.py T 2.2 --all` 量到 20 个长段(18 个顶死,
# 最长 16.30 s),签名**全部**是 `gap p50 = +0.013…+0.231`(**净空为正**)、
# `|speed| p50 ≈ 0.000`、策略原意 `steer = 0.000`,而且只落在 A、C 两道
# (B/D 判负 0.0~0.7 %、零倒车)。cap1.9 上同一签名:`A r1` 两段 10.40 s/3.69 s,
# `gap p50 +0.169/+0.196`。
# 机理(生产 `road_policy_node.py:240-258` 原文):告警判据是**策略那条弧**的扫掠净空
#   `gap = self.guard.gap(action_to_steer(steer))`,`gap < guard_stop(0.25)` 就
#   `self.neutral()`(全零 Twist)。**净空为正说明车不在碰撞航向上** —— 撞点实测
#   (`runs/hit_inspect.py`)锥到后轴 0.444~0.485 m,而车半宽+锥半宽 = 0.315 m ⇒ 锥在
#   **车侧**,不是车头前方。车一停,观测就冻结 ⇒ 策略输出不变 ⇒ 永远判负。
# 空档在哪:R/S/T 已有的救火层要求"净空 ≥ guard_stop + PROJECT_CLEAR_MARGIN = **0.40**"
#   才换弧,而闸门只在 0.25 判负 ⇒ 0.25~0.40 之间那 0.15 m 里,投影**诚实**返回 None,
#   于是 `_orig_neutral` ⇒ 停满 3 s ⇒ 盲倒。实测支持:`s_prearm_report T 2.2` 里崩的两局
#   `A r1`/`A r3` 是 `g0_p50 = 0.261/0.260`、`miss = 98/44`,而 9 局跑得好的
#   `g0_p50 = 0.51~0.62`、`miss ≤ 3`。
# 修法(**不换弧、不选线、不倒车**):判负且 `gap > CREEP_GAP_MIN` 时,沿**策略自己那条弧**
#   发一个速度,取 guard 自己的解析刹车律 `v_allow(gap, CREEP_RESERVE=0.0)`,即
#   "按 d(v) 能在净空用完之前停住"。这是 guard 本来就有的安全律(生产从不使用它),
#   不是第二套策略:`v_allow(0.169,0)=0.778`、`v_allow(0.05,0)=0.298`、
#   `v_allow(0.013,0)=0.089`(SCAN_PERIOD=1/12 口径;旧 0.18 时是 0.577/0.140/0.054)
#   ⇒ **净空越小走得越慢**,净空一掉到 0 就返回 None,
#   生产照旧停/倒车,安全网一字不动。
# 为什么"走"能解环:车一动,那颗锥相对扫掠弧就往车后滑;`scan_guard.py` 的弧从 θ=0
#   (后轴)开始扫,锥一过就出弧 ⇒ 净空立刻变大 ⇒ 下一拍闸门自己放行。
#   3 s 预算内按 v_allow 能走 0.15~1.7 m,足够。
# 进度续命:`CREEP_PROGRESS_M` 米。生产 3 s 规则的理由写在 `:265-266` —— "车已经不动了,
#   再等下去场景永远不会变"。本层只要**真的动了** CREEP_PROGRESS_M,就把 `block_t` 刷新
#   一次(= 把"车不动"这个前提证伪后重新计时);走不动(净空为 0)则不刷新,倒车仍是最后手段。
CREEP_GAP_MIN = float(os.environ.get('CORRIDOR_CREEP_GAP_MIN') or 0.005)
CREEP_RESERVE = float(os.environ.get('CORRIDOR_CREEP_RESERVE') or 0.0)
CREEP_V_MAX = float(os.environ.get('CORRIDOR_CREEP_V_MAX') or V_MIN)
CREEP_PROGRESS_M = float(os.environ.get('CORRIDOR_CREEP_PROGRESS_M') or 0.10)

# ---- V 臂:用户 m14584 / m14588 两条硬纠正(2026-10-07)-----------------------------
# 纠正一(速度下限):U 的蹭走层按 `v_allow` 反解出来的 0.02~0.44 m/s **是废指令** ——
#   `V_MIN = 0.45` 是策略层自己的下限,比它低的目标车里根本不动(实测那些长段里
#   `|speed|` 只有 0.106~0.129 m/s,是车被锥顶住时被物理推/溜车的读数,不是我们控出来的)。
#   ⇒ V 只发一个速度 `MOVE_V = V_MIN = 0.45`,绝不更低。能不能发只看净空够不够:
#       刹车账 need(v) = (L_DECEL + SCAN_PERIOD)*v + U_DECEL*v^2,need(0.45) = 0.0825 m,
#       再留 MOVE_RESERVE = 0.05 m 的垫子 ⇒ `MOVE_GAP_MIN = 0.133 m`。
#       (2026-10-08 前 SCAN_PERIOD 误写 0.18 ⇒ need(0.45)=0.126、MOVE_GAP_MIN=0.176;
#       下面 V/W 的历史实测数字都是那个口径下跑的,见 SCAN_PERIOD 定义处的订正。)
#       净空不够就不发指令(交回生产停、必要时倒车),不再"看着在动其实不动"。
# 纠正二(提前扫弧):不能等闸门判负、车停死了才在 neutral() 里第一次找方向。
#   `on_scan()`(每来一帧雷达 —— 车还在走、闸门还没判负)就把七条 ACTS 候选弧的净空
#   全算出来、缓存进 `node._arcs`、并按帧落盘 `<ROAD_DEBUG>.arcs`(节点自己的时钟,
#   判据自证)。判负那一拍直接**用这份提前扫好的数据**:
#     · 意图弧净空 ≥ MOVE_GAP_MIN ⇒ 沿意图弧走(只改速度,不动转向);
#     · 否则七条里挑净空最大的,只要 ≥ MOVE_GAP_MIN 就沿它走(这就是"切到能过闸门的弧");
#       已经在走的弧只要还够就继续走(不每帧换弧);
#     · 一条都不够 ⇒ 返回 None,交回生产(停,3 s 后倒车 —— 盲倒仍是最后手段)。
#   为什么"走"能解环(与 U 同一个机理):车一动,那颗锥相对扫掠弧就往车后滑,
#   `scan_guard.py` 的弧从 θ=0(后轴)开始扫,锥一过就出弧 ⇒ 净空立刻变大 ⇒ 闸门自己放行。
#   V = U 的全部上游层(投影/提前/预留)+ 把 U 的"蹭走"换成"换弧走",所以
#   `_use_creep_pos()` 仍然只对 U 生效,两条路互斥。
MOVE_V = float(os.environ.get('CORRIDOR_MOVE_V') or V_MIN)
MOVE_RESERVE = float(os.environ.get('CORRIDOR_MOVE_RESERVE') or 0.05)
MOVE_GAP_MIN = float(os.environ.get('CORRIDOR_MOVE_GAP_MIN')
                     or ((L_DECEL + SCAN_PERIOD) * MOVE_V
                         + U_DECEL * MOVE_V * MOVE_V + MOVE_RESERVE))
MOVE_PROGRESS_M = float(os.environ.get('CORRIDOR_MOVE_PROGRESS_M') or 0.10)
MOVE_SCAN_MAX_AGE = float(os.environ.get('CORRIDOR_MOVE_SCAN_MAX_AGE') or 0.30)
MOVE_MAX_SWING = float(os.environ.get('CORRIDOR_MOVE_MAX_SWING') or 0.0)   # 0 = 不限

# ---- W 臂:门槛回到闸门线 + 提前换道(2026-10-07,V 的实测结论)--------------------
# V 的实测(12 局,cap2.2,`runs/pin_report.py V 2.2 --all`):提前扫弧/换弧走**新层本身
#   工作正常**(`.arcs` 23655 行、`kind=move` 的 `v` 恒 0.4500、`g_sel` ≥0.177),但
#   **卡死没治好、A/C 上比 U 更差**:顶死段 13(U 3)、最长 34.78 s(U 14.14 s)、
#   steady 中位 16.70 s(U 16.10 s)、倒车 63 次/45.8 s/11.28 m(U 48/29.2/6.65)。
# 证据指向**两个具体缺陷**(下列 0.176 是当时口径;SCAN_PERIOD 改 1/12 后该常数现为 0.133,
#   见 arms/corridor_arm.py:106 —— 结论不受影响,W 仍取闸门线 0.25):
#   ① **门槛比闸门线低**:`MOVE_GAP_MIN=0.176`(= need(0.45)+0.05)比 `guard_stop=0.25`
#      低 0.074 m ⇒ 闸门已经说"这条弧不许走"之后,V 仍以 0.45 往里拱,一直拱到连 0.176
#      都测不到才停。这时车已经停在"七条弧全 <0.176"的死口袋里(`.arcs` 的 `kind=hand`
#      在 A_r3 有 597 帧 = 54 s,`g_want p50` 多为负),而 `_move_progress` 每 0.10 m
#      (0.45 m/s ⇒ 每 0.22 s)刷新一次 `block_t` ⇒ 3 s 倒车计时被反复清零,等真停下时
#      还得再干等 ~3 s 才倒、倒 1.5 s 又冲回同一口袋 ⇒ 长段被拉长(A_r3 一局倒车 50 次/
#      31.3 s/6.51 m、判负占比 51.6 %,正是"拱进-停-倒-再拱")。
#      用户 m14588 的原话本来就是「切到**能过闸门**的弧」= 净空 ≥ `guard_stop`。
#      ⇒ W 把门槛抬到 `_move_bar(node) = max(MOVE_GAP_MIN, guard_stop + MOVE_GAP_MARGIN)`
#      (`MOVE_GAP_MARGIN` 默认 0 ⇒ 0.25)。车停在闸门自己认可停的位置,不再拱进死角。
#   ② **没有提前量**:V 只在判负那一拍才第一次换弧,而 `runs/v_hand_probe.py` 显示
#      楔死时"最好那条弧"≥0.25 的拍数 = **0**(1281 hand 拍里 ≥0.176 也是 0)⇒ 一旦判负
#      就已经没有干净的弧可挑了;`runs/detour_probe.py V 2.2 --min 0.25` 又显示:44 个
#      楔死段首里,回看 1 帧有"合法替代弧(≥0.25 且转向差 ≥0.30)"的 15 个(34.1%),
#      其中**当时闸门还没判负**的 8 个(18.2%);回看 3~20 帧稳定在 ~20 %。
#      ⇒ W 加 `_detour()`:还在正常行驶(`publish()`)时就盯着意图弧净空,掉到
#      `DETOUR_TRIG=0.35` 以下且存在"明显更空且过得去"(≥`_move_bar`、比意图弧多
#      ≥`DETOUR_MARGIN=0.10`)的弧时,朝它**每拍最多挪 `DETOUR_STEP=0.05` 动作**,
#      整个事件累计不超过 `DETOUR_MAX_SWING=0.30`(= 0.12 rad,不是择线)。速度一字不改
#      (仍由限速/投影层按净空反解),所以"提前换道"改变的只有横向去向。
MOVE_GAP_MARGIN = float(os.environ.get('CORRIDOR_MOVE_MARGIN') or 0.0)
DETOUR_TRIG = float(os.environ.get('CORRIDOR_DETOUR_TRIG') or 0.35)
DETOUR_MARGIN = float(os.environ.get('CORRIDOR_DETOUR_MARGIN') or 0.10)
DETOUR_STEP = float(os.environ.get('CORRIDOR_DETOUR_STEP') or 0.05)
DETOUR_MAX_SWING = float(os.environ.get('CORRIDOR_DETOUR_MAX_SWING') or 0.30)
# v5.0_Y = v4.3_W 的层配置 + 仓内宿主（arms/arm_host.py，几何读 common/params）+ 几何修正。
# 见 ARM_LAYERS：Y 走 W 的分支，所以"层"这个变量被换了宿主这件事隔离掉了。
ARM_VERSION = 'v5.0_Y'

# ---- K 臂:倒车的**速度**不许跟着 cap 一起涨 -----------------------------------
# `road_policy_node.py:204-217` 的文档字符串写着"倒得短、倒得慢",但实现是
# `-self.cap * REVERSE_FRACTION`(sim 分支,`:214`),也就是**倒车速度正比于 cap**。
# 而 guard 只看车头(雷达在车头,背后看不见,`:209`),所以倒车是**盲的**。
# 实测(runs/reverse_budget.py,从 policy.csv 的实测 speed 积分,单位 m):
#   cap 1.2 A  每局 2.47 | cap 1.5 A 三局 7.70/7.26/4.74 | cap 1.8 A 19.26 / I 22.91
#   cap 3.0 A 18.88/17.50 | I 20.60 | narrow_01 一局 **65.50**(圈长才 40.88)
# 每次倒车的平均距离从 0.21 m(cap 1.5) 涨到 0.67 m(cap 3.0)。
# K 只把这件事改回注释里的原意:倒车速度取 min(cap, REV_ABS)。
# **REV_ABS = 1.5 ⇒ cap >= 1.5 时与 cap 1.5 的倒车行为逐字节相同**(不是新策略)。
REV_ABS = float(os.environ.get('CORRIDOR_REV_ABS') or 1.5)
_REV_FRACTION = RPN.REVERSE_FRACTION    # 宿主常量（旧件同名，0.5）

# ---- I 臂:把"停 + 倒车"换成"不停车换向" --------------------------------------
# 依据(见 HANDOFF 第十二节、b124):节点的控制律只有两支 —— 沿**策略自己那条弧**
# 往前 0.68m 无锥就走,否则停车,3 秒后盲倒 0.675m。而 `gap` 在 64~85% 的拍上已经
# < guard_stop ⇒ 车被反复冻住;冻住后姿态不变、观察不变 ⇒ 极限环(实测一局在 93.6s
# 里只挪了 10.65m,围在一个 1.23x0.17m 的球里)。
#
# G 的教训:`publish()` 只在 `gap >= guard_stop` 时才可达,所以"提前一档"在 gap 空间
# 里**没有窗口**(0.6% 的拍,实测 G 三局 0 次触发)。真正的杠杆不在"更早转",而在
# **"不要停"**:在闸门触发的那一拍就把 B 的可行弧搜索拿出来用,并且把 3 秒后的盲倒
# 也换成"继续慢速换向"。车只要在动,观测就在变,环就断了。
#
# 与 E 的性质完全不同:E 是拿走廊去**替策略择线**(失败);I 用的是 B 那套
# "在可行弧里挑离策略原意最近的一条"(筛选线 ESCAPE_MARGIN,实测 9 局 0 撞锥)。
KEEP_V_MAX = float(os.environ.get('KEEP_V_MAX') or 0.90)
KEEP_CLEAR_MARGIN = float(os.environ.get('KEEP_CLEAR_MARGIN') or ESCAPE_MARGIN)
# "最多许它把策略原意挪多远"(动作,1.0 = 满舵)。0 = 不限。
#   I 臂:不限 —— 只看"有没有别的路可走"。
#   J 臂:封顶 0.60(≈17 度舵角)—— 只许"微调",不许把策略原意翻面。
# runs/arm_i_smoke.py 的实测给了这条旋钮的存在理由:A r1 上 I 的 |da| p50 0.51 / p90 0.89,
# 且 45% 的触发要挪 0.6 以上;B r3 的明细里甚至出现 `want -1.00 -> a +1.00`(整根翻面)。
# 那种"翻面"就是 R=1.045m 那个弯的 cut 机理(见 HANDOFF 第十一节),所以必须有一个
# 能把它关掉的版本,否则 I 无法与"抄近路"区分开。
KEEP_MAX_SWING = float(os.environ.get('KEEP_MAX_SWING')
                       or (PREEMPT_MAX_SWING if ARM == 'J' else 0.0))

# ---- R 臂(v3.8):**不许停** —— 把策略的转向请求"投影"回 guard 可行的集合 ----------
# 用户 m13219/m13234 的要求:不要再谈抬 cap,先在 cap1.8 上**把卡死本身解决掉**;
# 而且明说"倒车卡死破坏那就是另辟蹊径"(m13209)⇒ 不许用倒车/破环来绕。
#
# 一步一步的因果链(证据都是实测,不是推测):
#   ① 节点拦停判据(`road_policy_node.py:240`)是 **策略当前要打的那个转角**的扫掠净空:
#      `gap = self.guard.gap(action_to_steer(steer))`,steer 来自策略输出。
#   ② `scan_guard._clear(steer)`(`scan_guard.py:61-80`)在 |steer|>=STRAIGHT_STEER 时不是
#      看正前方,而是看"后轴绕 R=WHEELBASE/|tan steer| 走圆"的**环带**
#      (|hypot(x, y-sR)-R| <= HALF_CORRIDOR=0.28 的点才算),`along = theta*R`。
#   ③ 于是 `gap` 会**跳变**:`pin_inspect.py` 在 armL_cap1.9_track_A_r3 上量到
#      t=9.30 `gap=+2.573` → t=9.35 `gap=-0.266 blocked=1` —— 0.05 s 一拍、车只走了
#      0.09 m,净空掉了 2.8 m。不是"前方 0.27 m 冒出锥",是**车一打舵,旁边那颗锥
#      就落进了这条扫掠环带**。撞点读数同向:被撞锥到后轴 0.444~0.485 m(横向),
#      而真实接触线是 0.175+0.14=0.315 m ⇒ 锥在**车侧**,不在车正前。
#   ④ 节点对这件事的反应是 `self.neutral()`(`:258`,全零 Twist:油门归零+转向回中),
#      然后 `if held < STUCK_SECONDS(3.0): return`。车不动 ⇒ 观测不变 ⇒ 策略输出不变
#      ⇒ 下一拍 gap 还是负的 ⇒ 再 neutral。**这是极限环,而环的成因就是那个"停"。**
#   ⑤ `:256-257` 的注释自己说明了为什么"停"还停不干净:截停时回中是为了不让 guard
#      按满舵算弧(回中会把 gap 报小 0.2 m),可回中之后车就彻底不动了。停是病,不是药。
#   ⑥ 3 秒后 `escape_reverse()` 盲倒(`-self.cap*REVERSE_FRACTION`),雷达在车头、
#      背后看不见 ⇒ 倒车这次盲动正是"擦到旁边那颗锥"的来源(见 §五 那一行)。
#
# 所以"避免卡死"的形态只有一种,而且不需要任何新信息:**在闸门判负的那一拍,不要停车,
# 而是把策略的转向请求投影到"guard 说可行"的集合里,再按解析刹车能停住的速度继续前进。**
# 这不是绕开约束,是把约束投影到动作空间(可行集 = {a : gap(a) >= guard_stop + margin}),
# 与生产行为唯一的区别是:生产在判负时用"停"来满足约束,这里用"换最近的可行动作"来满足。
# 车一直在动 ⇒ 观测一直在变 ⇒ 极限环在结构上不可能形成(不需要第 3 秒的盲倒)。
#
# 与 I 臂的区别(I 也是"不停车换向",但它输了,见 VERSIONS §五):
#   · I 在**整局**都可用 keep-moving,且用 `_best_escape` 的 7 条**离散**弧(门槛 0.15 m),
#     触发次数 21~29 次/局 —— 那是"第二个规划器",把线整个改掉(narrow 道上甚至搜不到
#     任何 0.15 m 净空的弧而全输)。
#   · R 只在**闸门判负那一拍**动手,用 `_min_swing` 的**连续最小摆动**(离策略原意最近),
#     并且速度按投影后的净空反解(不是固定 0.90)。策略意一旦可行,R 立即退出。
PROJECT_CLEAR_MARGIN = float(os.environ.get('CORRIDOR_PROJECT_MARGIN') or 0.15)
PROJECT_MAX_SWING = float(os.environ.get('CORRIDOR_PROJECT_MAX_SWING')
                          or PREEMPT_MAX_SWING)
PROJECT_STEP = float(os.environ.get('CORRIDOR_PROJECT_STEP') or 0.05)
PROJECT_V_MAX = float(os.environ.get('CORRIDOR_PROJECT_V_MAX') or 0.90)

# ---- S 臂(v3.9):**提前动手** —— 在闸门判负之前,就先把"正在逼近的那条弧"修开 ------
# R 在 cap1.8 上把 89 s 卡停那一类从"1/4 局"压到"2/10 局"(见 VERSIONS/REPORT),但
# `runs/r_approach.py` 把剩下那 2 局为什么崩给量出来了,判据不是"gap 有多负":
#   · 健康局(A r1,10 个判负段):每个段起点前 0.2 s 的净空都是 **inf**(或 2.0~2.3 m),
#     段长 0.00~0.10 s,段内最低速 1.28~1.78 ⇒ 判负只是"打舵时旁边那颗锥扫进环带"
#     **单拍假象**,下一拍投影就换开,车从没停过。
#   · 崩溃局(A r3,30 个判负段):六个 3~9.6 s 的长段,入口全是
#     **`g_in = +0.19…+0.24` 且 0.2 s 前也在 `+0.185…+0.254`** —— 净空**在闸门线附近
#     持续(≥0.2 s)并单调收缩**,车以 1.78 m/s 冲进去,0.25 m 内停不住 ⇒ 滑进
#     "锥桶已在车头下"的不可解区 ⇒ 投影**诚实**返回 None ⇒ 3 秒闸门 ⇒ 盲倒 ⇒ 长段。
#   · 对照 R36(arm A `C r1`,自己也有 28.8 s 卡停圈):同一签名(16.4 s/11.3 s 长段、
#     `g_in=0.157/0.148`)⇒ 这是**所有臂共有的病**,不是 R 引入的。
# 触发率实测(`runs/s_trigger_scan.py`,cap1.8 12 局 35922 拍):`gap < 0.60`(margin 0.35)
# 在健康局只占 **0.97~2.15 %** 的拍 ⇒ 这层"提前量"买得起。
# 形态:触发时**不停车**,把策略原意挪到"净空 ≥ guard_stop + PREARM_TARGET"的那条弧上,
# 摆幅上限与 R/G 同一个数(绝不许变成第二个规划器);挪不到理想净空就退一档,至少要
# 严格比现在好(PREARM_GAIN)。速度按**挪后那条弧**的净空反解(不是按策略原意的净空)。
PREARM_TRIG = float(os.environ.get('CORRIDOR_PREARM_TRIG') or 0.45)
PREARM_TARGET = float(os.environ.get('CORRIDOR_PREARM_TARGET') or 0.75)
PREARM_GAIN = float(os.environ.get('CORRIDOR_PREARM_GAIN') or 0.15)
PREARM_MAX_SWING = float(os.environ.get('CORRIDOR_PREARM_MAX_SWING')
                         or PREEMPT_MAX_SWING)
PREARM_STEP = float(os.environ.get('CORRIDOR_PREARM_STEP') or PROJECT_STEP)


def _use_corridor():
    return ARM in ('E', 'F')


def _use_clr_speed():
    """前视 clearance 限速开关(**独立于臂**,不依赖任何层配置)。

    做的事只有一件:用 `on_scan()` 解出的**前视走廊**净空 `clearance = w/2 - |c|` 反解
    一个速度上限,把策略的速度往下压。**只降速、不改转向**,所以即使误触发也只让车慢
    一点,不会把车带去别处。

    为什么需要它(2026-10-08):A 道死角的前向逃逸层全败,根因是 `gap(steer)` 是**沿当前
    转向弧**量的 —— 策略一转错方向,扫掠弧就指向锥桶、gap 立刻转负,限速层跟着失效。
    走廊量是**前视**的,与策略当拍选哪条弧无关,所以不受这个污染。

    为什么用 clearance 而不是走廊宽 `w`:A 的死点是**弯**不是**窄**(弯道处 `w` 正常,
    p50=1.20,`w<1.10` 只占 14.9%);崩溃前真正在掉的是**到最近墙的距离**
    `clearance = w/2 - |c|`(0.54→0.29→0.01,0.6 s 内)。

    默认关(逐位不变);置 `CORRIDOR_CLR_SPEED=1` 开。任何臂都能开,不写 `ARM == ...`。
    """
    return os.environ.get('CORRIDOR_CLR_SPEED') == '1'


# 前视 clearance 限速阈值(米):clearance >= FULL 不限;<= MIN 压到 V_MIN;中间线性。
# clearance = w/2 - |c| = 车轴到最近墙的距离(车半宽 0.175,所以 <0.175 就是已经压线)。
# 取 0.50/0.25:实测 A 道正常行驶 clearance p50=0.50(不限速),崩溃前掉到 0.29→0.01;
# 所以 FULL 设在正常值、MIN 设在"已经危险"处,直道不介入、进死角才收油。
CORRIDOR_CLR_FULL = float(os.environ.get('CORRIDOR_CLR_FULL') or 0.50)
CORRIDOR_CLR_MIN = float(os.environ.get('CORRIDOR_CLR_MIN') or 0.25)


def _need_corridor():
    """谁需要每帧解一次走廊。H 只要 `c` 的符号做同侧性否决,不要 E 那套常驻转向。
    L 只要 `c` 的**中位数**做有界偏置,同样不要纯追踪。
    clearance 限速层(见 `_use_clr_speed`)也只要 `w`/`c`,同样不要纯追踪。"""
    if _use_clr_speed():
        return True
    return ARM in ('E', 'F', 'H', 'L')


def _use_speed():
    return ARM in ('C', 'D', 'E', 'F')


def _use_speed_reserve():
    """N/P/T 臂:净空反解限速,但预留量与 guard 的硬闸线**解耦**(见 SAFE_RESERVE)。

    跟 `_use_speed()` 分开是因为那一条用的是 `node.guard_stop` —— 那是 guard 自己的
    闸线,不是我们想要的刹车余量。两条互斥。
    N:预留 = SAFE_RESERVE(默认 0.10,可用 `CORRIDOR_RESERVE` 覆盖)
    P:预留 = 0.25(= guard 自己的硬闸线,生产 C/D/E/F 的口径),写死不走环境变量。
    T:同 N(0.10),但和 `_use_project()`+`_use_prearm()` 叠在一起 —— 修的是抬档后
      "刹车预算不够却仍按原意全速直行 ⇒ 顶着锥磨"那条长判负段,见 ARM_VERSION 上面的块。
    U:同 T(v4.1_U ⊃ v4.0_T),再加"净空为正时限速蹭走",见 ARM_VERSION 上面的 U 块。
    V:同 U/... 的上游层(见 V 块);W:同 V(见 W 块)。
    """
    return ARM in ('N', 'P', 'T', 'U', 'V', 'W')


def _reserve():
    """本臂限速时使用的净空预留量(米)。"""
    return 0.25 if ARM == 'P' else SAFE_RESERVE


def _use_arc_escape():
    return ARM in ('B', 'D', 'F', 'G', 'H', 'I')


def _use_keep_moving():
    """I 臂:被拦的第一拍就换向继续走,不等车停、不盲倒。"""
    return ARM in ('I', 'J')


def _use_rev_ceiling():
    """K 臂:只把倒车速度封顶,别的什么都不动(对照组是生产行为 A)。"""
    return ARM == 'K'


def _use_bias():
    """L 臂:按**左右净空的差**给一个有界的侧向偏置(往更空的一侧)。v3.4 起不再看 `c`。"""
    return ARM == 'L'


def _use_preempt():
    return ARM in ('G', 'H')


def _use_project():
    """R 臂(以及 S 臂的前半):闸门判负时不停车,把策略的转向请求投影到 guard 可行的集合里再低速继续走。

    与 `_use_keep_moving()`(I/J)的区别见 R 的常量块注释:R 只在判负那一拍动手、
    只做连续最小摆动、速度按投影后的净空反解、策略意一可行就立刻退出。
    S(v3.9)= 这一层 **+  `_use_prearm()`**(在判负**之前**就换弧),两者共用同一个求解器
    形态与同一个摆幅上限,所以 R vs S 的差别就是纯粹的"早/晚"。
    T(v4.0)= S 再叠 `_use_speed_reserve()`(抬档后顶着锥磨的主因是"停不下来",见常量块)。
    U(v4.1)= T 再叠 `_use_creep_pos()`(投影 **None** 但净空为正时,沿策略自己的弧按
    guard 刹车律极低速走,不换弧也不倒车;见 U 的常量块)。
    V(v4.2)= T/U 的上游层全带 **+  `_use_move_arc()`**(把 U 的蹭走换成"提前扫好的七条弧里
    挑净空最大的那条、以 V_MIN 走");用户 m14584/m14588 的两条硬纠正,见 V 的常量块。
    W(v4.3)= V 的上游层不变,只改 V 那层的**门槛**(抬到 `guard_stop`,见 W 块)+ 新增
    `_use_detour()`(还在正常行驶时就提前换道)。这样 W vs V 的差 = 纯门槛差 + 纯提前量差。
    """
    return ARM in ('R', 'S', 'T', 'U', 'V', 'W')


def _use_detour():
    """W 臂:还在正常行驶(`publish()`)时,把转向朝"净空明显更大且过得去"的弧挪一点点。

    触发量是**意图弧自己的净空** `DETOUR_TRIG`(默认 0.35 = 闸门线 + 0.10),动手条件是
    "另有一条弧净空 ≥ `_move_bar()` 且比意图弧多 ≥ `DETOUR_MARGIN`(0.10)"。证据:
    `runs/detour_probe.py V 2.2 --min 0.25`(用 V 自己的 `.arcs` 在线读数回看)显示 44 个
    楔死段首里 18~20 % 在"闸门还没判负"时确实有一条合法替代弧 ⇒ 这层只能买到这 ~1/5,
    所以它只是**辅助**;真正的主力是 V 那层门槛抬回闸门线(见 W 常量块)。
    """
    return ARM == 'W'


def _use_prearm():
    """S 臂:在**闸门判负之前**(策略原意的净空掉进 PREARM_TRIG 带里)就换弧。

    与 `_use_project()`(R)的区别只在**时机**:R 是"已经被判负了,别停车";S 是"还没判负,
    但正在逼近 ⇒ 现在就把弧挪开"。求解器、摆幅上限、退出条件全部相同(策略意一可行
    立刻不动手)。见上面 S 的常量块:R 把卡停从 1/4 局压到 2/10 局,剩下的病例判据是
    "净空在闸门线附近持续 ≥0.2 s 并收缩",那正是"提前"要打的窗口。
    T(v4.0)也带这一层:T = S ∪ {净空反解限速},三个元组(`_use_project`、`_use_prearm`、
    `_use_speed_reserve`)全带上,只增不减 —— 这样 T vs S 的差就是纯限速层的差。
    U(v4.1)= T + 蹭走层,同样是只增不减(四个元组全带),这样 U vs T 的差就是纯蹭走层的差。
    V(v4.2)= 与 U 同源(投影/提前/预留三层全带),只是把蹭走层换成换弧走 —— 这样
    V vs U 的差就是纯"提前扫弧 + 只发 V_MIN"的差。
    W(v4.3)= 与 V 同源 ⇒ W vs V 的差只有两处:门槛抬到 `guard_stop` + `_use_detour()`。
    """
    return ARM in ('S', 'T', 'U', 'V', 'W')


def _corridor_side_ok(node, a):
    """H 臂才用的同侧性检查:走廊中心在 c 那边,换出来的弧也得往那边打。

    只用**符号**,不用大小 —— b115 已经证明 `c` 是一个横向偏差量(`c=(L+R)/2` 按构造
    在缝中间就是 0),不是一个可用的转向目标;E 臂拿它直接定转向,把车赶进了 A 臂从没
    卡过的新口袋(9.5,5.8)。这里只在 `|c| > 0.10` 时用它做一次否决,而且只在
    `_best_escape` 已经筛出可行弧之后,所以不可能把转向"抹平"。
    """
    corr = getattr(node, '_corr', None)
    if corr is None or not getattr(corr, 'corridor_valid', None):
        return True
    for k, x0 in enumerate(CORRIDOR_X0_CHAIN):
        if not corr.corridor_valid[k]:
            continue
        c = float(corr.c[k])
        w = float(corr.w[k])
        if not (math.isfinite(c) and math.isfinite(w)) or w <= 0.0 or w > CORRIDOR_W_MAX:
            continue
        if abs(c) <= 0.10:
            return True
        return (a * c) >= 0.0
    return True


# 动作 -> 转角的反查表。action_to_steer 是单调的(SERVO_CURVE 逐段递增),所以
# 201 点网格 + 线性插值就够,不用去解那条非线性曲线。
_A_GRID = np.linspace(-1.0, 1.0, 201)
_S_GRID = np.array([action_to_steer(float(a)) for a in _A_GRID])


def action_for_steer(delta):
    """前轮转角 rad -> 策略动作 ±1(与 action_to_steer 互逆)。"""
    return float(np.interp(max(_S_GRID[0], min(_S_GRID[-1], float(delta))), _S_GRID, _A_GRID))


def recent_min_gap(node, g):
    """最近 ``GAP_HOLD_N`` 个不同雷达帧里 gap 的最小值。

    单帧 gap 抖得很厉害:实测 sim 45.1..46.4 在 0.34 和 1.33 之间来回跳,而安全裕度
    不该跟着抖。取窗口最小值 = "最近半秒里最坏的那一眼",只会更保守,不会更激进。
    按 ``guard.stamp`` 分帧,所以 20 Hz 的 tick 不会把同一帧数进去多次。
    """
    stamp = getattr(node.guard, 'stamp', None)
    hist = getattr(node, '_gap_hist', None)
    if hist is None:
        hist = []
        node._gap_hist = hist
    if not hist or hist[-1][0] != stamp:
        hist.append((stamp, g))
        del hist[:-GAP_HOLD_N]
    elif g < hist[-1][1]:
        hist[-1] = (stamp, g)
    return min(v for _, v in hist)


def v_allow(gap, guard_stop):
    """让车能在 gap 之内停住的最大速度。

    约束: d(v) + SCAN_PERIOD*v <= gap - guard_stop,
    即留给"刹车 + 反应"的余量必须小于当前净空减去硬闸线。gap 跌到 guard_stop
    时返回 0 —— 车正好停在硬闸上,而不是撞上去再让 guard 拦。
    """
    room = gap - guard_stop
    if room <= 0.0:
        return 0.0
    b = L_DECEL + SCAN_PERIOD
    return (-b + math.sqrt(b * b + 4.0 * U_DECEL * room)) / (2.0 * U_DECEL)


def _score(node, a):
    """方向评分。**只给离线脚本(runs/arm_score_sweep.py / arm_offline_check.py)复用**,
    线上路径已经不靠它选方向了 —— 见 publish() 的注释。
    """
    g = node.guard.gap(action_to_steer(a))
    if g is None:
        return None
    return min(SCORE_CAP if math.isinf(g) else g, SCORE_CAP) - STEER_COST * abs(a)


def _corridor_min_clearance(node):
    """前视走廊里**最紧的那一点**:``CORRIDOR_X0_CHAIN`` 站位上 ``w/2 - |c|`` 的最小值(米)。

    ``w`` 是走廊净宽、``c`` 是中心偏移,所以单侧到墙的距离就是 ``w/2 - |c|``。取链上
    站位(0.8~1.2 m)的最小值 = "前方这段里,车轴离最近那堵墙最近还有多远",这正是限速
    该看的量:它同时包含"变窄"(w 掉)和"偏出中心"(|c| 涨)两种危险,A 道死点是后者。

    只用 `CORRIDOR_X0_CHAIN` 而不是全部站位:更近的 x0=0.4/0.6 落在车体自遮挡足迹里
    (见 `CORRIDOR_X0_CHAIN` 注释),更远的 1.6/2.0 两侧可见率只有 34%/4%,都不可信。
    拿不到有效站位返回 None(整层当拍不生效,宁可不改)。
    """
    est = getattr(node, '_corr', None)
    if est is None:
        return None
    xs = np.asarray(est.x0)
    ws = np.asarray(est.w)
    cs = np.asarray(est.c)
    valid = np.asarray(est.corridor_valid)
    clr = []
    for x0 in CORRIDOR_X0_CHAIN:
        k = int(np.argmin(np.abs(xs - x0)))
        if not bool(valid[k]):
            continue
        wk = float(ws[k])
        ck = float(cs[k])
        # 与 `_corridor_steer` 同口径的两道过滤:`w<=0` 是退化帧(切片里左右各一点、
        # 重合了,clearance 会算成假的 0 并误触发限速);`w>W_MAX` 是"视野里没墙",
        # 不是"路很宽"(见 CORRIDOR_W_MAX 注释)。漏了这两道,v3 直道被压 44% 的
        # 真凶就是前者。
        if not (math.isfinite(wk) and math.isfinite(ck)) or wk <= 0.0:
            continue
        if wk > CORRIDOR_W_MAX:
            continue
        d = wk / 2.0 - abs(ck)
        if math.isfinite(d):
            clr.append(float(d))
    return min(clr) if clr else None


def _corridor_steer(node):
    """走廊 -> (动作, c, w, x0)。拿不到两侧走廊时返回 None(= 完全交给策略)。

    纯追踪几何:目标点 ``(x0, c)`` 在**后轴系**里(原点 = 后轴中心,+x 车头,+y 车左),
    曲率 ``kappa = 2c / (x0^2 + c^2)``,前轮角 ``delta = atan(kappa * WHEELBASE)``。
    c 为正 = 走廊中心在车轴左侧 -> 左转 -> 正动作,与 action_to_steer 同号。
    """
    est = getattr(node, '_corr', None)
    if est is None:
        return None
    xs = np.asarray(est.x0)
    for x0 in CORRIDOR_X0_CHAIN:
        k = int(np.argmin(np.abs(xs - x0)))
        if not bool(est.corridor_valid[k]):
            continue
        c = float(est.c[k])
        w = float(est.w[k])
        if not (math.isfinite(c) and math.isfinite(w)) or w <= 0.0:
            continue
        if w > CORRIDOR_W_MAX:      # 见 CORRIDOR_W_MAX 的注释:这是"视野里没墙",不是"路很宽"
            continue
        delta = math.atan(2.0 * c / (x0 * x0 + c * c) * WHEELBASE)
        return action_for_steer(delta), c, w, x0
    return None


def _corridor_c_med(node):
    """L 臂用的走廊中心横向偏差:链上所有有效估计的**中位数**。

    为什么要中位数而不是链上第一个:``c`` 逐帧跳得很厉害(v2 记录过 |c| p90 到 0.31、
    个别帧报 3.3~4.7 m 的"路宽"),而 L 只取它的**符号** + 一个封顶幅度,再用链内中位数
    把单帧噪声压掉。返回 ``(c_med, w_med)`` 或 ``None``。
    """
    est = getattr(node, '_corr', None)
    if est is None:
        return None
    xs = np.asarray(est.x0)
    cs, ws = [], []
    for x0 in CORRIDOR_X0_CHAIN:
        k = int(np.argmin(np.abs(xs - x0)))
        if not bool(est.corridor_valid[k]):
            continue
        c = float(est.c[k])
        w = float(est.w[k])
        if not (math.isfinite(c) and math.isfinite(w)) or w <= 0.0:
            continue
        if w > CORRIDOR_W_MAX:        # "视野里没墙",不是"路很宽"(见常数注释)
            continue
        cs.append(c)
        ws.append(w)
    if not cs:
        return None
    return (float(np.median(cs)), float(np.median(ws)))


def _fmt(x):
    try:
        if x is None or not math.isfinite(float(x)):
            return ''
        return '%.4f' % float(x)
    except (TypeError, ValueError):
        return ''


_B_HEADER = 't,speed,g,gL,gR,asym,c,w,bias,a_in,a_out,reason\n'


def _b_dbg(node, g, gl, gr, asym, c, w, bias, a_in, a_out, reason):
    """L 臂诊断:``policy.csv.b``(缓冲=1,收局 SIGTERM 也落盘)。

    `g` 是**策略自己那条弧**的净空(只作参考:v3.3 用它当闸门,1.2 % 的时间才报警);
    `gL`/`gR` 是策略动作左右各挪 ``L2_DELTA`` 后的净空,`asym = gL - gR`,偏置由它定。
    """
    path = os.environ.get('ROAD_DEBUG')
    if not path:
        return
    try:
        if not hasattr(node, '_b_out'):
            node._b_out = open(path + '.b', 'w', buffering=1)   # noqa: SIM115
            node._b_out.write(_B_HEADER)
            node._b_t0 = node.now()
        node._b_out.write('%.3f,%.4f,%s,%s,%s,%.4f,%s,%s,%.4f,%.4f,%.4f,%s\n' % (
            node.now() - node._b_t0, float(node.speed),
            _fmt(g), _fmt(gl), _fmt(gr), asym, _fmt(c), _fmt(w),
            bias, a_in, a_out, reason))
    except (TypeError, ValueError, OSError):
        pass


def _gap_or(node, steer_a):
    """guard 对某个动作的净空;``None``/非有限一律回 ``L2_GAP_CAP``。

    "没有信息"和"很空"在比较左右时是同一件事:都表示"这一侧不挡路"。
    """
    g = node.guard.gap(action_to_steer(steer_a))
    if g is None or not math.isfinite(g):
        return L2_GAP_CAP
    return min(float(g), L2_GAP_CAP)


def _lateral_bias(node, steer_a):
    """L 臂的方向层:返回加过偏置的动作,并写一行诊断。**不动速度。**

    闸门(全部成立才动手,预登记 §三 逐字):
      ① 走廊估计有效;② 走廊窄(``w <= L2_W_MAX``);③ ``|asym| >= L2_ASYM_MIN``。
    偏置量 ``clamp(asym / L2_ASYM_FULL, ±1) * L2_MAX_SWING`` —— 朝**更空的那一侧**,
    幅度封顶(≤0.25 动作 ≈ 7.2°,小于一个动作格距)。

    v3.5_L3 追加第④条:``|asym| < L2_ASYM_MIN``(信号消失)时,若上一拍还留着偏置,
    就**沿用**它(reason=``hold``)最多 ``L3_HOLD_TICKS`` 拍;走廊失效或变宽立刻清掉。
    保持状态放在 ``node._l3_hold = (bias, 剩余拍)``,不在模块级——否则多实例会串。
    """
    a_in = float(steer_a)
    g = _gap_or(node, a_in)
    gl = _gap_or(node, max(-1.0, min(1.0, a_in + L2_DELTA)))
    gr = _gap_or(node, max(-1.0, min(1.0, a_in - L2_DELTA)))
    asym = gl - gr
    cm = _corridor_c_med(node)
    c = w = float('nan')
    bias = 0.0
    if cm is not None:
        c, w = cm
    held = getattr(node, '_l3_hold', None)
    if cm is None:
        reason = 'no_corr'
        held = None
    elif w > L2_W_MAX:
        reason = 'wide'
        held = None
    elif abs(asym) >= L2_ASYM_MIN:
        u = min(abs(asym) / L2_ASYM_FULL, 1.0)
        bias = L2_MAX_SWING * u * (1.0 if asym > 0.0 else -1.0)
        reason = 'bias'
        held = (bias, L3_HOLD_TICKS)
    elif held is not None and held[1] > 0:
        bias = held[0]
        reason = 'hold'
        held = (bias, held[1] - 1)
    else:
        reason = 'asym_small'
        held = None
    node._l3_hold = held
    a_out = max(-1.0, min(1.0, a_in + bias))
    _b_dbg(node, g, gl, gr, asym, c, w, bias, a_in, a_out, reason)
    return a_out


_orig_on_scan = RPN.RoadPolicyNode.on_scan


def _file_sha256(path):
    """算文件 sha256;读不到就返回 'unavailable'(诊断不许把跑批搞崩)。"""
    try:
        h = hashlib.sha256()
        with open(path, 'rb') as fh:
            for chunk in iter(lambda: fh.read(65536), b''):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return 'unavailable'


def _local_path(raw):
    """把 `.git`/`commondir` 里记的路径换成当前系统能打开的路径。

    这个仓库是 Windows 上建的 worktree,`.git` 里写的是 `D:\\...`;而跑批在 WSL 里,
    同一个盘挂在 `/mnt/d`。这里只做"Windows 盘符 → WSL 挂载点"的**通用**换算
    (盘符从文件里读,不写死机器路径);换出来的路径不存在就原样返回,让上层退到
    'unavailable'。
    """
    if os.path.exists(raw):
        return raw
    if len(raw) > 3 and raw[1] == ':' and raw[2] in ('\\', '/'):
        cand = '/mnt/%s/%s' % (raw[0].lower(), raw[3:].replace('\\', '/'))
        if os.path.exists(cand):
            return cand
    return raw


def _git_head():
    """当前 git HEAD 的短哈希;读不到就 'unavailable'。

    不走 `git` 命令:跑批在 WSL 里,而这个 worktree 的 `.git` 指向 Windows 路径,
    WSL 里执行 `git -C <ROOT>` 直接报 'not a git repository'。所以自己解析:
    `.git` 文件里的 `gitdir:` → `<gitdir>/HEAD` 的 `ref:` → worktree 的引用可能在
    `commondir` 指向的公共仓里(最后兜底 packed-refs)。这只是指纹,不许把跑批搞崩。
    """
    try:
        dot = os.path.join(ROOT, '.git')
        if os.path.isfile(dot):
            with open(dot, 'r') as fh:
                line = fh.readline().strip()
            if not line.startswith('gitdir:'):
                return 'unavailable'
            gdir = _local_path(line.split(':', 1)[1].strip())
        else:
            gdir = dot
        with open(os.path.join(gdir, 'HEAD'), 'r') as fh:
            head = fh.read().strip()
        if not head.startswith('ref:'):
            return head[:12] or 'unavailable'          # detached HEAD
        ref = head.split(':', 1)[1].strip()
        cdir = gdir
        common = os.path.join(gdir, 'commondir')
        if os.path.isfile(common):
            with open(common, 'r') as fh:
                common_path = fh.readline().strip()
            cdir = _local_path(common_path if os.path.isabs(common_path)
                               else os.path.join(gdir, common_path))
        for base in (gdir, cdir):
            ref_path = os.path.join(base, ref)
            if os.path.isfile(ref_path):
                with open(ref_path, 'r') as fh:
                    return fh.read().strip()[:12] or 'unavailable'
        with open(os.path.join(cdir, 'packed-refs'), 'r') as fh:
            for row in fh:
                parts = row.split()
                if len(parts) == 2 and parts[1] == ref:
                    return parts[0][:12]
        return 'unavailable'
    except OSError:
        return 'unavailable'


def _report_version(node):
    """把"这一局跑的是哪个版本"落到 `<ROAD_DEBUG>.ver` 并打一行日志。

    见 arms/VERSIONS.md 第二节:每个跑批目录都要能反查到臂文件的 SHA256。节点自己
    不会记这个,只能由臂自己写。没有 ROAD_DEBUG 就只打日志。

    2026-10-08:原来只写版本号 + 旋钮,规矩 4 要求的"臂文件
    SHA256 / git HEAD / 时间"一直没落进去 —— 文档写了、实现没做。现在补上
    `arm_file`/`arm_sha256`/`git_head`/`started`/`model_sha256`;模型 sha 从
    `sys.argv[1]`(宿主的 positional `model`)取,取不到写 'unavailable'(运行器
    另有 manifest.json 记全量 source_targets,那里一定查得到)。
    """
    text = ('%s arm=%s cap=%.3f guard=%.4f preempt_margin=%.3f speed_margin=%.3f '
            'corridor=%s clr_speed=%s gain=%.2f half_slice=%.2f wmax=%.2f '
            'keep=%s keep_v_max=%.2f keep_margin=%.2f keep_swing=%.2f '
            'lateral=%s l2_delta=%.2f l2_swing=%.2f l2_asym_min=%.2f '
            'l2_asym_full=%.2f l2_wmax=%.2f l3_hold_ticks=%d '
            'speed_n=%s safe_reserve=%.2f project=%s proj_margin=%.2f '
            'proj_swing=%.2f proj_vmax=%.2f prearm=%s prearm_trig=%.2f '
            'prearm_target=%.2f prearm_gain=%.2f prearm_swing=%.2f '
            'creep_pos=%s creep_gap_min=%.3f creep_vmax=%.2f creep_progress=%.2f '
            'move_arc=%s move_v=%.2f move_gap_min=%.3f move_reserve=%.2f '
            'move_progress=%.2f move_swing=%.2f move_gap_margin=%.3f move_bar=%.3f '
            'detour=%s detour_trig=%.2f detour_margin=%.2f detour_step=%.2f '
            'detour_swing=%.2f\n'
            % (ARM_VERSION, ARM_TAG, float(node.cap), float(node.guard_stop),
               PREEMPT_MARGIN, PREEMPT_SPEED_MARGIN, _use_corridor(),
               _use_clr_speed(),
               CORRIDOR_GAIN, CORRIDOR_HALF_SLICE, CORRIDOR_W_MAX,
               _use_keep_moving(), KEEP_V_MAX, KEEP_CLEAR_MARGIN,
               KEEP_MAX_SWING,
               _use_bias(), L2_DELTA, L2_MAX_SWING, L2_ASYM_MIN,
               L2_ASYM_FULL, L2_W_MAX, L3_HOLD_TICKS,
               _use_speed_reserve(), _reserve(),
               _use_project(), PROJECT_CLEAR_MARGIN,
               PROJECT_MAX_SWING, PROJECT_V_MAX,
               _use_prearm(), PREARM_TRIG, PREARM_TARGET, PREARM_GAIN,
               PREARM_MAX_SWING,
               _use_creep_pos(), CREEP_GAP_MIN, CREEP_V_MAX, CREEP_PROGRESS_M,
               _use_move_arc(), MOVE_V, MOVE_GAP_MIN, MOVE_RESERVE,
               MOVE_PROGRESS_M, MOVE_MAX_SWING, MOVE_GAP_MARGIN, _move_bar(node),
               _use_detour(), DETOUR_TRIG, DETOUR_MARGIN, DETOUR_STEP,
               DETOUR_MAX_SWING))
    model = sys.argv[1] if len(sys.argv) > 1 else ''
    text += ('arm_file=%s arm_sha256=%s git_head=%s started=%s model_sha256=%s\n'
             % (os.path.relpath(os.path.abspath(__file__), ROOT),
                _file_sha256(os.path.abspath(__file__)), _git_head(),
                time.strftime('%Y-%m-%d %H:%M:%S'),
                _file_sha256(model) if os.path.isfile(model) else 'unavailable'))
    logging.getLogger('road_policy_node').info('实验臂 %s' % text.strip())
    path = os.environ.get('ROAD_DEBUG')
    if path:
        try:
            with open(path + '.ver', 'w') as fh:
                fh.write(text)
        except OSError:
            pass


def on_scan(self, m):
    """每来一帧就解一次走廊,缓存在 self._corr。

    解算放在这里而不是 publish() 里:雷达帧率(约 10 Hz)低于 tick(20 Hz),
    同一帧会经过 publish() 两次,没必要重算。解算失败(异常)退回 None,
    整个 E 层当拍不生效 —— 宁可不改,也不能拿半截估计去开车。
    """
    _orig_on_scan(self, m)
    if not getattr(self, '_ver_done', False):
        self._ver_done = True
        _report_version(self)
    if _use_move_arc():
        # V 臂的"提前扫":**在还干净、车还在走的每一帧**就把七条候选弧的净空算出来
        # (用户 m14588)。放在 on_scan 而不是 publish 里,是因为闸门判负那一拍
        # `tick()` 直接 `return`、根本不会调 publish(`road_policy_node.py:250-263`),
        # 而 on_scan 每帧都来 ⇒ 判负期间这份扫描也一直是新鲜的。
        try:
            _scan_arcs(self, 'scan')
        except Exception:                                        # noqa: BLE001
            pass
    if not _need_corridor():
        return
    try:
        self._corr = corridor.extract(
            self.scan, m.angle_min, m.angle_increment,
            range_min=m.range_min, range_max=m.range_max,
            half_slice=CORRIDOR_HALF_SLICE)
    except Exception:                                            # noqa: BLE001
        self._corr = None


RPN.RoadPolicyNode.on_scan = on_scan


def _e_dbg(node, v_in, v_out, policy_a, corr, out_a):
    """E/F 臂自己的诊断表:打在 policy.csv 旁边(`policy.csv.e`)。

    节点自带的那张 CSV 记的是**策略原始输出**(a0/a1/steer_rad),看不见我们改成了什么,
    排查时必须两张对着看。没设 ROAD_DEBUG 就整个不写,不影响正常跑。
    """
    if not hasattr(node, '_e_out'):
        node._e_out = None
        node._e_t0 = None
        path = os.environ.get('ROAD_DEBUG')
        if path:
            try:
                f = open(path + '.e', 'w', buffering=1)
                f.write('t,speed,v_in,v_out,policy_a,corr_a,out_a,c,w,x0\n')
                node._e_out = f
            except OSError:
                node._e_out = None
    f = node._e_out
    if f is None:
        return
    if node._e_t0 is None:
        node._e_t0 = node.now()
    if corr is None:
        a_c, c, w, x0 = '', '', '', ''
    else:
        a_c, c, w, x0 = '%.4f' % corr[0], '%.4f' % corr[1], '%.4f' % corr[2], '%.2f' % corr[3]
    try:
        f.write('%.3f,%.4f,%.4f,%.4f,%.4f,%s,%.4f,%s,%s,%s\n' % (
            node.now() - node._e_t0, float(node.speed), v_in, v_out,
            float(policy_a), a_c, float(out_a), c, w, x0))
    except (TypeError, ValueError):
        pass


_PRE_HEADER = 't,speed,g_want,g_new,a_want,a_new,v_out\n'


_PROJ_HEADER = 't,want,a,g,v,kind\n'


def _proj_dbg(node, kind, want, a, g, v):
    """R 臂诊断:每投影一次写一行 `<ROAD_DEBUG>.proj`。

    R 的判据是"卡死次数"要掉到 0(`runs/reverse_budget.py` 的 rev#),而在内存里的
    `_proj_n` 一局跑完就没了 ⇒ 必须落盘。`kind` 区分两条来路:block=闸门判负那一拍、
    stuck=满 3 秒本该盲倒那一拍。行数就是"本该停车/倒车的拍数",grep 即可。
    """
    path = os.environ.get('ROAD_DEBUG')
    if not path:
        return
    try:
        if not hasattr(node, '_proj_out'):
            node._proj_out = open(path + '.proj', 'w', buffering=1)   # noqa: SIM115
            node._proj_t0 = node.now()
            node._proj_out.write(_PROJ_HEADER)
        node._proj_out.write('%.3f,%.4f,%.4f,%.4f,%.4f,%s\n' % (
            node.now() - node._proj_t0, float(want), float(a), float(g),
            float(v), kind))
    except (TypeError, ValueError, OSError):
        pass


def _pre_dbg(node, g_want, g_new, a_want, a_new, v_out):
    path = os.environ.get('ROAD_DEBUG')
    if not path:
        return
    try:
        if not hasattr(node, '_pre_out'):
            node._pre_out = open(path + '.pre', 'w', buffering=1)   # noqa: SIM115
            node._pre_out.write(_PRE_HEADER)
            node._pre_t0 = node.now()
        node._pre_out.write('%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n' % (
            node.now() - node._pre_t0, float(node.speed), g_want, g_new,
            a_want, a_new, v_out))
    except (TypeError, ValueError, OSError):
        pass


_PA_HEADER = 't,kind,g0,g_new,a_want,a_new,v_out\n'


def _pa_dbg(node, kind, g0, g_new, a_want, a_new, v_out):
    """S 臂诊断:每触发一次写一行 `<ROAD_DEBUG>.prearm`(**含失败**)。

    R 的 `.proj` 只在成功时落盘,导致"崩的两局为什么投影没成功"只能靠离线复算猜
    (见 r_replay_probe.py 的口径差)——S 这次把 `miss` 也记下来:`kind` 为
    `hit1`(挪到理想净空)/`hit2`(退一档)/`miss`(摆幅内无解)。
    """
    path = os.environ.get('ROAD_DEBUG')
    if not path:
        return
    try:
        if not hasattr(node, '_pa_out'):
            node._pa_out = open(path + '.prearm', 'w', buffering=1)   # noqa: SIM115
            node._pa_t0 = node.now()
            node._pa_out.write(_PA_HEADER)
        node._pa_out.write('%.3f,%s,%.4f,%.4f,%.4f,%.4f,%.4f\n' % (
            node.now() - node._pa_t0, kind, float(g0), float(g_new),
            float(a_want), float(a_new), float(v_out)))
    except (TypeError, ValueError, OSError):
        pass


def _min_swing(node, want):
    """从策略原意出发,朝可行方向走**最小**的一步。返回 (action, gap) 或 None。

    为什么不直接用 `_best_escape`(`_best_escape` 取"离原意最近的**离散**弧"):离散候选
    只有七条,最近那条可能离可行边界还有 0.3 个动作,而真正需要挪的也许只有 0.05。
    这里在 `want -> 最近的可行离散弧` 之间以 PREEMPT_STEP 线性试探,取第一个过线的。
    两个闸门:**净空过 `guard_stop + PREEMPT_CLEAR_MARGIN`**,以及**挪动不超过
    PREEMPT_MAX_SWING** —— 后者是这层不变成"第二个规划器"的硬保证。
    """
    cands = []
    for a in ACTS:
        g = node.guard.gap(action_to_steer(a))
        if g is None or math.isnan(g):
            continue
        eff = SCORE_CAP if math.isinf(g) else g
        if eff >= node.guard_stop + PREEMPT_CLEAR_MARGIN:
            cands.append(a)
    if not cands:
        return None
    a_far = min(cands, key=lambda a: abs(a - want))
    sign = 1.0 if a_far >= want else -1.0
    n = int(abs(a_far - want) / PREEMPT_STEP)
    for k in range(0, n + 1):
        a = max(-1.0, min(1.0, want + sign * k * PREEMPT_STEP))
        if k > 0 and abs(a - want) > PREEMPT_MAX_SWING + 1e-9:
            break
        if ARM == 'H' and not _corridor_side_ok(node, a):
            continue
        g = node.guard.gap(action_to_steer(a))
        if g is None or math.isnan(g):
            continue
        eff = SCORE_CAP if math.isinf(g) else g
        if eff >= node.guard_stop + PREEMPT_CLEAR_MARGIN:
            return a, eff
    return None


def _project(node, want):
    """把策略的转向请求投影到"guard 说可行"的集合里。返回 (action, gap) 或 None。

    可行集 = {a : gap(action_to_steer(a)) >= guard_stop + PROJECT_CLEAR_MARGIN}。
    从 `want`(策略原意)出发按 PROJECT_STEP 向两侧**对称**试探,取**第一个**可行的,
    也就是离原意最近的那条弧;最多挪 PROJECT_MAX_SWING(= 与 G 同一个上限、同一个理由:
    这层绝不许变成第二个规划器,见 `_min_swing` 的注释)。

    为什么取"第一个可行"而不是"净空最大":净空最大会把它变回择线器 —— 开阔处七条弧
    同分,`-|a|` 就会独占决定权、把舵掰直(这正是 B 臂 off_track 1040/1400 的机理)。
    "第一个可行"的性质是**最小介入**:策略原意一旦可行,这里立刻返回 None,调用方
    随即回到原行为,所以 R 在整局里只在"闸门判负"的那些拍上存在。

    返回 None 有三种含义:雷达不新鲜/NaN(不动手)、原意本来可行(不动手)、
    摆动上限内确实找不到任何可行弧(调用方退回生产行为:停,3 秒后倒车)。
    """
    g0 = node.guard.gap(action_to_steer(want))
    if g0 is None or math.isnan(g0):
        return None
    if math.isinf(g0) or g0 >= node.guard_stop + PROJECT_CLEAR_MARGIN:
        return None
    n = int(PROJECT_MAX_SWING / PROJECT_STEP + 1e-9)
    for k in range(1, n + 1):
        for s in (1.0, -1.0):
            a = max(-1.0, min(1.0, want + s * k * PROJECT_STEP))
            if abs(a - want) > PROJECT_MAX_SWING + 1e-9:
                continue
            g = node.guard.gap(action_to_steer(a))
            if g is None or math.isnan(g):
                continue
            eff = SCORE_CAP if math.isinf(g) else g
            if eff >= node.guard_stop + PROJECT_CLEAR_MARGIN:
                return a, eff
    return None


def _project_between(node, want, target, gain=0.0):
    """把策略原意挪到"净空至少 target"的那条弧上。返回 (action, eff, stage) 或 None。

    与 `_project` 逐字相同的求解形态(从 want 按 PREARM_STEP 向两侧对称试探、取第一个
    过线的、摆幅不过 PREARM_MAX_SWING),只换了**判据**:`_project` 是"低于闸线才救火",
    这里是"还没到闸线、但已经在逼近就提前修"。

    两级目标:
      stage 1 —— 净空 >= target(理想:把这条弧整个挪出逼近段);
      stage 2 —— 摆动上限内到不了 target,就退而求其次,要求净空
                 >= max(g0 + gain, guard_stop + PROJECT_CLEAR_MARGIN),
                 也就是**至少严格比现在好一点**,绝不接受"挪了等于没挪"。
    两级都到不了就返回 None(调用方原样放行,不改变既有行为)。
    """
    g0 = node.guard.gap(action_to_steer(want))
    if g0 is None or math.isnan(g0):
        return None
    if math.isinf(g0):
        return None
    if g0 >= target:
        return None          # 已经达标:这层不工作(调用方应当先用 trig 判断)
    floor = max(g0 + gain, node.guard_stop + PROJECT_CLEAR_MARGIN)
    n = int(PREARM_MAX_SWING / PREARM_STEP + 1e-9)
    cands = []
    for k in range(1, n + 1):
        for s in (1.0, -1.0):
            a = max(-1.0, min(1.0, want + s * k * PREARM_STEP))
            if abs(a - want) > PREARM_MAX_SWING + 1e-9:
                continue
            g = node.guard.gap(action_to_steer(a))
            if g is None or math.isnan(g):
                continue
            eff = SCORE_CAP if math.isinf(g) else g
            if eff >= target:
                return a, eff, 1
            if eff >= floor and not cands:
                cands.append((a, eff))
    if cands:
        return cands[0][0], cands[0][1], 2
    return None


def _project_v(node, g):
    """投影后那条弧允许的速度:解析刹车反解,封顶 PROJECT_V_MAX、下限 V_MIN。

    `v_allow` 的第二个参数用 **guard 自己的闸线**(不是 0.25 预留量):投影已经保证
    这条弧在闸线之外,所以"刹车能停在闸线"就是这条弧的正确速度口径(与 N/P 的区别:
    N/P 是**不换弧**、靠预留量保证,所以要用更保守的线)。
    """
    return min(PROJECT_V_MAX, max(v_allow(g, node.guard_stop), V_MIN))


def _use_creep_pos():
    """U 臂:投影无解但**净空为正**的那一拍,沿策略自己的弧按 guard 刹车律蹭走。"""
    return ARM == 'U'


def _use_move_arc():
    """V 臂(以及 W):判负那一拍用**提前扫好的七条弧**决定"沿哪条弧以 `MOVE_V` 走"。

    W 用同一个实现,差别只在门槛(见 `_move_bar()`):W 要求"能过闸门"。
    """
    return ARM in ('V', 'W')


def _move_bar(node):
    """允许"以 `MOVE_V` 走"的净空门槛(米)。

    V: `MOVE_GAP_MIN` = 0.133(纯刹车账 need(0.45)=0.0825 + 垫子 0.05)。
    W: `max(0.133, guard_stop + MOVE_GAP_MARGIN)`(默认 0 ⇒ 0.25)= **闸门自己那条线**;
       ⇒ 改 SCAN_PERIOD 不动 W 的门槛(W 取的是 0.25 那一项),只动 V 的。
       用户 m14588 的原话是"切到**能过闸门的**弧",而闸门的判据就是 `gap < guard_stop`
       ⇒ 门槛低于闸门线时,"我在走"与"闸门说这条弧不许走"会同时成立,那正是 V 拱进
       死口袋的机制(W 常量块 ①)。
    """
    try:
        gs = float(node.guard_stop)
    except (TypeError, ValueError, AttributeError):
        gs = 0.0
    return max(MOVE_GAP_MIN, gs + MOVE_GAP_MARGIN)


def _creep_progress(node):
    """U 臂:判负期间**真的在动**就把 3 s 计时刷新一次(生产那条规则的前提是"车不动")。

    只在"这一拍我们又发了一个正速度"时被调用;累计位移按实测 speed 积分。走够
    `CREEP_PROGRESS_M` 米才刷新,走不动就一动不动地攒着 ⇒ 攒不满就永远到不了刷新条件,
    生产照旧在 3 s 后倒车(安全网不动)。`_u_renew` 只用于诊断。
    """
    now = node.now()
    last = getattr(node, '_u_last_t', None)
    node._u_last_t = now
    if last is None:
        return
    dt = now - last
    if dt <= 0.0 or dt > 1.0:        # 断流/时钟跳变:这一拍不计
        return
    node._u_moved = getattr(node, '_u_moved', 0.0) + abs(float(node.speed)) * dt
    if node._u_moved >= CREEP_PROGRESS_M:
        node.block_t = now
        node._u_moved = 0.0
        node._u_renew = getattr(node, '_u_renew', 0) + 1


def _creep_pos(node):
    """U 臂:净空为**正**时的低速蹭走。返回 (action, gap, v) 或 None。

    绝不改转向:`a` 恒等于 `want`(策略原意)⇒ 这一层只回答"能不能用这个速度沿**这条弧**
    走",不回答"该往哪走"(那是 `_project`/`_prearm` 的事,也就不会变成第二个规划器)。

    返回 None 的四种含义:
      · 雷达不新鲜/NaN(不动手);
      · `gap >= guard_stop` —— 闸门本来就不该拦,这一层不在这种拍上动手;
      · `gap <= CREEP_GAP_MIN` —— 真的在碰撞航向上/已经贴住 ⇒ 交回生产(停,必要时倒车);
      · `gap` 为 inf ⇒ 这条路上本来就不该走到这里,交回生产。
    速度恒取 `v_allow(gap, CREEP_RESERVE=0.0)`:净空越小走得越慢,净空一归零速度就归零。
    """
    want = float(node.last_action[0])
    g = node.guard.gap(action_to_steer(want))
    if g is None or math.isnan(g) or math.isinf(g):
        return None
    if g >= node.guard_stop:
        # 这一层只活在**闸门线以内**:闸门不判负的拍本来走不到这里(防御性早退,
        # 也保证"净空够宽却停着"这种异常不会变成我们的责任)。
        return None
    if g <= CREEP_GAP_MIN:
        return None
    v = min(CREEP_V_MAX, v_allow(g, CREEP_RESERVE))
    if v <= 0.0:
        return None
    return want, g, v


# ---- V 臂:提前扫七条弧 + 只发 V_MIN -------------------------------------------
_ARC_HEADER = ('t,frame,kind,want,g_want,a_sel,g_sel,n_ok,'
               'g_-1.0,g_-0.6,g_-0.3,g_0.0,g_0.3,g_0.6,g_1.0\n')


def _move_ok(node, g):
    """这条弧的净空够不够"以 `MOVE_V` 走":`g >= _move_bar(node)`。

    门槛由 `_move_bar()` 给(V 是刹车账 0.133,W 是闸门线 0.25)—— 这一处替换就是
    W 相对 V 的**门槛修正**的落点(`_scan_arcs` 的 `n_ok`、`_move_arc` 的三处判断
    全走这里,不会有第二个口径)。

    `None`(雷达不新鲜)与 NaN 一律算不够 —— 宁可交回生产停,也不拿半截估计去开车。
    inf(MAX_LOOKAHEAD 之外全空)当然算够。
    """
    if g is None:
        return False
    try:
        f = float(g)
    except (TypeError, ValueError):
        return False
    if math.isnan(f):
        return False
    return f >= _move_bar(node)


def _arcs_dbg(node, kind, sc, a_sel=None, g_sel=None):
    """V 臂诊断:每扫一次写一行 `<ROAD_DEBUG>.arcs`(七条弧各是多少净空 + 选了哪条)。

    这是"提前扫弧"这件事的**判据自证**:被拦拍的七条弧净空全在这里,用节点自己的
    `now()` 时钟(不依赖 `scan_frames.jsonl` 那个对不上的 sim 时钟 —— 见
    `runs/arc_align.py` 的实测:帧时钟与 policy.csv 的 t 差 0.69 m 量级,不可用)。
    `kind`:scan = `on_scan()` 每帧的例行扫描;move = 判负拍真的沿某条弧走了;
    hand = 判负拍七条都不够,交回生产。
    """
    path = os.environ.get('ROAD_DEBUG')
    if not path:
        return
    try:
        if not hasattr(node, '_arcs_out'):
            node._arcs_out = open(path + '.arcs', 'w', buffering=1)   # noqa: SIM115
            node._arcs_t0 = getattr(node, '_arcs_t0', node.now())
            node._arcs_out.write(_ARC_HEADER)
        gs = sc['gs']
        best = sc['best']
        if a_sel is None:
            a_sel, g_sel = (best if best is not None else (float('nan'), None))
        cells = []
        for a in ACTS:
            g = gs.get(a)
            cells.append('' if g is None else '%.4f' % g)
        node._arcs_out.write('%.3f,%.3f,%s,%.4f,%s,%s,%s,%d,%s\n' % (
            node.now() - node._arcs_t0, sc['frame'], kind, float(sc['want']),
            '' if sc['g_want'] is None else '%.4f' % sc['g_want'],
            '%+.2f' % float(a_sel), '' if g_sel is None else '%.4f' % float(g_sel),
            sc['n_ok'], ','.join(cells)))
    except (TypeError, ValueError, OSError):
        pass


def _scan_arcs(node, log_kind=None):
    """V 臂:**提前**把七条 `ACTS` 候选弧的净空全算出来(每来一帧雷达一次)。

    调用点只有一个:`on_scan()` —— 也就是"车还在走、闸门还没判负"的那一刻。结果缓存在
    `node._arcs`(`MOVE_SCAN_MAX_AGE` 秒内不重算,同一帧只扫一次),判负那一拍直接取用;
    这样"有没有别的路"是在**还来得及的时候**知道的,不是等车停死了才第一次去看。

    返回 `dict(t, frame, want, g_want, gs, n_ok, best)`:
      · `want`/`g_want` = 策略原意那条弧(动作是连续的,单独算一次);
      · `gs[a]` = 七条离散弧各自的净空(`None` = 雷达不给值);
      · `n_ok` = 净空 ≥ `MOVE_GAP_MIN`、够以 `MOVE_V` 走的弧数;
      · `best` = `(动作, 净空)` 里净空最大的那条,全 `None` 时是 `None`。
    """
    now = node.now()
    sc = getattr(node, '_arcs', None)
    if sc is not None and (now - sc['t']) <= MOVE_SCAN_MAX_AGE:
        if log_kind is not None:
            _arcs_dbg(node, log_kind, sc)
        return sc
    gs = {}
    for a in ACTS:
        try:
            g = node.guard.gap(action_to_steer(a))
        except Exception:                                        # noqa: BLE001
            g = None
        if g is not None and math.isnan(g):
            g = None
        gs[a] = g
    want = float(node.last_action[0])
    g_want = None
    try:
        g_want = node.guard.gap(action_to_steer(want))
    except Exception:                                            # noqa: BLE001
        g_want = None
    if g_want is not None and math.isnan(g_want):
        g_want = None
    if g_want is not None and math.isinf(g_want):
        g_want = math.inf
    n_ok = sum(1 for a in ACTS if _move_ok(node, gs[a]))
    best = None
    for a in ACTS:
        if gs[a] is None:
            continue
        if best is None or gs[a] > best[1]:
            best = (float(a), float(gs[a]))
    sc = dict(t=now, frame=now - getattr(node, '_arcs_t0', now),
              want=want, g_want=g_want, gs=gs, n_ok=n_ok, best=best)
    node._arcs = sc
    if log_kind is not None:
        _arcs_dbg(node, log_kind, sc)
    return sc


def _move_progress(node):
    """V 臂:判负期间**真的在动**就把 3 s 计时刷新一次 —— 与 U 的 `_creep_progress`
    同一个理由(生产那条规则的前提是"车不动"),只是距离换成 `MOVE_PROGRESS_M`、
    计数器换成本层自己的 `_mv_moved`(两个臂的账互不干扰)。走不动就不刷新 ⇒
    攒不满就永远到不了刷新条件,生产照旧 3 s 后倒车(安全网不动)。"""
    now = node.now()
    last = getattr(node, '_mv_last_t', None)
    node._mv_last_t = now
    if last is None:
        return
    dt = now - last
    if dt <= 0.0 or dt > 1.0:        # 断流/时钟跳变:这一拍不计
        return
    node._mv_moved = getattr(node, '_mv_moved', 0.0) + abs(float(node.speed)) * dt
    if node._mv_moved >= MOVE_PROGRESS_M:
        node.block_t = now
        node._mv_moved = 0.0
        node._v_renew = getattr(node, '_v_renew', 0) + 1


def _move_arc(node):
    """V 臂:用**提前扫好的**七条弧,决定判负这一拍沿哪条弧以 `MOVE_V`(=V_MIN)走。

    返回 `(action, gap, v)` 或 `None`。规则:
      ① 意图弧净空 ≥ `MOVE_GAP_MIN` ⇒ 沿意图弧走 —— **只改速度,不动转向**;
      ② 否则七条 `ACTS` 里挑净空最大的那条,够就沿它走(这就是"切到能过闸门的弧");
         正在走的那条(`node._mv_a`)只要还够就继续走(避免每帧换弧);
      ③ 一条都不够 ⇒ `None`,交回生产(停,3 s 后倒车 —— 盲倒仍是最后手段)。
    速度恒为 `MOVE_V`:低于 0.45 的指令车里根本不动(用户 m14584 的实测)。
    """
    sc = _scan_arcs(node)
    want = float(node.last_action[0])
    if _move_ok(node, sc['g_want']):
        node._mv_a = None
        return want, float(sc['g_want']), MOVE_V
    hold = getattr(node, '_mv_a', None)
    if hold is not None:
        gh = sc['gs'].get(float(hold))
        if _move_ok(node, gh):
            return float(hold), float(gh), MOVE_V
    best = sc['best']
    if best is None or not _move_ok(node, best[1]):
        return None
    a, g = float(best[0]), float(best[1])
    if MOVE_MAX_SWING > 0.0 and abs(a - want) > MOVE_MAX_SWING:
        return None
    return a, g, MOVE_V


def _detour(node, v, steer_a):
    """W 臂:还在**正常行驶**的这一拍,就把转向朝"净空明显更大且过得去"的弧挪一点点。

    这叫"提前换道"(用户 m~14891 选定的方向):V 只在闸门判负那一拍才第一次换弧,而
    `runs/v_hand_probe.py` 显示楔死时"最好那条弧 ≥0.25"的拍数是 **0** —— 一旦判负就
    已经没有干净的弧可挑了;`runs/detour_probe.py V 2.2 --min 0.25` 则显示 44 个楔死段首
    里有 8~9 个(18~20 %)在"闸门还没判负"时确实存在合法替代弧 ⇒ 这就是这层能买到的东西
    (只有 ~1/5,所以它是**辅助**,主力是 `_move_bar()` 把门槛抬回闸门线)。

    判据(全部可关:`CORRIDOR_DETOUR_TRIG/MARGIN/STEP/MAX_SWING`):
      ① 当前点的净空 `g0`(拿 `a0` 现问一次 guard)掉到 `DETOUR_TRIG`(0.35)以下;
      ② 七条弧里最好的一条 **≥ `_move_bar(node)`(能过闸门)** 且 **比 `g0` 多 ≥
         `DETOUR_MARGIN`(0.10)** —— 两个都成立才算"明显更空";
      ③ 朝它挪 `min(|Δa|, DETOUR_STEP(0.05), 剩余摆幅)`,一次进逼事件累计不超过
         `DETOUR_MAX_SWING`(0.30 动作 ≈ 0.12 rad);净空回到 `DETOUR_TRIG + DEADBAND`
         以上、或需要往反方向修时,摆幅账清零。

    **速度一字不改**:横向去向改一点,纵向仍由限速/投影层按净空反解 ⇒ 这层不会自己
    把车开快。命中落盘 `kind=detour`(`.proj` 记 `a0/a_out/净空`、`.arcs` 记七弧读数)。
    """
    a0 = float(steer_a)
    sc = _scan_arcs(node)
    gw = sc['g_want']
    if gw is None:
        node._dt_swing = 0.0
        return a0
    if gw >= DETOUR_TRIG:
        node._dt_swing = 0.0
        return a0
    spent = float(getattr(node, '_dt_swing', 0.0))
    try:
        g0 = node.guard.gap(action_to_steer(a0))
    except Exception:                                            # noqa: BLE001
        g0 = None
    if g0 is None or math.isnan(g0):
        return a0
    best = sc['best']
    bar = _move_bar(node)
    if best is None or best[1] < bar or best[1] < g0 + DETOUR_MARGIN:
        return a0
    d = float(best[0]) - a0
    if abs(d) <= 1e-6:
        node._dt_swing = 0.0
        return a0
    if spent * d < 0.0:                 # 要往反方向修:上次那笔账作废
        spent = 0.0
    room = DETOUR_MAX_SWING - abs(spent)
    if room <= 1e-6:
        return a0
    step = math.copysign(min(abs(d), DETOUR_STEP, room), d)
    node._dt_swing = spent + step
    a_out = max(-1.0, min(1.0, a0 + step))
    node._dt_n = getattr(node, '_dt_n', 0) + 1
    _proj_dbg(node, 'detour', a0, a_out, float(best[1]), float(v))
    _arcs_dbg(node, 'detour', sc, a_out, float(best[1]))
    return a_out


def _preempt(node, v, steer_a):
    """G/H 臂的提前层:在**被停之前那一拍**换弧 + 把速度压到自己刹得住的上限。

    返回新的 v;换出来的方向放在 node._pre_a 上(没换就是原样)。调用点在 publish()
    内、`_orig_publish` 之前,也就是 `tick()` 的 `:240 gap = self.guard.gap(...)` 之前
    —— 这正是"提前"唯一能生效的位置(见模块 docstring 的三个挂载点)。

    触发量就是节点自己的那条线 `gap < guard_stop`(再宽 PREEMPT_SPEED_MARGIN),理由在
    runs/arm_g_smoke.py 的实测里:**命中率对 margin 几乎是平的**(A r3 在 margin
    0.05/0.20/0.35 下分别是 44.2%/44.9%/46.5%)—— gap(want) 的分布是双峰的,
    策略原意要么通畅要么已经贴墙,中间几乎没有过渡带,所以"把阈值往上抬一点"
    既买不到提前量、又只会让这层更常动手。真正有意义的提前量是**"车还在动"**:
    被停那一拍之后节点就 `neutral()` 了,而那一拍之前 `publish()` 仍然有效。
    """
    a_keep = float(steer_a)
    node._pre_a = a_keep
    node._pre_n = getattr(node, '_pre_n', 0)
    g = node.guard.gap(action_to_steer(a_keep))
    if g is None or math.isnan(g):
        return v
    node._pre_gap = g
    room = node.guard_stop + PREEMPT_MARGIN + PREEMPT_SPEED_MARGIN
    if g >= room:
        return v                      # 还早:这层整个不工作
    allow = v_allow(recent_min_gap(node, g), node.guard_stop)
    v_out = min(v, max(allow, V_MIN))
    pick = _min_swing(node, a_keep)
    if pick is not None and (pick[1] - g) >= DEADBAND:
        node._pre_a = float(pick[0])
        node._pre_n += 1
        _pre_dbg(node, g, pick[1], a_keep, pick[0], v_out)
    return v_out


def _prearm(node, v, steer_a):
    """S 臂的提前层:publish() 这一拍(也就是**车还在动**)就把正在逼近的弧修开。

    判据:策略原意那条弧的净空 `g0 < guard_stop + PREARM_TRIG`(默认 0.70 m)——
    不是"已经判负"(那是 R),是"还在逼近"。实测(见 S 的常量块)这个带在健康局
    只占 1.0~2.2 % 的拍,而崩溃局的每个长段入口都在这个带里持续了 ≥0.2 s。

    返回 (v_out, a_out):
      · 没触发 / 解不到 ⇒ 原样返回,这层当拍不存在(与生产行为一个字节不差);
      · 触发且解到 ⇒ 打挪后的弧,速度按**挪后那条弧**的净空反解(而不是策略原意、
        也不是"先停车再想办法")。车一直在动 ⇒ 不需要 3 秒后的盲倒。
    失败也落盘(kind='miss'),这是 R 那版留下的证据缺口(见 `_pa_dbg`)。
    """
    a0 = float(steer_a)
    g0 = node.guard.gap(action_to_steer(a0))
    if g0 is None or math.isnan(g0):
        return v, a0                      # 雷达不新鲜:不动手
    if math.isinf(g0) or g0 >= node.guard_stop + PREARM_TRIG:
        return v, a0                      # 还早 / 本来通畅:这层不工作
    node._pa_n = getattr(node, '_pa_n', 0) + 1
    target = node.guard_stop + PREARM_TARGET
    pick = _project_between(node, a0, target, PREARM_GAIN)
    if pick is None:
        _pa_dbg(node, 'miss', g0, float('nan'), a0, float('nan'), v)
        return v, a0
    a_out, eff, stage = pick
    v_out = min(v, max(v_allow(eff, node.guard_stop), V_MIN))
    node._pa_hit = getattr(node, '_pa_hit', 0) + 1
    _pa_dbg(node, 'hit%d' % stage, g0, eff, a0, a_out, v_out)
    return v_out, float(a_out)


_orig_publish = RPN.RoadPolicyNode.publish


def publish(self, v, steer_a):
    """正常行驶这一拍:E/F 常驻用走廊中心定转向;C/D/E/F 用净空反解限速。

    方向层为什么现在敢常驻了:B 臂那版在 `publish()` 里用 guard 的 gap 选方向,
    净空 >= SCORE_CAP 时七个候选同分,`-|a|` 独占决定权,**开阔区一律被掰成直行**
    (armB r1 sim 13.4 之后沿 y≈0 直线开到 (42.5,-26.9),off_track 1040/1400)。
    走廊层不会被这个问题污染:开阔处两侧墙(或锥阵边界)仍然存在,中点仍然唯一,
    而且它是**连续量**不是七选一,不靠 `-|a|` 这种对称破缺来决定。

    速度层留在这是安全的:它只会把 v 往下压,不会改变车去哪。
    """
    v_in = float(v)
    # 取走廊估计:E/F 用它做转向;clearance 限速层不转向,但 `_e_dbg` 要用它记 c/w 诊断列,
    # 所以两层任一开就解一次(对 W 只是多算一次 `_corridor_steer`,1656 的转向叠加有
    # `_use_corridor()` 挡着,不生效)。
    corr = _corridor_steer(self) if (_use_corridor() or _use_clr_speed()) else None
    if corr is not None and v > 0.0 and _use_corridor():
        a_c = corr[0]
        steer_a = float((1.0 - CORRIDOR_GAIN) * float(steer_a) + CORRIDOR_GAIN * a_c)
        steer_a = max(-1.0, min(1.0, steer_a))
    if v > 0.0:                                   # 倒车脱困原样放行
        if _use_speed() or _use_speed_reserve():
            g = self.guard.gap(action_to_steer(steer_a))
            if g is not None and not math.isinf(g):
                reserve = _reserve() if _use_speed_reserve() else self.guard_stop
                allow = v_allow(recent_min_gap(self, g), reserve)
                if allow > 0.0:
                    allow = max(allow, V_MIN)     # 不低于策略自己的下限
                v = min(v, allow)
            if _use_clr_speed():
                # 前视 clearance 限速:看所有站位里最紧的一点,而不是当前转向的 gap。
                clr = _corridor_min_clearance(self)
                if clr is not None:
                    span = CORRIDOR_CLR_FULL - CORRIDOR_CLR_MIN
                    v = min(v, max(self.cap * (clr - CORRIDOR_CLR_MIN) / span, V_MIN))
            elif corr is not None and corr[2] < CORRIDOR_W_FULL:
                span = CORRIDOR_W_FULL - CORRIDOR_W_MIN
                v = min(v, max(self.cap * (corr[2] - CORRIDOR_W_MIN) / span, V_MIN))
        if _use_prearm():
            v, steer_a = _prearm(self, v, steer_a)
        if _use_preempt():
            v = _preempt(self, v, steer_a)
            steer_a = self._pre_a
        if _use_bias():
            steer_a = _lateral_bias(self, steer_a)
        if _use_detour():
            steer_a = _detour(self, v, steer_a)
    _e_dbg(self, v_in, float(v), steer_a if corr is None else self.last_action[0],
           corr, steer_a)
    return _orig_publish(self, v, steer_a)


RPN.RoadPolicyNode.publish = publish


def _best_escape(node, want, margin=ESCAPE_MARGIN, side_check=False):
    """回头看有没有别的方向走得动。返回 (action, gap) 或 None。

    **选"离策略原意最近"的可行方向**,而不是"最空"的方向。原来的评分 `eff - |a|` 在
    开阔处会让所有方向同分,于是永远挑直行 —— 左弯里被拦一次就把车掰直,和 publish()
    里那个 bug 是同一个病。改判据:先筛出净空过线的方向,再取 |a - want| 最小的那个。

    want = 策略这一拍原本要打的动作。筛选线是 guard_stop + margin,过不了就返回 None,
    车照旧走"停 3 秒 -> 倒车脱困"——**不改变既有行为**。

    margin:G 臂的提前层用 PREEMPT_MARGIN(比 ESCAPE_MARGIN 宽),被拦兜底层仍用
    ESCAPE_MARGIN,这样 B/D/F 的行为一个字节不变。
    side_check:H 臂的走廊同侧性否决(见 _corridor_side_ok)。
    """
    ok = []
    for a in ACTS:
        if side_check and not _corridor_side_ok(node, a):
            continue
        g = node.guard.gap(action_to_steer(a))
        if g is None or math.isnan(g):
            continue
        eff = SCORE_CAP if math.isinf(g) else g
        if eff >= node.guard_stop + margin:
            ok.append((abs(a - want), -eff, a, eff))
    if not ok:
        return None
    ok.sort()
    return ok[0][2], ok[0][3]


def _keep_dbg(node, kind, a, g, v):
    """I 臂的专用日志:每次"不停车换向"落一行。

    与 `_e_dbg`/`_pre_dbg` 分开写,是为了让"这层到底动了几次、挪了多少、跑多快"
    能单独数出来 —— G 的教训之一就是没有独立计数,事后只能靠文件字节数反推
    "它一次都没触发"。
    """
    path = os.environ.get('ROAD_DEBUG')
    if not path:
        return
    if not hasattr(node, '_keep_fh'):
        try:
            node._keep_fh = open(path + '.keep', 'w', buffering=1)
            node._keep_fh.write('t,speed,kind,a,g,v,n\n')
        except OSError:
            node._keep_fh = None
    fh = node._keep_fh
    if fh is None:
        return
    try:
        fh.write('%.3f,%.3f,%s,%.4f,%.4f,%.4f,%d\n' % (
            node.now() - getattr(node, 't0', 0.0), float(node.speed), kind,
            float(a), float(g), float(v), int(getattr(node, '_keep_n', 0))))
    except (TypeError, ValueError, OSError):
        pass


def _keep_moving(node, kind):
    """搜一条可行弧并以物理允许的速度接着走。返回 True = 已经发出指令。

    与 B 的兜底共用 `_best_escape`(同一条 `ESCAPE_MARGIN` 筛选线、同一条"离策略
    原意最近"的判据),只改两处:
      1. **不等车停住** —— 闸门触发的第一拍就动手。b124:A 的僵停段 p50 2.5s,
         大部分时间花在"等车停稳"和"等满 3 秒"上;
      2. **速度不写死 0.45** —— 用 `v_allow(recent_min_gap, guard_stop)` 反解
         "这条新弧的净空允许我跑多快",再封顶 KEEP_V_MAX。
    找不到任何可行弧时返回 False,调用方照旧走"停 -> 倒车"。**安全网不拆**:
    真卡死(七条弧全不可行)时盲倒仍然会发生,只是它从"第一反应"降级成
    "最后手段"。
    """
    want = float(getattr(node, 'last_action', (0.0,))[0])
    pick = _best_escape(node, want, margin=KEEP_CLEAR_MARGIN)
    if pick is None:
        return False
    a, g = pick
    if KEEP_MAX_SWING > 0.0 and abs(a - want) > KEEP_MAX_SWING:
        # 只许微调。挪不过去就当这条弧不存在 —— 交给"停 -> 倒车"那条老路,
        # 宁可倒车也不把策略原意翻面(翻面 = cut 的机理)。
        node._keep_skip = getattr(node, '_keep_skip', 0) + 1
        return False
    v = v_allow(recent_min_gap(node, g), node.guard_stop)
    v = min(KEEP_V_MAX, max(v, V_MIN))
    node._esc_last = node.now()
    node._keep_n = getattr(node, '_keep_n', 0) + 1
    _keep_dbg(node, kind, a, g, v)
    node.get_logger().warn(
        f'{kind} 不停车改走 {a:+.1f}(策略要 {want:+.1f},净空 {g:.2f}m) '
        f'以 {v:.2f}m/s 继续走', throttle_duration_sec=1.0)
    _orig_publish(node, v, float(a))
    return True


_orig_neutral = RPN.RoadPolicyNode.neutral


def _in_block(node):
    """是不是真的"被拦住了动不了"。

    guard 在健康的一局里也有 64~85% 的 tick 处于"当前这条弧里有锥"的状态,但车在
    **动**,下一拍观测就变了,不需要任何人插手。真正要命的只有一种:被拦 + **车不动**
    —— 车不动则观测不变、策略输出不变,guard 就永远拦。所以入门条件是
    "被拦 + 车停住 + 停住满 ARM_ESCAPE_DELAY 秒"三者同时成立。

    出门条件要松一档:一旦我们自己开始改走,车就在动了(0.45 m/s > STOP_EPS),再要求
    "车停住"会让它 0.3 秒一停一冲地抖。所以刚改走过 ESCAPE_REARM 秒内不再要求车停。

    另外一处让路:已经在 `escape_reverse()` 里 —— `tick()` 的 `:280` 在我们之后执行,
    最后写赢的本来就是它,这里先让开。
    """
    if getattr(node, 'block_t', None) is None:
        return False
    now = node.now()
    if getattr(node, 'escape_until', None) is not None and now < node.escape_until:
        return False
    if now - node.block_t < ARM_ESCAPE_DELAY:
        return False
    if abs(float(node.speed)) <= STOP_EPS:
        return True
    return (now - getattr(node, '_esc_last', -1e9)) <= ESCAPE_REARM


def neutral(self):
    """guard 拦停时,E/F 先按走廊中心找方向;还不成(F/B/D)再按弧道扫描找方向。

    只在"使能中 + 雷达新鲜 + 真的动不了"时接管 —— `tick()` 里另外两条走 neutral()
    的路(`:222-225` 心跳断开、`:226-228` 雷达超时)分别靠 `self.enabled` 与雷达新鲜度
    排除,它们必须真的停车,不能被这一层改成"找个方向慢慢开"。

    注意 `road_policy_node.py:258` 是 `self.neutral()` 当语句调用、**丢弃返回值**的,
    所以这里发出去的改走指令并不阻止 `:260-281` 继续往下走。`:260` 的 3 秒闸门意味着
    改走有 3 秒预算:3 秒内把当前弧清开(0.45 m/s × 3 s = 1.35 m,通常够)就回到正常
    行驶;清不开就照旧升级成 `escape_reverse()` 倒车。这是刻意留的安全网。

    **I 臂在这里插了一档**:不等 `_in_block`(它要求"车已经停住 + 停满 0.3s"),
    闸门触发的第一拍就直接 `_keep_moving()` —— 车不停,观测就一直变,极限环断在
    第 0.05 秒而不是第 3 秒。用 `_in_tick` 排除 `main()` 的 `stop()`(停机时连发
    5 次 `neutral()` 刹车,那里绝不许发前进指令)。

    **R 臂插的是同一档的另一版**(用户 m13219/m13234:别谈抬 cap,先在 cap1.8 上把
    卡死本身解决掉):触发点与 I 相同(判负第一拍 + `_in_tick`),但
      ① 用 `_project` 的**连续最小摆动**替掉 I 的 `_best_escape` 七条离散弧;
      ② 速度按投影后的净空**反解**(`_project_v`),不是固定的 0.90;
      ③ `escape_reverse()` 里也投影一次 ⇒ 倒车只在"摆动上限内确实无解"时才发生。
    """
    if not (self.enabled and self.scan is not None
            and self.now() - self.scan_t <= self.scan_timeout):
        return _orig_neutral(self)
    if (_use_project() and getattr(self, '_in_tick', False)
            and getattr(self, 'block_t', None) is not None
            and not (getattr(self, 'escape_until', None) is not None
                     and self.now() < self.escape_until)):
        want = float(self.last_action[0])
        pick = _project(self, want)
        if pick is not None:
            a, g = pick
            self._esc_last = self.now()
            self._proj_n = getattr(self, '_proj_n', 0) + 1
            _proj_dbg(self, 'block', want, a, g, _project_v(self, g))
            self.get_logger().warn(
                f'判负不停车:投影 {want:+.2f}->{a:+.2f}(净空 {g:.2f}m)'
                f' 以 {_project_v(self, g):.2f}m/s 继续走',
                throttle_duration_sec=1.0)
            return _orig_publish(self, _project_v(self, g), float(a))
    if (_use_creep_pos() and getattr(self, '_in_tick', False)
            and getattr(self, 'block_t', None) is not None
            and not (getattr(self, 'escape_until', None) is not None
                     and self.now() < self.escape_until)):
        cr = _creep_pos(self)
        if cr is not None:
            a, g, v = cr
            _creep_progress(self)
            self._esc_last = self.now()
            self._proj_n = getattr(self, '_proj_n', 0) + 1
            _proj_dbg(self, 'creep', float(a), float(a), g, v)
            self.get_logger().warn(
                f'判负但净空为正 {g:.3f}m:沿策略弧 {a:+.2f} 以 {v:.3f}m/s 蹭走(不换弧/不倒车)',
                throttle_duration_sec=1.0)
            return _orig_publish(self, v, float(a))
    if (_use_move_arc() and getattr(self, '_in_tick', False)
            and getattr(self, 'block_t', None) is not None
            and not (getattr(self, 'escape_until', None) is not None
                     and self.now() < self.escape_until)):
        # V 臂:判负那一拍用**提前扫好的七条弧**(on_scan 里每帧算的)决定往哪走。
        # 速度恒为 MOVE_V = V_MIN:低于它车里不动(m14584);转向只在意图弧不够时才改。
        want = float(self.last_action[0])
        sc = _scan_arcs(self)
        mv = _move_arc(self)
        if mv is not None:
            a, g, v = mv
            self._mv_a = float(a) if abs(float(a) - want) > 1e-6 else None
            _move_progress(self)
            self._esc_last = self.now()
            self._proj_n = getattr(self, '_proj_n', 0) + 1
            _proj_dbg(self, 'move', want, a, g, v)
            _arcs_dbg(self, 'move', sc, a, g)
            self.get_logger().warn(
                f'判负换弧走 {want:+.2f}->{a:+.2f}(净空 {g:.3f}m)'
                f' 以 {v:.2f}m/s 走(七弧可行 {sc["n_ok"]}/7)',
                throttle_duration_sec=1.0)
            return _orig_publish(self, v, float(a))
        _arcs_dbg(self, 'hand', sc)
    if (_use_keep_moving() and getattr(self, '_in_tick', False)
            and getattr(self, 'block_t', None) is not None
            and not (getattr(self, 'escape_until', None) is not None
                     and self.now() < self.escape_until)
            and _keep_moving(self, '被拦即换向')):
        return None
    if _in_block(self):
        want = float(self.last_action[0])
        if _use_corridor():
            pick = _corridor_steer(self)
            if pick is not None:
                a, c, w, x0 = pick
                self._esc_last = self.now()
                self.get_logger().warn(
                    f'被拦 {self.now() - self.block_t:.1f}s 车不动,改走 {a:+.2f}'
                    f'(策略要 {want:+.2f},净空 {w:.2f}m) 走廊 c {c:+.2f}@{x0:.1f}',
                    throttle_duration_sec=1.0)
                return _orig_publish(self, ESCAPE_V, float(a))
        if _use_arc_escape():
            pick = _best_escape(self, want)
            if pick is not None:
                a, g = pick
                self._esc_last = self.now()
                self.get_logger().warn(
                    f'被拦 {self.now() - self.block_t:.1f}s 车不动,改走 {a:+.1f}'
                    f'(策略要 {want:+.1f},净空 {g:.2f}m)',
                    throttle_duration_sec=1.0)
                return _orig_publish(self, ESCAPE_V, float(a))
    return _orig_neutral(self)


RPN.RoadPolicyNode.neutral = neutral


_orig_escape_reverse = RPN.RoadPolicyNode.escape_reverse


def _capped_escape_reverse(node):
    """K 臂的盲倒:速度取 `min(cap, REV_ABS)`,其余(时长、反打方向)与生产行逐行一致。

    `road_policy_node.py:213-217`:sim 分支是 `publish(-self.cap * REVERSE_FRACTION,
    -self.escape_steer)`;这里有且只有速度那一项被换成 `min(cap, REV_ABS)`。
    非 sim(真车)分支不碰 —— K 只声明它在仿真里量到的东西。

    盲倒默认关(`arms/arm_host.py::REVERSE_ENABLED`):
    关时直接落到宿主出口(只停不倒),不再自己发负速度 —— 否则这条会绕过宿主的开关。
    """
    if not RPN.REVERSE_ENABLED:
        return _orig_escape_reverse(node)
    if node.sim:
        return node.publish(-min(float(node.cap), REV_ABS) * _REV_FRACTION,
                            -node.escape_steer)
    return _orig_escape_reverse(node)


def escape_reverse(self):
    """I/R 臂:把"盲倒 1.5 秒"换成"再找一条可行弧接着走"。

    节点只在"连续截停满 3 秒"之后才走到这里(`road_policy_node.py:280`),所以这是
    **明确被判定为卡住**的时刻。但"卡住"的原因是"策略那条弧不通",并不等于
    "没有别的路可走"。先搜;搜不到才真的倒车 —— 于是倒车从"第一反应"降级成
    "最后手段",这正是用户 m05621 要的"能避免倒车么"。

    R 臂在这里也投影一次,理由更硬:用户 m13209 明确说"倒车卡死破坏那就是另辟蹊径",
    R 的整个立论就是"车只要还在动,极限环就不存在"。如果 3 秒闸门到了还去盲倒,等于
    承认"投影没找到可行弧";而倒车那一下**雷达在车头、背后是盲的**,正是"擦到旁边那颗
    锥"的来源(§五)。所以 R 只在**投影确实无解**时才让生产的倒车逻辑执行。

    K 臂不在 `_use_keep_moving()` 里,所以它走的是下面那条 ceiling 分支:
    **仍然是"卡住就倒车"的 A 行为,只是倒得和 cap 1.5 一样慢。**
    (盲倒默认关后,这条 ceiling 分支在默认配置下也只停不倒 —— K 的封顶只在
    `ARM_HOST_REVERSE=1` 时才有意义。)
    """
    if (_use_project() and self.enabled and self.scan is not None
            and self.now() - self.scan_t <= self.scan_timeout):
        want = float(self.last_action[0])
        pick = _project(self, want)
        if pick is not None:
            a, g = pick
            self._esc_last = self.now()
            self._proj_n = getattr(self, '_proj_n', 0) + 1
            _proj_dbg(self, 'stuck', want, a, g, _project_v(self, g))
            self.get_logger().warn(
                f'满 3 秒仍判负:再次投影 {want:+.2f}->{a:+.2f}(净空 {g:.2f}m)'
                f' 以 {_project_v(self, g):.2f}m/s 继续走,不倒车',
                throttle_duration_sec=1.0)
            return _orig_publish(self, _project_v(self, g), float(a))
    if (_use_keep_moving() and self.enabled and self.scan is not None
            and self.now() - self.scan_t <= self.scan_timeout
            and _keep_moving(self, '卡住再换向')):
        return
    if _use_rev_ceiling():
        return _capped_escape_reverse(self)
    return _orig_escape_reverse(self)


RPN.RoadPolicyNode.escape_reverse = escape_reverse


_orig_tick = RPN.RoadPolicyNode.tick


def tick(self):
    """只做一件事:标出"这一拍是 tick 在跑"。

    `neutral()` 有三条来路:`tick()` 里的闸门拦截、心跳断开、雷达超时;另外
    `main()` 的 `stop()` 信号处理也会连发 5 次 `neutral()` 用来刹车。I 臂要在
    "闸门拦截"时把停车改成继续走,又**绝不能在停机时发前进指令**,所以用
    "是不是在 tick 里"来区分 —— 比猜 `block_t` 的状态可靠。
    """
    self._in_tick = True
    try:
        return _orig_tick(self)
    finally:
        self._in_tick = False


RPN.RoadPolicyNode.tick = tick


if __name__ == '__main__':
    RPN.main()
