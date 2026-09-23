#!/usr/bin/env python3
"""编码器速度闭环:目标速度 -> 电调 PWM。纯 Python,不依赖 ROS,便于测试。

2026-09-23/24 的落地数据:静态 11.24V 时破静点 1594、刹停点 1588、平台约
0.48m/s;静态 12.5V 时破静点仍是 1594、刹停点 1584、平台约 0.66m/s。
9 月 22 日那次"满电刹停点 1544"与后续测量不一致,不能拿它推断电压漂移。
车上读不到电压(厂家串口协议只有 send_cmd,没有回读),闭环用编码器跟踪车速。

## 为什么不是一个普通的 PI

这台车有一个**不可逆的失败模式**:PWM 掉到"维持点"以下车就停,而停了之后要重新
起步需要**更高**的破静 PWM。普通 PI 在超速时会一路往下压,压过头车停住,测速变 0,
积分再反向堆起来把 PWM 顶上去 —— 于是车一冲一停。用户在厂家那套闭环上看到的
"降速时卡顿一下"就是这个。

所以这里有三条硬约束,`SpeedLoop` 的测试逐条钉死:

1. **车在动的时候,输出永远不低于 `pwm_floor`**(维持点)。宁可比目标快一点,
   也不掉进停车区 —— 掉下去的代价是一次完整的停车+破静循环。
2. **车没动的时候先从中位爬过死区,再爬到 `pwm_breakaway`**,并且**冻结积分**。
   不冻的话架空台上(编码器是落地从动轮,架空恒读 0)积分会一路冲到上限,一落地
   就窜出去。
3. **向上和向下都有限速**,避免电调拒绝突变或输出跌入停车区。

## 自标定:floor 只往上学,不往下探

刹停点没法在开跑时直接测 —— 要测它就得先把车跑停。所以改成**从失败里学**:
速度先到过目标附近、之后又塌下去(低于目标的 `collapse_frac`),说明 floor 可能估低了,
立刻抬 `floor_bump_up`。配置里的 `pwm_floor` 只是初值。

**故意不做"往下探"。** 想过让它在跑得稳的时候慢慢往下试探更低的 floor,实现完
才想明白没有收益:这台车的最低可维持速度就是那个 ~0.56 m/s 的平台,比平台更慢的
目标本来就达不到,往下探不会让车更慢,只会探到刹停点把车探停 —— 一次真停车换
零收益。实测满电刹停点 1584、11.24V 时 1588,亏电后还要继续实测。

初值给偏高一点没关系(只是跑得比目标快一点),给低了才要命(直接停车)。换电池后
想要准确初值,跑 `ramp_curve.py` 重测。

`pwm_breakaway` 不学,实测它基本不随电量变,写死更稳。
"""
from __future__ import annotations

from dataclasses import dataclass

PWM_HARD_MAX = 1620     # 和 steer_policy_node.py 同一个硬上限,别单独放宽


@dataclass
class SpeedLoopConfig:
    """全部 PWM 量纲都是电调刻度(中位 1540)。"""
    pwm_mid: int = 1540         # 真中位,见 fastsim/params.py 的说明
    # 2026-09-24 静态 12.5V 落地:破静 1594、刹停 1584;9-23 静态 11.24V:
    # 破静 1594、刹停 1588。初始 floor 取 1590,运行中只允许往上学。
    # 9-22 的"满电刹停 1544"与后续测量不一致,保留原始记录但不用于默认参数。
    #
    # 曾经写过 breakaway=1554,那是 ramp_curve.py 只看一帧判起转被编码器毛刺骗了,
    # 按那个值车根本起不来。两个脚本都已经改成连续 3 档才算数。
    # 电压主要改的是速度(平台 668 vs 480 mm/s),PWM 特征点仅挪约 4~10。
    pwm_floor: int = 1590       # 车在动时的最低输出 —— 低于它会停,随电量变
    pwm_breakaway: int = 1594   # 从静止起步的最低输出 —— 必须 >= pwm_floor
    pwm_max: int = PWM_HARD_MAX

    # [2026-09-24] 按实测曲线 + θ=0.28s 仿真扫出来的(docs/validation/2026-09-24-pid)。
    # 原来的 kp=60 在拐点(~40 mm/s/PWM)上环路增益 2.4,稳态速度来回摆 0.4 m/s;
    # kp=8 ki=60 在 12.5V/11.24V/10V(估)三条曲线上都摆 ~0.1 m/s、不停车。
    # 实车(12.5V,loop_test.py)ki=60 起步后 3.5s 才跟上 0.75,ki=75 缩到 2.2s,
    # 稳态摆幅 0.07 m/s、没振荡,取 75。
    kp: float = 8.0             # PWM per (m/s)
    ki: float = 75.0            # PWM per (m/s·s)
    down_rate: float = 200.0    # 向下最多 PWM/s
    # 从中位爬到 pwm_floor 的速率。电调两边都卡:一步跳上去按突加油门保护处理
    # 不启动,爬太慢也不启动(2026-09-22 架空实测 2 PWM/s 从 1550 爬到 1650
    # 全程不转,20 PWM/s 立刻转)。和 steer_policy_node.PWM_RAMP_UP 同一个速率。
    # 全程生效,不只是起步那一段:车跑起来之后向上跳一大步电调认不认,没测过,
    # 而这台车 1599~1612 是个平台,压根没有"必须马上加 16 个 PWM"的场景,
    # 慢慢加的代价是零。要改先去实测。
    up_rate: float = 20.0

    # --- floor 自标定:只往上,不往下 ---
    floor_bump_up: int = 6          # 速度塌了,一次把 floor 抬这么多 PWM
    collapse_frac: float = 0.5      # 速度掉到目标的这个比例以下算"塌了"
    hold_frac: float = 0.8          # 到过目标的这个比例才开始判"塌"(躲开起步加速段)
    # 起步时如果到了 pwm_breakaway 还不动,允许继续往上爬到这里为止。
    # 破静点也可能漂,写死 1594 后若实际起转点更高,还要继续往上爬。
    # 车不动时无法靠 floor 自标定判断实际起转点。
    # 但不能无限爬:架空台上编码器恒读 0,无限爬会一路顶到 pwm_max,一落地就窜。
    # 所以给一个有界的余量,够覆盖实测到的漂移范围就行。
    start_margin: int = 20
    # 起步第一拍直接跳到这里,再按 up_rate 爬过破静点。默认 = 中位(不跳,和原来一样)。
    # 从中位爬到破静点 1594 要 2.7s,这段是电调死区,车不动、闭环也没有反馈可用 ——
    # 纯粹是等。跳到 ~1580 能把起步缩到 ~0.7s,过破静点那一下仍是慢爬,起步不会更猛。
    # **电调认不认一步跳到 1580 还没测**(它有时拒绝突加油门),测过再改默认值。
    launch_from: int = 1540

    stall_speed: float = 0.05   # 低于这个算"没在动"(m/s)
    # 目标速度斜坡(m/s²):闭环跟的是一个按这个速率追向目标的参考值,而不是目标本身。
    # 目标一步从 0.75 跳到 0.9 时,阶跃会让车冲过头(2026-09-24 实车 0.9 冲到 0.94)。
    # 0 = 不用斜坡,直接跟目标。
    target_rate: float = 0.0
    # 积分项本身的上下限(PWM)。floor 1590 到 1620 差 30,亏电时整段都靠积分顶,40 太紧
    i_limit: float = 80.0

    def __post_init__(self):
        if not self.pwm_mid < self.pwm_floor <= self.pwm_breakaway <= self.pwm_max:
            raise ValueError(
                f'要满足 pwm_mid < pwm_floor <= pwm_breakaway <= pwm_max,'
                f'给的是 {self.pwm_mid}/{self.pwm_floor}/'
                f'{self.pwm_breakaway}/{self.pwm_max}')
        if self.pwm_max > PWM_HARD_MAX:
            raise ValueError(f'pwm_max 不能超过硬上限 {PWM_HARD_MAX}')
        if not 10.0 <= self.up_rate <= 40.0:
            raise ValueError(
                f'up_rate {self.up_rate} 超出实测可用区间 [10, 40] PWM/s:'
                f'太慢电调不起转,太快等于跳变')
        if not 0.0 < self.collapse_frac < 1.0:
            raise ValueError('要满足 0 < collapse_frac < 1')
        if self.target_rate < 0:
            raise ValueError('target_rate 不能是负的')
        if not self.pwm_mid <= self.launch_from < self.pwm_breakaway:
            raise ValueError(f'launch_from 要在 [pwm_mid, pwm_breakaway) 里,给的是 {self.launch_from}:'
                             f'跳过破静点就是突加油门')


class SpeedLoop:
    """update(target, measured, dt) -> PWM。目标给 0 就回中位并清状态。"""

    def __init__(self, cfg: SpeedLoopConfig | None = None):
        self.cfg = cfg or SpeedLoopConfig()
        self.floor = float(self.cfg.pwm_floor)      # 学出来的值,跨 reset 保留
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.ref = 0.0              # 斜坡后的参考速度,见 target_rate
        self.last_pwm = float(self.cfg.pwm_mid)
        self.stalled = True         # 起步前当作停着
        self.reached = False        # 本轮有没有跑到过目标附近(自标定的武装条件)
        # floor 是学出来的,cfg.pwm_floor 只是初值。**不在 reset 里清掉学到的值**
        # 的话换目标速度就会丢标定 —— 所以只在第一次构造时初始化,见 __init__。

    def _adapt_floor(self, target, measured, dt):
        """快上慢下地学 pwm_floor(刹停点)。它随电量差 50 个 PWM,写死必然失效。"""
        c = self.cfg
        if measured >= target * c.hold_frac:
            self.reached = True             # 这一轮确实跑到过目标附近
            return
        if not (self.reached and measured < target * c.collapse_frac):
            return
        # 曾经跑到目标附近,现在塌了 -> **当前这个 PWM 维持不住**,floor 至少要比它高。
        #
        # 判据故意不要求"输出压在 floor 上"。原来是那么写的,结果漏掉了最要命的情况:
        # floor 估得远低于真值时,PI 会在真刹停点附近来回震(实测复现:floor 信 1546、
        # 真值 1598,输出在 1594~1598 之间摆,速度 0.24 <-> 0.51 循环),而输出从来
        # 没靠近过 1546,自标定永远不触发。那正是这个模块存在的理由 —— 一冲一停。
        self.floor = min(float(c.pwm_max), max(self.floor, self.last_pwm + c.floor_bump_up))
        self.reached = False                # 重新武装,等它再跑起来

    def update(self, target: float, measured: float, dt: float) -> float:
        c = self.cfg
        if dt <= 0:
            return self.last_pwm
        if target <= 0:
            self.reset()
            return float(c.pwm_mid)

        # 起步爬升:输出还在 pwm_floor 以下时,只许连续往上爬,不许一步跳到
        # breakaway。原来这里是 max(pwm_breakaway, last_pwm),从中位直接跳 54 个
        # PWM —— 电调按突加油门保护处理,车一动不动,现象和"带不动"一模一样。
        if self.last_pwm < self.floor:
            start = max(self.last_pwm, float(c.launch_from))
            self.last_pwm = min(self.floor, start + c.up_rate * dt)
            return self.last_pwm

        was_stalled = self.stalled
        self.stalled = measured < c.stall_speed
        if c.target_rate <= 0:
            self.ref = target
        else:
            # 参考值不许落在实际车速"后面":往上追目标时车已经比参考值快,参考值
            # 直接跟到车速;往下同理。斜坡只限制比车实际变化更快的那部分。
            # 不这么做的话,车一过破静点就跳到平台(~0.5 m/s),参考值却还从 0.05
            # 往上爬,闭环以为超速,把 PWM 往 floor 压 —— 实车复现过(2026-09-24,
            # target_rate=0.3:0.75 的段只跑到 0.63)。
            if target > self.ref and measured > self.ref:
                self.ref = min(target, measured)
            elif target < self.ref and measured < self.ref:
                self.ref = max(target, measured)
            if not self.stalled:
                step = c.target_rate * dt
                self.ref += max(-step, min(step, target - self.ref))
        error = self.ref - measured

        if was_stalled and not self.stalled:
            # 刚起步:让积分接住起步时的输出(无扰切换)。不接的话输出从破静 PWM
            # 一下掉到 floor + kp*error —— kp 小的时候那比真刹停点低得多,车刚动就
            # 又停,一冲一停,而且没跑到过目标,自标定也永远触发不了(仿真复现:
            # kp=8、floor 信 1560、真刹停 1598,40 秒里停 4~10 次)。
            lim = c.i_limit / max(c.ki, 1e-9)
            self.integral = max(-lim, min(lim, (self.last_pwm - self.floor - c.kp * error)
                                      / max(c.ki, 1e-9)))

        if self.stalled:
            # 起步:爬到破静 PWM;还不动就继续爬,但封顶在 breakaway + start_margin。
            # **不积分** —— 架空时编码器恒读 0,积分会一路冲顶,一落地就窜出去。
            pwm = min(float(c.pwm_breakaway + c.start_margin),
                      max(float(c.pwm_breakaway), self.last_pwm + c.up_rate * dt))
        else:
            self._adapt_floor(target, measured, dt)
            # 积分只在没顶到 floor 时累积。顶在 floor 上说明已经减无可减
            # (电调不刹车,再压也没用),这时继续积分等于纯粹的抗饱和失效:
            # 车靠滑行慢慢减速的那 0.8 秒里误差一直是正的,积分会堆满,
            # 等速度真降下来时反向超调。
            saturated_low = self.last_pwm <= self.floor + 1e-6 and error < 0
            # 往上同理:输出已经被爬升速率(或 pwm_max)卡住时,再积分只是在堆"欠账",
            # 等 PWM 爬到位后这笔账会让它冲过头。仿真:前馈把积分放到 1608 后,
            # 爬坡那 1 秒里积分又多堆 7 个 PWM,冲到 1615、速度 1.01(目标 0.75)。
            wanted = self.floor + c.kp * error + c.ki * self.integral
            capped_high = error > 0 and wanted >= min(float(c.pwm_max),
                                                      self.last_pwm + c.up_rate * dt)
            if not (saturated_low or capped_high):
                self.integral += error * dt
                self.integral = max(-c.i_limit / max(c.ki, 1e-9),
                                    min(c.i_limit / max(c.ki, 1e-9), self.integral))
            pwm = self.floor + c.kp * error + c.ki * self.integral

        pwm = max(self.floor, min(float(c.pwm_max), pwm))
        # 向上限速:电调不吃突变(见 up_rate 的注释)
        pwm = min(pwm, self.last_pwm + c.up_rate * dt)
        # 向下限速:降太快会掉进停车区,而停了要更高的破静 PWM 才起得来
        pwm = max(pwm, self.last_pwm - c.down_rate * dt)
        pwm = max(self.floor, min(float(c.pwm_max), pwm))
        self.last_pwm = pwm
        return pwm
