"""速度闭环的测试:python3 -m pytest vehicle/test_speed_loop.py

这些用例针对的是**这台车特有的那个不可逆失败**:PWM 掉到维持点以下车就停,
而停了之后要重新起步需要更高的破静 PWM。普通 PI 会在超速时一路往下压、压过头
车停住、测速变 0、积分再把 PWM 顶上去 —— 一冲一停,就是用户在厂家那套闭环上
看到的"降速时卡顿一下"。

所以下面第 1、2、5 条是硬约束,不是风格偏好。
"""
import math

import pytest

from vehicle.speed_loop import PWM_HARD_MAX, SpeedLoop, SpeedLoopConfig

DT = 0.05           # 20Hz,和策略节点的 control_dt 一致


class Plant:
    """一阶车辆模型 + 那个停车点。

    PWM 低于 pwm_floor 时**不产生推力**,车靠阻力滑停;停住之后要 PWM 达到
    pwm_breakaway 才重新动起来。这就是要避开的那个点。
    """

    # PWM->速度用 2026-09-24 落地、静态 12.5V 的 ramp_curve 下降段实测值做分段线性
    # (docs/实测/2026-09-24-pid/闭环标定.md)。原来用 9-22 傍晚那条(1584 = 1.245 m/s、
    # 刹停 1544),之后三次落地测量都对不上,换掉。
    #
    # 实测曲线最要紧的特征是**平台**:刹停点往上到 1602 一直是 ~0.66 m/s,每个 PWM
    # 才动几个 mm/s,1604 之后才进拐点(~40 mm/s/PWM)。也就是说比平台慢的速度
    # 根本达不到 —— 这不是模型的缺陷,是这台车的事实,测试必须反映它。
    CURVE = [(1588, 0.660), (1602, 0.668), (1606, 0.706), (1610, 0.802),
             (1614, 0.979), (1618, 1.133), (1620, 1.200)]

    # 车的真实刹停点/破静点。**故意不从 cfg 取** —— cfg 里的是闭环"以为"的值,
    # 自标定要测的就是它能不能把自己的信念修到真值上。共用一个数的话,
    # floor 自标定的测试等于自己考自己。
    TRUE_STALL = 1584
    TRUE_BREAK = 1594

    def __init__(self, cfg, tau=0.35, stall=None, brk=None):
        self.cfg, self.tau = cfg, tau
        self.stall = self.TRUE_STALL if stall is None else stall
        self.brk = self.TRUE_BREAK if brk is None else brk
        self.v = 0.0

    def _curve(self, pwm):
        pts = self.CURVE
        if pwm <= pts[0][0]:
            return pts[0][1]
        for (p0, v0), (p1, v1) in zip(pts, pts[1:]):
            if pwm <= p1:
                return v0 + (v1 - v0) * (pwm - p0) / (p1 - p0)
        return pts[-1][1]

    def step(self, pwm, dt):
        moving = self.v >= self.cfg.stall_speed
        threshold = self.stall if moving else self.brk
        target_v = self._curve(pwm) if pwm >= threshold else 0.0
        self.v += (target_v - self.v) * dt / self.tau
        if self.v < 1e-3:
            self.v = 0.0
        return self.v


def run(loop, plant, target, seconds):
    """跑一段,返回 [(pwm, v)]。"""
    out = []
    for _ in range(int(seconds / DT)):
        pwm = loop.update(target, plant.v, DT)
        out.append((pwm, plant.step(pwm, DT)))
    return out


# --- 硬约束 ---

def test_moving_output_never_drops_below_floor():
    """车在动的时候,输出永远不低于**当前学到的** floor。

    比的不再是 cfg.pwm_floor —— 那只是初值,floor 现在是学出来的。真正的不变式
    是"输出不低于此刻的 floor",以及最终目的:整段跑下来车不能停。
    """
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    run(loop, plant, 0.6, 6)                       # 先跑起来
    # 目标突然砍半,最容易把 PWM 压过头
    trace = run(loop, plant, 0.25, 6)
    assert min(p for p, _ in trace) >= loop.floor - 1e-6
    assert all(v >= cfg.stall_speed for _, v in trace), '车停了'


def test_never_exceeds_hard_max():
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    trace = run(loop, plant, 5.0, 10)              # 要一个根本达不到的速度
    assert max(p for p, _ in trace) <= PWM_HARD_MAX


def test_config_rejects_max_above_hard_limit():
    with pytest.raises(ValueError):
        SpeedLoopConfig(pwm_max=1700)


def test_config_rejects_floor_above_breakaway():
    """破静点必须 >= 维持点,反了说明数抄错了。"""
    with pytest.raises(ValueError):
        SpeedLoopConfig(pwm_floor=1600, pwm_breakaway=1560)


def test_stalled_does_not_wind_up():
    """架空台上编码器恒读 0。积分不冻的话会一路冲顶,一落地就窜出去。"""
    cfg = SpeedLoopConfig()
    loop = SpeedLoop(cfg)
    pwm = None
    for _ in range(400):                           # 20 秒都不动
        pwm = loop.update(0.6, 0.0, DT)
    assert loop.integral == 0.0
    # 允许爬过 breakaway 找更高的真破静点(它随电量漂),但必须封顶 ——
    # 架空时编码器恒读 0,不封顶就会一路顶到 pwm_max,一落地就窜出去。
    assert pwm <= cfg.pwm_breakaway + cfg.start_margin, '没动的时候越顶越高,没封顶'


def test_downward_rate_is_limited_upward_is_not():
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    run(loop, plant, 0.8, 8)
    prev = loop.last_pwm
    for _ in range(40):
        pwm = loop.update(0.1, plant.v, DT)
        assert pwm >= prev - cfg.down_rate * DT - 1e-6
        prev = pwm
        plant.step(pwm, DT)


# --- 功能 ---

def test_reaches_target_from_standstill():
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    trace = run(loop, plant, 0.6, 20)
    assert plant.v > cfg.stall_speed, '从静止起不来'
    assert abs(trace[-1][1] - 0.6) < 0.08, f'稳态差太多:{trace[-1][1]:.3f}'


def test_startup_ramps_instead_of_jumping():
    """保持已验证的默认爬升行为；突加油门可能突然起步，不能依赖电调拒绝。"""
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    trace = run(loop, plant, 0.6, 20)
    pwms = [p for p, _ in trace]

    assert pwms[0] <= cfg.pwm_mid + cfg.up_rate * DT + 1e-6, \
        f'第一步就发了 {pwms[0]},这是从中位跳上去'
    biggest = max(b - a for a, b in zip(pwms, pwms[1:]))
    assert biggest <= cfg.up_rate * DT + 1e-6, f'中间有一次 +{biggest:.0f} 的跳变'

    reached = next(i for i, p in enumerate(pwms) if p >= cfg.pwm_breakaway)
    assert reached * DT < 4.0, f'爬到破静点用了 {reached * DT:.1f}s,太慢'


def test_startup_ramp_passes_through_the_whole_dead_band():
    """爬升要逐档经过中位到 floor 之间,不能跳过 —— 跳过就等于突变。"""
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    pwms = [p for p, _ in run(loop, plant, 0.6, 10)]
    band = [p for p in pwms if cfg.pwm_mid < p < cfg.pwm_floor]
    assert len(band) >= (cfg.pwm_floor - cfg.pwm_mid) / (cfg.up_rate * DT) - 1


def test_config_rejects_ramp_rates_outside_the_measured_window():
    with pytest.raises(ValueError):
        SpeedLoopConfig(up_rate=2.0)               # 太慢,实测不起转
    with pytest.raises(ValueError):
        SpeedLoopConfig(up_rate=500.0)             # 太快,等于跳变


def test_never_stalls_once_moving():
    """整段跑完车不能停 —— 这是前面所有约束的最终目的。"""
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    run(loop, plant, 0.6, 6)
    for target in (0.2, 0.6, 0.15, 0.4):           # 反复变速
        for _, v in run(loop, plant, target, 5):
            assert v >= cfg.stall_speed, f'目标 {target} 时停住了'


def test_zero_target_returns_neutral_and_clears_state():
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    run(loop, plant, 0.6, 5)
    assert loop.update(0.0, plant.v, DT) == cfg.pwm_mid
    assert loop.integral == 0.0


def test_output_is_finite_for_absurd_inputs():
    loop = SpeedLoop()
    for target, measured in ((0.6, float('nan')), (0.6, 1e6), (1e6, 0.0)):
        pwm = loop.update(target, measured, DT)
        assert math.isfinite(pwm) or measured != measured


# --- floor 自标定 -----------------------------------------------------------
# floor(刹停点)随电量差 50 个 PWM,写死必然失效。这里只学"往上",见
# speed_loop.py 的说明。Plant 的真值是 TRUE_STALL/TRUE_BREAK,**不从 cfg 取** ——
# 闭环必须靠观测把自己的信念修到真值上,而不是一开始就知道答案。

def stops(trace, cfg):
    return sum(1 for (_, a), (_, b) in zip(trace, trace[1:])
               if a >= cfg.stall_speed > b)


def test_low_floor_belief_does_not_stop_the_car():
    """floor 初值比真刹停点低 38,车也不能一冲一停。

    kp 小的时候,起步那一下输出会从破静 PWM 掉到 floor + kp*error,远低于真刹停点
    —— 没有无扰切换时仿真里 40 秒停 4~10 次,而且没跑到过目标,自标定也触发不了。
    """
    cfg = SpeedLoopConfig(pwm_floor=1560)
    loop, plant = SpeedLoop(cfg), Plant(cfg, stall=1598, brk=1604)
    trace = run(loop, plant, 0.6, 40)
    assert stops(trace, cfg) == 0
    assert plant.v >= cfg.stall_speed, '最后车还是停的'


def test_floor_rises_when_the_real_stall_point_is_higher():
    """跑着跑着电压掉下去(真刹停点从 1584 升到 1600),floor 要自己抬到真值以上。"""
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    run(loop, plant, 0.6, 10)
    assert loop.floor > Plant.TRUE_STALL           # 起步学到的(不再是写死的初值)
    plant.stall, plant.brk = 1600, 1606
    trace = run(loop, plant, 0.6, 30)
    assert loop.floor >= 1600, f'没抬上去,还在 {loop.floor}'
    assert stops(trace, cfg) == 0 and plant.v >= cfg.stall_speed


def test_floor_does_not_move_after_it_is_learned():
    """起步学到 floor 之后,稳定跟目标时 floor 不该乱动 —— 不往下探是刻意的设计。
    破静和维持只差 5(比实测的 6~12 还紧)时,学到的 floor 也不能低于维持点。"""
    cfg = SpeedLoopConfig()
    loop = SpeedLoop(cfg)
    plant = Plant(cfg, stall=cfg.pwm_floor - 1, brk=cfg.pwm_breakaway)
    run(loop, plant, 0.6, 5)
    learned = loop.floor
    assert learned >= plant.stall
    trace = run(loop, plant, 0.6, 25)
    assert loop.floor == learned and stops(trace, cfg) == 0


def test_floor_never_probes_downward():
    """就算车一直超速(目标低于平台,根本达不到),也不许把 floor 往下探。

    往下探的收益是零(比平台更慢的速度达不到),代价是把车探停。
    """
    cfg = SpeedLoopConfig()
    loop = SpeedLoop(cfg)
    plant = Plant(cfg)
    run(loop, plant, 0.6, 6)
    before = loop.floor
    run(loop, plant, 0.20, 30)                 # 0.2 m/s 远低于 0.56 的平台
    assert loop.floor >= before


def run_with_corners(loop, plant, target, seconds, every=5.0, length=1.0, drag=0.2):
    """每 every 秒有 length 秒车被外力拖慢(急弯擦地、蹭墙):那段时间车速按 drag 衰减。
    这不是 PWM 不够 —— floor 自标定不该把它当成"塌了"。"""
    out, t = [], 0.0
    for _ in range(int(seconds / DT)):
        pwm = loop.update(target, plant.v, DT)
        v = plant.step(pwm, DT)
        if t % every > every - length:
            plant.v = v = v * (1 - (1 - drag) * DT / 0.2)
        out.append((pwm, v))
        t += DT
    return out


@pytest.mark.parametrize('target', [0.5, 0.7, 0.9])
def test_corner_slowdowns_do_not_ratchet_floor_to_max(target):
    """2026-09-29 实车:road_v2 节点限速 0.5,跑一阵后不管限速多少都冲到很快。
    原因:每次弯道减速都被当成"塌了"抬 floor,floor 只升不降,一路顶到 pwm_max。"""
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    trace = run_with_corners(loop, plant, target, 120)
    assert loop.floor <= cfg.pwm_breakaway + cfg.start_margin
    straight = [v for i, (_, v) in enumerate(trace) if i * DT > 60 and (i * DT) % 5.0 < 3.5]
    assert max(straight) <= max(target, Plant.CURVE[1][1]) + 0.25, max(straight)


def test_floor_relaxes_back_when_it_overshoots_the_plateau():
    """floor 已经被抬过头(1614,平台之上):输出压在 floor 上、车速还比目标高出一截,
    floor 要往回退,但不低于初值(不往下探到把车探停)。"""
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    run(loop, plant, 0.5, 5)
    loop.floor = cfg.pwm_breakaway + cfg.start_margin
    trace = run(loop, plant, 0.5, 30)
    assert cfg.pwm_floor <= loop.floor < 1608
    assert trace[-1][1] <= 0.5 + cfg.overspeed_margin + 0.05
    assert stops(trace, cfg) == 0


class LowPlant(Plant):
    """电调"低位"状态(2026-09-30 实测,12.40V):整条曲线比高位低 ~42 PWM,平台速度不变。
    破静 1552、最低维持 1544;1580≈0.95、1590≈1.37 m/s。"""
    CURVE = [(p - 42, v) for p, v in Plant.CURVE]
    TRUE_STALL = 1543
    TRUE_BREAK = 1552


@pytest.mark.parametrize('target', [0.6, 0.7, 0.9])
def test_low_state_esc_follows_target_instead_of_floor_speed(target):
    """低位时 floor 写死 1590 = 最低 1.37 m/s,当天策略跟不住 0.6/0.7。起步时要学到破静点,
    floor 放到它下面,车才能跟目标。"""
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), LowPlant(cfg)
    trace = run(loop, plant, target, 30)
    steady = [v for _, v in trace[len(trace) // 2:]]
    assert max(steady) <= max(target, 0.67) + 0.15, max(steady)
    assert loop.floor < 1560 and loop.floor > LowPlant.TRUE_STALL
    assert stops(trace, cfg) == 0


def test_high_state_learned_floor_stays_above_stall():
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    trace = run(loop, plant, 0.7, 30)
    assert Plant.TRUE_STALL < loop.floor <= cfg.pwm_breakaway + cfg.start_margin
    assert stops(trace, cfg) == 0


def test_standstill_encoder_spike_does_not_teach_a_low_floor():
    """编码器静止时偶尔单帧蹦 ±200mm/s:一帧"在动"不能当成起步了。"""
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    for i in range(int(3 / DT)):
        v = 0.2 if i == 5 else plant.v          # 第 5 拍(还在死区里)一帧毛刺
        loop.update(0.7, v, DT)
        plant.step(loop.last_pwm, DT)
    assert loop.floor > Plant.TRUE_STALL


def test_relearns_after_esc_power_cycle_changes_state():
    """同一个节点里电调重上电、从低位变高位:下一次起步要重新学到高位的破静点。"""
    cfg = SpeedLoopConfig()
    loop = SpeedLoop(cfg)
    run(loop, LowPlant(cfg), 0.7, 15)
    assert loop.floor < 1560
    loop.reset()
    trace = run(loop, Plant(cfg), 0.7, 30)
    assert loop.floor > Plant.TRUE_STALL
    assert stops(trace[int(8 / DT):], cfg) == 0


def test_floor_survives_a_target_change():
    """学到的 floor 不能被改目标速度清掉 —— reset() 只清控制状态。"""
    cfg = SpeedLoopConfig()
    loop = SpeedLoop(cfg)
    plant = Plant(cfg, stall=1598, brk=1604)
    run(loop, plant, 0.6, 40)
    learned = loop.floor
    loop.update(0.0, plant.v, DT)              # 停车,走 reset()
    assert loop.floor == learned


def test_integral_frozen_when_pinned_at_floor():
    """顶在 floor 上还要求减速时冻结积分 —— 电调不刹车,再积也没用,只会反向超调。"""
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    run(loop, plant, 0.6, 8)
    loop.integral = 0.0
    for _ in range(60):                        # 一直要求一个达不到的低速
        pwm = loop.update(0.1, plant.v, DT)
        plant.step(pwm, DT)
    assert loop.integral <= 0.0, f'压在 floor 上还在反向堆积分:{loop.integral}'


# --- 目标速度斜坡 -------------------------------------------------------------

def test_target_rate_limits_reference_change():
    cfg = SpeedLoopConfig(target_rate=0.3)
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    run(loop, plant, 0.75, 8)
    for _ in range(5):
        before = max(loop.ref, plant.v)
        pwm = loop.update(0.9, plant.v, DT)
        # 参考值可以跟到实际车速,但比那再往前只能走一个斜坡步长
        assert loop.ref <= before + 0.3 * DT + 1e-6
        plant.step(pwm, DT)
    assert loop.ref < 0.9


def test_target_rate_reference_starts_from_actual_speed():
    """车一动就跳到平台速度。参考值要从实际速度出发,不能从 0 慢慢爬 ——
    否则闭环会以为超速,把 PWM 往 floor 压。"""
    cfg = SpeedLoopConfig(target_rate=0.3)
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    trace = run(loop, plant, 0.8, 20)
    assert stops(trace, cfg) == 0
    assert abs(trace[-1][1] - 0.8) < 0.08


def test_target_rate_zero_follows_target_directly():
    loop = SpeedLoop(SpeedLoopConfig())
    loop.update(0.8, 0.0, DT)
    loop.last_pwm, loop.stalled = 1600.0, False
    loop.update(0.8, 0.6, DT)
    assert loop.ref == 0.8


def test_config_rejects_negative_target_rate():
    with pytest.raises(ValueError):
        SpeedLoopConfig(target_rate=-1.0)


def test_target_rate_does_not_hold_the_car_back_after_launch():
    """2026-09-24 实车:参考值从起步那一拍的 0.05 开始爬,车已经 0.5 了,
    闭环把 PWM 往下压,0.75 的段只跑到 0.63。参考值要直接跟上实际车速。"""
    cfg = SpeedLoopConfig(target_rate=0.3)
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    ref_below = 0
    for _ in range(int(8 / DT)):
        pwm = loop.update(0.8, plant.v, DT)
        if not loop.stalled and loop.ref < plant.v - 0.01:
            ref_below += 1
        plant.step(pwm, DT)
    assert ref_below == 0, f'{ref_below} 拍参考值落在车速后面'


def test_integral_does_not_wind_up_while_climb_is_rate_limited():
    """PWM 被 up_rate 卡着往上爬的时候不积分。不然爬到位时积分多出一截,冲过头
    (仿真:从平台切到 0.75,爬坡那一秒里多堆 7 个 PWM,冲到 1.01)。"""
    cfg = SpeedLoopConfig()
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    run(loop, plant, 0.5, 8)                   # 目标低于平台,压在 floor 上
    loop.integral = 0.0
    before = loop.integral
    pwm = loop.update(1.1, plant.v, DT)        # 大台阶:想要的远超一拍能爬的
    plant.step(pwm, DT)
    assert pwm == pytest.approx(loop.floor + cfg.up_rate * DT)
    assert loop.integral == before, '被爬升速率卡住时还在积分'


# --- 起步跳点 launch_from ------------------------------------------------------

def test_default_launch_is_the_old_ramp_from_neutral():
    loop = SpeedLoop()
    assert loop.update(0.7, 0.0, DT) == SpeedLoopConfig().pwm_mid + SpeedLoopConfig().up_rate * DT


def test_launch_from_skips_the_dead_band_but_still_ramps_through_breakaway():
    cfg = SpeedLoopConfig(launch_from=1580)
    loop, plant = SpeedLoop(cfg), Plant(cfg)
    trace = run(loop, plant, 0.7, 10)
    pwms = [p for p, _ in trace]
    assert pwms[0] == 1580 + cfg.up_rate * DT                  # 第一拍跳进死区
    later = [b - a for a, b in zip(pwms, pwms[1:])]
    assert max(later) <= cfg.up_rate * DT + 1e-6, '跳完之后必须慢爬'
    moving = next(i for i, (_, v) in enumerate(trace) if v >= cfg.stall_speed)
    assert moving * DT < 1.2, f'跳到 1580 之后还要 {moving * DT:.1f}s 才动'
    assert abs(trace[-1][1] - 0.7) < 0.08


def test_launch_from_cannot_jump_past_breakaway():
    with pytest.raises(ValueError):
        SpeedLoopConfig(launch_from=1600)
