"""老师:看得到整条路,先规划一条车真能开的路线,再跟踪它。

规划  在路面内求一条"曲率平方和最小"的路线(横向偏移 d(s) 的有界最小二乘),
      直角弯自然变成外-内-外。车宽和余量从可用宽度里先扣掉。
      曲率上限按实测最小转弯半径 0.83~0.88m 再留 10%:R >= 0.95m。
速度  按曲率限横向加速度,再倒推滑行减速(电调不刹车)、正推加速,
      上限取 min(速度上限, 这块电池能跑到的速度)。
      能刹车的速度契约(_d)下:brake_release_speed 以上按 A_BRK 倒推(入弯更快、松油更晚),
      光滑行来不及时标签给刹车(2026-10-19)。老契约逐位不变。
跟踪  纯追踪,按指令延迟往前推一点位姿;转角经实测舵机曲线反查成动作。
      老师知道这一局的真实车辆参数(舵机增益、中位偏差),这是它的特权。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import lsq_linear

from common.params import CarParams
from common.pwm import steer_to_turn_float
from common.road_policy import BRAKE_CMD, V_HI, speed_to_action
from simulation.fastsim.road import CONE, DS, EDGE, NONE, WALL, Road
from simulation.fastsim.road_env import CONTROL_DT, PHYSICS_DT, RoadEnv

R_MIN = 0.85                # 规划用最小转弯半径 m(车极限 0.828;0.95 是代码闸不是物理墙)
# 车身两侧到边界至少留这么多(规划时)。锥桶立在边界线上,半径 0.14 要额外扣;
# 再加 0.05 给入弯时车头外甩(车头在后轴前 0.43m)。
SIDE_MARGIN = {NONE: 0.10, WALL: 0.15, CONE: 0.14 + 0.15, EDGE: 0.15}   # EDGE:地面分割边界有几厘米误差
STEP = 5                    # 每隔几个中心线点取一个规划点(0.25m)
# 老师过弯"愿意用"的横向加速度 m/s²,决定标签里弯中能跑多快:v = sqrt(A_LAT/R)。
# 车物理抓地上限 a_lat_max = 7.85(0.8g,params.py 里标的是 [假设],实车没量过)。
#
# 默认 0.30 = main 上的历史行为,逐位不变。这个值比物理保守 26 倍,R=1m 的弯只规划到
# 0.55 m/s —— 它是压在动作空间之后的第二道闸,标签本身就从不要求快。2026-09-26 的
# "满速冲进弯"就是按它收紧的(见下面的 A_DEC/SPEED_MARGIN 注释)。
#
# 要放开得显式传 `a_lat`(road_train 的 `--a-lat`)。注意:它只有在 v_hi 明显大于 1 时
# 才有区别 —— v_hi=1.0 的默认契约下 sqrt(6R) >= 1.0 只要 R >= 0.17m,任何弯都不减速。
# 2026-09-28 在 L5c 契约上量到:v_hi=3.0 配 A_LAT 0.30->6.0,老师均速 0.94 -> 2.11 m/s、
# 均圈时 30.97 -> 13.40 s,仍 100% 出圈。
A_LAT = 0.30
A_DEC = 0.45                # 计划减速度,比实测滑行 0.68~0.78 保守得多
# 以上两个和 SPEED_MARGIN 是 2026-09-26 从 0.35/0.6/0 收紧的:L5 的失败多是满速冲进弯
# (松油晚了致命、早了无害,MSE 对两边一样罚)。老师在 A-D 上两圈只慢 0~1.8s

A_BRK = 0.85               # 计划刹车减速度(brake_release_speed 以上):实测 1.3~2.6,和 A_DEC 对滑行一样打六折左右
BRAKE_MARGIN = 0.05         # 车速比"只靠滑行来得及"的速度高出这么多才刹 m/s
A_BRK_EST = 1.4             # 估已发出、还在纯滞后里的刹车会降多少车速用(10-12 实测 1.3~1.45)
BRAKE_FLOOR = 0.7           # 不为低于它的目标刹车:平台 0.45~0.7(Plant.sample),车本来就稳不住更低的速度,
                            # 只滑行的老师在这个速度过弯没问题;不设的话平台附近刹停再起步来回顿(2026-10-19 试跑)
A_ACC = 0.4
SPEED_MARGIN = 0.25         # 速度前瞻额外多看这么远 m(提前松油)
HORIZON = 8.0               # 重规划看前方多远 m(雷达量程 10m)
REPLAN_EVERY = 1            # 拍:每拍都重规划,标签只取决于当前状态(每 5 拍时标签验证误差 0.052,每拍 0.047)
REPLAN_DEV = 0.15           # 偏离路线这么远立即重规划 m
R_REPLAN = 0.85             # 重规划允许半径 [建模选择]；左右转向极限与随机增益不同，不保证每局可实现


@dataclass
class Plan:
    pts: np.ndarray         # (m,2) 后轴要走的路线
    s: np.ndarray           # (m,)
    kappa: np.ndarray       # (m,) 有符号曲率
    v: np.ndarray           # (m,) 计划车速
    bound: np.ndarray       # (m,) 每点允许的最大 |d|
    d: np.ndarray           # (m,) 求出的横向偏移
    v_coast: np.ndarray | None = None   # (m,) 只靠滑行倒推的车速(只在 plan(brake=...) 时有)


def curvature(pts: np.ndarray, gap: int = 3) -> np.ndarray:
    """隔 gap 个点取三点外接圆,有符号(左正)。gap=3 即 0.3m 基线,压掉离散噪声。"""
    n = len(pts)
    k = np.zeros(n)
    if n < 2 * gap + 1:
        return k
    a, b, c = pts[:-2 * gap], pts[gap:-gap], pts[2 * gap:]
    ab, bc, ca = b - a, c - b, a - c
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    den = np.hypot(*ab.T) * np.hypot(*bc.T) * np.hypot(*ca.T)
    k[gap:-gap] = 2 * cross / np.maximum(den, 1e-12)
    k[:gap], k[-gap:] = k[gap], k[-gap - 1]
    return k


def plan(road: Road, i0: int, d0: float, car_width: float, pl,
         horizon: float | None = None, r_min: float = R_MIN,
         a_lat: float = A_LAT, v_end: float | None = None, brake: float | None = None) -> Plan | str:
    """返回 Plan,或者失败原因字符串('too_narrow' / 'curvature' / 'plan_error')。

    在 0.25m 一个的粗网格上解,再三次样条插回 5cm。粗点的边界取它前后半格内
    最紧的那个,插值后才不会在尖内角处越界。
    horizon  只规划前方这么远(m),None = 到路终点
    r_min    5cm 路线上允许的最小半径。**地板和闸是同一个值**:粗网格的目标函数按它
             摊弯(`over = |κ|·r_min`),插回 5cm 后再按它拒绝(`|κ| > 1/r_min`)。
             2026-09-30 之前这两处一个用参数、一个用模块常量,于是"只放宽闸"看着
             毫无效果 —— 地板没跟着动。合并成一个,改一处就是改一处。
    a_lat    弯中横向加速度上限,决定标签里弯中能跑多快(见 A_LAT)
    v_end    窗口没到路终点时,末端车速不超过它(None = 不限;见 Teacher)
    brake    计划刹车减速度(None = 不刹,老行为):brake_release_speed 以上按它倒推;
             另外存一条只靠滑行(A_DEC)倒推的 v_coast,Teacher 拿来判断要不要刹
    """
    i1 = len(road.half) if horizon is None else min(len(road.half), i0 + int(horizon / DS) + 1)
    margin = np.vectorize(SIDE_MARGIN.get)
    # 左右分开取余量:官方赛道走廊不对称(A 左下直角弯左侧 0.48m、右侧 1.65m),
    # 用 min(左,右) 会把宽的那侧白扔掉,可行域塌成一条线(2026-09-29)。
    hi_all = road.half_l[i0:i1] - car_width / 2 - margin(road.mat_left[i0:i1])
    lo_all = -(road.half_r[i0:i1] - car_width / 2 - margin(road.mat_right[i0:i1]))
    idx = np.arange(0, len(hi_all), STEP)
    if idx[-1] != len(hi_all) - 1:
        idx = np.r_[idx, len(hi_all) - 1]
    half = STEP // 2 + 1
    hi = np.array([hi_all[max(0, k - half):k + half + 1].min() for k in idx])
    lo = np.array([lo_all[max(0, k - half):k + half + 1].max() for k in idx])
    if np.any(hi[1:] < lo[1:]):
        return 'too_narrow'
    c = road.centre[i0 + idx]
    h = road.heading[i0 + idx]
    n = np.stack([-np.sin(h), np.cos(h)], axis=1)
    m = len(idx)
    lo[0], hi[0] = d0 - 1e-6, d0 + 1e-6             # 从车现在的位置出发
    # 目标:sum w_i |p_{i-1} - 2p_i + p_{i+1}|²,p = c + d·n,对 d 线性
    rhs0 = c[:-2] - 2 * c[1:-1] + c[2:]
    w = np.ones(m - 2)
    rows = np.arange(m - 2)
    for _ in range(6):
        A = np.zeros((2 * (m - 2), m))
        for ax in range(2):
            r = 2 * rows + ax
            A[r, rows] = w * n[:-2, ax]
            A[r, rows + 1] = -2 * w * n[1:-1, ax]
            A[r, rows + 2] = w * n[2:, ax]
        b = -(rhs0 * w[:, None]).reshape(-1)
        try:
            d = lsq_linear(A, b, bounds=(lo, hi), method='bvls').x
        except np.linalg.LinAlgError:
            return 'plan_error'
        if not np.all(np.isfinite(d)):      # bvls 偶尔除零给出 NaN(2026-09-25 训练第 42 轮崩过)
            return 'plan_error'
        kap = curvature(c + d[:, None] * n, gap=1)
        over = np.abs(kap[1:-1]) * r_min
        if over.max() <= 0.9:           # 粗网格留 10%,插值回 5cm 后曲率会略涨
            break
        w = w * (1 + 4 * np.maximum(0, over - 0.8) ** 2)      # 超限处加权,把弯摊开
    # 插回中心线的 5cm 采样
    dense = CubicSpline(idx, d)(np.arange(len(hi_all)))
    cd = road.centre[i0:i1]
    hd = road.heading[i0:i1]
    pts = cd + dense[:, None] * np.stack([-np.sin(hd), np.cos(hd)], axis=1)
    kap = curvature(pts, gap=6)
    if np.max(np.abs(kap)) > 1 / r_min:
        return 'curvature'
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    v_top = min(pl.cap, pl.v_top)
    v = np.minimum(v_top, np.sqrt(a_lat / np.maximum(np.abs(kap), 1e-6)))
    if v_end is not None and i1 < len(road.half):
        v[-1] = min(v[-1], v_end)
    v_coast = None
    if brake is not None:
        # 只按弯和窗口末端倒推,不含限速:平台(0.45~0.7)可能高于限速,为守限速去刹会刹停再起步来回顿
        v_coast = np.sqrt(a_lat / np.maximum(np.abs(kap), 1e-6))
        v_coast[-1] = v[-1] if v[-1] < v_top else v_coast[-1]
        for i in range(len(v) - 2, -1, -1):
            v_coast[i] = min(v_coast[i], math.sqrt(v_coast[i + 1] ** 2 + 2 * A_DEC * (s[i + 1] - s[i])))
        release = CarParams().brake_release_speed
    for i in range(len(v) - 2, -1, -1):     # 倒推:前面要慢,这里就得提前松油(能刹车时 release 以上按刹车)
        a = brake if (brake is not None and v[i + 1] >= release) else A_DEC
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a * (s[i + 1] - s[i])))
    return Plan(pts, s, kap, v, np.minimum(hi_all, -lo_all), dense, v_coast)


class Teacher:
    """replan=True:每 REPLAN_EVERY 拍、或偏离路线 > REPLAN_DEV,从车当前位姿重规划前方
    HORIZON 米(REPLAN_EVERY=1 即每拍)。标签只取决于车现在的状态和前方的路 —— 学生开偏了,老师给的是"从这里
    怎么开",不是"回到开局那条线"(那条线取决于起点,学生看不到,同一画面标签打架)。
    只钉起点位置、不钉车头方向:钉了的话被扰动后车头冲着边界,最小曲率路线把回正摊开,
    扰动恢复率掉一截(L2 0.70->0.50);不钉则纯追踪立刻打回,恢复率 0.97(2026-09-26 实测)。
    重规划失败(车已偏到规划不出来)就沿用旧路线,计入 fails。
    """

    def __init__(self, env: RoadEnv, pl_: Plan, replan: bool = True, privileged: bool = True,
                 a_lat: float = A_LAT):
        self.env, self.plan = env, pl_
        self.j = 0
        self.replan, self.n, self.fails = replan, 0, 0
        self.a_lat = a_lat
        # 高速契约(v_hi > 1):看不到的地方可能紧跟一个最急的弯,重规划窗口末端必须已经降到
        # 能过 R_MIN 的速度。v_hi<=1 时这条只在窗口最后 0.8m 起作用、老师看的点在 0.6m 内,
        # 不受影响,所以只对高速契约加,默认契约逐位不变。2026-09-29 road_v2.5 引入。
        self.v_end = math.sqrt(a_lat * R_MIN) if env.pl.enc.v_hi > V_HI else None
        self.brake = A_BRK if env.pl.enc.dead else None      # 能刹车的契约才规划刹车、给刹车标签
        self.release = CarParams().brake_release_speed
        # privileged=False:老师按名义车辆参数算(学生看不到这一局的延迟/增益/中位/纯滞后/满电速度),
        # 真实偏差靠每拍重规划吸收。标签就只取决于学生看得到的东西。
        self.pl = env.pl if privileged else replace(env.pl, lag=0.5, steer_gain=1.0, servo_offset=0.0,
                                                    dead_time=0.275, v_top=env.pl.enc.v_hi)

    def _replan(self):
        env, car = self.env, self.env.car
        i, d = env.road.project((car.x, car.y), env.idx)
        P = plan(env.road, i, d, env.cp.width, self.pl, HORIZON, R_REPLAN, self.a_lat, self.v_end, self.brake)
        if isinstance(P, str):
            self.fails += 1
            return
        self.plan, self.j = P, 0

    def __call__(self) -> np.ndarray:
        env, car = self.env, self.env.car
        if self.replan:
            dd = self.plan.pts[max(0, self.j - 20):self.j + 60] - (car.x, car.y)
            dev = math.sqrt(float(np.min(np.einsum('ij,ij->i', dd, dd))))
            if self.n % REPLAN_EVERY == 0 or dev > REPLAN_DEV:
                self._replan()
        self.n += 1
        P, pl, cp = self.plan, self.pl, env.cp
        # 按指令延迟往前推位姿:这一拍的动作要 lag 拍后才生效
        t = CONTROL_DT * (pl.lag + 0.5)
        v = max(car.v, 0.3)
        yaw = car.yaw + car.yaw_rate * t
        x = car.x + v * math.cos(yaw) * t
        y = car.y + v * math.sin(yaw) * t
        lo, hi = max(0, self.j - 20), min(len(P.pts), self.j + 60)
        dd = P.pts[lo:hi] - (x, y)
        self.j = lo + int(np.argmin(np.einsum('ij,ij->i', dd, dd)))
        look = 0.35 + 0.3 * v
        k = min(len(P.pts) - 1, int(np.searchsorted(P.s, P.s[self.j] + look)))
        tx, ty = P.pts[k]
        alpha = math.atan2(ty - y, tx - x) - yaw
        ld = max(math.hypot(tx - x, ty - y), 0.2)
        delta = math.atan2(2 * cp.wheelbase * math.sin(alpha), ld) / pl.steer_gain
        servo = steer_to_turn_float(delta, cp) - pl.servo_offset
        steer_a = max(-1.0, min(1.0, (servo - cp.turn_mid) / cp.turn_span))
        # 速度看 (纯滞后 + 一拍) 之后要到的地方
        ahead = P.s[self.j] + v * (pl.dead_time + CONTROL_DT * (pl.lag + 1)) + SPEED_MARGIN
        kv = min(len(P.v) - 1, int(np.searchsorted(P.s, ahead)))
        # 光滑行到前面来不及降下来:刹。扣掉已发出、还在纯滞后里的刹车,不然一直刹到车速降下来才松会刹过头
        q = env.speed.queue
        v_eff = car.v - A_BRK_EST * PHYSICS_DT * sum(1 for c in q if c < 0.0)
        if (P.v_coast is not None and car.v > self.release
                and v_eff > max(P.v_coast[kv], BRAKE_FLOOR) + BRAKE_MARGIN):
            return np.array([steer_a, speed_to_action(BRAKE_CMD, env.pl.enc)], np.float32)
        return np.array([steer_a, speed_to_action(float(P.v[kv]), env.pl.enc)], np.float32)
