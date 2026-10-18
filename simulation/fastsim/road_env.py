"""路段环境:结合实测参数与假设，近似模拟闭环行为,一局 = 从静止起步开到路的终点。

和实车对齐的点(数值出处见 params.py / docs/validation):
  转向  动作 a -> 舵机 turn_mid + 60a -> 实测曲线插值出转角;舵机速度、增益、中位偏差每局随机
  速度  动作 -> 目标速度 m/s -> 车上的 PID 闭环。这里模拟闭环之后的效果:
        低于平台速度就停在平台(电调在破静点一跳就到平台)、只能滑行减速(0.73 m/s²)、
        上限随电量、起步先在死区里空等
  时序  控制 20Hz;雷达 12Hz,两帧之间观测不变;指令延迟 0~1 拍
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from common.params import CarParams, TrackParams
from common.pwm import turn_to_steer
from simulation.fastsim.raycast import cast_cones, cast_walls
from simulation.fastsim.road import WALL_HEIGHT, Road, inside
from common.road_policy import DEFAULT_ENC, Encoding, action_to_speed, observe
from simulation.fastsim.vehicle import Car

CONTROL_DT = 0.05
PHYSICS_DT = 0.01
OBS_RANGE = 10.0
TRACK = TrackParams()
GOAL_BEFORE_END = 1.0       # 离路终点这么远就算到达
# 加强噪声(robust=True)。实车:编码器静止时偶尔单帧蹦 ±200mm/s(ramp_curve 被骗过);
# 雷达 USB 掉过线、会被人/杂物挡一段。概率是拍脑袋的量级,不是实测
ROBUST_SPIKE_P = 0.03       # 每拍编码器毛刺概率
ROBUST_BLOCK_P = 0.05       # 每帧雷达整段缺失概率


@dataclass
class Plant:
    """一局的车辆参数。老师与学生共用；检查通过只证明该模拟条件下老师可完成。"""
    steer_rate: float = 5.0         # rad/s
    steer_gain: float = 1.0
    servo_offset: float = 0.0       # 舵机单位,中位 80/81 分不清
    lag: int = 0                    # 指令延迟拍数
    v_plateau: float = 0.6          # 最低稳定速度
    v_top: float = 1.0              # PWM 上限能跑到的速度
    accel_up: float = 0.6           # 平台以上的加速 m/s²
    decel: float = 0.73             # 滑行减速 m/s²
    dead_time: float = 0.28         # 速度环纯滞后 s
    launch_delay: float = 1.5       # 起步在死区里空等 s(sample 抽 0~0.2:2026-10-18 实车 6 段放行后 0.3 s 内车速都已到 0.4~0.5;
                                    # 原来抽 0.5~3.0,模型出发前能在原地攒好几秒 BEV,车上放行时 BEV 是清空的)
    lidar_yaw: float = 0.0          # 雷达安装偏角 rad
    cap: float = 0.7                # 速度上限(网页设的),策略看得到
    enc: Encoding = DEFAULT_ENC     # 这一局的速度契约(观测尺度 + 动作解码区间),由训练配置决定

    @staticmethod
    def sample(rng: np.random.Generator, enc: Encoding = DEFAULT_ENC,
               cap_max: float | None = None) -> 'Plant':
        """cap_max:课程分档时速度上限只抽到这里(默认到契约的 v_hi)。抽取顺序不变。"""
        return Plant(steer_rate=rng.uniform(3.5, 7.0), steer_gain=rng.uniform(0.9, 1.1),
                     servo_offset=rng.uniform(-1.0, 1.0), lag=int(rng.integers(0, 2)),
                     v_plateau=rng.uniform(0.45, 0.70), v_top=rng.uniform(0.85, 1.1) * enc.v_hi,
                     accel_up=rng.uniform(0.4, 1.0), decel=rng.uniform(0.68, 0.78),
                     dead_time=rng.uniform(0.2, 0.35), launch_delay=rng.uniform(0.0, 0.2),
                     lidar_yaw=math.radians(rng.uniform(-1.0, 1.0)),
                     cap=rng.uniform(enc.v_lo, cap_max or enc.v_hi), enc=enc)


class SpeedPlant:
    """闭环后车速的近似模型，并未执行真实 SpeedLoop。仅用于正速度指令，不支持停车命令。"""

    def __init__(self, pl: Plant):
        self.pl, self.v, self.t = pl, 0.0, 0.0
        self.queue = [0.0] * max(1, int(round(pl.dead_time / PHYSICS_DT)))

    def step(self, cmd: float, dt: float = PHYSICS_DT) -> float:
        pl = self.pl
        self.queue.append(cmd)
        cmd = self.queue.pop(0)
        self.t += dt
        if self.t < pl.launch_delay:
            return self.v
        target = min(max(cmd, pl.v_plateau), pl.v_top)
        if self.v < pl.v_plateau - 0.01:                # 过了破静点一跳到平台
            self.v += (pl.v_plateau - self.v) * min(1.0, dt / 0.1)
        elif target > self.v:
            self.v = min(target, self.v + pl.accel_up * dt)
        else:
            self.v = max(target, self.v - pl.decel * dt)
        return self.v


class RoadEnv:
    """不走 gymnasium:训练是 DAgger,不需要那层接口,少一层开销。"""

    def __init__(self, road: Road, pl: Plant, seed: int, start: tuple = (0, 0.0, 0.0),
                 noise: bool = True, car_params: CarParams = CarParams(), sense: bool = True,
                 robust: bool = False):
        self.road, self.pl, self.sense = road, pl, sense
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.robust = robust and noise     # 加强噪声(实车见过的):编码器毛刺、雷达整段缺失
        # 纵向由 SpeedPlant 给定,Car 只管跟上(tau 小于步长 = 直接等于)
        self.cp = replace(car_params, steer_rate=pl.steer_rate, speed_tau=1e-4, accel_max=1e3)
        self.car = Car(self.cp)
        i0, d0, yaw0 = start
        c, h = road.centre[i0], road.heading[i0]
        self.car.reset(c[0] - math.sin(h) * d0, c[1] + math.cos(h) * d0, h + yaw0,
                       steer_gain=pl.steer_gain)
        self.speed = SpeedPlant(pl)
        self.idx = i0
        self.t, self.next_scan = 0.0, 0.0
        self.pending = [(0.0, pl.enc.v_lo)] * pl.lag       # 指令延迟队列
        self.cmd = (0.0, pl.enc.v_lo)
        n = car_params.lidar_points
        self.angles = -math.pi + np.arange(n) * (2 * math.pi / n)
        self.min_clear = math.inf
        self.last = np.zeros(2, np.float32)
        self.ranges = None
        if sense:
            self._scan()
        self.budget_s = self.budget()

    # ---- 传感器 ----
    def scan_at(self, x, y, yaw) -> np.ndarray:
        p = self.cp
        c, s = math.cos(yaw), math.sin(yaw)
        origin = np.array([x + p.lidar_x * c, y + p.lidar_x * s, p.lidar_z])
        world = self.angles + yaw + self.pl.lidar_yaw
        dirs = np.stack([np.cos(world), np.sin(world), np.zeros_like(world)], axis=1)
        r = np.full(len(dirs), np.inf)
        w = self.road.walls
        if len(w):
            near = _seg_dist(origin[:2], w) < OBS_RANGE + 0.5
            if near.any():
                r = cast_walls(origin, dirs, w[near], WALL_HEIGHT, p.lidar_max)
        if len(self.road.cones):
            r = np.minimum(r, cast_cones(origin, dirs, self.road.cones, self.road.cone_radius,
                                         TRACK.cone_top_radius, TRACK.cone_height, OBS_RANGE + 0.5))
        return r

    def _scan(self):
        car = self.car
        r = self.scan_at(car.x, car.y, car.yaw)
        if self.noise:
            r = r + self.rng.normal(0, self.cp.lidar_sigma, r.shape)
            r[self.rng.random(r.shape) < 0.02] = np.inf
            if self.robust and self.rng.random() < ROBUST_BLOCK_P:
                # 一整段角度没回波(USB 掉帧、被遮挡):10~40° 连续缺失
                a0, w = self.rng.uniform(-math.pi, math.pi), math.radians(self.rng.uniform(10, 40))
                r[np.abs((self.angles - a0 + math.pi) % (2 * math.pi) - math.pi) < w / 2] = np.inf
        self.ranges = np.where(r >= self.cp.lidar_min, r, np.inf)

    def observation(self) -> np.ndarray:
        n = self.noise
        enc = self.car.v + (self.rng.normal(0, 0.02) if n else 0.0)
        if self.robust and self.rng.random() < ROBUST_SPIKE_P:
            enc += self.rng.choice([-1.0, 1.0]) * self.rng.uniform(0.1, 0.25)   # 单帧毛刺
        gyro = self.car.yaw_rate + (self.rng.normal(0, self.cp.gyro_sigma) if n else 0.0)
        return observe(self.ranges, self.angles, enc, self.pl.cap, gyro, self.last,
                       scale=self.pl.enc.scale)

    # ---- 一拍 ----
    def step(self, action) -> str | None:
        """执行一拍 50ms。返回 None 继续,或结束原因 goal/collision/off_track/timeout。"""
        a = np.clip(np.asarray(action, np.float32), -1, 1)
        self.last = a
        self.pending.append((float(a[0]), action_to_speed(float(a[1]), self.pl.enc)))
        self.cmd = self.pending.pop(0)
        steer_a, v_cmd = self.cmd
        pl = self.pl
        servo = self.cp.turn_mid + steer_a * self.cp.turn_span + pl.servo_offset
        steer = turn_to_steer(servo, self.cp)
        v_cmd = min(v_cmd, pl.cap)
        for _ in range(int(round(CONTROL_DT / PHYSICS_DT))):
            v = self.speed.step(v_cmd)
            self.car.step(v, steer, PHYSICS_DT)
            self.t += PHYSICS_DT
            why = self._check()
            if why:
                return why
        if self.sense and self.t >= self.next_scan:
            self._scan()
            self.next_scan += 1.0 / self.cp.lidar_hz
        return None

    def budget(self) -> float:
        return self.pl.launch_delay + (self.road.length - self.road.s[self.idx]) / 0.3 + 10.0

    def _check(self) -> str | None:
        car, road = self.car, self.road
        self.idx, d = road.project((car.x, car.y), self.idx)
        clear = footprint_clearance(car, road)
        self.min_clear = min(self.min_clear, clear)
        if clear <= 0.0:
            return 'collision'
        # 车体中心离开路面(无边界段/没墙的一侧就靠这个判)
        mid = (car.x + math.cos(car.yaw) * (self.cp.length / 2 - self.cp.rear_overhang),
               car.y + math.sin(car.yaw) * (self.cp.length / 2 - self.cp.rear_overhang))
        if road.region is not None:
            outer, inner = road.region
            if not inside(outer, mid) or inside(inner, mid):
                return 'off_track'
        else:
            _, dm = road.project(mid, self.idx)
            if abs(dm) > road.half[self.idx]:
                return 'off_track'
        if road.s[self.idx] >= road.length - GOAL_BEFORE_END:
            return 'goal'
        if self.t > self.budget_s:
            return 'timeout'
        return None


# ---- 几何 ----
def _seg_dist(p, segs) -> np.ndarray:
    a, b = segs[:, 0], segs[:, 1]
    ab = b - a
    t = np.clip(np.einsum('ij,ij->i', p - a, ab) / np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-12), 0, 1)
    q = a + ab * t[:, None]
    return np.hypot(*(q - p).T)


def footprint_clearance(car: Car, road: Road, reach: float = 1.0) -> float:
    """车体矩形到最近的墙/锥桶的距离,<=0 就是碰上了。只看 reach 米内的东西。"""
    p = car.p
    c, s = math.cos(car.yaw), math.sin(car.yaw)
    x_lo, x_hi, hw = -p.rear_overhang, p.length - p.rear_overhang, p.width / 2
    ctr = np.array([car.x, car.y])
    best = math.inf
    if len(road.walls):
        w = road.walls
        mid = (w[:, 0] + w[:, 1]) / 2
        near = np.hypot(*(mid - ctr).T) < p.length + reach
        if near.any():
            best = min(best, _rect_seg_dist(w[near] - ctr, c, s, x_lo, x_hi, hw))
    if len(road.cones):
        rel = road.cones - ctr
        near = np.hypot(*rel.T) < p.length + reach
        if near.any():
            rel = rel[near]
            lx, ly = rel[:, 0] * c + rel[:, 1] * s, -rel[:, 0] * s + rel[:, 1] * c
            dx = np.maximum(np.maximum(x_lo - lx, 0.0), lx - x_hi)
            dy = np.maximum(np.abs(ly) - hw, 0.0)
            best = min(best, float(np.min(np.hypot(dx, dy))) - road.cone_radius)
    return best


def _rect_seg_dist(segs, c, s, x_lo, x_hi, hw) -> float:
    """车体系矩形 [x_lo,x_hi]x[-hw,hw] 到一组线段(世界系,已减车位置)的最小距离。

    线段穿过矩形 -> 0。否则最小距离在 线段端点到矩形 或 矩形角点到线段 之间取。
    """
    def to_body(q):
        return np.stack([q[..., 0] * c + q[..., 1] * s, -q[..., 0] * s + q[..., 1] * c], -1)
    a, b = to_body(segs[:, 0]), to_body(segs[:, 1])

    def pt_rect(q):
        dx = np.maximum(np.maximum(x_lo - q[:, 0], 0.0), q[:, 0] - x_hi)
        dy = np.maximum(np.abs(q[:, 1]) - hw, 0.0)
        return np.hypot(dx, dy)
    d = np.minimum(pt_rect(a), pt_rect(b))
    corners = np.array([[x_lo, -hw], [x_lo, hw], [x_hi, -hw], [x_hi, hw]])
    ab = b - a
    L2 = np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-12)
    for q in corners:
        t = np.clip(np.einsum('ij,ij->i', q - a, ab) / L2, 0, 1)
        d = np.minimum(d, np.hypot(*(a + ab * t[:, None] - q).T))
    # 穿过:Liang-Barsky 裁剪,区间非空就相交
    t0, t1 = np.zeros(len(a)), np.ones(len(a))
    for pk, qk in ((-ab[:, 0], a[:, 0] - x_lo), (ab[:, 0], x_hi - a[:, 0]),
                   (-ab[:, 1], a[:, 1] + hw), (ab[:, 1], hw - a[:, 1])):
        with np.errstate(divide='ignore', invalid='ignore'):
            r = qk / pk
        par = np.abs(pk) < 1e-12
        t0 = np.where(~par & (pk < 0), np.maximum(t0, r), t0)
        t1 = np.where(~par & (pk > 0), np.minimum(t1, r), t1)
        t1 = np.where(par & (qk < 0), -1.0, t1)
    d = np.where(t0 <= t1, 0.0, d)
    return float(d.min())
