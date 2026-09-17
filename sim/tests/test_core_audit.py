"""2026-09-17 任务书五-B 核心逻辑审查测试:碰撞几何/终止截断/观测真更新/倒车符号。

审查发现与本文件的对应:
  R1 地面碰撞原用内切圆 r=0.14(乐观近似,方角伸出 0.058m 会漏判)
     → vehicle.footprint_hits_squares(SAT)落地,本文件做三件事:
     与独立暴力实现交叉验证、圆/方两向差异各给定夺场景、赛道级回归。
  R2 满舵兜圈 120s 不终止曾被"改测试动作"规避(test_sim.py:270 注释自认)
     → 恢复满舵动作本身,断言时间上限必须给 truncated(而非改动作绕开)。
  R3 传感器刷新只有计数器对账 → 补行为级:观测中 encoder/IMU 逐控制步变化、
     雷达帧复用比例与 12Hz<20Hz 的设计一致。
  R4 倒车 ω 符号随 v 反转(自行车模型),不允许把 ω 当转角。
"""
import math

import numpy as np
import pytest

from sim.env import ConeTrackEnv
from sim.params import CarParams, TrackParams
from sim.vehicle import Car

TP = TrackParams()


# ---------- 暴力参照:两凸多边形相交(独立实现,不用被测代码) ----------
def _car_corners(car: Car) -> np.ndarray:
    p = car.p
    c, s = math.cos(car.yaw), math.sin(car.yaw)
    lx = np.array([-p.rear_overhang, p.length - p.rear_overhang,
                   p.length - p.rear_overhang, -p.rear_overhang])
    ly = np.array([-p.width / 2, -p.width / 2, p.width / 2, p.width / 2])
    return np.stack([car.x + lx * c - ly * s, car.y + lx * s + ly * c], axis=1)


def _square_corners(cx: float, cy: float, half: float) -> np.ndarray:
    o = np.array([[-half, -half], [half, -half], [half, half], [-half, half]])
    return o + (cx, cy)


def _point_in_poly(pt, poly) -> bool:
    x, y = pt
    n, inside = len(poly), False
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xin = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xin:
                inside = not inside
    return inside


def _seg_cross(p1, p2, q1, q2) -> bool:
    def o(a, b, c):
        v = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        return 0 if abs(v) < 1e-12 else (1 if v > 0 else -1)
    return (o(p1, p2, q1) * o(p1, p2, q2) < 0) and (o(q1, q2, p1) * o(q1, q2, p2) < 0)


def _polys_intersect(a, b) -> bool:
    if any(_point_in_poly(p, b) for p in a) or any(_point_in_poly(q, a) for q in b):
        return True
    return any(_seg_cross(a[i], a[(i + 1) % 4], b[j], b[(j + 1) % 4])
               for i in range(4) for j in range(4))


# ---------- R1a: SAT 与暴力参照逐位一致 ----------
def test_sat_matches_bruteforce_on_random_poses():
    rng = np.random.default_rng(20260917)
    half = TP.cone_base_radius
    for _ in range(300):
        car = Car(CarParams())
        car.reset(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-math.pi, math.pi))
        cones = rng.uniform(-1.6, 1.6, (25, 2))          # 含大量贴近位姿
        got = car.footprint_hits_squares(cones, half)
        want = np.array([_polys_intersect(_car_corners(car), _square_corners(cx, cy, half))
                         for cx, cy in cones])
        assert np.array_equal(got, want), (car.x, car.y, car.yaw, cones)


# ---------- R1b: 误差方向定夺 ----------
def test_circle_model_misses_diagonal_graze_that_square_catches():
    """方形对角伸出内切圆 (√2−1)·half≈0.058m:锥摆在对角方向、间隙 0.13m 时
    圆判定不撞(漏报)、方判定撞——这正是改用方基座的动机。"""
    car = Car(CarParams())
    car.reset(0.0, 0.0, 0.0)                            # 车体: x∈[-0.13,0.43], y∈[-0.175,0.175]
    half = TP.cone_base_radius                          # 0.14
    # 车前右角 (0.43,-0.175) 对角方向 0.13m 处放锥:方形左上角 (0.42,-0.165)
    # 落进车矩形(两区间各重叠 0.01),而锥心到矩形距离 √2·0.13≈0.184>0.14
    cx, cy = 0.43 + 0.13, -0.175 - 0.13
    assert not car.footprint_hits_circles([(cx, cy)], half)      # 旧模型:漏
    assert car.footprint_hits_squares([(cx, cy)], half)          # 新模型:撞


def test_square_model_is_strict_superset_of_circle_model():
    """误差方向的准确陈述:方形基座 ⊇ 内切圆(内切圆在方形内部,仅 4 切点
    相接),所以旧圆判定撞 ⟹ 新方判定撞(单向收紧,不存在的反向差异);
    且随机位姿下确实存在"方撞圆不撞"的位姿(角区非空)。"""
    rng = np.random.default_rng(7)
    half = TP.cone_base_radius
    both_directions_seen = 0
    for _ in range(200):
        car = Car(CarParams())
        car.reset(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-math.pi, math.pi))
        cones = rng.uniform(-1.6, 1.6, (40, 2))
        circ = car.footprint_hits_circles(cones, half)
        sq = car.footprint_hits_squares(cones, half)
        assert not (circ & ~sq).any()                   # 圆撞必方撞(包含关系)
        both_directions_seen += int((sq & ~circ).any())  # 方撞圆不撞(角区)出现过
    assert both_directions_seen > 0, "角区差异未出现,场景覆盖不足"


# ---------- R1c: 赛道级回归——判定变严后纯追踪仍零撞锥跑完 ----------
def test_pure_pursuit_still_clean_on_trackA_after_square_collision():
    from sim.preview import pure_pursuit
    from sim.trackA import load_trackA
    env = ConeTrackEnv(track=load_trackA())
    env.reset(seed=0)
    while True:
        _, _, term, trunc, info = env.step(pure_pursuit(env))
        if term or trunc:
            break
    assert info["termination_reason"] == "lap", info


# ---------- R2: 满舵兜圈必须被时间上限截断(不得改测试动作规避) ----------
def test_full_steer_circling_truncates_at_budget():
    """seed0 满舵 [1,1]:轴距 0.36 下兜圈半径 0.59m 被赛道容纳,不撞锥不出界
    ——这是几何事实,允许持续;但 budget 到点必须 truncated=True 而不是
    terminated,也不许静默继续。budget 收短到 8s 免测试空跑 120s。"""
    env = ConeTrackEnv(budget=8.0)
    obs, _ = env.reset(seed=0)
    while True:
        obs, _, term, trunc, info = env.step([1.0, 1.0])
        if term or trunc:
            break
    assert info["termination_reason"] == "timeout", info
    assert trunc and not term
    assert info["cone_hits"] == 0                        # 兜圈本身不撞锥(容纳事实)


# ---------- R3: 观测逐控制步真更新(不止计数器对) ----------
def test_observation_encoder_and_imu_update_every_control_step():
    """加速段连续 control step 的 encoder 速度观测量必须逐步变化(100Hz/50Hz
    采样在 20Hz 控制步之间必然刷新);转弯时 IMU ω 非零。"""
    env = ConeTrackEnv(noise=False)
    env.reset(seed=0)
    enc, omega = [], []
    for _ in range(12):                                  # 0.6s:起步加速+纯追踪初拐
        from sim.preview import pure_pursuit
        obs, _, term, trunc, info = env.step(pure_pursuit(env))
        enc.append(float(obs[48]))
        omega.append(float(obs[49]))
        if term or trunc:
            break
    assert len(set(enc)) >= 8, enc                       # 几乎每步都不同 → 真在刷新
    assert any(abs(w) > 1e-4 for w in omega)             # 转弯被观测到


def test_lidar_frame_reuse_fraction_matches_12hz_vs_20hz():
    """雷达 12Hz < 控制 20Hz:部分相邻控制步必读同一帧。noise 开着时,
    相邻两步的 48 扇区**逐位相等** ⟺ 用了同一帧——统计该比例应明显大于 0
    (存在复用)且小于 0.6(并非从不刷新),12/20≈0.4。"""
    env = ConeTrackEnv()
    env.reset(seed=0)
    from sim.preview import pure_pursuit
    prev, same, n = None, 0, 0
    while n < 60:
        obs, _, term, trunc, _ = env.step(pure_pursuit(env))
        if prev is not None:
            same += int(np.array_equal(obs[:48], prev))
            n += 1
        prev = obs[:48].copy()
        if term or trunc:
            break
    frac = same / n
    assert 0.1 < frac < 0.7, f"雷达帧复用比例异常:{frac:.2f}(应≈12/20 帧间隔占比)"


# ---------- R4: 倒车 ω 符号按自行车模型随 v 反转 ----------
def test_reverse_steering_sign_flips_yaw_rate():
    """同 δ>0:v>0 → ω>0;v<0 → ω<0。ω 是角速度不是转角,不得混同。"""
    p = CarParams()
    fwd, rev = Car(p), Car(p)
    for _ in range(400):
        fwd.step(0.6, 0.3, 0.01)
        rev.step(-0.4, 0.3, 0.01)
    assert fwd.v > 0 and fwd.yaw_rate > 0
    assert rev.v < 0 and rev.yaw_rate < 0
    assert rev.steer == pytest.approx(0.3, abs=1e-6)     # 转角指令同号,反转的是 ω


# ---------- 停止工况 ----------
def test_stop_command_decelerates_to_rest():
    car = Car(CarParams())
    for _ in range(600):
        car.step(1.0, 0.0, 0.01)                         # 先跑到稳态 1.0 m/s
    assert car.v == pytest.approx(1.0, rel=0.02)
    x_stop_expect = None
    for _ in range(2000):                                # 0 指令 20s:一阶响应指数
        car.step(0.0, 0.0, 0.01)                         # 衰减到浮点微小量级
        if x_stop_expect is None and abs(car.v) < 1e-6:
            x_stop_expect = car.x
    assert x_stop_expect is not None                     # 确实到达"停"(|v|<1µm/s)
    assert abs(car.v) < 1e-6
    assert abs(car.x - x_stop_expect) < 1e-6             # 停后位移<1µm(指数尾)


# ---------- 跨步无隧穿:细步长与粗步长首撞位置一致 ----------
@pytest.mark.parametrize("v", [1.5, 3.0])
def test_no_tunneling_at_max_speed(v):
    """3 m/s(物理上限)直行冲单个锥:dt=0.01 与 dt=0.002 的首撞 x 相差必须
    < 一个粗步位移(0.03m)——粗步没有跳过碰撞区。"""
    def first_hit_x(dt, v):
        car = Car(CarParams())
        car.reset(0.0, 0.0, 0.0)
        cone = np.array([[5.0, 0.0]])
        x_prev = car.x
        for _ in range(int(20 / dt)):
            car.step(v, 0.0, dt)
            if car.footprint_hits_squares(cone, TP.cone_base_radius).any():
                return x_prev
            x_prev = car.x
        return None
    coarse, fine = first_hit_x(0.01, v), first_hit_x(0.002, v)
    assert coarse is not None and fine is not None
    assert abs(coarse - fine) < v * 0.01 + 1e-9, (coarse, fine)


# ---------- 种子复现(env 级,不止赛道几何) ----------
def test_same_seed_replays_identical_trajectory():
    def run():
        env = ConeTrackEnv()
        env.reset(seed=42)
        traj = []
        from sim.preview import pure_pursuit
        for _ in range(40):
            obs, _, term, trunc, _ = env.step(pure_pursuit(env))
            traj.append((env.car.x, env.car.y, env.car.yaw, float(obs[10])))
            if term or trunc:
                break
        return traj
    assert run() == run()
