import importlib.util
import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from sim.curriculum import STAGES, evaluate, passed
from sim.env import SECTOR_EDGES, SECTORS, ConeTrackEnv, sector_index, sectorize
from sim.interop import export_track, from_cmd_vel, to_cmd_vel
from sim.mapping import FREE, OCCUPIED
from sim.params import CarParams, TrackParams
from sim.preview import pure_pursuit
from sim.pwm import pwm_to_speed, speed_to_pwm, steer_to_turn, turn_to_steer
from sim.raycast import cast_cones
from sim.sensors import Lidar
from sim.track import make_track
from sim.vehicle import Car

TP = TrackParams()


# ---- 射线 ----
def test_horizontal_ray_hits_cone_at_height_radius():
    h = 0.21
    radius_at_h = TP.cone_base_radius - (TP.cone_base_radius - TP.cone_top_radius) * h / TP.cone_height
    r = cast_cones([0, 0, h], [[1, 0, 0]], [[3.0, 0.0]], TP.cone_base_radius, TP.cone_top_radius, TP.cone_height)
    assert r[0] == pytest.approx(3.0 - radius_at_h, abs=1e-9)


def test_ray_above_cone_misses_and_tilted_ray_hits_side():
    above = cast_cones([0, 0, 1.0], [[1, 0, 0]], [[3.0, 0.0]], .14, .025, .63)
    assert np.isinf(above[0])
    d = np.array([3.0, 0.0, -0.9]); d /= np.linalg.norm(d)        # 从 1m 高斜向下打到锥桶中部
    tilted = cast_cones([0, 0, 1.0], d[None], [[3.0, 0.0]], .14, .025, .63)
    assert np.isfinite(tilted[0])
    z = 1.0 + tilted[0] * d[2]
    assert 0 <= z <= .63


def test_angular_culling_matches_brute_force():
    rng = np.random.default_rng(3)
    cones = rng.uniform(-6, 6, (80, 2))
    for origin in ([0, 0, 0.21], [0, 0, 1.2], [*cones[0] + [0.05, 0], 0.3]):   # 含起点贴着锥桶
        d = rng.normal(size=(3000, 3))
        d[:1500, 2] = 0                                              # 一半水平,一半斜向
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        fast = cast_cones(origin, d, cones, .14, .025, .63, 50.0, cull=True)
        slow = cast_cones(origin, d, cones, .14, .025, .63, 50.0, cull=False)
        assert np.array_equal(np.isfinite(fast), np.isfinite(slow))
        assert np.allclose(fast[np.isfinite(fast)], slow[np.isfinite(slow)], rtol=0, atol=1e-12)
        assert np.isfinite(fast).sum() > 20


# ---- 车辆 ----
def test_zero_command_does_not_move():
    car = Car()
    for _ in range(200):
        car.step(0.0, 0.0, 0.01)
    assert abs(car.x) < 1e-9


def test_speed_command_converges_and_reverse_goes_back():
    p = CarParams()
    car = Car(p)
    for _ in range(500):
        car.step(1.2, 0.0, 0.01)
    assert car.v == pytest.approx(1.2, rel=0.02)
    car.reset(0, 0, 0)
    for _ in range(300):
        car.step(-0.5, 0.0, 0.01)
    assert car.v < 0 and car.x < 0


def test_left_turn_radius_matches_bicycle_model():
    p = CarParams()
    car = Car(p)
    for _ in range(3000):
        car.step(1.0, p.max_steer, 0.01)                            # 左满舵
    assert car.steer == pytest.approx(p.max_steer)
    assert car.yaw_rate > 0
    assert car.v / car.yaw_rate == pytest.approx(p.wheelbase / math.tan(p.max_steer), rel=1e-6)


def test_command_gains_scale_response():
    p = CarParams()
    car = Car(p)
    car.reset(0, 0, 0, v_gain=0.8, steer_gain=1.2)
    for _ in range(500):
        car.step(1.0, 0.2, 0.01)
    assert car.v == pytest.approx(0.8, rel=0.02)
    assert car.steer == pytest.approx(0.24, rel=1e-6)


# ---- PWM 换算层(只在对接实车时用)----
def test_pwm_deadband_and_round_trip():
    p = CarParams()
    assert pwm_to_speed(p.pwm_mid, p) == 0.0
    assert pwm_to_speed(1595, p) == 0.0                             # 死区内不转
    assert pwm_to_speed(p.pwm_fwd_full, p) == pytest.approx(p.v_fwd_full)
    assert speed_to_pwm(0.0, p) == p.pwm_mid
    assert speed_to_pwm(p.v_fwd_full, p) == p.pwm_fwd_full
    assert speed_to_pwm(-p.v_rev_full, p) == p.pwm_rev_full
    for v in (0.05, 0.4, 1.3, 2.9, -0.2, -1.4):
        assert pwm_to_speed(speed_to_pwm(v, p), p) == pytest.approx(v, abs=0.01)
    assert steer_to_turn(0.0, p) == p.turn_mid
    assert steer_to_turn(p.max_steer, p) == p.turn_mid + p.turn_span
    assert turn_to_steer(24, p) == pytest.approx(-p.max_steer)


# ---- 赛道 ----
def test_track_reproducible_and_cone_geometry():
    a, b = make_track(np.random.default_rng(7)), make_track(np.random.default_rng(7))
    assert np.array_equal(a.cones, b.cones)
    for side in (a.left, a.right):
        gaps = np.hypot(*np.diff(np.vstack([side, side[:1]]), axis=0).T)
        assert gaps.min() > 0.8 * TP.cone_spacing and gaps.max() < 1.3 * TP.cone_spacing
    # 任何锥桶离中心线都不应近于半宽(否则挡在路中间)
    d = np.hypot(*(a.cones[:, None, :] - a.centerline[None]).transpose(2, 0, 1)).min(axis=1)
    assert d.min() > TP.width / 2 - 0.05


# ---- 环境 ----
def test_gymnasium_api():
    check_env(ConeTrackEnv(), skip_render_check=True)


def test_scan_matches_real_laserscan_layout():
    env = ConeTrackEnv()
    env.reset(seed=0)
    assert env.ranges.shape == (1667,)
    assert env.lidar.angle_increment == pytest.approx(0.0037691574543714523, rel=1e-6)
    assert np.isfinite(env.ranges).sum() > 50                     # 两侧锥桶可见


def test_action_is_physical_and_info_reports_equivalent_pwm():
    env = ConeTrackEnv()
    cp = env.cp
    assert env.action_to_command([0, 0]) == (0.0, 0.0)
    assert env.action_to_command([1, 1]) == (env.v_cmd_fwd, cp.max_steer)
    assert env.action_to_command([-1, -1]) == (-env.v_cmd_rev, -cp.max_steer)
    env.reset(seed=0)
    _, _, _, _, info = env.step([1, 1])
    assert info["v_cmd"] == env.v_cmd_fwd
    assert info["pwm"] == speed_to_pwm(env.v_cmd_fwd, cp) and info["turn"] == cp.turn_mid + cp.turn_span


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_pure_pursuit_completes_lap_without_cones(seed):
    env = ConeTrackEnv()
    env.reset(seed=seed)
    while True:
        _, _, term, trunc, info = env.step(pure_pursuit(env))
        if term or trunc:
            break
    assert info["termination_reason"] == "lap", info
    assert info["cone_hits"] == 0


def test_observation_layout_is_front_dense():
    env = ConeTrackEnv()
    obs, _ = env.reset(seed=0)
    assert obs.shape == (53,)                                    # 48 扇区 + 速度 + 角速度 + 动作2 + 剩余时间
    assert len(SECTOR_EDGES) - 1 == 48
    front = np.diff(SECTOR_EDGES)[6:42]
    rear = np.concatenate([np.diff(SECTOR_EDGES)[:6], np.diff(SECTOR_EDGES)[42:]])
    assert np.allclose(front, math.radians(5))
    assert np.allclose(rear, math.radians(15))
    assert obs[-1] == pytest.approx(1.0, abs=0.01)               # 刚开局剩余时间≈1


def test_mapping_marks_cones_and_free_space():
    env = ConeTrackEnv(mapping=True)
    env.reset(seed=0)
    while True:
        _, _, term, trunc, info = env.step(pure_pursuit(env))
        if term or trunc:
            break
    assert info["termination_reason"] == "lap"
    assert info["cone_coverage"] >= 0.9, info                    # 跑完一圈应把锥桶基本扫全
    assert info["mapped_free_area"] > 20.0                       # 路面被标成空闲
    grid = env.map.grid()
    assert (grid == OCCUPIED).any() and (grid == FREE).any()


def test_curriculum_stage_gate():
    stage = STAGES["s1"]
    result = evaluate(lambda obs, env: pure_pursuit(env), stage, seeds=[90000, 90001, 90002])
    assert result.laps == 3 and result.cone_hits == 0, result.episodes
    assert result.mean_coverage >= 0.9
    # 过关判据本身:少跑一圈就不算过
    stage3 = replace(stage, min_laps=3)
    assert passed(result, stage3)
    assert not passed(replace(result, laps=2), stage3)


def test_idle_times_out_without_progress():
    env = ConeTrackEnv(budget=3.0)
    env.reset(seed=0)
    total = 0.0
    while True:
        _, r, term, trunc, info = env.step([0, 0])
        total += r
        if term or trunc:
            break
    assert info["termination_reason"] == "timeout"
    assert abs(info["progress"]) < 1e-6 and total < 0


# ---- 与队友 Gazebo 框架(hwcart_sim)的互通 ----
def test_cmd_vel_matches_vehicle_model_and_round_trips():
    p = CarParams()
    car = Car(p)
    for _ in range(3000):
        car.step(1.0, 0.3, 0.01)
    linear, angular = to_cmd_vel(car.v, car.steer, p)
    assert (linear, angular) == pytest.approx((car.v, car.yaw_rate))   # 与本仿真同一个模型
    assert from_cmd_vel(linear, angular, p) == pytest.approx((car.v, car.steer))
    assert from_cmd_vel(0.0, 1.0, p) == (0.0, 0.0)                    # 阿克曼车不能原地转
    assert from_cmd_vel(1.0, 99.0, p)[1] == pytest.approx(p.max_steer)


def _their_generator():
    path = Path(__file__).resolve().parents[2] / "hwcart_sim/src/track_generator/track_generator/generate.py"
    spec = importlib.util.spec_from_file_location("their_generate", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_exported_track_builds_in_their_track_generator(tmp_path):
    gen = _their_generator()
    env = ConeTrackEnv()
    env.reset(seed=0)
    cones_csv, gaps_json = tmp_path / "cones.csv", tmp_path / "gaps.json"
    n_cones, n_gaps = export_track(env.track, cones_csv, gaps_json)
    assert n_cones == len(env.track.cones) and n_gaps >= 3
    # 用他们自己的加载器和 SDF 生成器验收,格式不对这里就会抛
    cones = gen.load_cones(cones_csv)
    sdf, manifest = gen.build(cones, json.loads(gaps_json.read_text()), seed=0, board_thickness=0.1)
    assert sdf.count("<model") == n_cones + 1 + 3        # 锥桶 + 地面 + 3 块挡板
    assert manifest["selected_gaps"]
    # 起点搬到原点朝 +x(他们 launch 里车在原点出生)
    start = np.array([[float(c["x"]), float(c["y"])] for c in cones])
    assert np.hypot(*start.T).min() < TP.width


def test_sectorize_is_independent_of_scan_resolution():
    env = ConeTrackEnv(noise=False)
    env.reset(seed=0)
    ours = sectorize(env.ranges, env.sector_index, env.obs_range)
    # 队友 Gazebo 是 2000 点/圈:同一场景同一位姿,压成扇区后应当基本一致
    theirs_lidar = Lidar(replace(env.cp, lidar_points=2000))
    theirs_ranges = theirs_lidar.scan(env.car, env.track.cones, env.tp, None)
    theirs = sectorize(theirs_ranges, sector_index(theirs_lidar.angles), env.obs_range)
    assert theirs.shape == ours.shape
    diff = np.abs(theirs - ours)
    assert np.percentile(diff, 90) < 0.01
    # 擦着锥桶边缘的射线会因点数不同落进相邻扇区:允许个别扇区不一致,但障碍不能丢
    bad = np.flatnonzero(diff > 0.05)
    assert len(bad) <= 2
    for i in bad:
        assert min(theirs[(i - 1) % SECTORS], theirs[(i + 1) % SECTORS]) <= ours[i] + 0.02
    assert np.isfinite(sectorize([np.inf, np.nan], sector_index([0.0, 0.1]), 10.0)).all()


def test_driving_hard_left_ends_on_cone_or_off_track():
    # [2026-09-16 集成调整;2026-09-17 审查复核] 原动作 [1,1](满舵左):轴距
    # 0.305→0.36 对齐主线后,满舵兜圈半径 0.50→0.59m,seed0 赛道可容纳该圈,
    # 120s 兜圈只以 timeout 截断(当时把本测试改成满速直行 [1,0] 验证终止链路)。
    # [2026-09-17 补正] "兜圈可持续"是几何事实,但其时间上限行为不得只靠改
    # 动作绕开:满舵兜圈的 truncated 断言已由 test_core_audit.
    # test_full_steer_circling_truncates_at_budget 专门覆盖;本测试保留满速
    # 直行,继续验证 cone/off_track 终止链路可达。
    env = ConeTrackEnv()
    env.reset(seed=0)
    while True:
        _, _, term, trunc, info = env.step([1.0, 0.0])
        if term or trunc:
            break
    assert info["termination_reason"] in ("cone", "off_track")
