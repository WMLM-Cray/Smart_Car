import importlib.util
import itertools
import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from fastsim.curriculum import STAGES, evaluate, passed
from fastsim.env import SECTOR_EDGES, SECTORS, ConeTrackEnv, sector_index, sectorize
from fastsim.interop import export_track, from_cmd_vel, to_cmd_vel
from fastsim.mapping import FREE, OCCUPIED
from fastsim.params import CarParams, RuleTrackParams, TrackParams
from fastsim.preview import pure_pursuit
from fastsim.pwm import (action_to_steer, pwm_to_speed, speed_to_pwm, steer_to_action,
                         steer_to_turn, turn_to_steer)
from fastsim.raycast import cast_cones, cast_walls
from fastsim.sensors import Lidar
from fastsim.track import make_track
from fastsim.track_rules import make_rule_track
from fastsim.vehicle import Car

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
    # 舵机曲线:实测表上的点原样回来,中间插值,两端钳住
    assert turn_to_steer(111, p) == pytest.approx(math.radians(11.9))
    assert turn_to_steer(21, p) == pytest.approx(math.radians(-21.7))
    assert turn_to_steer(160, p) == pytest.approx(math.radians(20.5))
    assert steer_to_turn(math.radians(11.9), p) == 111
    assert steer_to_turn(1.0, p) == 141


def test_action_maps_to_the_servo_value_the_car_sends():
    """动作 a 在车上发 turn_mid + a·turn_span,仿真里的转角必须是这个舵机值实测转多少。"""
    p = CarParams()
    for a in (-1.0, -0.5, -0.1, 0.0, 0.25, 0.5, 1.0):
        servo = p.turn_mid + a * p.turn_span
        assert action_to_steer(a, p) == pytest.approx(turn_to_steer(servo, p))
        assert steer_to_turn(action_to_steer(a, p), p) == round(servo)
        assert steer_to_action(action_to_steer(a, p), p) == pytest.approx(a, abs=1e-9)
    # 旧的线性假设满舵 0.55 rad(31.5°),实测左满舵只有 20.5°
    assert action_to_steer(1.0, p) == pytest.approx(math.radians(20.5))
    assert steer_to_action(0.55, p) == 1.0


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


# ---- 规则赛道(直道 + 45/90/135° 弯 + 挡板)----
def test_wall_ray_geometry():
    seg = [[[2.0, -1.0], [2.0, 1.0]]]                            # x=2 处一块横着的挡板
    assert cast_walls([0, 0, 0.21], [[1, 0, 0]], seg, 0.5)[0] == pytest.approx(2.0)
    assert np.isinf(cast_walls([0, 0, 0.8], [[1, 0, 0]], seg, 0.5)[0])     # 从挡板上方掠过
    assert np.isinf(cast_walls([0, 0, 0.21], [[0, 1, 0]], seg, 0.5)[0])    # 打不到
    assert np.isinf(cast_walls([0, 0, 0.21], [[-1, 0, 0]], seg, 0.5)[0])   # 背对着
    assert cast_walls([0, 0, 0.21], [[1, 0, 0]], seg, 0.5, max_range=1.0)[0] == np.inf


def test_rule_track_uses_only_legal_elements():
    p = RuleTrackParams()
    a = make_rule_track(np.random.default_rng(4))
    b = make_rule_track(np.random.default_rng(4))
    assert np.array_equal(a.centerline, b.centerline) and np.array_equal(a.boards, b.boards)
    # 闭环、尺寸在场地内、挡板数量按规则
    assert math.dist(a.centerline[0], a.centerline[-1]) < 0.1
    assert max(np.ptp(a.centerline[:, 0]), np.ptp(a.centerline[:, 1])) <= p.max_extent
    assert len(a.boards) == p.n_boards
    assert all(math.dist(*board) == pytest.approx(p.board_length) for board in a.boards)
    # 闭环转一整圈,且每个弯的转角只能是规则里的 45/90/135°
    closed = np.vstack([a.centerline, a.centerline[:1]])
    h = np.arctan2(*np.diff(closed, axis=0).T[::-1])
    turn = np.degrees((np.diff(np.concatenate([h, h[:1]])) + np.pi) % (2 * np.pi) - np.pi)
    assert turn.sum() == pytest.approx(360.0, abs=0.1)
    corners = [sum(v for _, v in grp)
               for straight, grp in itertools.groupby(enumerate(turn), key=lambda kv: abs(kv[1]) < 0.05)
               if not straight]
    legal = (*p.turn_angles, 180.0)                 # 180° 是两端掉头的大弯
    assert all(min(abs(abs(c) - a) for a in legal) < 2.0 for c in corners), corners
    assert len(corners) >= 10                                         # 弯要够多,不能是个圆角矩形
    # 长条形往返:主直道方向和它的反方向都要占掉可观的里程(端部掉头无论方角还是圆弯)
    heading = np.arctan2(*np.diff(closed, axis=0).T[::-1])
    out = np.mean(np.abs((heading - heading[0] + np.pi) % (2 * np.pi) - np.pi) < math.radians(20))
    back = np.mean(np.abs((heading - heading[0]) % (2 * np.pi) - np.pi) < math.radians(20))
    assert out > 0.15 and back > 0.15, (out, back)
    span = sorted((np.ptp(a.centerline[:, 0]), np.ptp(a.centerline[:, 1])))
    assert span[1] > 1.5 * span[0] and span[0] <= p.max_short_side    # 是长条形不是团
    # 主直道:最长的直段应当明显长于其它直段(规则图 1 的大直道)
    straights = [len(list(grp)) * 0.05
                 for straight, grp in itertools.groupby(turn, key=lambda v: abs(v) < 0.05) if straight]
    straights.sort()
    assert straights[-1] >= p.main_straight_min - 0.2 and straights[-1] > 2 * straights[-2]
    # 赛道宽:每个锥桶到中心线的距离在 1.25/2 与 1.5/2 之间
    d = np.hypot(*(a.cones[:, None, :] - a.centerline[None]).transpose(2, 0, 1)).min(axis=1)
    assert d.min() > p.narrow_width / 2 - 0.05 and d.max() < p.width / 2 + 0.05


def test_projection_stays_on_the_current_leg():
    """方正赛道折回来的两段挨得近,投影必须跟着上一帧走,不能跳到隔壁那段。"""
    track = make_rule_track(np.random.default_rng(2))
    pts = track.centerline
    cum = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    jumped = 0
    for i in range(0, len(pts) - 1, 7):                  # 沿中心线走一圈,每步约 0.35m
        here = pts[i] + 0.6 * np.array([-math.sin(0), math.cos(0)]) * 0   # 就在中心线上
        s, _, _ = track.project(here, near_s=cum[i])
        if abs(s - cum[i]) > 1.0:
            jumped += 1
    assert jumped == 0
    # 窗口确实在起作用:提示放到半圈外,投影就只会落在那一带
    far = track.project(pts[0], near_s=cum[-1] / 2)[0]
    assert abs(far - cum[-1] / 2) <= 4.1                # 窗口 4m,再加最多一个 5cm 采样段


def test_rule_track_boards_are_visible_and_block_the_car():
    env = ConeTrackEnv(track_params=RuleTrackParams(), noise=False)
    env.reset(seed=0)
    assert len(env.obstacles) == len(env.track.cones) + len(env.track.board_points)
    # 把车挪到某块挡板正前方 1m,雷达应当看到约 1m 处有东西
    board = env.track.boards[0]
    mid = board.mean(axis=0)
    normal = np.array([-(board[1] - board[0])[1], (board[1] - board[0])[0]])
    normal /= np.hypot(*normal)
    env.car.reset(*(mid + normal), math.atan2(*(-normal)[::-1]))
    ranges = env.lidar.scan(env.car, env.track, None)
    front = ranges[sector_index(env.lidar.angles) == SECTORS // 2]
    assert front.min() == pytest.approx(1.0 - env.cp.lidar_x, abs=0.05)
    assert not env.car.footprint_hits_circles(env.track.board_points, 0.14).any()   # 1m 外不算撞
    env.car.reset(*mid, 0.0)                                     # 车压在挡板上
    assert env.car.footprint_hits_circles(env.track.board_points, 0.14).any()


@pytest.mark.parametrize("seed", [90000, 90003])
def test_pure_pursuit_laps_rule_track_and_maps_it(seed):
    env = STAGES["s3"].make_env()
    env.reset(seed=seed)
    while True:
        _, _, term, trunc, info = env.step(pure_pursuit(env, throttle=0.5))
        if term or trunc:
            break
    assert info["termination_reason"] == "lap", info
    assert info["cone_hits"] == 0
    assert info["cone_coverage"] >= 0.9, info                     # 锥桶 + 挡板都要扫到


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
    assert env.action_to_command([1, 1]) == (env.v_cmd_fwd, action_to_steer(1.0, cp))
    assert env.action_to_command([-1, -1]) == (-env.v_cmd_rev, action_to_steer(-1.0, cp))
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


# ---- PPO 训练 ----
def test_stacked_policy_frame_order_matches_vecframestack():
    pytest.importorskip("stable_baselines3")
    from stable_baselines3.common.vec_env import DummyVecEnv, VecFrameStack

    from fastsim.train import FRAMES, StackedPolicy
    vec = VecFrameStack(DummyVecEnv([lambda: ConeTrackEnv(noise=False)]), FRAMES)
    env = ConeTrackEnv(noise=False)
    dim = env.observation_space.shape[0]
    recorder = StackedPolicy(model=None, obs_dim=dim)
    vec.seed(0)                                             # 两边同一条赛道
    stacked = vec.reset()
    obs, _ = env.reset(seed=0)
    for i in range(5):
        recorder.buf = np.roll(recorder.buf, -dim)          # 与 __call__ 里同一套写法
        recorder.buf[-dim:] = obs
        assert np.allclose(recorder.buf, stacked[0], atol=1e-6), f"第 {i} 帧顺序不一致"
        action = np.array([[0.5, 0.1]], np.float32)
        stacked, _, _, _ = vec.step(action)
        obs, _, _, _, _ = env.step(action[0])


def test_train_stage_produces_model_and_plots(tmp_path):
    pytest.importorskip("stable_baselines3")
    from fastsim.train import train_stage
    stage = replace(STAGES["s1"], budget=4.0)               # 只跑通流程,不指望学出东西
    result = train_stage(stage, steps=1024, n_envs=1, seed=0, out_dir=tmp_path,
                         init_model=None, seeds=[90000, 90001])
    assert result.stage == "s1" and len(result.episodes) == 2
    assert (tmp_path / "model.zip").exists() and (tmp_path / "curve.png").exists()
    saved = json.loads((tmp_path / "eval.json").read_text(encoding="utf-8"))
    assert saved["episodes_total"] == 2 and saved["passed"] is False


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
    path = Path(__file__).resolve().parents[2] / "3Dsim/hwcart_sim/src/track_generator/track_generator/generate.py"
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
    theirs_ranges = theirs_lidar.scan(env.car, env.track, None)
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
    env = ConeTrackEnv()
    env.reset(seed=0)
    while True:
        _, _, term, trunc, info = env.step([1, 1])
        if term or trunc:
            break
    assert info["termination_reason"] in ("cone", "off_track")
