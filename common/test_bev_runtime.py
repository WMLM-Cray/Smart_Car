"""BevRuntime(实车 bev_v1 的大脑)与 road_car_node 接入的纯逻辑测试,不需要 ROS。"""
import math

import numpy as np
import pytest

from common.bev_policy import CONVS, HIDDEN, N_SCALAR, BevPolicy, H, W, bev_image
from common.bev_runtime import BevRuntime
from common.road_runtime import STOP

SCHEMA = 'bev_v1_s3_v0.45-3'


def _model(tmp_path, seed=0):
    """随机权重的 bev_v1 模型(和 bev_net.export 同键名),不依赖 torch。"""
    rng = np.random.default_rng(seed)
    p = {}
    for k, (ci, co, ks) in enumerate(CONVS):
        p[f'c{k}w'] = rng.normal(0, 0.1, (co, ci, ks, ks)).astype(np.float32)
        p[f'c{k}b'] = np.zeros(co, np.float32)
    dims = [CONVS[-1][1] * (H // 16) * (W // 16) + N_SCALAR, HIDDEN, HIDDEN, 2]
    for k in range(3):
        p[f'f{k}w'] = rng.normal(0, 0.05, (dims[k + 1], dims[k])).astype(np.float32)
        p[f'f{k}b'] = np.zeros(dims[k + 1], np.float32)
    path = tmp_path / 'bev.npz'
    np.savez(path, schema=np.array(SCHEMA), **p)
    return str(path)


def _replay(rt, steps=300, seed=5):
    """FastSim 一局按实车话题节奏喂给运行时:雷达 12 Hz、编码器/IMU 每个物理拍、控制 20 Hz。
    返回 (运行时, 环境, 每拍 (运行时图, 真位姿图))。"""
    from common.bev import Bev
    from simulation.fastsim.road import LEVELS, generate
    from simulation.fastsim.road_env import Plant, RoadEnv
    from training.road_teacher import Teacher, plan
    rng = np.random.default_rng(seed)
    road = generate(rng, next(l for l in LEVELS if l.name == 'L6f'))
    pl = Plant.sample(rng)
    env = RoadEnv(road, pl, seed, noise=False)
    teacher = Teacher(env, plan(road, 0, 0.0, rt.params.width, pl))
    car = env.car
    x0, y0, yaw0 = car.x, car.y, car.yaw
    truth = Bev(params=rt.params)

    def rel(x, y, yaw):                                   # 真位姿换到积分起点的坐标系
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        return (c * (x - x0) - s * (y - y0), s * (x - x0) + c * (y - y0), yaw - yaw0)

    scan = env._scan
    last = {}

    def hooked():
        scan()
        last['pose'] = rel(car.x, car.y, car.yaw)
        truth.update_lidar(env.ranges, env.angles, last['pose'], env.t)
        rt.on_scan(env.ranges, env.angles[0], env.angles[1] - env.angles[0], env.t)

    env._scan = hooked
    rt.on_odometry(car.v, 0.0)
    rt.on_imu(car.yaw_rate, 0.0)
    rt.set_enabled(True, 0.0)                               # 使能时 BEV 清空,先使能再登记第一帧
    hooked()
    pairs = []
    for _ in range(steps):
        rt.on_odometry(car.v, env.t)
        rt.on_imu(car.yaw_rate, env.t)
        rt.set_enabled(True, env.t)
        d = rt.decide(env.t)
        assert d is not None and d is not STOP
        pose = rel(car.x, car.y, car.yaw)
        pairs.append((bev_image(rt.bev, rt.odom.pose, env.t, rt.scan, rt.angles, rt.scan_pose),
                      bev_image(truth, pose, env.t, env.ranges, env.angles, last['pose'])))
        if env.step(teacher()) is not None:
            break
    return env, pose, pairs


def test_runtime_bev_matches_true_pose_bev_on_a_fastsim_episode(tmp_path):
    """编码器 + 陀螺积分出来的位姿与 BEV,和真位姿算的基本重合(FastSim 无噪声,差别只来自
    话题节奏下的积分误差)。"""
    rt = BevRuntime(BevPolicy(_model(tmp_path)), speed_cap=1.0)
    env, pose, pairs = _replay(rt)
    assert math.hypot(rt.odom.x - pose[0], rt.odom.y - pose[1]) < 0.1
    for img, ref in pairs[::10]:
        for a, b in ((img[0], ref[0]), (ref[0], img[0]), (img[4], ref[4])):
            assert _near(a, b) > 0.95                       # 积分误差在厘米级:差 1 格(10 cm)内
        assert img[1:4].sum() == 0                          # 车上还没有摄像头:三层为空


def _near(a, b):
    """a 里的格子有多少比例在 b 的某个格子 1 格以内。"""
    A, B = np.argwhere(a > 0), np.argwhere(b > 0)
    if not len(A):
        return 1.0
    if not len(B):
        return 0.0
    return float(np.mean(np.min(np.abs(A[:, None] - B[None]).max(2), 1) <= 1))


def test_runtime_decides_through_road_runtime_contract(tmp_path):
    rt = BevRuntime(BevPolicy(_model(tmp_path)), speed_cap=0.7)
    assert rt.decide(0.0) is None                           # 没使能:不发
    rt.on_odometry(0.0, 0.0)
    rt.on_imu(0.0, 0.0)
    ang = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    rt.on_scan(np.full(360, 2.0), ang[0], ang[1] - ang[0], 0.0)
    rt.set_enabled(True, 0.0)
    steer, v = rt.decide(0.05)
    assert math.isfinite(steer) and 0.0 < v <= 0.7
    assert rt.decide(1.0) is STOP                           # 心跳/传感器过期:回中位
    assert len(rt.bev.keys) == 0                            # 过期后 BEV 清空


def test_car_node_picks_bev_runtime_by_schema(tmp_path):
    from vehicle.road_car_node import RoadCarController, load_policy
    pol = load_policy(_model(tmp_path))
    assert isinstance(pol, BevPolicy)
    assert isinstance(RoadCarController(pol, 0.7).rt, BevRuntime)
    road = load_policy('models/road_v2.5/candidate_b036.npz')
    assert not isinstance(RoadCarController(road, 0.7).rt, BevRuntime)


@pytest.mark.parametrize('cap', [0.3, 3.5])
def test_car_node_rejects_cap_outside_bev_contract(tmp_path, cap):
    from vehicle.road_car_node import main
    with pytest.raises(SystemExit):
        main([_model(tmp_path), '--speed-cap', str(cap)])
