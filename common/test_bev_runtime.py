"""BevRuntime(实车 bev_v1 的大脑)与 road_car_node 接入的纯逻辑测试,不需要 ROS。"""
import math

import numpy as np
import pytest

from common.bev_policy import CONVS, HIDDEN, N_SCALAR, BevPolicy, H, W, bev_image
from common.bev_runtime import BevRuntime
from common.road_runtime import STOP

SCHEMA = 'bev_v1_s3_v0.45-3'


def _model(tmp_path, seed=0, schema=SCHEMA):
    """随机权重的 bev_v1 / bev_v2 模型(和 bev_net.export 同键名),不依赖 torch。"""
    from common.bev_policy import channels_of
    rng = np.random.default_rng(seed)
    p = {}
    for k, (ci, co, ks) in enumerate(CONVS):
        ci = channels_of(schema) if k == 0 else ci
        p[f'c{k}w'] = rng.normal(0, 0.1, (co, ci, ks, ks)).astype(np.float32)
        p[f'c{k}b'] = np.zeros(co, np.float32)
    dims = [CONVS[-1][1] * (H // 16) * (W // 16) + N_SCALAR, HIDDEN, HIDDEN, 2]
    for k in range(3):
        p[f'f{k}w'] = rng.normal(0, 0.05, (dims[k + 1], dims[k])).astype(np.float32)
        p[f'f{k}b'] = np.zeros(dims[k + 1], np.float32)
    path = tmp_path / f'{schema}.npz'
    np.savez(path, schema=np.array(schema), **p)
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


def test_v2_registers_floor_at_capture_pose(tmp_path):
    """bev_v2:地面点按拍摄时刻的位姿登记(车已往前开了 0.5 m,点仍落在原地),图里第 5 通道有东西。"""
    from vehicle.road_car_node import RoadCarController, load_policy
    pol = load_policy(_model(tmp_path, schema='bev_v2_s3_v0.45-3'))
    assert isinstance(pol, BevPolicy) and pol.free
    rt = RoadCarController(pol, 0.7).rt
    assert isinstance(rt, BevRuntime)
    for k in range(51):                                     # 1 m/s 直行 0.5 s
        t = k * 0.01
        rt.on_odometry(1.0, t)
        rt.on_imu(0.0, t)
    rt.on_free(np.array([[2.0, 0.0]]), stamp=0.0, now=0.5)
    img = bev_image(rt.bev, rt.odom.pose, 0.5, free=True)
    assert img[5].sum() >= 1
    row, col = np.argwhere(img[5] > 0)[0]
    ahead = (60 - row - 0.5) * 0.1                          # 后轴在第 60 行,每格 0.1 m
    assert abs(ahead - 1.5) < 0.11                          # 拍摄时在前方 2 m,车又开了 0.5 m


def test_v2_stops_without_ground_seg(tmp_path):
    """bev_v2:地面分割断了(FREE 层只会变空、模型照开)要当传感器过期回中位;v1 不看这个。"""
    ang = np.linspace(-math.pi, math.pi, 360, endpoint=False)

    def feed(rt, t):
        rt.on_odometry(0.0, t)
        rt.on_imu(0.0, t)
        rt.on_scan(np.full(360, 2.0), ang[0], ang[1] - ang[0], t)
        rt.set_enabled(True, t)

    v2 = BevRuntime(BevPolicy(_model(tmp_path, schema='bev_v2_s3_v0.45-3')), speed_cap=0.7)
    feed(v2, 0.0)
    assert v2.decide(0.05) is STOP and v2.stale_sensors(0.05) == ['free']
    v2.on_free(np.zeros((0, 2)), stamp=0.0, now=0.1)        # 一帧没认出地面也算活着
    feed(v2, 0.1)
    assert v2.decide(0.15) is not STOP
    feed(v2, 0.6)
    assert v2.decide(0.65) is not STOP                      # 0.55 s 没新分割:3 Hz 下正常的间隔
    feed(v2, 1.15)
    assert v2.decide(1.2) is STOP                           # 1.1 s 没新分割
    v1 = BevRuntime(BevPolicy(_model(tmp_path)), speed_cap=0.7)
    feed(v1, 0.0)
    assert v1.decide(0.05) is not STOP


@pytest.mark.parametrize('stamp', [1.0, 9.5, 10.1, float('nan')])
def test_old_duplicate_future_or_invalid_floor_cannot_refresh_runtime(tmp_path, stamp):
    rt = BevRuntime(BevPolicy(_model(tmp_path, schema='bev_v2_s3_v0.45-3')), speed_cap=0.7)
    xy = np.array([[2.0, 0.0]])
    assert rt.on_free(xy, stamp=9.5, now=9.6)
    before = rt.bev.count.copy()
    assert not rt.on_free(xy, stamp=stamp, now=10.0)
    assert rt.free_t == 9.5 and np.array_equal(before, rt.bev.count)
    assert 'free' in rt.stale_sensors(10.6)
    assert not rt.on_free(xy, stamp=9.5, now=10.6)
    assert 'free' in rt.stale_sensors(10.6)
    assert rt.on_free(np.empty((0, 2)), stamp=10.5, now=10.6)
    assert 'free' not in rt.stale_sensors(10.6)


def test_delayed_floor_expires_from_capture_time(tmp_path):
    rt = BevRuntime(BevPolicy(_model(tmp_path, schema='bev_v2_s3_v0.45-3')), speed_cap=0.7)
    assert not rt.on_free(np.array([[2.0, 0.0]]), stamp=1.0, now=10.0)
    assert rt.free_t is None and not len(rt.bev)
    assert rt.on_free(np.array([[2.0, 0.0]]), stamp=10.0, now=10.8)
    assert 'free' in rt.stale_sensors(11.1)
    assert not rt.bev.crop_layers((0, 0, 0), 30.1)[5].any()


def test_car_node_logs_global_map_in_segments(tmp_path):
    """bev 开车记全局地图:放行一次一段;松手不到 SEG_GAP 接着记,超过就存盘收尾;没放行时不记。"""
    from vehicle.bev_logbook import Logbook
    from vehicle.road_car_node import SEG_GAP, RoadCarController, load_policy
    book = Logbook(str(tmp_path / 'log'))
    ctl = RoadCarController(load_policy(_model(tmp_path)), 0.7, book=book)
    ang = np.linspace(-math.pi, math.pi, 360, endpoint=False)

    class Scan:
        angle_min, angle_increment, range_min, range_max = ang[0], ang[1] - ang[0], 0.15, 25.0
        ranges = list(np.full(360, 2.0))

    def run(t0, t1, enabled):
        t = t0
        while t < t1:
            ctl.on_encoder(0.0, t)
            ctl.on_imu(0.0, t)
            ctl.on_scan(Scan, t)
            if enabled:
                ctl.on_enable(True, t)
            ctl.tick(t)
            t += 0.05
        return t

    run(0.0, 1.0, False)
    assert not book.path and len(book.lidar.ij()[0]) == 0           # 没放行:不记
    t = run(1.0, 3.0, True)
    assert book.seg == 1 and book.path and len(book.lidar.ij(1)[0])
    t = run(t, t + 2.0, False)                                      # 短暂松手
    t = run(t, t + 1.0, True)
    assert book.seg == 1 and book.active
    t = run(t, t + SEG_GAP + 1.0, False)                            # 松手够久:收尾存盘
    assert not book.active and (tmp_path / 'log' / 'seg_01' / 'seg.npz').is_file()
    run(t, t + 1.0, True)
    assert book.seg == 2 and book.active


def test_car_node_records_frames_for_replay(tmp_path):
    """开车时逐帧记 BEV 输入 + 决策 + 摄像头调试图,段收尾时存 frames.npz 和 frames/cam_NNNN.jpg。"""
    from vehicle.bev_logbook import FRAME_DT, Logbook
    from vehicle.road_car_node import RoadCarController, load_policy
    book = Logbook(str(tmp_path / 'log'))
    ctl = RoadCarController(load_policy(_model(tmp_path)), 0.7, book=book)
    ang = np.linspace(-math.pi, math.pi, 360, endpoint=False)

    class Scan:
        angle_min, angle_increment, range_min, range_max = ang[0], ang[1] - ang[0], 0.15, 25.0
        ranges = list(np.full(360, 2.0))

    t = 0.0
    while t < 3.0:
        ctl.on_encoder(0.0, t)
        ctl.on_imu(0.0, t)
        ctl.on_scan(Scan, t)
        ctl.on_enable(True, t)
        if abs(t - 1.0) < 1e-6:
            ctl.cam_jpg = b'jpg-1'
        ctl.tick(t)
        t = round(t + 0.05, 6)
    book.pause()
    z = np.load(tmp_path / 'log' / 'seg_01' / 'frames.npz')
    n = len(z['num'])
    assert abs(n - 3.0 / FRAME_DT) <= 2 and z['bev'].shape[1:] == (5, 80, 80)
    assert z['num'][-1, 5] > 0                                      # 记的是模型的目标速度
    k = int(z['cam'][-1])
    assert k >= 0 and (z['cam'][:k] == -1).all()
    assert (tmp_path / 'log' / 'seg_01' / 'frames' / f'cam_{k:04d}.jpg').read_bytes() == b'jpg-1'
