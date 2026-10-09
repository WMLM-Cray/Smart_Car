"""common/bev.py 与 simulation/fastsim/camera.py 的测试:python -m pytest common/test_bev.py"""
import math

import numpy as np
import pytest

from common.bev import LAYERS, Bev, BevConfig, CellType, PoseIntegrator, body_to_odom, lidar_points
from common.params import CarParams

P = CarParams()
N = 1667
ANG = -math.pi + np.arange(N) * (2 * math.pi / N)


def scan_of(points, pose):
    """理想雷达:每个点落在最近的那条射线上,其余方向无回波。"""
    x, y, yaw = pose
    ox, oy = x + P.lidar_x * math.cos(yaw), y + P.lidar_x * math.sin(yaw)
    r = np.full(N, np.inf)
    for px, py in points:
        a = math.atan2(py - oy, px - ox) - yaw
        k = int(round(((a + math.pi) % (2 * math.pi)) / (2 * math.pi / N))) % N
        r[k] = min(r[k], math.hypot(px - ox, py - oy))
    return r


def test_contract_resolution_window_and_types_are_fixed():
    """训练出来的模型和这些数字绑定,改了就是换输入,要走 PR。"""
    cfg = BevConfig()
    assert cfg.res == 0.05 and cfg.shape == (160, 160)
    assert [int(t) for t in CellType] == list(range(7))
    assert [t.name for t in CellType] == ['UNKNOWN', 'FREE', 'OBSTACLE', 'CONE', 'TAPE', 'ZONE',
                                          'TRAFFIC_LIGHT']
    assert LAYERS[0] == CellType.OBSTACLE and len(LAYERS) == 5


def test_points_land_where_the_object_is_from_any_pose():
    cone = (3.0, 1.0)
    for pose in [(0.0, 0.0, 0.0), (1.0, -0.5, 0.4), (-1.0, 2.0, -1.2)]:
        pts = lidar_points(scan_of([cone], pose), ANG, pose)
        assert len(pts) == 1
        assert np.hypot(*(pts[0] - cone)) < 0.02     # 角分辨率 0.2°,3 m 处 ~1 cm


def test_lidar_offset_comes_from_car_params():
    """雷达在后轴前 CarParams.lidar_x;位姿是后轴。错一个 lidar_x 就会整张图平移。"""
    pts = lidar_points([1.0], [0.0], (0.0, 0.0, 0.0))     # 雷达正前方 1 m
    assert pts[0] == pytest.approx([P.lidar_x + 1.0, 0.0], abs=1e-9)


def test_accumulates_and_merges_repeated_hits():
    bev = Bev()
    cones = [(3.0, 1.0), (3.0, -1.0), (5.0, 0.8)]
    for k in range(5):
        pose = (0.2 * k, 0.0, 0.0)
        bev.update_lidar(scan_of(cones, pose), ANG, pose, t=0.1 * k)
    assert len(bev) <= 3 * 2                           # 同一个桶落在同一个(或相邻)格子
    for c in cones:
        assert np.min(np.hypot(*(bev.points() - c).T)) < 0.05


def test_crop_is_heading_up_and_left_is_left():
    cfg = BevConfig()
    bev = Bev(cfg)
    pose = (10.0, 5.0, math.radians(90))               # 车头朝 +y
    ahead_left = (10.0 - 1.0, 5.0 + 3.0)               # 车前 3 m、车左 1 m
    bev.update_lidar(scan_of([ahead_left], pose), ANG, pose, 0.0)
    img = bev.crop(pose)
    rows, cols = np.nonzero(img)
    assert len(rows) == 1 and img[rows[0], cols[0]] == CellType.OBSTACLE
    assert (cfg.ahead - (rows[0] + 0.5) * cfg.res) == pytest.approx(3.0, abs=cfg.res)
    assert (cfg.side - (cols[0] + 0.5) * cfg.res) == pytest.approx(1.0, abs=cfg.res)


def test_camera_detections_register_with_their_class_and_priority():
    bev = Bev()
    pose = (2.0, 1.0, math.radians(30))
    cone_body, tape_body = (3.0, 0.5), (2.0, -1.0)
    cone_odom = body_to_odom([cone_body], pose)[0]
    bev.update_lidar(scan_of([cone_odom], pose), ANG, pose, 0.0)          # 雷达先看到:障碍
    assert set(bev.types()) == {CellType.OBSTACLE}
    bev.update_camera([CellType.CONE, CellType.TAPE], [cone_body, tape_body], pose, 0.1)
    assert (bev.crop(pose) == CellType.CONE).sum() == 0                     # 只检到 1 次:先不画(防误检)
    bev.update_camera([CellType.CONE], [(cone_body[0] + 0.03, cone_body[1])], pose, 0.2)
    img = bev.crop(pose)
    assert len(bev.cones()) == 1                                            # 两次检测并成一个地标
    assert np.hypot(*(bev.cones()[0] - body_to_odom([(cone_body[0] + 0.015, cone_body[1])], pose)[0])) < 1e-9
    assert (img == CellType.CONE).sum() > 20                                # 画成一个实心圆(半径 0.14)
    assert (img == CellType.TAPE).sum() == 1                                # 雷达看不见的胶带
    layers = bev.crop_layers(pose)
    assert layers.shape == (5, 160, 160) and layers[0].sum() >= 1           # 雷达层仍在
    with pytest.raises(ValueError):
        bev.update_camera([CellType.OBSTACLE], [cone_body], pose, 0.2)      # 摄像头不登记"障碍"


def test_old_cells_are_forgotten():
    bev = Bev(BevConfig(max_age=2.0))
    pose = (0.0, 0.0, 0.0)
    bev.update_lidar(scan_of([(3.0, 0.0)], pose), ANG, pose, t=0.0)
    bev.update_lidar(scan_of([(4.0, 1.0)], pose), ANG, pose, t=1.5)
    assert (bev.crop(pose, t=3.0) != 0).sum() == 1     # 0 s 的那个已超龄,不画
    bev.prune(t=3.0)
    assert len(bev) == 1


def test_seen_cones_stay_after_they_leave_the_field_of_view():
    """BEV 的意义:转过头以后,刚才看到的桶还在。"""
    bev = Bev()
    cone = (2.0, 2.0)
    p0, p1 = (0.0, 0.0, math.radians(45)), (0.0, 0.0, math.radians(-135))
    bev.update_lidar(scan_of([cone], p0), ANG, p0, 0.0)
    bev.update_lidar(np.full(N, np.inf), ANG, p1, 0.1)
    assert np.min(np.hypot(*(bev.points() - cone).T)) < 0.05


def _episode(level='L6f', seed=5):
    from simulation.fastsim.road import LEVELS, generate
    from simulation.fastsim.road_env import Plant, RoadEnv
    from training.road_teacher import Teacher, plan
    rng = np.random.default_rng(seed)
    road = generate(rng, next(l for l in LEVELS if l.name == level))
    pl = Plant.sample(rng)
    env = RoadEnv(road, pl, seed, noise=False)
    return road, pl, env, Teacher(env, plan(road, 0, 0.0, P.width, pl))


def test_fastsim_episode_every_hit_cone_is_in_the_bev():
    """FastSim 一局:真位姿累积。雷达打中过的每个锥桶,BEV 里离它一个桶半径内都有点。"""
    road, pl, env, teacher = _episode()
    bev = Bev(BevConfig(max_age=1e9), lidar_yaw=pl.lidar_yaw)
    hit = np.zeros(len(road.cones), bool)
    for _ in range(200):
        pose = (env.car.x, env.car.y, env.car.yaw)
        bev.update_lidar(env.ranges, env.angles, pose, env.t)
        pts = lidar_points(env.ranges, env.angles, pose, lidar_yaw=pl.lidar_yaw)
        if len(pts):
            d = np.hypot(*(road.cones[:, None] - pts[None]).transpose(2, 0, 1))
            hit |= d.min(1) < road.cone_radius + 0.05
        if env.step(teacher()) is not None:
            break
    assert hit.sum() > 10
    bp = bev.points()
    for c in road.cones[hit]:
        assert np.min(np.hypot(*(bp - c).T)) < road.cone_radius + 0.05


def test_fake_yolo_sees_cones_in_front_within_range_only():
    from simulation.fastsim.camera import FakeYolo, YoloSimConfig
    from simulation.fastsim.road import CONE, build
    road = build([dict(kind='straight', length=12.0, width=1.5, mat=(CONE, CONE))])
    cam = FakeYolo(road, cfg=YoloSimConfig(miss=0.0, sigma0=0.0, sigma_per_m=0.0, false_rate=0.0))
    kinds, xy = cam.detect((0.0, 0.0, 0.0))
    assert len(kinds) and set(kinds) == {int(CellType.CONE)}
    cx = xy[:, 0] - P.camera_x
    assert (cx > 0).all() and (np.hypot(cx, xy[:, 1]) <= P.yolo_range + 1e-9).all()
    assert (np.abs(np.arctan2(xy[:, 1], cx)) <= P.camera_hfov / 2 + 1e-9).all()
    for side in (1, -1):                               # 每侧视场内最近的那个桶一定看得到、位置准
        c = road.cones[(np.sign(road.cones[:, 1]) == side) & (road.cones[:, 0] > P.camera_x)]
        c = c[np.abs(np.arctan2(c[:, 1], c[:, 0] - P.camera_x)) <= P.camera_hfov / 2]
        nearest = c[np.argmin(c[:, 0])]
        assert np.min(np.hypot(*(xy - nearest).T)) < 1e-9


def test_fake_yolo_far_cones_in_a_row_are_hidden_by_nearer_ones():
    """一排桶斜着看,远处的会被近处的挡住;正对着的单个桶不挡。"""
    from simulation.fastsim.camera import FakeYolo
    from simulation.fastsim.road import CONE, build
    road = build([dict(kind='straight', length=12.0, width=1.5, mat=(CONE, CONE))])
    cam = FakeYolo(road)
    row = np.array([[P.camera_x + d, 0.75] for d in (1.0, 2.0, 3.0, 4.0, 4.5)])
    vis = cam.visible((0.0, 0.0, 0.0), row, road.cone_radius)
    assert vis[0] == 1.0 and vis[-1] < 0.5            # 最近的全露,4.5 m 那个被前面的挡掉大半
    assert np.all(np.diff(vis) <= 1e-9)                # 越远露得越少
    alone = cam.visible((0.0, 0.0, 0.0), row[:1], road.cone_radius)
    assert alone[0] == 1.0


def test_fake_yolo_reports_tape_on_bare_boundaries():
    """边界材质 NONE(只有胶带,例如黄 V)只有摄像头看得到。"""
    from simulation.fastsim.camera import FakeYolo, YoloSimConfig
    from simulation.fastsim.road import CONE, NONE, build
    road = build([dict(kind='straight', length=12.0, width=1.5, mat=(NONE, CONE))])
    cam = FakeYolo(road, cfg=YoloSimConfig(miss=0.0, sigma0=0.0, sigma_per_m=0.0, false_rate=0.0))
    kinds, xy = cam.detect((0.0, 0.0, 0.0))
    tape = xy[kinds == int(CellType.TAPE)]
    assert len(tape) > 10 and np.all(tape[:, 1] > 0.5)        # 都在左边线附近


def test_fastsim_episode_camera_cones_line_up_with_lidar():
    """同一局里摄像头登记的锥桶落在雷达看到的桶上(位置误差 ~ 几厘米)。"""
    from simulation.fastsim.camera import FakeYolo
    road, pl, env, teacher = _episode()
    cam = FakeYolo(road, seed=1)
    bev = Bev(BevConfig(max_age=1e9), lidar_yaw=pl.lidar_yaw)
    for _ in range(120):
        pose = (env.car.x, env.car.y, env.car.yaw)
        bev.update_lidar(env.ranges, env.angles, pose, env.t)
        bev.update_camera(*cam.detect(pose), pose, env.t)
        if env.step(teacher()) is not None:
            break
    cones = bev.cones()
    assert len(cones) >= 8                             # 视场 70°,120 拍里看到的桶
    d = np.hypot(*(cones[:, None] - road.cones[None]).transpose(2, 0, 1)).min(1)
    # 多次检测平均之后,地标基本落在真桶上;一个真桶只对应一个地标
    assert np.median(d) < 0.08 and np.mean(d > 0.3) < 0.1
    owner = np.hypot(*(cones[:, None] - road.cones[None]).transpose(2, 0, 1)).argmin(1)
    assert len(set(owner[d < 0.3])) == (d < 0.3).sum()


def test_pose_integrator_straight_and_circle():
    p = PoseIntegrator()
    for _ in range(100):
        p.step(1.0, 0.0, 0.01)
    assert p.pose == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)
    p = PoseIntegrator()
    v, r = 1.0, 2.0                                    # 半径 2 m 走半圈
    n = 2000
    dt = math.pi * r / v / n
    for _ in range(n):
        p.step(v, v / r, dt)
    assert p.x == pytest.approx(0.0, abs=1e-3) and p.y == pytest.approx(2 * r, abs=1e-3)
