"""bev_v2:可行驶地面层(FREE)、假地面分割、6 通道输入与 bev_v1 热启动。"""
import math

import numpy as np
import pytest

from common.bev import Bev, CellType
from common.bev_policy import C, C2, BevPolicy, bev_image, channels_of


def _road(level='L6f', seed=5):
    from simulation.fastsim.road import LEVELS, generate
    return generate(np.random.default_rng(seed), next(l for l in LEVELS if l.name == level))


def _seg(road):
    from simulation.fastsim.camera import FakeFloorSeg, FloorSegConfig
    cfg = FloorSegConfig(edge_sigma=0.0, jitter=0.0, miss=0.0)
    return FakeFloorSeg(road, cfg=cfg, seed=1)


def _start_pose(road):
    return (*road.centre[0], float(road.heading[0]))


def _lateral(road, pose, xy_body):
    from common.bev import body_to_odom
    w = body_to_odom(xy_body, pose)
    i = np.argmin(((road.centre[None] - w[:, None]) ** 2).sum(-1), axis=1)
    return np.einsum('ij,ij->i', w - road.centre[i], road.normal[i]), i


def test_floor_skips_cone_bases():
    road = _road()
    pts = _seg(road).detect(_start_pose(road))
    assert len(pts) > 500
    from common.bev import body_to_odom
    w = body_to_odom(pts, _start_pose(road))
    dc = np.min(np.hypot(*(road.cones[None] - w[:, None]).transpose(2, 0, 1)), axis=1)
    assert np.all(dc > road.cone_radius)


def test_floor_past_cones_counts_but_not_past_walls():
    from simulation.fastsim.road import WALL
    road = _road('L6', seed=3)
    pose = _start_pose(road)
    pts = _seg(road).detect(pose)
    d, i = _lateral(road, pose, pts)
    out_left = d > road.half_l[i]
    out_right = d < -road.half_r[i]
    ml, mr = np.asarray(road.mat_left)[i], np.asarray(road.mat_right)[i]
    assert not np.any(out_left & (ml == WALL)) and not np.any(out_right & (mr == WALL))
    assert np.any((out_left & (ml != WALL)) | (out_right & (mr != WALL)))


def test_cone_casts_shadow_behind_it():
    """桶比相机高:桶后面那一扇地面相机看不到,桶旁边照样看得到。"""
    road = _road()
    seg = _seg(road)
    pose = _start_pose(road)
    from common.bev import body_to_odom
    from common.params import CarParams
    cam_x = CarParams().camera_x
    w = body_to_odom(seg.grid_body, pose)
    seen = {tuple(np.round(p, 3)) for p in seg.detect(pose)}
    cam = body_to_odom(np.array([[cam_x, 0.0]]), pose)[0]
    near = [c for c in road.cones if 0.6 < np.hypot(*(c - cam)) < 2.0 and seg.in_view(c[None], pose)[0]]
    assert near
    c = near[0]
    u = (c - cam) / np.hypot(*(c - cam))
    along = (w - cam) @ u
    perp = np.abs((w - cam) @ np.array([-u[1], u[0]]))
    dc = np.hypot(*(c - cam))
    behind = (along > dc + 0.3) & (perp < 0.03)
    assert behind.any()
    assert not any(tuple(np.round(p, 3)) in seen for p in seg.grid_body[behind])


def test_free_layer_is_lowest_priority_and_feeds_channel_5_only_for_v2():
    bev = Bev()
    pose = (0.0, 0.0, 0.0)
    bev.update_camera([CellType.FREE] * 2, [(2.0, 0.0), (2.0, 1.0)], pose, 0.0)
    bev.update_lidar(np.array([2.0 - bev.params.lidar_x]), np.array([0.0]), pose, 0.0)
    t = bev.types()
    assert set(t.tolist()) == {int(CellType.FREE), int(CellType.OBSTACLE)}
    v1 = bev_image(bev, pose, 0.0)
    v2 = bev_image(bev, pose, 0.0, free=True)
    assert v1.shape == (C, 80, 80) and v2.shape == (C2, 80, 80)
    assert np.array_equal(v1, v2[:C]) and v2[5].sum() == 2


def test_schema_picks_channel_count():
    assert channels_of('bev_v1_s3_v0.45-3') == 5 and channels_of('bev_v2_s3_v0.45-3') == 6
    with pytest.raises(ValueError):
        channels_of('road_v2.5')


def test_v2_warm_start_from_v1_keeps_v1_behaviour(tmp_path):
    """bev_v1 权重热启动成 bev_v2:新通道权重为 0,FREE 层再怎么填,输出都和 bev_v1 一样;
    numpy 前向和 torch 逐元素一致。"""
    torch = pytest.importorskip('torch')
    from common.road_policy import DEFAULT_ENC
    from training.bev_net import export, load_into, make_net
    torch.manual_seed(0)
    v1 = make_net()
    export(v1, tmp_path / 'v1.npz', DEFAULT_ENC)
    v2 = make_net(C2)
    load_into(v2, tmp_path / 'v1.npz')
    export(v2, tmp_path / 'v2.npz', DEFAULT_ENC, 'bev_v2')
    p1, p2 = BevPolicy(tmp_path / 'v1.npz'), BevPolicy(tmp_path / 'v2.npz')
    assert not p1.free and p2.free and p1.enc == p2.enc
    rng = np.random.default_rng(0)
    img = (rng.random((C2, 80, 80)) > 0.9).astype(np.float32)
    sc = rng.uniform(-1, 1, 3).astype(np.float32)
    assert np.allclose(p1(img[:C], sc), p2(img, sc), atol=1e-6)
    with torch.no_grad():
        ref = v2(torch.from_numpy(img[None]), torch.from_numpy(sc[None]))[0].clamp(-1, 1).numpy()
    assert np.allclose(p2(img, sc), ref, atol=1e-5)


def test_benv_free_episode_fills_channel_5():
    from simulation.fastsim.road_env import Plant
    from training.bev_env import BevEnv
    road = _road()
    rng = np.random.default_rng(2)
    benv = BevEnv(road, Plant.sample(rng), 2, noise=False, free=True)
    img, _ = benv.observation()
    assert img.shape == (C2, 80, 80) and img[5].sum() > 100
    assert BevEnv(road, Plant.sample(rng), 2, noise=False).observation()[0].shape == (C, 80, 80)


def test_edge_road_has_no_lidar_objects_and_floor_stops_at_edge():
    from simulation.fastsim.road import EDGE
    road = _road('LE4', seed=4)
    assert len(road.cones) == 0 and len(road.walls) == 0
    assert np.all(np.asarray(road.mat_left)[np.asarray(road.mat_left) != 0] == EDGE)
    pose = _start_pose(road)
    pts = _seg(road).detect(pose)
    assert len(pts) > 300
    d, i = _lateral(road, pose, pts)
    assert np.all(d <= road.half_l[i] + 1e-6) and np.all(d >= -road.half_r[i] - 1e-6)


def test_teacher_drives_edge_road_only_with_floor_seg():
    """EDGE 路沿雷达扫不到:老师的"看到过"只来自地面分割视场;开了地面分割能开到终点。"""
    from common.road_policy import spec_of
    from training.bev_env import BevEnv, SeenTeacher
    from training.road_check import sample_episode
    road, pl, start, P, _ = sample_episode(7, 'LE3', enc=spec_of('road_v2.5'), cap_max=1.8)
    b = BevEnv(road, pl, 7, start, noise=True, free=True)
    t = SeenTeacher(b, P, 1.47)
    assert t.seen.any()
    why = None
    while why is None:
        why = b.step(t())
    assert why == 'goal'
    assert not SeenTeacher(BevEnv(road, pl, 7, start, noise=True), P, 1.47).seen.any()
