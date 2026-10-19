"""bev_v1 输入规格与网络:python -m pytest common/test_bev_policy.py"""
import math

import numpy as np
import pytest

from common.bev import Bev, CellType, body_to_odom
from common.bev_policy import (C, C2, H, NEAR_COL0, NEAR_ROW0, NEAR_SIZE, SCHEMA, SCHEMA_V25, W,
                               BevPolicy, bev_image, bev_near_image, forward, n_params, n_params_v25, scalars)
from common.road_policy import spec_of


def test_input_contract():
    assert (C, H, W) == (5, 80, 80)
    assert 500_000 < n_params() < 600_000


def test_image_channels_and_geometry():
    bev = Bev()
    pose = (1.0, 2.0, math.radians(20))
    cone_body = (3.0, -1.0)
    for k in range(2):                                            # 地标至少检到 2 次才画
        bev.update_camera([CellType.CONE], [cone_body], pose, 0.1 * k)
    bev.update_camera([CellType.TAPE], [(2.0, 1.5)], pose, 0.2)
    ranges = np.array([2.0])
    img = bev_image(bev, pose, 0.3, ranges_now=ranges, angles=np.array([0.0]))
    assert img.shape == (5, 80, 80) and set(np.unique(img)) <= {0.0, 1.0}
    rows, cols = np.nonzero(img[1])                               # 锥桶:车前 3 m、右 1 m
    assert (6.0 - (rows.mean() + 0.5) * 0.1) == pytest.approx(3.0, abs=0.1)
    assert (4.0 - (cols.mean() + 0.5) * 0.1) == pytest.approx(-1.0, abs=0.1)
    assert img[2].sum() == 1 and img[3].sum() == 0
    r4, _ = np.nonzero(img[4])                                    # 本帧雷达:雷达前 2 m
    assert (6.0 - (r4[0] + 0.5) * 0.1) == pytest.approx(2.0 + 0.145, abs=0.1)
    assert img[0].sum() == 0                                      # 没 update_lidar,记忆层是空的


def test_numpy_forward_matches_torch(tmp_path):
    torch = pytest.importorskip('torch')
    from training.bev_net import export, make_net
    torch.manual_seed(0)
    net = make_net()
    enc = spec_of('road_v2.5')
    p = tmp_path / 'm.npz'
    export(net, p, enc)
    pol = BevPolicy(p)
    assert pol.schema == SCHEMA + '_s3_v0.45-3' and pol.enc == enc
    rng = np.random.default_rng(0)
    for _ in range(5):
        img = (rng.random((5, 80, 80)) < 0.05).astype(np.float32)
        sc = scalars(rng.uniform(0, 2), 1.5, rng.uniform(-1, 1), enc)
        want = np.clip(net(torch.from_numpy(img)[None], torch.from_numpy(sc)[None]).detach().numpy()[0], -1, 1)
        np.testing.assert_allclose(pol(img, sc), want, atol=1e-5)


def test_train_init_loads_weights_and_checks_contract(tmp_path):
    pytest.importorskip('torch')
    import torch
    from training.bev_net import export, load_into, make_net, weights_of
    torch.manual_seed(1)
    a, b = make_net(), make_net()
    p = tmp_path / 'a.npz'
    export(a, p, spec_of('road_v2.5'))
    load_into(b, p)
    for k, v in weights_of(a).items():
        np.testing.assert_array_equal(v, weights_of(b)[k])

def test_v25_input_contract_and_near_crop_geometry():
    assert (C2, NEAR_SIZE, NEAR_ROW0, NEAR_COL0) == (6, 64, 68, 48)
    assert 680_000 < n_params_v25() < 710_000
    bev = Bev()
    pose = (0.0, 0.0, 0.0)
    # 前/后、左/右边界内外各一个地面标记,固定朝向下检查裁剪方向与范围。
    points = [(2.5, 0.0), (2.7, 0.0), (-0.5, 0.0), (-0.7, 0.0),
              (0.0, 1.5), (0.0, -1.5)]
    bev.update_camera([CellType.TAPE] * len(points), points, pose, 0.0)
    main = bev_image(bev, pose, 0.0, free=True)
    near = bev_near_image(bev, pose, 0.0)
    assert near.shape == (C2, 64, 64)
    assert set(np.unique(near)) <= {0.0, 1.0}
    assert near[2].sum() == 4                       # x=2.7 / x=-0.7 超出近场纵向范围
    rows, cols = np.nonzero(near[2])
    assert rows.min() < 32 < rows.max()             # 前方在上、后方在下
    assert cols.min() < 32 < cols.max()             # 左侧在左、右侧在右
    # 近场边界与 10 cm 主图网格对齐;每个主图格正好是对应 2×2 近场格的 max。
    blocks = near.reshape(C2, 32, 2, 32, 2).max(axis=(2, 4))
    np.testing.assert_array_equal(main[:, NEAR_ROW0 // 2:NEAR_ROW0 // 2 + 32,
                                       NEAR_COL0 // 2:NEAR_COL0 // 2 + 32], blocks)


def test_v25_numpy_forward_matches_torch_and_npz_dispatch(tmp_path):
    torch = pytest.importorskip('torch')
    from training.bev_net import export, make_net
    torch.manual_seed(48)
    net = make_net(C2, near=True)
    enc = spec_of('road_v2.5')
    path = tmp_path / 'v25.npz'
    export(net, path, enc, SCHEMA_V25)
    pol = BevPolicy(path)
    assert pol.schema == SCHEMA_V25 + '_s3_v0.45-3'
    assert pol.near and pol.free and pol.enc == enc
    rng = np.random.default_rng(48)
    for _ in range(3):
        img = (rng.random((C2, 80, 80)) < 0.05).astype(np.float32)
        near = (rng.random((C2, 64, 64)) < 0.05).astype(np.float32)
        sc = scalars(rng.uniform(0, 2), 1.5, rng.uniform(-1, 1), enc)
        with torch.no_grad():
            want = np.clip(net(torch.from_numpy(img)[None], torch.from_numpy(sc)[None],
                               torch.from_numpy(near)[None]).numpy()[0], -1, 1)
        np.testing.assert_allclose(pol(img, sc, near), want, atol=1e-5)


def test_v25_warm_start_from_v2_keeps_main_and_scalar_weights(tmp_path):
    pytest.importorskip('torch')
    import torch
    from common.road_policy import DEFAULT_ENC
    from training.bev_net import export, load_into, make_net
    torch.manual_seed(49)
    old, new = make_net(C2), make_net(C2, near=True)
    path = tmp_path / 'v2.npz'
    export(old, path, DEFAULT_ENC, 'bev_v2')
    load_into(new, path)
    near_width = 512
    assert torch.count_nonzero(new.f0.weight[:, 1600:1600 + near_width]) == 0
    for a, b in zip(old.convs, new.convs):
        torch.testing.assert_close(a.weight, b.weight, rtol=0, atol=0)
        torch.testing.assert_close(a.bias, b.bias, rtol=0, atol=0)
    img = torch.rand(2, C2, 80, 80)
    near = torch.rand(2, C2, 64, 64)
    sc = torch.rand(2, 3)
    torch.testing.assert_close(old(img, sc), new(img, sc, near), rtol=1e-5, atol=1e-6)

def test_near_crop_preserves_individual_5cm_cells_and_all_channels():
    from common.bev import CAMERA_LAYER
    from common.bev_policy import bev_images
    class KnownGrid(Bev):
        def crop_layers(self, pose, t):
            grid = np.zeros((6, 160, 160), np.float32)
            # 两个相邻 5 cm 胶带格,10 cm max 主图会把它们合并为一个。
            grid[CAMERA_LAYER[CellType.TAPE], 80, 80:82] = 1
            grid[0, 68, 48] = 1
            grid[CAMERA_LAYER[CellType.CONE], 131, 111] = 1
            grid[CAMERA_LAYER[CellType.ZONE], 100, 70] = 1
            grid[CAMERA_LAYER[CellType.FREE], 110, 90] = 1
            return grid
    bev = KnownGrid()
    main, near = bev_images(bev, (0, 0, 0), 0, [1.0], [0.0])
    assert near[2, 80 - NEAR_ROW0, 80 - NEAR_COL0] == 1
    assert near[2, 80 - NEAR_ROW0, 81 - NEAR_COL0] == 1
    assert near[2].sum() == 2 and main[2].sum() == 1
    assert near[0, 0, 0] == near[1, 63, 63] == 1
    assert near[3, 100 - NEAR_ROW0, 70 - NEAR_COL0] == 1
    assert near[5, 110 - NEAR_ROW0, 90 - NEAR_COL0] == 1
    assert near[4].sum() == 1


@pytest.mark.parametrize('yaw', [0.0, 0.7, -1.4])
def test_near_current_scan_uses_scan_pose_after_motion(yaw):
    from common.bev_policy import bev_images
    bev = Bev()
    scan_pose = (0.2, -0.3, yaw)
    ranges, angles = np.array([0.7, 1.0, 1.3]), np.array([-0.2, 0.0, 0.2])
    bev.update_lidar(ranges, angles, scan_pose, 0.0)
    pose = (0.25, -0.33, yaw + 0.05)
    main, near = bev_images(bev, pose, 0.05, ranges, angles, scan_pose)
    assert near[4].sum() > 0
    from common.bev import lidar_points
    points = lidar_points(ranges, angles, scan_pose, bev.params, bev.lidar_yaw, bev.cfg.max_range)
    dx, dy = points[:, 0] - pose[0], points[:, 1] - pose[1]
    fx = math.cos(pose[2]) * dx + math.sin(pose[2]) * dy
    fy = -math.sin(pose[2]) * dx + math.cos(pose[2]) * dy
    rows = np.floor((2.6 - fx) / 0.05).astype(int)
    cols = np.floor((1.6 - fy) / 0.05).astype(int)
    expected = np.zeros((64, 64), np.float32)
    expected[rows, cols] = 1
    np.testing.assert_array_equal(near[4], expected)
    # 记忆层先量化到 odom 格心,旋转重裁后可相差一格;当前扫描直接投影原始点。
    memory = np.argwhere(near[0])
    for cell in np.argwhere(expected):
        assert np.min(np.max(np.abs(memory - cell), axis=1)) <= 1
    np.testing.assert_array_equal(near.reshape(C2, 32, 2, 32, 2).max(axis=(2, 4)),
                                  main[:, 34:66, 24:56])


def test_v25_round_trip_load_into_preserves_all_weights(tmp_path):
    torch = pytest.importorskip('torch')
    from training.bev_net import export, load_into, make_net, weights_of
    torch.manual_seed(50)
    a, b = make_net(C2, near=True), make_net(C2, near=True)
    p = tmp_path / 'v25.npz'
    export(a, p, schema=SCHEMA_V25)
    load_into(b, p)
    for k, v in weights_of(a).items():
        np.testing.assert_array_equal(v, weights_of(b)[k])
    assert sum(v.numel() for v in a.parameters()) == n_params_v25() == 694_498
    with pytest.raises(ValueError):
        load_into(make_net(C2), p)
    with pytest.raises(ValueError):
        export(a, tmp_path / 'bad.npz', schema='bev_v2')
    with pytest.raises(ValueError):
        make_net(5, near=True)


def test_published_bev_models_bitwise_match_legacy_numpy_forward():
    """参考改动前的运算顺序;已发布旧 npz 不改 schema、不补权重。"""
    from pathlib import Path
    from common.bev_policy import _conv_s2
    paths = sorted((Path(__file__).resolve().parents[1] / 'models' / 'v3.5_bev').glob('*.npz'))
    assert paths
    rng = np.random.default_rng(48)
    for path in paths:
        pol = BevPolicy(path)
        assert not pol.near
        img = (rng.random((pol.channels, 80, 80)) < 0.2).astype(np.float32)
        sc = rng.uniform(-1, 1, 3).astype(np.float32)
        x = img.astype(np.float32)
        for k in range(4):
            x = np.maximum(_conv_s2(x, pol.params[f'c{k}w'], pol.params[f'c{k}b']), 0.0)
        h = np.concatenate([x.reshape(-1), sc.astype(np.float32)])
        for k in range(2):
            h = np.tanh(pol.params[f'f{k}w'] @ h + pol.params[f'f{k}b'])
        expected = np.clip(pol.params['f2w'] @ h + pol.params['f2b'], -1, 1).astype(np.float32)
        np.testing.assert_array_equal(pol(img, sc), expected)
