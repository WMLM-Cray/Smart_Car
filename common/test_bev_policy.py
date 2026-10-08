"""bev_v1 输入规格与网络:python -m pytest common/test_bev_policy.py"""
import math

import numpy as np
import pytest

from common.bev import Bev, CellType, body_to_odom
from common.bev_policy import C, H, SCHEMA, W, BevPolicy, bev_image, forward, n_params, scalars
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
