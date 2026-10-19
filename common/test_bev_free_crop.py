"""FREE 面积采样的旋转回归;稀疏目标继续按格心正向投点。"""
import math

import numpy as np
import pytest

from common.bev import (Bev, BevConfig, CAMERA_LAYER, CellType, LAYERS,
                        body_to_odom)
from common.bev_policy import bev_image, bev_images, NEAR_ROW0, NEAR_COL0


def _floor(bev, keep=None):
    # 整数索引生成源格中心,避免 arange 浮点累加恰落格边界。
    ix, iy = np.meshgrid(np.arange(-180, 180), np.arange(-180, 180))
    xy = np.column_stack(((ix.ravel() + 0.5) * bev.cfg.res,
                          (iy.ravel() + 0.5) * bev.cfg.res))
    if keep is not None:
        xy = xy[keep(xy)]
    bev.update_camera(np.full(len(xy), CellType.FREE), xy, (0, 0, 0), 0.0)
    return xy


@pytest.mark.parametrize('deg', [0, 5, 15, 30, 45, 90, -30, -135])
@pytest.mark.parametrize('res', [0.05, 0.1])
def test_dense_floor_has_no_rotation_holes(deg, res):
    bev = Bev(BevConfig(res=res, ahead=2.0, behind=1.0, side=1.5))
    _floor(bev)
    pose = (-0.013, 0.017, math.radians(deg))
    free = bev.crop_layers(pose, 0.0)[CAMERA_LAYER[CellType.FREE]]
    assert free.dtype == np.float32 and free.all()
    assert (bev.crop(pose, 0.0) == CellType.FREE).all()


def test_unknown_hole_and_outside_floor_remain_unknown_after_rotation():
    bev = Bev(BevConfig(ahead=2.0, behind=1.0, side=1.5))

    def keep(p):
        patch = (np.abs(p[:, 0]) < 1.0) & (np.abs(p[:, 1]) < 0.8)
        unseen = (np.abs(p[:, 0]) < 0.2) & (np.abs(p[:, 1]) < 0.2)
        return patch & ~unseen

    _floor(bev, keep)
    pose = (-0.013, 0.017, math.radians(30))
    row, col = np.indices(bev.cfg.shape)
    centers = np.column_stack((bev.cfg.ahead - (row.ravel()+0.5)*bev.cfg.res,
                               bev.cfg.side - (col.ravel()+0.5)*bev.cfg.res))
    world = body_to_odom(centers, pose)
    # 离边界至少两格,用连续几何判定而非复刻栅格索引算法。
    hole_core = (np.abs(world[:, 0]) < 0.1) & (np.abs(world[:, 1]) < 0.1)
    outside = (np.abs(world[:, 0]) > 1.1) | (np.abs(world[:, 1]) > 0.9)
    known = ((np.abs(world[:, 0]) < 0.9) & (np.abs(world[:, 1]) < 0.7)
             & ((np.abs(world[:, 0]) > 0.3) | (np.abs(world[:, 1]) > 0.3)))
    free = bev.crop_layers(pose, 0.0)[CAMERA_LAYER[CellType.FREE]].ravel()
    assert hole_core.any() and outside.any() and known.any()
    assert not free[hole_core | outside].any()
    assert free[known].all()
    kinds = bev.crop(pose, 0.0).ravel()
    assert (kinds[hole_core | outside] == CellType.UNKNOWN).all()
    assert (kinds[known] == CellType.FREE).all()


def test_free_inverse_sampling_obeys_own_expiry_and_prune():
    bev = Bev(BevConfig(ahead=2.0, behind=1.0, side=1.5, max_age=1.0))
    xy = _floor(bev)
    bev._add(xy, 0, 2.0)  # 其它层更新不能延长 FREE 寿命。
    pose = (-0.013, 0.017, math.pi/4)
    free_layer = CAMERA_LAYER[CellType.FREE]
    assert bev.crop_layers(pose, 1.0)[free_layer].all()  # max_age 边界包含在内
    assert bev.crop_layers(pose, None)[free_layer].all()
    assert not bev.crop_layers(pose, 2.0)[free_layer].any()
    assert CellType.FREE not in bev.crop(pose, 2.0)
    bev.prune(2.0)
    assert bev.crop_layers(pose, 2.0)[0].any()
    assert not bev.crop_layers(pose, None)[free_layer].any()


@pytest.mark.parametrize('with_other_layer', [False, True])
def test_empty_free_or_crop_far_from_map_is_all_unknown(with_other_layer):
    bev = Bev()
    if with_other_layer:
        bev._add(np.array([[0.025, 0.025]]), 0, 0.0)
    assert not bev.crop_layers((0, 0, 0.4), 0.0)[CAMERA_LAYER[CellType.FREE]].any()
    bev.update_camera([CellType.FREE], [[0.025, 0.025]], (0, 0, 0), 0.0)
    # 同时覆盖 searchsorted 的头端、尾端未命中。
    for pose in [(100, 100, 0.4), (-100, -100, -0.4)]:
        assert not bev.crop_layers(pose, 0.0).any()
        assert not bev.crop(pose, 0.0).any()


def test_sparse_layers_and_type_priority_keep_forward_projection():
    bev = Bev()
    _floor(bev)
    xy = np.array([[0.325, 0.225], [1.125, -0.425], [2.225, 1.125]])
    bev._add(xy, 0, 0.0)
    for kind in (CellType.TAPE, CellType.ZONE, CellType.TRAFFIC_LIGHT):
        bev.update_camera([kind]*len(xy), xy, (0, 0, 0), 0.0)
    bev.update_camera([CellType.CONE]*2, [xy[0], xy[0]], (0, 0, 0), 0.0)
    pose = (-0.013, 0.017, math.pi/4)
    # 冻结原非 FREE 层的正向投点,包括锥桶实心圆。
    expected = np.zeros((len(LAYERS),) + bev.cfg.shape, np.float32)
    idx, row, col = bev._window(pose, 0.0)
    for k, kind in enumerate(LAYERS):
        if kind != CellType.FREE:
            hit = bev.count[idx, k] > 0
            expected[k, row[hit], col[hit]] = 1
    rr, cc = bev._cone_cells(pose, 0.0)
    expected[CAMERA_LAYER[CellType.CONE], rr, cc] = 1
    layers = bev.crop_layers(pose, 0.0)
    np.testing.assert_array_equal(layers[:-1], expected[:-1])
    assert layers[-1].all()
    kinds = bev.crop(pose, 0.0)
    assert (kinds[rr, cc] == CellType.CONE).all()
    # 剩余同格冲突中 TRAFFIC_LIGHT 优先于雷达、胶带和区域,FREE 最低。
    traffic = expected[CAMERA_LAYER[CellType.TRAFFIC_LIGHT]].astype(bool)
    traffic[rr, cc] = False
    assert traffic.any() and (kinds[traffic] == CellType.TRAFFIC_LIGHT).all()
    main, near = bev_images(bev, pose, 0.0)
    assert main[5].all() and near[5].all()
    np.testing.assert_array_equal(main, bev_image(bev, pose, 0.0, free=True))
    np.testing.assert_array_equal(main[:5], bev_image(bev, pose, 0.0, free=False))
    np.testing.assert_array_equal(near.reshape(6, 32, 2, 32, 2).max(axis=(2, 4)),
                                 main[:, NEAR_ROW0//2:NEAR_ROW0//2+32,
                                      NEAR_COL0//2:NEAR_COL0//2+32])
