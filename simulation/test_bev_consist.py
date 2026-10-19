"""v4 训练开关:默认关时采集逐位不变;--consist 的第二张图是同一帧另一种亚格偏移;lag_max 跑得通。"""
import numpy as np

from common.road_policy import V_LO, Encoding
from training.bev_train import run_episode

ENC = Encoding(3.0, V_LO, 3.0)
KW = dict(collect=True, enc=ENC, free=True, a_lat=1.47, cap_max=2.5)


def test_defaults_unchanged_and_pair_differs():
    a = run_episode(3, 'LW3', **KW)
    b = run_episode(3, 'LW3', **KW)
    assert np.array_equal(a[0], b[0]) and np.array_equal(a[2], b[2]) and a[4] is None
    c = run_episode(3, 'LW3', aug=(0.05, 0.017), pair=True, **KW)
    assert c[4].shape == c[0].shape
    assert (c[4] != c[0]).any(axis=1).mean() > 0.5          # 大多数帧两张图量化相位不同


def test_lag_max_runs():
    rows = [run_episode(100 + s, 'LW3', enc=ENC, a_lat=1.47, cap_max=2.5, lag_max=3) for s in range(3)]
    assert all(r is None or r['reason'] in ('goal', 'collision', 'off_track', 'timeout') for r in rows)


def test_no_yolo_keeps_cones_on_the_road_but_empties_the_cone_layer():
    from training.bev_env import BevEnv
    from training.road_check import sample_episode
    road, pl, start, _, _ = sample_episode(11, 'L5', enc=ENC, cap_max=2.5, a_lat=1.47)
    assert len(road.cones)                                  # 路上有锥桶
    on = BevEnv(road, pl, 11, start, noise=False, free=True)
    off = BevEnv(road, pl, 11, start, noise=False, free=True, yolo=False)
    for _ in range(20):
        on.step((0.0, 0.0)); off.step((0.0, 0.0))
    img_on, img_off = on.observation()[0], off.observation()[0]
    assert img_on[1].any() and not img_off[1].any()         # 锥桶层:有 YOLO 才有
    assert img_off[0].any() or img_off[4].any()             # 雷达照样扫到(锥桶 + 其他)


def test_guard_stops_like_the_car():
    """直走廊尽头一堵墙:离墙远时照常走,保险杠前不到 0.25 m 就判 guard(和车上截停同一规则)。"""
    from simulation.fastsim.road import WALL, build
    from simulation.fastsim.road_env import Plant
    from training.bev_env import BevEnv
    road = build([dict(kind='straight', length=3.0, width=1.2, mat=(WALL, WALL))])
    import numpy as np
    from simulation.fastsim.road import _resample
    p, _ = _resample(np.array([road.left[-1], road.right[-1]]), 0.2)
    road.walls = np.concatenate([road.walls, np.stack([p[:-1], p[1:]], axis=1)])      # 尽头封墙
    pl = Plant.sample(np.random.default_rng(0))
    far = BevEnv(road, pl, 0, (0, 0.0, 0.0), noise=False, guard=True)
    assert far.step((0.0, 0.5)) is None
    i = len(road.centre) - 1 - int(0.55 / 0.05)                 # 后轴离墙 0.55 m:车头离墙 0.135 m
    near = BevEnv(road, pl, 0, (i, 0.0, 0.0), noise=False, guard=True)
    assert near.step((0.0, 0.5)) == 'guard'
    assert BevEnv(road, pl, 0, (i, 0.0, 0.0), noise=False).step((0.0, 0.5)) != 'guard'   # 默认不管


def test_floor_seg_lut_mode_is_sparse_far_and_sometimes_short():
    """车上的投影方式:远处像素行稀(条纹),一部分帧只到 2 m;默认还是老的密网。"""
    from dataclasses import replace
    from simulation.fastsim.camera import FakeFloorSeg, FloorSegConfig
    from simulation.fastsim.road import WALL, build
    road = build([dict(kind='straight', length=10.0, width=3.0, mat=(WALL, WALL))])
    pose = (1.0, 0.0, 0.0)
    clean = dict(miss=0.0, edge_sigma=0.0, jitter=0.0)
    dense = FakeFloorSeg(road, cfg=replace(FloorSegConfig(), **clean)).detect(pose)
    lut = FakeFloorSeg(road, cfg=replace(FloorSegConfig(), lut=(72, 96), **clean), seed=1)
    far = lambda xy: int(((xy[:, 0] > 2.6) & (xy[:, 0] < 3.2) & (abs(xy[:, 1]) < 0.5)).sum())
    frames = [lut.detect(pose) for _ in range(30)]
    assert max(far(f) for f in frames) < far(dense) / 3          # 远处稀
    assert min(far(f) for f in frames) == 0                      # 有的帧只到 2 m

def test_v25_replay_bitpacking_and_pair_alignment_through_wraparound():
    from training.bev_train import Replay
    from common.bev_policy import C2, NEAR_SIZE
    replay = Replay(3, C2, pair=True, near=True)
    rng = np.random.default_rng(48)
    imgs = rng.integers(0, 2, (5, C2, 80, 80), dtype=np.uint8)
    near = rng.integers(0, 2, (5, C2, NEAR_SIZE, NEAR_SIZE), dtype=np.uint8)
    imgs2, near2 = 1 - imgs, 1 - near
    sc = np.column_stack([np.arange(5), np.zeros((5, 2))]).astype(np.float32)
    y = np.column_stack([np.arange(5), -np.arange(5)]).astype(np.float32)
    pack = lambda a: np.packbits(a.reshape(len(a), -1), axis=1)
    replay.add(pack(imgs), sc, y, pack(imgs2), pack(near), pack(near2))
    assert replay.size == 3 and replay.ptr == 2
    assert replay.img.shape == (3, 4800) and replay.near_img.shape == (3, 3072)
    image, scalar, label, image2, near_image, near_image2 = replay.batch(rng, 16)
    assert set(scalar[:, 0]) <= {2, 3, 4}
    for k, row in enumerate(scalar):
        j = int(row[0])
        np.testing.assert_array_equal(image[k], imgs[j])
        np.testing.assert_array_equal(image2[k], imgs2[j])
        np.testing.assert_array_equal(near_image[k], near[j])
        np.testing.assert_array_equal(near_image2[k], near2[j])
        np.testing.assert_array_equal(label[k], y[j])


def test_v25_collection_pairs_main_and_near_crops_share_augmentation():
    from common.bev_policy import C2, NEAR_SIZE
    sample = run_episode(3, 'LW3', aug=(0.05, 0.017), pair=True, near=True, **KW)
    img, sc, y, row, img2, near, near2 = sample
    assert len(sc) == len(y) == row['steps']
    for m, n in ((img, near), (img2, near2)):
        main = np.unpackbits(m, axis=1).reshape(-1, C2, 80, 80)
        close = np.unpackbits(n, axis=1).reshape(-1, C2, NEAR_SIZE, NEAR_SIZE)
        pooled = close.reshape(-1, C2, 32, 2, 32, 2).max(axis=(3, 5))
        np.testing.assert_array_equal(pooled, main[:, :, 34:66, 24:56])
    assert (near != near2).any()


def test_legacy_replay_interface_and_storage_remain_unchanged():
    from training.bev_train import Replay
    for channels, frame_bytes in ((5, 4000), (6, 4800)):
        replay = Replay(2, channels)
        assert replay.img.shape == (2, frame_bytes)
        assert replay.near_img is replay.near_img2 is None
        replay.add(np.zeros((1, frame_bytes), np.uint8), np.zeros((1, 3), np.float32),
                   np.zeros((1, 2), np.float32))
        img, sc, y, pair = replay.batch(np.random.default_rng(1), 1)
        assert img.shape == (1, channels, 80, 80) and pair is None
