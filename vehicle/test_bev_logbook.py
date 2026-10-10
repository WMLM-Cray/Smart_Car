"""影子模式记录本:抱车检测、分段不串数据。"""
import json
import math

import numpy as np

from vehicle.bev_logbook import CLAMP, OCC_ON, PICKUP_HOLD, SETTLE, Logbook, PickupDetector


def _feed(det, seconds, t0, wx=0.0, tilt_deg=0.0, dt=0.01):
    ev, t = [], t0
    az, ax = 9.8 * math.cos(math.radians(tilt_deg)), 9.8 * math.sin(math.radians(tilt_deg))
    for _ in range(int(seconds / dt)):
        e = det.update(wx, 0.0, ax, 0.0, az, t)
        if e:
            ev.append((e, round(t, 2)))
        t += dt
    return ev, t


def test_bump_is_not_a_pickup_but_lifting_is():
    d = PickupDetector()
    ev, t = _feed(d, 1.0, 0.0)
    assert ev == []
    ev, t = _feed(d, PICKUP_HOLD / 2, t, wx=1.0)               # 过坎:短尖峰
    assert ev == []
    ev, t = _feed(d, 0.5, t)
    ev, t = _feed(d, 1.0, t, tilt_deg=20)                       # 抱起来:持续倾斜
    assert [e for e, _ in ev] == ['pickup']
    ev, t = _feed(d, SETTLE / 2, t)                             # 刚放下,还没静止够
    assert ev == []
    ev, t = _feed(d, SETTLE, t)
    assert [e for e, _ in ev] == ['settled']


def test_segments_do_not_mix(tmp_path):
    b = Logbook(str(tmp_path))
    b.add_lidar(np.array([[1.0, 0.0], [1.0, 0.0]]), (0.0, 0.0))
    b.add_pose(0.0, (0.0, 0.0, 0.0), 0.1, 0.5, 0.3)
    b.pause()                                                    # 抱起
    b.add_lidar(np.array([[5.0, 5.0]]), (0.0, 0.0))                          # 抱着走时的数据不能进任何一段
    b.add_pose(1.0, (9.0, 9.0, 0.0), 0.0, 0.0, 0.0)
    b.resume()                                                   # 放下:新段从零开始
    b.add_lidar(np.array([[2.0, 0.0]]), (0.0, 0.0))
    b.add_pose(2.0, (0.0, 0.0, 0.0), -0.1, 0.5, 0.3)
    b.save()
    s1, s2 = np.load(tmp_path / 'seg_01' / 'seg.npz'), np.load(tmp_path / 'seg_02' / 'seg.npz')
    hit = lambda z: z['lidar_ij'][z['lidar_n'] > 0].tolist()
    assert hit(s1) == [[20, 0]] and hit(s2) == [[40, 0]]                 # 一帧里同一格只算一次
    assert len(s1['path']) == 1 and len(s2['path']) == 1 and s2['path'][0, 0] == 2.0
    assert json.load(open(tmp_path / 'seg_02' / 'marks.json')) == []


def _scan_wall(b, origin, wall_x, ys):
    b.add_lidar(np.array([[wall_x, y] for y in ys]), origin)


def test_person_trail_is_cleared_when_rescanned_but_wall_stays(tmp_path):
    b = Logbook(str(tmp_path))
    ys = np.arange(-1.5, 1.51, 0.02)
    for _ in range(200):                                         # 停着:人站在 x=1 处被扫到 200 帧
        b.add_lidar(np.array([[1.0, 0.0]] + [[3.0, y] for y in ys]), (0.0, 0.0))
    for _ in range(CLAMP + OCC_ON):                              # 人走了:几帧就压回去(证据封顶)
        _scan_wall(b, (0.0, 0.0), 3.0, ys)
    for _ in range(10):                                          # 车开到墙边斜着扫墙
        _scan_wall(b, (2.5, -1.0), 3.0, ys)
    k = lambda x, y: (round(x / 0.05), round(y / 0.05))
    assert b.lidar[k(1.0, 0.0)] < OCC_ON                          # 人的影子不再画成障碍
    assert all(b.lidar[k(3.0, y)] >= OCC_ON for y in ys)          # 墙一格都没被擦掉


def test_wrong_floor_is_overturned_by_later_views(tmp_path):
    fov = np.array([[x, y] for x in np.arange(0.5, 3.0, 0.05) for y in np.arange(-1.0, 1.0, 0.05)])
    b = Logbook(str(tmp_path), fov_body=fov)
    for _ in range(100):                                         # 分割把 (2, 0) 错当地面 100 帧
        b.add_free(np.array([[2.0, 0.0]]), (0.0, 0.0, 0.0))
    assert b.free[(40, 0)] == CLAMP
    for _ in range(2 * CLAMP):                                   # 之后分割改说 (1, 0) 是地面、(2, 0) 不是
        b.add_free(np.array([[1.0, 0.0]]), (0.0, 0.0, 0.0))
    assert b.free[(40, 0)] == -CLAMP and b.free[(20, 0)] == CLAMP   # 封顶:翻过来最多 2×CLAMP 帧
