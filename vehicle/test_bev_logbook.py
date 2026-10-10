"""影子模式记录本:抱车检测、分段不串数据。"""
import json
import math

import numpy as np

from vehicle.bev_logbook import PICKUP_HOLD, SETTLE, Logbook, PickupDetector


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
    b.add_lidar(np.array([[1.0, 0.0], [1.0, 0.0]]))
    b.add_pose(0.0, (0.0, 0.0, 0.0), 0.1, 0.5, 0.3)
    b.pause()                                                    # 抱起
    b.add_lidar(np.array([[5.0, 5.0]]))                          # 抱着走时的数据不能进任何一段
    b.add_pose(1.0, (9.0, 9.0, 0.0), 0.0, 0.0, 0.0)
    b.resume()                                                   # 放下:新段从零开始
    b.add_lidar(np.array([[2.0, 0.0]]))
    b.add_pose(2.0, (0.0, 0.0, 0.0), -0.1, 0.5, 0.3)
    b.save()
    s1, s2 = np.load(tmp_path / 'seg_01' / 'seg.npz'), np.load(tmp_path / 'seg_02' / 'seg.npz')
    assert s1['lidar_ij'].tolist() == [[20, 0]] and s1['lidar_n'].tolist() == [2]
    assert s2['lidar_ij'].tolist() == [[40, 0]]
    assert len(s1['path']) == 1 and len(s2['path']) == 1 and s2['path'][0, 0] == 2.0
    assert json.load(open(tmp_path / 'seg_02' / 'marks.json')) == []
