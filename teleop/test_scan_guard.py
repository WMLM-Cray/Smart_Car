"""防撞几何的测试:python3 -m pytest teleop/test_scan_guard.py

不需要 ROS,自己造 LaserScan 形状的对象。跑在 PC 上就行。
"""
import math
import time

import pytest

from scan_guard import BUMPER_AHEAD, HALF_CORRIDOR, ScanGuard

N = 1667
INC = 2 * math.pi / N


class FakeScan:
    range_min, range_max = 0.15, 50.0
    angle_min, angle_increment = -math.pi, INC

    def __init__(self, f):
        self.ranges = [f(-math.pi + i * INC) for i in range(N)]


def gap_of(f):
    g = ScanGuard()
    g.update(FakeScan(f))
    return g.gap()


def test_wall_ahead_measured_from_the_bumper():
    gap = gap_of(lambda a: 1.2 / math.cos(a) if abs(a) < 1.0 else float('inf'))
    assert gap == pytest.approx(1.2 - BUMPER_AHEAD, abs=0.01)   # 减掉车头到雷达的距离


def test_obstacles_outside_the_corridor_are_ignored():
    # 只有两侧的墙(横向 0.5m,已经超出走廊半宽)
    assert math.isinf(gap_of(lambda a: 0.5 / abs(math.sin(a)) if 0.5 < abs(a) < 2.6 else float('inf')))
    # 只有后方的障碍
    assert math.isinf(gap_of(lambda a: 0.8 if abs(a) > 2.0 else float('inf')))


def test_pillar_just_inside_the_corridor_counts():
    lateral, ahead = HALF_CORRIDOR - 0.01, 0.9
    ang, r = math.atan2(lateral, ahead), math.hypot(lateral, ahead)
    assert gap_of(lambda a: r if abs(a - ang) < INC else float('inf')) == pytest.approx(
        ahead - BUMPER_AHEAD, abs=0.02)
    lateral = HALF_CORRIDOR + 0.05                                # 挪到走廊外就不算
    ang, r = math.atan2(lateral, ahead), math.hypot(lateral, ahead)
    assert math.isinf(gap_of(lambda a: r if abs(a - ang) < INC else float('inf')))


def test_stale_or_missing_scan_reports_unknown():
    g = ScanGuard()
    assert g.gap() is None                                        # 从来没收到过
    g.update(FakeScan(lambda a: 2.0))
    assert g.gap() is not None
    g.stamp = time.time() - 2.0                                   # 雷达掉线
    assert g.gap() is None
