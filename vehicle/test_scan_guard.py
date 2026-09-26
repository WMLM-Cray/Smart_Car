"""防撞几何的测试:python3 -m pytest teleop/test_scan_guard.py

不需要 ROS,自己造 LaserScan 形状的对象。跑在 PC 上就行。
"""
import math
import time

import pytest

from vehicle.scan_guard import (BUMPER_AHEAD, HALF_CORRIDOR, LIDAR_X, NOSE_AHEAD,
                        WHEELBASE, ScanGuard)

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


# --- 扫掠路径(2026-09-22 加)---
# 赛道 A 的 180° 掉头内径 1.25m,内侧锥桶会落在正前方的直走廊里,但车在打方向,
# 实际绕得过去。下面几项钉死"直走廊会误报、扫掠路径不会"。

def scan_of(points):
    """一堆 (x, y)(雷达系)变成一帧扫描,其余方向无穷远。"""
    hits = {}
    for x, y in points:
        ang = math.atan2(y, x)
        i = int(round((ang + math.pi) / INC)) % N
        hits[i] = min(hits.get(i, float('inf')), math.hypot(x, y))
    return _Scan(hits)


class _Scan:
    range_min, range_max = 0.15, 50.0
    angle_min, angle_increment = -math.pi, INC

    def __init__(self, hits):
        self.ranges = [hits.get(i, float('inf')) for i in range(N)]


def guard_with(points):
    g = ScanGuard()
    g.update(_Scan({}) if not points else scan_of(points))
    return g


def test_cone_off_to_the_side_is_dropped_when_turning_away():
    """转向背离的一侧那个锥桶,直走廊算它,扫掠路径不算。

    注意扫掠路径**不能**把正前方的锥桶放掉:赛道 A 内径 1.25m 的弯,走 0.65m
    弧长才让开 0.165m,而车半宽 0.175 + 锥桶半宽 0.14 = 0.315m —— 正前方的桶
    是真要刮上去的,截停是对的。这个改动只去掉"已经偏在一边、还往反方向打"
    那一类误报。
    """
    g = guard_with([(0.9 - LIDAR_X, 0.25)])          # 左前方,横向 0.25 < 走廊半宽
    assert g.gap(0.0) == pytest.approx(0.9 - NOSE_AHEAD, abs=0.03), '直行时要算它'
    assert math.isinf(g.gap(-0.4)), '往右打方向,这个点在环带外,应该放掉'


def test_cone_dead_ahead_still_stops_you_mid_turn():
    """正前方的锥桶,打着方向也得报 —— 弯太缓,横向让不开。"""
    g = guard_with([(0.65 - LIDAR_X, 0.0)])
    assert g.gap(math.atan(WHEELBASE / 1.25)) is not None
    assert not math.isinf(g.gap(math.atan(WHEELBASE / 1.25)))


def test_obstacle_on_the_arc_still_counts():
    """障碍正好落在转弯圆弧上,必须报,而且距离要按弧长算(比直线远)。"""
    steer = 0.4
    radius = WHEELBASE / math.tan(steer)
    theta = 0.9                                     # 沿弧转 0.9 rad 处
    x = radius * math.sin(theta)
    y = radius * (1 - math.cos(theta))
    g = guard_with([(x - LIDAR_X, y)])              # 换回雷达系
    along = radius * theta
    assert g.gap(steer) == pytest.approx(along - NOSE_AHEAD, abs=0.05)
    assert g.gap(steer) > math.hypot(x, y) - NOSE_AHEAD, '弧长应该大于直线距离'


def test_straight_corridor_unchanged_by_tiny_steer():
    """小转角(<STRAIGHT_STEER)按直行算,别在直道上因为微调方向就换了口径。"""
    g = guard_with([(1.2, 0.0)])
    assert g.gap(0.0) == pytest.approx(g.gap(0.01), abs=1e-6)


def test_obstacle_behind_on_the_same_circle_is_ignored():
    """圆弧绕一圈会回到车身后。身后的点不能被当成'前方还有很远'。"""
    steer = 0.5
    radius = WHEELBASE / math.tan(steer)
    theta = 5.5                                     # 快绕满一圈,已经在车后
    x, y = radius * math.sin(theta), radius * (1 - math.cos(theta))
    g = guard_with([(x - LIDAR_X, y)])
    assert math.isinf(g.gap(steer))
