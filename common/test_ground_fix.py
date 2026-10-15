"""在线俯仰修正 + 脏点筛除:合成场景(平地 + 正前方一堵墙,真实俯仰和标定不同)。"""
import math
from dataclasses import replace

import numpy as np

from common.ground_fix import GroundFixer, bottom_connected, wall_points
from common.ground_proj import GroundLut
from common.params import CarParams

P = CarParams()
ROWS, COLS = 72, 96
N, A0 = 1667, -math.pi
INC = 2 * math.pi / N


def scene(wall_x=2.0, true_pitch_deg=None):
    """车体系 x=wall_x 处一堵横墙(其余方向 5 m 外有墙):返回 (按真实俯仰渲染的掩码, 雷达扫描)。"""
    q = P if true_pitch_deg is None else replace(P, camera_pitch=math.radians(true_pitch_deg))
    lut = GroundLut(ROWS, COLS, q, 10.0)
    mask = (lut.valid & (lut.xy[:, 0] < wall_x)).reshape(ROWS, COLS)
    a = A0 + np.arange(N) * INC
    with np.errstate(divide='ignore'):
        r = np.where(np.cos(a) > 0.05, (wall_x - P.lidar_x) / np.cos(a), np.inf)
    r = np.minimum(r, 5.0)
    return mask, (r, A0, INC)


def test_wall_points_keep_walls_drop_legs():
    _, (r, a0, inc) = scene()
    r = r.copy()
    r[830:834] = 1.0                                             # 正前方一根椅腿:4 根回波
    xy = wall_points(r, a0, inc, P)
    assert len(xy) > 200
    assert np.allclose(xy[:, 0], 2.0, atol=1e-6)                 # 全在墙上,椅腿没进来


def test_recovers_true_pitch_and_stops_floor_at_the_wall():
    true = math.degrees(P.camera_pitch) + 3.0
    mask, scan = scene(true_pitch_deg=true)
    fx = GroundFixer(ROWS, COLS, P)
    xy, info = fx.process(mask, now=0.0, scan=scan)
    assert info.source == 'lidar'
    assert abs(math.degrees(info.pitch) - true) <= 0.5
    assert xy[:, 0].max() <= 2.0 + 0.1                           # 地面停在墙前

    plain = GroundLut(ROWS, COLS, P, 3.0).points(mask)           # 不修:按标定值投,穿墙
    assert plain[:, 0].max() > 2.5


def test_no_scan_falls_back_to_calibration_and_two_metres():
    mask, _ = scene(wall_x=9.0)
    fx = GroundFixer(ROWS, COLS, P)
    xy, info = fx.process(mask, now=0.0, scan=None)
    assert info.source == 'calib' and info.pitch == P.camera_pitch
    assert np.hypot(xy[:, 0] - P.camera_x, xy[:, 1]).max() <= 2.0 + 0.05


def test_holds_last_lidar_pitch_for_a_second():
    true = math.degrees(P.camera_pitch) + 2.0
    mask, scan = scene(true_pitch_deg=true)
    fx = GroundFixer(ROWS, COLS, P)
    _, a = fx.process(mask, now=10.0, scan=scan)
    _, b = fx.process(mask, now=10.5, scan=None)
    _, c = fx.process(mask, now=11.6, scan=None)
    assert a.source == 'lidar' and b.source == 'hold' and b.pitch == a.pitch
    assert c.source == 'calib'


def test_lidar_cuts_floor_behind_an_obstacle():
    mask, (r, a0, inc) = scene(wall_x=9.0)
    r = np.full(N, np.inf)
    r[np.abs(A0 + np.arange(N) * INC) < 0.3] = 1.0               # 正前方 ±17° 1 m 处有东西(雷达高度)
    fx = GroundFixer(ROWS, COLS, P)
    xy, info = fx.process(mask, now=0.0, scan=(r, a0, inc))
    ahead = np.abs(np.arctan2(xy[:, 1], xy[:, 0] - P.lidar_x)) < 0.25
    assert info.cut > 0
    assert np.hypot(xy[ahead, 0] - P.lidar_x, xy[ahead, 1]).max() <= 1.0 + 0.15 + 0.05


def test_floor_islands_not_touching_the_bottom_are_dropped():
    m = np.zeros((ROWS, COLS), bool)
    m[50:, :] = True
    m[10:15, 40:50] = True                                       # 墙上的反光
    keep = bottom_connected(m)
    assert keep[50:].all() and not keep[:50].any()
