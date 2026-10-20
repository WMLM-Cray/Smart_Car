"""``common/corridor.py`` 的测试。

合成帧的思路：先用线段（墙）在后轴系里做光线投射，得到一帧和真激光同几何
（1667 条光线、``-pi..pi``）的 ``ranges``；要伪造野点就往对应光线里塞一个距离。
这样"解析解 vs 估计值"的对比是干净的：斜墙在切片里有斜率，所以只有**平行墙**
能拿常数当解析解，弯道只验证曲率符号/量级和 ``c`` 的单调性。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from common.corridor import (
    BODY_FRONT,
    BODY_HALF_WIDTH,
    BODY_REAR,
    DEFAULT_X0,
    LIDAR_X,
    MAX_RAY_GAP,
    NOSE_AHEAD,
    SIDES_BOTH,
    SIDES_LEFT,
    SIDES_NONE,
    SIDES_RIGHT,
    CorridorEstimate,
    extract,
    front_gap,
    parse_ranges,
    polar_to_cartesian,
)
from common.params import CarParams

# 和真帧同几何：1667 条光线铺满 2*pi
ANGLE_MIN = -math.pi
N_RAYS = 1667
ANGLE_INCREMENT = 2.0 * math.pi / N_RAYS
WALL_HALF = 0.75
STATION_08 = DEFAULT_X0.index(0.8)


def _cast(segments, *, n: int = N_RAYS, lidar_x: float = LIDAR_X) -> np.ndarray:
    """把后轴系里的线段（``[(p, q), ...]``）用 ``n`` 条光线投出来，返回极坐标 ``ranges``。"""
    ang = ANGLE_MIN + np.arange(n) * ANGLE_INCREMENT
    cos_a, sin_a = np.cos(ang), np.sin(ang)
    out = np.full(n, np.inf)
    for p, q in segments:
        p = np.asarray(p, dtype=np.float64)
        e = np.asarray(q, dtype=np.float64) - p
        po = p - np.array([lidar_x, 0.0])
        den = cos_a * e[1] - sin_a * e[0]
        ok = np.abs(den) > 1e-12
        t = np.full(n, np.inf)
        s = np.full(n, np.inf)
        t[ok] = (po[0] * e[1] - po[1] * e[0]) / den[ok]
        s[ok] = (po[0] * sin_a[ok] - po[1] * cos_a[ok]) / den[ok]
        hit = ok & (t > 0.0) & (s >= 0.0) & (s <= 1.0)
        out = np.where(hit & (t < out), t, out)
    return out


def _put_point(ranges: np.ndarray, pt, *, lidar_x: float = LIDAR_X) -> int:
    """把 ``ranges`` 里最贴近 ``pt`` 的那条光线改成打中 ``pt``，返回光线号。"""
    dx, dy = pt[0] - lidar_x, pt[1]
    i = int(round((math.atan2(dy, dx) - ANGLE_MIN) / ANGLE_INCREMENT)) % ranges.size
    ranges[i] = math.hypot(dx, dy)
    return i


def _walls(left_y: float | None = WALL_HALF, right_y: float | None = -WALL_HALF,
           x_lo: float = -0.5, x_hi: float = 4.0) -> list:
    """平行墙。``left_y``/``right_y`` 给 ``None`` 表示那一侧没有墙。"""
    segs = []
    if left_y is not None:
        segs.append(((x_lo, left_y), (x_hi, left_y)))
    if right_y is not None:
        segs.append(((x_lo, right_y), (x_hi, right_y)))
    return segs


def _arc_walls(radius: float, h: float = WALL_HALF, t_hi: float = 0.8, n_seg: int = 240) -> list:
    """半径 ``radius``（正 = 左弯）的弯道，两侧边界是同心圆弧，中心线过原点。"""
    s = math.copysign(1.0, radius)
    R = abs(radius)
    t = np.linspace(-t_hi, t_hi, n_seg + 1)

    def arc(rad: float) -> np.ndarray:
        return np.stack([rad * np.sin(t), s * (R - rad * np.cos(t))], axis=1)

    inner, outer = arc(R - h), arc(R + h)
    left, right = (inner, outer) if radius > 0 else (outer, inner)
    segs = []
    for line in (left, right):
        segs.extend((line[i], line[i + 1]) for i in range(n_seg))
    return segs


# --------------------------------------------------------------------------- 解析解

@pytest.mark.parametrize("x0", DEFAULT_X0)
def test_parallel_walls_match_analytic_solution(x0):
    """两侧平行墙：L=+0.75、R=-0.75、w=1.50、c=0，与站位无关。"""
    est = extract(_cast(_walls()), ANGLE_MIN, ANGLE_INCREMENT)
    k = DEFAULT_X0.index(x0)
    assert est.sides[k] == SIDES_BOTH
    assert est.valid[k]
    assert est.n_pts[k] >= 12
    assert est.L[k] == pytest.approx(0.75, abs=5e-3)
    assert est.R[k] == pytest.approx(-0.75, abs=5e-3)
    assert est.w[k] == pytest.approx(1.50, abs=1e-2)
    assert est.c[k] == pytest.approx(0.0, abs=5e-3)
    assert est.frame_valid and est.corridor_valid[k]


def test_parallel_walls_offset_wall_pair():
    """墙对整体左移：L=1.00、R=-0.50 → w=1.50、c=+0.25。"""
    est = extract(_cast(_walls(left_y=1.0, right_y=-0.5)), ANGLE_MIN, ANGLE_INCREMENT)
    assert est.L[STATION_08] == pytest.approx(1.00, abs=5e-3)
    assert est.R[STATION_08] == pytest.approx(-0.50, abs=5e-3)
    assert est.w[STATION_08] == pytest.approx(1.50, abs=1e-2)
    assert est.c[STATION_08] == pytest.approx(0.25, abs=5e-3)


def test_straight_corridor_has_zero_curvature():
    est = extract(_cast(_walls()), ANGLE_MIN, ANGLE_INCREMENT)
    assert est.kappa_valid
    assert est.kappa == pytest.approx(0.0, abs=1e-6)


def test_curved_corridor_kappa_sign_and_scale():
    """同心圆弧走廊：解析曲率 kappa = R/(R^2-h^2)（R=6、h=0.75 → 0.169）。

    解析 ``c(x) = x^2*R/(2*(R^2-h^2))``。只取到 x0=1.6：内墙在掠入射下
    每个切片只剩三四条光线（见 ``test_inner_wall_grazing_starves_far_station``），
    那是可见性问题不是拟合问题。
    """
    x0s = (0.4, 0.6, 0.8, 1.0, 1.2, 1.6)
    est = extract(_cast(_arc_walls(6.0)), ANGLE_MIN, ANGLE_INCREMENT, x0s=x0s)
    assert est.kappa_valid and est.frame_valid
    assert np.all(est.sides == SIDES_BOTH) and np.all(est.valid)
    assert 0.12 < est.kappa < 0.22
    for x0, got in zip(x0s, est.c):
        want = x0 * x0 * 6.0 / (2.0 * (6.0 ** 2 - WALL_HALF ** 2))
        assert got == pytest.approx(want, abs=0.01)
    assert np.all(np.diff(est.c) > 0.0)      # 左弯 → 越远越偏左


def test_curved_corridor_right_turn_has_negative_kappa():
    est = extract(_cast(_arc_walls(-6.0)), ANGLE_MIN, ANGLE_INCREMENT,
                  x0s=(0.4, 0.6, 0.8, 1.0, 1.2, 1.6))
    assert est.kappa_valid
    assert -0.22 < est.kappa < -0.12
    assert np.all(est.c < 0.0)
    assert np.all(np.diff(est.c) < 0.0)


def test_inner_wall_grazing_starves_far_station():
    """急弯里远站位的内墙只剩三四条光线，簇判据会把它判成"没看见"。

    R=4、左墙 0.75 m：内墙在 x≈2.0 处几乎和光线相切，一个 0.2 m 的切片里
    只落得下 3 条光线 < ``MIN_CLUSTER_PTS`` → ``sides`` 退化成单侧、``c`` 给 nan。
    这是可见性/采样密度的取舍，宁可报"只看见一侧"也不报一个底层只有三个点的中心。
    """
    est = extract(_cast(_arc_walls(4.0)), ANGLE_MIN, ANGLE_INCREMENT)
    assert est.sides[DEFAULT_X0.index(1.6)] == SIDES_BOTH
    k = DEFAULT_X0.index(2.0)
    assert est.sides[k] == SIDES_RIGHT
    assert est.n_pts[k] < 25
    assert math.isnan(est.c[k]) and math.isnan(est.w[k])
    assert est.kappa_valid and est.kappa > 0.0      # 近站位仍够拟合


# --------------------------------------------------------------------------- 边界

def test_one_sided_corridor_reports_sides_not_corridor():
    """只有左墙：L 照给，R/w/c 给 nan，valid 真但 corridor_valid 假。"""
    est = extract(_cast(_walls(right_y=None)), ANGLE_MIN, ANGLE_INCREMENT)
    k = STATION_08
    assert est.sides[k] == SIDES_LEFT
    assert est.valid[k]
    assert est.L[k] == pytest.approx(0.75, abs=5e-3)
    assert math.isnan(est.R[k]) and math.isnan(est.w[k]) and math.isnan(est.c[k])
    assert not est.corridor_valid[k]
    assert not est.frame_valid and not est.both_sided
    assert not est.kappa_valid


def test_right_only_side():
    est = extract(_cast(_walls(left_y=None)), ANGLE_MIN, ANGLE_INCREMENT)
    k = STATION_08
    assert est.sides[k] == SIDES_RIGHT
    assert est.R[k] == pytest.approx(-0.75, abs=5e-3)
    assert math.isnan(est.L[k]) and math.isnan(est.c[k])
    assert not est.frame_valid


def test_empty_slice_is_invalid():
    """墙只铺到 x=1.2，x0=2.0 的切片里一个点都没有。"""
    frame = _cast(_walls(x_lo=-0.5, x_hi=1.2))
    est = extract(frame, ANGLE_MIN, ANGLE_INCREMENT, x0s=(0.8, 2.0))
    assert est.valid[0] and est.n_pts[0] > 0
    assert est.n_pts[1] == 0
    assert not est.valid[1] and est.sides[1] == SIDES_NONE
    assert math.isnan(est.L[1]) and math.isnan(est.R[1])
    assert math.isnan(est.w[1]) and math.isnan(est.c[1])


def test_axis_stray_points_do_not_capture_the_corridor():
    """本任务的核心回归：车轴附近的孤立散点不能把 L/R 拉到 0。

    原型 ``runs/corridor_extract_probe.py`` 的 ``min(y>0)`` 在这里会返回 ~0.002，
    本模块靠"占用簇 + 最小点数"把它们整簇丢掉。
    """
    frame = _cast(_walls())
    strays = ((0.80, 0.0020), (0.82, -0.0015), (0.78, 0.0040))
    for pt in strays:
        _put_point(frame, pt)

    # 反证：朴素规则 min(y>0) 确实会被带偏
    x, y, _, _ = polar_to_cartesian(frame, ANGLE_MIN, ANGLE_INCREMENT)
    m = np.abs(x - 0.8) <= 0.10
    assert float(y[m][y[m] > 0.0].min()) < 0.01

    est = extract(frame, ANGLE_MIN, ANGLE_INCREMENT)
    k = STATION_08
    assert est.n_pts[k] > 20
    assert est.sides[k] == SIDES_BOTH
    assert est.L[k] == pytest.approx(0.75, abs=1e-2)
    assert est.R[k] == pytest.approx(-0.75, abs=1e-2)
    assert est.w[k] == pytest.approx(1.50, abs=2e-2)
    assert est.c[k] == pytest.approx(0.0, abs=1e-2)


def test_wall_running_along_the_corridor_is_not_fragmented():
    """回归：沿走廊铺开的墙不能被拆成孤立点（真实数据里最主要的失败原因）。

    实证 sim 41.50：真值墙在 y=0.542，真实帧在 x∈[0.3,0.5] 切片里有 30+ 个回波，
    但这些回波的 ``|lat|`` 只差几毫米、激光序号却跳 5~20——簇游走若按 ``|lat|``
    排，每个点都和前一个"序号断裂"，自成一个个 <min_pts 的小簇被全部拒掉，L 从
    0.54 跳到 1.42。必须按激光序号（角度相邻 = 表面相邻）排。
    """
    frame = _cast(_walls(left_y=0.55))
    rng = np.random.default_rng(20261005)
    fin = np.isfinite(frame)
    frame[fin] += rng.normal(0.0, 0.002, int(fin.sum()))  # 2 mm 量级测距噪声

    x, y, rays, _ = polar_to_cartesian(frame, ANGLE_MIN, ANGLE_INCREMENT)
    # 反证：这面墙按 |lat| 排序后，相邻点的激光序号断裂远超门槛
    m = (np.abs(x - 0.4) <= 0.10) & (y > 0.0)
    o = np.argsort(np.abs(y[m]), kind="stable")
    assert int(np.abs(np.diff(rays[m][o])).max()) > MAX_RAY_GAP

    est = extract(frame, ANGLE_MIN, ANGLE_INCREMENT, x0s=(0.4, 0.8))
    assert est.sides[0] == SIDES_BOTH and est.sides[1] == SIDES_BOTH
    assert est.L[0] == pytest.approx(0.55, abs=1e-2)
    assert est.L[1] == pytest.approx(0.55, abs=1e-2)


def test_real_obstacle_blob_is_kept():
    """簇判据不能把真实障碍也丢掉：轴侧一整块回波仍要报出来。"""
    frame = np.full(N_RAYS, np.inf)
    for i in range(10):
        _put_point(frame, (0.80, 0.50 + 0.01 * i))
    est = extract(frame, ANGLE_MIN, ANGLE_INCREMENT, x0s=(0.8,))
    assert est.n_pts[0] >= 8
    assert est.sides[0] == SIDES_LEFT
    assert est.L[0] == pytest.approx(0.50, abs=1e-2)


def test_all_infinite_and_nan_frames_do_not_crash():
    frames = [
        [float("inf")] * N_RAYS,
        [float("nan")] * N_RAYS,
        ["+Infinity"] * N_RAYS,
        ["-Infinity"] * N_RAYS,
        ["NaN"] * N_RAYS,
        np.full(N_RAYS, np.inf),
        [],
    ]
    for frame in frames:
        est = extract(frame, ANGLE_MIN, ANGLE_INCREMENT)
        assert isinstance(est, CorridorEstimate)
        assert est.n_returns == 0
        assert not est.frame_valid and not est.both_sided and not est.kappa_valid
        assert not bool(np.any(est.valid))
        assert np.all(np.isnan(est.L)) and np.all(np.isnan(est.R))
        assert np.all(np.isnan(est.w)) and np.all(np.isnan(est.c))
        assert est.kappa == 0.0
        got = est.at(0.8)
        assert got["sides"] == SIDES_NONE and math.isnan(got["L"])
        assert front_gap(est) == math.inf


# --------------------------------------------------------------------------- 解析与过滤

def test_parse_ranges_handles_strings_and_floats():
    got = parse_ranges(["+Infinity", "-Infinity", "NaN", "1.5", 2, np.float32(3.0)])
    assert np.all(np.isnan(got[:3]))
    np.testing.assert_allclose(got[3:], [1.5, 2.0, 3.0])
    got = parse_ranges(np.array([1.0, np.inf, np.nan, 4.0]))
    assert math.isnan(got[1]) and math.isnan(got[2])
    np.testing.assert_allclose([got[0], got[3]], [1.0, 4.0])


def test_string_frame_matches_float_frame():
    frame = _cast(_walls())
    as_str = ["+Infinity" if not np.isfinite(v) else repr(float(v)) for v in frame]
    a = extract(frame, ANGLE_MIN, ANGLE_INCREMENT)
    b = extract(as_str, ANGLE_MIN, ANGLE_INCREMENT)
    assert a.n_returns == b.n_returns
    np.testing.assert_allclose(a.L, b.L, equal_nan=True)
    np.testing.assert_allclose(a.c, b.c, equal_nan=True)


def test_range_min_and_range_max_filter_returns():
    frame = _cast(_walls())
    i_stray = _put_point(frame, (0.80, 0.001))
    assert frame[i_stray] < 0.8

    est = extract(frame, ANGLE_MIN, ANGLE_INCREMENT, range_min=0.8)
    assert est.L[STATION_08] == pytest.approx(0.75, abs=1e-2)   # 墙在 ~1.0 m，还在

    _, _, _, keep = polar_to_cartesian(frame, ANGLE_MIN, ANGLE_INCREMENT, range_min=0.8)
    assert not keep[i_stray]

    est = extract(frame, ANGLE_MIN, ANGLE_INCREMENT, range_max=0.9)
    assert not est.valid[STATION_08]        # 只剩那个野点，凑不够 min_station_pts


def test_self_filter_drops_body_returns():
    frame = np.full(N_RAYS, np.inf)
    body = (0.30, 0.10)       # 车体足迹内
    side = (0.30, 0.30)       # 同一纵向位置但在车宽之外
    nose = (0.50, 0.10)       # 车头之前
    far = (0.60, 0.10)
    for pt in (body, side, nose, far):
        _put_point(frame, pt)

    x, y, _, keep = polar_to_cartesian(frame, ANGLE_MIN, ANGLE_INCREMENT)
    assert x.size == 3
    assert not keep[_put_point(np.full(N_RAYS, np.inf), body)]
    # 被丢掉的是 body；留下的是 side/nose/far
    assert np.all(y > 0.05)

    x2, y2, _, _ = polar_to_cartesian(frame, ANGLE_MIN, ANGLE_INCREMENT, self_filter=False)
    assert x2.size == 4
    assert np.min(y2) < 0.12


def test_body_footprint_is_single_sourced_from_params():
    """车体足迹不许写死：必须等于 common/params.py 的实测轮廓，且与车头口径同源。"""
    p = CarParams()
    assert BODY_FRONT == pytest.approx(p.length - p.rear_overhang)   # 0.415
    assert BODY_REAR == pytest.approx(-p.rear_overhang)              # -0.065
    assert BODY_HALF_WIDTH == pytest.approx(p.width / 2.0)           # 0.175
    assert BODY_FRONT == pytest.approx(NOSE_AHEAD)
    # 旧仿真口径（车长 0.56 ⇒ 前界 0.45）比实测车头多 3.5 cm，会把贴车头的锥桶点一起滤掉
    assert BODY_FRONT < 0.45


def test_self_filter_keeps_a_cone_just_ahead_of_the_nose():
    """车头前 3 cm 的锥桶点必须留下：旧 0.45 足迹会把它当自回波滤掉。"""
    frame = np.full(N_RAYS, np.inf)
    idx = _put_point(frame, (0.44, 0.05))
    x, _, _, keep = polar_to_cartesian(frame, ANGLE_MIN, ANGLE_INCREMENT)
    assert x.size == 1
    assert keep[idx]


def test_front_gap_matches_scan_guard_style():
    est = extract(_cast(_walls()), ANGLE_MIN, ANGLE_INCREMENT)
    assert front_gap(est) == pytest.approx(0.75 - NOSE_AHEAD, abs=1e-2)   # x0=0.4 在车头之后，不算

    # 走廊中心偏出 scan_guard 的 |c|<=0.28 带 → 它那套口径无从谈起
    shifted = extract(_cast(_walls(left_y=1.25, right_y=-0.25)), ANGLE_MIN, ANGLE_INCREMENT)
    assert front_gap(shifted) == math.inf
    assert front_gap(shifted, half_corridor=0.5) == pytest.approx(1.25 - NOSE_AHEAD, abs=1e-2)


def test_at_picks_nearest_station():
    est = extract(_cast(_walls()), ANGLE_MIN, ANGLE_INCREMENT)
    got = est.at(0.79)
    assert got["x0"] == 0.8
    assert got["L"] == pytest.approx(0.75, abs=5e-3)
    assert got["valid"] is True
