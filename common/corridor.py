"""从单帧 360° 激光里解出"可行走廊"：左墙多远、右墙多远、走廊中心在哪、前面弯不弯。

坐标系
------
一切输出都在**后轴坐标系**里：原点 = 后轴中心，+x 指向车头，+y 指向车左。
雷达不在原点上，装在后轴前 ``LIDAR_X`` m（见下方常量；单源于 ``common/params.py`` 的实测值），
所以极坐标 ``(r, ang)`` 先换成 ``x = r*cos(ang) + LIDAR_X``、``y = r*sin(ang)``。
``ang = 0`` 是正前，``ang > 0`` 是左（与前向激光的 ``angle_min + i*increment`` 同一约定）。

"走廊"怎么定义
--------------
对每个前瞻站位 ``x0``，取纵向切片 ``|x - x0| <= half_slice``，在切片里找车轴
左右两侧各自的**最近连续占用簇**，取其靠车轴那一侧的边缘：

    L(x0) = 该簇里 y > 0 的最小 y      R(x0) = 该簇里 y < 0 的最大 y
    w(x0) = L - R                      c(x0) = (L + R) / 2

``L``/``R`` 是"离车轴最近的障碍的横向位置"，不是"自由空间边界"：正前方 0.6 m 处
骑在车轴上的锥面会把 ``L`` 压到 0 附近、``w`` 压到车宽以下，这是**真实几何**而不是
野点（本仓 ``runs/corridor_truth_check.py`` 用真值验证过），消费方看到 ``w`` 小于车宽
应当理解为"正前方有障碍"，而不是"估计失败"。

为什么不能用 ``min(y>0)`` / ``max(y<0)``
----------------------------------------
``runs/corridor_extract_probe.py`` 的写法对孤立野点毫无抵抗力：切片里任何一个落在
车轴附近的坏点都会把 ``L`` 拉到 0 附近。本模块改成"占用簇"——把切片内的点按
**激光序号**排好，序号断裂超过 ``MAX_RAY_GAP`` 或横向断裂超过 ``MAX_LAT_GAP`` 就算新簇，
只有点数 ≥ ``MIN_CLUSTER_PTS`` 且跨 ≥ ``MIN_CLUSTER_RAYS`` 条光线的簇才被承认。
真实墙面/锥面总是一段连续光线，孤立野点只有一两条，据此区分。

已知边界
--------
- 只看得见表面。锥的背面被自己挡住，所以"锥的整个方形足迹是否压住车轴"这种真值
  在本模块里观测不到（本仓真值核对报告里同时给了"可见面真值"和"整块真值"两套口径）。
- 车体自身回波被 ``self_filter`` 按足迹剔掉；被剔掉的区域无法再报障，属已知取舍。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from common.params import CarParams

# 雷达相对后轴的前向偏移（m）。单源于 common/params.py 的实测值（2026-09-30 实测 0.145）。
# ⚠️ 待定（两套几何值）：旧值是 0.07，
#    那也是 Gazebo 世界自己用的值（Auto_Car-main/3Dsim/.../config/params.yaml laser_x），
#    与 vehicle/scan_guard.py 的 LIDAR_X 一致。这里对齐仓库内实测值后，仿真里读到的
#    点云会比仿真真实雷达位置前移 7.5 cm —— 与宿主/实车口径一致，但与旧跑批（E/F/L/H）
#    的读数不可直接混比。
LIDAR_X = CarParams().lidar_x

# 默认前瞻站位（m）。0.4 是车头(0.415)附近，2.0 是 scan_guard 的 MAX_LOOKAHEAD 的一半；
# 更远的站位在窄道里常常只剩零星回波，由 valid 标出来。
DEFAULT_X0 = (0.4, 0.6, 0.8, 1.0, 1.2, 1.6, 2.0)

# 纵向切片半宽（m）
HALF_SLICE = 0.10

# 占用簇的判据
MAX_RAY_GAP = 3          # 相邻点的激光序号差超过它就算断开
MAX_LAT_GAP = 0.08       # 横向断裂超过它也算断开
MIN_CLUSTER_PTS = 4      # 一个簇至少这么多点
MIN_CLUSTER_RAYS = 4     # 且至少跨这么多条光线
MIN_STATION_PTS = 6      # 切片点数少于它就判该站位不可信

# 车体足迹（后轴系，m）。落在这里面的回波当成自遮挡丢掉。
# 单源于仓库内实测几何（common/params.py），不再写死：车长 0.48 − 后悬 0.065 = 0.415、
# 后缘 −0.065、半宽 0.35/2 = 0.175。这与 vehicle/scan_guard.py 的
# NOSE_AHEAD = LIDAR_X + BUMPER_AHEAD = 0.145 + 0.27 = 0.415 同值，两边一个来源。
# ⚠️ 旧值 0.45 / −0.15 / 0.195 是**仿真车体**（车长 0.56）的自回波口径，比实测车头多 3.5 cm，
#    会把贴着车头的锥桶点一起滤掉。
# 若实车自回波实测后需要外扩，必须作为带实测依据的具名常量加回来，不要就地改数。
BODY_FRONT = CarParams().length - CarParams().rear_overhang
BODY_REAR = -CarParams().rear_overhang
BODY_HALF_WIDTH = CarParams().width / 2.0

# 后轴到车头前缘（m）。单源于仓库内实测几何：车长 0.48 − 后悬 0.065 = 0.415
# （与 vehicle/scan_guard.py 的 NOSE_AHEAD = LIDAR_X + BUMPER_AHEAD = 0.145 + 0.27 同值）。
# ⚠️ 旧值是 0.43（旧车长 0.56 口径）。
NOSE_AHEAD = CarParams().length - CarParams().rear_overhang

# 曲率拟合至少要几个有效站位、站位跨度至少多少米
KAPPA_MIN_STATIONS = 3
KAPPA_MIN_SPAN = 0.8

SIDES_NONE, SIDES_LEFT, SIDES_RIGHT, SIDES_BOTH = 0, 1, 2, 3


@dataclass(frozen=True)
class CorridorEstimate:
    """一帧走廊估计。数组长度都等于 ``x0`` 的长度。"""

    x0: tuple = field(default_factory=tuple)
    L: np.ndarray = field(default_factory=lambda: np.zeros(0))
    R: np.ndarray = field(default_factory=lambda: np.zeros(0))
    w: np.ndarray = field(default_factory=lambda: np.zeros(0))
    c: np.ndarray = field(default_factory=lambda: np.zeros(0))
    valid: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))
    sides: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int8))
    n_pts: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int32))
    kappa: float = 0.0
    kappa_valid: bool = False
    n_returns: int = 0

    @property
    def frame_valid(self) -> bool:
        """整帧能不能给出走廊：至少有一个站位左右两侧都看到了障碍。"""
        return bool(np.any(self.corridor_valid))

    @property
    def corridor_valid(self) -> np.ndarray:
        """``w``/``c`` 真正成立的站位：切片可信且左右两侧都有障碍。"""
        return self.valid & (self.sides == SIDES_BOTH)

    @property
    def both_sided(self) -> bool:
        """至少有一个站位左右两侧都看到了障碍。"""
        return bool(np.any(self.sides == SIDES_BOTH))

    def at(self, x0: float) -> dict:
        """取最接近 ``x0`` 的那个站位，返回普通 dict（给不便用 numpy 的调用方）。"""
        k = int(np.argmin(np.abs(np.asarray(self.x0) - x0)))
        return {
            "x0": float(self.x0[k]),
            "L": float(self.L[k]),
            "R": float(self.R[k]),
            "w": float(self.w[k]),
            "c": float(self.c[k]),
            "valid": bool(self.valid[k]),
            "sides": int(self.sides[k]),
        }


def parse_ranges(ranges) -> np.ndarray:
    """把一帧 ``ranges`` 变成 float64 数组，无效量（``inf``/``NaN``/``'-Infinity'``）变 ``nan``。

    同时接受已经是 float 的数组（rospy 的 ``LaserScan``）和字符串数组
    （``runs/*/scan_frames.jsonl`` 里是字符串）。
    """
    if isinstance(ranges, np.ndarray) and ranges.dtype.kind == "f":
        out = ranges.astype(np.float64, copy=True)
    else:
        out = np.empty(len(ranges), dtype=np.float64)
        for i, v in enumerate(ranges):
            if isinstance(v, str):
                try:
                    out[i] = float(v)
                except ValueError:
                    out[i] = np.nan
            else:
                out[i] = float(v)
    out[~np.isfinite(out)] = np.nan
    return out


def polar_to_cartesian(
    ranges,
    angle_min: float,
    angle_increment: float,
    *,
    lidar_x: float = LIDAR_X,
    range_min: float = 0.0,
    range_max: float = math.inf,
    self_filter: bool = True,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """极坐标 → 后轴系直角坐标。

    返回 ``(x, y, ray_index, keep)``，只含留下的点。``keep`` 是原数组长度的布尔掩码，
    方便调用方对照原始光线；``x``/``y``/``ray_index`` 是压缩后的数组。

    被丢掉的是：非有限值、超出 ``[range_min, range_max]`` 的值、以及
    ``self_filter`` 打开时落在车体足迹内的自回波。
    """
    r = parse_ranges(ranges)
    ang = angle_min + np.arange(r.size, dtype=np.float64) * angle_increment
    # 归一到 [-pi, pi)，和 scan_guard.py 一致
    ang = (ang + math.pi) % (2.0 * math.pi) - math.pi
    x = r * np.cos(ang) + lidar_x
    y = r * np.sin(ang)

    keep = np.isfinite(r)
    if range_min > 0.0:
        keep &= r >= range_min
    if math.isfinite(range_max):
        keep &= r <= range_max
    if self_filter:
        in_body = (x >= BODY_REAR) & (x <= BODY_FRONT) & (np.abs(y) <= BODY_HALF_WIDTH)
        keep &= ~in_body

    idx = np.nonzero(keep)[0]
    return x[idx], y[idx], idx, keep


def _nearest_cluster(
    lat: np.ndarray,
    rays: np.ndarray,
    *,
    max_ray_gap: int = MAX_RAY_GAP,
    max_lat_gap: float = MAX_LAT_GAP,
    min_pts: int = MIN_CLUSTER_PTS,
    min_rays: int = MIN_CLUSTER_RAYS,
) -> float | None:
    """在单侧（``lat`` 全同号）里找离车轴最近的合格占用簇，返回它的内边缘。

    按**激光序号**排（不是按 ``|lat|`` 排）：相邻光线打到的是相邻表面patch，
    这才是"连续占用"的物理含义。沿走廊方向铺开的墙（比如 y≈0.55 的侧墙），它
    各点的 ``|lat|`` 几乎相同、而光线序号单调——按 ``|lat|`` 排会把同一堵墙打乱
    成一个个孤点（每个都被当成野点丢掉），L 于是跳到很远的下一堵墙。反过来，
    按序号排以后，就靠"序号断裂 > ``max_ray_gap``（丢点）"或"横向跳变 >
    ``max_lat_gap``（换了个物体）"切段，取长度 ≥ ``min_pts`` 且跨 ≥ ``min_rays``
    条光线的段，从这些段里挑内边缘最小（最近）的那个。

    这样贴轴的孤立散点（1–3 条光线）照样被拒，而真实的墙不会再被拆散。
    """
    if lat.size == 0:
        return None
    order = np.argsort(rays, kind="stable")
    d = np.abs(lat[order])
    ray = rays[order]

    best: float | None = None
    start = 0
    n = d.size
    for i in range(1, n + 1):
        broken = (
            i == n
            or int(ray[i]) - int(ray[i - 1]) > max_ray_gap
            or abs(d[i] - d[i - 1]) > max_lat_gap
        )
        if not broken:
            continue
        cnt = i - start
        span = int(ray[start:i].max() - ray[start:i].min()) + 1 if cnt else 0
        if cnt >= min_pts and span >= min_rays:
            inner = float(d[start:i].min())
            if best is None or inner < best:
                best = inner
        start = i
    return best


def _station(
    x: np.ndarray,
    y: np.ndarray,
    rays: np.ndarray,
    x0: float,
    half_slice: float,
    **kw,
) -> tuple[float, float, int, int]:
    """单个站位。返回 ``(L, R, n_pts, sides)``，没找到的那侧给 ``nan``。"""
    m = np.abs(x - x0) <= half_slice
    n = int(m.sum())
    if n == 0:
        return math.nan, math.nan, 0, SIDES_NONE
    yy = y[m]
    rr = rays[m]

    left = _nearest_cluster(yy[yy > 0.0], rr[yy > 0.0], **kw)
    right = _nearest_cluster(yy[yy < 0.0], rr[yy < 0.0], **kw)

    L = math.nan if left is None else left
    R = math.nan if right is None else -right
    sides = SIDES_NONE | (SIDES_LEFT if left is not None else 0) | (SIDES_RIGHT if right is not None else 0)
    return L, R, n, sides


def _fit_kappa(x0: np.ndarray, c: np.ndarray, valid: np.ndarray) -> tuple[float, bool]:
    """从 ``c(x0)`` 拟合走廊曲率。

    模型 ``c(x) ≈ a0 + a1*x + a2*x**2/2``，取 ``kappa = a2``（小斜率近似，
    即忽略 ``(1 + c'**2)**1.5`` 的分母）。只在 ≥ ``KAPPA_MIN_STATIONS`` 个有效站位、
    且这些站位的 x 跨度 ≥ ``KAPPA_MIN_SPAN`` 时给值，否则 ``kappa_valid=False``。
    正 ``kappa`` = 走廊向左弯。适用范围：直道和缓弯（``|c'|`` 明显小于 1）；
    窄道里两侧交替丢点会让 ``c(x0)`` 跳变，这时必须看 ``kappa_valid`` 和残差。
    """
    sel = np.isfinite(c)
    if int(sel.sum()) < KAPPA_MIN_STATIONS:
        return 0.0, False
    xs = x0[sel]
    if xs.max() - xs.min() < KAPPA_MIN_SPAN:
        return 0.0, False
    coef = np.polyfit(xs, c[sel], 2)
    return float(coef[0]) * 2.0, True


def extract(
    ranges,
    angle_min: float,
    angle_increment: float,
    x0s=DEFAULT_X0,
    *,
    half_slice: float = HALF_SLICE,
    lidar_x: float = LIDAR_X,
    range_min: float = 0.0,
    range_max: float = math.inf,
    self_filter: bool = True,
    min_station_pts: int = MIN_STATION_PTS,
) -> CorridorEstimate:
    """从一帧激光解出走廊。

    ``L``/``R`` 在 ``valid`` 为真时可用；``w``/``c`` 还要求 ``sides == BOTH``
    （只看见一侧时给 ``nan``，不拿单侧外推当走廊）。
    """
    x0s = tuple(float(v) for v in x0s)
    x, y, rays, _ = polar_to_cartesian(
        ranges, angle_min, angle_increment,
        lidar_x=lidar_x, range_min=range_min, range_max=range_max, self_filter=self_filter,
    )

    K = len(x0s)
    L = np.full(K, np.nan)
    R = np.full(K, np.nan)
    npts = np.zeros(K, dtype=np.int32)
    sides = np.zeros(K, dtype=np.int8)
    for k, x0 in enumerate(x0s):
        L[k], R[k], npts[k], sides[k] = _station(x, y, rays, x0, half_slice)

    have_both = np.isfinite(L) & np.isfinite(R)
    w = np.where(have_both, L - R, np.nan)
    c = np.where(have_both, 0.5 * (L + R), np.nan)

    # 站位有效性：切片本身要够厚实，且至少找到一侧
    valid = (npts >= min_station_pts) & (sides != SIDES_NONE)
    w = np.where(valid, w, np.nan)
    c = np.where(valid, c, np.nan)

    kappa, kappa_ok = _fit_kappa(np.asarray(x0s), c, valid)
    return CorridorEstimate(
        x0=x0s, L=L, R=R, w=w, c=c, valid=valid, sides=sides, n_pts=npts,
        kappa=kappa, kappa_valid=kappa_ok, n_returns=int(x.size),
    )


def front_gap(est: CorridorEstimate, nose_ahead: float = NOSE_AHEAD, half_corridor: float = 0.28) -> float:
    """``scan_guard.py`` 口径的前方余量，用来和它的 ``gap()`` 对齐。

    取走廊在 ``x0 >= nose_ahead`` 且 ``|c| <= half_corridor`` 的站位里最小的 ``L``，
    再减掉车头前伸量。没有任何站位落在走廊里时返回 ``inf``。
    """
    xs = np.asarray(est.x0)
    ok = est.valid & (xs >= nose_ahead) & np.isfinite(est.L) & (np.abs(np.nan_to_num(est.c, nan=1e9)) <= half_corridor)
    if not bool(np.any(ok)):
        return math.inf
    return float(np.nanmin(est.L[ok])) - nose_ahead


__all__ = [
    "LIDAR_X", "NOSE_AHEAD", "DEFAULT_X0", "HALF_SLICE",
    "SIDES_NONE", "SIDES_LEFT", "SIDES_RIGHT", "SIDES_BOTH",
    "CorridorEstimate", "parse_ranges", "polar_to_cartesian", "extract", "front_gap",
]