"""随机双边锥桶环形赛道。左侧红、右侧蓝(逆时针行驶)。"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .params import TrackParams


def _resample_closed(points: np.ndarray, spacing: float) -> np.ndarray:
    closed = np.vstack([points, points[:1]])
    seg = np.hypot(*np.diff(closed, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    n = max(8, int(s[-1] / spacing))
    t = np.linspace(0, s[-1], n, endpoint=False)
    return np.stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])], axis=1)


def _curvature_radius(points: np.ndarray) -> np.ndarray:
    prev, nxt = np.roll(points, 1, axis=0), np.roll(points, -1, axis=0)
    a, b, c = np.hypot(*(points - prev).T), np.hypot(*(nxt - points).T), np.hypot(*(nxt - prev).T)
    cross = np.abs((points - prev)[:, 0] * (nxt - points)[:, 1] - (points - prev)[:, 1] * (nxt - points)[:, 0])
    with np.errstate(divide="ignore"):
        return np.where(cross > 1e-12, a * b * c / (2 * cross), np.inf)


@dataclass
class Track:
    centerline: np.ndarray      # (n,2) 逆时针,约 5cm 一个点
    left: np.ndarray            # (m,2) 左侧锥桶(红)
    right: np.ndarray           # (k,2) 右侧锥桶(蓝)
    params: TrackParams

    @property
    def cones(self) -> np.ndarray:
        return np.vstack([self.left, self.right])

    @property
    def length(self) -> float:
        return float(np.hypot(*np.diff(np.vstack([self.centerline, self.centerline[:1]]), axis=0).T).sum())

    def project(self, xy) -> tuple[float, float, float]:
        """点投影到中心线:返回 (弧长 s, 横向偏移(左正), 切向角)。"""
        pts = self.centerline
        nxt = np.roll(pts, -1, axis=0)
        seg = nxt - pts
        seg_len2 = (seg ** 2).sum(axis=1)
        t = np.clip(((np.asarray(xy) - pts) * seg).sum(axis=1) / seg_len2, 0, 1)
        proj = pts + seg * t[:, None]
        i = int(np.argmin(np.hypot(*(proj - xy).T)))
        cum = np.concatenate([[0], np.cumsum(np.sqrt(seg_len2))])
        s = cum[i] + t[i] * math.sqrt(seg_len2[i])
        heading = math.atan2(seg[i, 1], seg[i, 0])
        d = np.asarray(xy) - proj[i]
        lateral = -d[0] * math.sin(heading) + d[1] * math.cos(heading)
        return float(s), float(lateral), heading


def make_track(rng: np.random.Generator, params: TrackParams = TrackParams(),
               attempts: int = 200) -> Track:
    half = params.width / 2
    for _ in range(attempts):
        theta = np.linspace(0, 2 * math.pi, 720, endpoint=False)
        radius = np.full_like(theta, params.mean_radius)
        for k in (2, 3, 4):
            radius += (params.mean_radius * params.shape_noise / k
                       * rng.uniform(-1, 1) * np.cos(k * theta + rng.uniform(0, 2 * math.pi)))
        raw = np.stack([radius * np.cos(theta), radius * np.sin(theta)], axis=1)
        centre = _resample_closed(raw, 0.05)
        if _curvature_radius(_resample_closed(centre, 0.5)).min() < params.min_curvature_radius:
            continue
        tangent = np.roll(centre, -1, axis=0) - np.roll(centre, 1, axis=0)
        tangent /= np.hypot(*tangent.T)[:, None]
        normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)   # 左法向
        left = _resample_closed(centre + half * normal, params.cone_spacing)
        right = _resample_closed(centre - half * normal, params.cone_spacing)
        return Track(centre, left, right, params)
    raise RuntimeError("无法生成满足最小曲率半径的赛道,放宽 shape_noise 或 min_curvature_radius")
