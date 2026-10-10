"""地面投影:和标定脚本同一约定、查表与正投影互逆、畸变来回一致。"""
import math

import numpy as np

from common.ground_proj import GroundLut, distort, ground_to_pixel, undistort
from common.params import CarParams

P = CarParams()


def _calib_proj(bx, by):
    """标定时用的投影(runs/calib/wallfit.py 的 proj),逐行照抄当参照。"""
    Pp, Y, R, H, CX0 = P.camera_pitch, P.camera_yaw, P.camera_roll, P.camera_z, P.camera_x
    dF, dL, dU = bx - CX0, by, -H
    F1, L1 = dF * math.cos(-Y) - dL * math.sin(-Y), dF * math.sin(-Y) + dL * math.cos(-Y)
    Fc, Uc = F1 * math.cos(-Pp) + dU * math.sin(-Pp), dU * math.cos(-Pp) - F1 * math.sin(-Pp)
    L2, U2 = L1 * math.cos(R) - Uc * math.sin(R), L1 * math.sin(R) + Uc * math.cos(R)
    xd, yd = distort(-L2 / Fc, -U2 / Fc, P.camera_dist)
    fx, fy, cx, cy = P.camera_K
    return cx + fx * xd, cy + fy * yd


def test_projection_matches_calibration_convention():
    pts = np.array([[1.0, 0.0], [1.5, 0.4], [2.5, -0.8], [3.0, 1.0]])
    uv = ground_to_pixel(pts)
    ref = np.array([_calib_proj(*p) for p in pts])
    assert np.allclose(uv, ref, atol=1e-6)


def test_undistort_inverts_distort():
    rng = np.random.default_rng(0)
    xn, yn = rng.uniform(-0.6, 0.6, 200), rng.uniform(-0.45, 0.45, 200)
    xd, yd = distort(xn, yn, P.camera_dist)
    x2, y2 = undistort(xd, yd, P.camera_dist)
    assert np.allclose(x2, xn, atol=1e-6) and np.allclose(y2, yn, atol=1e-6)


def test_lut_back_projects_cell_centres_onto_ground():
    """查表得到的地面点再正投影回图像,应落回该格中心。"""
    lut = GroundLut(128, 128)
    W, H = P.camera_size
    i = np.flatnonzero(lut.valid)
    assert len(i) > 2000
    uv = ground_to_pixel(lut.xy[i])
    r, c = np.divmod(i, 128)
    want = np.c_[(c + 0.5) * W / 128, (r + 0.5) * H / 128]
    assert np.abs(uv - want).max() < 0.05
    d = np.hypot(lut.xy[i, 0] - P.camera_x, lut.xy[i, 1])
    assert d.max() <= 3.0 + 1e-9 and lut.xy[i, 0].min() > P.camera_x       # 只在车前、3 m 内


def test_points_dedupe_to_grid():
    lut = GroundLut(128, 128)
    pts = lut.points(np.ones((128, 128), bool))
    assert len(pts) > 500
    assert np.allclose(pts / 0.05, np.round(pts / 0.05))
    assert len(np.unique(pts, axis=0)) == len(pts)
    assert len(lut.points(np.zeros((128, 128), bool))) == 0
