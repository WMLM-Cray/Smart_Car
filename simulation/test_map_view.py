"""simulation/fastsim/map_view:x 朝上、y 朝左,和车上 Logbook.render 一致。"""
import numpy as np

from simulation.fastsim.map_view import MapView


def test_orientation_and_size():
    v = MapView((0.0, 0.0, 2.0, 1.0), cell=0.05, px=5)
    (u0, v0), (u1, v1), (u2, v2) = v.uv([(0.0, 0.0), (2.0, 0.0), (0.0, 1.0)])
    assert v1 < v0 and u1 == u0                 # x 增大 -> 往上
    assert u2 < u0 and v2 == v0                 # y 增大 -> 往左
    assert (v.h, v.w) == (41 * 5, 21 * 5)


def test_base_draws_cells():
    import cv2
    v = MapView((0.0, 0.0, 1.0, 1.0))
    img = v.base(cv2, occ_ij=[(10, 10)], free_ij=[(5, 5)])
    u, w = v.uv([(0.5, 0.5)])[0]
    assert (img[w - 2:w + 3, u - 2:u + 3] == 30).all(axis=2).any()
    assert img.shape == (v.h, v.w, 3)
