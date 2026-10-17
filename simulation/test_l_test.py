"""evaluation/l_test 的 L 型拐角:外侧是方角(不是半径 = 路宽的大圆弧)。"""
import numpy as np

from evaluation.l_test import FRONT, RIGHT, ell


def test_outer_corner_is_square():
    for open_left in (False, True):
        road = ell(5.0, open_left)
        e = road.elems[1]
        i0 = e['_i0']
        p0 = road.right[i0]
        d0 = np.array([np.cos(road.heading[i0]), np.sin(road.heading[i0])])
        corner = p0 + d0 * (e['r_in'] + e['width'])            # 方角顶点:右墙延长到前墙
        w = road.walls.reshape(-1, 2)
        assert np.hypot(*(w - corner).T).min() < 0.06
        assert len(road.pockets) == 1


def test_geometry_matches_seg06():
    road = ell(5.0)
    e = road.elems[1]
    dist = FRONT - 5.0 - 0.05
    i = e['_i0'] - int(round(dist / 0.05))
    car = road.centre[i] + road.normal[i] * (-2.5 + RIGHT)
    d0 = np.array([np.cos(road.heading[i]), np.sin(road.heading[i])])
    w = road.walls.reshape(-1, 2) - car
    ahead = w[np.abs(w @ road.normal[i]) < 0.1]                  # 正前方那条线上的墙点
    assert abs((ahead @ d0)[ahead @ d0 > 0].min() - FRONT) < 0.1
