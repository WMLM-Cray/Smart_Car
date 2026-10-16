"""录包(车上 bag_export 导出的 npz)回放进车上同一套 Logbook 建图:雷达 + 地面分割,出 seg.npz(RealMap.from_seg 能读)和点阵 map.png。

    python -m evaluation.bag_map runs/map_1016/map_1015_185021.npz --match --out runs/map_1016/matched

--match:位姿先用扫描匹配(simulation/fastsim/scan_match)修正;不加就是里程计(和车上建图一样)。
地面分割帧的位姿按时间在相邻两帧雷达的位姿之间插值(匹配只修正雷达帧)。
"""
from __future__ import annotations

import argparse
import os

import numpy as np


def scan_poses(z, p, match=False):
    """每帧雷达的位姿 (n, 3)(里程计时间范围外的帧是 nan)。"""
    od = z['odom']
    yaw = np.unwrap(od[:, 3])
    t = z['scan_t']
    P = np.stack([np.interp(t, od[:, 0], od[:, 1]), np.interp(t, od[:, 0], od[:, 2]), np.interp(t, od[:, 0], yaw)], 1)
    ok = (t >= od[0, 0]) & (t <= od[-1, 0])
    P[~ok] = np.nan
    if match:
        from common.bev import lidar_points
        from simulation.fastsim.scan_match import match_trajectory
        ang = z['a_min'] + np.arange(z['scan_r'].shape[1]) * z['a_inc']
        idx = np.flatnonzero(ok)
        body = [lidar_points(z['scan_r'][i], ang, (0.0, 0.0, 0.0), p, 0.0, 8.0)[::2] for i in idx]
        P[idx], pairs = match_trajectory(body, P[idx])
        print(f'扫描匹配:配对点中位 {np.median(pairs):.0f},退回里程计的帧 {np.mean(pairs < 80):.1%}')
    return P


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag_npz')
    ap.add_argument('--out', required=True)
    ap.add_argument('--match', action='store_true')
    a = ap.parse_args(argv)
    import cv2
    from common.bev import lidar_points
    from common.params import CarParams
    from vehicle.bev_logbook import Logbook
    p = CarParams()
    z = dict(np.load(a.bag_npz))            # 一次读进内存:npz 每次 z[key] 都重新解压整个数组
    P = scan_poses(z, p, a.match)
    ang = z['a_min'] + np.arange(z['scan_r'].shape[1]) * z['a_inc']
    ok = np.isfinite(P[:, 0])
    st, Pk = z['scan_t'][ok], P[ok]
    pose_at = lambda t: tuple(np.interp(t, st, Pk[:, c]) for c in range(3))
    book = Logbook(a.out)
    offs = np.r_[0, np.cumsum(z['gf_n'])]
    events = sorted([(t, 0, i) for i in np.flatnonzero(ok) for t in [z['scan_t'][i]]] +
                    [(t, 1, i) for i, t in enumerate(z['gf_t']) if st[0] <= t <= st[-1]])
    for t, kind, i in events:
        if kind == 0:
            x, y, yaw = P[i]
            book.add_lidar(lidar_points(z['scan_r'][i], ang, (x, y, yaw), p, 0.0, 10.0),
                           (x + p.lidar_x * np.cos(yaw), y + p.lidar_x * np.sin(yaw)))
            book.add_pose(t, (x, y, yaw), float('nan'), 0.0, 0.0)
        else:
            book.add_free(z['gf_xy'][offs[i]:offs[i + 1]], pose_at(t))
    book.save()
    img = book.render(cv2, px=5, dot=3)
    cv2.imwrite(os.path.join(book.dir(), 'map.png'), img)
    print('->', book.dir())


if __name__ == '__main__':
    main()
