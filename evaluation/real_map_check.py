"""实景地图仿真的还原度检查:录包里偶数帧雷达建图、奇数帧验证 —— 在录到的位姿上用这张图打射线,和当时真实扫描逐根比。

    python -m evaluation.real_map_check runs/map_1016/map_1015_185021.npz

录包 npz 由车上 bag_export.py 导出(scan_t / scan_r / a_min / a_inc / odom 等)。建图走车上同一套 Logbook
(射线穿过的格减分,人走过不留墙)。验证帧不参与建图,所以这是"没见过的帧能不能还原"。
报:真实有回波(0.15~8 m)的射线里,仿真距离误差的中位数和 |误差| < 5 / 10 / 20 cm 的比例,按距离分段。
"""
from __future__ import annotations

import argparse

import numpy as np


def poses(od):
    yaw = np.unwrap(od[:, 3])
    return lambda t: (float(np.interp(t, od[:, 0], od[:, 1])), float(np.interp(t, od[:, 0], od[:, 2])),
                      float(np.interp(t, od[:, 0], yaw)))


def build(z, pose_at, idx, ang, p):
    import tempfile
    from common.bev import lidar_points
    from vehicle.bev_logbook import OCC_ON, Logbook
    from simulation.fastsim.real_map import RealMap
    book = Logbook(tempfile.mkdtemp())
    for i in idx:
        x, y, yaw = pose_at(z['scan_t'][i])
        book.add_lidar(lidar_points(z['scan_r'][i], ang, (x, y, yaw), p, 0.0, 10.0),
                       (x + p.lidar_x * np.cos(yaw), y + p.lidar_x * np.sin(yaw)))
    return RealMap(book.lidar.ij(OCC_ON)[0], 0.05)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag_npz')
    a = ap.parse_args(argv)
    from common.params import CarParams
    p = CarParams()
    z = np.load(a.bag_npz)
    od = z['odom']
    pose_at = poses(od)
    ang = z['a_min'] + np.arange(z['scan_r'].shape[1]) * z['a_inc']
    ok_t = (z['scan_t'] >= od[0, 0]) & (z['scan_t'] <= od[-1, 0])
    idx = np.flatnonzero(ok_t)
    world = build(z, pose_at, idx[::2], ang, p)
    err, rng = [], []
    for i in idx[1::2]:
        x, y, yaw = pose_at(z['scan_t'][i])
        real = z['scan_r'][i].astype(np.float64)
        sim = world.raycast(x + p.lidar_x * np.cos(yaw), y + p.lidar_x * np.sin(yaw), ang + yaw, 10.0, p.lidar_min)
        m = np.isfinite(real) & (real > 0.15) & (real < 8.0)
        e = sim[m] - real[m]
        err.append(e)
        rng.append(real[m])
    e, r = np.concatenate(err), np.concatenate(rng)
    fin = np.isfinite(e)
    print(f'验证帧 {len(idx[1::2])},射线 {len(e)};图里没打到东西的 {1 - fin.mean():.1%}')
    for lo, hi in ((0.15, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 8.0), (0.15, 8.0)):
        m = (r >= lo) & (r < hi)
        ae = np.abs(np.where(np.isfinite(e[m]), e[m], 99.0))
        print(f'  真实距离 {lo:.2f}-{hi:.1f} m(n={m.sum()}):误差中位 {np.median(e[m][np.isfinite(e[m])]) * 100:+.1f} cm  '
              f'|误差|<5 cm {np.mean(ae < 0.05):.0%}  <10 cm {np.mean(ae < 0.10):.0%}  <20 cm {np.mean(ae < 0.20):.0%}')


if __name__ == '__main__':
    main()
