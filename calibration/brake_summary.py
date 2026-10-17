"""刹车实测(docs/实测/2026-10-12-brake)逐次汇总:从发刹车 PWM 那一拍起,多久、多远降到 0.15 m/s,
稳定段减速度,起效滞后,回中位后倒退了多少。

    python -m calibration.brake_summary [docs/实测/2026-10-12-brake]

文件名 bNN_v<目标车速>_pwm<刹车 PWM>.csv;列 t,pwm,turn,v,x,y,yaw,ax..wz,gap,phase(dynamics_test 录的,50 Hz)。
phase:accel 加速到目标车速、brake 给刹车 PWM、coast 车速 <= 0.15 后回中位、stop 脚本被中断后补发的中位。
"""
from __future__ import annotations

import csv
import glob
import os
import sys

import numpy as np

STOP_V = 0.15           # 降到这个车速算刹住(dynamics_test 在这里回中位)
LAG_DV = 0.05           # 车速比发刹车时低这么多算开始起效


def load(path):
    with open(path, encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r['t']) for r in rows])
    v = np.array([float(r['v']) for r in rows])
    pwm = np.array([int(r['pwm']) for r in rows])
    phase = np.array([r['phase'] for r in rows])
    return t, v, pwm, phase


def summarize(path):
    t, v, pwm, phase = load(path)
    k0 = int(np.flatnonzero(phase == 'brake')[0])
    t, v, pwm, phase = t[k0:] - t[k0], v[k0:], pwm[k0:], phase[k0:]
    v0 = float(v[0])
    stop = np.flatnonzero(v <= STOP_V)
    ks = int(stop[0]) if len(stop) else len(v) - 1
    lag = float(t[np.flatnonzero(v < v0 - LAG_DV)[0]])
    steady = (t >= lag) & (v > 0.3) & (np.arange(len(v)) <= ks)       # 起效后到 0.3 m/s 的直线拟合
    a = -float(np.polyfit(t[steady], v[steady], 1)[0]) if steady.sum() >= 5 else float('nan')
    d = float(np.sum((v[1:ks + 1] + v[:ks]) / 2 * np.diff(t[:ks + 1])))
    after = v[ks:]
    back = float(-np.sum(np.minimum(after[:-1], 0) * np.diff(t[ks:])))
    return dict(name=os.path.basename(path)[:3], pwm=int(pwm[0]), v0=v0, lag=lag, t_stop=float(t[ks]), d_stop=d,
                a=a, v_min=float(after.min()), back=back, aborted=bool((phase == 'stop').any()))


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    folder = argv[0] if argv else os.path.join(os.path.dirname(__file__), '..', 'docs', '实测', '2026-10-12-brake')
    print('次   PWM   v0    滞后s  到0.15用s  距离m  稳定段减速m/s²  之后最低v  倒退m')
    for p in sorted(glob.glob(os.path.join(folder, 'b*.csv'))):
        r = summarize(p)
        print(f"{r['name']}  {r['pwm']}  {r['v0']:.2f}  {r['lag']:.2f}   {r['t_stop']:.2f}       {r['d_stop']:.2f}   "
              f"{r['a']:.2f}           {r['v_min']:+.2f}     {r['back']:.2f}{'  (脚本中途被打断)' if r['aborted'] else ''}")


if __name__ == '__main__':
    main()
