"""达标判据:这个模型最高能在哪个速度上保持 >=95% 的干净率?

目标(/goal)=「干净率 >=95% 且接近理论赛道速度上限」。两个量分别怎么量:

- **干净率**:`eval_speed.run` 在固定 cap 下的出圈率,分速度档报。
- **理论上限**:老师的实测值 —— L5c 上 cap 3.0 时 118/120、均速 2.177 m/s,
  且 cap 3.2 均速一模一样(饱和),所以 2.177 就是这条赛道族的实测天花板。

判据:找出**最高的那个速度**,使干净率 >= --target(默认 0.95)。
再报它相对老师天花板的百分比。

    python3 -m evaluation.goal_check --model runs/spica_s3_A/candidate_020.npz
    python3 -m evaluation.goal_check --model X.npz --speeds 1.0,1.5,2.0,2.5,3.0 --count 150
"""
import os
import sys
from pathlib import Path

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import argparse
import json
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from evaluation.eval_speed import run, wilson

# 老师实测:cap 3.0/3.2 都是均速 2.177,是这条赛道族的速度天花板(饱和,再加 cap
# 也不涨)。干净率判据不看它,只用来算"接近上限"的比例。
TEACHER_CEILING = 2.177


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--level', default='L5c')
    ap.add_argument('--speeds', default='0.8,1.2,1.6,2.0,2.5,3.0')
    ap.add_argument('--count', type=int, default=150)
    ap.add_argument('--workers', type=int, default=10)
    ap.add_argument('--target', type=float, default=0.95)
    ap.add_argument('--seed-start', type=int, default=71_000_000)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()

    speeds = [float(x) for x in a.speeds.split(',')]
    jobs = [(a.model, a.seed_start + i, a.level, s)
            for s in speeds for i in range(a.count)]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run, jobs))

    print('模型 %s   赛道 %s   n=%d/档' % (a.model, a.level, a.count))
    print('%-8s %-9s %-18s %-9s %s' % ('cap', '干净率', 'Wilson 95%', '均速', '失败原因'))
    table = []
    for j, s in enumerate(speeds):
        sub = rows[j * a.count:(j + 1) * a.count]
        ok = np.array([r['reason'] == 'goal' for r in sub])
        sp = np.mean([r['mean_speed'] for r in sub])
        why = {k: sum(r['reason'] == k for r in sub) for k in sorted({r['reason'] for r in sub})}
        lo, hi = wilson(ok.sum(), len(ok))
        table.append(dict(speed=s, clean=float(ok.mean()), lo=lo, hi=hi,
                          mean_speed=float(sp), reasons=why, n=len(sub)))
        print('%-8.2f %-9.3f %.3f~%.3f       %-9.3f %s'
              % (s, ok.mean(), lo, hi, sp, why))

    # 达标线:取干净率下界(Wilson 下限)也 >= target 的最高速度 —— 比点估计严格
    good = [t for t in table if t['lo'] >= a.target]
    print()
    if good:
        best = max(good, key=lambda t: t['speed'])
        print('★ 达标:cap %.2f 上干净率 %.3f(Wilson 下界 %.3f >= %.2f),均速 %.3f'
              % (best['speed'], best['clean'], best['lo'], a.target, best['mean_speed']))
        print('  相对老师天花板 %.3f m/s 的均速达成 %.1f%%'
              % (TEACHER_CEILING, 100 * best['mean_speed'] / TEACHER_CEILING))
    else:
        top = max(table, key=lambda t: t['clean'])
        print('✗ 未达标:没有任何速度档的 Wilson 下界 >= %.2f' % a.target)
        print('  最好的是 cap %.2f 干净率 %.3f(下界 %.3f)'
              % (top['speed'], top['clean'], top['lo']))

    if a.out:
        json.dump(dict(model=a.model, level=a.level, target=a.target,
                       ceiling=TEACHER_CEILING, table=table),
                  open(a.out, 'w'), indent=1)
        print('写出', a.out)


if __name__ == '__main__':
    main()
