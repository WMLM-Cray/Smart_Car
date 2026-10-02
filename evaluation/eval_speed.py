"""测一个模型在**固定目标速度**下的闭环干净率。

现有的 evaluation/road_eval.py 是"按模型 schema 的 enc 采样一局"——对 v1 模型
它按 DEFAULT_ENC(scale 1.5)采样,所以测不了 3.0。这个脚本把 cap 钉死,
直接回答"这个模型在 3.0 m/s 上干净率是多少"。

    python3 eval_speed.py --model <npz> --speed 3.0 --level L5c --count 150

报:总干净率 + 失败原因分布 + 净空 p10 + 均速。
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
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import numpy as np

from common import low_speed_policy as v1
from common.road_policy import DEFAULT_ENC, Encoding, RoadPolicy, spec_of, speed_to_action
from simulation.fastsim.road import LEVELS
from simulation.fastsim.road_env import Plant, RoadEnv
from training.road_check import sample_episode

SEED0 = 71_000_000


def _schema(path):
    with np.load(path) as z:
        return str(z['schema'])


def run(args):
    model, seed, level, speed = args
    schema = _schema(model)
    is_v1 = schema.startswith(v1.SCHEMA_PREFIX)
    # 观测/动作契约必须覆盖目标速度,否则 cap=3.0 会被观测 clip 成 1.0,且
    # speed_to_action(3.0) 会饱和到 1.0 —— 测出来的就不是 3.0 了。
    enc = Encoding(scale=max(speed, 1.5), v_lo=0.45, v_hi=max(speed, 1.0))
    if not is_v1:
        enc = spec_of(schema)          # v2/spica 用模型自己的契约
    road, pl, start, _, _ = sample_episode(seed, level, enc=enc)
    # 把这一局的 cap / v_top 钉到目标速度。注意 pl.enc 就是 enc,action_to_speed
    # 用的也是它,两者必须一致 —— 否则 v_a 解出来的物理速度不是 speed。
    pl = replace(pl, cap=speed, v_top=max(pl.v_top, speed), enc=enc)
    env = RoadEnv(road, pl, seed, start, noise=True)
    if is_v1:
        pol, last = v1.SteeringPolicy(model), 0.0
        v_a = speed_to_action(speed, enc)

        def act():
            nonlocal last
            last = pol(v1.observe(env.ranges, env.angles, env.car.v, speed,
                                  env.car.yaw_rate, last, scale=pol.scale))
            return [last, v_a]
    else:
        pol = RoadPolicy(model)

        def act():
            return pol(env.observation())
    speeds, why = [], None
    while why is None:
        why = env.step(act())
        speeds.append(env.car.v)
    return dict(seed=seed, level=level, reason=why, cap=speed,
                min_clear=float(env.min_clear), time=env.t,
                mean_speed=float(np.mean(speeds)) if speeds else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--speed', type=float, required=True)
    ap.add_argument('--level', default='L5c')
    ap.add_argument('--count', type=int, default=150)
    ap.add_argument('--workers', type=int, default=10)
    ap.add_argument('--seed-start', type=int, default=SEED0)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()

    jobs = [(a.model, a.seed_start + i, a.level, a.speed) for i in range(a.count)]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run, jobs))
    ok = np.array([r['reason'] == 'goal' for r in rows])
    clr = np.array([r['min_clear'] for r in rows])
    spd = np.array([r['mean_speed'] for r in rows])
    reasons = {k: sum(r['reason'] == k for r in rows) for k in sorted({r['reason'] for r in rows})}
    print('model=%s  speed=%.2f  level=%s  n=%d' % (a.model, a.speed, a.level, a.count))
    print('  干净率 = %.3f  (Wilson 95%%: %.3f~%.3f)'
          % (ok.mean(), *wilson(ok.sum(), len(ok))))
    print('  失败原因 %s' % reasons)
    print('  净空 p10=%.3f  中位=%.3f' % (np.percentile(clr, 10), np.median(clr)))
    print('  均速 %.3f  (目标 %.2f, 达成 %.1f%%)'
          % (spd.mean(), a.speed, 100 * spd.mean() / a.speed))
    if a.out:
        import json
        json.dump(dict(model=a.model, speed=a.speed, level=a.level, n=a.count,
                       clean=float(ok.mean()), reasons=reasons,
                       clear_p10=float(np.percentile(clr, 10)),
                       clear_med=float(np.median(clr)),
                       mean_speed=float(spd.mean()), episodes=rows),
                  open(a.out, 'w'), indent=1)


def wilson(k, n):
    if n == 0:
        return (0.0, 1.0)
    z, p = 1.96, k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - h) / d, (c + h) / d)


if __name__ == '__main__':
    main()
