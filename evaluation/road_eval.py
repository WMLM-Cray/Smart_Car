"""在路段环境里独立评估一个模型(v2 路段策略,或旧的 v1 转向策略做对照)。

    python -m evaluation.road_eval --model runs/road_v2_a/candidate_040.npz --level L4 --count 500
    python -m evaluation.road_eval --model models/steering_v1/candidate_060.npz --level L3

默认评估种子从 90M 起；这些种子已用于开发诊断，不能宣称为未见最终测试集。
v1 模型只输出转向:速度指令固定为这一局的速度上限,观测里的目标速度也填速度上限。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from common import low_speed_policy as v1
from simulation.road import LEVELS
from training.road_check import sample_episode
from simulation.road_env import RoadEnv
from common.road_policy import RoadPolicy, speed_to_action
from training.road_train import summarize

TEST_SEED0 = 90_000_000


def _schema(path):
    with np.load(path) as z:
        return str(z['schema'])


def run(args):
    model, seed, level = args
    road, pl, start, _, _ = sample_episode(seed, level)
    env = RoadEnv(road, pl, seed, start, noise=True)
    if _schema(model) == v1.SCHEMA:
        pol, last = v1.SteeringPolicy(model), 0.0
        v_a = speed_to_action(pl.cap)

        def act():
            nonlocal last
            last = pol(v1.observe(env.ranges, env.angles, env.car.v, pl.cap,
                                  env.car.yaw_rate, last))
            return [last, v_a]
    else:
        pol = RoadPolicy(model)

        def act():
            return pol(env.observation())
    why = None
    while why is None:
        why = env.step(act())
    return dict(seed=seed, level=level, reason=why, cap=pl.cap, min_clear=float(env.min_clear),
                time=env.t, elems=[e['kind'] for e in road.elems])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', required=True)
    ap.add_argument('--level', default='L4', choices=[l.name for l in LEVELS])
    ap.add_argument('--count', type=int, default=500)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--seed-start', type=int, default=TEST_SEED0)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run, [(a.model, a.seed_start + i, a.level) for i in range(a.count)]))
    summ = summarize(rows)
    report = dict(model=a.model, sha256=hashlib.sha256(open(a.model, 'rb').read()).hexdigest(),
                  schema=_schema(a.model), level=a.level, seed_start=a.seed_start,
                  summary=summ, episodes=rows)
    print(json.dumps(dict(model=a.model, level=a.level, **summ), ensure_ascii=False))
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(report, f, ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
