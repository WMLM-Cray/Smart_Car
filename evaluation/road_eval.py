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
from simulation.fastsim.road import LEVELS
from training.road_check import sample_episode
from simulation.fastsim.road_env import RoadEnv
from common.road_policy import DEFAULT_ENC, RoadPolicy, spec_of, speed_to_action
from training.road_train import summarize

TEST_SEED0 = 90_000_000


def _schema(path):
    with np.load(path) as z:
        return str(z['schema'])


def _enc(schema):
    return DEFAULT_ENC if schema.startswith(v1.SCHEMA_PREFIX) else spec_of(schema)


def run(args):
    """args = (model, seed, level[, cap_max, robust])。"""
    model, seed, level, cap_max, robust = (*args, None, False)[:5]
    schema = _schema(model)
    # 采样必须用模型自己的速度契约:cap 的分布要和模型能表达的速度范围一致,
    # 否则评估一个 v_hi=3 的模型时它永远看不到高速档,数字没有意义。
    enc = _enc(schema)
    road, pl, start, _, _ = sample_episode(seed, level, enc=enc, cap_max=cap_max)
    env = RoadEnv(road, pl, seed, start, noise=True, robust=robust)
    # v1 的 schema 带训练时速度尺度后缀(low_speed_steering_v1_s2.4 等),按前缀认,
    # 观测用模型自己的 scale,不能用模块默认 1.5
    if schema.startswith(v1.SCHEMA_PREFIX):
        pol, last = v1.SteeringPolicy(model), 0.0
        v_a = speed_to_action(pl.cap)

        def act():
            nonlocal last
            last = pol(v1.observe(env.ranges, env.angles, env.car.v, pl.cap,
                                  env.car.yaw_rate, last, scale=pol.scale))
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
    ap.add_argument('--cap-max', type=float, default=None, help='每局速度上限只抽到这里(课程分档)')
    ap.add_argument('--robust', action='store_true', help='加实车见过的噪声(编码器毛刺、雷达整段缺失)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run, [(a.model, a.seed_start + i, a.level, a.cap_max, a.robust)
                                 for i in range(a.count)]))
    summ = summarize(rows, _enc(_schema(a.model)))
    report = dict(model=a.model, sha256=hashlib.sha256(open(a.model, 'rb').read()).hexdigest(),
                  schema=_schema(a.model), level=a.level, seed_start=a.seed_start,
                  summary=summ, episodes=rows)
    print(json.dumps(dict(model=a.model, level=a.level, **summ), ensure_ascii=False))
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(report, f, ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
