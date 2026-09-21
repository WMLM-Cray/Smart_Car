"""Larger development evaluation, isolated from training seeds and replay.

python -m fastsim.low_speed_eval --model runs/low_speed_a/candidate_020.npz
"""
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import torch

from .low_speed_policy import SCHEMA
from .low_speed_train import episode, make_net

_net = None


def load_model(path):
    net = make_net()
    with np.load(path) as z, torch.no_grad():
        if str(z['schema']) != SCHEMA:
            raise ValueError('Wrong policy schema')
        for i, index in enumerate((0, 2, 4)):
            net[index].weight.copy_(torch.from_numpy(z[f'w{i}']))
            net[index].bias.copy_(torch.from_numpy(z[f'b{i}']))
    return net.eval()


def init_worker(path):
    global _net
    torch.set_num_threads(1)
    _net = load_model(path)


def run(seed):
    row = episode(_net, seed, .6)[2]
    if row['reason'] != 'lap':
        row['teacher_check'] = episode(None, seed, .6)[2]
    return row


def summarize(rows):
    if not rows:
        return {}
    n = len(rows)
    k = sum(r['reason'] == 'lap' and r['cone_hits'] == 0 for r in rows)
    p, z = k / n, 1.96
    denominator = 1 + z*z/n
    centre = (p + z*z/(2*n)) / denominator
    radius = z * math.sqrt(p*(1-p)/n + z*z/(4*n*n)) / denominator
    return dict(episodes=n, clean_laps=k, clean_rate=p,
                wilson_95=[centre-radius, centre+radius],
                reasons=dict(Counter(r['reason'] for r in rows)),
                teacher_failed_on_student_failures=sum(
                    r.get('teacher_check', {}).get('reason', 'lap') != 'lap' for r in rows))


def main():
    ap = argparse.ArgumentParser(__doc__)
    ap.add_argument('--model', type=Path, default=Path('runs/low_speed_a/candidate_020.npz'))
    ap.add_argument('--out', type=Path, default=Path('runs/low_speed_a/validation_200.json'))
    ap.add_argument('--count', type=int, default=200)
    ap.add_argument('--workers', type=int, default=4)
    args = ap.parse_args()
    if args.count < 1 or args.workers < 1 or args.count > 100000:
        ap.error('Invalid count or workers')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    if args.out.exists():
        ap.error('Report already exists; select a fresh output path')
    digest = hashlib.sha256(args.model.read_bytes()).hexdigest()
    started = time.monotonic()
    rows = []
    with ProcessPoolExecutor(max_workers=args.workers, initializer=init_worker,
                             initargs=(str(args.model.resolve()),)) as pool:
        jobs = [pool.submit(run, seed) for seed in range(30000000, 30000000+args.count)]
        for job in as_completed(jobs):
            rows.append(job.result())
            rows.sort(key=lambda r: r['seed'])
            report = dict(model=str(args.model), sha256=digest, target_speed=.6,
                          requested_episodes=args.count, complete=len(rows) == args.count,
                          elapsed_seconds=time.monotonic()-started,
                          summary=summarize(rows),
                          original_50=summarize([r for r in rows if r['seed'] < 30000050]),
                          new_seeds=summarize([r for r in rows if r['seed'] >= 30000050]),
                          episodes=rows)
            tmp = args.out.with_suffix('.tmp')
            tmp.write_text(json.dumps(report, indent=2), encoding='utf-8')
            os.replace(tmp, args.out)
            if len(rows) % 10 == 0 or report['complete']:
                print(json.dumps({k: v for k, v in report.items() if k != 'episodes'}), flush=True)


if __name__ == '__main__':
    main()
