# 训练集 1600 局评测(与 road_td3 内评同口径:全动作、cap≤cap_max 随机、7 级赛道)
# 用法: python3 -m scripts.eval_trainset --model <npz> [--count 1600] [--cap-max 2.4]
import argparse
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import torch

sys.path.insert(0, '.')
from common.road_policy import DIM, spec_of
from training.road_teacher import A_LAT
from training.road_train import SELECT_SEED0, _job, summarize, weights_of


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--count', type=int, default=1600)
    ap.add_argument('--cap-max', type=float, default=2.4)
    ap.add_argument('--levels', default='L5,L5s,L6h,L6f,L4,L6,L3')
    ap.add_argument('--workers', type=int, default=8)
    a = ap.parse_args()

    D = DIM * 3  # FRAMES=3,与 road_td3 一致
    from common.road_policy import FRAMES
    D = DIM * FRAMES
    actor = torch.nn.Sequential(
        torch.nn.Linear(D, 256), torch.nn.Tanh(),
        torch.nn.Linear(256, 256), torch.nn.Tanh(),
        torch.nn.Linear(256, 2))
    with np.load(a.model) as z:
        enc = spec_of(str(z['schema']))
        with torch.no_grad():
            for i, k in enumerate((0, 2, 4)):
                actor[k].weight.copy_(torch.from_numpy(z[f'w{i}']))
                actor[k].bias.copy_(torch.from_numpy(z[f'b{i}']))
    w = weights_of(actor)
    levels = a.levels.split(',')
    jobs = [(SELECT_SEED0 + i, levels[i % len(levels)], w, 0.0, False, enc, A_LAT, a.cap_max)
            for i in range(a.count)]
    with ProcessPoolExecutor(a.workers) as ex:
        ev = [e for e in ex.map(_job, jobs) if e is not None]
    summ = summarize(ev, enc)
    by_level = {nm: summarize([e for e in ev if e['level'] == nm], enc)['success'] for nm in levels}
    print(f'model={a.model} n={len(ev)} success={summ["success"]:.4f}')
    print('by_level:', {k: round(v, 4) for k, v in by_level.items()})


if __name__ == '__main__':
    main()
