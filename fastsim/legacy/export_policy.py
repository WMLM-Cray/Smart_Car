"""把 DAgger 的 .pt 检查点导成车上用的 .npz(纯 numpy 权重)。

用法:python fastsim/legacy/export_policy.py runs/dagger_s3_w256.pt runs/policy_s3.npz
"""
import sys

sys.path.insert(0, '.')

import numpy as np
import torch

from fastsim.params import CarParams


def main(src, dst):
    ck = torch.load(src, map_location='cpu')
    if ck.get('schema', '') == 'speed_conditioned_v1':
        raise ValueError('54-dimensional multispeed experiments need a dedicated runtime; do not deploy with the legacy exporter')
    sd = ck['state_dict']
    keys = [k[:-7] for k in sd if k.endswith('.weight')]      # Sequential 里的三层 Linear
    assert len(keys) == 3, f'期望三层全连接,实际 {keys}'
    w = {f'w{i}': sd[f'{k}.weight'].numpy() for i, k in enumerate(keys)}
    b = {f'b{i}': sd[f'{k}.bias'].numpy() for i, k in enumerate(keys)}
    obs_dim = int(ck['obs_dim'])
    p = CarParams()
    np.savez(dst, **w, **b, obs_dim=obs_dim, frames=int(ck['frames']),
             # 高速检查点存了 v_max,低速的没存,退回 CarParams 的 1.5
             v_cmd_fwd=float(ck.get('v_max', p.v_cmd_fwd)), v_cmd_rev=float(p.v_cmd_rev),
             front_sectors=obs_dim - 5 - 12)                  # 观测 = 扇区 + 5,后方固定 12 段
    print(f'{src} -> {dst}  观测 {obs_dim} 维 x {ck["frames"]} 帧,'
          f'速度上限 {float(ck.get("v_max", p.v_cmd_fwd))} m/s')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
