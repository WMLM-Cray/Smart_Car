"""bev_v1 网络的 torch 版(训练用)和导出;结构定义见 common/bev_policy.py。"""
from __future__ import annotations

import numpy as np

from common.bev_policy import C, CONVS, H, HIDDEN, N_SCALAR, SCHEMA, W
from common.road_policy import DEFAULT_ENC, Encoding, suffix_of


def make_net():
    import torch
    from torch import nn

    class BevNet(nn.Module):
        def __init__(self):
            super().__init__()
            self.convs = nn.ModuleList(nn.Conv2d(ci, co, k, stride=2, padding=k // 2) for ci, co, k in CONVS)
            flat = CONVS[-1][1] * (H // 16) * (W // 16)
            self.f0 = nn.Linear(flat + N_SCALAR, HIDDEN)
            self.f1 = nn.Linear(HIDDEN, HIDDEN)
            self.f2 = nn.Linear(HIDDEN, 2)

        def forward(self, img, sc):
            x = img
            for conv in self.convs:
                x = torch.relu(conv(x))
            h = torch.cat([x.flatten(1), sc], 1)
            h = torch.tanh(self.f1(torch.tanh(self.f0(h))))
            return self.f2(h)                       # 训练时不裁;推理/导出后裁到 ±1

    return BevNet()


def weights_of(net) -> dict:
    out = {}
    for k, conv in enumerate(net.convs):
        out[f'c{k}w'] = conv.weight.detach().numpy().copy()
        out[f'c{k}b'] = conv.bias.detach().numpy().copy()
    for k, fc in enumerate((net.f0, net.f1, net.f2)):
        out[f'f{k}w'] = fc.weight.detach().numpy().copy()
        out[f'f{k}b'] = fc.bias.detach().numpy().copy()
    return out


def export(net, path, enc: Encoding = DEFAULT_ENC):
    np.savez(path, schema=np.array(SCHEMA + suffix_of(enc)), **weights_of(net))


def load_into(net, path):
    import torch
    with np.load(path) as z:
        with torch.no_grad():
            for k, conv in enumerate(net.convs):
                conv.weight.copy_(torch.from_numpy(z[f'c{k}w']))
                conv.bias.copy_(torch.from_numpy(z[f'c{k}b']))
            for k, fc in enumerate((net.f0, net.f1, net.f2)):
                fc.weight.copy_(torch.from_numpy(z[f'f{k}w']))
                fc.bias.copy_(torch.from_numpy(z[f'f{k}b']))
