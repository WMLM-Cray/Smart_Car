"""bev_v1/v2 与 bev_v2.5 的 torch 网络、导出和权重迁移。"""
from __future__ import annotations

import numpy as np

from common.bev_policy import (C, C2, CONVS, H, HIDDEN, N_SCALAR, NEAR_CONVS,
                               NEAR_SIZE, SCHEMA, W, SCHEMA_V25, channels_of, schema_prefix)
from common.road_policy import DEFAULT_ENC, Encoding, suffix_of


def make_net(c_in: int = C, near: bool = False):
    import torch
    from torch import nn
    if near and c_in != C2:
        raise ValueError('bev_v2.5 的主图和近场都需要 6 个通道')

    class BevNet(nn.Module):
        def __init__(self):
            super().__init__()
            convs = ((c_in,) + CONVS[0][1:],) + CONVS[1:]
            self.convs = nn.ModuleList(nn.Conv2d(ci, co, k, stride=2, padding=k // 2)
                                       for ci, co, k in convs)
            self.near_convs = nn.ModuleList(
                nn.Conv2d(ci, co, k, stride=2, padding=k // 2) for ci, co, k in NEAR_CONVS
            ) if near else nn.ModuleList()
            main_flat = CONVS[-1][1] * (H // 16) * (W // 16)
            near_flat = NEAR_CONVS[-1][1] * (NEAR_SIZE // 16) ** 2 if near else 0
            self.f0 = nn.Linear(main_flat + near_flat + N_SCALAR, HIDDEN)
            self.f1 = nn.Linear(HIDDEN, HIDDEN)
            self.f2 = nn.Linear(HIDDEN, 2)

        def forward(self, img, sc, near_img=None):
            x = img
            for conv in self.convs:
                x = torch.relu(conv(x))
            features = [x.flatten(1)]
            if self.near_convs:
                if near_img is None:
                    raise ValueError('bev_v2.5 训练前向需要近场图')
                nx = near_img
                for conv in self.near_convs:
                    nx = torch.relu(conv(nx))
                features.append(nx.flatten(1))
            elif near_img is not None:
                raise ValueError('bev_v1/v2 网络不接受近场图')
            features.append(sc)
            h = torch.cat(features, 1)
            h = torch.tanh(self.f1(torch.tanh(self.f0(h))))
            return self.f2(h)                   # 训练时不裁;导出推理时裁到 ±1

    return BevNet()


def weights_of(net) -> dict:
    out = {}
    for k, conv in enumerate(net.convs):
        out[f'c{k}w'] = conv.weight.detach().numpy().copy()
        out[f'c{k}b'] = conv.bias.detach().numpy().copy()
    for k, conv in enumerate(net.near_convs):
        out[f'n{k}w'] = conv.weight.detach().numpy().copy()
        out[f'n{k}b'] = conv.bias.detach().numpy().copy()
    for k, fc in enumerate((net.f0, net.f1, net.f2)):
        out[f'f{k}w'] = fc.weight.detach().numpy().copy()
        out[f'f{k}b'] = fc.bias.detach().numpy().copy()
    return out


def export(net, path, enc: Encoding = DEFAULT_ENC, schema: str = SCHEMA):
    if (schema_prefix(schema) == SCHEMA_V25) != bool(net.near_convs):
        raise ValueError('bev_v2.5 schema 与网络近场分支不匹配')
    if channels_of(schema) != net.convs[0].in_channels:
        raise ValueError('schema 与主图输入通道数不匹配')
    np.savez(path, schema=np.array(schema + suffix_of(enc)), **weights_of(net))


def load_into(net, path):
    """兼容旧模型热启动;v1/v2 -> v2.5 时保留主图/标量权重并零初始化近场 FC 接口。"""
    import torch
    target_near = bool(net.near_convs)
    main_flat = CONVS[-1][1] * (H // 16) * (W // 16)
    with np.load(path) as z:
        source_near = schema_prefix(str(z['schema'])) == SCHEMA_V25
        if source_near != ('n0w' in z.files):
            raise ValueError('源模型 schema 与近场权重不匹配')
        if source_near and not target_near:
            raise ValueError('不能将 bev_v2.5 权重加载到没有近场分支的网络')
        with torch.no_grad():
            for k, conv in enumerate(net.convs):
                w = torch.from_numpy(z[f'c{k}w'])
                if w.shape[1] < conv.weight.shape[1]:
                    conv.weight.zero_()
                    conv.weight[:, :w.shape[1]].copy_(w)
                elif w.shape == conv.weight.shape:
                    conv.weight.copy_(w)
                else:
                    raise ValueError(f'主图卷积 c{k} 权重形状不兼容: {tuple(w.shape)} -> {tuple(conv.weight.shape)}')
                conv.bias.copy_(torch.from_numpy(z[f'c{k}b']))
            if target_near and source_near:
                for k, conv in enumerate(net.near_convs):
                    conv.weight.copy_(torch.from_numpy(z[f'n{k}w']))
                    conv.bias.copy_(torch.from_numpy(z[f'n{k}b']))
            for k, fc in enumerate((net.f0, net.f1, net.f2)):
                w, b = torch.from_numpy(z[f'f{k}w']), torch.from_numpy(z[f'f{k}b'])
                if w.shape == fc.weight.shape:
                    fc.weight.copy_(w)
                elif k == 0 and target_near and not source_near:
                    fc.weight.zero_()
                    fc.weight[:, :main_flat].copy_(w[:, :main_flat])
                    fc.weight[:, -N_SCALAR:].copy_(w[:, -N_SCALAR:])
                else:
                    raise ValueError(f'全连接 f{k} 权重形状不兼容: {tuple(w.shape)} -> {tuple(fc.weight.shape)}')
                fc.bias.copy_(b)
