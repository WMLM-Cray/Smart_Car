"""_nola 通路的测试:网络输入层删掉 last_action 两列(265 -> 255)。

背景:last_action 通道让策略"抄上一步",闭环退化成自激反馈环(官方赛道 A 上 0/30,
剪断后 30/30)。修法是**真删**这两列(不是置零,置零是 OOD)。观测向量仍是 53 维
(环境/v1 兼容/rescale_spica 全不动),只是网络接不到。
"""
from __future__ import annotations

import numpy as np
import pytest
import torch

from common.road_policy import (DIM, FRAMES, KEEP_COLS, NOLA, Encoding, History,
                                RoadPolicy, forward, spec_of)
from training.road_train import export, make_net, weights_of

N = DIM * FRAMES
ENC = Encoding(3.0, 0.45, 3.0)


def test_keep_cols_drops_exactly_the_two_last_action_columns():
    assert len(KEEP_COLS) == N - 2 * FRAMES == 255
    kept = set(KEEP_COLS.tolist())
    for f in range(FRAMES):
        for i in (51, 52):
            assert (f * DIM + i) not in kept
    assert list(KEEP_COLS) == sorted(KEEP_COLS.tolist()), 'KEEP_COLS 必须保持列序'


def test_nola_suffix_does_not_disturb_the_speed_contract():
    assert spec_of('spica_v3_s3_v0.45-3' + NOLA) == ENC
    assert spec_of('spica_v3' + NOLA) == spec_of('spica_v3')
    assert spec_of('spica_v3_s3_v0.45-3') == ENC


def test_forward_truncates_a_265_input_for_a_255_net():
    """255 网络收到 265 观测时,应自己切列 —— 这样所有调用点都不用改。"""
    torch.manual_seed(0)
    net255 = make_net(255)
    w255, b255 = weights_of(net255)
    rng = np.random.default_rng(1)
    x = rng.uniform(-1, 1, N).astype(np.float32)
    assert np.allclose(forward(w255, b255, x), forward(w255, b255, x[KEEP_COLS]), atol=1e-6)


def test_255_net_is_blind_to_last_action_but_265_net_is_not():
    torch.manual_seed(0)
    w255, b255 = weights_of(make_net(255))
    w265, b265 = weights_of(make_net(N))
    rng = np.random.default_rng(2)
    x = rng.uniform(-1, 1, N).astype(np.float32)
    x2 = x.copy()
    x2[51], x2[52] = 0.9, -0.9
    x2[DIM + 51], x2[DIM + 52] = 0.9, -0.9
    assert np.allclose(forward(w255, b255, x), forward(w255, b255, x2)), \
        '255 网络对 last_action 两列必须完全不敏感(真删,不是置零)'
    assert not np.allclose(forward(w265, b265, x), forward(w265, b265, x2)), \
        '265 网络应该受 last_action 影响'


def test_export_writes_the_nola_suffix_and_255_input_width(tmp_path):
    torch.manual_seed(0)
    p_nola = tmp_path / 'nola.npz'
    p_ctl = tmp_path / 'ctl.npz'
    export(make_net(255), p_nola, ENC, nola=True)
    export(make_net(N), p_ctl, ENC, nola=False)
    with np.load(p_nola) as z:
        assert str(z['schema']) == 'spica_v3_s3_v0.45-3' + NOLA
        assert z['w0'].shape == (256, 255)
    with np.load(p_ctl) as z:
        assert str(z['schema']) == 'spica_v3_s3_v0.45-3'
        assert z['w0'].shape == (256, N)


def test_road_policy_runs_a_nola_model_end_to_end(tmp_path):
    torch.manual_seed(0)
    p = tmp_path / 'nola.npz'
    export(make_net(255), p, ENC, nola=True)
    pol = RoadPolicy(str(p))
    assert pol.enc == ENC
    obs = np.random.default_rng(3).uniform(-1, 1, DIM).astype(np.float32)
    out = pol(obs)
    assert out.shape == (2,)


def test_a_nola_model_ignores_last_action_in_a_real_history(tmp_path):
    """端到端:观测里的 last_action 两列变了,255 模型输出必须一模一样。"""
    torch.manual_seed(0)
    p = tmp_path / 'nola.npz'
    export(make_net(255), p, ENC, nola=True)
    rng = np.random.default_rng(4)
    base = rng.uniform(-1, 1, DIM).astype(np.float32)
    a, b = base.copy(), base.copy()
    b[51], b[52] = 0.8, -0.8
    p1, p2 = RoadPolicy(str(p)), RoadPolicy(str(p))
    p1.reset(); p2.reset()
    for _ in range(FRAMES):
        p1(base); p2(b)
    assert np.allclose(p1(base), p2(b), atol=1e-6)
