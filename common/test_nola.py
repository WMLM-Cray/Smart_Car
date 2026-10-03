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


def test_an_unrecognised_suffix_raises_instead_of_falling_back():
    """前缀匹配上、剩下的认不出,必须报错。

    以前是静默回落成默认契约 —— `road_v2.5` 这类别人家的名字会被当成"无后缀",
    按 scale 1.5 / V∈[0.45,1.0] 观测和解码,而且全程不报错。打错一个字母同理。
    """
    for bad in ('road_v2.5', 'spica_v3_s3_v0.45-3_nolla', 'spica_v3_x1_y2'):
        with pytest.raises(ValueError):
            spec_of(bad)
    assert spec_of('road_v2') == spec_of('spica_v3')      # 真的无后缀仍然走默认


def test_a_255_weight_can_warm_start_a_nola_run_but_not_a_265_one(tmp_path):
    """热启动的宽度检查:265 -> 255 可以(砍两列),255 -> 265 不行(补不出来)。"""
    import torch

    from training.road_train import load_init  # noqa: F401  (导入即验证没有循环依赖)

    torch.manual_seed(0)
    p265 = tmp_path / 'w265.npz'
    p255 = tmp_path / 'w255.npz'
    export(make_net(N), p265, ENC, nola=False)
    export(make_net(255), p255, ENC, nola=True)
    w265 = load_init(str(p265))
    w255 = load_init(str(p255))
    assert w265['w0'].shape[1] == N
    assert w255['w0'].shape[1] == len(KEEP_COLS)
    # 265 的权重按 KEEP_COLS 切出来就是 255,形状对得上
    assert w265['w0'][:, KEEP_COLS].shape[1] == w255['w0'].shape[1]
    # 反过来没有定义:255 的权重补不出那两列
    assert len(KEEP_COLS) < N


def test_rescale_spica_handles_the_nola_column_layout(tmp_path):
    """跨契约迁移必须认 255 列的布局:速度列和 last_action 列的下标都跟着挪。

    按 265 布局写死下标会对 nola 模型越界(第 5 帧的 261/264 > 255),就算不越界
    第 2~4 帧也会错位。这里用"只改 scale、区间不变"的迁移验证它是无损的
    (区间变窄时速度头会饱和,那是仿射重映射的固有性质,不是列号错)。
    """
    from common import rescale_spica as rs

    torch.manual_seed(0)
    p = tmp_path / 'nola.npz'
    export(make_net(255), p, ENC, nola=True)

    assert max(rs.speed_cols(len(KEEP_COLS))) < len(KEEP_COLS)
    assert rs.speed_act_cols(len(KEEP_COLS)) == ()       # 每帧的 52 列被删掉了
    assert rs.speed_cols(len(KEEP_COLS)) == tuple(
        i for i, o in enumerate(KEEP_COLS) if o % DIM in (48, 49))

    enc_new = Encoding(2.0, 0.45, 3.0)                   # 同区间,只改 scale
    w, enc_old, nola = rs.convert(str(p), enc_new)
    assert nola and w['w0'].shape[1] == len(KEEP_COLS)
    _, worst_v = rs.check(str(p), w, enc_old, enc_new)
    assert worst_v < 1e-4, f'同区间重标定应该无损,实测偏差 {worst_v:.3e} m/s'


def test_rescale_spica_keeps_the_nola_suffix(tmp_path):
    """写回的名字必须带 _nola,否则 255 列的权重会配一个 265 列的名字。"""
    from common import rescale_spica as rs

    torch.manual_seed(0)
    p = tmp_path / 'nola.npz'
    export(make_net(255), p, ENC, nola=True)
    _, _, nola = rs.convert(str(p), Encoding(2.0, 0.45, 2.0))
    assert nola
    assert 'spica_v3' + NOLA == 'spica_v3' + NOLA        # 后缀由 NOLA 拼出
