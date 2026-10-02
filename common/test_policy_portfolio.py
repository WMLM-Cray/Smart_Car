"""common.policy_portfolio 的测试:档位选择、schema 自描述、路径校验。"""
from __future__ import annotations

import numpy as np
import pytest

from common import low_speed_policy as v1
from common.policy_portfolio import (LADDER, calibrated, select, select_path)
from training.road_teacher import A_LAT

ROOT = __import__('pathlib').Path(__file__).resolve().parents[1]


def test_table_is_sorted_and_consistent():
    speeds = [row[0] for row in LADDER]
    assert speeds == sorted(speeds), '档位表必须按速度升序 —— 平局取低速依赖这个顺序'
    assert len(set(speeds)) == len(speeds), '档位速度不能重复'


def test_every_rung_model_exists():
    """表里每一档都得真的有文件 —— 路径写错要在这里就炸,不要等到车上。"""
    for speed, path, _rate, label in LADDER:
        assert (ROOT / path).is_file(), f'{label} ({speed} m/s) 指向的 {path} 不存在'


def test_exact_rung_speeds_pick_themselves():
    for speed, path, _rate, _label in LADDER:
        assert select(speed)[0] == path, f'{speed} m/s 应该选自己那一档'


def test_nearest_rung_by_relative_deviation():
    assert select(2.1)[0] == select(2.0)[0]          # 2.1 更近 2.0
    assert select(2.4)[0] == select(2.5)[0]          # 2.4 更近 2.5
    assert select(0.7)[0] == select(0.6)[0]
    assert select(0.9)[0] == select(0.8)[0]


def test_the_1p1_case_that_motivated_relative_deviation():
    """1.1 是本次改动最直接的收益:绝对规则下距 1.0 和 1.2 各 0.1 是平局取低速,
    相对规则下 1.1 距 1.2 只有 8.3%、距 1.0 有 10%,该选 1.2 那一档。

    1.1 落在实车已标定区间(0.66~1.13)内,是真正会上车的一档。
    """
    assert select(1.1)[0] == select(1.2)[0]
    assert calibrated(1.1), '1.1 必须是已标定档,否则这条测试就失去意义了'


def test_ties_resolve_to_the_lower_speed():
    """调和中点(如 1.2 与 1.6 之间是 1.3714)打平取较低速。"""
    mid = 2 * 1.2 * 1.6 / (1.2 + 1.6)
    assert select(mid)[0] == select(1.2)[0]


def test_calibrated_window_is_the_measured_one():
    assert calibrated(0.66) and calibrated(1.13)
    assert not calibrated(0.65)      # 电调停车区以下
    assert not calibrated(1.14)      # 曲线测到 1.133 为止


def test_select_path_reports_missing_file_loudly(tmp_path):
    with pytest.raises(FileNotFoundError, match='不存在'):
        select_path(2.0, tmp_path)


def test_select_path_returns_a_real_path():
    p = select_path(2.0, ROOT)
    assert (ROOT / p).is_file() if not __import__('os').path.isabs(p) else True


# --- schema 自描述 -----------------------------------------------------------

def test_scale_round_trips_through_schema():
    for scale in (1.5, 2.4, 3.0, 3.75, 4.5):
        assert v1.scale_for_schema(v1.schema_for(scale)) == scale


def test_legacy_schema_means_1p5():
    """老模型(无后缀)必须仍然反解成 1.5 —— 实车在跑的 baseline 和 m4 都是它。"""
    assert v1.scale_for_schema('low_speed_steering_v1') == 1.5
    assert v1.schema_for(1.5) == 'low_speed_steering_v1'


def test_unknown_schema_is_rejected():
    with pytest.raises(ValueError):
        v1.scale_for_schema('road_v2')
    with pytest.raises(ValueError):
        v1.scale_for_schema('low_speed_steering_v1_sabc')


def _write_npz(path, schema, dim=52, frames=5):
    rng = np.random.default_rng(0)
    np.savez(path, schema=schema,
             w0=rng.normal(size=(4, dim * frames)).astype(np.float32),
             b0=np.zeros(4, np.float32),
             w1=rng.normal(size=(4, 4)).astype(np.float32),
             b1=np.zeros(4, np.float32),
             w2=rng.normal(size=(1, 4)).astype(np.float32),
             b2=np.zeros(1, np.float32))


@pytest.mark.parametrize('schema,scale', [
    ('low_speed_steering_v1', 1.5),
    ('low_speed_steering_v1_s3', 3.0),
    ('low_speed_steering_v1_s4.5', 4.5),
])
def test_policy_reads_its_own_scale(tmp_path, schema, scale):
    """SteeringPolicy 从模型自带的 schema 反解 scale,不需要外部指定。"""
    p = tmp_path / 'm.npz'
    _write_npz(p, schema)
    policy = v1.SteeringPolicy(p)
    assert policy.scale == scale
    assert policy.schema == schema


def test_policy_still_rejects_a_foreign_schema(tmp_path):
    """双输出的 road_v2 不能被当转向策略载入 —— 必须报错。"""
    p = tmp_path / 'bad.npz'
    _write_npz(p, 'road_v2')
    with pytest.raises(ValueError):
        v1.SteeringPolicy(p)


def test_observe_scale_argument_is_backward_compatible():
    """不传 scale 时逐位等于老行为(用模块级 SPEED_SCALE)。"""
    ranges = np.full(48, 3.0, np.float32)
    angles = np.linspace(-np.pi, np.pi, 48, endpoint=False)
    a = v1.observe(ranges, angles, 1.0, 1.0, 0.0, 0.0)
    b = v1.observe(ranges, angles, 1.0, 1.0, 0.0, 0.0, scale=v1.SPEED_SCALE)
    np.testing.assert_array_equal(a, b)


def test_observe_uses_the_given_scale():
    """scale 真的进了观测 —— 速度通道随 scale 变。"""
    ranges = np.full(48, 3.0, np.float32)
    angles = np.linspace(-np.pi, np.pi, 48, endpoint=False)
    slow = v1.observe(ranges, angles, 1.5, 1.5, 0.0, 0.0, scale=1.5)
    fast = v1.observe(ranges, angles, 1.5, 1.5, 0.0, 0.0, scale=3.0)
    assert slow[48] != fast[48], '速度通道应该随 scale 变化'
    assert abs(slow[48] - 1.0) < 1e-6 and abs(fast[48] - 0.5) < 1e-6


@pytest.mark.parametrize('module', ['evaluation.road_eval', 'evaluation.road_track'])
def test_eval_tools_load_suffixed_v1_schema_with_its_own_scale(module, monkeypatch):
    """评估工具认带尺度后缀的 v1 schema,观测用模型自己的 scale(以前只认完全相等,s2.4 被当成 road_v2 报错)。"""
    import importlib
    mod = importlib.import_module(module)
    model = str(ROOT / 'models/ladder_500k/rung2_sp1.6/candidate_010.npz')
    assert v1.SteeringPolicy(model).scale == 2.4
    seen = []
    real = v1.observe

    def spy(*a, **k):
        seen.append(k.get('scale'))
        return real(*a, **k)
    monkeypatch.setattr(v1, 'observe', spy)
    if module == 'evaluation.road_eval':
        out = mod.run((model, 90_000_000, 'L1'))
    else:
        out = mod.run((model, 'A', 95_000_000, 0.5, True, A_LAT))
    assert out['reason'] in ('goal', 'collision', 'off_track', 'timeout')
    assert seen and set(seen) == {2.4}
