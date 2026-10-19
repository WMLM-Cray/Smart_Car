"""evaluation/l_test 的 run / compare(v4.5 主指标)。"""
from evaluation.l_test import case, paired, run_case


def test_cases_are_deterministic_and_cover_both_turns():
    cs = [case(96_000_000 + i) for i in range(60)]
    assert cs == [case(96_000_000 + i) for i in range(60)]
    assert {c['right_turn'] for c in cs} == {True, False}
    assert all(5.0 <= c['front'] <= 9.0 and abs(c['yaw']) <= 10.0 and c['open_left'] == (c['width'] >= 5.0) for c in cs)


def test_teacher_turns_early_on_a_mirrored_right_turn():
    c = next(c for c in (case(96_000_000 + i) for i in range(20)) if c['right_turn'])
    r = run_case(('teacher', c, 0.7))
    assert r['why'] == 'goal' and r['onset'] > 2.0          # 老师在离前墙 2 m 以外就开始往右转


def test_paired_compares_the_same_episodes():
    a = [dict(seed=1, why='goal', onset=2.0), dict(seed=2, why='guard', onset=1.0), dict(seed=3, why='goal', onset=None)]
    b = [dict(seed=3, why='goal', onset=3.0), dict(seed=2, why='goal', onset=1.5), dict(seed=1, why='goal', onset=2.5)]
    p = paired(a, b)
    assert p['n'] == 3 and p['only_b'] == 1 and p['only_a'] == 0 and p['n_onset'] == 2
    assert abs(p['d_onset'] - 0.5) < 1e-9
