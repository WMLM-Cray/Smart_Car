import pytest

from evaluation.bev_testset import paired_diff_ci


def test_paired_ci_matches_audit_example():
    # 1500 局、两边各独胜 100 局:差 0,下限约 −1.85 个百分点(《强化学习奖励审计》§10 第 1 条)
    a = [True] * 100 + [False] * 100 + [True] * 1300
    b = [False] * 100 + [True] * 100 + [True] * 1300
    d, lo, hi = paired_diff_ci(a, b)
    assert d == 0 and lo == pytest.approx(-0.0185, abs=2e-4) and hi == pytest.approx(0.0185, abs=2e-4)


def test_paired_ci_sign_is_b_minus_a():
    a = [False] * 30 + [True] * 70
    b = [True] * 100
    d, lo, hi = paired_diff_ci(a, b)
    assert d == pytest.approx(0.3) and lo > 0
