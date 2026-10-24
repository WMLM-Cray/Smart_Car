"""top_speed 的纯逻辑:起步 -> 缓升不超过 slew -> 保持取样 -> 刹车;净空不够 / 超速立刻刹车。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from calibration.top_speed import LAUNCH_FROM, P, TopRun, stop_dist  # noqa: E402


def speed_of(pwm):
    """粗略的高位曲线:1594 起转 0.67,1602 以后每 PWM +25 mm/s。"""
    if pwm < 1594:
        return 0.0
    return 0.67 + max(0, pwm - 1602) * 0.025


def simulate(run, gap=20.0, dt=0.05, t_end=30.0):
    t, v, out = 0.0, 0.0, []
    while not run.done and t < t_end:
        pwm = run.step(t, v, gap(v, t) if callable(gap) else gap)
        out.append((t, pwm, v))
        want = speed_of(pwm) if pwm > P.pwm_mid else max(0.0, v - (1.3 if pwm < P.pwm_mid else 0.73) * dt)
        v += (want - v) * min(1.0, dt / 0.2)
        t += dt
    return out


def test_ramps_within_slew_holds_then_brakes_to_neutral():
    run = TopRun(1650, slew=40.0, hold=1.2)
    out = simulate(run)
    assert out[0][1] == LAUNCH_FROM
    fwd = [p for _, p, _ in out if p > LAUNCH_FROM]
    assert max(b - a for a, b in zip(fwd, fwd[1:])) <= 40.0 * 0.05 + 1
    assert max(fwd) == 1650
    assert run.result() > 1.6 and run.why is None
    assert P.brake_pwm in [p for _, p, _ in out]
    assert out[-1][1] == P.pwm_mid and run.done


def test_wall_too_close_brakes_immediately():
    run = TopRun(1680)
    out = simulate(run, gap=lambda v, t: 5.0 if t < 2.0 else 0.8)
    assert run.why and '前方' in run.why
    t_hit = next(t for t, p, _ in out if t >= 2.0)
    assert all(p <= P.pwm_mid for t, p, _ in out if t > t_hit)


def test_stop_dist_grows_with_speed():
    assert stop_dist(0.0) == 0.5
    assert stop_dist(2.5) > 3.0
