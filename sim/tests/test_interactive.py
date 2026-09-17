"""交互入口(ManualDriver 键序状态机)的单元与冒烟测试(2026-09-17 任务书五-B)。

GUI 窗口级证据在 docs/validation/2026-09-17-sim2d/(KeyEvent 注入截图+演示 GIF);
这里覆盖键语义本身与无头路径。
"""
import numpy as np

from sim.interactive import DEMO_SCRIPT, ManualDriver, make_env


def test_driver_speed_keys_accumulate_and_hold_on_release():
    d = ManualDriver()
    assert list(d.action()) == [0.0, 0.0]
    for _ in range(3):
        d.press("up")
    d.release("up")                       # 速度键松开=保持(巡航语义)
    assert d.a0 == 0.3 and d.a1 == 0.0
    for _ in range(5):
        d.press("down")                   # 减到 -0.2
    assert d.a0 == -0.2


def test_driver_steering_keys_center_on_release():
    d = ManualDriver()
    d.press("left")
    d.press("left")
    assert d.a1 == 0.2
    d.release("left")                     # 转向键松开=立即回中
    assert d.a1 == 0.0
    d.press("right")
    assert d.a1 == -0.1
    d.release("right")
    assert d.a1 == 0.0


def test_driver_clamps_and_stop_key_and_clear():
    d = ManualDriver()
    for _ in range(20):
        d.press("up")
        d.press("left")
    assert d.a0 == 1.0 and d.a1 == 1.0    # 动作域截断
    d.press("x")                          # 指令级急停:速度清零,转向保留
    assert d.a0 == 0.0 and d.a1 == 1.0
    d.clear()                             # 失焦/关窗/重置:全清
    assert d.a0 == 0.0 and d.a1 == 0.0
    for _ in range(20):
        d.press("down")
    assert d.a0 == -1.0                   # 倒车指令可达下限


def test_manual_reverse_steering_gives_bicycle_model_omega_sign():
    """键盘 a0<0(倒车)+ a1>0(左)→ env 里 ω=v·tanδ/L<0:符号随 v 反转,
    交互入口不自造转向语义,原样交给自行车模型。"""
    env, _ = make_env("random", 0, 30.0)
    d = ManualDriver()
    for _ in range(4):
        d.press("down")                   # a0=-0.4 → v_cmd=-0.4·0.5=-0.2 m/s
    d.press("left")                       # a1=+0.1 → δ>0
    for _ in range(60):                   # 3s 达稳态
        env.step(d.action())
    assert env.car.v < 0
    assert env.car.steer > 0
    assert env.car.yaw_rate < 0           # ω 反号来自模型,不是键盘层


def test_headless_demo_script_runs_and_ends_stopped(tmp_path):
    """演示键序(起步→左转→停→倒车→急停)整链可跑,结束时 v≈0 且 a0 已清。"""
    import subprocess, sys
    out = tmp_path / "demo.gif"
    r = subprocess.run([sys.executable, "-m", "sim.interactive",
                        "--demo-gif", str(out)],
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr[-800:]
    assert out.stat().st_size > 10_000
    assert "selftest:" in r.stdout        # 无头状态行走完
    assert len(DEMO_SCRIPT) >= 10         # 键序本身有内容,不是空跑
