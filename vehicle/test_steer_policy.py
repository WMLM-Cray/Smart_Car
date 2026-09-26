"""转向策略上车前的口径检查:python3 -m pytest vehicle/test_steer_policy.py

observe() 的六个参数里有两对很容易搞反:(speed, target_speed) 和
(yaw_rate, last_steer)。传反了不会报错 —— 网络照样输出一个数,车照样开,
只是开得莫名其妙。这类问题在车上极难查,所以在这里钉死。

不 import steer_policy_node 本体:它 import rclpy,开发机上装不了。
"""
import ast
from pathlib import Path

import numpy as np
import pytest

from common.low_speed_policy import DIM, FRAMES, SCHEMA, History, SteeringPolicy, observe
from common.params import CarParams

NODE = Path(__file__).with_name('steer_policy_node.py')


def observe_call(path):
    """源码里那次 observe(...) 调用的实参列表(按位置)。"""
    tree = ast.parse(path.read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'observe'):
            return [ast.unparse(a) for a in node.args]
    raise AssertionError(f'{path.name} 里没有 observe() 调用')


def test_node_passes_six_positional_args():
    args = observe_call(NODE)
    assert len(args) == 6, f'observe 要六个位置参数,节点给了 {len(args)}:{args}'


def test_node_argument_order_matches_contract():
    args = observe_call(NODE)
    assert args[2:] == ['self.speed', 'self.target_speed', 'self.yaw_rate', 'self.last_steer']


def test_node_feeds_back_raw_action_not_angle():
    """last_steer 存的必须是网络原始输出(±1),不是乘了 max_steer 的转角。

    训练里是 last = float(np.clip(steer, -1, 1)),量纲是 ±1。存成转角的话
    观测第 52 维会小一个 max_steer 倍(0.55),整条历史都偏。
    """
    src = NODE.read_text(encoding='utf-8')
    body = src.split('def tick(self):')[1]
    assert 'self.last_steer = action' in body, 'last_steer 要存网络原始输出'
    assert 'steer = action_to_steer(action, self.p)' in body, '转角才乘 max_steer'
    assert body.index('self.last_steer = action') < body.index('steer = action_to_steer('), \
        '先存原始输出再换算转角,顺序反了会把转角存进去'


def test_observe_shape_and_range():
    n = 1667
    ranges = np.full(n, 3.0)
    angles = np.linspace(-np.pi, np.pi, n, endpoint=False)
    obs = observe(ranges, angles, 0.58, 0.6, 0.1, -0.3)
    assert obs.shape == (DIM,)
    assert obs.dtype == np.float32
    assert np.all(np.abs(obs) <= 1.0), 'observe 出来必须已经 clip 到 ±1'


def test_target_speed_actually_changes_the_observation():
    """target_speed 那一维要真的带信息 —— 否则跨速度泛化无从谈起。"""
    n = 1667
    ranges, angles = np.full(n, 3.0), np.linspace(-np.pi, np.pi, n, endpoint=False)
    a = observe(ranges, angles, 0.5, 0.4, 0.0, 0.0)
    b = observe(ranges, angles, 0.5, 1.0, 0.0, 0.0)
    assert not np.allclose(a, b), 'target_speed 没进观测'


def test_history_stacking_matches_training():
    """SteeringPolicy.__call__ 内部 push 一次,和训练里 history.push(obs) 等价。"""
    rng = np.random.default_rng(0)
    h = History()
    stacked = None
    for _ in range(FRAMES + 2):
        obs = rng.standard_normal(DIM).astype(np.float32)
        stacked = h.push(obs)
    assert stacked.shape == (DIM * FRAMES,)
    assert np.allclose(stacked[-DIM:], obs), '最新一帧应该在末尾'


@pytest.mark.parametrize('name', ['candidate_060.npz'])
def test_trained_checkpoint_loads(name):
    path = Path(__file__).resolve().parent.parent / 'models' / 'steering_v1' / name
    if not path.exists():
        pytest.skip(f'{name} 不在本机')
    policy = SteeringPolicy(path)
    n = 1667
    obs = observe(np.full(n, 3.0), np.linspace(-np.pi, np.pi, n, endpoint=False),
                  0.6, 0.6, 0.0, 0.0)
    a = policy(obs)
    assert -1.0 <= a <= 1.0
    assert isinstance(a, float)


def test_schema_guard_rejects_the_wrong_policy(tmp_path):
    """双输出的 policy_s3.npz 不能被当转向策略载入 —— 维度不同,必须报错。"""
    bad = tmp_path / 'bad.npz'
    np.savez(bad, schema='not-' + SCHEMA,
             **{f'w{i}': np.zeros((2, 2)) for i in range(3)},
             **{f'b{i}': np.zeros(2) for i in range(3)})
    with pytest.raises(ValueError):
        SteeringPolicy(bad)


# --- 油门爬升 ---------------------------------------------------------------
# 20 PWM/s 有实测依据；直接跳变曾使落地车移动，不能依赖电调拒绝指令。

def node_const(name):
    """从源码里取一个模块级常量,不 import(节点 import rclpy)。"""
    tree = ast.parse(NODE.read_text(encoding='utf-8'))
    for node in tree.body:
        if isinstance(node, ast.Assign) and node.targets[0].id == name:
            return ast.literal_eval(node.value)
    raise AssertionError(f'{NODE.name} 里没有常量 {name}')


def test_ramp_rate_is_in_the_window_that_actually_starts_the_esc():
    """20Hz 下每 tick 加 PWM_RAMP_UP,换算成每秒必须落在实测可用的区间里。"""
    rate = node_const('PWM_RAMP_UP') / 0.05          # tick 是 0.05s
    assert 10 <= rate <= 40, (
        f'{rate:.0f} PWM/s:低于 10 电调不起转(实测 2 PWM/s 不转),'
        f'高于 40 接近跳变(实测一步到位不转)')


def test_ramp_reaches_full_throttle_quickly_enough_to_be_usable():
    """从中位爬到硬上限的时间。太久的话车一使能要等半天才动。"""
    step = node_const('PWM_RAMP_UP')
    span = node_const('PWM_HARD_MAX') - CarParams().pwm_mid
    assert span / step * 0.05 < 5.0


def test_tick_ramps_instead_of_jumping_to_target():
    src = NODE.read_text(encoding='utf-8')
    body = src.split('def tick(self):')[1]
    assert 'self.cur_pwm + PWM_RAMP_UP' in body, 'tick 要爬升,不能直接发 self.pwm'
    assert 'self.send(self.cur_pwm, steer)' in body, '发出去的应该是爬升值'
    assert 'self.send(self.pwm, steer)' not in body, '还留着一步到位的发送'


def test_neutral_resets_the_ramp():
    """回中位必须清掉爬升状态,否则恢复时是一个跳变 —— 截停后就再也起不来。"""
    src = NODE.read_text(encoding='utf-8')
    body = src.split('def send(self, pwm, steer):')[1].split('def ')[0]
    assert 'self.cur_pwm = float(self.p.pwm_mid)' in body


# --- --speed-loop:油门走编码器闭环 -----------------------------------------------

def test_speed_loop_limits_match_the_node():
    """闭环和节点必须是同一条安全线、同一个爬升速率。"""
    from vehicle.speed_loop import SpeedLoopConfig
    cfg = SpeedLoopConfig()
    assert cfg.pwm_max == node_const('PWM_HARD_MAX')
    assert cfg.up_rate == pytest.approx(node_const('PWM_RAMP_UP') / 0.05)


def test_tick_sends_loop_output_when_speed_loop_is_on():
    body = NODE.read_text(encoding='utf-8').split('def tick(self):')[1]
    assert 'self.loop.update(self.target_speed, self.speed, 0.05)' in body


def test_releasing_the_stick_resets_throttle_state():
    """松手那条路径不发中位。再推摇杆时油门状态必须是中位,不然第一拍就是跳变。"""
    body = NODE.read_text(encoding='utf-8').split('def on_enable(self, m):')[1].split('def ')[0]
    assert 'self.cur_pwm = float(self.p.pwm_mid)' in body
    assert 'self.loop.reset()' in body


def test_neutral_resets_the_loop_too():
    body = NODE.read_text(encoding='utf-8').split('def send(self, pwm, steer):')[1].split('def ')[0]
    assert 'self.loop.reset()' in body


def test_speed_loop_rejects_conflicting_flags():
    src = NODE.read_text(encoding='utf-8').split('def main():')[1].split('rclpy.init')[0]
    assert "if speed_loop and (sim or pwm is not None):" in src
    assert "0.0 < (target or 0.0) <= 1.0" in src


def test_releasing_the_stick_sends_neutral_itself():
    """松手必须自己发中位,不能只靠看门狗(它有门槛,2026-09-24 就漏过一次)。"""
    body = NODE.read_text(encoding='utf-8').split('def on_enable(self, m):')[1].split('def ')[0]
    stop_branch = body.split('if not m.data:')[1].split('return')[0]
    assert 'self.neutral()' in stop_branch
