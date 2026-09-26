"""策略模式的放行条件:python3 -m pytest teleop/test_policy_hold.py

这一层管的是"人手上的动作"和"车动不动"的对应关系,错了就是没人推杆车自己开。
和 test_reverse_output.py 一样,用 AST 摘出 server.py 里不碰 ROS 的部分来跑,
PC 上没有 rclpy 也能测。
"""
import ast
import threading
import time
from pathlib import Path

import pytest

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.params import CarParams

KEEP = ('State', 'policy_held', 'compute', 'creep_or_pwm', 'handle_cmd')


@pytest.fixture
def ns():
    tree = ast.parse(Path(__file__).with_name('server.py').read_text(encoding='utf-8'))
    # 只保留不碰 ROS 的顶层赋值。原来的判据是"不含任何调用",但 2026-09-22
    # 起 server.py 的舵机中位改成从 fastsim/params.py 读(避免四个脚本各抄一份
    # 84),那一行带了 CarParams() 调用。CarParams 是纯 dataclass 没有 ROS 依赖,
    # 所以放行它,别的调用照旧挡掉。
    def safe(node):
        if not isinstance(node, ast.Assign):
            return False
        return all(isinstance(c.func, ast.Name) and c.func.id == 'CarParams'
                   for c in ast.walk(node) if isinstance(c, ast.Call))
    keep = [n for n in tree.body if safe(n) or getattr(n, 'name', '') in KEEP]
    out = {'threading': threading, 'time': time, 'CarParams': CarParams,
           'log_event': lambda message: None}
    exec(compile(ast.Module(body=keep, type_ignores=[]), 'server_control', 'exec'), out)
    s = out['State']()
    s.policy, s.estop, s.thr = 1, 0, 1000       # 策略已开、摇杆推满、没急停
    s.last_t = 100.0
    out['S'] = s
    return out


def held(ns, now=100.0):
    return ns['policy_held'](now)


def test_pushing_the_stick_lets_the_policy_drive(ns):
    assert held(ns)


def test_releasing_the_stick_stops_it(ns):
    ns['S'].thr = ns['POLICY_THR']              # 正好在阈值上也不算推着
    assert not held(ns)
    ns['S'].thr = 0
    assert not held(ns)


def test_reverse_does_not_engage_the_policy(ns):
    """往回拉不是"推着" —— 策略只管往前开。"""
    ns['S'].thr = -1000
    assert not held(ns)


def test_estop_wins(ns):
    ns['S'].estop = 1
    assert not held(ns)


def test_stale_joystick_stops_it(ns):
    """网页关了 / Wi-Fi 断了,摇杆数据不再更新 —— 这时必须当成松手。"""
    assert not held(ns, 100.0 + ns['POLICY_DEADMAN'] + 0.01)


def test_hotspot_jitter_does_not_flap_the_policy(ns):
    """摇杆推着,t 的间隔抖到 0.4s 也不能断放行 —— 断一次策略节点就回中位重新爬 3 秒。"""
    assert held(ns, 100.0 + 0.4)


def test_policy_deadman_still_bounded(ns):
    """放宽归放宽,网真断了 1 秒以内必须停。"""
    assert ns['POLICY_DEADMAN'] <= 1.0


def test_mode_off_means_manual(ns):
    ns['S'].policy = 0
    assert not held(ns)


def test_policy_mode_never_publishes_cmd_vel(ns):
    """策略模式下摇杆只是死人开关,不许影响转向和油门。

    2026-09-20 实车上就栽在这:网页摇杆前后和左右共用一根,往侧面推时油门分量
    掉到阈值以下,服务器就按摇杆转向接管,松一点又切回策略 —— 控制权来回跳,
    车往墙上走。所以 tick() 在 S.policy 为真时必须直接 return,一条都不发。
    """
    src = (Path(__file__).resolve().parent / 'server.py').read_text(encoding='utf-8')
    body = src.split('    def tick(self):')[1].split('\n    def ')[0]
    head = body.split('pwm, turn, active = compute(now)')[0]
    assert 'if policy_mode:' in head and 'return' in head, 'tick() 里策略模式没有提前 return'
    assert head.index('if policy_mode:') < head.index('self.enable_pub.publish'), \
        '心跳要在策略分支里发'


def test_manual_mode_still_drives(ns):
    """关掉策略之后,普通遥控必须照常能开 —— 别把手动挡也堵死了。"""
    s = ns['S']
    s.policy, s.thr, s.steer, s.last_t = 0, 1000, 500, 100.0
    s.enc_t, s.range, s.mode, s.guard = 100.0, 1, 1, 0
    pwm, turn, active = ns['compute'](100.0)
    assert active, '手动模式下推杆应该发指令'
    assert turn != ns['TURN_MID'], '打了方向舵机值该变'


def test_other_page_cannot_change_active_driver(ns):
    s = ns['S']
    command = ns['handle_cmd']
    command('t 800 300', 'driver')
    assert s.owner == 'driver'
    assert command('t 1000 -1000', 'other') == '另一页面正在控制小车'
    assert command('p 0', 'other') == '另一页面正在控制小车'
    assert command('g 0', 'other') == '另一页面正在控制小车'
    assert (s.thr, s.steer, s.policy, s.guard) == (800, 300, 1, 1)
    assert command('e 1', 'other') == '急停'
    assert s.estop == 1
    assert command('e 0', 'other') == '另一页面正在控制小车'


def test_stale_driver_can_be_replaced_and_old_driver_cannot_resume(ns):
    s = ns['S']
    command = ns['handle_cmd']
    command('t 800 300', 'old')
    s.owner_t -= ns['CONTROL_LEASE'] + 0.01
    command('p 0', 'new')
    assert (s.owner, s.policy, s.thr, s.last_t) == ('new', 0, 0, 0)
    assert command('t 800 300', 'old') == '另一页面正在控制小车'
    assert s.thr == 0
