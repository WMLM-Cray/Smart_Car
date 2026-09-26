"""Exercise actual server control code without importing ROS or publishing commands."""
import ast
from pathlib import Path
import threading
import unittest

import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.params import CarParams


class ReverseOutputTests(unittest.TestCase):
    def setUp(self):
        tree = ast.parse(Path(__file__).with_name('server.py').read_text(encoding='utf-8'))
        # 只保留不碰 ROS 的顶层赋值。原判据是"不含任何调用",但 2026-09-22 起
        # server.py 的舵机中位改成从 common/params.py 读(避免四个脚本各抄一份
        # 84),那一行带了 CarParams() 调用。CarParams 是纯 dataclass 无 ROS 依赖,
        # 所以放行它,别的调用照旧挡掉。
        def safe(node):
            if not isinstance(node, ast.Assign):
                return False
            return all(isinstance(c.func, ast.Name) and c.func.id == 'CarParams'
                       for c in ast.walk(node) if isinstance(c, ast.Call))
        keep = [n for n in tree.body
                if safe(n) or getattr(n, 'name', '') in ('State', 'compute', 'creep_or_pwm')]
        self.ns = {'threading': threading, 'CarParams': CarParams}
        exec(compile(ast.Module(body=keep, type_ignores=[]), 'server_control', 'exec'), self.ns)
        self.state = self.ns['State']()
        self.ns['S'] = self.state
        self.state.thr = -1000
        self.state.last_t = self.state.enc_t = 10.

    def test_speed_does_not_cut_reverse(self):
        for speed in (0., .41, -.8, 2.):
            self.state.spd = speed
            pwm, _, active = self.ns['compute'](10.1)
            self.assertEqual(pwm, self.ns['REV_CAP'])
            self.assertTrue(active)

    def test_stale_encoder_still_blocks(self):
        self.state.enc_t = 0.
        self.assertEqual(self.ns['compute'](10.1)[0], self.ns['SPEED_MID'])
        self.assertEqual(self.state.rev_block, 2)

    def test_estop_and_deadman_still_stop(self):
        self.state.estop = 1
        self.assertFalse(self.ns['compute'](10.1)[2])
        self.state.estop = 0
        self.assertFalse(self.ns['compute'](11.)[2])


    def test_all_manual_forward_modes_obey_hard_limit(self):
        from vehicle.speed_loop import PWM_HARD_MAX
        self.state.guard = False
        for mode in range(3):
            self.state.mode = mode
            for throttle in (1, 100, 450, 700, 1000):
                self.state.thr = throttle
                self.state.last_dir = 1
                self.state.dir_change_t = 0.
                for _ in range(self.ns['DUTY_TICKS']):
                    pwm, _, active = self.ns['compute'](10.1)
                    self.assertTrue(active)
                    self.assertLessEqual(pwm, PWM_HARD_MAX)
