"""Exercise actual server control code without importing ROS or publishing commands."""
import ast
from pathlib import Path
import threading
import unittest


class ReverseOutputTests(unittest.TestCase):
    def setUp(self):
        tree = ast.parse(Path(__file__).with_name('server.py').read_text(encoding='utf-8'))
        keep = [n for n in tree.body if isinstance(n, (ast.Assign, ast.ClassDef, ast.FunctionDef))
                and (isinstance(n, ast.Assign)
                     and not any(isinstance(x, ast.Call) for x in ast.walk(n))
                     or getattr(n, 'name', '') in ('State', 'compute', 'creep_or_pwm'))]
        self.ns = {'threading': threading}
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
