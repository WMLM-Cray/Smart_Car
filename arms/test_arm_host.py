"""锁住宿主的恢复出口默认行为。

盲倒默认关 = 只停不倒，与实车 `vehicle/road_car_node.py:102-103` 同行为；
置 `ARM_HOST_REVERSE=1` 才恢复旧实验宿主盲倒（复现旧 W/Y 面板读数）。

不 import ROS：`arms.arm_host` 在无 rclpy 时 `Node` 退化成 `object`，本模块仍可 import。
"""
import importlib
import os
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _reload_arm_host(reverse_env):
    """按给定 `ARM_HOST_REVERSE` 重新 import，返回模块。"""
    if reverse_env is None:
        os.environ.pop('ARM_HOST_REVERSE', None)
    else:
        os.environ['ARM_HOST_REVERSE'] = reverse_env
    import arms.arm_host as H
    return importlib.reload(H)


class _Stub:
    """绕过 ROS 构造的最小替身：只记 publish/neutral 调了什么。"""

    def __init__(self, module):
        self.cap = 2.2
        self.escape_steer = module.ESCAPE_STEER
        self.calls = []

    def publish(self, v, a):
        self.calls.append(('publish', round(float(v), 4), round(float(a), 4)))

    def neutral(self):
        self.calls.append(('neutral',))


class RestoreDefaultTests(unittest.TestCase):
    def tearDown(self):
        os.environ.pop('ARM_HOST_REVERSE', None)
        import arms.arm_host as H
        importlib.reload(H)

    def test_default_is_stop_only(self):
        H = _reload_arm_host(None)
        self.assertFalse(H.REVERSE_ENABLED)
        node = _Stub(H)
        H.RoadPolicyNode.escape_reverse(node)
        self.assertEqual(node.calls, [('neutral',)])

    def test_env_switch_restores_blind_reverse(self):
        H = _reload_arm_host('1')
        self.assertTrue(H.REVERSE_ENABLED)
        node = _Stub(H)
        H.RoadPolicyNode.escape_reverse(node)
        self.assertEqual(node.calls,
                         [('publish', -round(2.2 * H.REVERSE_FRACTION, 4),
                           -round(H.ESCAPE_STEER, 4))])


if __name__ == '__main__':
    unittest.main()
