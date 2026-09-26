"""Pure-logic tests for the simulation-only road_v2 runtime."""
import ast
import math
import unittest
from pathlib import Path

import numpy as np

from common.control import to_cmd_vel
from common.params import CarParams
from common.pwm import action_to_steer
from common.road_policy import action_to_speed, observe
from common.road_runtime import RoadRuntime

NODE = Path(__file__).with_name('road_policy_node.py')


class FakePolicy:
    def __init__(self, action=(0.5, 1.0)):
        self.action = np.asarray(action, dtype=np.float32)
        self.observations = []
        self.resets = 0

    def reset(self):
        self.resets += 1

    def __call__(self, obs):
        self.observations.append(np.asarray(obs).copy())
        return self.action.copy()


def ready_runtime(policy=None, **kwargs):
    policy = policy or FakePolicy()
    rt = RoadRuntime(policy, **kwargs)
    n = 360
    rt.on_scan(np.full(n, 4.0), -math.pi, 2 * math.pi / n, 10.0)
    rt.on_odometry(0.42, 10.0)
    rt.on_imu(-0.12, 10.0)
    rt.set_enabled(True, 10.0)
    return rt, policy


class RoadRuntimeTests(unittest.TestCase):
    def test_output_uses_measured_steering_curve_and_cmd_vel_kinematics(self):
        rt, _ = ready_runtime(speed_cap=0.7)
        actual = rt.step(10.05)
        steer = action_to_steer(0.5, CarParams())
        expected = to_cmd_vel(min(action_to_speed(1.0), 0.7), steer, CarParams())
        np.testing.assert_allclose(actual, expected)
        self.assertGreater(actual[1], 0.0)
        self.assertNotAlmostEqual(actual[1], 0.7 * math.tan(0.5 * CarParams().max_steer) / 0.33)

    def test_observation_preserves_training_order_and_raw_previous_action(self):
        rt, policy = ready_runtime(FakePolicy(action=(0.25, -0.5)), speed_cap=0.65)
        rt.step(10.05)
        expected = observe(rt.scan, rt.angles, 0.42, 0.65, -0.12, (0.0, 0.0))
        np.testing.assert_allclose(policy.observations[-1], expected)
        rt.on_scan(rt.scan, -math.pi, 2 * math.pi / len(rt.scan), 10.1)
        rt.on_odometry(0.42, 10.1)
        rt.on_imu(-0.12, 10.1)
        rt.step(10.15)
        expected2 = observe(rt.scan, rt.angles, 0.42, 0.65, -0.12, (0.25, -0.5))
        np.testing.assert_allclose(policy.observations[-1], expected2)

    def test_each_sensor_must_be_fresh(self):
        for sensor in ('scan', 'odom', 'imu'):
            with self.subTest(sensor=sensor):
                rt, _ = ready_runtime(sensor_timeout=0.3)
                setattr(rt, sensor + '_t', 9.7)
                self.assertEqual(rt.step(10.05), (0.0, 0.0))

    def test_enable_heartbeat_expires_to_zero_and_disables(self):
        rt, _ = ready_runtime(enable_timeout=0.3)
        self.assertEqual(rt.step(10.31), (0.0, 0.0))
        self.assertFalse(rt.enabled)
        self.assertIsNone(rt.step(10.32))

    def test_explicit_disable_requests_immediate_neutral(self):
        rt, _ = ready_runtime()
        self.assertTrue(rt.set_enabled(False, 10.1))
        self.assertFalse(rt.set_enabled(False, 10.2))
        self.assertIsNone(rt.step(10.2))

    def test_module_import_path_has_no_top_level_ros_import_or_pwm_topic(self):
        tree = ast.parse(NODE.read_text(encoding='utf-8'))
        for n in tree.body:
            if isinstance(n, ast.Import):
                self.assertFalse(any(a.name == 'rclpy' for a in n.names))
            if isinstance(n, ast.ImportFrom):
                self.assertNotEqual(n.module, 'rclpy')
        source = NODE.read_text(encoding='utf-8')
        self.assertIn("CAR_DOMAIN = '42'", source)
        self.assertIn('domain is None or domain == CAR_DOMAIN', source)
        self.assertIn("CMD_TOPIC = '/cmd_vel'", source)
        self.assertNotIn('/teleop_cmd_vel', source)
        self.assertNotIn('PWM', source)



class NodeArgumentGuardTests(unittest.TestCase):
    """main() 在 import rclpy 之前就拒绝:没设域、实车域 42、训练范围外的速度上限。"""

    def run_main(self, domain, *args):
        from unittest import mock
        from vehicle import road_policy_node
        env = {} if domain is None else {'ROS_DOMAIN_ID': domain}
        with mock.patch.dict('os.environ', env, clear=True):
            with self.assertRaises(SystemExit) as cm:
                road_policy_node.main([str(NODE), *args])
        return str(cm.exception)

    def test_rejects_unset_and_car_domain(self):
        self.assertIn('simulation only', self.run_main(None))
        self.assertIn('simulation only', self.run_main('42'))

    def test_rejects_speed_cap_outside_training_range(self):
        for cap in ('0.3', '1.2'):
            with self.subTest(cap=cap):
                self.assertIn('training range', self.run_main('60', '--speed-cap', cap))

    def test_sim_domain_and_valid_cap_pass_the_guards(self):
        # 过了域和上限检查,停在"模型文件不存在"这一步
        from unittest import mock
        from vehicle import road_policy_node
        with mock.patch.dict('os.environ', {'ROS_DOMAIN_ID': '60'}, clear=True):
            with self.assertRaises(SystemExit) as cm:
                road_policy_node.main(['no_such_model.npz', '--speed-cap', '0.7'])
        self.assertIn('model file not found', str(cm.exception))


if __name__ == '__main__':
    unittest.main()
