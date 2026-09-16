import copy
import json
from pathlib import Path
import unittest
from check_contract import audit_values, class_defaults, exit_code, json_safe


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.contract = json.loads(Path(__file__).with_name('contract.json').read_text(encoding='utf-8'))
        self.values = {}
        for spec in self.contract['parameters']:
            for profile, field in spec['bindings'].items():
                self.values.setdefault(profile, {})[field] = spec['expected']
        self.values['sim']['pwm_mid'] = 1550
        self.values['teleop'].update(max_speed_forward=.1, max_speed_reverse=.1,
            max_steering_angle=.35, low_speed_scale=.4, input_timeout=.3)
        self.values['safety'] = dict(max_linear=.1, max_angular=.35, cmd_timeout=.5)

    def audit(self):
        return audit_values(self.contract, self.values, {})

    def test_aligned_defaults_allow_disclosed_pwm_difference(self):
        findings = self.audit()
        self.assertEqual(exit_code(findings), 0)
        self.assertTrue(any(f['id']=='pwm.profile_separation' and f['status']=='KNOWN_DIFFERENCE' for f in findings))

    def test_two_profiles_wrong_together_still_fail(self):
        self.values['sim']['wheelbase'] = .305
        self.values['gazebo']['wheel_base'] = .305
        self.assertEqual(exit_code(self.audit()), 1)

    def test_sensor_regression(self):
        self.values['gazebo']['lidar_samples'] = 2000
        self.assertTrue(any(f['id']=='lidar_points.gazebo' and f['status']=='FAIL' for f in self.audit()))

    def test_invalid_number_is_not_pass_or_invalid_json(self):
        self.values['teleop']['wheelbase'] = float('nan')
        findings = self.audit()
        self.assertEqual(exit_code(findings), 1)
        json.dumps(json_safe(findings), allow_nan=False)

    def test_reverse_limit(self):
        self.values['teleop']['max_speed_reverse'] = .2
        self.assertEqual(exit_code(self.audit()), 1)

    def test_timeout_order(self):
        self.values['teleop']['input_timeout'] = .8
        self.assertEqual(exit_code(self.audit()), 1)

    def test_input_error_is_not_success(self):
        self.assertEqual(exit_code(audit_values(self.contract, self.values, {}, [('sim','missing')])), 2)

    def test_project_code_is_not_executed(self):
        source = "raise RuntimeError('must not execute')\nclass CarParams:\n    wheelbase: float = 0.36\n"
        self.assertEqual(class_defaults(source, 'CarParams')['wheelbase'], .36)
        with self.assertRaises(ValueError):
            class_defaults('class CarParams:\n    wheelbase: float = dangerous()\n', 'CarParams')

    def test_cleanup_review_ignores_comment(self):
        result = audit_values(self.contract, self.values, {'launcher':'# pkill x\npkill -f "ros2 launch"\n'})
        finding = next(f for f in result if f['id']=='launcher.global_cleanup')
        self.assertEqual(finding['detail']['lines'], [2])


if __name__ == '__main__':
    unittest.main()
