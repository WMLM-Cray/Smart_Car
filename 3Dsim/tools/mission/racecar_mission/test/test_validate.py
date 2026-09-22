import math
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from racecar_mission import validate
from racecar_mission.validate import ValidationError


def doc(**m):
    base = {'mission': {'frame_id': 'map', 'goals': []}}
    base['mission'].update(m)
    return base


def g(seq=1, name='n', x=0.0, y=0.0, yaw=0.0, **kw):
    d = {'seq': seq, 'name': name, 'x': x, 'y': y, 'yaw': yaw}
    d.update(kw)
    return d


class TestValidate(unittest.TestCase):
    def test_valid_minimal(self):
        cfg, goals = validate.mission_from_dict(
            doc(goals=[g(1, 'a'), g(2, 'b')]))
        self.assertEqual(cfg.mode, 'multi')
        self.assertFalse(cfg.loop)
        self.assertEqual(cfg.on_failure, 'stop')
        self.assertEqual(len(goals), 2)

    def test_missing_frame_id(self):
        with self.assertRaises(ValidationError) as cm:
            validate.mission_from_dict({'mission': {'goals': [g()]}})
        self.assertIn('frame_id', str(cm.exception))

    def test_wrong_frame(self):
        with self.assertRaises(ValidationError) as cm:
            validate.mission_from_dict(doc(frame_id='base_link', goals=[g()]))
        self.assertIn('allowed frame', str(cm.exception))

    def test_nonfinite_coords(self):
        for bad in (float('nan'), float('inf'), -float('inf')):
            with self.assertRaises(ValidationError) as cm:
                validate.mission_from_dict(doc(goals=[g(x=bad)]))
            self.assertIn('x', str(cm.exception))

    def test_duplicate_name(self):
        with self.assertRaises(ValidationError) as cm:
            validate.mission_from_dict(doc(goals=[g(1, 'dup'), g(2, 'dup')]))
        self.assertIn('duplicate', str(cm.exception))

    def test_duplicate_seq(self):
        with self.assertRaises(ValidationError) as cm:
            validate.mission_from_dict(doc(goals=[g(1, 'a'), g(1, 'b')]))
        self.assertIn('duplicate', str(cm.exception))

    def test_bad_seq(self):
        for bad in (0, -1, 1.5, True, '1'):
            with self.assertRaises(ValidationError):
                validate.mission_from_dict(doc(goals=[g(seq=bad)]))

    def test_missing_fields(self):
        for drop in ('name', 'seq', 'x', 'y', 'yaw'):
            gg = {'seq': 1, 'name': 'n', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            del gg[drop]
            with self.assertRaises(ValidationError):
                validate.mission_from_dict(doc(goals=[gg]))

    def test_invalid_mode(self):
        with self.assertRaises(ValidationError):
            validate.mission_from_dict(doc(mode='lala', goals=[g()]))

    def test_invalid_loop(self):
        with self.assertRaises(ValidationError):
            validate.mission_from_dict(doc(loop='yes', goals=[g()]))

    def test_invalid_on_failure(self):
        with self.assertRaises(ValidationError):
            validate.mission_from_dict(doc(on_failure='nope', goals=[g()]))

    def test_invalid_enabled(self):
        with self.assertRaises(ValidationError):
            validate.mission_from_dict(doc(enabled='yes', goals=[g()]))

    def test_enabled_false(self):
        cfg, _ = validate.mission_from_dict(doc(enabled=False, goals=[g()]))
        self.assertFalse(cfg.enabled)

    def test_sorts_by_seq(self):
        _, goals = validate.mission_from_dict(doc(goals=[g(3, 'c'), g(1, 'a'), g(2, 'b')]))
        self.assertEqual([x.seq for x in goals], [1, 2, 3])

    def test_per_goal_overrides(self):
        cfg, goals = validate.mission_from_dict(doc(
            frame_id='map', default={'timeout': 10.0},
            goals=[g(1, 'a', frame_id='odom', timeout=5.0, on_failure='skip',
                     xy_tolerance=0.2, yaw_tolerance=0.3)]))
        gg = goals[0]
        self.assertEqual(gg.frame_id, 'odom')
        self.assertEqual(gg.timeout, 5.0)
        self.assertEqual(gg.on_failure, 'skip')
        self.assertEqual(gg.xy_tolerance, 0.2)
        self.assertEqual(gg.yaw_tolerance, 0.3)

    def test_timeout_none_when_no_default(self):
        _, goals = validate.mission_from_dict(doc(goals=[g()]))
        self.assertIsNone(goals[0].timeout)

    def test_empty_goals(self):
        with self.assertRaises(ValidationError):
            validate.mission_from_dict(doc(goals=[]))

    def test_negative_default_timeout(self):
        with self.assertRaises(ValidationError):
            validate.mission_from_dict(doc(default={'timeout': -1.0}, goals=[g()]))

    def test_allowed_frames_override(self):
        with self.assertRaises(ValidationError):
            validate.mission_from_dict(doc(goals=[g(frame_id='camera')]),
                                       allowed_frames=('map', 'odom'))
        cfg, goals = validate.mission_from_dict(
            doc(goals=[g(frame_id='odom')]), allowed_frames=('map', 'odom'))
        self.assertEqual(goals[0].frame_id, 'odom')


if __name__ == '__main__':
    unittest.main()
