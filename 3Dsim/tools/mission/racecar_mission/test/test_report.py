import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from racecar_mission import report as rep
from racecar_mission.engine import MissionEngine
from racecar_mission.goal import Goal, MissionConfig


def G(seq, on_failure='stop', timeout=None):
    return Goal(seq=seq, name='g%d' % seq, frame_id='map', x=float(seq), y=0.0,
                yaw=0.0, timeout=timeout, on_failure=on_failure,
                xy_tolerance=0.1, yaw_tolerance=0.1)


def C(mode='multi', loop=False, on_failure='stop', frame_id='map', enabled=True):
    return MissionConfig(mode=mode, loop=loop, on_failure=on_failure,
                         frame_id=frame_id, robot_base_frame='base_footprint',
                         enabled=enabled)


class TestReport(unittest.TestCase):
    def test_counts(self):
        e = MissionEngine(C(), [G(1), G(2)])
        e.start(0.0)
        e.record(G(1), 'success', 1.0)
        e.record(G(2), 'failed', 2.0)
        r = rep.build_report(e, goal_file='x.yaml')
        self.assertEqual(r['mission']['counts']['success'], 1)
        self.assertEqual(r['mission']['counts']['failed'], 1)
        self.assertEqual(r['mission']['total_goals'], 2)
        self.assertEqual(r['mission']['state'], 'aborted')

    def test_checkpoint_roundtrip(self):
        e = MissionEngine(C(), [G(1), G(2), G(3)])
        e.start(0.0)
        e.record(G(1), 'success', 1.0)
        cp = rep.build_checkpoint(e, 'x.yaml', e.index)
        cfg, goals, records, idx, state = rep.parse_checkpoint(cp)
        self.assertEqual(cfg.mode, 'multi')
        self.assertEqual(len(goals), 3)
        self.assertEqual(idx, 1)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['status'], 'success')

    def test_checkpoint_goal_roundtrip(self):
        g = G(2, on_failure='skip', timeout=15.0)
        g2 = Goal.from_dict(g.to_dict())
        self.assertEqual(g2.seq, 2)
        self.assertEqual(g2.on_failure, 'skip')
        self.assertEqual(g2.timeout, 15.0)

    def test_parse_checkpoint_rejects_bad_version(self):
        from racecar_mission.validate import ValidationError
        with self.assertRaises(ValidationError):
            rep.parse_checkpoint({'version': 2})

    def test_parse_checkpoint_rejects_duplicate_seq(self):
        from racecar_mission.validate import ValidationError
        cp = rep.build_checkpoint(MissionEngine(C(), [G(1)]), 'x.yaml', 0)
        cp['goals'] = [G(1).to_dict(), G(1).to_dict()]
        with self.assertRaises(ValidationError):
            rep.parse_checkpoint(cp)

    def test_dispatched_accepted_terminal_counts(self):
        # 修复：dispatched=实际发出的目标数，不含 not_attempted；accepted/terminal 独立计数。
        from racecar_mission.validate import ValidationError  # noqa: F401
        e = MissionEngine(C(), [G(1), G(2), G(3)])
        e.start(0.0)
        e.record(G(1), 'failed', 1.0)   # on_failure=stop -> aborted
        r = rep.build_report(e)
        self.assertEqual(r['mission']['dispatched'], 1)
        self.assertEqual(r['mission']['accepted'], 0)
        self.assertEqual(r['mission']['terminal'], 1)
        self.assertEqual(r['mission']['not_attempted'], 2)

    def test_parse_checkpoint_rejects_nan_coords(self):
        from racecar_mission.validate import ValidationError
        cp = rep.build_checkpoint(MissionEngine(C(), [G(1)]), 'x.yaml', 0)
        cp['goals'][0]['x'] = float('nan')
        with self.assertRaises(ValidationError):
            rep.parse_checkpoint(cp)

    def test_parse_checkpoint_rejects_bad_frame(self):
        from racecar_mission.validate import ValidationError
        cp = rep.build_checkpoint(MissionEngine(C(), [G(1)]), 'x.yaml', 0)
        cp['goals'][0]['frame_id'] = 'not_allowed'
        with self.assertRaises(ValidationError):
            rep.parse_checkpoint(cp)

    def test_parse_checkpoint_rejects_string_false_enabled(self):
        from racecar_mission.validate import ValidationError
        cp = rep.build_checkpoint(MissionEngine(C(), [G(1)]), 'x.yaml', 0)
        cp['enabled'] = 'false'
        with self.assertRaises(ValidationError):
            rep.parse_checkpoint(cp)

    def test_parse_checkpoint_rejects_bad_next_index(self):
        from racecar_mission.validate import ValidationError
        cp = rep.build_checkpoint(MissionEngine(C(), [G(1)]), 'x.yaml', 0)
        cp['next_index'] = 999
        with self.assertRaises(ValidationError):
            rep.parse_checkpoint(cp)

    def test_parse_checkpoint_rejects_not_attempted_in_nonterminal(self):
        from racecar_mission.validate import ValidationError
        e = MissionEngine(C(), [G(1), G(2)])
        e.start(0.0)
        e.record(G(1), 'failed', 1.0)   # aborted，含 not_attempted
        cp = rep.build_checkpoint(e, 'x.yaml', e.index)
        cp['state'] = 'running'         # 伪装成可恢复，但含 not_attempted -> 拒收
        with self.assertRaises(ValidationError):
            rep.parse_checkpoint(cp)

    def test_parse_checkpoint_accepts_final_state(self):
        e = MissionEngine(C(), [G(1), G(2)])
        e.start(0.0)
        e.record(G(1), 'failed', 1.0)   # aborted，index 停在失败的 g1（0），g2 记 not_attempted
        cp = rep.build_checkpoint(e, 'x.yaml', e.index)
        cfg, goals, records, idx, state = rep.parse_checkpoint(cp)
        self.assertEqual(state, 'aborted')
        self.assertEqual(idx, 0)
        self.assertEqual(records[-1]['status'], 'not_attempted')


if __name__ == '__main__':
    unittest.main()
