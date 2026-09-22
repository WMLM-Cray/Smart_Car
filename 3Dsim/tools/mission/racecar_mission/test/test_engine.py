import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from racecar_mission.engine import MissionEngine
from racecar_mission.goal import Goal, MissionConfig


def G(seq, on_failure='stop', timeout=None, frame_id='map', xy=0.0, yaw_tol=0.0):
    return Goal(seq=seq, name='g%d' % seq, frame_id=frame_id, x=float(seq), y=0.0,
                yaw=0.0, timeout=timeout, on_failure=on_failure,
                xy_tolerance=xy, yaw_tolerance=yaw_tol)


def C(mode='multi', loop=False, on_failure='stop', frame_id='map',
      robot_base_frame='base_footprint', enabled=True):
    return MissionConfig(mode=mode, loop=loop, on_failure=on_failure,
                         frame_id=frame_id, robot_base_frame=robot_base_frame,
                         enabled=enabled)


class TestEngine(unittest.TestCase):
    def test_multi_all_success(self):
        e = MissionEngine(C(), [G(1), G(2), G(3)])
        self.assertEqual(e.start(0.0).seq, 1)
        self.assertEqual(e.record(G(1), 'success', 1.0).seq, 2)
        self.assertEqual(e.record(G(2), 'success', 2.0).seq, 3)
        self.assertIsNone(e.record(G(3), 'success', 3.0))
        self.assertEqual(e.state, 'completed')
        self.assertEqual([r['status'] for r in e.records], ['success'] * 3)

    def test_single_mode_marks_rest(self):
        e = MissionEngine(C(mode='single'), [G(1), G(2), G(3)])
        self.assertEqual(e.start(0.0).seq, 1)
        self.assertIsNone(e.record(G(1), 'success', 1.0))
        self.assertEqual(e.state, 'completed')
        self.assertEqual([r['status'] for r in e.records],
                         ['success', 'not_attempted', 'not_attempted'])

    def test_stop_on_failure(self):
        e = MissionEngine(C(on_failure='stop'), [G(1), G(2), G(3)])
        e.start(0.0)
        e.record(G(1), 'success', 1.0)
        self.assertIsNone(e.record(G(2), 'failed', 2.0))
        self.assertEqual(e.state, 'aborted')
        self.assertEqual([r['status'] for r in e.records],
                         ['success', 'failed', 'not_attempted'])

    def test_skip_on_failure(self):
        e = MissionEngine(C(on_failure='skip'),
                          [G(1, on_failure='skip'), G(2, on_failure='skip'), G(3, on_failure='skip')])
        e.start(0.0)
        e.record(G(1, on_failure='skip'), 'success', 1.0)
        nxt = e.record(G(2, on_failure='skip'), 'failed', 2.0)
        self.assertEqual(nxt.seq, 3)
        self.assertIsNone(e.record(G(3, on_failure='skip'), 'success', 3.0))
        self.assertEqual(e.state, 'completed')
        self.assertEqual([r['status'] for r in e.records],
                         ['success', 'failed', 'success'])

    def test_per_goal_skip_override(self):
        # 任务级 stop，但 g2 自己 skip
        e = MissionEngine(C(on_failure='stop'), [G(1), G(2, on_failure='skip'), G(3)])
        e.start(0.0)
        e.record(G(1), 'success', 1.0)
        nxt = e.record(G(2, on_failure='skip'), 'failed', 2.0)
        self.assertEqual(nxt.seq, 3)

    def test_timeout_is_failure(self):
        e = MissionEngine(C(on_failure='stop'), [G(1), G(2)])
        e.start(0.0)
        self.assertIsNone(e.record(G(1), 'timeout', 1.0))
        self.assertEqual(e.state, 'aborted')

    def test_loop(self):
        e = MissionEngine(C(loop=True), [G(1), G(2)])
        e.start(0.0)
        e.record(G(1), 'success', 1.0)
        e.record(G(2), 'success', 2.0)
        self.assertEqual(e.laps, 1)
        self.assertEqual(e.current().seq, 1)   # 回卷到 g1
        self.assertEqual(e.state, 'running')

    def test_pause_resume(self):
        e = MissionEngine(C(), [G(1), G(2), G(3)])
        e.start(0.0)
        self.assertTrue(e.pause())
        self.assertIsNone(e.record(G(1), 'success', 1.0))  # 完成后停住
        self.assertEqual(e.state, 'paused')
        self.assertTrue(e.resume())
        self.assertEqual(e.state, 'running')
        self.assertEqual(e.current().seq, 2)   # 继续派发 g2

    def test_cancel_active_goal(self):
        e = MissionEngine(C(), [G(1), G(2), G(3)])
        e.start(0.0)
        e.record(G(1), 'success', 1.0)
        self.assertIsNone(e.record(G(2), 'canceled', 2.0, force_stop=True,
                                   stop_reason='mission canceled by user'))
        self.assertEqual(e.state, 'canceled')
        self.assertEqual([r['status'] for r in e.records],
                         ['success', 'canceled', 'not_attempted'])

    def test_cancel_idle(self):
        e = MissionEngine(C(), [G(1), G(2)])
        e.start(0.0)
        e.pause()
        e.record(G(1), 'success', 1.0)   # 停住，index=1
        e.cancel(2.0, 'mission canceled by user')
        self.assertEqual(e.state, 'canceled')
        self.assertEqual(e.records[-1]['status'], 'not_attempted')
        self.assertEqual(e.records[-1]['seq'], 2)

    def test_localization_lost_force_stop(self):
        e = MissionEngine(C(on_failure='skip'), [G(1), G(2), G(3)])
        e.start(0.0)
        e.record(G(1), 'success', 1.0)
        self.assertIsNone(e.record(G(2), 'failed', 2.0, force_stop=True,
                                   stop_reason='localization lost',
                                   reason='localization lost'))
        self.assertEqual(e.state, 'aborted')
        self.assertEqual([r['status'] for r in e.records],
                         ['success', 'failed', 'not_attempted'])
        self.assertEqual(e.stop_reason, 'localization lost')

    def test_restore(self):
        e = MissionEngine(C(), [G(1), G(2), G(3)])
        e.start(0.0)
        e.record(G(1), 'success', 1.0)
        idx, records = e.index, list(e.records)
        # 从检查点恢复
        e2 = MissionEngine(C(), [G(1), G(2), G(3)])
        self.assertEqual(e2.restore(records, idx, started_at=1.0).seq, 2)
        self.assertEqual(e2.current().seq, 2)
        self.assertEqual(len(e2.records), 1)

    def test_record_fields_preserved(self):
        e = MissionEngine(C(), [G(1)])
        e.start(0.0)
        e.record(G(1), 'success', 1.0, send_time=0.0, elapsed_s=1.0,
                 position_error_m=0.05, yaw_error_rad=0.01, recoveries=2,
                 within_tolerance=True, nav_status=4, nav_error_code=0)
        r = e.records[0]
        self.assertEqual(r['position_error_m'], 0.05)
        self.assertEqual(r['recoveries'], 2)
        self.assertTrue(r['within_tolerance'])

    def test_external_canceled_is_terminal(self):
        # 修复：收到外部 CANCELED 结果也终止整段（不默认跳过被取消点）。
        e = MissionEngine(C(), [G(1), G(2), G(3)])
        e.start(0.0)
        self.assertIsNone(e.record(G(1), 'canceled', 1.0))
        self.assertEqual(e.state, 'canceled')
        self.assertEqual([r['status'] for r in e.records],
                         ['canceled', 'not_attempted', 'not_attempted'])

    def test_fault_locks_and_records_active_goal(self):
        # 故障锁定时，在途目标结局未确认，记 unconfirmed（不是 failed/canceled，避免
        # 谎称“旧目标已结束/车辆已停”），其余 not_attempted，终态 fault。
        e = MissionEngine(C(), [G(1), G(2), G(3)])
        e.start(0.0)
        e.record(G(1), 'success', 1.0)
        e.fault(2.0, 'cancel unconfirmed', active_goal=G(2), accepted=True)
        self.assertEqual(e.state, 'fault')
        self.assertEqual([r['status'] for r in e.records],
                         ['success', 'unconfirmed', 'not_attempted'])
        self.assertEqual(e.stop_reason, 'cancel unconfirmed')
        self.assertEqual(e.records[1]['reason'], 'cancel unconfirmed')
        self.assertTrue(e.records[1]['accepted'])

    def test_fault_goal_status_is_unconfirmed_not_failed(self):
        # 取消被拒/未确认时，在途目标绝不记 failed/canceled/success，而是 unconfirmed。
        e = MissionEngine(C(), [G(1)])
        e.start(0.0)
        e.fault(1.0, 'cancel rejected', active_goal=G(1))
        self.assertEqual(e.records[0]['status'], 'unconfirmed')
        self.assertNotIn(e.records[0]['status'], ('failed', 'canceled', 'success'))

    def test_fault_no_active_goal(self):
        e = MissionEngine(C(), [G(1), G(2)])
        e.start(0.0)
        e.fault(1.0, 'action server disappeared')
        self.assertEqual(e.state, 'fault')
        self.assertEqual([r['status'] for r in e.records],
                         ['not_attempted', 'not_attempted'])

    def test_fault_is_terminal_not_canceled(self):
        # fault 终态不是 canceled：不把未确认取消伪装成已安全停止。
        e = MissionEngine(C(), [G(1)])
        e.start(0.0)
        e.fault(1.0, 'cancel rejected')
        self.assertIn(e.state, ('fault',))
        self.assertNotIn(e.state, ('canceled', 'aborted'))


if __name__ == '__main__':
    unittest.main()
