import subprocess
import unittest
from unittest.mock import patch
from mapping_control import MappingControl


class MappingTests(unittest.TestCase):
    def test_already_running_does_not_spawn(self):
        manager = MappingControl(lambda: 1)
        with patch('mapping_control.mapper_pids', return_value=[123]), patch('mapping_control.subprocess.Popen') as spawn:
            manager._start()
            spawn.assert_not_called()

    def test_other_mapper_blocks_start(self):
        with patch('mapping_control.mapper_pids', return_value=[]), patch('mapping_control.subprocess.Popen') as spawn:
            with self.assertRaises(RuntimeError):
                MappingControl(lambda: 1)._start()
            spawn.assert_not_called()

    def test_failed_export_never_stops(self):
        with patch('mapping_control.mapper_pids', return_value=[123]), \
             patch('mapping_control.os.pidfd_open', return_value=42, create=True), \
             patch('mapping_control.os.close'), \
             patch('mapping_control.subprocess.run', side_effect=subprocess.TimeoutExpired('export', 30)), \
             patch('mapping_control.signal.pidfd_send_signal', create=True) as stop:
            manager = MappingControl(lambda: 1)
            manager._work('stop')
            stop.assert_not_called()
            self.assertIn('操作失败', manager.message)
            self.assertFalse(manager.busy)

    def test_duplicate_mappers_never_stop(self):
        with patch('mapping_control.mapper_pids', return_value=[123, 456]):
            with self.assertRaises(RuntimeError):
                MappingControl(lambda: 2)._stop()
