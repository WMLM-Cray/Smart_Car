import math
import unittest
from types import SimpleNamespace as NS
from scan_view import scan_payload


def message(ranges):
    return NS(ranges=ranges, range_min=0.1, range_max=20., angle_min=0.,
              angle_increment=math.pi/2,
              header=NS(frame_id='laser_link', stamp=NS(sec=1, nanosec=500000000)))


class ScanViewTests(unittest.TestCase):
    def test_axes_and_timestamp(self):
        result = scan_payload(message([1., 2., 3., 4.]))
        self.assertEqual(result['points'], [[1., 0.], [0., 2.], [-3., 0.], [0., -4.]])
        self.assertEqual(result['stamp'], 1.5)

    def test_invalid_and_empty(self):
        self.assertEqual(scan_payload(message([math.inf, math.nan, 0., 21.]))['points'], [])
        self.assertEqual(scan_payload(message([]))['points'], [])

    def test_preserve_normal_scan_and_bound_size(self):
        self.assertEqual(len(scan_payload(message([1.]*1667))['points']), 1667)
        self.assertLessEqual(len(scan_payload(message([1.]*10000))['points']), 2000)


if __name__ == '__main__':
    unittest.main()
