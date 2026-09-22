import json
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for package in ('track_generator', 'racecar_strategy'):
    sys.path.insert(0, str(ROOT / 'hwcart_sim/src' / package))
from track_generator.generate import load_cones, build
from racecar_strategy.rules import Ledger, effective_time, score, parking_success


class RulesTests(unittest.TestCase):
    def test_formula_subtracts_both_unweighted_waits(self):
        self.assertEqual(effective_time(100, 60, 5, 3, 10), 64)

    def test_invalid_wait(self):
        with self.assertRaises(ValueError):
            effective_time(2, 60, 0, 3, 0)

    def test_score_endpoints_and_degenerate_field(self):
        self.assertEqual(score(10, 10, 20, 20), 100)
        self.assertEqual(score(20, 10, 20, 20), 20)
        with self.assertRaises(ValueError):
            score(10, 10, 10, 20)

    def test_line_penalty_once_per_lap(self):
        ledger = Ledger()
        for event, lap in [('a', 1), ('b', 1), ('c', 2)]:
            ledger.record(event, 'zigzag_line', 1, lap)
        self.assertEqual(ledger.penalty_seconds, 20)

    def test_deduplication_and_wrong_cone_threshold(self):
        ledger = Ledger()
        for i in range(3):
            ledger.record(str(i), 'wrong_cone', i, 1)
        self.assertFalse(ledger.failed)
        self.assertFalse(ledger.record('2', 'wrong_cone', 3, 1))
        ledger.record('3', 'wrong_cone', 3, 2)
        self.assertTrue(ledger.failed)
        self.assertEqual(ledger.penalty_seconds, 20)

    def test_stop_threshold(self):
        self.assertFalse(Ledger().check_timeout(stopped_seconds=10))
        self.assertTrue(Ledger().check_timeout(stopped_seconds=10.001))
        self.assertTrue(Ledger().check_timeout(offtrack_seconds=10.001))

    def test_clock_reset_requires_new_ledger(self):
        ledger = Ledger()
        ledger.record('a', 'cone_collision', 10, 1)
        with self.assertRaises(ValueError):
            ledger.record('b', 'cone_collision', 1, 1)

    def test_parking(self):
        self.assertTrue(parking_success([True, True, True, False], 'A', 'A'))
        self.assertFalse(parking_success([True, True, False, False], 'A', 'A'))
        self.assertFalse(parking_success([True]*4, 'A', 'B'))


class TrackTests(unittest.TestCase):
    def setUp(self):
        self.cones = load_cones(ROOT / 'examples/cones.csv')
        self.gaps = json.loads((ROOT / 'examples/gaps.json').read_text())

    def test_reproducible_world_and_three_unique_boards(self):
        first = build(self.cones, self.gaps, 42, .02)
        self.assertEqual(first, build(self.cones, list(reversed(self.gaps)), 42, .02))
        xml = ET.fromstring(first[0])
        boards = [m for m in xml.findall('world/model') if m.attrib['name'].startswith('board_')]
        self.assertEqual(len(boards), 3)
        self.assertEqual(len(set(first[1]['selected_gaps'])), 3)

    def test_invalid_candidate_sets(self):
        for gaps in (self.gaps[:2], self.gaps + [self.gaps[0]], [['r0', 'b0']] + self.gaps):
            with self.assertRaises(ValueError):
                build(self.cones, gaps, 0, .02)

    def test_invalid_thickness(self):
        for thickness in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                build(self.cones, self.gaps, 0, thickness)

    def test_seed_sweep(self):
        choices = set()
        for seed in range(100):
            _, manifest = build(self.cones, self.gaps, seed, .02)
            self.assertEqual(len(set(manifest['selected_gaps'])), 3)
            choices.add(tuple(manifest['selected_gaps']))
        self.assertGreater(len(choices), 1)


if __name__ == '__main__':
    unittest.main()
