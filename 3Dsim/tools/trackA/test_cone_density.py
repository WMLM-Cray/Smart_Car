"""Verify every cone in rule Fig. 9A against the committed data."""

import csv
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fig9 = load(Path(__file__).with_name("fig9a_cones.py"), "fig9a_cones")


class ConePlacementTest(unittest.TestCase):
    def test_committed_csv_matches_generator(self):
        expected = list(fig9.rows())
        with (ROOT / "docs/track/trackA_cones.csv").open(newline="", encoding="utf-8") as f:
            actual = list(csv.DictReader(f))
        self.assertEqual((len(fig9.BLUE), len(fig9.RED), len(actual)), (36, 35, 71))
        self.assertEqual(len({(r["color"], r["source_px_x"], r["source_px_y"])
                              for r in expected}), 71)
        for row, cone in zip(actual, expected):
            self.assertEqual((row["id"], row["color"], row["segment"]),
                             (cone["id"], cone["color"], cone["segment"]))
            self.assertAlmostEqual(float(row["x"]), cone["x"], places=6)
            self.assertAlmostEqual(float(row["y"]), cone["y"], places=6)

    def test_spacing_and_yellow_tape(self):
        cones = list(fig9.rows())
        self.assertFalse(any(c["segment"] in {"E3", "E4", "I3", "I4"} for c in cones))
        for color in ("blue", "red"):
            ordered = [c for c in cones if c["color"] == color]
            for a, b in zip(ordered, ordered[1:] + ordered[:1]):
                if (a["segment"], b["segment"]) in (("E2", "E5"), ("I2", "I5")):
                    continue
                gap = math.hypot(b["x"]-a["x"], b["y"]-a["y"])
                self.assertLess(gap, 1.5)

    def test_second_generator_agrees(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "cones.csv"
            subprocess.run([sys.executable, str(ROOT / "3Dsim/tools/gen_trackA_cones.py"),
                            str(ROOT / "docs/track/trackA_layout.yaml"), "-o", str(output)],
                           check=True, capture_output=True, text=True)
            self.assertEqual(output.read_bytes(),
                             (ROOT / "docs/track/trackA_cones.csv").read_bytes())

    def test_example_layout_is_current_copy(self):
        master = (ROOT / "docs/track/trackA_layout.yaml").read_text(encoding="utf-8")
        example = (ROOT / "3Dsim/examples/trackA_layout.yaml").read_text(encoding="utf-8")
        self.assertTrue(example.endswith(master))


if __name__ == "__main__":
    unittest.main()
