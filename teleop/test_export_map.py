import unittest
from types import SimpleNamespace as NS
from export_map import encode_map


class ExportTests(unittest.TestCase):
    def test_flip_and_unknown(self):
        msg = NS(info=NS(width=2, height=2, resolution=.05,
                        origin=NS(position=NS(x=-1., y=-2.), orientation=NS(x=0., y=0., z=0., w=1.))),
                 data=[0, 100, -1, 50])
        pgm, yaml = encode_map(msg)
        self.assertEqual(pgm, b'P5\n2 2\n255\n' + bytes([205,205,254,0]))
        self.assertIn('origin: [-1.0, -2.0, 0.0]', yaml)
        msg.data = []
        with self.assertRaises(ValueError):
            encode_map(msg)
