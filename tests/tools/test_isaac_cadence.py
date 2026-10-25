"""Verify aligned publications and odometry over decimated physics intervals."""
import importlib.util
import math
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('isaac_cadence',
    Path(__file__).resolve().parents[2]/'tools/validation/isaac/cadence.py')
cadence = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cadence)


def pose(x, y, angle):
    return [x, y, 0., 0., 0., math.sin(angle/2), math.cos(angle/2)]


class CadenceTest(unittest.TestCase):
    def test_counts_and_scan_transform_alignment(self):
        for profile in ('legacy', 'split'):
            values = cadence.publication_config(profile, 500)
            ticks = {name: {tick for tick in range(500) if tick % (500//values[name]) == 0}
                     for name in ('clock_hz', 'odometry_hz', 'scan_hz')}
            for name in ticks:
                self.assertEqual(len(ticks[name]), values[name])
            self.assertLessEqual(ticks['scan_hz'], ticks['odometry_hz'])
            self.assertLessEqual(ticks['odometry_hz'], ticks['clock_hz'])

    def test_no_rounded_nonintegral_period(self):
        with self.assertRaises(ValueError):
            cadence.publication_config('split', 480)

    def test_full_20ms_interval_and_body_frame(self):
        vx, vy, wz = cadence.measured_twist(pose(1., 2., math.pi/2),
                                          pose(1.002, 2.004, math.pi/2), .02)
        self.assertAlmostEqual(vx, .2)
        self.assertAlmostEqual(vy, -.1)
        self.assertAlmostEqual(wz, 0.)

    def test_yaw_wrap_and_nonincreasing_time(self):
        self.assertAlmostEqual(cadence.measured_twist(pose(0, 0, math.pi-.001),
                                                     pose(0, 0, -math.pi+.001), .02)[2], .1)
        with self.assertRaises(ValueError):
            cadence.measured_twist(pose(0, 0, 0), pose(0, 0, 0), 0.)

    def test_received_rate_exposes_dropped_and_duplicate_messages(self):
        stats = cadence.StampStats()
        for ns in (0, 20_000_000, 60_000_000):
            stats.observe(1, ns)
        self.assertAlmostEqual(stats.result()['received_hz_sim'], 100/3)
        stats.observe(1, 60_000_000)
        self.assertEqual(stats.result()['nonincreasing_stamps'], 1)


if __name__ == '__main__':
    unittest.main()
