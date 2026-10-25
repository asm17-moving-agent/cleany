"""Stationary confirmation must work below real time without accepting stale data."""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]/'tools/validation/isaac'))
from cadence import confirmation_window


class ConfirmationTest(unittest.TestCase):
    def test_same_simulation_evidence_at_different_real_time_factors(self):
        for rtf in [1., .3, .1]:
            samples = [(i*.02/rtf, i*.02, i) for i in range(51)]
            window = confirmation_window(samples, samples[-1][0]+.01)
            self.assertTrue(window)
            self.assertGreaterEqual(window[-1][1]-window[0][1], .5)
            self.assertEqual(window[-1][2], 50)

    def test_stale_or_insufficient_stream_is_not_confirmation(self):
        samples = [(i*.2, i*.02, i) for i in range(31)]
        self.assertFalse(confirmation_window(samples, samples[-1][0]+1.))
        self.assertFalse(confirmation_window(samples[:20], samples[19][0]))
        self.assertFalse(confirmation_window([], 1.))


if __name__ == '__main__':
    unittest.main()
