import itertools
import unittest
from cleany_teleop.core import joystick_targets, wheel_targets


class JoystickTests(unittest.TestCase):
    def test_cardinal_axes_match_existing_wheel_order(self):
        for vector, direction in [
            ([1, 0, 0], "forward"),
            ([-1, 0, 0], "backward"),
            ([0, 1, 0], "left"),
            ([0, -1, 0], "right"),
            ([0, 0, 1], "ccw"),
            ([0, 0, -1], "cw"),
        ]:
            self.assertEqual(joystick_targets(vector, 1), wheel_targets(direction, 1))

    def test_combined_axes_preserve_ratios_and_wheel_speed_limit(self):
        for vector in itertools.product((-1, -0.5, 0, 0.5, 1), repeat=3):
            targets = joystick_targets(vector, 2)
            self.assertLessEqual(max(map(abs, targets)), 2)
            self.assertTrue(all(type(v) is float for v in targets))
        self.assertEqual(joystick_targets([1, 1, 1], 2), (-2 / 3, 2, 2 / 3, 2 / 3))
