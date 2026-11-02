from types import SimpleNamespace
import unittest

from cleany_teleop.core import FeedbackClock, wheel_targets


def state(sequence=10, timestamp_us=1_000_000, boot_id=123):
    return SimpleNamespace(
        protocol_version=2,
        boot_id=boot_id,
        sequence=sequence,
        timestamp_us=timestamp_us,
    )


class ClockTests(unittest.TestCase):
    def test_deadline_uses_mcu_clock_and_rejects_stale_feedback(self):
        clock = FeedbackClock()
        clock.observe(state(), 5000.0)
        self.assertEqual(clock.deadline(5000.02, 250), 1_170_000)
        with self.assertRaises(RuntimeError):
            clock.deadline(5000.11, 250)

    def test_duplicate_does_not_refresh_freshness(self):
        clock = FeedbackClock()
        clock.observe(state(), 1.0)
        self.assertFalse(clock.observe(state(), 1.09))
        with self.assertRaises(RuntimeError):
            clock.require_fresh(1.11)

    def test_reboot_and_clock_regression_abort(self):
        for changed in [state(11, boot_id=124), state(11, timestamp_us=999_999)]:
            clock = FeedbackClock()
            clock.observe(state(), 1.0)
            with self.assertRaises(RuntimeError):
                clock.observe(changed, 1.01)

    def test_command_sequence_seeds_high_counter_and_wraps(self):
        clock = FeedbackClock()
        self.assertEqual(clock.next_sequence(0xFFFFFFFE), 0xFFFFFFFF)
        self.assertEqual(clock.next_sequence(0xFFFFFFFE), 0)
        self.assertEqual(clock.next_sequence(0), 1)

    def test_wheel_speed_validation_and_logical_order(self):
        self.assertEqual(wheel_targets("left", 0.5), (-0.5, 0.5, 0.5, -0.5))
        for speed in [float("nan"), float("inf"), -1, 0, 10.1]:
            with self.assertRaises(ValueError):
                wheel_targets("forward", speed)

    def test_integer_web_speed_produces_ros_float_values(self):
        targets = wheel_targets("forward", 1)
        self.assertEqual(targets, (1.0,) * 4)
        self.assertTrue(all(type(value) is float for value in targets))


if __name__ == "__main__":
    unittest.main()
