import pytest
pytest.importorskip("lerobot", reason="Optional LeRobot commissioning SDK is not installed")

#!/usr/bin/env python3

import unittest
from unittest.mock import MagicMock, patch

from calibrate_so101 import (
    FOLLOWER_LEFT_PROFILE,
    FOLLOWER_PROFILE,
    FOLLOWER_RIGHT_PROFILE,
    FOLLOWER_PROFILES,
    LEADER_PROFILE,
    build_motor_map,
    calibrate_without_motion,
    parse_args,
)


class CalibrationProfileTest(unittest.TestCase):
    def test_cli_requires_explicit_follower_side(self) -> None:
        with patch('sys.argv', ['calibrate', '--arm', 'follower',
                                '--port', '/unused', '--id', 'test']):
            with self.assertRaises(SystemExit):
                parse_args()

    def test_cli_selects_physical_sides(self) -> None:
        for side, ids in [('left', list(range(1, 7))),
                          ('right', list(range(7, 13)))]:
            with patch('sys.argv', ['calibrate', '--arm', 'follower',
                                    '--port', '/unused', '--id', 'test',
                                    '--follower-side', side, '--dry-run']):
                args = parse_args()
            self.assertEqual(
                [m.motor_id for m in FOLLOWER_PROFILES[args.follower_side].motors], ids,
            )

    def test_leader_uses_six_sts3215_motors(self) -> None:
        self.assertEqual(
            [(motor.motor_id, motor.model) for motor in LEADER_PROFILE.motors],
            [
                (1, "sts3215"),
                (2, "sts3215"),
                (3, "sts3215"),
                (4, "sts3215"),
                (5, "sts3215"),
                (6, "sts3215"),
            ],
        )

    def test_follower_uses_sts3250_for_load_bearing_joints(self) -> None:
        motors = build_motor_map(FOLLOWER_PROFILE)
        self.assertEqual(motors["shoulder_lift"].model, "sts3250")
        self.assertEqual(motors["elbow_flex"].model, "sts3250")
        self.assertEqual(motors["shoulder_pan"].model, "sts3215")
        self.assertEqual(motors["wrist_flex"].model, "sts3215")
        self.assertEqual(motors["wrist_roll"].model, "sts3215")
        self.assertEqual(motors["gripper"].model, "sts3215")

    def test_motor_ids_cover_one_through_six(self) -> None:
        self.assertEqual([motor.motor_id for motor in FOLLOWER_PROFILE.motors], list(range(1, 7)))

    def test_physical_sides_match_motor_ids(self) -> None:
        self.assertIs(FOLLOWER_PROFILES["left"], FOLLOWER_LEFT_PROFILE)
        self.assertIs(FOLLOWER_PROFILES["right"], FOLLOWER_RIGHT_PROFILE)
        self.assertEqual(
            [motor.motor_id for motor in FOLLOWER_LEFT_PROFILE.motors],
            list(range(1, 7)),
        )
        self.assertEqual(
            [motor.motor_id for motor in FOLLOWER_RIGHT_PROFILE.motors],
            list(range(7, 13)),
        )
        self.assertEqual(
            [motor.model for motor in FOLLOWER_LEFT_PROFILE.motors],
            [motor.model for motor in FOLLOWER_RIGHT_PROFILE.motors],
        )


class CalibrationLifecycleTest(unittest.TestCase):
    def make_device(self):
        device = MagicMock()
        device.bus.is_connected = False
        device.bus.motors = build_motor_map(FOLLOWER_LEFT_PROFILE)
        device.connect.side_effect = AssertionError('Robot setup can enable torque')
        device.bus.enable_torque.side_effect = AssertionError('Calibration must keep torque OFF')
        device.bus.connect.side_effect = lambda: setattr(device.bus, 'is_connected', True)
        device.bus.disconnect.side_effect = lambda **kwargs: setattr(device.bus, 'is_connected', False)
        return device

    def test_manual_calibration_keeps_torque_off_and_relocks_selected_motors(self):
        device = self.make_device()

        def manual_calibration():
            self.assertTrue(device.bus.is_connected)
            self.assertGreaterEqual(device.bus.disable_torque.call_count, 1)

        device.calibrate.side_effect = manual_calibration
        calibrate_without_motion(device)
        device.connect.assert_not_called()
        device.configure.assert_not_called()
        device.bus.enable_torque.assert_not_called()
        self.assertFalse(device.bus.is_connected)
        self.assertEqual(
            {(call.args[0], call.args[1], call.args[2]) for call in device.bus.write.call_args_list},
            {('Lock', motor, 1) for motor in device.bus.motors},
        )
        device.bus.disconnect.assert_called_once_with(disable_torque=False)

    def test_interrupted_calibration_closes_port_with_torque_off_and_eeprom_locked(self):
        device = self.make_device()
        device.calibrate.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            calibrate_without_motion(device)
        device.bus.enable_torque.assert_not_called()
        self.assertEqual(device.bus.disable_torque.call_count, 2)
        self.assertEqual(device.bus.write.call_count, 6)
        self.assertFalse(device.bus.is_connected)

    def test_cleanup_failure_still_closes_port(self):
        device = self.make_device()
        device.bus.write.side_effect = RuntimeError('Lock write failed')
        with self.assertRaisesRegex(RuntimeError, 'Lock write failed'):
            calibrate_without_motion(device)
        self.assertFalse(device.bus.is_connected)
        device.bus.disconnect.assert_called_once_with(disable_torque=False)


if __name__ == "__main__":
    unittest.main()
