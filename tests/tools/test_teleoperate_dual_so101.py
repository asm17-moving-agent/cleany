import pytest
pytest.importorskip("lerobot", reason="Optional LeRobot commissioning SDK is not installed")

#!/usr/bin/env python3

import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from teleoperate_dual_so101 import (
    JOINTS,
    load_dual_calibration,
    motor_map,
    position_deltas,
    set_follower_torque,
)


def _write_calibration(path: Path, first_id: int) -> None:
    path.write_text(json.dumps({
        name: {
            "id": first_id + offset,
            "drive_mode": 0,
            "homing_offset": offset,
            "range_min": 100,
            "range_max": 3000,
        }
        for offset, name in enumerate(JOINTS)
    }))


class DualTeleoperateTest(unittest.TestCase):
    def test_torque_cleanup_attempts_every_motor_after_write_failure(self) -> None:
        class FailingBus:
            motors = {"motor_1": object(), "motor_2": object(), "motor_3": object()}

            def __init__(self) -> None:
                self.writes: list[tuple[str, str, int]] = []

            def write(self, register: str, motor: str, value: int, **_kwargs: object) -> None:
                self.writes.append((register, motor, value))
                if register == "Torque_Enable" and motor == "motor_2":
                    raise OSError("test failure")

        bus = FailingBus()
        with self.assertRaisesRegex(RuntimeError, "motor_2 torque"):
            set_follower_torque(bus, False)  # type: ignore[arg-type]
        attempted = [
            motor for register, motor, value in bus.writes
            if register == "Torque_Enable" and value == 0
        ]
        self.assertEqual(attempted, list(bus.motors))

    def test_loads_shared_bus_calibrations(self) -> None:
        with TemporaryDirectory() as directory:
            left = Path(directory) / "left.json"
            right = Path(directory) / "right.json"
            _write_calibration(left, 1)
            _write_calibration(right, 7)
            calibration = load_dual_calibration(left, right)
        self.assertEqual(calibration["left_shoulder_pan"].id, 1)
        self.assertEqual(calibration["right_gripper"].id, 12)

    def test_rejects_wrong_side_ids(self) -> None:
        with TemporaryDirectory() as directory:
            left = Path(directory) / "left.json"
            right = Path(directory) / "right.json"
            _write_calibration(left, 1)
            _write_calibration(right, 1)
            with self.assertRaisesRegex(ValueError, "right calibration IDs"):
                load_dual_calibration(left, right)

    def test_motor_models_match_cleany_hardware(self) -> None:
        leaders = motor_map("leader")
        followers = motor_map("follower")
        self.assertEqual({motor.id for motor in leaders.values()}, set(range(1, 13)))
        self.assertEqual(leaders["right_shoulder_lift"].model, "sts3215")
        self.assertEqual(followers["left_shoulder_lift"].model, "sts3250")
        self.assertEqual(followers["right_elbow_flex"].model, "sts3250")

    def test_position_deltas_cover_both_sides(self) -> None:
        leader = {"left_joint": 3.0, "right_joint": -2.0}
        follower = {"left_joint": 1.0, "right_joint": 4.0}
        self.assertEqual(position_deltas(leader, follower), {
            "left_joint": 2.0,
            "right_joint": 6.0,
        })


if __name__ == "__main__":
    unittest.main()
