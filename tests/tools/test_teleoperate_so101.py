import pytest
pytest.importorskip("lerobot", reason="Optional LeRobot commissioning SDK is not installed")

#!/usr/bin/env python3

import unittest
from unittest.mock import patch

from teleoperate_so101 import parse_args, position_deltas


class TeleoperateSafetyTest(unittest.TestCase):
    def test_follower_side_is_required(self) -> None:
        with patch('sys.argv', ['teleoperate', '--leader-port', '/leader',
                                '--follower-port', '/follower',
                                '--follower-id', 'test']):
            with self.assertRaises(SystemExit):
                parse_args()

    def test_follower_side_is_explicit(self) -> None:
        for side in ('left', 'right'):
            with patch('sys.argv', ['teleoperate', '--leader-port', '/leader',
                                    '--follower-port', '/follower',
                                    '--follower-id', 'test',
                                    '--follower-side', side, '--dry-run']):
                self.assertEqual(parse_args().follower_side, side)

    def test_position_deltas_match_joint_names(self) -> None:
        deltas = position_deltas(
            {"shoulder_pan.pos": 10.0, "gripper.pos": 25.0},
            {"shoulder_pan.pos": 7.5, "gripper.pos": 30.0},
        )
        self.assertEqual(deltas, {"shoulder_pan": 2.5, "gripper": 5.0})

    def test_position_deltas_ignore_matching_non_position_values(self) -> None:
        deltas = position_deltas(
            {"shoulder_pan.pos": 10.0, "button": 1.0},
            {"shoulder_pan.pos": 8.0},
        )
        self.assertEqual(deltas, {"shoulder_pan": 2.0})

    def test_position_deltas_reject_mismatched_joints(self) -> None:
        with self.assertRaisesRegex(ValueError, "position fields differ"):
            position_deltas(
                {"shoulder_pan.pos": 10.0, "gripper.pos": 20.0},
                {"shoulder_pan.pos": 8.0},
            )


if __name__ == "__main__":
    unittest.main()
