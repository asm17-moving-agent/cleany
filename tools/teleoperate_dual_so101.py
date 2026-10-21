#!/usr/bin/env python3
"""Teleoperate both Cleany SO-101 followers from two leaders on shared buses."""

from __future__ import annotations

import argparse
import json
import logging
import os
from pathlib import Path
import signal
import time

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.utils.robot_utils import precise_sleep


JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
SIDES = {"left": 1, "right": 7}
DEFAULT_CALIBRATION_ROOT = (
    Path.home() / ".cache/huggingface/lerobot/calibration"
)
_STOP_REQUESTED = False


def request_stop(_signum: int, _frame: object) -> None:
    global _STOP_REQUESTED
    _STOP_REQUESTED = True


def calibration_path(role: str, device_id: str) -> Path:
    category = "teleoperators/so_leader" if role == "leader" else "robots/so_follower"
    return DEFAULT_CALIBRATION_ROOT / category / f"{device_id}.json"


def load_side_calibration(
    path: Path,
    side: str,
) -> dict[str, MotorCalibration]:
    raw = json.loads(path.read_text())
    if set(raw) != set(JOINTS):
        raise ValueError(f"{path}: expected joints {sorted(JOINTS)}, found {sorted(raw)}")
    expected_ids = range(SIDES[side], SIDES[side] + len(JOINTS))
    actual_ids = [int(raw[name]["id"]) for name in JOINTS]
    if actual_ids != list(expected_ids):
        raise ValueError(
            f"{path}: {side} calibration IDs must be {list(expected_ids)}, found {actual_ids}"
        )
    return {
        f"{side}_{name}": MotorCalibration(
            id=int(raw[name]["id"]),
            drive_mode=int(raw[name]["drive_mode"]),
            homing_offset=int(raw[name]["homing_offset"]),
            range_min=int(raw[name]["range_min"]),
            range_max=int(raw[name]["range_max"]),
        )
        for name in JOINTS
    }


def load_dual_calibration(
    left_path: Path,
    right_path: Path,
) -> dict[str, MotorCalibration]:
    return {
        **load_side_calibration(left_path, "left"),
        **load_side_calibration(right_path, "right"),
    }


def motor_map(role: str) -> dict[str, Motor]:
    motors: dict[str, Motor] = {}
    for side, first_id in SIDES.items():
        for offset, joint in enumerate(JOINTS):
            model = (
                "sts3250"
                if role == "follower" and joint in {"shoulder_lift", "elbow_flex"}
                else "sts3215"
            )
            norm_mode = (
                MotorNormMode.RANGE_0_100
                if joint == "gripper"
                else MotorNormMode.DEGREES
            )
            motors[f"{side}_{joint}"] = Motor(first_id + offset, model, norm_mode)
    return motors


def position_deltas(
    leader_positions: dict[str, float],
    follower_positions: dict[str, float],
) -> dict[str, float]:
    if set(leader_positions) != set(follower_positions):
        raise ValueError("Leader and follower position fields differ")
    return {
        name: abs(float(value) - float(follower_positions[name]))
        for name, value in leader_positions.items()
    }


def set_follower_torque(bus: FeetechMotorsBus, enabled: bool) -> None:
    value = 1 if enabled else 0
    errors: list[str] = []
    for motor in bus.motors:
        try:
            bus.write("Torque_Enable", motor, value, num_retry=2)
        except Exception as exc:
            errors.append(f"{motor} torque: {exc}")
        try:
            # Keep EEPROM protected. FeetechMotorsBus.disable_torque() unlocks it.
            bus.write("Lock", motor, 1, num_retry=2)
        except Exception as exc:
            errors.append(f"{motor} lock: {exc}")
    if errors:
        raise RuntimeError("; ".join(errors))


def set_torque_limit(bus: FeetechMotorsBus, value: int) -> None:
    for motor in bus.motors:
        current_maximum = int(
            bus.read("Max_Torque_Limit", motor, normalize=False, num_retry=2)
        )
        if current_maximum != value:
            bus.write("Torque_Enable", motor, 0, num_retry=2)
            bus.write("Lock", motor, 0, num_retry=2)
            bus.write("Max_Torque_Limit", motor, value, num_retry=2)
            written = int(
                bus.read("Max_Torque_Limit", motor, normalize=False, num_retry=2)
            )
            if written != value:
                raise RuntimeError(
                    f"{motor}: failed to set EEPROM max torque to {value}; read {written}"
                )
            bus.write("Lock", motor, 1, num_retry=2)
        bus.write("Torque_Limit", motor, value, num_retry=2)


def validate_runtime_state(bus: FeetechMotorsBus, *, follower: bool) -> None:
    if not bus.is_calibrated:
        raise RuntimeError(f"Calibration mismatch on {bus.port}")
    for motor in bus.motors:
        status = int(bus.read("Status", motor, normalize=False, num_retry=2))
        temperature = int(bus.read("Present_Temperature", motor, normalize=False, num_retry=2))
        if status != 0:
            raise RuntimeError(f"{motor}: motor status is {status}")
        if temperature >= 80:
            raise RuntimeError(f"{motor}: temperature is {temperature} C")
        if follower:
            torque = int(bus.read("Torque_Enable", motor, normalize=False, num_retry=2))
            if torque != 0:
                raise RuntimeError(f"{motor}: follower torque must be off before linking")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Safely teleoperate both Cleany SO-101 arms.")
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-left-id", default="cleany_so101_leader_left")
    parser.add_argument("--leader-right-id", default="cleany_so101_leader_right")
    parser.add_argument("--follower-left-id", default="cleany_so101_left")
    parser.add_argument("--follower-right-id", default="cleany_so101_right")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--max-relative-target",
        type=float,
        default=5.0,
        help="Per-cycle target limit; 0 disables target limiting.",
    )
    parser.add_argument("--max-start-delta", type=float, default=20.0)
    parser.add_argument(
        "--torque-limit",
        type=int,
        help="Set follower EEPROM maximum and RAM torque limit (1-1000).",
    )
    parser.add_argument(
        "--skip-safety-checks",
        action="store_true",
        help="Skip motor status, temperature, calibration, and startup-pose checks.",
    )
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Start without LINK and START prompts.",
    )
    parser.add_argument("--duration", type=float)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    global _STOP_REQUESTED
    _STOP_REQUESTED = False
    args = parse_args()
    signal.signal(signal.SIGTERM, request_stop)
    if args.leader_port == args.follower_port:
        raise ValueError("Leader and follower ports must differ")
    if args.fps <= 0 or args.max_relative_target < 0 or args.max_start_delta <= 0:
        raise ValueError("fps/startup limit must be positive and target limit cannot be negative")
    if args.torque_limit is not None and not 1 <= args.torque_limit <= 1000:
        raise ValueError("--torque-limit must be in [1, 1000]")

    paths = {
        "leader_left": calibration_path("leader", args.leader_left_id),
        "leader_right": calibration_path("leader", args.leader_right_id),
        "follower_left": calibration_path("follower", args.follower_left_id),
        "follower_right": calibration_path("follower", args.follower_right_id),
    }
    for label, path in paths.items():
        if not path.is_file():
            raise FileNotFoundError(f"{label} calibration does not exist: {path}")
        print(f"{label} calibration: {path}")
    for label, port in (("leader", args.leader_port), ("follower", args.follower_port)):
        if not Path(port).exists() or not os.access(port, os.R_OK | os.W_OK):
            raise FileNotFoundError(f"{label} port is unavailable: {port}")

    leader_calibration = load_dual_calibration(paths["leader_left"], paths["leader_right"])
    follower_calibration = load_dual_calibration(paths["follower_left"], paths["follower_right"])
    print("leader IDs: 1-6 left, 7-12 right")
    print("follower IDs: 1-6 left, 7-12 right")
    print(f"control rate: {args.fps} Hz")
    print(f"per-cycle target limit: {args.max_relative_target}")
    print(f"startup delta limit: {args.max_start_delta}")
    if args.dry_run:
        print("Dry run: serial ports were not opened and no motor command was sent.")
        return 0

    if not args.auto_start:
        answer = input("Clear the workspace, keep power cutoff in reach, then type LINK: ")
        if answer.strip().upper() != "LINK":
            print("Cancelled before opening either motor bus.")
            return 2

    leader = FeetechMotorsBus(args.leader_port, motor_map("leader"), leader_calibration)
    follower = FeetechMotorsBus(args.follower_port, motor_map("follower"), follower_calibration)
    leader_connected = follower_connected = False
    try:
        leader.connect()
        leader_connected = True
        follower.connect()
        follower_connected = True
        set_follower_torque(follower, False)
        leader_positions = leader.sync_read("Present_Position", num_retry=2)
        follower_positions = follower.sync_read("Present_Position", num_retry=2)
        if not args.skip_safety_checks:
            validate_runtime_state(leader, follower=False)
            validate_runtime_state(follower, follower=True)
            deltas = position_deltas(leader_positions, follower_positions)
            print("Calibrated startup pose deltas:")
            for joint, delta in deltas.items():
                print(f"  {joint}: {delta:.2f}")
            unsafe = {
                joint: delta for joint, delta in deltas.items()
                if delta > args.max_start_delta
            }
            if unsafe:
                details = ", ".join(
                    f"{joint}={delta:.2f}" for joint, delta in unsafe.items()
                )
                raise RuntimeError(
                    "Follower torque remains off; startup poses differ: " + details
                )

        if not args.auto_start:
            answer = input("Type START to stage current goals and enable both follower arms: ")
            if answer.strip().upper() != "START":
                print("Cancelled with follower torque disabled.")
                return 2

        if _STOP_REQUESTED:
            return 0
        follower.sync_write("Goal_Position", follower_positions)
        if args.torque_limit is not None:
            set_torque_limit(follower, args.torque_limit)
        if _STOP_REQUESTED:
            return 0
        set_follower_torque(follower, True)
        print("Dual-arm teleoperation active. Press Ctrl+C to stop.")
        # Per-cycle clamping is expected during normal tracking; avoid flooding
        # the terminal with the helper's warning on every control iteration.
        logging.getLogger().setLevel(logging.ERROR)
        started = time.perf_counter()
        while (
            not _STOP_REQUESTED
            and (args.duration is None or time.perf_counter() - started < args.duration)
        ):
            loop_started = time.perf_counter()
            action = leader.sync_read("Present_Position", num_retry=2)
            present = follower.sync_read("Present_Position", num_retry=2)
            safe_action = (
                ensure_safe_goal_position(
                    {name: (goal, present[name]) for name, goal in action.items()},
                    args.max_relative_target,
                )
                if args.max_relative_target > 0
                else action
            )
            follower.sync_write("Goal_Position", safe_action)
            precise_sleep(max(1.0 / args.fps - (time.perf_counter() - loop_started), 0.0))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if follower_connected and follower.is_connected:
                try:
                    set_follower_torque(follower, False)
                finally:
                    follower.disconnect(disable_torque=False)
        finally:
            if leader_connected and leader.is_connected:
                leader.disconnect(disable_torque=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
