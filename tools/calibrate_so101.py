#!/usr/bin/env python3
"""Calibrate one Cleany SO-101 arm with its actual Feetech motor models.

This runner intentionally calibrates one arm per invocation.  The two follower
arms share one serial bus: the left arm uses IDs 1-6 and the right arm uses
IDs 7-12.  Both use STS3250 motors for shoulder lift and elbow flex, unlike the
stock LeRobot SO-101 profile which declares every follower motor as STS3215.
"""

from __future__ import annotations

import argparse
import os
import shutil
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Literal

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS, TELEOPERATORS

ArmRole = Literal["leader", "follower"]
FollowerSide = Literal["right", "left"]


@dataclass(frozen=True)
class MotorSpec:
    name: str
    motor_id: int
    model: str
    gripper: bool = False


@dataclass(frozen=True)
class ArmProfile:
    role: ArmRole
    voltage_hint: str
    motors: tuple[MotorSpec, ...]


LEADER_PROFILE = ArmProfile(
    role="leader",
    voltage_hint="5 V leader supply",
    motors=(
        MotorSpec("shoulder_pan", 1, "sts3215"),
        MotorSpec("shoulder_lift", 2, "sts3215"),
        MotorSpec("elbow_flex", 3, "sts3215"),
        MotorSpec("wrist_flex", 4, "sts3215"),
        MotorSpec("wrist_roll", 5, "sts3215"),
        MotorSpec("gripper", 6, "sts3215", gripper=True),
    ),
)

FOLLOWER_LEFT_PROFILE = ArmProfile(
    role="follower",
    voltage_hint="12 V follower supply",
    motors=(
        MotorSpec("shoulder_pan", 1, "sts3215"),
        MotorSpec("shoulder_lift", 2, "sts3250"),
        MotorSpec("elbow_flex", 3, "sts3250"),
        MotorSpec("wrist_flex", 4, "sts3215"),
        MotorSpec("wrist_roll", 5, "sts3215"),
        MotorSpec("gripper", 6, "sts3215", gripper=True),
    ),
)

FOLLOWER_RIGHT_PROFILE = ArmProfile(
    role="follower",
    voltage_hint="12 V follower supply",
    motors=tuple(
        MotorSpec(spec.name, spec.motor_id + 6, spec.model, spec.gripper)
        for spec in FOLLOWER_LEFT_PROFILE.motors
    ),
)

FOLLOWER_PROFILES: dict[FollowerSide, ArmProfile] = {
    "right": FOLLOWER_RIGHT_PROFILE,
    "left": FOLLOWER_LEFT_PROFILE,
}

# Compatibility name for the original IDs 1-6 profile, physically the LEFT arm.
FOLLOWER_PROFILE = FOLLOWER_LEFT_PROFILE

PROFILES: dict[ArmRole, ArmProfile] = {
    "leader": LEADER_PROFILE,
    "follower": FOLLOWER_PROFILE,
}


def build_motor_map(profile: ArmProfile, *, use_degrees: bool = True) -> dict[str, Motor]:
    body_mode = MotorNormMode.DEGREES if use_degrees else MotorNormMode.RANGE_M100_100
    return {
        spec.name: Motor(
            spec.motor_id,
            spec.model,
            MotorNormMode.RANGE_0_100 if spec.gripper else body_mode,
        )
        for spec in profile.motors
    }


def calibration_path(role: ArmRole, device_id: str, calibration_dir: Path | None) -> Path:
    if calibration_dir is not None:
        return calibration_dir / f"{device_id}.json"
    category = TELEOPERATORS if role == "leader" else ROBOTS
    device_type = "so_leader" if role == "leader" else "so_follower"
    return HF_LEROBOT_CALIBRATION / category / device_type / f"{device_id}.json"


def build_device(
    role: ArmRole,
    port: str,
    device_id: str,
    calibration_dir: Path | None,
    *,
    follower_profile: ArmProfile = FOLLOWER_PROFILE,
) -> SO101Leader | SO101Follower:
    if role == "leader":
        config = SO101LeaderConfig(
            port=port,
            id=device_id,
            calibration_dir=calibration_dir,
        )
        return SO101Leader(config)

    config = SO101FollowerConfig(
        port=port,
        id=device_id,
        calibration_dir=calibration_dir,
    )
    follower = SO101Follower(config)
    follower.bus = FeetechMotorsBus(
        port=port,
        motors=build_motor_map(follower_profile, use_degrees=config.use_degrees),
        calibration=follower.calibration,
    )
    return follower


def validate_port(port: str) -> None:
    path = Path(port)
    if not path.exists():
        raise FileNotFoundError(f"Serial port does not exist: {port}")
    if not os.access(path, os.R_OK | os.W_OK):
        raise PermissionError(f"Serial port is not readable and writable: {port}")


def back_up_existing_calibration(path: Path) -> Path | None:
    if not path.is_file():
        return None
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    backup = path.with_suffix(f".json.backup-{timestamp}")
    shutil.copy2(path, backup)
    return backup


def print_plan(profile: ArmProfile, port: str, device_id: str, output_path: Path) -> None:
    print(f"role: {profile.role}")
    print(f"port: {port}")
    print(f"id: {device_id}")
    print(f"expected power: {profile.voltage_hint}")
    print(f"calibration file: {output_path}")
    print("motors:")
    for spec in profile.motors:
        print(f"  ID {spec.motor_id}: {spec.name} ({spec.model})")


def calibrate_without_motion(device: SO101Follower | SO101Leader) -> None:
    # Robot.connect() also configures the arm and temporarily enables torque.
    # Calibration only needs the selected motors' bus and its model handshake.
    bus = device.bus
    try:
        bus.connect()
        bus.disable_torque()
        device.calibrate()
    finally:
        if bus.is_connected:
            try:
                bus.disable_torque()
                for motor in bus.motors:
                    bus.write("Lock", motor, 1)
            finally:
                bus.disconnect(disable_torque=False)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate one Cleany SO-101 leader or mixed-motor follower arm.",
    )
    parser.add_argument("--arm", choices=tuple(PROFILES), required=True)
    parser.add_argument(
        "--follower-side",
        choices=tuple(FOLLOWER_PROFILES),
        help="Required for follower; left=IDs 1-6, right=IDs 7-12.",
    )
    parser.add_argument("--port", required=True, help="Prefer a stable /dev/serial/by-id path.")
    parser.add_argument("--id", required=True, help="Stable LeRobot calibration identifier.")
    parser.add_argument(
        "--calibration-dir",
        type=Path,
        help="Override LeRobot's default calibration directory.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the resolved profile without opening the serial port.",
    )
    args = parser.parse_args()
    if args.arm == "follower" and args.follower_side is None:
        parser.error("--follower-side is required for --arm follower")
    if args.arm == "leader" and args.follower_side is not None:
        parser.error("--follower-side only applies to --arm follower")
    return args


def main() -> int:
    args = parse_args()
    role: ArmRole = args.arm
    profile = FOLLOWER_PROFILES[args.follower_side] if role == "follower" else LEADER_PROFILE
    output_path = calibration_path(role, args.id, args.calibration_dir)
    print_plan(profile, args.port, args.id, output_path)

    if args.dry_run:
        print("Dry run: no serial port was opened and no motor command was sent.")
        return 0

    validate_port(args.port)
    expected = args.follower_side.upper() if role == "follower" else "LEADER"
    answer = input(f"Type {expected} to open only this arm and begin calibration: ")
    if answer.strip().upper() != expected:
        print("Cancelled before opening the serial port.")
        return 2

    backup = back_up_existing_calibration(output_path)
    if backup is not None:
        print(f"Existing calibration backed up to {backup}")

    device = build_device(
        role,
        args.port,
        args.id,
        args.calibration_dir,
        follower_profile=profile,
    )
    calibrate_without_motion(device)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
