#!/usr/bin/env python3
"""Teleoperate the mixed-motor Cleany SO-101 follower from its leader arm."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from lerobot.processor import make_default_processors
from lerobot.scripts.lerobot_teleoperate import teleop_loop

from calibrate_so101 import FOLLOWER_PROFILES, build_device, calibration_path


def validate_file(path: Path, label: str) -> None:
    if not path.is_file():
        raise FileNotFoundError(f"{label} calibration file does not exist: {path}")


def validate_port(port: str, label: str) -> None:
    path = Path(port)
    if not path.exists():
        raise FileNotFoundError(f"{label} serial port does not exist: {port}")
    if not os.access(path, os.R_OK | os.W_OK):
        raise PermissionError(f"{label} serial port is not readable and writable: {port}")


def position_deltas(
    leader_action: dict[str, float],
    follower_observation: dict[str, float],
) -> dict[str, float]:
    leader_keys = {key for key in leader_action if key.endswith(".pos")}
    follower_keys = {key for key in follower_observation if key.endswith(".pos")}
    if leader_keys != follower_keys:
        raise ValueError(
            "Leader and follower position fields differ: "
            f"leader={sorted(leader_keys)}, follower={sorted(follower_keys)}"
        )
    return {
        key.removesuffix(".pos"): abs(float(value) - float(follower_observation[key]))
        for key, value in leader_action.items()
        if key.endswith(".pos")
    }


def stage_current_follower_position(robot, observation: dict[str, float]) -> None:
    current_goals = {
        key.removesuffix(".pos"): value
        for key, value in observation.items()
        if key.endswith(".pos")
    }
    robot.bus.sync_write("Goal_Position", current_goals)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safely teleoperate the mixed STS3215/STS3250 Cleany SO-101 follower.",
    )
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-id", default="cleany_so101_leader")
    parser.add_argument("--follower-id", required=True)
    parser.add_argument("--follower-side", choices=tuple(FOLLOWER_PROFILES), required=True)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--max-relative-target",
        type=float,
        default=5.0,
        help="Maximum follower target change per control cycle; default: 5.0.",
    )
    parser.add_argument(
        "--max-start-delta",
        type=float,
        default=20.0,
        help="Refuse torque enable if any calibrated joint delta exceeds this value; default: 20.0.",
    )
    parser.add_argument("--duration", type=float, help="Optional run duration in seconds.")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate arguments and print the plan without opening either serial port.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.leader_port == args.follower_port:
        raise ValueError("Leader and follower ports must be different.")
    if args.fps <= 0:
        raise ValueError("--fps must be positive.")
    if args.max_relative_target <= 0 or args.max_start_delta <= 0:
        raise ValueError("Safety limits must be positive.")

    leader_calibration = calibration_path("leader", args.leader_id, None)
    follower_calibration = calibration_path("follower", args.follower_id, None)
    print(f"leader port: {args.leader_port}")
    print(f"leader calibration: {leader_calibration}")
    print(f"follower port: {args.follower_port}")
    print(f"follower calibration: {follower_calibration}")
    profile = FOLLOWER_PROFILES[args.follower_side]
    print(f"follower side: {args.follower_side}; IDs: {[m.motor_id for m in profile.motors]}")
    print(f"control rate: {args.fps} Hz")
    print(f"per-cycle follower limit: {args.max_relative_target}")
    print(f"startup pose-delta limit: {args.max_start_delta}")

    validate_file(leader_calibration, "Leader")
    validate_file(follower_calibration, "Follower")
    if args.dry_run:
        print("Dry run: no serial port was opened and no motor command was sent.")
        return 0

    validate_port(args.leader_port, "Leader")
    validate_port(args.follower_port, "Follower")
    answer = input(
        "Place both arms in closely matching poses, clear the workspace, then type LINK to inspect them: "
    )
    if answer.strip().upper() != "LINK":
        print("Cancelled before opening either serial port.")
        return 2

    leader = build_device("leader", args.leader_port, args.leader_id, None)
    follower = build_device(
        "follower", args.follower_port, args.follower_id, None,
        follower_profile=profile,
    )
    follower.config.max_relative_target = args.max_relative_target
    leader_connected = False
    follower_connected = False

    try:
        # Connect buses directly so follower torque stays off while startup poses
        # and stored calibrations are checked.
        leader.bus.connect()
        leader_connected = True
        leader.configure()
        if not leader.is_calibrated:
            raise RuntimeError("Leader motor calibration does not match its saved calibration file.")

        follower.bus.connect()
        follower_connected = True
        follower.bus.disable_torque()
        if not follower.is_calibrated:
            raise RuntimeError("Follower motor calibration does not match its saved calibration file.")

        leader_action = leader.get_action()
        follower_observation = follower.get_observation()
        deltas = position_deltas(leader_action, follower_observation)
        print("Calibrated startup pose deltas:")
        for joint, delta in deltas.items():
            print(f"  {joint}: {delta:.2f}")

        unsafe = {joint: delta for joint, delta in deltas.items() if delta > args.max_start_delta}
        if unsafe:
            details = ", ".join(f"{joint}={delta:.2f}" for joint, delta in unsafe.items())
            raise RuntimeError(
                "Follower torque remains off because startup poses differ too much: " + details
            )

        answer = input("Keep the power cutoff within reach, then type START to enable follower torque: ")
        if answer.strip().upper() != "START":
            print("Cancelled with follower torque disabled.")
            return 2

        # Seed the goal with the measured follower pose before configure() enables
        # torque. This prevents the servo's previous goal register from causing a jump.
        stage_current_follower_position(follower, follower_observation)
        follower.configure()

        teleop_action_processor, robot_action_processor, robot_observation_processor = (
            make_default_processors()
        )
        print("Teleoperation active. Press Ctrl+C to stop and disable follower torque.")
        try:
            teleop_loop(
                teleop=leader,
                robot=follower,
                fps=args.fps,
                duration=args.duration,
                display_data=False,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
            )
        except KeyboardInterrupt:
            pass
    finally:
        if follower_connected and follower.is_connected:
            follower.disconnect()
        if leader_connected and leader.is_connected:
            leader.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
