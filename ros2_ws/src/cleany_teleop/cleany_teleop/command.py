#!/usr/bin/env python3
"""Temporary ROS 2 command source; does not flash or reset the ESP32."""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time

from cleany_teleop.core import FeedbackClock, newer, wheel_targets


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action",
        choices=[
            "status",
            "stop",
            "forward",
            "backward",
            "left",
            "right",
            "ccw",
            "cw",
            "wheels",
        ],
    )
    parser.add_argument(
        "--speed", type=float, default=0.5, help="wheel speed in rad/s (default 0.5)"
    )
    parser.add_argument(
        "--wheels", type=float, nargs=4, metavar=("FL", "FR", "RL", "RR")
    )
    parser.add_argument(
        "--duration", type=float, default=1.0, help="motion seconds (0 < seconds <= 10)"
    )
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 10:
        parser.error("duration must be finite and in (0, 10]")
    if args.action == "wheels":
        if args.wheels is None or any(
            not math.isfinite(v) or abs(v) > 10 for v in args.wheels
        ):
            parser.error(
                "wheels requires four finite --wheels values within +/-10 rad/s"
            )
        args.targets = tuple(args.wheels)
    elif args.action in ("status", "stop"):
        args.targets = (0.0,) * 4
    else:
        try:
            args.targets = wheel_targets(args.action, args.speed)
        except ValueError as error:
            parser.error(str(error))
    return args


def main() -> int:
    args = arguments()
    import rclpy
    from cleany_base_interfaces.msg import WheelCommand, WheelState
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("esp32_temporary_motion")
    publisher = node.create_publisher(
        WheelCommand, "/base/wheel_command", qos_profile_sensor_data
    )
    clock = FeedbackClock()
    state = None
    failure = None
    interrupted = False
    owns_commands = False

    def on_signal(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    def feedback(message):
        nonlocal state, failure
        try:
            if clock.observe(message, time.monotonic()):
                state = message
        except RuntimeError as error:
            failure = str(error)

    node.create_subscription(
        WheelState, "/base/wheel_state", feedback, qos_profile_sensor_data
    )

    def spin(seconds=0.02):
        rclpy.spin_once(node, timeout_sec=seconds)

    def check():
        if interrupted:
            raise RuntimeError("interrupted")
        if failure:
            raise RuntimeError(failure)
        clock.require_fresh(time.monotonic())
        if node.count_publishers("/base/wheel_command") != 1:
            raise RuntimeError("another wheel-command publisher is present")

    def send(mode, targets=(0.0,) * 4, sequence=None, deadline=None):
        message = WheelCommand()
        message.protocol_version = 2
        message.mode = mode
        message.sequence = (
            clock.next_sequence(state.last_command_sequence if state else 0)
            if sequence is None
            else sequence
        )
        message.valid_until_us = (
            0
            if mode == WheelCommand.STOP
            else clock.deadline(time.monotonic(), state.watchdog_ms)
            if deadline is None
            else deadline
        )
        message.velocity_rad_s = list(targets)
        publisher.publish(message)
        return message

    def acknowledged(sequence):
        return state is not None and (
            state.last_command_sequence == sequence
            or newer(state.last_command_sequence, sequence)
        )

    def stopped():
        return (
            state is not None
            and not state.enabled
            and all(v == 0 for v in state.target_rad_s)
            and all(v == 0 for v in state.commanded_rad_s)
            and all(v == 0 for v in state.pwm_percent)
        )

    def stop_and_confirm():
        # STOP ignores deadlines and is also attempted after stale/reboot errors.
        message = send(WheelCommand.STOP)
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            spin()
            send(WheelCommand.STOP, sequence=message.sequence)
            if (
                not failure
                and state is not None
                and time.monotonic() - clock.received_at <= 0.10
                and acknowledged(message.sequence)
                and stopped()
            ):
                print("STOP confirmed: MCU disabled, target/command/PWM zero.")
                return True
        print("STOP sent; MCU confirmation unavailable.", file=sys.stderr)
        return False

    result = 0
    try:
        if args.action == "stop":
            # A manual STOP must still be attempted with missing feedback or
            # while another command source is present. Never issue ENABLE here.
            owns_commands = True
            end = time.monotonic() + 1.0
            while (
                node.count_subscribers("/base/wheel_command") < 1
                and time.monotonic() < end
            ):
                spin()
        else:
            end = time.monotonic() + 5.0
            while state is None or (
                args.action != "status"
                and node.count_subscribers("/base/wheel_command") < 1
            ):
                if interrupted or failure or time.monotonic() > end:
                    raise RuntimeError(
                        failure
                        or "no MCU state/command subscription; start the serial Agent"
                    )
                spin()
        if args.action == "status":
            print(
                json.dumps(
                    {
                        field: getattr(state, field)
                        for field in (
                            "protocol_version",
                            "boot_id",
                            "timestamp_us",
                            "enabled",
                            "fault_bits",
                            "last_command_sequence",
                            "watchdog_ms",
                        )
                    },
                    indent=2,
                )
            )
            print("wheel order FL FR RL RR; rad/s:", list(state.velocity_rad_s))
            print("PWM:", list(state.pwm_percent))
        elif args.action != "stop":
            check()
            owns_commands = True
            if args.action != "stop":
                if any(abs(v) > state.max_velocity_rad_s for v in args.targets):
                    raise RuntimeError("requested speed exceeds MCU advertised limit")
                if not stop_and_confirm():
                    raise RuntimeError("initial STOP was not confirmed")
                check()
                enable = send(WheelCommand.ENABLE)
                # Retransmit identical ENABLE; never extend its original deadline.
                end = time.monotonic() + 0.10
                while not (state.enabled and acknowledged(enable.sequence)):
                    spin(0.01)
                    check()
                    if time.monotonic() >= end:
                        raise RuntimeError("ENABLE was not confirmed")
                    send(
                        WheelCommand.ENABLE,
                        sequence=enable.sequence,
                        deadline=enable.valid_until_us,
                    )
                print(f"Moving FL/FR/RL/RR={args.targets} rad/s for {args.duration:g}s")
                end = time.monotonic() + args.duration
                next_send = time.monotonic()
                while time.monotonic() < end:
                    check()
                    if not state.enabled or state.fault_bits:
                        raise RuntimeError(
                            "MCU disabled or reported a fault; motion will not resume"
                        )
                    if state.command_age_ms > 150:
                        raise RuntimeError("MCU is not accepting fresh commands")
                    if time.monotonic() >= next_send:
                        send(WheelCommand.VELOCITY, args.targets)
                        next_send = time.monotonic() + 0.02
                    spin(0.005)
    except RuntimeError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        result = 1
    finally:
        if owns_commands and not stop_and_confirm():
            result = 1
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
