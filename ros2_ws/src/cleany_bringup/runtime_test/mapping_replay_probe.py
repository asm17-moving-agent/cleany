#!/usr/bin/env python3
"""Replay captured mapping CDR in a separate mock-only ROS domain."""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_msgs.msg import TFMessage
from cleany_base_interfaces.msg import WheelState, WheelCommand


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "175":
        raise SystemExit("Replay requires isolated ROS_DOMAIN_ID=175")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("mapping_replay_probe")
    counts = Counter()
    types = {
        "/scan_raw": LaserScan,
        "/scan": LaserScan,
        "/base/wheel_state": WheelState,
        "/base/wheel_command": WheelCommand,
        "/joint_states": JointState,
        "/odom": Odometry,
        "/wheel/odom": Odometry,
        "/tf": TFMessage,
        "/tf_static": TFMessage,
        "/map": OccupancyGrid,
    }
    for topic, message in types.items():
        node.create_subscription(
            message,
            topic,
            lambda _, name=topic: counts.update([name]),
            qos_profile_sensor_data,
        )
    log = args.output.with_suffix(".log").open("w")
    process = subprocess.Popen(
        [
            "ros2",
            "bag",
            "play",
            str(args.bag),
            "--rate",
            "4",
            "--delay",
            "1",
            "--disable-keyboard-controls",
        ],
        cwd="/tmp",
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    try:
        end = time.monotonic() + 25
        while process.poll() is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert process.wait(timeout=2) == 0
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert all(counts[topic] > 0 for topic in types), dict(counts)
        args.output.write_text(
            json.dumps(
                {
                    "success": True,
                    "bag": str(args.bag),
                    "received": dict(counts),
                    "domain": 175,
                },
                indent=2,
            )
        )
        print(dict(counts))
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
        log.close()


if __name__ == "__main__":
    main()
