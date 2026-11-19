"""Capture stationary hardware telemetry and ICP quality without enabling motion."""

import argparse
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from cleany_base_interfaces.msg import WheelState

from evaluation_results import json_finite, match_metadata, write_data_json
from lidar_scan_matcher import LidarScanMatcher


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration < 2:
        parser.error("duration must be finite and at least 2 seconds")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    rclpy.init()
    node = Node("calibration_preflight")
    qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
    wheel, odom, scans = [], [], []
    subscriptions = [
        node.create_subscription(WheelState, "/base/wheel_state", wheel.append, qos),
        node.create_subscription(Odometry, "/odom", odom.append, qos),
        node.create_subscription(LaserScan, "/scan", scans.append, qos),
    ]
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        matcher = LidarScanMatcher(voxel_size_m=.015)
        bursts = []
        for group in (scans[:6], scans[-6:]):
            points = [matcher.laserscan_to_points(
                msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max
            ) for msg in group]
            bursts.append(matcher.downsample_points(np.vstack(points)) if points else np.empty((0, 2)))
        match = matcher.match(*bursts)
        publishers = [p.node_name for p in node.get_publishers_info_by_topic("/cmd_vel")]
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in odom]
        gaps = np.diff(stamps)
        stationary = bool(wheel) and all(
            not m.enabled and not m.fault_bits
            and not any(m.target_rad_s) and not any(m.commanded_rad_s)
            and not any(m.pwm_percent) for m in wheel
        )
        result = {
            "timestamp": timestamp, "motion_enabled_by_capture": False,
            "topic_samples": {"wheel_state": len(wheel), "odom": len(odom), "scan": len(scans)},
            "cmd_vel_publishers": publishers,
            "stationary_disabled": stationary,
            "boot_ids": sorted(set(m.boot_id for m in wheel)),
            "odom_max_stamp_gap_s": float(max(gaps)) if len(gaps) else None,
            "stationary_icp": match_metadata(match),
            "wheel_state": [{
                key: list(getattr(msg, key)) if key in (
                    "encoder_counts", "velocity_rad_s", "target_rad_s",
                    "commanded_rad_s", "pwm_percent"
                ) else getattr(msg, key)
                for key in (
                    "protocol_version", "boot_id", "sequence", "timestamp_us",
                    "enabled", "fault_bits", "control_mode", "command_age_ms",
                    "encoder_counts", "velocity_rad_s", "target_rad_s",
                    "commanded_rad_s", "pwm_percent",
                )
            } for msg in wheel],
            "odometry": [{
                "stamp_s": stamp, "x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y,
                "orientation": [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w],
                "velocity": [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z],
            } for stamp, msg in zip(stamps, odom)],
            "source_sha256": {
                str(path.name): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in Path(__file__).parent.glob("*.py")
            },
        }
        write_data_json(args.output_dir / f"preflight_{timestamp}.json", result)
        arrays = {"start_points": bursts[0], "end_points": bursts[1]}
        for index, msg in enumerate(scans):
            arrays[f"ranges_{index}"] = np.asarray(msg.ranges)
            arrays[f"metadata_{index}"] = np.asarray([
                msg.header.stamp.sec, msg.header.stamp.nanosec, msg.angle_min,
                msg.angle_increment, msg.range_min, msg.range_max,
            ])
        with (args.output_dir / f"preflight_{timestamp}_scans.npz").open("xb") as stream:
            np.savez_compressed(stream, **arrays)
        print(json.dumps({
            key: result[key] for key in (
                "topic_samples", "cmd_vel_publishers", "stationary_disabled",
                "boot_ids", "odom_max_stamp_gap_s", "stationary_icp",
            )
        }, indent=2))
        return 0 if stationary and odom and len(scans) >= 12 and match.is_valid and not publishers else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
