#!/usr/bin/env python3
"""Replace odometry in one recorded SLAM bag with deterministic error levels."""

from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import fields
import json
from math import atan2, cos, sin
from pathlib import Path
from typing import Any

import yaml


LEVEL_CONFIGS = {
    "level0": "odometry_error_ideal.yaml",
    "level1": "odometry_error_level1.yaml",
    "level2": "odometry_error_level2.yaml",
    "level3": "odometry_error_stress.yaml",
}

WHEEL_DIAGNOSTIC_TOPICS = {
    "/joint_states",
    "/wheel/odom",
    "/wheel/odom_raw",
    "/wheel_encoder/joint_states",
}


def load_parameters(config: Path) -> dict[str, Any]:
    """Load only core error-model fields from a ROS parameter file."""
    from cleany_gazebo_sim.odometry_error import OdometryErrorParameters

    document = yaml.safe_load(config.read_text(encoding="utf-8"))
    values = document["simulated_odometry_error"]["ros__parameters"]
    names = {item.name for item in fields(OdometryErrorParameters)}
    return {name: values[name] for name in names if name in values}


def stamp_seconds(stamp: object) -> float:
    """Convert a builtin_interfaces Time-like object to seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_of(orientation: object) -> float:
    """Return planar yaw from a quaternion-like object."""
    return atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def apply_error(model: object, message: object) -> object:
    """Apply one error-model update to an Odometry message copy."""
    from cleany_gazebo_sim.odometry_error import Pose2D

    output = deepcopy(message)
    pose = message.pose.pose
    estimate = model.update(
        Pose2D(
            x_m=float(pose.position.x),
            y_m=float(pose.position.y),
            yaw_rad=yaw_of(pose.orientation),
        ),
        stamp_seconds(message.header.stamp),
    )
    output.pose.pose.position.x = estimate.pose.x_m
    output.pose.pose.position.y = estimate.pose.y_m
    output.pose.pose.orientation.x = 0.0
    output.pose.pose.orientation.y = 0.0
    output.pose.pose.orientation.z = sin(estimate.pose.yaw_rad / 2.0)
    output.pose.pose.orientation.w = cos(estimate.pose.yaw_rad / 2.0)
    output.twist.twist.linear.x = estimate.linear_x_mps
    output.twist.twist.linear.y = estimate.linear_y_mps
    output.twist.twist.angular.z = estimate.angular_z_rps
    return output


def materialize(
    input_bag: Path,
    output_root: Path,
    config_root: Path,
    source_odom_topic: str,
) -> None:
    """Read a bag once and write four bags with level-specific odometry."""
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message, serialize_message
    from cleany_gazebo_sim.odometry_error import (
        OdometryErrorParameters,
        StatefulOdometryError,
    )

    if not (input_bag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"input bag metadata not found: {input_bag}")

    destinations = {
        level: output_root / level / "input_30cm_trial1"
        for level in LEVEL_CONFIGS
    }
    existing = [path for path in destinations.values() if path.exists()]
    if existing:
        raise FileExistsError(
            "refusing to overwrite derived bags: "
            + ", ".join(str(path) for path in existing)
        )

    metadata = yaml.safe_load(
        (input_bag / "metadata.yaml").read_text(encoding="utf-8")
    )
    bag_information = metadata.get("rosbag2_bagfile_information", metadata)
    storage_id = str(bag_information["storage_identifier"])
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(input_bag), storage_id=storage_id
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topics = reader.get_all_topics_and_types()
    topic_names = {topic.name for topic in topics}
    if source_odom_topic not in topic_names:
        raise RuntimeError(
            f"input bag has no {source_odom_topic} topic: {input_bag}"
        )
    if "/odom" not in topic_names:
        raise RuntimeError(f"input bag has no /odom topic contract: {input_bag}")
    omitted_topics = (
        WHEEL_DIAGNOSTIC_TOPICS
        if source_odom_topic != "/odom"
        else set()
    )

    writers: dict[str, object] = {}
    models: dict[str, object | None] = {}
    configs: dict[str, Path] = {}
    for level, filename in LEVEL_CONFIGS.items():
        destination = destinations[level]
        destination.parent.mkdir(parents=True, exist_ok=True)
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(
                uri=str(destination), storage_id="sqlite3"
            ),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        for topic in topics:
            if topic.name in omitted_topics:
                continue
            writer.create_topic(topic)
        config = config_root / filename
        configs[level] = config
        writers[level] = writer
        models[level] = (
            None
            if level == "level0"
            else StatefulOdometryError(
                OdometryErrorParameters(**load_parameters(config))
            )
        )

    message_counts = {level: 0 for level in LEVEL_CONFIGS}
    odometry_counts = {level: 0 for level in LEVEL_CONFIGS}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        source_odometry = (
            deserialize_message(data, Odometry)
            if topic == source_odom_topic
            else None
        )
        for level, writer in writers.items():
            if source_odom_topic != "/odom" and topic == "/odom":
                continue
            if topic in omitted_topics and source_odometry is None:
                continue
            output_data = data
            output_topic = topic
            if source_odometry is not None:
                output_topic = "/odom"
            if source_odometry is not None and models[level] is not None:
                output_data = serialize_message(
                    apply_error(models[level], source_odometry)
                )
            writer.write(output_topic, output_data, timestamp)
            message_counts[level] += 1
            if source_odometry is not None:
                odometry_counts[level] += 1

    for level, writer in writers.items():
        writer.close()
        manifest = {
            "source_bag": str(input_bag.resolve()),
            "source_odom_topic": source_odom_topic,
            "output_odom_topic": "/odom",
            "odometry_level": level,
            "parameter_file": str(configs[level].resolve()),
            "random_seed": 42,
            "message_count": message_counts[level],
            "odometry_message_count": odometry_counts[level],
            "omitted_diagnostic_topics": sorted(omitted_topics),
            "note": "Synthetic robustness input; not measured hardware noise.",
        }
        (destinations[level] / "derivation.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
        )
        print(destinations[level])


def main() -> None:
    """Parse command-line arguments and create the four derived bags."""
    repository = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input-bag",
        type=Path,
        default=(
            repository
            / "ros2_ws/slam_results/algorithm_compare_inputs/measured"
            / "input_30cm_trial1"
        ),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=(
            repository
            / "ros2_ws/slam_results/odometry_noise/inputs"
        ),
    )
    parser.add_argument(
        "--config-root",
        type=Path,
        default=(
            repository
            / "ros2_ws/src/cleany_gazebo_sim/config"
        ),
    )
    parser.add_argument(
        "--source-odom-topic",
        default="/odom",
        help="Odometry topic to perturb and publish as /odom.",
    )
    args = parser.parse_args()
    materialize(
        args.input_bag,
        args.output_root,
        args.config_root,
        args.source_odom_topic,
    )


if __name__ == "__main__":
    main()
