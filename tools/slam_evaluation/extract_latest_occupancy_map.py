#!/usr/bin/env python3
"""Extract the latest OccupancyGrid from a result bag as a ROS map."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image
import rosbag2_py
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message


def extract(input_bag: Path, output_prefix: Path) -> None:
    """Write the final /map message to PGM, PNG and YAML files."""
    metadata = yaml.safe_load(
        (input_bag / "metadata.yaml").read_text(encoding="utf-8")
    )
    information = metadata.get("rosbag2_bagfile_information", metadata)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(input_bag),
            storage_id=str(information["storage_identifier"]),
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    latest: OccupancyGrid | None = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/map":
            latest = deserialize_message(data, OccupancyGrid)
    if latest is None:
        raise RuntimeError(f"no /map message in result bag: {input_bag}")

    grid = np.asarray(latest.data, dtype=np.int16).reshape(
        int(latest.info.height), int(latest.info.width)
    )
    pixels = np.full(grid.shape, 205, dtype=np.uint8)
    pixels[grid <= 25] = 254
    pixels[grid >= 65] = 0
    pixels[grid < 0] = 205
    pixels = np.flipud(pixels)
    image = Image.fromarray(pixels)
    image.save(output_prefix.with_suffix(".pgm"))
    image.save(output_prefix.with_suffix(".png"))
    origin = latest.info.origin
    document = {
        "image": output_prefix.with_suffix(".pgm").name,
        "mode": "trinary",
        "resolution": float(latest.info.resolution),
        "origin": [
            float(origin.position.x),
            float(origin.position.y),
            0.0,
        ],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    output_prefix.with_suffix(".yaml").write_text(
        yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
    )


def main() -> None:
    """Parse arguments and extract the latest map."""
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_prefix", type=Path)
    args = parser.parse_args()
    extract(args.input_bag, args.output_prefix)


if __name__ == "__main__":
    main()
