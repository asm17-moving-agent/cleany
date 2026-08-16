#!/usr/bin/env python3
"""Materialize the study cafe with the lower LiDAR at a 16.5 cm height."""

from __future__ import annotations

import argparse
from pathlib import Path
from xml.etree import ElementTree

import yaml
from ament_index_python.packages import get_package_share_directory

from cleany_gazebo_sim.world_generator import materialize_study_cafe_world


LOWER_LIDAR_RELATIVE_Z_M = -0.215


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)

    package_share = Path(get_package_share_directory("cleany_gazebo_sim"))
    world_path = args.output_dir / "world.sdf"
    materialize_study_cafe_world(
        package_share / "worlds/cleany_mecanum_harmonic.sdf",
        world_path,
        simulator="harmonic",
        max_step_size=0.004,
        real_time_factor=2.5,
    )
    tree = ElementTree.parse(world_path)
    model = tree.getroot().find(
        "./world/model[@name='cleany_mecanum']"
    )
    if model is None:
        raise RuntimeError("generated world has no Cleany model")
    mount_pose = model.find("joint[@name='lidar_12cm_mount']/pose")
    if mount_pose is None:
        raise RuntimeError("generated world has no lower LiDAR mount")
    mount_pose.text = f"0.16 0 {LOWER_LIDAR_RELATIVE_Z_M} 0 0 0"
    tree.write(world_path, encoding="unicode", xml_declaration=True)

    with (package_share / "config/base.yaml").open(encoding="utf-8") as stream:
        sensor_config = yaml.safe_load(stream)
    parameters = sensor_config["gazebo_sensor_tf_publisher"]["ros__parameters"]
    parameters["lidar_12cm_translation"] = [
        0.16,
        0.0,
        LOWER_LIDAR_RELATIVE_Z_M,
    ]
    sensor_path = args.output_dir / "sensor_tf.yaml"
    with sensor_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(sensor_config, stream, sort_keys=False)

    manifest = {
        "height_cm": 16.5,
        "frame_id": "lidar_12cm_link",
        "legacy_gazebo_topic": "/model/cleany_mecanum/lidar_12cm/scan",
        "translation_from_base_link": [0.16, 0.0, LOWER_LIDAR_RELATIVE_Z_M],
        "physics_max_step_size": 0.004,
        "physics_real_time_factor": 2.5,
    }
    with (args.output_dir / "manifest.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(manifest, stream, sort_keys=False)


if __name__ == "__main__":
    main()
