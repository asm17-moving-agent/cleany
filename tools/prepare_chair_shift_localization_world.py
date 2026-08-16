#!/usr/bin/env python3
"""Create a deterministic moved-chair study-cafe localization world."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from xml.etree import ElementTree

import yaml
from ament_index_python.packages import get_package_share_directory

from cleany_gazebo_sim.world_generator import materialize_study_cafe_world


SHIFTED_CHAIRS = (2, 7, 10, 15, 18, 23, 26, 31, 34, 39, 42, 47)
HEIGHTS = {
    "12": (12.0, "lidar_12cm_mount", -0.26, "lidar_12cm_link"),
    "16p5": (16.5, "lidar_12cm_mount", -0.215, "lidar_12cm_link"),
    "26": (26.0, "lidar_mount", -0.12, "lidar_link"),
}


def shift_chairs(world: ElementTree.Element, distance_m: float) -> list[dict]:
    changes: list[dict] = []
    for order, index in enumerate(SHIFTED_CHAIRS):
        chair = world.find(f"model[@name='office_chair_{index:02d}']")
        desk = world.find(f"model[@name='demo_desk_{index:02d}']")
        if chair is None or desk is None:
            raise RuntimeError(f"chair/desk pair {index:02d} is missing")
        pose = chair.find("pose")
        if pose is None or pose.text is None:
            raise RuntimeError(f"chair {index:02d} has no pose")
        values = [float(value) for value in pose.text.split()]
        desk_y = float(desk.findtext("pose", "0 0").split()[1])
        direction = 1.0 if values[1] > desk_y else -1.0
        before = values.copy()
        # Tuck the chair toward its desk. Pulling both opposing rows into a
        # cross aisle by 20 cm leaves less clearance than the robot width and
        # turns a localization test into an obstacle-blockage test.
        values[1] -= direction * distance_m
        values[5] += math.radians(10.0 if order % 2 == 0 else -10.0)
        pose.text = " ".join(str(value) for value in values)
        changes.append({
            "name": chair.get("name"),
            "before": before,
            "after": values,
        })
    return changes


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("height", choices=HEIGHTS)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--chair-shift-m", type=float, default=0.20)
    args = parser.parse_args()
    if not 0.0 < args.chair_shift_m <= 0.5:
        parser.error("--chair-shift-m must be within (0, 0.5]")
    args.output_dir.mkdir(parents=True, exist_ok=False)

    height_cm, mount_name, relative_z, frame_id = HEIGHTS[args.height]
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
    world = tree.getroot().find("world")
    if world is None:
        raise RuntimeError("generated world is missing its world element")
    changes = shift_chairs(world, args.chair_shift_m)
    robot = world.find("model[@name='cleany_mecanum']")
    if robot is None:
        raise RuntimeError("generated world has no Cleany model")
    mount = robot.find(f"joint[@name='{mount_name}']/pose")
    if mount is None:
        raise RuntimeError(f"generated world has no {mount_name}")
    mount.text = f"0.16 0 {relative_z} 0 0 0"
    tree.write(world_path, encoding="unicode", xml_declaration=True)

    with (package_share / "config/base.yaml").open(encoding="utf-8") as stream:
        sensor_config = yaml.safe_load(stream)
    if args.height in {"12", "16p5"}:
        parameters = sensor_config[
            "gazebo_sensor_tf_publisher"
        ]["ros__parameters"]
        parameters["lidar_12cm_translation"] = [0.16, 0.0, relative_z]
    with (args.output_dir / "sensor_tf.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(sensor_config, stream, sort_keys=False)
    manifest = {
        "height_cm": height_cm,
        "frame_id": frame_id,
        "chair_shift_m": args.chair_shift_m,
        "chair_yaw_deg": 10.0,
        "changes": changes,
        "physics_max_step_size": 0.004,
        "physics_real_time_factor": 2.5,
    }
    with (args.output_dir / "manifest.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(manifest, stream, sort_keys=False)


if __name__ == "__main__":
    main()
