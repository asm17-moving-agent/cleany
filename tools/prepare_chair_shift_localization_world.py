#!/usr/bin/env python3
"""Create a deterministic moved-chair study-cafe localization world."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from xml.etree import ElementTree

import yaml
from ament_index_python.packages import get_package_share_directory

from cleany_gazebo_sim.gazebo_slam_experiment import (
    load_mount_profiles,
    write_sensor_tf_config,
)
from cleany_gazebo_sim.world.generator import materialize_study_cafe_world


SHIFTED_CHAIRS = (2, 7, 10, 15, 18, 23, 26, 31, 34, 39, 42, 47)
HEIGHTS = {
    "16p5": (16.5, "floor_16p5cm"),
    "26": (26.0, "floor_26cm"),
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

    height_cm, profile_name = HEIGHTS[args.height]
    package_share = Path(get_package_share_directory("cleany_gazebo_sim"))
    profile = load_mount_profiles(
        package_share / "config/lidar_mount_profiles.yaml"
    )[profile_name]
    world_path = args.output_dir / "world.sdf"
    materialize_study_cafe_world(
        package_share / "worlds/cleany_mecanum_harmonic.sdf",
        world_path,
        simulator="harmonic",
        max_step_size=0.004,
        real_time_factor=2.5,
        lidar_translation=profile.transform.translation,
    )
    tree = ElementTree.parse(world_path)
    world = tree.getroot().find("world")
    if world is None:
        raise RuntimeError("generated world is missing its world element")
    changes = shift_chairs(world, args.chair_shift_m)
    tree.write(world_path, encoding="unicode", xml_declaration=True)
    write_sensor_tf_config(profile, args.output_dir / "sensor_tf.yaml")
    manifest = {
        "height_cm": height_cm,
        "frame_id": profile.transform.child_frame_id,
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
