"""Compose the full floor around the existing D-HUB furniture/robot frame."""

from __future__ import annotations

import json
from math import atan2, cos, hypot, radians, sin
from pathlib import Path
from tempfile import gettempdir
from xml.etree import ElementTree as ET

from cleany_gazebo_sim.world.facility_layout import load_facility_layout
from cleany_gazebo_sim.world.felt_floor import apply_felt_floor
from cleany_gazebo_sim.world.facility_architecture import (
    add_central_table,
    add_ceiling,
    add_glazing,
)
from cleany_gazebo_sim.world.generator import (
    _add_box_model,
    _add_box_part,
    _add_office_chair,
    materialize_study_cafe_world,
)


def materialize_facility_world(
    robot_template_path: Path,
    target_path: Path | None = None,
    *,
    facility_layout_path: Path | None = None,
    layout_path: Path | None = None,
    robot_model: str = "cad_frame",
    chair_model: str = "roly",
    chair_config_path: Path | None = None,
    max_step_size: float = 0.001,
    real_time_factor: float = 1.0,
    lidar_translation: tuple[float, float, float] | None = None,
) -> Path:
    package = robot_template_path.parent.parent
    layout = load_facility_layout(
        facility_layout_path or package / "config/facility_18f/facility_layout.yaml"
    )
    target = target_path or Path(gettempdir()) / "cleany_facility_18f/world.sdf"
    base = materialize_study_cafe_world(
        robot_template_path,
        target.with_suffix(".study.sdf"),
        max_step_size=max_step_size,
        real_time_factor=real_time_factor,
        layout_path=layout_path,
        lidar_translation=lidar_translation,
        chair_model=chair_model,
        robot_model=robot_model,
        chair_config_path=chair_config_path,
    )
    root = ET.parse(base).getroot()
    if robot_model == "cad_frame":
        base.with_suffix(".model.json").replace(target.with_suffix(".model.json"))
    world = root.find("world")
    world.set("name", "cleany_facility_18f")
    for name in ("wall_north", "wall_south", "wall_east", "wall_west"):
        world.remove(world.find(f"model[@name='{name}']"))
    world.find("model[@name='cleany_mecanum']/pose").text = " ".join(
        map(str, layout.raw["spawn_pose"])
    )
    bounds = layout.raw["floor_bounds"]
    west, north = layout.world(bounds[:2])
    east, south = layout.world(bounds[2:])
    ground = world.find("model[@name='ground_plane']")
    ground.find("pose").text = f"{(west + east) / 2} {(north + south) / 2} 0 0 0 0"
    for size in ground.findall("link/*/geometry/plane/size"):
        size.text = f"{east - west + 0.4} {north - south + 0.4}"
    apply_felt_floor(
        ground.find("link/visual"),
        east - west + 0.4,
        north - south + 0.4,
        package,
        target,
        layout.raw["floor"]["texture_tile_m"],
    )
    thickness = layout.raw["wall"]["thickness_m"]
    height = layout.raw["wall"]["height_m"]
    color = " ".join(map(str, layout.raw["wall"]["color"]))
    for segment in layout.segments:
        (ax, ay), (bx, by) = segment.start, segment.end
        if segment.material == "glass":
            add_glazing(
                world,
                f"facility_wall_{segment.name}",
                segment.start,
                segment.end,
                height,
                layout.raw["glass"],
            )
            continue
        _add_box_model(
            world,
            f"facility_wall_{segment.name}",
            (
                (ax + bx) / 2,
                (ay + by) / 2,
                height / 2,
                0.0,
                0.0,
                atan2(by - ay, bx - ax),
            ),
            (hypot(bx - ax, by - ay), thickness, height),
            "0.48 0.39 0.22 1" if segment.guard else color,
            roughness=0.92,
        )
    for door in layout.raw["doors"]:
        if door.get("material") == "glass":
            add_glazing(
                world,
                f"facility_door_{door['id']}",
                *(layout.world(point) for point in door["points"]),
                height,
                layout.raw["glass"],
                door["open_angle_deg"],
            )
    add_central_table(world, layout)
    add_ceiling(world, layout, target)
    manifest = []
    for i in range(1, 49):
        model = world.find(f"model[@name='demo_desk_{i:02d}']")
        manifest.append(
            {
                "seat_id": f"{i:02d}",
                "zone_id": "d-hub",
                "desk_model": model.get("name"),
                "chair_model": f"office_chair_{i:02d}",
                "pose": list(map(float, model.findtext("pose").split())),
            }
        )
    furniture = layout.raw["room_furniture"]
    width, depth = (x / layout.units_per_meter for x in furniture["desk_size_units"])
    top = furniture["desk_top_m"]
    for room in layout.raw["rooms"]:
        angle = radians(room["rotation_deg"])
        for row, row_sign in enumerate((-1, 1)):
            for column in range(room["columns"]):
                local_x = (column - (room["columns"] - 1) / 2) * furniture[
                    "column_spacing_units"
                ]
                local_y = row_sign * furniture["row_offset_units"]
                map_point = (
                    room["center"][0] + local_x * cos(angle) - local_y * sin(angle),
                    room["center"][1] + local_x * sin(angle) + local_y * cos(angle),
                )
                x, y = layout.world(map_point)
                name = f"{room['label']}{row * room['columns'] + column + 1}"
                model = ET.SubElement(world, "model", name=f"facility_desk_{name}")
                ET.SubElement(model, "static").text = "true"
                ET.SubElement(model, "pose").text = f"{x} {y} 0 0 0 {-angle}"
                link = ET.SubElement(model, "link", name="body")
                _add_box_part(
                    link,
                    "tabletop",
                    (width, depth, 0.04),
                    (0.0, 0.0, top - 0.02, 0.0, 0.0, 0.0),
                    "0.88 0.90 0.89 1",
                )
                for ix in (-1, 1):
                    for iy in (-1, 1):
                        _add_box_part(
                            link,
                            f"leg_{ix}_{iy}",
                            (0.04, 0.04, top - 0.04),
                            (
                                ix * (width / 2 - 0.06),
                                iy * (depth / 2 - 0.06),
                                (top - 0.04) / 2,
                                0.0,
                                0.0,
                                0.0,
                            ),
                            "0.62 0.65 0.66 1",
                        )
                # Like D-HUB, this is measured from the desk center; the chair
                # seat tucks beneath the edge rather than floating in the aisle.
                outward = furniture["chair_offset_m"]
                cx, cy = (
                    x - row_sign * outward * sin(angle),
                    y - row_sign * outward * cos(angle),
                )
                _add_office_chair(
                    world,
                    f"facility_chair_{name}",
                    (cx, cy, 0.0, 0.0, 0.0, atan2(y - cy, x - cx)),
                )
                manifest.append(
                    {
                        "seat_id": name,
                        "zone_id": room["id"],
                        "desk_model": model.get("name"),
                        "chair_model": f"facility_chair_{name}",
                        "pose": [x, y, 0.0, 0.0, 0.0, -angle],
                    }
                )
    target.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(target, encoding="unicode", xml_declaration=True)
    target.with_suffix(".seats.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    return target
