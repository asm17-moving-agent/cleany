"""Translate the existing Gazebo facility-18F static scene into explicit primitives."""
from __future__ import annotations

from collections import Counter
import ast
import hashlib
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml


def prepare_facility(source: Path, output: Path, target_id: str) -> dict:
    package = source / "ros2_ws/src/cleany_gazebo_sim"
    sys.path.insert(0, str(package))
    sys.path.insert(0, str(source / "ros2_ws/src/cleany_navigation"))
    from cleany_gazebo_sim.world.facility_generator import materialize_facility_world
    from cleany_gazebo_sim.world.static_scene import _matrix
    from cleany_navigation.body_geometry import load_body_boxes
    layout_path = package / "config/facility_18f/facility_layout.yaml"
    layout = yaml.safe_load(layout_path.read_text())
    park_file = package / "config/cad_frame.yaml"
    park = yaml.safe_load(park_file.read_text())["park_joints"]
    urdf = ET.parse(output / "assets/cleany.urdf")
    adjustments = {}
    for name, angle in park.items():
        limit = urdf.find(f"joint[@name='{name}']/limit")
        low, high = float(limit.get("lower")), float(limit.get("upper"))
        bounded = min(high, max(low, angle))
        if angle != bounded:
            adjustments[name] = {"source": angle, "within_original_limit": bounded}
        park[name] = bounded
    # Use the existing park profile without its arm exclusions or limit extensions.
    profile = output / "config/park.yaml"
    profile.write_text(yaml.safe_dump({"park_joints": park}))
    boxes = load_body_boxes(source / "ros2_ws/src/cleany_description", profile, merge=False)
    corners = np.concatenate([box.corners() for box in boxes])
    bounds = [corners.min(axis=0).tolist(), corners.max(axis=0).tolist()]
    safety_file = source / "ros2_ws/src/cleany_navigation/config/nav2_safety.yaml"
    safety = yaml.safe_load(safety_file.read_text())
    polygon = np.array(ast.literal_eval(safety["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]))
    assert (corners[:, :2] >= polygon.min(axis=0)).all() and (corners[:, :2] <= polygon.max(axis=0)).all(), bounds
    world_file = materialize_facility_world(package / "worlds/cleany_mecanum_fortress.sdf",
        output / "assets/facility18.sdf", facility_layout_path=layout_path,
        robot_model="legacy", chair_model="roly")
    root = ET.parse(world_file)
    # Facility targets and the saved map use the unchanged SDF world frame.
    map_from_world = np.eye(4)
    entries, counts = [], Counter()
    checked_meshes = set()
    for model in root.findall("./world/model"):
        name = model.get("name")
        if name == "cleany_mecanum":
            continue
        if model.findtext("static", "false").lower() != "true":
            raise ValueError(f"Unsupported dynamic world model: {name}")
        counts[name.rsplit("_", 1)[0]] += 1
        for link in model.findall("link"):
            for kind in ("collision", "visual"):
                for element in link.findall(kind):
                    if any(p.attrib for p in (model.find("pose"), link.find("pose"), element.find("pose")) if p is not None):
                        raise ValueError("Explicit SDF pose frames require a frame resolver")
                    transform = map_from_world @ _matrix(model.find("pose")) @ _matrix(link.find("pose")) @ _matrix(element.find("pose"))
                    geometry = element.find("geometry")
                    shape = next(iter(geometry))
                    data = {"type": shape.tag}
                    if shape.tag == "box":
                        data["size"] = list(map(float, shape.findtext("size").split()))
                    elif shape.tag == "cylinder":
                        data.update(radius=float(shape.findtext("radius")), length=float(shape.findtext("length")))
                    elif shape.tag == "sphere":
                        data["radius"] = float(shape.findtext("radius"))
                    elif shape.tag == "plane":
                        assert shape.findtext("normal").split() == ["0", "0", "1"]
                        data.update(type="box", size=[*map(float, shape.findtext("size").split()), .1])
                        transform[2, 3] -= .05
                    elif shape.tag == "mesh":
                        uri = shape.findtext("uri")
                        if uri.endswith("/OfficeChairGrey.obj"):
                            asset = output / "assets/OfficeChairGrey.obj"
                        elif uri.startswith("file://"):
                            from urllib.parse import unquote, urlparse
                            asset = Path(unquote(urlparse(uri).path))
                            asset.relative_to(output / "assets")
                        else:
                            raise ValueError(f"Unsupported mesh: {uri}")
                        from mesh_geometry import read_geometry
                        if asset not in checked_meshes:
                            read_geometry(asset)
                            checked_meshes.add(asset)
                        data.update(asset=str(asset.relative_to(output)),
                                    scale=list(map(float, shape.findtext("scale", "1 1 1").split())))
                    else:
                        raise ValueError(f"Unsupported SDF shape: {shape.tag}")
                    color = list(map(float, element.findtext("material/diffuse", "0.25 0.27 0.29 1").split()))
                    entries.append(dict(model=name, link=link.get("name"), name=element.get("name"), kind=kind,
                                        transform=transform.tolist(), shape=data, color=color[:3],
                                        opacity=1.-float(element.findtext("transparency", "0"))))
    assert counts["demo_desk"] == counts["office_chair"] == counts["desk_monitor"] == 48
    assert counts["desk_partition"] == 24
    targets_file = source / "ros2_ws/src/cleany_mission_manager/config/full_facility_targets.yaml"
    targets = yaml.safe_load(targets_file.read_text())
    if target_id not in targets["seats"]:
        raise ValueError(f"Unknown facility-18F target: {target_id}")
    shutil.copyfile(targets_file, output / "config/targets.yaml")
    (output / "config/world_geometry.json").write_text(json.dumps(entries))
    check = verify_map(output, entries, {"home": targets["home"], "seats": {target_id: targets["seats"][target_id]}})
    sources = [layout_path, world_file, targets_file, output / "assets/facility_integration.yaml",
               output / "assets/facility_integration.pgm", *sorted(checked_meshes),
               package / "config/study_cafe/study_cafe_layout.yaml",
               package / "config/furniture/roly_p1g210m.yaml", park_file, profile, safety_file]
    evidence = {"source": "existing materialize_facility_world; ROLY D-HUB chairs; original room chair mesh; all static models",
                "counts": dict(counts), "collisions": sum(e["kind"] == "collision" for e in entries),
                "visuals": sum(e["kind"] == "visual" for e in entries),
                "map_from_sdf_world": map_from_world.tolist(), "map_check": check,
                "parking": {"source": str(park_file), "limit_adjustments": adjustments,
                            "full_body_bounds_in_base_frame": bounds, "excluded_links": [],
                            "within_existing_stop_footprint": True,
                            "initialization": "simulation starts parked; folding transition is not tested"},
                "sha256": {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}
    (output / "provenance/facility18.json").write_text(json.dumps(evidence, indent=2))
    return {"world": "facility18", "map_id": targets["map_id"], "map_file": "assets/facility_integration.yaml",
            "spawn": [*targets["home"][:2], .38], "spawn_yaw": targets["home"][2],
            "home": targets["home"], "target": targets["seats"][target_id], "target_id": target_id,
            "scan_z": .26, "scan_x": .16, "physics_hz": 500, "command_timeout_s": .5,
            "park_joints": park,
            "navigation_timeout_s": 600., "validation_timeout_s": 1800.,
            "navigation_start_timeout_s": 120.}


def verify_map(output: Path, entries: list[dict], targets: dict) -> dict:
    metadata = yaml.safe_load((output / "assets/facility_integration.yaml").read_text())
    if metadata["image"] != "facility_integration.pgm" or metadata["origin"][2] != 0.:
        raise ValueError("Expected facility_integration.pgm with an unrotated map origin")
    pixels = np.asarray(Image.open(output / "assets/facility_integration.pgm"))
    res, origin = metadata["resolution"], metadata["origin"]
    distance = distance_transform_edt(pixels > 100)*res
    distances = []
    # Compare unchanged vertical wall/desk/partition surfaces with the saved map.
    for entry in entries:
        if entry["kind"] != "collision" or entry["shape"]["type"] != "box" or not entry["model"].startswith("facility_wall_"):
            continue
        matrix = np.array(entry["transform"])
        half = np.array(entry["shape"]["size"])/2
        if not matrix[2, 3]-half[2] <= .26 <= matrix[2, 3]+half[2]:
            continue
        for axis in (0, 1):
            for sign in (-1, 1):
                for value in np.linspace(-half[1-axis], half[1-axis], 25):
                    point = np.array([0., 0., .26-matrix[2, 3], 1.])
                    point[axis], point[1-axis] = sign*half[axis], value
                    x, y = (matrix @ point)[:2]
                    column, row = int((x-origin[0])/res), pixels.shape[0]-1-int((y-origin[1])/res)
                    if 0 <= row < pixels.shape[0] and 0 <= column < pixels.shape[1]:
                        distances.append(float(distance[row, column]))
    assert distances and np.quantile(distances, .95) <= .15, "Facility wall/map alignment failed"
    for name, (x, y, yaw) in {"home": targets["home"], **targets["seats"]}.items():
        for dx in np.linspace(-.48, .48, 25):
            for dy in np.linspace(-.40, .40, 21):
                px, py = x+np.cos(yaw)*dx-np.sin(yaw)*dy, y+np.sin(yaw)*dx+np.cos(yaw)*dy
                column, row = int((px-origin[0])/res), pixels.shape[0]-1-int((py-origin[1])/res)
                assert 0 <= row < pixels.shape[0] and 0 <= column < pixels.shape[1] and pixels[row, column] > 200, (name, dx, dy)
    return {"surface_samples": len(distances), "median_surface_error_m": float(np.median(distances)),
            "p95_surface_error_m": float(np.quantile(distances, .95)), "targets_clear": ["home", *targets["seats"]], "scope": "wall alignment and selected target footprint; furniture checked by live collision/scan"}
