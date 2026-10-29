"""Export unchanged Cleany geometry with a synthetic room or existing facility18."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import xacro
import yaml


def prepare(source: Path, output: Path, world: str = "room", target_id: str | None = None,
            publication_profile: str = "split", physics_sync: str = "fabric") -> None:
    from cadence import publication_config
    for name in ("assets", "config", "logs", "evaluations", "provenance"):
        (output / name).mkdir(parents=True, exist_ok=True)
    description = source / "ros2_ws/src/cleany_description"
    index = output / "ament/share/ament_index/resource_index/packages"
    index.mkdir(parents=True, exist_ok=True)
    (index / "cleany_description").touch()
    link = output / "ament/share/cleany_description"
    if not link.exists():
        link.symlink_to(description)
    os.environ["AMENT_PREFIX_PATH"] = str(output / "ament") + ":" + os.environ.get("AMENT_PREFIX_PATH", "")
    xml = xacro.process_file(str(description / "urdf/cleany.urdf.xacro"),
                             mappings={"include_wheel_joints": "false"}).toxml()
    (output / "assets/cleany.urdf").write_text(xml)
    meshes = {}
    for mesh in ET.fromstring(xml).findall(".//mesh"):
        name = mesh.attrib["filename"]
        path = Path(name.replace("package://cleany_description", str(description)))
        meshes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    (output / "provenance/model.json").write_text(json.dumps({
        "urdf_sha256": hashlib.sha256(xml.encode()).hexdigest(), "mesh_sha256": meshes,
        "base_actuation": "ideal planar root velocity; fixed wheels; not wheel/contact dynamics",
    }, indent=2))
    sim = source / "ros2_ws/src/cleany_mujoco_sim"
    (output / "config/controllers.yaml").write_bytes((sim / "config/study_cafe_ros2_controllers.yaml").read_bytes())
    motor_source = description / "mjcf/cleany.xml"
    mjcf = ET.parse(motor_source).getroot()
    motors = {}
    for actuator in mjcf.findall("./actuator/position"):
        defaults = mjcf.find(f".//default[@class='{actuator.attrib['class']}']")
        motors[actuator.attrib["joint"]] = dict(defaults.find("joint").attrib)
    (output / "config/motors.json").write_text(json.dumps(motors, indent=2))
    if world == "facility18":
        from facility import prepare_facility
        room = prepare_facility(source, output, target_id or "seat-12")
        room.update(publication_config(publication_profile, room['physics_hz']))
        room['physics_sync'] = physics_sync
        (output / "config/room.json").write_text(json.dumps(room, indent=2))
        return
    # The room and map share this single geometry source. This is not the cafe map.
    room = {"world": "room", "map_id": "isaac_room", "map_file": "assets/isaac_room.yaml", "target_id": "seat-1",
            "bounds": [-2., -2., 4., 2.], "resolution": .05,
            "spawn": [0., 0., .38], "desk": {"center": [2.45, -.35, .675], "size": [.6, .8, .05]},
            "home": [0., 0., 0.], "target": [1.55, 0., 0.],
            "scan_z": .50, "physics_hz": 500, "command_timeout_s": .5}
    room.update(publication_config(publication_profile, room['physics_hz']))
    room['physics_sync'] = physics_sync
    (output / "config/room.json").write_text(json.dumps(room, indent=2))
    xmin, ymin, xmax, ymax = room["bounds"]
    res = room["resolution"]
    grid = np.full((round((ymax-ymin)/res), round((xmax-xmin)/res)), 254, dtype=np.uint8)
    grid[[0, -1], :] = 0
    grid[:, [0, -1]] = 0
    # Solid desk pedestal is visible at LiDAR height and represented in the map.
    cx, cy, _ = room["desk"]["center"]
    sx, sy, _ = room["desk"]["size"]
    for iy in range(grid.shape[0]):
        for ix in range(grid.shape[1]):
            x, y = xmin+(ix+.5)*res, ymax-(iy+.5)*res
            if abs(x-cx) <= sx/2 and abs(y-cy) <= sy/2:
                grid[iy, ix] = 0
    (output / "assets/isaac_room.pgm").write_bytes(f"P5\n{grid.shape[1]} {grid.shape[0]}\n255\n".encode()+grid.tobytes())
    (output / "assets/isaac_room.yaml").write_text(yaml.safe_dump({
        "image": "isaac_room.pgm", "resolution": res, "origin": [xmin, ymin, 0.],
        "negate": 0, "occupied_thresh": .65, "free_thresh": .196}))
    (output / "config/targets.yaml").write_text(yaml.safe_dump({
        "schema_version": 1, "map_id": "isaac_room", "frame_id": "map",
        "home": room["home"], "seats": {"seat-1": room["target"]}}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path("/main"))
    parser.add_argument("--output", type=Path, default=Path("/output"))
    parser.add_argument("--world", choices=["room", "facility18"], default="room")
    parser.add_argument("--target-id", type=str)
    parser.add_argument("--publication-profile", choices=["legacy", "split"], default="split")
    parser.add_argument("--physics-sync", choices=["usd", "fabric"], default="fabric")
    parser.add_argument("--cleanup", action="store_true")
    parser.add_argument('--cleanup-backend', choices=['physical', 'mock'], default='physical')
    parser.add_argument("--cleanup-object", choices=['cup','tissue'], default='cup')
    parser.add_argument("--cleanup-profile", choices=['canonical','slender-cup-95mm'], default='canonical')
    args = parser.parse_args()
    prepare(args.source, args.output, args.world, args.target_id, args.publication_profile, args.physics_sync)

    if args.cleanup:
        from prepare_cleanup import prepare_cleanup
        prepare_cleanup(args.output, args.source, object_name=args.cleanup_object,
                        profile=args.cleanup_profile, backend=args.cleanup_backend)
