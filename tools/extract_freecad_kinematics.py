"""Extract evidence-backed axes and a CAD-relative kinematic preview.

Preview coordinates are changes from the saved assembly, not calibrated robot
angles. No encoder zeros or operational limits are inferred. Gripper loops
are solved only when their pin graph is supported by the source geometry.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any
from zipfile import ZipFile

import numpy as np
from export_freecad_model import (
    ROOT,
    numbers,
    placement,
    properties,
    save_xml,
    transformed,
)
from freecad_gripper_linkage import extract_linkages, resolved_deltas
from freecad_tip_mounts import analyze_tip_mounts, apply_tip_mounts


def canonical_direction(direction: np.ndarray) -> np.ndarray:
    """Choose an axis-line sign for reproducibility, not motor direction."""
    direction = np.asarray(direction, dtype=float)
    length = np.linalg.norm(direction)
    if not np.isfinite(length) or length == 0:
        raise ValueError("Axis must have a finite nonzero direction")
    direction = direction / length
    return direction if direction[np.argmax(np.abs(direction))] >= 0 else -direction


def axis_difference(first: dict, second: dict) -> tuple[float, float]:
    a, b = np.array(first["direction"]), np.array(second["direction"])
    angle = float(np.arccos(np.clip(abs(a @ b), 0, 1)))
    distance = float(
        np.linalg.norm(np.cross(np.array(second["point"]) - first["point"], a))
    )
    return distance, angle


def cylinder_axes(data: bytes) -> list[dict[str, Any]]:
    """Read analytic cylindrical faces directly from BREP, in CAD millimetres."""
    from OCP.BRep import BRep_Builder
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepGProp import BRepGProp
    from OCP.BRepTools import BRepTools
    from OCP.GeomAbs import GeomAbs_Cylinder
    from OCP.GProp import GProp_GProps
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS, TopoDS_Shape

    shape = TopoDS_Shape()
    BRepTools.Read_s(shape, io.BytesIO(data), BRep_Builder())
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    face_cast = getattr(TopoDS, "Face_s", None) or TopoDS.Face
    result = []
    index = 0
    while explorer.More():
        face = face_cast(explorer.Current())
        surface = BRepAdaptor_Surface(face)
        if surface.GetType() == GeomAbs_Cylinder:
            cylinder = surface.Cylinder()
            direction = canonical_direction(
                np.array(cylinder.Axis().Direction().Coord())
            )
            location = np.array(cylinder.Axis().Location().Coord())
            properties = GProp_GProps()
            BRepGProp.SurfaceProperties_s(face, properties)
            centroid = np.array(properties.CentreOfMass().Coord())
            point = location + direction * ((centroid - location) @ direction)
            result.append(
                {
                    "face_index": index,
                    "point": point.tolist(),
                    "direction": direction.tolist(),
                    "radius_mm": cylinder.Radius(),
                    "area_mm2": properties.Mass(),
                }
            )
        index += 1
        explorer.Next()
    return result


def grouped_axes(faces: list[dict], distance_mm: float, angle_rad: float) -> list[dict]:
    groups: list[dict] = []
    for face in faces:
        for group in groups:
            distance, angle = axis_difference(group, face)
            if distance <= distance_mm and angle <= angle_rad:
                group["faces"].append(face)
                group["area_mm2"] += face["area_mm2"]
                break
        else:
            groups.append(
                {
                    "point": face["point"],
                    "direction": face["direction"],
                    "faces": [face],
                    "area_mm2": face["area_mm2"],
                }
            )
    return sorted(groups, key=lambda item: -item["area_mm2"])


def matched_axis(
    spec: dict, axes: dict[str, list[dict]], distance: float, angle: float
) -> dict:
    """Require a unique horn axis supported by the servo and each named idler."""
    supports = [spec["servo"], *spec["supports"]]
    candidates = []
    for candidate in axes[spec["drive_horn"]]:
        matches = {}
        for name in supports:
            compatible = [
                a
                for a in axes[name]
                if all(
                    x <= y
                    for x, y in zip(axis_difference(candidate, a), (distance, angle))
                )
            ]
            if not compatible:
                break
            matches[name] = max(compatible, key=lambda a: a["area_mm2"])
        else:
            candidates.append((candidate, matches))
    if not candidates:
        raise ValueError(f"No supported axis for {spec.get('joint', spec.get('name'))}")
    if len(candidates) != 1:
        raise ValueError(
            f"Ambiguous supported axes for {spec.get('joint', spec.get('name'))}"
        )
    candidate, matches = max(candidates, key=lambda pair: pair[0]["area_mm2"])
    records = {spec["drive_horn"]: candidate, **matches}
    return {
        "point_cad_mm": candidate["point"],
        "direction_cad": candidate["direction"],
        "evidence": {
            name: {
                "faces": item["faces"],
                "axis_distance_mm": axis_difference(candidate, item)[0],
                "axis_angle_rad": axis_difference(candidate, item)[1],
            }
            for name, item in records.items()
        },
        "compatible_horn_axes": len(candidates),
    }


def extract(source: Path, package: Path, recipe_path: Path) -> dict[str, Any]:
    static = json.loads((package / "config/geometry/freecad_final.json").read_text())
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    if source_hash != static["source"]["sha256"]:
        raise ValueError("CAD source differs from the exported static assets")
    recipe = json.loads(recipe_path.read_text())
    distance, angle = recipe["axis_match_distance_mm"], recipe["axis_match_angle_rad"]
    if not np.isfinite([distance, angle]).all() or min(distance, angle) <= 0:
        raise ValueError("Axis matching tolerances must be finite and positive")
    specifications = [*recipe["links"], *recipe["unarticulated_drives"]]
    names = {
        name
        for spec in specifications
        for name in [spec["servo"], spec["drive_horn"], *spec["supports"]]
    }
    gripper_path = package / "config/geometry/freecad_gripper_structure.json"
    gripper_recipe = (
        json.loads(gripper_path.read_text()) if gripper_path.exists() else None
    )
    tip_path = package / "config/geometry/freecad_tip_mounts.json"
    tip_recipe = json.loads(tip_path.read_text()) if tip_path.exists() else None
    tip_evidence = []
    if gripper_recipe:
        names.update(
            p["name"]
            for p in static["parts"]
            if any(
                p["name"].startswith(hand["prefix"]) for hand in gripper_recipe["hands"]
            )
        )
    with ZipFile(source) as archive:
        document = ET.fromstring(archive.read("Document.xml"))
        objects = {o.get("name"): o for o in document.find("ObjectData")}
        parents = {}

        def visit(name, transform):
            props = properties(objects[name])
            if "Group" in props:
                for child in props["Group"].findall("LinkList/Link"):
                    visit(
                        child.get("value"),
                        transform @ placement(props.get("Placement")),
                    )
            else:
                parents[name] = transform

        visit("CleanyAssembly", np.eye(4))
        axes = {
            name: grouped_axes(
                cylinder_axes(archive.read(name + ".Shape.brp")), distance, angle
            )
            for name in sorted(names)
        }
        for name, groups in axes.items():
            for group in groups:
                group["point"] = transformed(np.array([group["point"]]), parents[name])[
                    0
                ].tolist()
                group["direction"] = canonical_direction(
                    parents[name][:3, :3] @ group["direction"]
                ).tolist()
                for face in group["faces"]:
                    face["point"] = transformed(
                        np.array([face["point"]]), parents[name]
                    )[0].tolist()
                    face["direction"] = canonical_direction(
                        parents[name][:3, :3] @ face["direction"]
                    ).tolist()
        if gripper_recipe and tip_recipe:
            tip_evidence = analyze_tip_mounts(
                archive, axes, parents, gripper_recipe, tip_recipe
            )
    matrix = np.array(static["coordinates"]["cad_to_base_link"])
    rotation = matrix[:3, :3] / 0.001
    result: dict[str, Any] = {
        "schema_version": 1,
        "source_sha256": source_hash,
        "recipe_sha256": hashlib.sha256(recipe_path.read_bytes()).hexdigest(),
        "coordinate": "delta_radians_from_saved_CAD_assembly",
        "axis_sign": "largest_base_link_component_positive; not encoder direction",
        "limits": {"axis_match_distance_mm": distance, "axis_match_angle_rad": angle},
        "capabilities": {
            "kinematic_preview": True,
            "hardware_control": False,
            "dynamic_simulation": False,
            "gripper_actuation": False,
        },
        "links": [],
        "unarticulated_drives": [],
        "part_owners": {},
    }
    reference_origins = {"base_link": np.zeros(3)}
    for spec in recipe["links"]:
        axis = matched_axis(spec, axes, distance, angle)
        point = transformed(np.array([axis["point_cad_mm"]]), matrix)[0]
        direction = canonical_direction(rotation @ axis["direction_cad"])
        if spec["parent"] not in reference_origins:
            raise ValueError("Recipe must list parents before children")
        result["links"].append(
            {
                **spec,
                **axis,
                "point_base_m": point.tolist(),
                "axis_base": direction.tolist(),
                "origin_parent_m": (point - reference_origins[spec["parent"]]).tolist(),
                "encoder_zero_tick": None,
                "physical_zero_rad": None,
                "encoder_direction": None,
                "operational_limits_rad": None,
                "velocity_limit_rad_s": None,
                "effort_limit": None,
            }
        )
        reference_origins[spec["link"]] = point
    for spec in recipe["unarticulated_drives"]:
        axis = matched_axis(spec, axes, distance, angle)
        result["unarticulated_drives"].append(
            {
                **spec,
                **axis,
                "point_base_m": transformed(np.array([axis["point_cad_mm"]]), matrix)[
                    0
                ].tolist(),
                "axis_base": canonical_direction(
                    rotation @ axis["direction_cad"]
                ).tolist(),
                "linkage": None,
                "aperture_curve": None,
                "physical_zero_rad": None,
            }
        )
    for part in static["parts"]:
        owners = [
            spec["link"]
            for spec in recipe["links"]
            if part["name"] in spec["parts"]
            or any(g in spec["groups"] for g in part["groups"])
        ]
        if len(owners) > 1:
            raise ValueError(f"Multiple rigid owners: {part['name']}")
        result["part_owners"][part["name"]] = owners[0] if owners else "base_link"
    for spec in recipe["links"]:
        if not any(owner == spec["link"] for owner in result["part_owners"].values()):
            raise ValueError(f"Empty kinematic link {spec['link']}")
        servo_owner = result["part_owners"][spec["servo"]]
        horn_owner = result["part_owners"][spec["drive_horn"]]
        if servo_owner != spec["parent"] or horn_owner != spec["link"]:
            raise ValueError(f"Drive/servo ownership disagrees with {spec['joint']}")
    if gripper_recipe:
        extract_linkages(axes, static, gripper_recipe, result)
        result["gripper_recipe_sha256"] = hashlib.sha256(
            gripper_path.read_bytes()
        ).hexdigest()
    if tip_evidence:
        apply_tip_mounts(result, tip_evidence, matrix)
        result["tip_recipe_sha256"] = hashlib.sha256(tip_path.read_bytes()).hexdigest()
    write_models(package, static, result)
    (package / "config/geometry/freecad_kinematics.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    return result


def write_models(package: Path, static: dict, kinematics: dict) -> None:
    urdf = ET.Element("robot", name="cleany_freecad_delta_preview")
    urdf.append(
        ET.Comment(
            "Unbounded CAD-relative preview coordinates only; not physical zeros or limits."
        )
    )
    urdf_links = {"base_link": ET.SubElement(urdf, "link", name="base_link")}
    origins = {"base_link": np.zeros(3)}
    for spec in [*kinematics["links"], *kinematics.get("fixed_attachments", [])]:
        name = spec["link"]
        origins[name] = np.array(spec["point_base_m"])
        urdf_links[name] = ET.SubElement(urdf, "link", name=name)
        joint = ET.SubElement(
            urdf, "joint", name=spec["joint"], type=spec.get("joint_type", "continuous")
        )
        ET.SubElement(joint, "parent", link=spec["parent"])
        ET.SubElement(joint, "child", link=name)
        ET.SubElement(
            joint, "origin", xyz=numbers(spec["origin_parent_m"]), rpy="0 0 0"
        )
        if "axis_base" in spec:
            ET.SubElement(joint, "axis", xyz=numbers(spec["axis_base"]))
    mjcf = ET.parse(package / "mjcf/cleany_freecad_final.xml").getroot()
    mjcf.set("model", "cleany_freecad_mocap_preview")
    body = mjcf.find("worldbody/body")
    body.set("name", "base_link")
    mjcf_bodies = {"base_link": body}
    for spec in kinematics["links"]:
        name = spec["link"]
        mjcf_bodies[name] = ET.SubElement(
            mjcf.find("worldbody"),
            "body",
            name=name,
            mocap="true",
            pos=numbers(origins[name] + [0, 0, 0.38]),
        )
    for spec in kinematics.get("fixed_attachments", []):
        mjcf_bodies[spec["link"]] = ET.SubElement(
            mjcf_bodies[spec["parent"]],
            "body",
            name=spec["link"],
            pos=numbers(spec["origin_parent_m"]),
        )
    for part in static["parts"]:
        owner = kinematics["part_owners"][part["name"]]
        offset = -origins[owner]
        for kind in ("visual", "collision"):
            element = ET.SubElement(
                urdf_links[owner], kind, name=part["name"] + "_" + kind
            )
            ET.SubElement(element, "origin", xyz=numbers(offset), rpy="0 0 0")
            ET.SubElement(
                ET.SubElement(element, "geometry"),
                "mesh",
                filename="package://cleany_description/" + part["mesh"],
                scale="1 1 1",
            )
            if kind == "visual":
                ET.SubElement(
                    ET.SubElement(element, "material", name=part["name"] + "_color"),
                    "color",
                    rgba=numbers(part["color_rgba"]),
                )
        if owner != "base_link":
            for mesh in part["mujoco_meshes"]:
                geom = body.find(f"geom[@name='{mesh['name']}']")
                body.remove(geom)
                geom.set("pos", numbers(offset))
                mjcf_bodies[owner].append(geom)
    for plane in static["image_planes"]:
        visual = ET.SubElement(urdf_links["base_link"], "visual", name=plane["name"])
        ET.SubElement(
            ET.SubElement(visual, "geometry"),
            "mesh",
            filename="package://cleany_description/meshes/freecad_final/"
            + plane["name"]
            + ".dae",
            scale="1 1 1",
        )
    save_xml(package / "urdf/cleany_freecad_kinematic.urdf", urdf)
    save_xml(package / "mjcf/cleany_freecad_kinematic.xml", mjcf)


def rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def forward_kinematics(
    kinematics: dict, delta_radians: dict[str, float]
) -> dict[str, np.ndarray]:
    """Return link transforms for CAD-relative deltas, without motor calibration."""
    delta_radians = resolved_deltas(kinematics, delta_radians)
    transforms = {"base_link": np.eye(4)}
    for spec in kinematics["links"]:
        local = np.eye(4)
        local[:3, 3] = spec["origin_parent_m"]
        local[:3, :3] = rotation_matrix(
            spec["axis_base"], delta_radians.get(spec["joint"], 0.0)
        )
        transforms[spec["link"]] = transforms[spec["parent"]] @ local
    for spec in kinematics.get("fixed_attachments", []):
        local = np.eye(4)
        local[:3, 3] = spec["origin_parent_m"]
        transforms[spec["link"]] = transforms[spec["parent"]] @ local
    return transforms


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument(
        "--package", type=Path, default=ROOT / "ros2_ws/src/cleany_description"
    )
    parser.add_argument("--recipe", type=Path)
    args = parser.parse_args()
    recipe = args.recipe or args.package / "config/geometry/freecad_link_structure.json"
    result = extract(args.source, args.package, recipe)
    print(
        json.dumps(
            {
                "preview_joints": len(result["links"]),
                "independent_inputs": len(
                    result.get("independent_joint_names", result["links"])
                ),
                "gripper_pin_loops": len(result.get("gripper_linkages", [])),
                "fixed_tip_mounts": len(result.get("fixed_attachments", [])),
                "owned_parts": len(result["part_owners"]),
            }
        )
    )


if __name__ == "__main__":
    main()
