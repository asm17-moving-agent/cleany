"""Export a saved FreeCAD assembly as a non-actuated ROS/MuJoCo reference.

Only persisted BREP geometry, placements, colors and image planes are read.
FreeCAD expressions, macros and external documents are never executed.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any
from zipfile import ZipFile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PROFILE = "freecad_final"
MJCF_MAX_STL_TRIANGLES = 200_000
# Coordinate convention only: retain the existing base_link 380 mm above ground.
CAD_TO_BASE = np.array(
    [
        [0.0, -0.001, 0.0, 0.0],
        [0.001, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.001, -0.2845],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
STL_TRIANGLE = np.dtype(
    [
        ("normal", "<f4", (3,)),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ]
)


@dataclass(frozen=True)
class Part:
    """One persisted solid in the source assembly, expressed in base_link."""

    name: str
    label: str
    groups: tuple[str, ...]
    vertices: np.ndarray
    faces: np.ndarray
    color: tuple[float, float, float, float]
    source_brep_sha256: str


def properties(element: ET.Element) -> dict[str, ET.Element]:
    return {p.attrib["name"]: p for p in element.findall("Properties/Property")}


def placement(prop: ET.Element | None) -> np.ndarray:
    """Decode a FreeCAD placement in millimetres without evaluating expressions."""
    result = np.eye(4)
    if prop is None:
        return result
    values = prop.find("PropertyPlacement").attrib
    x, y, z, w = (float(values[f"Q{i}"]) for i in range(4))
    norm = np.linalg.norm([x, y, z, w])
    if not np.isfinite(norm) or norm == 0:
        raise ValueError("Invalid placement quaternion")
    x, y, z, w = np.array([x, y, z, w]) / norm
    result[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    result[:3, 3] = [float(values[f"P{axis}"]) for axis in "xyz"]
    if not np.isfinite(result).all():
        raise ValueError("Nonfinite placement")
    return result


def transformed(vertices: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return vertices @ matrix[:3, :3].T + matrix[:3, 3]


def within_bounds(
    vertices: np.ndarray, faces: np.ndarray, bounds: np.ndarray, tolerance: float
) -> bool:
    """Reject mesher output outside the source face, including bogus origin nodes."""
    if not len(faces) or faces.min() < 0 or faces.max() >= len(vertices):
        return False
    used = vertices[faces]
    return bool(
        np.isfinite(used).all()
        and (used >= bounds[0] - tolerance).all()
        and (used <= bounds[1] + tolerance).all()
    )


def solid_mesh(data: bytes, deflection_mm: float) -> tuple[np.ndarray, np.ndarray]:
    """Tessellate BREP including its own stored TopLoc placement.

    The XML object's placement must not be applied again. App::Part ancestor
    placements are applied by the caller because they are outside the BREP.
    """
    from OCP.Bnd import Bnd_Box
    from OCP.BRep import BRep_Builder, BRep_Tool
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepTools import BRepTools
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS, TopoDS_Shape

    shape = TopoDS_Shape()
    BRepTools.Read_s(shape, io.BytesIO(data), BRep_Builder())
    if shape.IsNull():
        raise ValueError("Empty BREP")
    BRepMesh_IncrementalMesh(shape, deflection_mm, False, 0.35, True)
    vertices, faces = [], []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    face_cast = getattr(TopoDS, "Face_s", None) or TopoDS.Face
    while explorer.More():
        face = face_cast(explorer.Current())
        box = Bnd_Box()
        BRepBndLib.AddOptimal_s(face, box, False, False)
        bounds = np.array([box.CornerMin().Coord(), box.CornerMax().Coord()])
        face_vertices, face_indices = None, None
        for tolerance in (None, 0.05, 0.005, 0.0005):
            if tolerance is not None:
                BRepTools.Clean_s(face)
                BRepMesh_IncrementalMesh(
                    face, min(deflection_mm, tolerance), False, 0.15, False
                )
            location = TopLoc_Location()
            triangles = BRep_Tool.Triangulation_s(face, location)
            if triangles is None:
                continue
            candidate_vertices = np.array(
                [
                    triangles.Node(i).Transformed(location.Transformation()).Coord()
                    for i in range(1, triangles.NbNodes() + 1)
                ]
            )
            candidate_faces = (
                np.array(
                    [
                        triangles.Triangle(i).Get()
                        for i in range(1, triangles.NbTriangles() + 1)
                    ],
                    dtype=np.int64,
                )
                - 1
            )
            if within_bounds(
                candidate_vertices, candidate_faces, bounds, deflection_mm
            ):
                used, inverse = np.unique(candidate_faces, return_inverse=True)
                face_vertices = candidate_vertices[used]
                face_indices = inverse.reshape(-1, 3)
                break
        if face_vertices is None:
            from OCP.BRepGProp import BRepGProp
            from OCP.GProp import GProp_GProps

            area = GProp_GProps()
            BRepGProp.SurfaceProperties_s(face, area)
            if abs(area.Mass()) <= 1e-10:
                explorer.Next()
                continue
            raise ValueError(
                f"A BREP face has no valid tessellation; area={area.Mass()} mm^2"
            )
        start = len(vertices)
        vertices.extend(face_vertices)
        for indices in face_indices + start:
            faces.append(
                indices[::-1] if face.Orientation() == TopAbs_REVERSED else indices
            )
        explorer.Next()
    v, f = np.asarray(vertices), np.asarray(faces, dtype=np.int64)
    if not len(f) or not np.isfinite(v).all():
        raise ValueError("Empty or nonfinite tessellation")
    return v, f


def color_for(archive: ZipFile, view: ET.Element) -> tuple[float, float, float, float]:
    materials = properties(view)["ShapeAppearance"].find("MaterialList")
    if materials.attrib.get("version", "3") != "3":
        raise ValueError("Unsupported FreeCAD material format")
    data = archive.read(materials.attrib["file"])
    if struct.unpack_from("<I", data)[0] != 1:
        raise ValueError("Per-face materials require a separate exporter")
    # Serialized packed RGBA uses little-endian A, B, G, R byte order.
    return tuple(
        component / 255 for component in (data[11], data[10], data[9], data[8])
    )


def read_assembly(
    archive: ZipFile, deflection_mm: float
) -> tuple[list[Part], dict[str, Any]]:
    document = ET.fromstring(archive.read("Document.xml"))
    gui = ET.fromstring(archive.read("GuiDocument.xml"))
    objects = {o.attrib["name"]: o for o in document.find("ObjectData")}
    views = {v.attrib["name"]: v for v in gui.find("ViewProviderData")}
    if "CleanyAssembly" not in objects:
        raise ValueError("Expected the CleanyAssembly source root")
    parts: list[Part] = []
    images: list[dict[str, Any]] = []
    visited: set[str] = set()

    def visit(name: str, ancestors: tuple[str, ...], parent: np.ndarray) -> None:
        if name in visited:
            raise ValueError(f"Repeated or cyclic assembly reference: {name}")
        visited.add(name)
        p = properties(objects[name])
        visibility = p.get("Visibility")
        if visibility is not None and visibility.find("Bool").attrib["value"] != "true":
            return
        group = p.get("Group")
        if group is not None:
            matrix = parent @ placement(p.get("Placement"))
            for child in group.findall("LinkList/Link"):
                visit(child.attrib["value"], (*ancestors, name), matrix)
        elif "Shape" in p:
            source = p["Shape"].find("Part")
            if source is None:
                source = next(c for c in p["Shape"] if "file" in c.attrib)
            brep = archive.read(source.attrib["file"])
            try:
                vertices, faces = solid_mesh(brep, deflection_mm)
            except ValueError as error:
                raise ValueError(f"Cannot export {name}: {error}") from error
            parts.append(
                Part(
                    name=name,
                    label=p["Label"].find("String").attrib["value"],
                    groups=ancestors[1:],
                    vertices=transformed(vertices, CAD_TO_BASE @ parent),
                    faces=faces,
                    color=color_for(archive, views[name]),
                    source_brep_sha256=hashlib.sha256(brep).hexdigest(),
                )
            )
        elif "ImageFile" in p:
            width = float(p["XSize"].find("Float").attrib["value"])
            height = float(p["YSize"].find("Float").attrib["value"])
            local = np.array(
                [
                    [-width / 2, -height / 2, 0],
                    [width / 2, -height / 2, 0],
                    [width / 2, height / 2, 0],
                    [-width / 2, height / 2, 0],
                ]
            )
            images.append(
                {
                    "name": name,
                    "groups": ancestors[1:],
                    "vertices": transformed(
                        local, CAD_TO_BASE @ parent @ placement(p.get("Placement"))
                    ),
                    "source_file": p["ImageFile"].find("FileIncluded").attrib["file"]
                    if p["ImageFile"].find("FileIncluded") is not None
                    else next(iter(p["ImageFile"])).attrib["file"],
                }
            )

    visit("CleanyAssembly", (), np.eye(4))
    status = properties(objects["CleanyAssembly"]).get("DesignStatus")
    return parts, {
        "program": document.attrib["ProgramVersion"],
        "images": images,
        "design_status": status.find("String").get("value")
        if status is not None
        else None,
    }


def stl_bytes(part: Part) -> bytes:
    triangles = part.vertices[part.faces]
    normal = np.cross(
        triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    )
    length = np.linalg.norm(normal, axis=1)
    normal = normal / np.maximum(length[:, None], 1e-30)
    data = np.zeros(len(triangles), dtype=STL_TRIANGLE)
    data["normal"], data["vertices"] = normal, triangles
    return (
        b"Cleany FreeCAD saved assembly; metres".ljust(80, b"\0")
        + struct.pack("<I", len(data))
        + data.tobytes()
    )


def numbers(values: Any) -> str:
    return " ".join(f"{float(v):.12g}" for v in np.asarray(values).ravel())


def save_xml(path: Path, root: ET.Element) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def write_image_mesh(path: Path, vertices: np.ndarray, source_timestamp: str) -> None:
    """Write a textured quad for URDF; this cosmetic plane has no collision."""
    root = ET.Element(
        "COLLADA", xmlns="http://www.collada.org/2005/11/COLLADASchema", version="1.4.1"
    )
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "created").text = source_timestamp
    ET.SubElement(asset, "modified").text = source_timestamp
    ET.SubElement(asset, "unit", name="meter", meter="1")
    ET.SubElement(asset, "up_axis").text = "Z_UP"
    images = ET.SubElement(root, "library_images")
    ET.SubElement(
        ET.SubElement(images, "image", id="screen-image"), "init_from"
    ).text = "idle.png"
    effect = ET.SubElement(
        ET.SubElement(root, "library_effects"), "effect", id="screen-effect"
    )
    profile = ET.SubElement(effect, "profile_COMMON")
    surface = ET.SubElement(
        ET.SubElement(profile, "newparam", sid="surface"), "surface", type="2D"
    )
    ET.SubElement(surface, "init_from").text = "screen-image"
    sampler = ET.SubElement(
        ET.SubElement(profile, "newparam", sid="sampler"), "sampler2D"
    )
    ET.SubElement(sampler, "source").text = "surface"
    lambert = ET.SubElement(
        ET.SubElement(profile, "technique", sid="common"), "lambert"
    )
    ET.SubElement(
        ET.SubElement(lambert, "diffuse"),
        "texture",
        texture="sampler",
        texcoord="UVSET0",
    )
    material = ET.SubElement(
        ET.SubElement(root, "library_materials"), "material", id="screen-material"
    )
    ET.SubElement(material, "instance_effect", url="#screen-effect")
    geometry = ET.SubElement(
        ET.SubElement(root, "library_geometries"), "geometry", id="screen-geometry"
    )
    mesh = ET.SubElement(geometry, "mesh")
    for name, values, components in [
        ("position", vertices, "XYZ"),
        ("uv", [[0, 0], [1, 0], [1, 1], [0, 1]], "ST"),
    ]:
        source = ET.SubElement(mesh, "source", id=name)
        ET.SubElement(
            source,
            "float_array",
            id=name + "-array",
            count=str(np.asarray(values).size),
        ).text = numbers(values)
        accessor = ET.SubElement(
            ET.SubElement(source, "technique_common"),
            "accessor",
            source="#" + name + "-array",
            count="4",
            stride=str(len(components)),
        )
        for component in components:
            ET.SubElement(accessor, "param", name=component, type="float")
    ET.SubElement(
        ET.SubElement(mesh, "vertices", id="vertices"),
        "input",
        semantic="POSITION",
        source="#position",
    )
    triangles = ET.SubElement(mesh, "triangles", count="2", material="screen")
    ET.SubElement(triangles, "input", semantic="VERTEX", source="#vertices", offset="0")
    ET.SubElement(
        triangles, "input", semantic="TEXCOORD", source="#uv", offset="1", set="0"
    )
    ET.SubElement(triangles, "p").text = "0 0 1 1 2 2 0 0 2 2 3 3"
    scene = ET.SubElement(
        ET.SubElement(root, "library_visual_scenes"), "visual_scene", id="scene"
    )
    instance = ET.SubElement(
        ET.SubElement(scene, "node", id="screen-node"),
        "instance_geometry",
        url="#screen-geometry",
    )
    binding = ET.SubElement(
        ET.SubElement(ET.SubElement(instance, "bind_material"), "technique_common"),
        "instance_material",
        symbol="screen",
        target="#screen-material",
    )
    ET.SubElement(
        binding,
        "bind_vertex_input",
        semantic="UVSET0",
        input_semantic="TEXCOORD",
        input_set="0",
    )
    ET.SubElement(ET.SubElement(root, "scene"), "instance_visual_scene", url="#scene")
    save_xml(path, root)


def export(source: Path, package: Path, deflection_mm: float = 0.35) -> dict[str, Any]:
    """Write metre-scale assets and static model files under a package directory."""
    if not np.isfinite(deflection_mm) or deflection_mm <= 0:
        raise ValueError("Deflection must be a finite positive millimetre value")
    with ZipFile(source) as archive:
        parts, metadata = read_assembly(archive, deflection_mm)
        mesh_dir = package / "meshes" / PROFILE
        mesh_dir.mkdir(parents=True, exist_ok=True)
        urdf = ET.Element("robot", name="cleany_freecad_final_static")
        urdf.append(
            ET.Comment(
                "Saved CAD assembly only. No calibrated joint zeros, actuators or inertias."
            )
        )
        links = {"base_link": ET.SubElement(urdf, "link", name="base_link")}
        mjcf = ET.Element("mujoco", model="cleany_freecad_final_static")
        ET.SubElement(
            mjcf,
            "compiler",
            angle="radian",
            inertiafromgeom="false",
            meshdir="../meshes/freecad_final",
            texturedir="../meshes/freecad_final",
        )
        assets = ET.SubElement(mjcf, "asset")
        world = ET.SubElement(mjcf, "worldbody")
        body = ET.SubElement(world, "body", name="freecad_final_static", pos="0 0 0.38")
        manifest: dict[str, Any] = {
            "schema_version": 1,
            "profile": PROFILE,
            "model_kind": "saved_static_assembly",
            "source": {
                "filename": source.name,
                "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "program": metadata["program"],
                "design_status": metadata["design_status"],
                "license": "not specified in supplied file",
            },
            "coordinates": {
                "source_length_unit": "mm",
                "target_length_unit": "m",
                "cad_to_base_link": CAD_TO_BASE.tolist(),
                "base_link_height_above_ground_m": 0.38,
                "cad_ground_z_mm": -95.5,
                "meaning": "Coordinate convention, not a measured calibration. Front is the display side (CAD -Y), ROS +X.",
            },
            "tessellation": {
                "linear_deflection_mm": deflection_mm,
                "angular_deflection_rad": 0.35,
            },
            "capabilities": {
                "static_geometry": True,
                "joint_motion": False,
                "actuator_commands": False,
                "dynamic_simulation": False,
                "calibrated_sensor_frames": False,
            },
            "source_groups_to_ros_side": {"RightArm": "right", "LeftArm": "left"},
            "unresolved": {
                key: None
                for key in [
                    "joint_axes_and_zero_positions",
                    "joint_limits_and_encoder_mapping",
                    "gripper_linkage_and_aperture_curve",
                    "gripper_tcp_and_contact_thresholds",
                    "material_densities_masses_and_inertias",
                    "sensor_optical_and_measurement_frames",
                    "motion_collision_geometry_and_self_collision_exclusions",
                    "home_and_return_trajectories",
                ]
            },
            "parts": [],
            "image_planes": [],
        }

        def group_link(groups: tuple[str, ...]) -> ET.Element:
            group = groups[-1] if groups else "assembly"
            name = "cad_snapshot_" + group
            if name not in links:
                links[name] = ET.SubElement(urdf, "link", name=name)
                joint = ET.SubElement(urdf, "joint", name=name + "_fixed", type="fixed")
                ET.SubElement(joint, "parent", link="base_link")
                ET.SubElement(joint, "child", link=name)
                ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
            return links[name]

        for part in parts:
            filename = part.name + ".stl"
            data = stl_bytes(part)
            (mesh_dir / filename).write_bytes(data)
            link = group_link(part.groups)
            for kind in ("visual", "collision"):
                element = ET.SubElement(link, kind, name=part.name + "_" + kind)
                ET.SubElement(
                    ET.SubElement(element, "geometry"),
                    "mesh",
                    filename="package://cleany_description/meshes/freecad_final/"
                    + filename,
                    scale="1 1 1",
                )
                if kind == "visual":
                    ET.SubElement(
                        ET.SubElement(element, "material", name=part.name + "_color"),
                        "color",
                        rgba=numbers(part.color),
                    )
            mjcf_meshes = []
            for index, start in enumerate(
                range(0, len(part.faces), MJCF_MAX_STL_TRIANGLES)
            ):
                if len(part.faces) <= MJCF_MAX_STL_TRIANGLES:
                    name, relative, chunk_data = part.name, filename, data
                    count = len(part.faces)
                else:
                    name = part.name + f"__chunk_{index:02}"
                    relative = "mujoco/" + name + ".stl"
                    chunk = Part(
                        name,
                        part.label,
                        part.groups,
                        part.vertices,
                        part.faces[start : start + MJCF_MAX_STL_TRIANGLES],
                        part.color,
                        part.source_brep_sha256,
                    )
                    chunk_data = stl_bytes(chunk)
                    count = len(chunk.faces)
                    (mesh_dir / "mujoco").mkdir(exist_ok=True)
                    (mesh_dir / relative).write_bytes(chunk_data)
                ET.SubElement(assets, "mesh", name=name, file=relative)
                ET.SubElement(
                    body,
                    "geom",
                    name=name,
                    type="mesh",
                    mesh=name,
                    rgba=numbers(part.color),
                    contype="0",
                    conaffinity="0",
                    group="2",
                )
                mjcf_meshes.append(
                    {
                        "name": name,
                        "mesh": "meshes/freecad_final/" + relative,
                        "triangles": count,
                        "sha256": hashlib.sha256(chunk_data).hexdigest(),
                    }
                )
            manifest["parts"].append(
                {
                    "name": part.name,
                    "label": part.label,
                    "groups": part.groups,
                    "mesh": "meshes/freecad_final/" + filename,
                    "sha256": hashlib.sha256(data).hexdigest(),
                    "source_brep_sha256": part.source_brep_sha256,
                    "triangles": len(part.faces),
                    "color_rgba": part.color,
                    "mujoco_meshes": mjcf_meshes,
                    "bounds_base_link_m": [
                        part.vertices.min(axis=0).tolist(),
                        part.vertices.max(axis=0).tolist(),
                    ],
                }
            )
        for image in metadata["images"]:
            name, vertices = image["name"], image["vertices"]
            image_data = archive.read(image["source_file"])
            (mesh_dir / "idle.png").write_bytes(image_data)
            source_timestamp = datetime(
                *archive.getinfo("Document.xml").date_time
            ).isoformat()
            write_image_mesh(mesh_dir / (name + ".dae"), vertices, source_timestamp)
            visual = ET.SubElement(group_link(image["groups"]), "visual", name=name)
            ET.SubElement(
                ET.SubElement(visual, "geometry"),
                "mesh",
                filename="package://cleany_description/meshes/freecad_final/"
                + name
                + ".dae",
                scale="1 1 1",
            )
            ET.SubElement(
                assets, "texture", name=name + "_texture", type="2d", file="idle.png"
            )
            ET.SubElement(
                assets, "material", name=name + "_material", texture=name + "_texture"
            )
            horizontal, vertical = vertices[1] - vertices[0], vertices[3] - vertices[0]
            width, height = np.linalg.norm(horizontal), np.linalg.norm(vertical)
            ET.SubElement(
                body,
                "geom",
                name=name,
                type="plane",
                pos=numbers(vertices.mean(axis=0)),
                xyaxes=numbers([horizontal / width, vertical / height]),
                size=numbers([width / 2, height / 2, 0.01]),
                material=name + "_material",
                contype="0",
                conaffinity="0",
                group="2",
            )
            manifest["image_planes"].append(
                {
                    "name": name,
                    "source_file": image["source_file"],
                    "image_sha256": hashlib.sha256(image_data).hexdigest(),
                    "vertices_base_link_m": vertices.tolist(),
                    "collision": False,
                }
            )
        save_xml(package / "urdf" / "cleany_freecad_final.urdf", urdf)
        save_xml(package / "mjcf" / "cleany_freecad_final.xml", mjcf)
        manifest_path = package / "config" / "geometry" / "freecad_final.json"
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n"
        )
        return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument(
        "--package", type=Path, default=ROOT / "ros2_ws/src/cleany_description"
    )
    parser.add_argument("--deflection-mm", type=float, default=0.35)
    args = parser.parse_args()
    manifest = export(args.source, args.package, args.deflection_mm)
    print(
        json.dumps(
            {
                "source_sha256": manifest["source"]["sha256"],
                "parts": len(manifest["parts"]),
                "image_planes": len(manifest["image_planes"]),
                "triangles": sum(p["triangles"] for p in manifest["parts"]),
            }
        )
    )


if __name__ == "__main__":
    main()
