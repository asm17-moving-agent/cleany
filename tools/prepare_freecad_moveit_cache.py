"""Build CAD collision primitives, partitioned enclosures and reduced surfaces."""

from __future__ import annotations

import argparse
import json
import shutil
import tempfile
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
from hashlib import sha256
from pathlib import Path
from urllib.parse import unquote, urlsplit
from xml.etree import ElementTree as ET

import numpy as np
import trimesh
from cleany_moveit_config.freecad_collision import inspection_urdf
from freecad_component_enclosures import selected_parts, write_enclosures
from optimize_arm_collision_meshes import Mesh, read_stl
from prepare_freecad_collision_cache import write_indexed_glb
from scipy.spatial.transform import Rotation


def _values(values: np.ndarray) -> str:
    return " ".join(format(float(value), ".17g") for value in values)


def transform_points(collision: ET.Element, vertices: np.ndarray) -> np.ndarray:
    origin = collision.find("origin")
    xyz = (
        [float(value) for value in origin.get("xyz", "0 0 0").split()]
        if origin is not None
        else [0.0] * 3
    )
    rpy = (
        [float(value) for value in origin.get("rpy", "0 0 0").split()]
        if origin is not None
        else [0.0] * 3
    )
    mesh = collision.find("geometry/mesh")
    scale = [float(value) for value in mesh.get("scale", "1 1 1").split()]
    return (vertices * scale) @ Rotation.from_euler("xyz", rpy).as_matrix().T + xyz


def enclosing_primitive(
    name: str, points: np.ndarray, axis: np.ndarray | None = None
) -> tuple[ET.Element, dict]:
    """Enclose all vertices, hence complete source triangles, after serialization.

    Only floating-point outward rounding is added; no physical clearance or
    calibration is inferred. A cylinder is coaxial with the supplied joint axis.
    """
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or not len(points)
        or not np.isfinite(points).all()
    ):
        raise ValueError("Primitive input must contain finite vertices")
    collision = ET.Element("collision", name=f"moveit_enclosure_{name}")
    if axis is None:
        center = (points.min(axis=0) + points.max(axis=0)) / 2
        half = np.nextafter(np.max(np.abs(points - center), axis=0), np.inf)
        ET.SubElement(collision, "origin", xyz=_values(center), rpy="0 0 0")
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "box", size=_values(2 * half))
        report = {"type": "box", "center": center.tolist(), "size": (2 * half).tolist()}
    else:
        if (
            axis.shape != (3,)
            or not np.isfinite(axis).all()
            or np.linalg.norm(axis) == 0
        ):
            raise ValueError("Cylinder needs a finite nonzero joint axis")
        axis = axis / np.linalg.norm(axis)
        order = np.array([2, 0, 1])
        helper = np.eye(3)[order[np.argmin(np.abs(axis[order]))]]
        first = np.cross(helper, axis)
        first /= np.linalg.norm(first)
        matrix = np.column_stack((first, np.cross(axis, first), axis))
        rpy = Rotation.from_matrix(matrix).as_euler("xyz")
        matrix = Rotation.from_euler("xyz", rpy).as_matrix()
        projected = points @ matrix
        center = matrix[:, 2] * ((projected[:, 2].min() + projected[:, 2].max()) / 2)
        local = (points - center) @ matrix
        radius = np.nextafter(np.linalg.norm(local[:, :2], axis=1).max(), np.inf)
        length = 2 * np.nextafter(np.abs(local[:, 2]).max(), np.inf)
        ET.SubElement(collision, "origin", xyz=_values(center), rpy=_values(rpy))
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(
            geometry,
            "cylinder",
            radius=format(radius, ".17g"),
            length=format(length, ".17g"),
        )
        report = {
            "type": "cylinder",
            "center": center.tolist(),
            "rpy": rpy.tolist(),
            "radius": float(radius),
            "length": float(length),
        }
    # Evaluate the exact numbers carried by the output URDF.
    restored = ET.fromstring(ET.tostring(collision))
    origin = restored.find("origin")
    center = np.fromstring(origin.get("xyz"), sep=" ")
    matrix = Rotation.from_euler(
        "xyz", np.fromstring(origin.get("rpy"), sep=" ")
    ).as_matrix()
    local = (points - center) @ matrix
    shape = restored.find("geometry")[0]
    if shape.tag == "box":
        slack = np.fromstring(shape.get("size"), sep=" ") / 2 - np.abs(local)
        minimum = float(slack.min())
    else:
        minimum = min(
            float(shape.get("radius")) - np.linalg.norm(local[:, :2], axis=1).max(),
            float(shape.get("length")) / 2 - np.abs(local[:, 2]).max(),
        )
    if minimum < 0:
        raise ValueError("Serialized primitive fails source containment")
    report["minimum_source_containment_slack_m"] = float(minimum)
    report["source_vertices"] = len(points)
    return restored, report


def select_sets(parts: dict[str, dict], settings: dict) -> dict[str, list[str]]:
    if settings.get("schema") != "cleany.freecad_moveit_primitives/v1":
        raise ValueError("Unknown primitive configuration schema")
    selections = {}
    enclosed = set(selected_parts(parts, settings))
    body = settings.get("body_enclosure")
    if body:
        selections[body["name"]] = [
            name
            for name, part in parts.items()
            if set(part["groups"]) & set(body["groups"])
        ]
    for group in settings["box_each_groups"]:
        for name, part in parts.items():
            if group in part["groups"] and name not in enclosed:
                selections[name] = [name]
    for group in settings["box_together_groups"]:
        selections[group] = [
            name for name, part in parts.items() if group in part["groups"]
        ]
    for entry in settings["box_sets"]:
        selections[entry["name"]] = sorted(
            set(entry.get("parts", []))
            | {
                name
                for name in parts
                if any(
                    name.startswith(prefix) for prefix in entry.get("part_prefixes", [])
                )
            }
        )
    flattened = [name for names in selections.values() for name in names]
    expected = (
        set(settings.get("require_parts", []))
        | {
            name
            for name, part in parts.items()
            if set(part["groups"]) & set(settings["require_groups"])
        }
    ) - enclosed
    if (
        any(not names for names in selections.values())
        or len(flattened) != len(set(flattened))
        or set(flattened) != expected
    ):
        raise ValueError(
            "Primitive selections must cover the configured groups exactly once"
        )
    if not set(settings["wheel_parts"]) <= set(flattened):
        raise ValueError("Wheel parts must be explicitly selected")
    return selections


def build_moveit_cache(
    package: Path, source_cache: Path, output: Path, config: Path, *, jobs: int = 1
) -> dict:
    """Preserve all nonselected collisions and all joints, visuals and frames."""
    package, source_cache, output = (
        package.resolve(),
        source_cache.resolve(),
        output.resolve(),
    )
    if (
        output.exists()
        or output.is_relative_to(package)
        or output.is_relative_to(source_cache)
    ):
        raise ValueError("Use a new output outside the source package and cache")
    metadata = (package / "config/geometry/freecad_final.json").read_bytes()
    parts = {part["name"]: part for part in json.loads(metadata)["parts"]}
    settings = json.loads(config.read_text())
    surface_errors = settings.get("surface_error_by_group_m", {})
    if jobs < 1 or any(
        not np.isfinite(value) or not 0 < value <= 0.001
        for value in surface_errors.values()
    ):
        raise ValueError("Use positive jobs and surface error limits in (0, 0.001]")
    if set(surface_errors) - {
        group for part in parts.values() for group in part["groups"]
    }:
        raise ValueError("Unknown surface reduction group")
    preserve_parts = set(settings.get("surface_preserve_parts", []))
    if preserve_parts - parts.keys():
        raise ValueError("Unknown preserved surface part")
    selections = select_sets(parts, settings)
    enclosure_parts = selected_parts(parts, settings)
    protected_parts = set(settings.get("preserve_collision_parts", []))
    if protected_parts - parts.keys():
        raise ValueError("Unknown protected collision part")
    if protected_parts & (
        set(enclosure_parts) | {name for names in selections.values() for name in names}
    ):
        raise ValueError("Protected collision parts cannot become enclosures")
    preserve_parts |= protected_parts
    source_manifest = json.loads((source_cache / "manifest.json").read_text())
    if source_manifest.get("mode", "indexed") not in ("indexed", "surface_bounded"):
        raise ValueError("Start from an indexed or surface-bounded mesh cache")
    filename = "cleany_freecad_kinematic.urdf"
    original = (package / "urdf" / filename).read_bytes()
    reference = ET.fromstring(original)
    root = ET.fromstring(inspection_urdf(package, "freecad_kinematic", source_cache))
    source_collisions = {}
    target_collisions = {}
    retained_sources = {}
    mesh_parts = {
        "package://cleany_description/" + asset["mesh"]: (part, asset)
        for part in parts.values()
        for asset in part["meshes"]
    }
    for before, after in zip(
        reference.findall("link"), root.findall("link"), strict=True
    ):
        for source, target in zip(
            before.findall("collision"), after.findall("collision"), strict=True
        ):
            uri = source.find("geometry/mesh").get("filename")
            if uri in source_collisions:
                raise ValueError(
                    "Source collision assets must have unambiguous ownership"
                )
            source_collisions[uri] = (before, source)
            target_collisions[uri] = (after, target)
            retained_sources[target.find("geometry/mesh").get("filename")] = mesh_parts[
                uri
            ]
    joints = {
        joint.find("child").get("link"): joint for joint in reference.findall("joint")
    }
    records = []
    body_report = None
    for name, members in selections.items():
        points, targets, owners = [], [], set()
        member_meshes = {}
        triangles = 0
        for member in members:
            for asset in parts[member]["meshes"]:
                relative = asset["mesh"]
                if (
                    not Path(relative).is_relative_to("meshes/freecad_final")
                    or ".." in Path(relative).parts
                ):
                    raise ValueError("Invalid source mesh path")
                uri = "package://cleany_description/" + relative
                owner, collision = source_collisions[uri]
                path = package / relative
                if sha256(path.read_bytes()).hexdigest() != asset["sha256"]:
                    raise ValueError("Source mesh differs from its FreeCAD manifest")
                mesh = read_stl(path)
                transformed = transform_points(collision, mesh.vertices)
                points.append(transformed)
                member_meshes.setdefault(member, []).append(
                    Mesh(transformed, mesh.faces)
                )
                targets.append(target_collisions[uri])
                owners.add(owner.get("name"))
                triangles += len(mesh.faces)
        if len(owners) != 1:
            raise ValueError("Cannot merge geometry across independently moving links")
        owner_name = owners.pop()
        body = settings.get("body_enclosure")
        if body and name == body["name"]:
            from freecad_body_enclosure import body_boxes

            if owner_name != "base_link":
                raise ValueError("The torso enclosure must belong to base_link")
            boxes, body_report = body_boxes(
                [mesh for meshes in member_meshes.values() for mesh in meshes],
                [
                    mesh
                    for part in body["opening_parts"]
                    for mesh in member_meshes[part]
                ],
                member_meshes[body["floor_part"]],
            )
            for owner, target in targets:
                owner.remove(target)
            for index, bounds in enumerate(boxes):
                collision, record = enclosing_primitive(f"{name}_{index}", bounds)
                targets[0][0].append(collision)
                records.append(
                    {
                        "name": f"{name}_{index}",
                        "link": owner_name,
                        "parts": members,
                        "containment": "collective_body_boxes",
                        **record,
                    }
                )
            continue
        axis = None
        if set(members) & set(settings["wheel_parts"]):
            if (
                len(members) != 1
                or owner_name not in joints
                or joints[owner_name].get("type") not in ("revolute", "continuous")
                or joints[owner_name].find("axis") is None
            ):
                raise ValueError("Each wheel must retain its own joint link")
            axis = np.fromstring(joints[owner_name].find("axis").get("xyz"), sep=" ")
        collision, record = enclosing_primitive(name, np.vstack(points), axis)
        for owner, target in targets:
            owner.remove(target)
        targets[0][0].append(collision)
        records.append(
            {
                "name": name,
                "link": owner_name,
                "parts": members,
                "source_triangles": triangles,
                **record,
            }
        )
    # Neither visual nor kinematic state may be changed by this profile.
    check_before, check_after = deepcopy(reference), deepcopy(root)
    for model in (check_before, check_after):
        for link in model.findall("link"):
            for collision in link.findall("collision"):
                link.remove(collision)
    if ET.tostring(check_before) != ET.tostring(check_after):
        raise ValueError("Primitive profile changed non-collision model data")
    output.parent.mkdir(parents=True, exist_ok=True)
    with (
        tempfile.TemporaryDirectory(dir=output.parent) as temporary,
        ProcessPoolExecutor(max_workers=jobs) as executor,
    ):
        stage = Path(temporary) / "cache"
        (stage / "meshes").mkdir(parents=True)
        assets = {}
        checks = {}
        pending = []
        enclosure_pending = []
        enclosure_records = []
        rigid_sets = settings.get("component_enclosures", {}).get("sets", [])
        grouped = {name for group in rigid_sets for name in group["parts"]}
        plans = [
            (name, [name], spec)
            for name, spec in enclosure_parts.items()
            if name not in grouped
        ]
        plans.extend(
            (group["name"], group["parts"], {**group, "combine_components": True})
            for group in rigid_sets
        )
        for name, members, spec in plans:
            sources = [asset for member in members for asset in parts[member]["meshes"]]
            targets = []
            for source_asset in sources:
                source = package / source_asset["mesh"]
                if sha256(source.read_bytes()).hexdigest() != source_asset["sha256"]:
                    raise ValueError("Source mesh differs from its FreeCAD manifest")
                uri = "package://cleany_description/" + source_asset["mesh"]
                owner, collision = target_collisions[uri]
                if list(
                    map(
                        float,
                        collision.find("geometry/mesh").get("scale", "1 1 1").split(),
                    )
                ) != [1.0, 1.0, 1.0]:
                    raise ValueError(
                        "Enclosures require source mesh coordinates in metres"
                    )
                targets.append((owner, collision))
            owner, template = targets[0]
            if any(other is not owner for other, _ in targets):
                raise ValueError("Cannot merge independently moving enclosure parts")

            def serialized_origin(collision):
                element = collision.find("origin")
                return ET.tostring(element) if element is not None else None

            if any(
                serialized_origin(collision) != serialized_origin(template)
                for _, collision in targets
            ):
                raise ValueError(
                    "Rigid enclosure sources must share a mesh coordinate frame"
                )
            directory = Path("meshes/enclosures") / name
            for owner, collision in targets:
                owner.remove(collision)
            enclosure_pending.append(
                (
                    name,
                    members,
                    sources,
                    owner,
                    template,
                    directory,
                    executor.submit(
                        write_enclosures,
                        [package / asset["mesh"] for asset in sources],
                        stage / directory,
                        settings["component_enclosures"],
                        spec,
                    ),
                )
            )
        for mesh in root.findall("link/collision/geometry/mesh"):
            url = urlsplit(mesh.get("filename"))
            path = Path(unquote(url.path)).resolve()
            if (
                url.scheme != "file"
                or url.netloc
                or not path.is_relative_to(source_cache / "meshes")
            ):
                raise ValueError("Unexpected retained collision mesh path")
            relative = str(path.relative_to(source_cache))
            if relative not in assets:
                (stage / relative).parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(path, stage / relative)
                assets[relative] = sha256((stage / relative).read_bytes()).hexdigest()
                checks[relative] = source_manifest.get("geometry_checks", {}).get(
                    relative
                ) or {
                    "triangles": len(
                        trimesh.load(path, force="mesh", process=False).faces
                    )
                }
                checks[relative] = dict(checks[relative])
                checks[relative]["surface_error_limit_m"] = source_manifest.get(
                    "max_surface_error_m"
                )
                part, source_asset = retained_sources[mesh.get("filename")]
                limits = [
                    surface_errors[group]
                    for group in part["groups"]
                    if group in surface_errors
                ]
                if limits and part["name"] not in preserve_parts:
                    source = package / source_asset["mesh"]
                    if (
                        sha256(source.read_bytes()).hexdigest()
                        != source_asset["sha256"]
                    ):
                        raise ValueError(
                            "Source mesh differs from its FreeCAD manifest"
                        )
                    limit = min(limits)
                    pending.append(
                        (
                            relative,
                            path,
                            limit,
                            executor.submit(
                                write_indexed_glb, source, stage / relative, limit
                            ),
                        )
                    )
            mesh.set("filename", relative)
        for relative, baseline, limit, future in pending:
            report = future.result()
            if report["triangles"] < checks[relative]["triangles"]:
                checks[relative] = {**report, "surface_error_limit_m": limit}
            else:
                shutil.copyfile(baseline, stage / relative)
            assets[relative] = sha256((stage / relative).read_bytes()).hexdigest()
        for (
            name,
            members,
            sources,
            owner,
            template,
            directory,
            future,
        ) in enclosure_pending:
            pieces = future.result()
            for index, report in enumerate(pieces):
                relative = str(directory / report["file"])
                collision = deepcopy(template)
                collision.set("name", f"cad_enclosure__{name}_{index}")
                collision.find("geometry/mesh").set("filename", relative)
                owner.append(collision)
                assets[relative] = sha256((stage / relative).read_bytes()).hexdigest()
                checks[relative] = {**report, "mode": "convex_enclosure"}
            enclosure_records.append(
                {
                    "part": name,
                    "link": owner.get("name"),
                    "source_parts": members,
                    "source_meshes": sources,
                    "directory": str(directory),
                    "pieces": pieces,
                }
            )
        data = ET.tostring(root, encoding="utf-8", xml_declaration=True)
        (stage / filename).write_bytes(data)
        manifest = {
            "format": "glb",
            "mode": "moveit_primitives",
            "source_geometry_sha256": sha256(metadata).hexdigest(),
            "source_cache_manifest_sha256": sha256(
                (source_cache / "manifest.json").read_bytes()
            ).hexdigest(),
            "config": settings,
            "config_sha256": sha256(config.read_bytes()).hexdigest(),
            "models": {
                filename: {
                    "source_sha256": sha256(original).hexdigest(),
                    "indexed_sha256": sha256(data).hexdigest(),
                }
            },
            "assets": assets,
            "geometry_checks": checks,
            "primitives": records,
            "body_enclosure": body_report,
            "component_enclosures": enclosure_records,
            "preserved_collision_parts": sorted(protected_parts),
            "collision_detector": "cleany/CADInspectionFCL"
            if enclosure_records
            else "FCL",
            "meshes": len(assets),
            "triangles": sum(record["triangles"] for record in checks.values()),
            "indexed_bytes": sum(
                (stage / relative).stat().st_size for relative in assets
            ),
            "source_cache_triangles": source_manifest["triangles"],
            "retained_surface_error_m": max(
                (
                    record.get("surface_error_limit_m") or 0
                    for record in checks.values()
                ),
                default=0,
            ),
            "source_collisions": len(reference.findall("link/collision")),
            "collisions": len(root.findall("link/collision")),
        }
        (stage / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        stage.rename(output)
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("description_package", type=Path)
    parser.add_argument("source_cache", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "configs/robot/freecad_moveit_collision.json",
    )
    args = parser.parse_args()
    result = build_moveit_cache(
        args.description_package,
        args.source_cache,
        args.output,
        args.config,
        jobs=args.jobs,
    )
    print(
        json.dumps(
            {
                key: result[key]
                for key in (
                    "meshes",
                    "triangles",
                    "indexed_bytes",
                    "source_collisions",
                    "collisions",
                )
            }
        )
    )
