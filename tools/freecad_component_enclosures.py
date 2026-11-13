"""Apply the existing partitioned enclosure algorithm to FreeCAD part meshes."""

from pathlib import Path

import numpy as np
import trimesh
from optimize_arm_collision_meshes import (
    Mesh,
    components,
    enclosing_hull,
    maximum_plane_violation,
    partition_surface,
    radial_bounds,
    read_stl,
)
from scipy.spatial import ConvexHull


def selected_parts(parts: dict[str, dict], settings: dict) -> dict[str, dict]:
    profile = settings.get("component_enclosures", {})
    groups = set(profile.get("groups", []))
    overrides = profile.get("parts", {})
    sets = profile.get("sets", [])
    members = [name for group in sets for name in group["parts"]]
    names = [group["name"] for group in sets]
    if (
        len(members) != len(set(members))
        or len(names) != len(set(names))
        or set(members) - parts.keys()
        or set(names) & parts.keys()
        or any(not group["parts"] for group in sets)
        or any(not name.replace("_", "").isalnum() for name in names)
    ):
        raise ValueError("Invalid rigid enclosure sets")
    if (
        groups - {group for part in parts.values() for group in part["groups"]}
        or overrides.keys() - parts.keys()
    ):
        raise ValueError("Unknown enclosure group or part")
    return {
        name: overrides.get(name, {})
        for name, part in parts.items()
        if groups.intersection(part["groups"]) or name in overrides or name in members
    }


def write_enclosures(
    source: Path | list[Path], directory: Path, settings: dict, spec: dict
) -> list[dict]:
    """Write each convex piece separately, certifying the serialized GLB surface."""
    sources = [source] if isinstance(source, Path) else source
    originals = [read_stl(path) for path in sources]
    offsets = np.cumsum([0, *[len(mesh.vertices) for mesh in originals[:-1]]])
    mesh = Mesh(
        np.vstack([m.vertices for m in originals]),
        np.vstack([m.faces + offset for m, offset in zip(originals, offsets)]),
    )
    planes = list(spec.get("planes", []))
    if "slabs" in spec:
        axis, count = spec["slabs"]["axis"], spec["slabs"]["count"]
        if axis not in (0, 1, 2) or type(count) is not int or count < 2:
            raise ValueError("Invalid enclosure slabs")
        lo, hi = mesh.vertices[:, axis].min(), mesh.vertices[:, axis].max()
        planes.extend(
            {"axis": axis, "offset_m": float(offset)}
            for offset in np.linspace(lo, hi, count + 1)[1:-1]
        )
    limit = spec.get("max_hull_expansion_m", settings["max_hull_expansion_m"])
    combined = spec.get("combine_components", False)
    if type(combined) is not bool or (combined and planes):
        raise ValueError("Component merging requires an unpartitioned rigid part")
    source_components = components(mesh)
    pieces = [
        (index, piece)
        for index, component in enumerate([mesh] if combined else source_components)
        for piece in partition_surface(component, planes)
    ]
    directory.mkdir(parents=True)
    reports = []
    for index, (component, piece) in enumerate(pieces):
        result, report = enclosing_hull(
            piece,
            max_expansion=limit,
            margin=settings["enclosure_margin_m"],
            max_iterations=settings["max_iterations"],
            planes_per_iteration=settings["planes_per_iteration"],
        )
        filename = f"piece-{index:02d}.glb"
        trimesh.Trimesh(result.vertices, result.faces, process=False).export(
            directory / filename
        )
        restored = trimesh.load(directory / filename, force="mesh", process=False)
        hull = ConvexHull(restored.vertices)
        violation = maximum_plane_violation(piece.vertices, hull.equations)
        bound = float(
            radial_bounds(
                restored.vertices, ConvexHull(piece.vertices), piece.vertices.mean(0)
            )[0].max()
        )
        if (
            violation > 0
            or bound > limit
            or not restored.is_watertight
            or not restored.is_winding_consistent
        ):
            raise ValueError("Serialized enclosure failed its solid/containment check")
        # Each actual triangle must support the convex solid, not cut through it.
        triangles = restored.triangles
        normals = restored.face_normals
        for normal, point in zip(normals, triangles[:, 0]):
            if np.max((restored.vertices - point) @ normal) > 1e-9:
                raise ValueError("An enclosure face cuts through its convex hull")
        reports.append(
            {
                **report,
                "file": filename,
                "source_component": component,
                "combined_components": combined,
                "source_component_count": len(source_components),
                "source_planes": planes,
                "maximum_source_plane_violation_m": violation,
                "max_expansion_from_component_hull_m": bound,
                "expansion_limit_m": limit,
                "interior_point_base_m": restored.vertices.mean(0).tolist(),
            }
        )
    return reports
