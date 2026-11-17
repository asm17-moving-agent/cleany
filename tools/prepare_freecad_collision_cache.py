"""Build an indexed or surface-bounded FreeCAD collision inspection cache."""

import argparse
import json
import tempfile
from concurrent.futures import ProcessPoolExecutor
from hashlib import sha256
from pathlib import Path
from xml.etree import ElementTree as ET

import trimesh
from optimize_arm_collision_meshes import read_stl


def write_indexed_glb(
    source: Path, target: Path, max_surface_error_m: float | None
) -> dict:
    """Write float32 triangles, optionally reducing them after a surface check."""
    mesh = read_stl(source)
    report = {"source_triangles": len(mesh.faces), "triangles": len(mesh.faces)}
    if max_surface_error_m is not None:
        from freecad_mesh_reduction import reduce_mesh

        mesh, report = reduce_mesh(mesh, max_surface_error_m)
    trimesh.Trimesh(vertices=mesh.vertices, faces=mesh.faces, process=False).export(
        target
    )
    return report


def build_cache(
    package: Path,
    output: Path,
    *,
    max_surface_error_m: float | None = None,
    jobs: int = 1,
    preserve_links: tuple[str, ...] = (),
) -> dict:
    """Prepare both CAD URDFs and share their indexed collision assets."""
    package, output = package.resolve(), output.resolve()
    if jobs < 1:
        raise ValueError("jobs must be positive")
    if output.is_relative_to(package) or output.exists():
        raise ValueError("Use a new cache directory outside the description package")
    output.parent.mkdir(parents=True, exist_ok=True)
    geometry = (package / "config/geometry/freecad_final.json").read_bytes()
    source_hashes = {
        mesh["mesh"]: mesh["sha256"]
        for part in json.loads(geometry)["parts"]
        for mesh in part["meshes"]
    }
    static = ET.parse(package / "urdf/cleany_freecad_final.urdf").getroot()
    links = {link.get("name"): link for link in static.findall("link")}
    if set(preserve_links) - links.keys():
        raise ValueError("Unknown static CAD link in preserve_links")
    preserved = {
        mesh.get("filename")
        for name in preserve_links
        for mesh in links[name].findall("collision/geometry/mesh")
    }
    stats = {
        "format": "glb",
        "mode": "indexed" if max_surface_error_m is None else "surface_bounded",
        "max_surface_error_m": max_surface_error_m,
        "preserve_links": list(preserve_links),
        "source_geometry_sha256": sha256(geometry).hexdigest(),
        "models": {},
        "assets": {},
        "geometry_checks": {},
        "meshes": 0,
        "source_triangles": 0,
        "triangles": 0,
        "stl_bytes": 0,
        "indexed_bytes": 0,
    }
    converted = {}
    pending = []
    prefix = "package://cleany_description/"
    with (
        tempfile.TemporaryDirectory(dir=output.parent) as temporary,
        ProcessPoolExecutor(max_workers=jobs) as executor,
    ):
        stage = Path(temporary) / "cache"
        (stage / "meshes").mkdir(parents=True)
        for variant in ("freecad_final", "freecad_kinematic"):
            filename = f"cleany_{variant}.urdf"
            original = (package / "urdf" / filename).read_bytes()
            root = ET.fromstring(original)
            for mesh in root.findall("link/collision/geometry/mesh"):
                if max_surface_error_m is not None and [
                    float(value) for value in mesh.get("scale", "1 1 1").split()
                ] != [1.0, 1.0, 1.0]:
                    raise ValueError(
                        "Surface reduction requires source mesh coordinates in metres"
                    )
                uri = mesh.attrib["filename"]
                relative = Path(uri.removeprefix(prefix))
                if (
                    not uri.startswith(prefix)
                    or not relative.is_relative_to("meshes/freecad_final")
                    or ".." in relative.parts
                    or relative.suffix != ".stl"
                ):
                    raise ValueError(f"Unexpected FreeCAD collision mesh: {uri}")
                if uri not in converted:
                    # The index prefix also distinguishes meshes with the same stem.
                    target = (
                        Path("meshes") / f"{len(converted):03d}-{relative.stem}.glb"
                    )
                    source = package / relative
                    if sha256(source.read_bytes()).hexdigest() != source_hashes.get(
                        str(relative)
                    ):
                        raise ValueError(
                            f"Source mesh differs from its FreeCAD manifest: {source}"
                        )
                    pending.append(
                        (
                            target,
                            executor.submit(
                                write_indexed_glb,
                                source,
                                stage / target,
                                None if uri in preserved else max_surface_error_m,
                            ),
                        )
                    )
                    stats["stl_bytes"] += source.stat().st_size
                    converted[uri] = str(target)
                mesh.set("filename", converted[uri])
            indexed = ET.tostring(root, encoding="utf-8", xml_declaration=True)
            (stage / filename).write_bytes(indexed)
            stats["models"][filename] = {
                "source_sha256": sha256(original).hexdigest(),
                "indexed_sha256": sha256(indexed).hexdigest(),
            }
        for target, future in pending:
            check = future.result()
            stats["geometry_checks"][str(target)] = check
            stats["source_triangles"] += check["source_triangles"]
            stats["triangles"] += check["triangles"]
            stats["indexed_bytes"] += (stage / target).stat().st_size
            stats["assets"][str(target)] = sha256(
                (stage / target).read_bytes()
            ).hexdigest()
        stats["meshes"] = len(converted)
        (stage / "manifest.json").write_text(json.dumps(stats, indent=2) + "\n")
        stage.rename(output)
    return stats


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("description_package", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--max-surface-error-m",
        type=float,
        help="Opt in to bounded surface reduction for inspection (requires VTK)",
    )
    parser.add_argument(
        "--jobs", type=int, default=1, help="Independent mesh conversion workers"
    )
    parser.add_argument(
        "--preserve-link",
        action="append",
        default=[],
        help="Keep original triangles for this static CAD link in both model variants",
    )
    args = parser.parse_args()
    stats = build_cache(
        args.description_package,
        args.output,
        max_surface_error_m=args.max_surface_error_m,
        jobs=args.jobs,
        preserve_links=tuple(args.preserve_link),
    )
    print(
        json.dumps(
            {
                key: stats[key]
                for key in (
                    "mode",
                    "meshes",
                    "source_triangles",
                    "triangles",
                    "stl_bytes",
                    "indexed_bytes",
                )
            }
        )
    )
