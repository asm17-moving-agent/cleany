"""Select a FreeCAD URDF and reject caches built from a different model."""

import json
from hashlib import sha256
from pathlib import Path
from xml.etree import ElementTree as ET


def inspection_urdf(package: Path, variant: str, cache: Path | None = None) -> str:
    """Load the original or a verified collision cache of the same CAD model."""
    filename = f"cleany_{variant}.urdf"
    source = (package / "urdf" / filename).read_bytes()
    if cache is None:
        return source.decode()
    cache = cache.resolve()
    manifest = json.loads((cache / "manifest.json").read_text())
    if manifest.get("format") != "glb":
        raise ValueError("Generate a GLB collision cache for FreeCAD inspection")
    if manifest.get("mode", "indexed") not in (
        "indexed",
        "surface_bounded",
        "moveit_primitives",
    ):
        raise ValueError("Unknown collision cache reduction mode")
    geometry = (package / "config/geometry/freecad_final.json").read_bytes()
    if sha256(geometry).hexdigest() != manifest["source_geometry_sha256"]:
        raise ValueError("Collision cache was built from different FreeCAD meshes")
    if filename not in manifest["models"]:
        raise ValueError(f"Collision cache does not provide {variant}")
    record = manifest["models"][filename]
    if sha256(source).hexdigest() != record["source_sha256"]:
        raise ValueError(f"Collision cache was built from a different {filename}")
    indexed = (cache / filename).read_bytes()
    if sha256(indexed).hexdigest() != record["indexed_sha256"]:
        raise ValueError(f"Collision cache URDF was modified: {filename}")
    for relative, digest in manifest["assets"].items():
        path = (cache / relative).resolve()
        if not path.is_relative_to(cache / "meshes") or path.suffix != ".glb":
            raise ValueError(f"Invalid indexed collision mesh: {relative}")
        if sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f"Collision cache mesh was modified: {path}")
    root = ET.fromstring(indexed)
    for mesh in root.findall("link/collision/geometry/mesh"):
        relative = mesh.attrib["filename"]
        if relative not in manifest["assets"]:
            raise ValueError(f"Unregistered indexed collision mesh: {relative}")
        mesh.set("filename", (cache / relative).as_uri())
    return ET.tostring(root, encoding="unicode")
