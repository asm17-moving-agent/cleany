"""Attach CAD camera datums while preserving collision geometry and calibration state."""

from __future__ import annotations

import json
import math
from hashlib import sha256
from pathlib import Path
from xml.etree import ElementTree as ET


PROFILE_PATH = "config/geometry/freecad_camera_frames.json"
SOURCE_FILES = {
    "config/geometry/freecad_final.json", "config/geometry/freecad_kinematics.json",
    "config/geometry/freecad_wrist_camera_depth.json", "urdf/cleany_freecad_final.urdf",
    "urdf/cleany_freecad_kinematic.urdf",
}


def load_camera_frames(package: Path) -> dict | None:
    """Reject stale nominal datums; absence preserves older geometry-only packages."""
    path = package / PROFILE_PATH
    if not path.exists():
        return None
    profile = json.loads(path.read_text())
    if profile.get("schema") != "cleany.cad_camera_frames/v1" or profile.get("calibrated") is not False:
        raise ValueError("Expected an explicitly uncalibrated CAD camera profile")
    if profile.get("units") != "metres_and_radians":
        raise ValueError("CAD camera transforms require metres and radians")
    hashes = profile["source_files_sha256"]
    if set(hashes) != SOURCE_FILES:
        raise ValueError("Incomplete CAD camera provenance")
    for relative, digest in hashes.items():
        if sha256((package / relative).read_bytes()).hexdigest() != digest:
            raise ValueError(f"Stale CAD camera frame: {relative}")
    geometry = json.loads((package / "config/geometry/freecad_final.json").read_text())
    if profile.get("source_fcstd_sha256") != geometry["source"]["sha256"]:
        raise ValueError("Camera datum source differs from the model source")
    frames = profile["frames"]
    expected = {f"cad_{camera}_camera_optical_frame" for camera in ("left_wrist", "right_wrist", "head")}
    if len(frames) != 3 or {f["frame"] for f in frames} != expected:
        raise ValueError("Expected both wrist cameras and one nominal head camera")
    for frame in frames:
        if frame.get("status") != "cad_nominal_unverified" or any(
            value is not None for value in frame["unresolved"].values()
        ):
            raise ValueError("CAD datums cannot assert a measured optical calibration")
    return profile


def _triple(values: list[float]) -> str:
    if len(values) != 3 or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                                  and math.isfinite(v) for v in values):
        raise ValueError("Expected three finite camera transform coordinates")
    return " ".join(format(v, ".15g") for v in values)


def with_camera_frames(urdf: str, package: Path, variant: str) -> str:
    """Append fixed, geometry-free optical datums in metres and URDF xyz Euler radians.

    The head datum is a window centroid, not a per-stream RGB/depth pinhole.
    These frames never authorize physical feedback, perception or actuation.
    """
    profile = load_camera_frames(package)
    if profile is None:
        return urdf
    if variant not in ("freecad_final", "freecad_kinematic"):
        raise ValueError("Camera datums only belong to the matching FreeCAD models")
    root = ET.fromstring(urdf)
    links = {link.get("name"): link for link in root.findall("link")}
    for frame in profile["frames"]:
        pose = frame["poses"][variant]
        parent, child = pose["parent"], frame["frame"]
        if parent not in links:
            raise ValueError(f"Missing CAD camera attachment: {parent}")
        origin = {"xyz": _triple(pose["xyz_m"]), "rpy": _triple(pose["rpy_rad"])}
        existing = root.find(f"joint[@name='{frame['joint']}']")
        if child in links or existing is not None:
            incoming = [j for j in root.findall("joint") if j.find("child").get("link") == child]
            if (child not in links or len(links[child]) or existing is None or incoming != [existing]
                    or existing.get("type") != "fixed" or existing.find("parent").get("link") != parent
                    or existing.find("origin").attrib != origin):
                raise ValueError(f"Conflicting existing CAD camera frame: {child}")
            continue
        links[child] = ET.SubElement(root, "link", name=child)
        joint = ET.SubElement(root, "joint", name=frame["joint"], type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", **origin)
    return ET.tostring(root, encoding="unicode")
