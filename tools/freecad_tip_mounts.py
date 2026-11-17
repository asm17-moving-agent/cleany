"""Inspect fastening bores and rigid-body rotation clearance in the tip seats."""

from __future__ import annotations

import io
from itertools import combinations
from typing import Any
from zipfile import ZipFile

import numpy as np
from export_freecad_model import transformed
from freecad_gripper_linkage import unique_pin


def _solid(archive: ZipFile, name: str, parent: np.ndarray):
    from OCP.BRep import BRep_Builder
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.BRepTools import BRepTools
    from OCP.gp import gp_Trsf
    from OCP.TopoDS import TopoDS_Shape

    shape = TopoDS_Shape()
    BRepTools.Read_s(
        shape, io.BytesIO(archive.read(name + ".Shape.brp")), BRep_Builder()
    )
    if not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError(f"Invalid BREP cannot establish a tip seat: {name}")
    placement = gp_Trsf()
    placement.SetValues(*parent[:3].ravel())
    return BRepBuilderAPI_Transform(shape, placement, True).Shape()


def _volume(shape) -> float:
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, properties)
    return float(abs(properties.Mass()))


def _intersection(first, second) -> float:
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common

    common = BRepAlgoAPI_Common(first, second)
    if not common.IsDone():
        raise ValueError("Tip intersection computation did not finish")
    return _volume(common.Shape())


def _distance(first, second) -> float:
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape

    distance = BRepExtrema_DistShapeShape(first, second)
    distance.Perform()
    if not distance.IsDone():
        raise ValueError("Tip clearance computation did not finish")
    return float(distance.Value())


def _rotated(shape, point: list[float], direction: list[float], degrees: float):
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf

    transform = gp_Trsf()
    transform.SetRotation(
        gp_Ax1(gp_Pnt(*point), gp_Dir(*direction)), np.deg2rad(degrees)
    )
    return BRepBuilderAPI_Transform(shape, transform, True).Shape()


def blocked_rotation(probes: list[dict], tolerance: float) -> bool:
    """Require interference on both signs; clearance alone never proves a hinge."""
    return bool(
        probes
        and any(p["angle_deg"] < 0 for p in probes)
        and any(p["angle_deg"] > 0 for p in probes)
        and all(p["carrier_overlap_mm3"]["insert"] > tolerance for p in probes)
    )


def analyze_tip_mounts(
    archive: ZipFile, axes: dict, parents: dict, gripper_recipe: dict, tip_recipe: dict
) -> list[dict[str, Any]]:
    """Measure the supplied seats; only classify the tested rotational mode.

    Shared bores alone do not establish a hinge or screw preload. A collision
    on both rotation signs rejects free rigid rotation about the bore axis.
    """
    probe_angles = tip_recipe["probe_rotation_deg"]
    tolerance = tip_recipe["baseline_overlap_tolerance_mm3"]
    if (
        not np.isfinite([*probe_angles, tolerance]).all()
        or tolerance <= 0
        or not any(a < 0 for a in probe_angles)
        or not any(a > 0 for a in probe_angles)
    ):
        raise ValueError(
            "Tip seat probes need finite angles of both signs and a positive tolerance"
        )
    result = []
    for hand in gripper_recipe["hands"]:
        for finger in hand["fingers"]:
            names = {
                role: hand["prefix"] + part + "_" + finger
                for role, part in tip_recipe["parts"].items()
            }
            pin = unique_pin(
                axes,
                [names[r] for r in ["carrier", "shell", "insert"]],
                tip_recipe["axis_match_distance_mm"],
                tip_recipe["axis_match_angle_rad"],
            )
            shapes = {
                role: _solid(archive, name, parents[name])
                for role, name in names.items()
            }
            if any(_volume(s) <= tolerance for s in shapes.values()):
                raise ValueError("Tip analysis requires nonempty solid geometry")
            baseline = {
                f"{a}:{b}": {
                    "clearance_mm": _distance(shapes[a], shapes[b]),
                    "overlap_mm3": _intersection(shapes[a], shapes[b]),
                }
                for a, b in combinations(shapes, 2)
            }
            if any(p["overlap_mm3"] > tolerance for p in baseline.values()):
                raise ValueError(
                    "Source tip solids already overlap; seat classification is ambiguous"
                )
            probes = []
            for angle in probe_angles:
                rotated = {
                    role: _rotated(
                        shapes[role], pin["point_cad_mm"], pin["direction_cad"], angle
                    )
                    for role in ["shell", "insert", "pad"]
                }
                probes.append(
                    {
                        "angle_deg": angle,
                        "carrier_overlap_mm3": {
                            role: _intersection(shapes["carrier"], shape)
                            for role, shape in rotated.items()
                        },
                        "shell_to_insert_overlap_mm3": _intersection(
                            rotated["shell"], shapes["insert"]
                        ),
                        "pad_to_insert_overlap_mm3": _intersection(
                            rotated["pad"], shapes["insert"]
                        ),
                    }
                )
            blocked = blocked_rotation(probes, tolerance)
            result.append(
                {
                    "hand": hand["hand"],
                    "finger": finger,
                    "parts": names,
                    "shared_fastening_axis": pin,
                    "bore_diameters_mm": {
                        role: sorted(
                            {
                                round(2 * f["radius_mm"], 8)
                                for f in pin["faces"][names[role]]
                            }
                        )
                        for role in ["carrier", "shell", "insert"]
                    },
                    "baseline": baseline,
                    "rotation_probes": probes,
                    "classification": "nominal_fixed_keyed_seat"
                    if blocked
                    else "unresolved",
                    "free_rigid_rotation_about_fastening_axis": False
                    if blocked
                    else None,
                    "screw_specification": None,
                    "clamp_preload": None,
                    "pad_material_and_retention": None,
                    "compliance_and_backlash": None,
                    "physical_load_retention": None,
                }
            )
    return result


def apply_tip_mounts(
    kinematics: dict, evidence: list[dict], matrix: np.ndarray
) -> None:
    """Separate confirmed nominal seats into fixed children of their carriers."""
    kinematics["fixed_attachments"] = []
    by_joint = {spec["joint"]: spec for spec in kinematics["links"]}
    for mount in evidence:
        hand, finger = mount["hand"], mount["finger"]
        linkage = next(g for g in kinematics["gripper_linkages"] if g["hand"] == hand)
        linkage.setdefault("tip_mounts", []).append(mount)
        if mount["classification"] != "nominal_fixed_keyed_seat":
            continue
        parent_spec = by_joint[linkage["fingers"][finger]["joints"]["distal"]]
        origin = transformed(
            np.array([mount["shared_fastening_axis"]["point_cad_mm"]]), matrix
        )[0]
        name = f"cad_{hand}_gripper_tip_{finger}"
        parts = [mount["parts"][role] for role in ["shell", "insert", "pad"]]
        kinematics["fixed_attachments"].append(
            {
                "link": name,
                "parent": parent_spec["link"],
                "joint": f"cad_fixed_{hand}_gripper_tip_{finger}",
                "parts": parts,
                "point_base_m": origin.tolist(),
                "origin_parent_m": (origin - parent_spec["point_base_m"]).tolist(),
                "joint_type": "fixed",
                "connection": "CAD_nominal_fixed_keyed_seat",
                "physical_preload_and_compliance": None,
            }
        )
        for part in parts:
            kinematics["part_owners"][part] = name
    for linkage in kinematics["gripper_linkages"]:
        mounts = linkage.get("tip_mounts", [])
        if len(mounts) == 2 and all(
            m["classification"] == "nominal_fixed_keyed_seat" for m in mounts
        ):
            linkage["tip_attachment_constraint"] = "CAD_nominal_fixed_keyed_seat"
            linkage.pop("tip_visual_payloads", None)
