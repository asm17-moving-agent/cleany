"""Extract and solve nominal pin-joint loops in the supplied PincOpen CAD."""

from __future__ import annotations

from typing import Any

import numpy as np
from export_freecad_model import transformed


def cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def rotation2(angle: float) -> np.ndarray:
    return np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])


def angle_between(reference: np.ndarray, current: np.ndarray) -> float:
    return float(np.arctan2(cross2(reference, current), reference @ current))


def circle_point(
    a: np.ndarray,
    b: np.ndarray,
    radius_a: float,
    radius_b: float,
    branch: int,
    epsilon: float,
) -> np.ndarray:
    """Solve a pin closure on the branch present in the saved assembly.

    Tangency is rejected because the CAD alone does not select a continuation
    through a toggle. These checks are mathematical domains, not motion limits.
    """
    direction = b - a
    distance = np.linalg.norm(direction)
    if distance <= epsilon:
        raise ValueError("Coincident loop centers do not determine a pin position")
    along = (radius_a**2 - radius_b**2 + distance**2) / (2 * distance)
    height_squared = radius_a**2 - along**2
    if height_squared <= epsilon**2:
        raise ValueError("Pin loop is infeasible or at an unresolved toggle")
    unit = direction / distance
    return (
        a
        + along * unit
        + branch * np.sqrt(height_squared) * np.array([-unit[1], unit[0]])
    )


def unique_pin(
    axes: dict, parts: list[str], tolerance_mm: float, tolerance_rad: float
) -> dict:
    """Require exactly one common axis among the named parts."""
    candidates = []
    for axis in axes[parts[0]]:
        matches = {parts[0]: axis}
        for name in parts[1:]:
            found = []
            for other in axes[name]:
                a, b = np.array(axis["direction"]), np.array(other["direction"])
                angle = np.arccos(np.clip(abs(a @ b), 0, 1))
                distance = np.linalg.norm(
                    np.cross(np.array(other["point"]) - axis["point"], a)
                )
                if distance <= tolerance_mm and angle <= tolerance_rad:
                    found.append(other)
            if len(found) != 1:
                break
            matches[name] = found[0]
        else:
            candidates.append((axis, matches))
    if len(candidates) != 1:
        raise ValueError(f"Expected one shared pin for {parts}, got {len(candidates)}")
    axis, matches = candidates[0]
    return {
        "point_cad_mm": axis["point"],
        "direction_cad": axis["direction"],
        "parts": parts,
        "faces": {name: match["faces"] for name, match in matches.items()},
    }


def extract_linkages(axes: dict, static: dict, recipe: dict, kinematics: dict) -> None:
    """Append cam/rocker links supported by the CAD pin graph."""
    tolerances = [
        recipe["numerical_tolerance_m"],
        recipe["geometry_tolerance_m"],
        recipe["plane_parallel_tolerance_rad"],
    ]
    if not np.isfinite(tolerances).all() or min(tolerances) <= 0:
        raise ValueError("Gripper geometry tolerances must be finite and positive")
    matrix = np.asarray(static["coordinates"]["cad_to_base_link"])
    rotation = matrix[:3, :3] / 0.001
    origins = {
        "base_link": np.zeros(3),
        **{s["link"]: np.array(s["point_base_m"]) for s in kinematics["links"]},
    }
    match_mm = kinematics["limits"]["axis_match_distance_mm"]
    match_angle = kinematics["limits"]["axis_match_angle_rad"]
    kinematics["independent_joint_names"] = [s["joint"] for s in kinematics["links"]]
    linkages = []

    for hand in recipe["hands"]:
        side, prefix, parent = hand["hand"], hand["prefix"], hand["parent"]

        def pin(*names, prefix=prefix):
            return unique_pin(axes, [prefix + n for n in names], match_mm, match_angle)

        center = pin("Cam", "servo", "CamHorn")
        normal = rotation @ np.array(center["direction_cad"])
        normal /= np.linalg.norm(normal)
        if normal[np.argmax(np.abs(normal))] < 0:
            normal = -normal
        origin = transformed(np.array([center["point_cad_mm"]]), matrix)[0]
        u = np.array([1.0, 0.0, 0.0]) - normal * normal[0]
        u /= np.linalg.norm(u)
        basis = np.array([u, np.cross(normal, u)])
        linkage: dict[str, Any] = {
            "hand": side,
            "parent": parent,
            "origin_base_m": origin.tolist(),
            "plane_basis": basis.tolist(),
            "normal_base": normal.tolist(),
            "cam_joint": f"cad_delta_{side}_gripper_cam",
            "cam_center_evidence": center,
            "numerical_tolerance_m": recipe["numerical_tolerance_m"],
            "fingers": {},
            "physical_zero_rad": None,
            "motor_id": None,
            "encoder_direction": None,
            "operating_limits_rad": None,
            "calibrated_aperture_curve": None,
            "contact_and_compliance": None,
            "mechanism": "eccentric_pin_crank_and_two_closed_pin_loops",
            "tip_attachment_constraint": None,
            "tip_visual_payloads": {
                finger: [prefix + p for p in members["distal"][1:]]
                for finger, members in hand["fingers"].items()
            },
        }
        expected = set(hand["cam_parts"] + hand["fixed_parts"])
        expected.update(
            part
            for members in hand["fingers"].values()
            for parts in members.values()
            for part in parts
        )
        actual = {
            part["name"].removeprefix(prefix)
            for part in static["parts"]
            if part["name"].startswith(prefix)
        }
        if expected != actual:
            raise ValueError("Gripper recipe does not account for every source part")
        if any(
            kinematics["part_owners"][prefix + p] != parent for p in hand["fixed_parts"]
        ):
            raise ValueError(
                "Fixed gripper body parts do not belong to the wrist carrier"
            )

        def point(
            record: dict, normal=normal, basis=basis, origin=origin
        ) -> np.ndarray:
            direction = rotation @ record["direction_cad"]
            if (
                np.arccos(np.clip(abs(direction @ normal), 0, 1))
                > recipe["plane_parallel_tolerance_rad"]
            ):
                raise ValueError(
                    "Gripper pin axes do not define the same planar linkage"
                )
            return basis @ (
                transformed(np.array([record["point_cad_mm"]]), matrix)[0] - origin
            )

        def add_link(
            role: str,
            anchor: np.ndarray,
            owner: str,
            parts: list[str],
            kind: str,
            side=side,
            prefix=prefix,
            origin=origin,
            basis=basis,
            normal=normal,
        ) -> tuple[str, str]:
            name, joint = (
                f"cad_{side}_gripper_{role}",
                f"cad_delta_{side}_gripper_{role}",
            )
            xyz = origin + basis.T @ anchor
            kinematics["links"].append(
                {
                    "link": name,
                    "parent": owner,
                    "joint": joint,
                    "parts": [prefix + p for p in parts],
                    "point_base_m": xyz.tolist(),
                    "origin_parent_m": (xyz - origins[owner]).tolist(),
                    "axis_base": normal.tolist(),
                    "source_kind": "gripper_pin_loop",
                    "coordinate_kind": kind,
                    "encoder_zero_tick": None,
                    "physical_zero_rad": None,
                    "encoder_direction": None,
                    "operational_limits_rad": None,
                    "velocity_limit_rad_s": None,
                    "effort_limit": None,
                }
            )
            origins[name] = xyz
            for part in parts:
                if prefix + part not in kinematics["part_owners"]:
                    raise ValueError(f"Missing source part {prefix + part}")
                kinematics["part_owners"][prefix + part] = name
            return name, joint

        cam_link, cam_joint = add_link(
            "cam", np.zeros(2), parent, hand["cam_parts"], "independent_cam_delta"
        )
        kinematics["independent_joint_names"].append(cam_joint)
        # Four separate coaxial fastener locations lock the cam to the servo horn.
        cam_horn_axes = []
        for a in axes[prefix + "Cam"]:
            for b in axes[prefix + "CamHorn"]:
                if np.linalg.norm(
                    np.cross(np.array(a["point"]) - b["point"], b["direction"])
                ) <= match_mm and abs(
                    np.array(a["direction"]) @ b["direction"]
                ) > np.cos(match_angle):
                    cam_horn_axes.append(a)
        center_line = np.asarray(center["point_cad_mm"])
        fasteners = [
            a
            for a in cam_horn_axes
            if np.linalg.norm(
                np.cross(np.array(a["point"]) - center_line, center["direction_cad"])
            )
            > match_mm
        ]
        if len(fasteners) != 4:
            raise ValueError(
                "Cam/horn rigid mounting does not have the expected four shared bores"
            )
        linkage["cam_horn_fasteners"] = fasteners

        for finger, members in hand["fingers"].items():
            suffix = "_" + finger
            evidence = {
                "P": pin("Cam", "Driving_Rod" + suffix),
                "D": pin("Driving_Rod" + suffix, "Internal_Rod" + suffix),
                "A": pin("Internal_Rod" + suffix, "Bottom_Plate", "Top_Plate"),
                "B": pin("External_Rod" + suffix, "Bottom_Plate", "Top_Plate"),
                "C": pin("Internal_Rod" + suffix, "Distal_Rod" + suffix),
                "E": pin("External_Rod" + suffix, "Distal_Rod" + suffix),
            }
            points = {key: point(value) for key, value in evidence.items()}
            a, b, c, d, e, p = (points[key] for key in "ABCDE P".replace(" ", ""))
            if np.linalg.norm((c - a) - (e - b)) > recipe["geometry_tolerance_m"]:
                raise ValueError(
                    "Inner/outer rocker spans do not match the CAD parallelogram"
                )
            if np.linalg.norm((b - a) - (e - c)) > recipe["geometry_tolerance_m"]:
                raise ValueError("Base and distal pin spacing do not match")
            branch_drive = int(np.sign(cross2(p - a, d - a)))
            branch_distal = int(np.sign(cross2(b - c, e - c)))
            if 0 in (branch_drive, branch_distal):
                raise ValueError("Saved gripper pose is singular")
            internal_link, internal_joint = add_link(
                "internal" + suffix,
                a,
                parent,
                members["internal"],
                "derived_loop_delta",
            )
            _, external_joint = add_link(
                "external" + suffix,
                b,
                parent,
                members["external"],
                "derived_loop_delta",
            )
            _, driving_joint = add_link(
                "driving" + suffix,
                p,
                cam_link,
                members["driving"],
                "derived_loop_delta",
            )
            _, distal_joint = add_link(
                "distal" + suffix,
                c,
                internal_link,
                members["distal"],
                "derived_loop_delta",
            )
            linkage["fingers"][finger] = {
                "points_m": {key: value.tolist() for key, value in points.items()},
                "evidence": evidence,
                "branch_drive": branch_drive,
                "branch_distal": branch_distal,
                "joints": {
                    "internal": internal_joint,
                    "external": external_joint,
                    "driving": driving_joint,
                    "distal": distal_joint,
                },
                "lengths_m": {
                    "AD": float(np.linalg.norm(d - a)),
                    "PD": float(np.linalg.norm(d - p)),
                    "AC": float(np.linalg.norm(c - a)),
                    "BE": float(np.linalg.norm(e - b)),
                    "CE": float(np.linalg.norm(e - c)),
                    "AB": float(np.linalg.norm(b - a)),
                },
            }
        left = np.asarray(linkage["fingers"]["L"]["points_m"]["P"])
        right = np.asarray(linkage["fingers"]["R"]["points_m"]["P"])
        linkage["cam_pin_angle_separation_rad"] = float(
            np.arccos(
                np.clip(
                    left @ right / (np.linalg.norm(left) * np.linalg.norm(right)), -1, 1
                )
            )
        )
        linkages.append(linkage)
    kinematics["gripper_linkages"] = linkages
    kinematics["unarticulated_drives"] = []
    kinematics["capabilities"]["gripper_linkage_preview"] = True


def solve_linkage(linkage: dict, cam_delta: float) -> tuple[dict[str, float], dict]:
    """Return dependent joint deltas and pin positions on the CAD assembly branch."""
    if not np.isfinite(cam_delta):
        raise ValueError("Cam displacement must be finite")
    values, states = {}, {}
    epsilon = linkage["numerical_tolerance_m"]
    for finger, spec in linkage["fingers"].items():
        points = {key: np.asarray(value) for key, value in spec["points_m"].items()}
        a, b, c0, d0, e0, p0 = (points[key] for key in "ABCDE P".replace(" ", ""))
        length = spec["lengths_m"]
        p = rotation2(cam_delta) @ p0
        d = circle_point(
            a, p, length["AD"], length["PD"], spec["branch_drive"], epsilon
        )
        internal = angle_between(d0 - a, d - a)
        c = a + rotation2(internal) @ (c0 - a)
        e = circle_point(
            c, b, length["CE"], length["BE"], spec["branch_distal"], epsilon
        )
        external = angle_between(e0 - b, e - b)
        distal = angle_between(e0 - c0, e - c)
        driving = angle_between(d0 - p0, d - p)
        values.update(
            {
                spec["joints"]["internal"]: internal,
                spec["joints"]["external"]: external,
                spec["joints"]["distal"]: distal - internal,
                spec["joints"]["driving"]: driving - cam_delta,
            }
        )
        states[finger] = {
            key: value.tolist()
            for key, value in {"A": a, "B": b, "C": c, "D": d, "E": e, "P": p}.items()
        }
    return values, states


def resolved_deltas(kinematics: dict, inputs: dict[str, float]) -> dict[str, float]:
    independent = kinematics.get(
        "independent_joint_names", [s["joint"] for s in kinematics["links"]]
    )
    if set(inputs) - set(independent) or not all(
        np.isfinite(v) for v in inputs.values()
    ):
        raise ValueError(
            "Only finite independent CAD-delta coordinates may be supplied"
        )
    values = dict.fromkeys(independent, 0.0)
    values.update(inputs)
    for linkage in kinematics.get("gripper_linkages", []):
        dependent, _ = solve_linkage(linkage, inputs.get(linkage["cam_joint"], 0.0))
        values.update(dependent)
    return values
