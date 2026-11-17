"""Validate pin-loop closure without assuming a calibrated gripper actuator."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE.parents[2] / "tools"))
from extract_freecad_kinematics import forward_kinematics
from freecad_gripper_linkage import circle_point, resolved_deltas, solve_linkage

MODEL = json.loads((PACKAGE / "config/geometry/freecad_kinematics.json").read_text())


@pytest.mark.parametrize("index", [0, 1])
@pytest.mark.parametrize("angle", [-0.6, -0.3, 0.0, 0.15])
def test_closed_loops_preserve_all_measured_pin_distances(index, angle) -> None:
    linkage = MODEL["gripper_linkages"][index]
    _, fingers = solve_linkage(linkage, angle)
    for name, state in fingers.items():
        p = {key: np.array(value) for key, value in state.items()}
        expected = linkage["fingers"][name]["lengths_m"]
        for segment in ("AD", "PD", "AC", "BE", "CE", "AB"):
            assert np.linalg.norm(p[segment[0]] - p[segment[1]]) == pytest.approx(
                expected[segment], abs=1e-12
            )
        assert p["E"] - p["C"] == pytest.approx(p["B"] - p["A"], abs=1e-12)


def test_measured_parallelogram_and_cam_asymmetry_are_preserved() -> None:
    for linkage in MODEL["gripper_linkages"]:
        assert len(linkage["cam_horn_fasteners"]) == 4
        assert abs(np.pi - linkage["cam_pin_angle_separation_rad"]) > 0.02
        for finger in linkage["fingers"].values():
            length = finger["lengths_m"]
            assert length["AC"] == pytest.approx(0.040, abs=1e-9)
            assert length["BE"] == pytest.approx(0.040, abs=1e-9)
            assert length["AB"] == pytest.approx(0.0115, abs=1e-9)
            assert length["CE"] == pytest.approx(0.0115, abs=1e-9)
            assert length["PD"] == pytest.approx(0.03264768, abs=1e-8)


def test_fk_keeps_both_ends_of_each_moving_pin_together() -> None:
    inputs = {
        "cad_delta_right_gripper_cam": -0.4,
        "cad_delta_left_gripper_cam": -0.3,
        "cad_delta_right_shoulder_yaw": 0.13,
        "cad_delta_left_wrist_roll": 0.21,
    }
    fk = forward_kinematics(MODEL, inputs)
    specs = {spec["joint"]: spec for spec in MODEL["links"]}
    for linkage in MODEL["gripper_linkages"]:
        basis, origin = (
            np.array(linkage["plane_basis"]),
            np.array(linkage["origin_base_m"]),
        )
        for finger in linkage["fingers"].values():
            points = {
                name: origin + basis.T @ point
                for name, point in finger["points_m"].items()
            }

            def moved(role, point, joints=finger["joints"]):
                spec = specs[joints[role]]
                transform = fk[spec["link"]]
                return (
                    transform[:3, :3] @ (point - spec["point_base_m"])
                    + transform[:3, 3]
                )

            assert moved("driving", points["D"]) == pytest.approx(
                moved("internal", points["D"]), abs=1e-11
            )
            assert moved("distal", points["C"]) == pytest.approx(
                moved("internal", points["C"]), abs=1e-11
            )
            assert moved("distal", points["E"]) == pytest.approx(
                moved("external", points["E"]), abs=1e-11
            )


def test_unknown_calibration_and_tip_contact_constraints_stay_unknown() -> None:
    for linkage in MODEL["gripper_linkages"]:
        for key in [
            "physical_zero_rad",
            "encoder_direction",
            "operating_limits_rad",
            "calibrated_aperture_curve",
            "contact_and_compliance",
        ]:
            assert linkage[key] is None
        assert linkage["tip_attachment_constraint"] == "CAD_nominal_fixed_keyed_seat"
    assert MODEL["capabilities"]["gripper_linkage_preview"]
    assert not MODEL["capabilities"]["gripper_actuation"]
    with pytest.raises(ValueError, match="independent"):
        resolved_deltas(MODEL, {"cad_delta_right_gripper_internal_L": 0.1})


def test_loop_solver_rejects_impossible_and_ambiguous_toggles() -> None:
    with pytest.raises(ValueError, match="infeasible"):
        circle_point(np.zeros(2), np.array([3.0, 0.0]), 1, 1, 1, 1e-9)
    with pytest.raises(ValueError, match="toggle"):
        circle_point(np.zeros(2), np.array([2.0, 0.0]), 1, 1, 1, 1e-9)
    positive = circle_point(np.zeros(2), np.array([1.0, 0.0]), 1, 1, 1, 1e-9)
    negative = circle_point(np.zeros(2), np.array([1.0, 0.0]), 1, 1, -1, 1e-9)
    assert positive == pytest.approx(negative * [1, -1])
