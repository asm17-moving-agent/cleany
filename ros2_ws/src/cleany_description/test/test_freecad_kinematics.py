"""Check geometry-relative motion while leaving robot calibration unresolved."""

from __future__ import annotations

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from extract_freecad_kinematics import (
    axis_difference,
    canonical_direction,
    forward_kinematics,
    matched_axis,
)
from freecad_gripper_linkage import resolved_deltas

STATIC = json.loads((PACKAGE / "config/geometry/freecad_final.json").read_text())
KINEMATICS = json.loads(
    (PACKAGE / "config/geometry/freecad_kinematics.json").read_text()
)


def test_axes_have_independent_servo_and_horn_evidence() -> None:
    assert len(KINEMATICS["links"]) == 34
    assert len(KINEMATICS["independent_joint_names"]) == 18
    assert len(KINEMATICS["gripper_linkages"]) == 2
    assert not KINEMATICS["unarticulated_drives"]
    assert KINEMATICS["source_sha256"] == STATIC["source"]["sha256"]
    for spec in [
        s for s in KINEMATICS["links"] if s.get("source_kind") != "gripper_pin_loop"
    ]:
        assert spec["servo"] != spec["drive_horn"]
        assert {spec["servo"], spec["drive_horn"], *spec["supports"]} == set(
            spec["evidence"]
        )
        assert np.linalg.norm(spec["axis_base"]) == pytest.approx(1)
        for evidence in spec["evidence"].values():
            assert evidence["axis_distance_mm"] <= 0.02
            assert evidence["axis_angle_rad"] <= 0.0001
            assert evidence["faces"]
        assert spec["physical_zero_rad"] is None
    for spec in KINEMATICS["links"]:
        assert all(
            spec[key] is None
            for key in [
                "encoder_zero_tick",
                "encoder_direction",
                "operational_limits_rad",
                "velocity_limit_rad_s",
                "effort_limit",
            ]
        )


def test_every_part_has_one_owner_and_gripper_has_no_actuator() -> None:
    owners = KINEMATICS["part_owners"]
    assert set(owners) == {p["name"] for p in STATIC["parts"]}
    for spec in KINEMATICS["links"]:
        if spec.get("source_kind") == "gripper_pin_loop":
            continue
        assert owners[spec["servo"]] == spec["parent"]
        assert owners[spec["drive_horn"]] == spec["link"]
    for side, prefix in [("right", "Right"), ("left", "Left")]:
        parts = [
            p for p in owners if p.startswith(prefix + "Kinematic_CameraTool180_Pinc_")
        ]
        assert len(parts) == 22
        assert (
            owners[prefix + "Kinematic_CameraTool180_Pinc_servo"]
            == f"cad_{side}_wrist_roll"
        )
        assert (
            owners[prefix + "Kinematic_CameraTool180_Pinc_Cam"]
            == f"cad_{side}_gripper_cam"
        )
        assert (
            owners[prefix + "Kinematic_CameraTool180_Pinc_Internal_Rod_L"]
            == f"cad_{side}_gripper_internal_L"
        )
    assert not KINEMATICS["capabilities"]["gripper_actuation"]


def urdf_fk(deltas: dict[str, float]) -> dict[str, np.ndarray]:
    deltas = resolved_deltas(KINEMATICS, deltas)
    root = ET.parse(PACKAGE / "urdf/cleany_freecad_kinematic.urdf").getroot()
    transforms = {"base_link": np.eye(4)}
    for joint in root.findall("joint"):
        origin = np.fromstring(joint.find("origin").get("xyz"), sep=" ")
        local = np.eye(4)
        if joint.find("axis") is not None:
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            local[:3, :3] = Rotation.from_rotvec(
                axis * deltas.get(joint.get("name"), 0.0)
            ).as_matrix()
        local[:3, 3] = origin
        transforms[joint.find("child").get("link")] = (
            transforms[joint.find("parent").get("link")] @ local
        )
    return transforms


def test_saved_pose_is_reproduced_without_claiming_a_physical_zero() -> None:
    root = ET.parse(PACKAGE / "urdf/cleany_freecad_kinematic.urdf").getroot()
    fk = urdf_fk({})
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            offset = (
                np.zeros(3)
                if visual.find("origin") is None
                else np.fromstring(visual.find("origin").get("xyz"), sep=" ")
            )
            transform = fk[link.get("name")].copy()
            transform[:3, 3] += transform[:3, :3] @ offset
            assert transform == pytest.approx(np.eye(4), abs=2e-11)
    for joint in root.findall("joint"):
        assert joint.get("name").startswith(
            "cad_fixed_" if joint.get("type") == "fixed" else "cad_delta_"
        )
        assert joint.find("limit") is None
    assert root.find(".//inertial") is None
    assert root.find(".//ros2_control") is None


def test_random_delta_fk_matches_independent_urdf_calculation() -> None:
    random = np.random.default_rng(817)
    for _ in range(6):
        deltas = {
            name: float(random.uniform(-0.2, 0.2))
            for name in KINEMATICS["independent_joint_names"]
        }
        expected, actual = urdf_fk(deltas), forward_kinematics(KINEMATICS, deltas)
        for link in expected:
            assert actual[link] == pytest.approx(expected[link], abs=2e-11)


def test_delta_rotates_descendants_around_the_extracted_axis_only() -> None:
    spec = next(
        s for s in KINEMATICS["links"] if s["joint"] == "cad_delta_right_shoulder_yaw"
    )
    angle = 0.17
    base, rotated = (
        forward_kinematics(KINEMATICS, {}),
        forward_kinematics(KINEMATICS, {spec["joint"]: angle}),
    )
    pivot = np.asarray(spec["point_base_m"])
    rotation = Rotation.from_rotvec(np.asarray(spec["axis_base"]) * angle).as_matrix()
    for name in [
        "cad_right_shoulder_pitch",
        "cad_right_elbow_pitch",
        "cad_right_wrist_roll",
    ]:
        assert rotated[name][:3, 3] == pytest.approx(
            pivot + rotation @ (base[name][:3, 3] - pivot)
        )
    assert rotated["cad_left_wrist_roll"] == pytest.approx(base["cad_left_wrist_roll"])


def test_mocap_preview_matches_delta_fk_without_mass_or_actuators() -> None:
    model = mujoco.MjModel.from_xml_path(
        str(PACKAGE / "mjcf/cleany_freecad_kinematic.xml")
    )
    data = mujoco.MjData(model)
    assert model.nmocap == 34
    assert model.nq == model.nv == model.nu == 0
    assert np.all(model.body_mass == 0)
    deltas = {name: 0.13 for name in KINEMATICS["independent_joint_names"]}
    expected = forward_kinematics(KINEMATICS, deltas)
    for spec in KINEMATICS["links"]:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, spec["link"])
        mocap_id = model.body_mocapid[body_id]
        transform = expected[spec["link"]]
        data.mocap_pos[mocap_id] = transform[:3, 3] + [0, 0, 0.38]
        x, y, z, w = Rotation.from_matrix(transform[:3, :3]).as_quat()
        data.mocap_quat[mocap_id] = [w, x, y, z]
    mujoco.mj_forward(model, data)
    for spec in KINEMATICS["links"]:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, spec["link"])
        assert data.xpos[body_id] == pytest.approx(
            expected[spec["link"]][:3, 3] + [0, 0, 0.38]
        )
        assert data.xmat[body_id].reshape(3, 3) == pytest.approx(
            expected[spec["link"]][:3, :3]
        )
        part = next(
            p
            for p in STATIC["parts"]
            if KINEMATICS["part_owners"][p["name"]] == spec["link"]
        )
        geom = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, part["mujoco_meshes"][0]["name"]
        )
        mesh = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        actual_vertices = (
            model.mesh_vert[start : start + count]
            @ data.geom_xmat[geom].reshape(3, 3).T
            + data.geom_xpos[geom]
        )
        dtype = np.dtype(
            [("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")]
        )
        source_vertices = np.frombuffer(
            (PACKAGE / part["mesh"]).read_bytes(), dtype=dtype, offset=84
        )["v"].reshape(-1, 3)
        transform = expected[spec["link"]]
        expected_vertices = (
            (source_vertices - spec["point_base_m"]) @ transform[:3, :3].T
            + transform[:3, 3]
            + [0, 0, 0.38]
        )
        assert np.array(
            [actual_vertices.min(0), actual_vertices.max(0)]
        ) == pytest.approx(
            np.array([expected_vertices.min(0), expected_vertices.max(0)]), abs=2e-6
        )


def test_axis_line_comparison_ignores_sign_and_axial_origin() -> None:
    first = {"point": [1, 2, 3], "direction": [0, 0, 1]}
    second = {"point": [1, 2, 17], "direction": [0, 0, -1]}
    assert axis_difference(first, second) == pytest.approx((0, 0))
    assert canonical_direction(np.array([0.0, -1.0, 0.0])) == pytest.approx([0, 1, 0])
    with pytest.raises(ValueError):
        canonical_direction(np.zeros(3))
    with pytest.raises(ValueError):
        forward_kinematics(KINEMATICS, {"right_shoulder_yaw_joint": 0.2})


def test_ambiguous_or_missing_shaft_match_cannot_become_a_joint() -> None:
    axis = {"point": [0, 0, 0], "direction": [0, 0, 1], "area_mm2": 1, "faces": []}
    other = {**axis, "point": [1, 0, 0]}
    spec = {"joint": "test", "drive_horn": "horn", "servo": "servo", "supports": []}
    with pytest.raises(ValueError, match="Ambiguous"):
        matched_axis(
            spec, {"horn": [axis, other], "servo": [axis, other]}, 0.02, 0.0001
        )
    with pytest.raises(ValueError, match="No supported"):
        matched_axis(spec, {"horn": [axis], "servo": [other]}, 0.02, 0.0001)
