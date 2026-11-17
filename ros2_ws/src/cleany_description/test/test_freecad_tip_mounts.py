"""Verify nominal fixed tip seats without asserting physical clamp preload."""

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
sys.path.insert(0, str(PACKAGE.parents[2] / "tools"))
from extract_freecad_kinematics import forward_kinematics
from freecad_tip_mounts import blocked_rotation

MODEL = json.loads((PACKAGE / "config/geometry/freecad_kinematics.json").read_text())


def test_all_four_seats_have_shared_bores_and_blocked_rigid_rotation() -> None:
    mounts = [m for g in MODEL["gripper_linkages"] for m in g["tip_mounts"]]
    assert len(mounts) == 4
    for mount in mounts:
        assert mount["classification"] == "nominal_fixed_keyed_seat"
        assert mount["free_rigid_rotation_about_fastening_axis"] is False
        assert mount["bore_diameters_mm"] == {
            "carrier": [2.05],
            "shell": [3.1, 5.5],
            "insert": [3.1],
        }
        assert all(p["overlap_mm3"] <= 1e-5 for p in mount["baseline"].values())
        assert mount["baseline"]["carrier:shell"]["clearance_mm"] == pytest.approx(
            0.2021655884, abs=1e-7
        )
        assert blocked_rotation(mount["rotation_probes"], 1e-5)
        assert {p["angle_deg"] for p in mount["rotation_probes"]} == {
            -1.0,
            -0.1,
            0.1,
            1.0,
        }


def test_fixed_tip_links_do_not_add_rotation_commands() -> None:
    root = ET.parse(PACKAGE / "urdf/cleany_freecad_kinematic.urdf").getroot()
    assert len(MODEL["fixed_attachments"]) == 4
    assert len(MODEL["independent_joint_names"]) == 18
    for attachment in MODEL["fixed_attachments"]:
        joint = root.find(f"joint[@name='{attachment['joint']}']")
        assert joint.get("type") == "fixed"
        assert joint.find("axis") is None and joint.find("limit") is None
        assert attachment["joint"] not in MODEL["independent_joint_names"]
        assert all(
            MODEL["part_owners"][p] == attachment["link"] for p in attachment["parts"]
        )


@pytest.mark.parametrize("cam_delta", [-0.5, 0.0, 0.15])
def test_tips_follow_their_carriers_in_fk_and_mujoco(cam_delta) -> None:
    pose = {g["cam_joint"]: cam_delta for g in MODEL["gripper_linkages"]}
    pose["cad_delta_right_wrist_roll"] = 0.12
    fk = forward_kinematics(MODEL, pose)
    model = mujoco.MjModel.from_xml_path(
        str(PACKAGE / "mjcf/cleany_freecad_kinematic.xml")
    )
    data = mujoco.MjData(model)
    assert model.nmocap == 34
    for spec in MODEL["links"]:
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, spec["link"])
        index = model.body_mocapid[body]
        data.mocap_pos[index] = fk[spec["link"]][:3, 3] + [0, 0, 0.38]
        q = Rotation.from_matrix(fk[spec["link"]][:3, :3]).as_quat()
        data.mocap_quat[index] = [q[3], *q[:3]]
    mujoco.mj_forward(model, data)
    for attachment in MODEL["fixed_attachments"]:
        parent, child = fk[attachment["parent"]], fk[attachment["link"]]
        relative = np.linalg.inv(parent) @ child
        assert relative[:3, :3] == pytest.approx(np.eye(3), abs=1e-12)
        assert relative[:3, 3] == pytest.approx(
            attachment["origin_parent_m"], abs=1e-12
        )
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, attachment["link"])
        assert model.body_mocapid[body] == -1
        assert data.xpos[body] == pytest.approx(child[:3, 3] + [0, 0, 0.38])
        assert data.xmat[body].reshape(3, 3) == pytest.approx(child[:3, :3])


def test_only_physical_retention_and_material_parameters_remain_unknown() -> None:
    for group in MODEL["gripper_linkages"]:
        for mount in group["tip_mounts"]:
            for key in [
                "screw_specification",
                "clamp_preload",
                "pad_material_and_retention",
                "compliance_and_backlash",
                "physical_load_retention",
            ]:
                assert mount[key] is None


def test_one_sided_or_missing_probe_cannot_establish_a_fixed_seat() -> None:
    assert not blocked_rotation([], 1e-5)
    assert not blocked_rotation(
        [{"angle_deg": 1.0, "carrier_overlap_mm3": {"insert": 2.0}}], 1e-5
    )
    assert not blocked_rotation(
        [
            {"angle_deg": -1.0, "carrier_overlap_mm3": {"insert": 2.0}},
            {"angle_deg": 1.0, "carrier_overlap_mm3": {"insert": 0.0}},
        ],
        1e-5,
    )
