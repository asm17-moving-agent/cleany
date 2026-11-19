"""Check the portable measured profile against the expanded robot frame tree."""

import json
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from test_model_parity import _expand_urdf, _rpy, _urdf_fk


PACKAGE = Path(__file__).resolve().parents[1]
PROFILE = PACKAGE / "config/calibration/jetson_20261009"
PROFILE_ARG = "calibration_profile:=jetson_20261009"


def _calibration(filename: str) -> dict:
    return yaml.safe_load((PROFILE / filename).read_text())


def _origin_transform(joint: ET.Element) -> np.ndarray:
    origin = joint.find("origin")
    transform = np.eye(4)
    transform[:3, 3] = np.fromstring(origin.get("xyz"), sep=" ")
    transform[:3, :3] = _rpy(np.fromstring(origin.get("rpy"), sep=" "))
    return transform


@pytest.mark.parametrize("side", ("left", "right"))
def test_wrist_extrinsics_match_frozen_handeye_matrix(side: str) -> None:
    root = _expand_urdf("cleany.urdf.xacro", PROFILE_ARG)
    calibration = _calibration(f"{side}_wrist_camera_extrinsics.yaml")
    joint = root.find(f"joint[@name='{side}_wrist_camera_calibrated_joint']")
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == calibration["parent_frame"] == f"{side}_gripper_frame"
    assert joint.find("child").get("link") == calibration["child_frame"]
    expected = np.array(calibration["parent_T_child"])
    assert _origin_transform(joint) == pytest.approx(expected, abs=1e-12)
    assert expected[3] == pytest.approx([0, 0, 0, 1])
    assert np.linalg.det(expected[:3, :3]) == pytest.approx(1)
    assert expected[:3, :3].T @ expected[:3, :3] == pytest.approx(np.eye(3), abs=1e-12)

    alias = root.find(f"joint[@name='{side}_wrist_rgb_optical_alias_joint']")
    assert alias.find("parent").get("link") == calibration["child_frame"]
    assert alias.find("child").get("link") == f"{side}_wrist_rgb_optical_frame"
    assert _origin_transform(alias) == pytest.approx(np.eye(4))
    joints = {element.get("name"): element for element in root.findall("joint")}
    chain = (
        f"{side}_shoulder_yaw_joint",
        f"{side}_shoulder_pitch_joint", f"{side}_elbow_pitch_joint",
        f"{side}_wrist_pitch_joint", f"{side}_wrist_roll_joint",
    )
    for overrides in ({}, {f"{side}_shoulder_yaw_joint": 0.4, f"{side}_elbow_pitch_joint": 0.8}):
        angles = dict.fromkeys(chain, 0.0) | overrides
        parent = _urdf_fk(joints, chain, angles)
        camera = _urdf_fk(joints, (*chain, joint.get("name")), angles)
        assert camera == pytest.approx(parent @ expected, abs=1e-12)


def test_head_rgb_calibration_changes_only_rgb_optical_origin() -> None:
    nominal = _expand_urdf("cleany.urdf.xacro")
    calibrated = _expand_urdf("cleany.urdf.xacro", PROFILE_ARG)
    data = _calibration("head_rgb_extrinsics.yaml")
    joint = calibrated.find("joint[@name='head_camera_rgb_optical_joint']")
    assert joint.find("parent").get("link") == data["parent_frame"]
    assert joint.find("child").get("link") == data["child_frame"]
    transform = _origin_transform(joint)
    assert transform[:3, 3] == pytest.approx(np.fromstring(data["xyz"], sep=" "))
    assert transform[:3, :3] == pytest.approx(_rpy(np.fromstring(data["rpy"], sep=" ")))
    assert np.linalg.norm(transform[:3, 3]) > 0.03
    for baseline in nominal.findall("joint"):
        if baseline.get("name") == "head_camera_rgb_optical_joint":
            continue
        actual = calibrated.find(f"joint[@name='{baseline.get('name')}']")
        assert ET.tostring(actual).strip() == ET.tostring(baseline).strip()
    for baseline in nominal.findall("link"):
        actual = calibrated.find(f"link[@name='{baseline.get('name')}']")
        assert ET.tostring(actual).strip() == ET.tostring(baseline).strip()


def test_profile_is_opt_in_and_missing_profile_fails() -> None:
    nominal = _expand_urdf("cleany.urdf.xacro")
    assert nominal.find("link[@name='left_wrist_camera_optical_frame']") is None
    assert nominal.find("link[@name='right_wrist_camera_optical_frame']") is None
    assert nominal.find("joint[@name='head_camera_rgb_optical_joint']/origin").get("xyz") == "0 0 0"
    with pytest.raises(subprocess.CalledProcessError):
        _expand_urdf("cleany.urdf.xacro", "calibration_profile:=missing_profile")


def test_arm_only_model_keeps_wrist_calibration_without_head() -> None:
    root = _expand_urdf("cleany.urdf.xacro", PROFILE_ARG, "include_head_camera:=false")
    assert root.find("link[@name='head_camera_rgb_optical_frame']") is None
    for side in ("left", "right"):
        assert root.find(f"link[@name='{side}_wrist_camera_optical_frame']") is not None
    assert root.find("joint[@name='top_base_joint']") is not None


@pytest.mark.parametrize("filename", (
    "head_rgb_extrinsics.yaml", "left_wrist_camera_extrinsics.yaml",
    "right_wrist_camera_extrinsics.yaml",
))
def test_calibration_inputs_are_portable_and_provisional(filename: str) -> None:
    data = _calibration(filename)
    assert data["absolute_calibration_verified"] is False
    assert data["status"] == "provisional_20plus5_manual_stationary"
    assert data["training_views"] == 20
    assert data.get("validation_views", data.get("held_out_views")) == 5
    intrinsics_path = Path(data["intrinsics_file"])
    assert not intrinsics_path.is_absolute()
    intrinsics = json.loads((PROFILE / intrinsics_path).read_text())
    assert np.isfinite(np.array(intrinsics["K"])).all()
    assert np.isfinite(np.array(intrinsics["D"])).all()
    assert intrinsics["width"] > 0 and intrinsics["height"] > 0


def test_recorded_left_wrist_range_follows_measured_ticks() -> None:
    reference = json.loads((PROFILE / "reference/left_arm_ros_reference_20261005.json").read_text())
    profile = _calibration("arm_joint_calibration.yaml")
    mapping = profile["arms"]["left"]["joints"]["left_wrist_roll_joint"]
    expected = sorted(
        mapping["direction"] * (tick - mapping["zero_tick"])
        * profile["encoder_scale_rad_per_tick"] + mapping["offset_rad"]
        for tick in reference["wrist_range"]["applied_raw_limits"]
    )
    assert expected == pytest.approx(reference["wrist_range"]["ros_angle_limits_rad"])
    assert mapping["observed_position_limits_rad"] == pytest.approx(expected)


def test_encoder_references_keep_device_configuration_separate() -> None:
    profile = _calibration("arm_joint_calibration.yaml")
    assert profile["encoder_scale_rad_per_tick"] == pytest.approx(math.tau / 4096)
    assert profile["absolute_angle_accuracy_validated"] is False
    ids = []
    for arm in profile["arms"].values():
        assert (PROFILE / arm["motor_calibration_file"]).is_file()
        ids.extend(joint["motor_id"] for joint in arm["joints"].values())
    assert sorted(ids) == list(range(1, 13))
    head = json.loads((PROFILE / "head_joint_calibration.json").read_text())
    assert head["reference_ticks"] == [2009, 2067]
    assert head["encoder_directions"] == [-1, 1]
    assert head["motor_configuration_written"] is False
