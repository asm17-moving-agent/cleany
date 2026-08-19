from __future__ import annotations

import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest


CANONICAL_LIMITS = {
    "left_shoulder_yaw_joint": (-2.16, 2.16),
    "left_shoulder_pitch_joint": (-0.22, 3.37),
    "left_elbow_pitch_joint": (-0.22, 3.14),
    "left_wrist_pitch_joint": (-1.6580628494556928, 1.6580627293335335),
    "left_wrist_roll_joint": (-2.7438472969992493, 2.841206309382605),
    "left_gripper_joint": (-0.37453297762778586, 1.7453291995659765),
    "right_shoulder_yaw_joint": (-2.16, 2.16),
    "right_shoulder_pitch_joint": (-0.22, 3.37),
    "right_elbow_pitch_joint": (-0.22, 3.14),
    "right_wrist_pitch_joint": (-1.6580628494556928, 1.6580627293335335),
    "right_wrist_roll_joint": (-2.7438472969992493, 2.841206309382605),
    "right_gripper_joint": (-0.37453297762778586, 1.7453291995659765),
}
URDF_ENTRYPOINTS = (
    "cleany.urdf.xacro",
    "cleany_control.urdf.xacro",
)


def _source_root() -> Path:
    return Path(__file__).parents[1]


def _expand_urdf(entrypoint: str, *xacro_args: str) -> ET.Element:
    description = _source_root()
    return ET.fromstring(
        subprocess.check_output(
            [
                "xacro",
                str(description / "urdf" / entrypoint),
                *xacro_args,
            ]
        )
    )


def _physical_model_xml(root: ET.Element) -> tuple[bytes, ...]:
    return tuple(
        ET.tostring(node).strip()
        for node in root
        if node.tag in {"material", "link", "joint"}
    )


def test_description_entrypoints_share_canonical_geometry() -> None:
    roots = tuple(_expand_urdf(entrypoint) for entrypoint in URDF_ENTRYPOINTS)
    assert _physical_model_xml(roots[0]) == _physical_model_xml(roots[1])

    for root in roots:
        links = root.findall("./link")
        joints = root.findall("./joint")
        assert len(links) == 13
        assert len(joints) == 12
        joint_names = {joint.attrib["name"] for joint in joints}
        assert joint_names == set(CANONICAL_LIMITS)
        for joint in joints:
            limit = joint.find("limit")
            assert limit is not None
            assert (
                float(limit.attrib["lower"]),
                float(limit.attrib["upper"]),
            ) == pytest.approx(CANONICAL_LIMITS[joint.attrib["name"]])


def test_basic_description_does_not_select_a_control_backend() -> None:
    root = _expand_urdf("cleany.urdf.xacro")
    assert root.find("./ros2_control") is None
    assert root.find(".//hardware/plugin") is None


def test_control_description_exposes_arm_and_gripper_interfaces() -> None:
    root = _expand_urdf(
        "cleany_control.urdf.xacro",
        "mujoco_model:=/tmp/cleany_control_scene.xml",
        "headless:=true",
        "sim_speed_factor:=1.25",
    )
    control = root.find("./ros2_control")
    assert control is not None
    assert control.attrib == {"name": "MujocoSystem", "type": "system"}
    hardware = control.find("./hardware")
    assert hardware is not None
    plugin = hardware.find("./plugin")
    assert plugin is not None
    assert plugin.text == "mujoco_ros2_control/MujocoSystemInterface"
    parameters = {
        parameter.attrib["name"]: parameter.text
        for parameter in hardware.findall("./param")
    }
    assert parameters["mujoco_model"] == "/tmp/cleany_control_scene.xml"
    assert parameters["headless"].lower() == "true"
    assert parameters["sim_speed_factor"] == "1.25"
    assert parameters["camera_publish_rate"] == "10.0"
    assert parameters["initial_keyframe"] == "handeye_ros2_control_home"

    sensors = control.findall("./sensor")
    assert len(sensors) == 1
    assert sensors[0].attrib == {"name": "left_wrist_rgb"}
    sensor_parameters = {
        parameter.attrib["name"]: parameter.text
        for parameter in sensors[0].findall("./param")
    }
    assert sensor_parameters == {
        "frame_name": "left_wrist_rgb_vendor_frame",
        "info_topic": "/left_wrist_rgb/camera_info",
        "image_topic": "/left_wrist_rgb/color",
        "depth_topic": "/left_wrist_rgb/depth",
    }

    expected_arm_joints = {
        f"{side}_{suffix}"
        for side in ("left", "right")
        for suffix in (
            "shoulder_yaw_joint",
            "shoulder_pitch_joint",
            "elbow_pitch_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint",
        )
    }
    control_joints = {
        joint.attrib["name"]: joint for joint in control.findall("./joint")
    }
    expected_gripper_joints = {
        "left_gripper_joint",
        "right_gripper_joint",
    }
    assert set(control_joints) == expected_arm_joints | expected_gripper_joints
    for joint_name in expected_arm_joints:
        joint = control_joints[joint_name]
        assert [
            interface.attrib["name"]
            for interface in joint.findall("./command_interface")
        ] == ["position"]
        state_interfaces = {
            interface.attrib["name"]: interface
            for interface in joint.findall("./state_interface")
        }
        assert set(state_interfaces) == {"position", "velocity"}
        initial_value = state_interfaces["position"].find(
            "./param[@name='initial_value']"
        )
        assert initial_value is not None
        assert initial_value.text == "0.0"
        assert state_interfaces["velocity"].find("./param") is None

    for joint_name in expected_gripper_joints:
        joint = control_joints[joint_name]
        assert joint.findall("./command_interface") == []
        state_interfaces = {
            interface.attrib["name"]: interface
            for interface in joint.findall("./state_interface")
        }
        assert set(state_interfaces) == {"position", "velocity"}
        initial_value = state_interfaces["position"].find(
            "./param[@name='initial_value']"
        )
        assert initial_value is not None
        assert initial_value.text == "0.0"


def test_mjcf_uses_canonical_arm_joint_names_and_limits() -> None:
    root = ET.parse(_source_root() / "mjcf" / "cleany.xml").getroot()
    joints = {
        node.attrib["name"]: node
        for node in root.findall(".//joint")
        if "name" in node.attrib
    }
    for name, expected in CANONICAL_LIMITS.items():
        assert name in joints
        actual = tuple(
            float(value) for value in joints[name].attrib["range"].split()
        )
        assert actual == pytest.approx(expected)

    legacy = {
        "Rotation_L",
        "Pitch_L",
        "Elbow_L",
        "Wrist_Pitch_L",
        "Wrist_Roll_L",
        "Jaw_L",
        "Rotation_R",
        "Pitch_R",
        "Elbow_R",
        "Wrist_Pitch_R",
        "Wrist_Roll_R",
        "Jaw_R",
    }
    assert legacy.isdisjoint(joints)


def test_mjcf_non_free_joints_have_unique_names() -> None:
    model = mujoco.MjModel.from_xml_path(
        str(_source_root() / "mjcf" / "cleany.xml")
    )
    names = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE:
            continue
        name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
        )
        assert name is not None
        names.append(name)
    assert len(names) == len(set(names))


def test_mjcf_arm_axes_are_finite_and_normalized() -> None:
    root = ET.parse(_source_root() / "mjcf" / "cleany.xml").getroot()
    for node in root.findall(".//joint"):
        if node.attrib.get("name") not in CANONICAL_LIMITS:
            continue
        axis = tuple(float(value) for value in node.attrib["axis"].split())
        assert all(math.isfinite(value) for value in axis)
        assert math.sqrt(
            sum(value * value for value in axis)
        ) == pytest.approx(1.0)


def test_mjcf_uses_rep103_base_and_joint_axes() -> None:
    model = mujoco.MjModel.from_xml_path(
        str(_source_root() / "mjcf" / "cleany.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    base_rotation = data.xmat[base_id].reshape(3, 3)
    expected_axes = {
        "rear_left_wheel_joint": (0.0, 1.0, 0.0),
        "rear_right_wheel_joint": (0.0, 1.0, 0.0),
        "front_left_wheel_joint": (0.0, 1.0, 0.0),
        "front_right_wheel_joint": (0.0, 1.0, 0.0),
        "left_shoulder_yaw_joint": (0.0, 0.0, 1.0),
        "right_shoulder_yaw_joint": (0.0, 0.0, 1.0),
        "head_pan_joint": (0.0, 0.0, 1.0),
        "head_tilt_joint": (0.0, 1.0, 0.0),
    }
    for joint_name, expected_axis in expected_axes.items():
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        axis_in_base = base_rotation.T @ data.xaxis[joint_id]
        assert axis_in_base == pytest.approx(expected_axis, abs=1e-5)


def test_mjcf_uses_rep103_camera_optical_axes() -> None:
    model = mujoco.MjModel.from_xml_path(
        str(_source_root() / "mjcf" / "cleany.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    base_rotation = data.xmat[base_id].reshape(3, 3)
    camera_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, "head_realsense_rgb"
    )
    camera_rotation = data.cam_xmat[camera_id].reshape(3, 3)
    assert base_rotation.T @ camera_rotation[:, 0] == pytest.approx(
        (0.0, -1.0, 0.0), abs=1e-5
    )
    assert base_rotation.T @ camera_rotation[:, 1] == pytest.approx(
        (0.0, 0.0, 1.0), abs=1e-5
    )
    assert base_rotation.T @ -camera_rotation[:, 2] == pytest.approx(
        (1.0, 0.0, 0.0), abs=1e-5
    )

    for stream in ("rgb", "depth"):
        site_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            f"head_camera_{stream}_optical_frame",
        )
        optical_rotation = (
            base_rotation.T @ data.site_xmat[site_id].reshape(3, 3)
        )
        assert optical_rotation[:, 0] == pytest.approx(
            (0.0, -1.0, 0.0), abs=1e-5
        )
        assert optical_rotation[:, 1] == pytest.approx(
            (0.0, 0.0, -1.0), abs=1e-5
        )
        assert optical_rotation[:, 2] == pytest.approx(
            (1.0, 0.0, 0.0), abs=1e-5
        )

    for side in ("left", "right"):
        camera_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, f"{side}_wrist_rgb"
        )
        site_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            f"{side}_wrist_rgb_optical_frame",
        )
        camera_rotation = data.cam_xmat[camera_id].reshape(3, 3)
        optical_rotation = data.site_xmat[site_id].reshape(3, 3)
        assert optical_rotation[:, 0] == pytest.approx(
            camera_rotation[:, 0], abs=1e-5
        )
        assert optical_rotation[:, 1] == pytest.approx(
            -camera_rotation[:, 1], abs=1e-5
        )
        assert optical_rotation[:, 2] == pytest.approx(
            -camera_rotation[:, 2], abs=1e-5
        )


def test_mjcf_mounts_arms_at_canonical_sides() -> None:
    model = mujoco.MjModel.from_xml_path(
        str(_source_root() / "mjcf" / "cleany.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    base_rotation = data.xmat[base_id].reshape(3, 3)
    left_base_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "Base"
    )
    right_base_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "Base_2"
    )
    left_position = base_rotation.T @ (
        data.xpos[left_base_id] - data.xpos[base_id]
    )
    right_position = base_rotation.T @ (
        data.xpos[right_base_id] - data.xpos[base_id]
    )
    assert left_position[:2] == pytest.approx((0.09, 0.11), abs=1e-6)
    assert right_position[:2] == pytest.approx((0.09, -0.11), abs=1e-6)


@pytest.mark.parametrize("urdf_entrypoint", URDF_ENTRYPOINTS)
def test_random_arm_fk_matches_mjcf(urdf_entrypoint: str) -> None:
    description = _source_root()
    urdf = _expand_urdf(urdf_entrypoint)
    urdf_joints = {node.attrib["name"]: node for node in urdf.findall("joint")}
    model = mujoco.MjModel.from_xml_path(
        str(description / "mjcf" / "cleany.xml")
    )
    data = mujoco.MjData(model)
    random = np.random.default_rng(260727)
    suffixes = (
        "shoulder_yaw_joint",
        "shoulder_pitch_joint",
        "elbow_pitch_joint",
        "wrist_pitch_joint",
        "wrist_roll_joint",
    )
    bodies = {"left": "Fixed_Jaw", "right": "Fixed_Jaw_2"}
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    for side in ("left", "right"):
        names = tuple(f"{side}_{suffix}" for suffix in suffixes)
        for _ in range(10):
            values = {
                name: float(random.uniform(*CANONICAL_LIMITS[name]))
                for name in names
            }
            mujoco.mj_resetData(model, data)
            for name, value in values.items():
                joint_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_JOINT, name
                )
                data.qpos[model.jnt_qposadr[joint_id]] = value
            mujoco.mj_forward(model, data)

            body_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, bodies[side]
            )
            base_rotation = data.xmat[base_id].reshape(3, 3)
            actual = np.eye(4)
            actual[:3, :3] = base_rotation.T @ data.xmat[body_id].reshape(3, 3)
            actual[:3, 3] = base_rotation.T @ (
                data.xpos[body_id] - data.xpos[base_id]
            )
            expected = _urdf_fk(urdf_joints, names, values)
            assert actual == pytest.approx(expected, abs=1e-5)


def _urdf_fk(
    joints: dict[str, ET.Element],
    names: tuple[str, ...],
    values: dict[str, float],
) -> np.ndarray:
    result = np.eye(4)
    for name in names:
        joint = joints[name]
        origin = joint.find("origin")
        axis = joint.find("axis")
        assert origin is not None and axis is not None
        fixed = np.eye(4)
        fixed[:3, :3] = _rpy(
            np.fromstring(origin.attrib.get("rpy", "0 0 0"), sep=" ")
        )
        fixed[:3, 3] = np.fromstring(origin.attrib["xyz"], sep=" ")
        motion = np.eye(4)
        motion[:3, :3] = _axis_angle(
            np.fromstring(axis.attrib["xyz"], sep=" "),
            values[name],
        )
        result = result @ fixed @ motion
    return result


def _rpy(values: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = values
    return (
        _axis_angle(np.array((0.0, 0.0, 1.0)), yaw)
        @ _axis_angle(np.array((0.0, 1.0, 0.0)), pitch)
        @ _axis_angle(np.array((1.0, 0.0, 0.0)), roll)
    )


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus = 1.0 - cosine
    return np.array(
        [
            [
                cosine + x * x * one_minus,
                x * y * one_minus - z * sine,
                x * z * one_minus + y * sine,
            ],
            [
                y * x * one_minus + z * sine,
                cosine + y * y * one_minus,
                y * z * one_minus - x * sine,
            ],
            [
                z * x * one_minus - y * sine,
                z * y * one_minus + x * sine,
                cosine + z * z * one_minus,
            ],
        ]
    )
