import math

import mujoco
import numpy as np
import pytest
from rclpy.time import Time
from sensor_msgs.msg import JointState

from cleany_mujoco_sim.state import (
    actuated_joint_names,
    apply_joint_cmd,
    joint_positions,
    joint_velocities,
    joint_state_msg,
    laser_scan_msg,
    odometry_msg,
    ros_quaternion_from_mujoco,
    scan_sample_count,
    static_site_transform_msg,
    steps_per_tick,
    transform_msg,
)


def test_actuated_joint_names_only_includes_actuator_backed_joints(model_data):
    model, _ = model_data
    assert actuated_joint_names(model) == ["shoulder"]


def test_joint_positions_matches_qpos(model_data):
    model, data = model_data
    data.qpos[model.jnt_qposadr[1]] = 0.5
    assert joint_positions(model, data) == pytest.approx([0.5])


def test_joint_velocities_matches_qvel(model_data):
    model, data = model_data
    data.qvel[model.jnt_dofadr[1]] = 0.7
    assert joint_velocities(model, data) == pytest.approx([0.7])


def test_joint_state_msg_fields(model_data):
    model, data = model_data
    data.qvel[model.jnt_dofadr[1]] = 0.7
    msg = joint_state_msg(model, data, Time())
    assert msg.name == ["shoulder"]
    assert msg.position == pytest.approx(joint_positions(model, data))
    assert msg.velocity == pytest.approx(joint_velocities(model, data))


def test_apply_joint_cmd_writes_qpos(model_data):
    model, data = model_data
    cmd = JointState()
    cmd.name = ["shoulder"]
    cmd.position = [0.3]
    apply_joint_cmd(model, data, cmd)
    assert data.qpos[model.jnt_qposadr[1]] == pytest.approx(0.3)


def test_steps_per_tick_matches_tick_period_to_timestep():
    assert steps_per_tick(timestep=0.002, publish_rate_hz=60.0) == 8


def test_steps_per_tick_never_zero_when_timestep_exceeds_tick_period():
    assert steps_per_tick(timestep=0.1, publish_rate_hz=60.0) == 1


def test_ros_quaternion_from_mujoco_reorders_wxyz_to_xyzw():
    assert ros_quaternion_from_mujoco(np.array([1.0, 0.1, 0.2, 0.3])) == pytest.approx(
        (0.1, 0.2, 0.3, 1.0)
    )


def test_odometry_msg_uses_body_pose_and_frames(model_data):
    model, data = model_data
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    mujoco.mj_forward(model, data)

    msg = odometry_msg(model, data, body_id, Time(), "odom", "base_link")

    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "base_link"
    assert msg.pose.pose.position.z == pytest.approx(0.0)


def test_odometry_twist_is_expressed_in_base_link(model_data):
    model, data = model_data
    body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "chassis"
    )
    joint_id = model.body_jntadr[body_id]
    qpos_address = model.jnt_qposadr[joint_id]
    qvel_address = model.jnt_dofadr[joint_id]
    half_yaw = math.pi / 4.0
    data.qpos[qpos_address + 3:qpos_address + 7] = (
        math.cos(half_yaw),
        0.0,
        0.0,
        math.sin(half_yaw),
    )
    # MuJoCo free-joint linear velocity is world-frame: world +Y is
    # base_link +X after a +90 degree yaw.
    data.qvel[qvel_address:qvel_address + 3] = (0.0, 2.0, 0.0)
    # Free-joint angular velocity is already body-local.
    data.qvel[qvel_address + 3:qvel_address + 6] = (0.1, 0.2, 0.3)
    mujoco.mj_forward(model, data)

    msg = odometry_msg(model, data, body_id, Time(), "odom", "base_link")

    assert (
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z,
    ) == pytest.approx((2.0, 0.0, 0.0), abs=1e-12)
    assert (
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z,
    ) == pytest.approx((0.1, 0.2, 0.3))


def test_transform_msg_uses_body_pose_and_frames(model_data):
    model, data = model_data
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    mujoco.mj_forward(model, data)

    msg = transform_msg(data, body_id, Time(), "odom", "base_link")

    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "base_link"


def test_static_site_transform_msg_uses_relative_site_pose(model_data):
    model, data = model_data
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "lidar_site")
    mujoco.mj_forward(model, data)

    msg = static_site_transform_msg(model, data, body_id, site_id, Time(), "base_link", "laser")

    assert msg.header.frame_id == "base_link"
    assert msg.child_frame_id == "laser"


def test_scan_sample_count_derives_a1m8_default_samples():
    assert scan_sample_count(0, 8000.0, 5.5) == 1455
    assert scan_sample_count(90, 8000.0, 5.5) == 90


def test_laser_scan_msg_matches_a1m8_defaults_and_hits_target(model_data):
    model, data = model_data
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "lidar_site")
    mujoco.mj_forward(model, data)

    msg = laser_scan_msg(
        model,
        data,
        site_id,
        body_id,
        Time(),
        "laser",
        scan_rate_hz=5.5,
        samples=4,
        range_min=0.15,
        range_max=12.0,
    )

    assert msg.header.frame_id == "laser"
    assert msg.range_max == pytest.approx(12.0)
    assert msg.scan_time == pytest.approx(1.0 / 5.5)
    assert msg.angle_increment == pytest.approx(math.pi / 2.0)
    assert msg.ranges[2] == pytest.approx(0.95)
    assert math.isinf(msg.ranges[0])
