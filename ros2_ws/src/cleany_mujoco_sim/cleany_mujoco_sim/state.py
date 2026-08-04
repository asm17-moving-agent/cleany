from __future__ import annotations

import math
from functools import lru_cache

import mujoco
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan

_SCALAR_JOINT_TYPES = (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)


@lru_cache(maxsize=None)
def actuated_joint_ids(model: mujoco.MjModel) -> list[int]:
    joint_ids: list[int] = []
    seen: set[int] = set()
    for actuator_id in range(model.nu):
        if (
            model.actuator_trntype[actuator_id]
            != mujoco.mjtTrn.mjTRN_JOINT
        ):
            continue
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if (
            joint_id < 0
            or joint_id in seen
            or model.jnt_type[joint_id] not in _SCALAR_JOINT_TYPES
        ):
            continue
        seen.add(joint_id)
        joint_ids.append(joint_id)
    return joint_ids


@lru_cache(maxsize=None)
def steps_per_tick(timestep: float, publish_rate_hz: float) -> int:
    return max(1, round((1.0 / publish_rate_hz) / timestep))


@lru_cache(maxsize=None)
def actuated_joint_names(model: mujoco.MjModel) -> list[str]:
    return [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        for i in actuated_joint_ids(model)
    ]


def joint_positions(model: mujoco.MjModel, data: mujoco.MjData) -> list[float]:
    return [float(data.qpos[model.jnt_qposadr[i]]) for i in actuated_joint_ids(model)]


def joint_velocities(model: mujoco.MjModel, data: mujoco.MjData) -> list[float]:
    return [float(data.qvel[model.jnt_dofadr[i]]) for i in actuated_joint_ids(model)]


def joint_state_msg(
    model: mujoco.MjModel, data: mujoco.MjData, stamp: Time
) -> JointState:
    msg = JointState()
    msg.header.stamp = stamp.to_msg()
    msg.name = actuated_joint_names(model)
    msg.position = joint_positions(model, data)
    msg.velocity = joint_velocities(model, data)
    return msg


def apply_joint_cmd(
    model: mujoco.MjModel, data: mujoco.MjData, msg: JointState
) -> None:
    name_to_qposadr = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i): model.jnt_qposadr[i]
        for i in actuated_joint_ids(model)
    }
    for name, position in zip(msg.name, msg.position):
        qposadr = name_to_qposadr.get(name)
        if qposadr is not None:
            data.qpos[qposadr] = position


def ros_quaternion_from_mujoco(wxyz: np.ndarray) -> tuple[float, float, float, float]:
    return (float(wxyz[1]), float(wxyz[2]), float(wxyz[3]), float(wxyz[0]))


def odometry_msg(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    stamp: Time,
    odom_frame_id: str,
    base_frame_id: str,
) -> Odometry:
    msg = Odometry()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = odom_frame_id
    msg.child_frame_id = base_frame_id

    pos = data.xpos[body_id]
    quat = ros_quaternion_from_mujoco(data.xquat[body_id])
    msg.pose.pose.position.x = float(pos[0])
    msg.pose.pose.position.y = float(pos[1])
    msg.pose.pose.position.z = float(pos[2])
    msg.pose.pose.orientation.x = quat[0]
    msg.pose.pose.orientation.y = quat[1]
    msg.pose.pose.orientation.z = quat[2]
    msg.pose.pose.orientation.w = quat[3]

    jnt_adr = model.body_jntadr[body_id]
    if model.body_jntnum[body_id] > 0 and model.jnt_type[jnt_adr] == mujoco.mjtJoint.mjJNT_FREE:
        dof_adr = model.jnt_dofadr[jnt_adr]
        # MuJoCo stores free-joint linear velocity in the world frame, while
        # nav_msgs/Odometry requires twist in child_frame_id (base_link).
        body_rotation = data.xmat[body_id].reshape(3, 3)
        body_linear_velocity = body_rotation.T @ data.qvel[dof_adr:dof_adr + 3]
        msg.twist.twist.linear.x = float(body_linear_velocity[0])
        msg.twist.twist.linear.y = float(body_linear_velocity[1])
        msg.twist.twist.linear.z = float(body_linear_velocity[2])
        # MuJoCo free-joint angular velocity is already body-local.
        msg.twist.twist.angular.x = float(data.qvel[dof_adr + 3])
        msg.twist.twist.angular.y = float(data.qvel[dof_adr + 4])
        msg.twist.twist.angular.z = float(data.qvel[dof_adr + 5])
    return msg


def transform_msg(
    data: mujoco.MjData,
    body_id: int,
    stamp: Time,
    parent_frame_id: str,
    child_frame_id: str,
) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = parent_frame_id
    msg.child_frame_id = child_frame_id

    pos = data.xpos[body_id]
    quat = ros_quaternion_from_mujoco(data.xquat[body_id])
    msg.transform.translation.x = float(pos[0])
    msg.transform.translation.y = float(pos[1])
    msg.transform.translation.z = float(pos[2])
    msg.transform.rotation.x = quat[0]
    msg.transform.rotation.y = quat[1]
    msg.transform.rotation.z = quat[2]
    msg.transform.rotation.w = quat[3]
    return msg


def static_site_transform_msg(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    site_id: int,
    stamp: Time,
    parent_frame_id: str,
    child_frame_id: str,
) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = parent_frame_id
    msg.child_frame_id = child_frame_id

    body_rot = data.xmat[body_id].reshape(3, 3)
    site_rot = data.site_xmat[site_id].reshape(3, 3)
    rel_pos = body_rot.T @ (data.site_xpos[site_id] - data.xpos[body_id])
    rel_rot = body_rot.T @ site_rot
    rel_quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(rel_quat, rel_rot.reshape(9))
    quat = ros_quaternion_from_mujoco(rel_quat)

    msg.transform.translation.x = float(rel_pos[0])
    msg.transform.translation.y = float(rel_pos[1])
    msg.transform.translation.z = float(rel_pos[2])
    msg.transform.rotation.x = quat[0]
    msg.transform.rotation.y = quat[1]
    msg.transform.rotation.z = quat[2]
    msg.transform.rotation.w = quat[3]
    return msg


def scan_sample_count(scan_samples: int, scan_sample_rate_hz: float, scan_rate_hz: float) -> int:
    if scan_rate_hz <= 0:
        raise ValueError('scan_rate_hz must be positive')
    if scan_samples > 0:
        return scan_samples
    return max(1, round(scan_sample_rate_hz / scan_rate_hz))


def laser_scan_msg(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site_id: int,
    bodyexclude: int,
    stamp: Time,
    frame_id: str,
    scan_rate_hz: float,
    samples: int,
    range_min: float,
    range_max: float,
) -> LaserScan:
    msg = LaserScan()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = frame_id
    msg.angle_min = -math.pi
    msg.angle_increment = 2.0 * math.pi / samples
    msg.angle_max = math.pi - msg.angle_increment
    msg.time_increment = (1.0 / scan_rate_hz) / samples
    msg.scan_time = 1.0 / scan_rate_hz
    msg.range_min = range_min
    msg.range_max = range_max

    origin = np.asarray(data.site_xpos[site_id], dtype=np.float64)
    site_rot = data.site_xmat[site_id].reshape(3, 3)
    geomid = np.array([-1], dtype=np.int32)
    excluded_root_id = model.body_rootid[bodyexclude]
    ranges: list[float] = []
    for i in range(samples):
        angle = msg.angle_min + i * msg.angle_increment
        local_dir = np.array([math.cos(angle), math.sin(angle), 0.0], dtype=np.float64)
        ray_dir = site_rot @ local_dir
        ray_origin = origin.copy()
        total_distance = 0.0
        hit_distance = math.inf
        while total_distance < range_max:
            geomid[0] = -1
            distance = float(
                mujoco.mj_ray(
                    model, data, ray_origin, ray_dir, None, True, bodyexclude, geomid
                )
            )
            if distance < 0:
                break

            total_distance += distance
            hit_body_id = model.geom_bodyid[geomid[0]] if geomid[0] >= 0 else -1
            if hit_body_id >= 0 and model.body_rootid[hit_body_id] == excluded_root_id:
                skip_distance = max(1e-4, range_min * 1e-3)
                ray_origin = ray_origin + ray_dir * (distance + skip_distance)
                total_distance += skip_distance
                continue

            hit_distance = total_distance
            break

        if range_min <= hit_distance <= range_max:
            ranges.append(hit_distance)
        else:
            ranges.append(math.inf)
    msg.ranges = ranges
    return msg
