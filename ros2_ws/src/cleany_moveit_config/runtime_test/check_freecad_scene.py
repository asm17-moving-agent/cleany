"""Verify the CAD MoveIt scene and OctoMap with a cloud in the CAD base frame."""

import json
import os
import struct
import time
from hashlib import sha256
from pathlib import Path
from xml.etree import ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.action.graph import get_action_names_and_types
from sensor_msgs.msg import JointState, PointCloud2, PointField


def main() -> None:
    """Publish a CAD pose and diagnostic points to the isolated inspection graph."""
    if (
        os.environ.get("ROS_LOCALHOST_ONLY") != "1"
        or int(os.environ.get("ROS_DOMAIN_ID", "0")) == 0
    ):
        raise ValueError("Run with the inspection launch in a localhost ROS domain")
    package = Path(get_package_share_directory("cleany_description"))
    model = ET.parse(package / "urdf/cleany_freecad_kinematic.urdf").getroot()
    joints = JointState()
    joints.name = [
        joint.get("name")
        for joint in model.findall("joint")
        if joint.get("type") == "continuous"
    ]
    joints.position = [0.0] * len(joints.name)
    points = [
        (0.60 + i * 0.012, -0.08 + j * 0.012, 0.30 + k * 0.012)
        for i in range(4)
        for j in range(4)
        for k in range(4)
    ]
    cloud = PointCloud2()
    cloud.header.frame_id = "base_link"
    cloud.height, cloud.width = 1, len(points)
    cloud.fields = [
        PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate("xyz")
    ]
    cloud.point_step, cloud.row_step = 12, 12 * len(points)
    cloud.data = b"".join(struct.pack("<fff", *point) for point in points)
    cloud.is_dense = True

    rclpy.init()
    node = rclpy.create_node("check_freecad_scene", namespace="/freecad_inspection")
    try:
        state_pub = node.create_publisher(JointState, "joint_states", 10)
        cloud_pub = node.create_publisher(PointCloud2, "scene_cloud", 10)

        def publish():
            stamp = node.get_clock().now().to_msg()
            joints.header.stamp = stamp
            cloud.header.stamp = stamp
            state_pub.publish(joints)
            cloud_pub.publish(cloud)

        timer = node.create_timer(0.1, publish)
        client = node.create_client(GetPlanningScene, "get_planning_scene")
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.SCENE_SETTINGS
            | PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.OCTOMAP
        )
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            if not client.service_is_ready():
                rclpy.spin_once(node, timeout_sec=0.2)
                continue
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=8)
            if not future.done():
                client.remove_pending_request(future)
                continue
            scene = future.result().scene
            octomap = scene.world.octomap.octomap
            if octomap.data:
                assert scene.robot_model_name == model.get("name")
                assert set(scene.robot_state.joint_state.name) == set(joints.name)
                assert scene.world.octomap.header.frame_id == "base_link"
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        else:
            raise RuntimeError("The CAD planning scene did not return an OctoMap")
        services = {
            name.rsplit("/", 1)[-1] for name, _ in node.get_service_names_and_types()
        }
        actions = {
            name.rsplit("/", 1)[-1] for name, _ in get_action_names_and_types(node)
        }
        assert not services & {
            "plan_kinematic_path",
            "compute_ik",
            "compute_cartesian_path",
        }
        assert not actions & {
            "move_action",
            "execute_trajectory",
            "follow_joint_trajectory",
        }
        print(
            json.dumps(
                {
                    "model": scene.robot_model_name,
                    "joints": len(joints.name),
                    "octomap_frame": scene.world.octomap.header.frame_id,
                    "octomap_bytes": len(octomap.data),
                    "octomap_sha256": sha256(bytes(octomap.data)).hexdigest(),
                    "motion_interfaces_absent": True,
                }
            )
        )
        node.destroy_timer(timer)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
