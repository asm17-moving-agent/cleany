"""Exercise lower-body occupancy filtering and obstacle contacts without motion."""

import argparse
import json
import math
import os
import struct
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2, PointField
from shape_msgs.msg import SolidPrimitive


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probes", type=Path)
    args = parser.parse_args()
    if (
        os.environ.get("ROS_LOCALHOST_ONLY") != "1"
        or int(os.environ.get("ROS_DOMAIN_ID", "0")) == 0
    ):
        raise ValueError("Use a separate localhost inspection domain")
    probes = json.loads(args.probes.read_text())
    package = Path(get_package_share_directory("cleany_description"))
    model = ET.parse(package / "urdf/cleany_freecad_kinematic.urdf").getroot()
    joints = JointState()
    joints.name = [
        joint.get("name")
        for joint in model.findall("joint")
        if joint.get("type") == "continuous"
    ]
    positions = probes.get("joint_positions_rad", {})
    if set(positions) - set(joints.name) or not all(
        math.isfinite(value) for value in positions.values()
    ):
        raise ValueError("Probe positions must be finite CAD joint coordinates")
    joints.position = [float(positions.get(name, 0.0)) for name in joints.name]
    rclpy.init()
    node = rclpy.create_node(
        "check_freecad_lower_body", namespace="/freecad_inspection"
    )
    try:
        state_pub = node.create_publisher(JointState, "joint_states", 10)
        cloud_pub = node.create_publisher(
            PointCloud2, "scene_cloud", qos_profile_sensor_data
        )
        cloud = PointCloud2()
        cloud.header.frame_id = "base_link"
        cloud.height, cloud.width = 1, len(probes["cloud"])
        cloud.fields = [
            PointField(
                name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1
            )
            for index, name in enumerate("xyz")
        ]
        cloud.point_step, cloud.row_step = 12, 12 * cloud.width
        cloud.data = b"".join(
            struct.pack("<fff", *probe["point"]) for probe in probes["cloud"]
        )
        cloud.is_dense = True
        received = []

        def publish_state() -> None:
            joints.header.stamp = node.get_clock().now().to_msg()
            state_pub.publish(joints)

        def publish() -> None:
            publish_state()
            cloud.header.stamp = joints.header.stamp
            cloud_pub.publish(cloud)

        timer = node.create_timer(0.1, publish)
        subscription = node.create_subscription(
            PointCloud2,
            "scene_cloud_filtered",
            received.append,
            qos_profile_sensor_data,
        )
        deadline = time.monotonic() + 90
        while len(received) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not received:
            raise RuntimeError("No self-filtered point cloud received")
        filtered = received[-1]
        offsets = {field.name: field.offset for field in filtered.fields}
        actual = {
            tuple(
                struct.unpack_from(
                    "<f",
                    filtered.data,
                    row * filtered.row_step + col * filtered.point_step + offsets[axis],
                )[0]
                for axis in "xyz"
            )
            for row in range(filtered.height)
            for col in range(filtered.width)
        }
        failures = []
        for probe in probes["cloud"]:
            point = struct.unpack("<fff", struct.pack("<fff", *probe["point"]))
            if (point in actual) != probe["keep"]:
                failures.append(probe["name"])
        # Stop cloud updates before contact tests; joint state publication continues.
        node.destroy_timer(timer)
        timer = node.create_timer(0.1, publish_state)
        apply = node.create_client(ApplyPlanningScene, "apply_planning_scene")
        validity = node.create_client(GetStateValidity, "check_state_validity")

        def call(client, request):
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError("Inspection service missing")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=20)
            if not future.done() or future.exception():
                raise RuntimeError("Inspection service failed")
            return future.result()

        contacts = []
        for probe in probes["contacts"]:
            obstacle = CollisionObject()
            obstacle.header.frame_id = "base_link"
            obstacle.id = "lower_body_inspection_probe"
            obstacle.operation = CollisionObject.ADD
            shape = SolidPrimitive(
                type=SolidPrimitive.BOX,
                dimensions=[float(value) for value in probe["size"]],
            )
            pose = Pose()
            pose.orientation.w = 1.0
            pose.position.x, pose.position.y, pose.position.z = map(
                float, probe["point"]
            )
            obstacle.primitives = [shape]
            obstacle.primitive_poses = [pose]
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            scene.world.collision_objects = [obstacle]
            if not call(apply, ApplyPlanningScene.Request(scene=scene)).success:
                raise RuntimeError("Test obstacle was rejected")
            request = GetStateValidity.Request()
            request.robot_state.joint_state = joints
            request.robot_state.is_diff = True
            response = call(validity, request)
            # Existing CAD self contacts are unrelated to this world-obstacle test.
            hit = any(
                obstacle.id in (contact.contact_body_1, contact.contact_body_2)
                for contact in response.contacts
            )
            contacts.append(
                {
                    "name": probe["name"],
                    "collision": hit,
                    "expected": probe["collision"],
                }
            )
        obstacle.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [obstacle]
        call(apply, ApplyPlanningScene.Request(scene=scene))
        result = {
            "cloud_probes": len(probes["cloud"]),
            "expected_filtered_out": sum(
                not probe["keep"] for probe in probes["cloud"]
            ),
            "output_points": len(actual),
            "filter_failures": failures,
            "contact_probes": contacts,
        }
        print(json.dumps(result), flush=True)
        if failures or any(
            check["collision"] != check["expected"] for check in contacts
        ):
            raise RuntimeError("Lower-body collision/filter regression")
        node.destroy_timer(timer)
        node.destroy_subscription(subscription)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
