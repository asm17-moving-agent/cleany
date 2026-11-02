"""Opt-in study-cafe Nav2 drive followed by two logical Mock Action collections."""

import json
import os
from pathlib import Path
import signal
import subprocess
import time
from uuid import uuid4

import pytest
import yaml

pytestmark = pytest.mark.skipif(os.environ.get("CLEANY_RUN_GAZEBO_MANIPULATION_TESTS") != "1",
    reason="requires Fortress, full Nav2 dependencies, integrated install and isolated ROS_DOMAIN_ID")


def test_gazebo_nav2_and_mock_action_without_gateway(tmp_path):
    import rclpy
    from cleany_interfaces.msg import MissionResult, MissionStatus, RuntimeDebugSnapshot
    from cleany_interfaces.srv import OfferMission
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile

    post_mission = os.environ.get("CLEANY_GAZEBO_POST_MISSION", "wait_for_next")
    root = Path(__file__).parents[1]
    # The saved map starts at the study-cafe spawn, rather than the SDF world origin.
    initial = yaml.safe_load((root.parent / "cleany_gazebo_sim/config/nav2_amcl.yaml").read_text())[
        "amcl"]["ros__parameters"]["initial_pose"]
    command = ["ros2", "launch", "cleany_bringup", "mission_sim.launch.py",
        "profile:=fortress", "headless:=true", "enable_bridge:=false", "enable_telemetry:=false",
        "start_manipulation_mock:=true", "post_mission:=" + post_mission,
        "runtime_config:=" + str(root / "config/runtime_gazebo_manipulation_mock.yaml"),
        "journal_path:=" + str(tmp_path / "missions.db"),
        "bridge_journal_path:=" + str(tmp_path / "bridge.db"),
        "manipulation_database_path:=" + str(tmp_path / "executions.db"),
        "sim_partition:=cleany-mission-mock-test-" + uuid4().hex]
    log_path = tmp_path / "launch.log"
    rclpy.init()
    observer = Node("mission_mock_smoke")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    seen = dict(status=None, result=None, debug=None, odom=None)
    def receive(key):
        return lambda message: seen.update({key: message})
    observer.create_subscription(MissionStatus, "mission/status", receive("status"), qos)
    observer.create_subscription(MissionResult, "mission/result", receive("result"), qos)
    observer.create_subscription(RuntimeDebugSnapshot, "mission/debug_snapshot", receive("debug"), qos)
    observer.create_subscription(Odometry, "ground_truth/odom", receive("odom"), 10)
    initial_pose = observer.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
    offer = observer.create_client(OfferMission, "mission/offer")

    def wait(predicate, process, timeout):
        deadline = time.monotonic() + timeout
        seeded_at = float("-inf")
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()[-12000:]
            rclpy.spin_once(observer, timeout_sec=.05)
            if predicate():
                return
            if (not seen["status"] or not seen["status"].ready) and seen["odom"] is not None:
                if (seen["status"] is None or seen["status"].state == "IDLE") and time.monotonic() - seeded_at > 2:
                    if initial_pose.get_subscription_count():
                        message = PoseWithCovarianceStamped()
                        message.header.frame_id = "map"
                        message.header.stamp = seen["odom"].header.stamp
                        message.pose.pose.position.x = initial["x"]
                        message.pose.pose.position.y = initial["y"]
                        from math import cos, sin
                        message.pose.pose.orientation.z = sin(initial["yaw"] / 2)
                        message.pose.pose.orientation.w = cos(initial["yaw"] / 2)
                        message.pose.covariance[0] = message.pose.covariance[7] = .01
                        message.pose.covariance[35] = .01
                        initial_pose.publish(message)
                        seeded_at = time.monotonic()
        pytest.fail(f"Gazebo checkpoint timeout; status={seen['status']}; log={log_path}\n"
                    + log_path.read_text()[-12000:])

    with log_path.open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait(lambda: seen["status"] is not None and seen["status"].ready and offer.service_is_ready(),
                 process, 90)
            assert observer.count_publishers("/clock") == 1
            graph = observer.get_node_names_and_namespaces()
            assert ("manipulation_server", "/mock") in graph
            assert not any("mujoco" in name.lower() for name, _ in graph)
            publishers = observer.get_publisher_names_and_types_by_node("manipulation_server", "/mock")
            forbidden = {"/clock", "/tf", "/tf_static", "/odom", "/joint_states"}
            assert not any(name in forbidden or "camera" in name for name, _ in publishers)
            mission_id = "gazebo-mock-" + uuid4().hex
            future = offer.call_async(OfferMission.Request(mission_id=mission_id, target_id="seat-12",
                requested_by="integration-test", mission_type="clean_desk", target_kind="SEAT"))
            wait(future.done, process, 10)
            assert future.result().accepted, future.result().reason
            wait(lambda: seen["result"] is not None and seen["result"].mission_id == mission_id, process, 240)
            report = json.loads(seen["result"].report_json)
            assert report["outcome"] == "SUCCESS", report
            assert report["completed_tasks"] == ["trash-1", "trash-2"]
            assert report["execution_profile"] == {"navigation": "sim", "perception": "mock",
                                                    "planning": "mock", "execution": "mock"}
            assert len({action["execution_id"] for action in report["actions"]}) == 2
            for action in report["actions"]:
                outcome = action["manipulation"]
                assert outcome["status"] == "SUCCESS" and outcome["placement_state"] == "CONFIRMED"
                assert outcome["stop_confirmed"] and outcome["arm_recovered"]
                assert "failed_substage" not in outcome and "cancel_mode" not in outcome
            assert report["after_observation"].endswith("mock-snapshot-004")
            assert report["return_result"] == ("OK" if post_mission == "return_home" else "NOT_REQUESTED")
            wait(lambda: seen["status"].ready, process, 10)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            observer.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
