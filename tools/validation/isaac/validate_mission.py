"""Offer a Mission through the public service and retain state/result evidence."""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sqlite3
import time
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosidl_runtime_py.convert import message_to_ordereddict
from cleany_interfaces.msg import MissionResult, MissionStatus, RuntimeDebugSnapshot
from cleany_interfaces.srv import OfferMission, CancelMission, VerifyPlacement
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from cadence import StampStats, publication_config, confirmation_window
from ros_scope import scoped_ros_args


def validate(output: Path, scenario: str, require_no_recovery: bool = False) -> dict:
    scene = json.loads((output / "config/room.json").read_text())
    physical = bool(scene.get('cleanup')) and scene.get('cleanup_backend', 'physical') == 'physical'
    rclpy.init(args=scoped_ros_args())
    node = Node("isaac_mission_validation")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    seen = {"status": None, "result": None, "odom": None, "debug": None}
    events = []
    odometry = deque(maxlen=2000)
    parking = {"max_error_rad": 0., "violation_since": None, "samples": 0, "latest_errors": {}}
    park = scene.get("park_joints", {})
    recoveries = {}
    timing = {name: StampStats() for name in ('clock', 'odometry', 'tf', 'scan', 'joint_states')}

    def observe(name, stamp):
        timing[name].observe(stamp.sec, stamp.nanosec)

    def transforms(message):
        for transform in message.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                observe('tf', transform.header.stamp)

    node.create_subscription(Clock, '/clock', lambda msg: observe('clock', msg.clock), 100)
    node.create_subscription(LaserScan, '/scan', lambda msg: observe('scan', msg.header.stamp), 100)
    node.create_subscription(TFMessage, '/tf', transforms, 100)

    def navigation_feedback(message):
        goal = bytes(message.goal_id.uuid).hex()
        recoveries[goal] = max(recoveries.get(goal, 0), message.feedback.number_of_recoveries)

    node.create_subscription(NavigateToPose.Impl.FeedbackMessage,
        "/navigate_to_pose/_action/feedback", navigation_feedback, 10)

    def joints(message):
        observe('joint_states', message.header.stamp)
        values = dict(zip(message.name, message.position))
        if not park:
            return
        if physical and seen["status"] is not None and seen["status"].state not in (
                "NAVIGATE_TO_TARGET", "RETURN_HOME", "IDLE"):
            parking["violation_since"] = None
            return
        errors = {name: abs(values[name]-target) for name, target in park.items() if name in values}
        error = max(errors.values(), default=float("inf")) if len(errors) == len(park) else float("inf")
        parking.update(max_error_rad=max(parking["max_error_rad"], error),
                       latest_errors=errors, samples=parking["samples"]+1)
        if error > .12:
            parking["violation_since"] = parking["violation_since"] or time.monotonic()
        else:
            parking["violation_since"] = None

    node.create_subscription(JointState, "/joint_states", joints, 10)

    def receive(key):
        def callback(message):
            seen[key] = message
            if key == "status":
                events.append({"wall_time": time.time(), **message_to_ordereddict(message)})
            if key == "odom":
                observe('odometry', message.header.stamp)
                odometry.append((time.monotonic(), message))
        return callback

    for kind, topic, key, quality in (
            (MissionStatus, "/mission/status", "status", qos),
            (MissionResult, "/mission/result", "result", qos),
            (RuntimeDebugSnapshot, "/mission/debug_snapshot", "debug", qos),
            (Odometry, "/wheel/odom", "odom", 10)):
        node.create_subscription(kind, topic, receive(key), quality)
    offer = node.create_client(OfferMission, "/mission/offer")
    cancel = node.create_client(CancelMission, "/mission/cancel")
    block = node.create_publisher(Bool, "/isaac/test/block_base", 10)

    def wait(predicate, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if parking["violation_since"] and time.monotonic()-parking["violation_since"] > .5:
                raise RuntimeError(f"Parked arm/head posture not maintained: {parking}")
            if predicate():
                return
        raise TimeoutError(f"Checkpoint timeout; status={seen['status']}; debug={seen['debug']}")

    report = {"scenario": scenario, "passed": False, "world": scene["world"], "target_id": scene["target_id"],
              "scope": {"mission": "actual", "navigation": "actual Nav2/AMCL",
                        "base": "Isaac PhysX with ideal root velocity actuator", "lidar": "PhysX raycast",
                        "perception": "mock", "planning": "mock", "manipulation": "mock Action",
                        "physical_cleanup": False, "depth_safety": False}}
    report['cleanup_backend'] = 'physical' if physical else 'mock'
    if physical:
        report["scope"].update(perception="RTX RGB-D + YOLOE + local VLM", planning="policy + MoveIt",
                               manipulation="sensor-driven BT Action")
    try:
        wait(lambda: seen["status"] is not None and seen["status"].ready and offer.service_is_ready(), 120.)
        # A latched ready status can arrive before discovery of all sensor
        # publishers. Require actual input, then allow RPC response discovery.
        wait(lambda: all(stats.count > 0 for stats in timing.values()) and all(
            node.count_publishers(node.resolve_topic_name(topic)) >= 1
            for topic in ('/clock', '/wheel/odom', '/joint_states', '/scan')), 120.)
        discovery_settled = time.monotonic()+2.
        wait(lambda: time.monotonic() >= discovery_settled, 5.)
        report["clock_publishers"] = node.count_publishers(node.resolve_topic_name("/clock"))
        assert report["clock_publishers"] == 1
        report["publishers"] = {topic: node.count_publishers(node.resolve_topic_name(topic))
                                for topic in ("/wheel/odom", "/joint_states", "/scan")}
        assert all(count == 1 for count in report["publishers"].values()), report["publishers"]
        if scenario == "navigation_failure":
            wait(lambda: block.get_subscription_count() > 0, 10.)
            block.publish(Bool(data=True))
        mission_id = "isaac-"+uuid4().hex
        future = offer.call_async(OfferMission.Request(mission_id=mission_id, target_id=scene["target_id"],
            requested_by="isaac-integration", mission_type="clean_desk", target_kind="SEAT"))
        wait(future.done, 10.)
        report["offer"] = message_to_ordereddict(future.result())
        assert future.result().accepted, future.result().reason
        if scenario == "cancel":
            wait(lambda: seen["status"].state == "NAVIGATE_TO_TARGET"
                 and seen["odom"] is not None
                 and math.hypot(seen["odom"].twist.twist.linear.x,
                                seen["odom"].twist.twist.linear.y) > .03,
                 scene.get("navigation_start_timeout_s", 30.))
            report["odom_at_cancel"] = message_to_ordereddict(seen["odom"])
            future = cancel.call_async(CancelMission.Request(mission_id=mission_id))
            wait(future.done, 10.)
            report["cancel"] = message_to_ordereddict(future.result())
            assert future.result().accepted, report["cancel"]
        wait(lambda: seen["result"] is not None and seen["result"].mission_id == mission_id, scene.get("validation_timeout_s", 300.))
        result = json.loads(seen["result"].report_json)
        report["mission_result"] = result
        if scenario == "success":
            assert result["outcome"] == "SUCCESS", result
            assert result["return_result"] == "OK", result
            if physical:
                assert len(result["completed_tasks"]) == 1 and len(result["actions"]) == 1, result
                action = result["actions"][0]["manipulation"]
                assert action["execution_profile"] == "isaac" and action["placement_state"] == "CONFIRMED", result
                assert action["object_state"] == "LEFT_GRIPPER" and action["stop_confirmed"] and action["arm_recovered"], result
                assert result["before_observation"].startswith("perception://snapshots/"), result
                assert result["after_observation"] != result["before_observation"], result
            else:
                assert result["completed_tasks"] == ["trash-1", "trash-2"], result
            assert any(event["state"] == "RETURN_HOME" for event in events), events
            wait(lambda: seen["status"].ready, 10.)
        elif scenario == "navigation_failure":
            assert result["outcome"] == "FAILED" and result["failure_code"] == "TIMEOUT", result
            assert not result["actions"] and not result["completed_tasks"], result
            assert not any(event["state"] == "WORKING" for event in events), events
        elif scenario == "manipulation_failure":
            assert result["outcome"] == "FAILED" and result["failure_code"] == "GRASP_FAIL", result
            assert result["actions"] and not result["completed_tasks"], result
            assert all(action["manipulation"]["stop_confirmed"] for action in result["actions"]), result
        else:
            assert result["outcome"] == "CANCELLED", result
        assert result["execution_profile"] == ({"navigation": "sim", "perception": "sensor", "planning": "policy", "execution": "isaac"}
            if physical else {"navigation": "sim", "perception": "mock", "planning": "mock", "execution": "mock"})
        result_time = time.monotonic()

        def stopped():
            fresh = confirmation_window([(received, msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, msg)
                                         for received, msg in odometry if received > result_time], time.monotonic())
            return bool(fresh) and all(
                math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y) < .03
                and abs(msg.twist.twist.angular.z) < .05 for _, _, msg in fresh)

        wait(stopped, 60.)
        expected_state = "ERROR" if scenario == "manipulation_failure" else "IDLE"
        wait(lambda: seen["status"].state == expected_state
             and seen["status"].phase == "TERMINAL", 10.)
        assert seen["status"].ready == (expected_state == "IDLE"), seen["status"]
        if scenario == "manipulation_failure":
            assert result["needs_human_review"] and result["return_result"] == "NOT_STARTED", result
            assert all(not action["manipulation"]["arm_recovered"] for action in result["actions"]), result
        report["final_odom"] = message_to_ordereddict(seen["odom"])
        if scenario == "success":
            point = seen["odom"].pose.pose.position
            report["home_error_m"] = math.hypot(point.x-scene["home"][0], point.y-scene["home"][1])
            assert report["home_error_m"] < .08, report["final_odom"]
            if physical:
                verifier = node.create_client(VerifyPlacement, '/sorting/verify_placement')
                wait(verifier.service_is_ready, 5.)
                stamp = seen['odom'].header.stamp
                barrier = stamp.sec*10**9+stamp.nanosec
                # This fixture has one registered body. The legacy unique-label
                # oracle is an extra post-home check, never a planning input.
                label = ('cup' if scene['cleanup']['payload']['visual_type'] == 'paper_cup'
                         else 'crumpled tissue')
                request = VerifyPlacement.Request(label=label,
                    destination_id=result['actions'][0]['destination_id'], after_stamp_ns=barrier)
                deadline = time.monotonic()+15.
                report['post_home_placement'] = {'success': False, 'after_stamp_ns': barrier}
                while time.monotonic() < deadline:
                    future = verifier.call_async(request)
                    wait(future.done, 2.)
                    response = future.result()
                    report['post_home_placement'].update(message_to_ordereddict(response))
                    if response.success:
                        break
                    rclpy.spin_once(node, timeout_sec=.1)
                assert report['post_home_placement']['success'], report['post_home_placement']
        report["database_integrity"] = {}
        if park:
            assert parking["samples"] >= 10
        for filename in ("missions.db", "executions.db"):
            with sqlite3.connect(f"file:{output/filename}?mode=ro", uri=True) as database:
                integrity = database.execute("PRAGMA integrity_check").fetchone()[0]
            report["database_integrity"][filename] = integrity
            assert integrity == "ok", report["database_integrity"]
        if require_no_recovery:
            assert len(recoveries) >= 2, 'Expected feedback for outbound and home navigation'
            assert not any(recoveries.values()), recoveries
        rates = publication_config(scene.get('publication_profile', 'legacy'), scene['physics_hz'])
        expected = {'clock': rates['clock_hz'], 'odometry': rates['odometry_hz'],
                    'tf': rates['odometry_hz'], 'scan': rates['scan_hz']}
        for name, rate in expected.items():
            actual = timing[name].result()
            assert actual['count'] > 10 and actual['nonincreasing_stamps'] == 0, (name, actual)
            assert .95*rate <= actual['received_hz_sim'] <= 1.02*rate, (name, rate, actual)
        report["passed"] = True
        report["scope"]["physical_cleanup"] = physical and scenario == "success"
    except Exception as error:
        report["error"] = repr(error)
        raise
    finally:
        report["states"] = events
        report["navigation_recoveries"] = recoveries
        report["parking"] = parking
        report['publication_timing'] = {name: value.result() for name, value in timing.items()}
        report["final_status"] = message_to_ordereddict(seen["status"]) if seen["status"] else None
        (output / "evaluations/mission.json").write_text(json.dumps(report, indent=2))
        if rclpy.ok(context=node.context):
            block.publish(Bool(data=False))
        node.destroy_node()
        rclpy.try_shutdown()
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/output"))
    parser.add_argument("--scenario", choices=["success", "navigation_failure", "manipulation_failure", "cancel"], default="success")
    parser.add_argument("--require-no-recovery", action="store_true")
    args = parser.parse_args()
    print(json.dumps(validate(args.output, args.scenario, args.require_no_recovery), indent=2))
