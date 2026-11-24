"""Field adapter contracts with synthetic sensor inputs and no actuators."""

from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS
from uuid import uuid4
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.action import ActionServer
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cleany_interfaces.action import InspectScene

from cleany_mission_manager.adapters.physical_scene import PhysicalScenePort, scene_objects
from cleany_mission_manager.adapters.physical_desk import PhysicalDesk
from cleany_mission_manager.core.manipulation import map_outcome
from cleany_mission_manager.core.operations import DeferredPort
from cleany_mission_manager.core.result import ModuleResult
from cleany_mission_manager.core.runtime import MissionRuntime
from cleany_mission_manager.core.runtime_models import (
    RuntimeRequest,
    RuntimePolicy,
    SceneSnapshot,
    SceneObject,
)
from test_manipulation import outcome
from test_runtime_supervision import Lease


def test_jetson_evidence_requires_real_confirmed_bin_and_stowed_arm():
    collected = replace(outcome("one"), execution_profile="jetson")
    assert map_outcome(collected, "one", 4, expected_profile="jetson").ok
    for changes in (
        {"execution_profile": "mock"},
        {"placement_state": "ASSUMED"},
        {"arm_recovered": False},
    ):
        with pytest.raises(ValueError):
            map_outcome(replace(collected, **changes), "one", 4, expected_profile="jetson")


def test_only_classified_tracked_allowed_trash_is_selectable():
    data = dict(
        label="Crumpled Paper",
        object_id=2,
        track_id="a",
        tracking_state="TRACKED",
        distance_valid=True,
        distance_m=0.6,
        position_valid=True,
        sorting_category="trash",
    )

    def check(**overrides):
        item = NS(**(data | overrides))
        return scene_objects([item], {"crumpled paper"})[0].disposition

    assert check() == "collect_trash"
    for overrides in (
        dict(label="wallet"),
        dict(sorting_category="review"),
        dict(tracking_state="AMBIGUOUS"),
        dict(distance_valid=False),
        dict(distance_m=float("nan")),
        dict(position_valid=False),
    ):
        assert check(**overrides) == "human_review"


def test_inspection_matches_capture_stamp_and_frame_before_writing_jpeg(tmp_path):
    rclpy.init()
    node = rclpy.create_node("field_capture_fixture")
    image_topic = "test/field/image"
    publisher = node.create_publisher(Image, image_topic, qos_profile_sensor_data)
    mission = str(uuid4())
    received = []

    def inspect(handle):
        received.append(handle.request.tracking_session_id)
        header = node.get_clock().now().to_msg()
        response = InspectScene.Result(success=True)
        scene = response.detections
        scene.header.stamp, scene.header.frame_id = header, "rgb"
        scene.snapshot_id, scene.tracking_session_id, scene.tracking_epoch = "snap-1", mission, "epoch"
        image = Image(width=2, height=2, step=6, encoding="rgb8", data=[170] * 12)
        image.header.stamp, image.header.frame_id = header, "wrong"
        publisher.publish(image)
        handle.succeed()
        return response

    server = ActionServer(node, InspectScene, "test/field/inspect", execute_callback=inspect)
    try:
        port = PhysicalScenePort(node, "test/field/inspect", image_topic, tmp_path, ["paper"], 8.0)
        assert port.client.wait_for_server(timeout_sec=3)
        op = port.start(mission)
        end = time.monotonic() + 4
        while not (port.images and port.finished and port.finished.done()) and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            assert port.poll(op) is None
        assert port.images and received == [mission] and not list(tmp_path.rglob("*.jpg"))
        matched = deepcopy(port.images[-1])
        matched.header.frame_id = "rgb"
        publisher.publish(matched)
        result = None
        while not result and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            result = port.poll(op)
        assert result and result.ok and result.data.source == "real"
        jpeg = (tmp_path / result.data.reference.removeprefix("observation://")).read_bytes()
        assert jpeg.startswith(bytes.fromhex("ffd8")) and jpeg.endswith(bytes.fromhex("ffd9"))
    finally:
        server.destroy()
        node.destroy_node()
        rclpy.shutdown()


def test_two_separate_operator_requests_use_fresh_observations_and_return_home_each_time():
    class Navigation(DeferredPort):
        at_home = lambda self: self.home
        home = True

    nav = Navigation(lambda _: ModuleResult.success(), polls=0)
    desk = PhysicalDesk.__new__(PhysicalDesk)
    desk.destination = "demo_trash_bin"
    remaining = []

    def inspect(mid):
        snap = str(uuid4())
        return ModuleResult.success(
            SceneSnapshot(
                snap, time.monotonic(), tuple(remaining), f"observation://{mid}/{snap}.jpg", "real"
            )
        )

    def place(command):
        obj = remaining.pop(0)
        assert (
            command.wire_object_id == obj.wire_object_id and command.destination_id == desk.destination
        )
        validated = replace(outcome(command.execution_id), execution_profile="jetson")
        return map_outcome(validated, command.execution_id, 4, expected_profile="jetson")

    policy = RuntimePolicy(manipulation_destinations=(desk.destination,), max_skill_retries=0)
    arm = DeferredPort(place, polls=0)
    runtime = MissionRuntime(
        navigator=nav,
        planner=DeferredPort(desk.plan, polls=0),
        perception=DeferredPort(inspect, polls=0),
        executor=arm,
        supported_targets={"seat-02", "seat-05"},
        clock=time.monotonic,
        supervision=Lease(),
        policy=policy,
        navigation_mode="real",
    )
    assert runtime.offer(RuntimeRequest("invalid", "seat-02", "fixture")).reason == "INVALID_REQUEST"
    assert not nav.commands
    nav.home = False
    assert not runtime.readiness()[0] and not nav.commands
    nav.home = True
    for seat in ("seat-02", "seat-05"):
        mid = str(uuid4())
        remaining.extend([SceneObject("a", wire_object_id=7), SceneObject("b", wire_object_id=11)])
        assert runtime.offer(RuntimeRequest(mid, seat, "fixture-operator")).accepted
        for _ in range(80):
            runtime.tick()
            if runtime.last_report and runtime.last_report.request.mission_id == mid:
                break
        report = runtime.last_report
        assert report.outcome == "SUCCESS" and report.return_result == "OK"
        assert report.cleaning_mode == "real" and report.completed_tasks == ["a", "b"]
        assert report.before_observation != report.after_observation
    assert nav.commands == ["seat-02", "home", "seat-05", "home"]


def test_physical_arm_requires_matching_executor_stow_and_outcome_source(tmp_path):
    import json
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from cleany_interfaces.msg import ManipulationOutcome
    from cleany_mission_manager.adapters.physical_arm import PhysicalArmPort
    from cleany_mission_manager.core.journal import MissionJournal
    from test_manipulation import Transport

    calibration = tmp_path / "bin.json"
    arm_config = dict(home_joint_rad=[0.0] * 5, bins={"demo_trash_bin": [0.1] * 5})
    calibration.write_text(
        json.dumps(
            dict(
                operation_mode="full_collection",
                commissioning_verified=True,
                label_destinations={"crumpled paper": "demo_trash_bin"},
                arms=dict(left=arm_config, right=arm_config),
            )
        )
    )
    journal = MissionJournal()
    rclpy.init()
    node = rclpy.create_node("field_arm_fixture")
    try:
        arm = PhysicalArmPort(
            node, Transport(journal), journal, calibration, "demo_trash_bin", 0.05, 1.0
        )
        status = dict(
            schema=1,
            collection_config_sha256="other",
            operation_mode="full_collection",
            guard_fresh=True,
            guard={"state": "ready"},
            backend_ready=True,
            hardware_fault="",
            active_execution_id="",
            blocked=[],
            inhibited=False,
            external_motion_active=False,
            recovery_active=False,
            arm_return_active=False,
            payload_attached=False,
        )

        def control_state(_, response):
            response.success = True
            response.message = json.dumps(status)
            return response

        node.create_service(Trigger, "manipulation/control_state", control_state)
        joints = node.create_publisher(JointState, "joint_states", qos_profile_sensor_data)
        pose = JointState(name=list(arm.stow), position=list(arm.stow.values()))

        def wait_for(predicate):
            end = time.monotonic() + 4
            while time.monotonic() < end:
                pose.header.stamp = node.get_clock().now().to_msg()
                joints.publish(pose)
                rclpy.spin_once(node, timeout_sec=0.01)
                arm.maintain()
                if predicate():
                    return
            pytest.fail("timed out waiting for physical arm fixture")

        wait_for(lambda: arm.stowed and arm.state_at > 0)
        assert not arm.safe_to_drive()[0]
        status["collection_config_sha256"] = arm.digest
        wait_for(lambda: arm.safe_to_drive()[0])
        assert arm.ready() == (False, "PLACEMENT_EVIDENCE_UNAVAILABLE")
        node.create_publisher(ManipulationOutcome, "/perception/manipulation_outcome", 10)
        wait_for(lambda: arm.ready()[0])
        pose.position[0] = 0.3
        wait_for(lambda: not arm.stowed)
        assert not arm.ready()[0]
        pose.position[0] = 0.0
        status["external_motion_active"] = True
        wait_for(lambda: arm.stowed and not arm.idle)
        assert not arm.ready()[0]
    finally:
        node.destroy_node()
        rclpy.shutdown()
