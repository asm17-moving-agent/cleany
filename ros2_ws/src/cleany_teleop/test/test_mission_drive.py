"""Exercise the field action over real ROS transport and a synthetic MCU."""

import hashlib
import json
import threading
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from cleany_interfaces.action import DriveMissionTarget
from cleany_interfaces.msg import SystemState, MissionPermission
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer
from rclpy.parameter import Parameter

from cleany_teleop import navigation_bridge as module
from cleany_teleop.map_view import MapView
from test_navigation_ros import Control, Authority


@pytest.mark.parametrize("revoke", [False, True])
def test_parent_lease_named_target_and_measured_stop(tmp_path, monkeypatch, revoke):
    class Parent(Authority):
        mission_id = ""
        heartbeat_timeout = 1.0

        def _alive(self):
            return True

        def acquire(self, _):
            pytest.fail("drive server acquired a second lease")

    monkeypatch.setattr(module, "RosSupervision", Parent)
    grid = MapView(20, 20, 0.1, 0.0, 0.0, 0.0, [0] * 400)
    target = dict(
        schema_version=2,
        map_id="fixture",
        frame_id="map",
        source_sha256="a" * 64,
        layout_revision=1,
        validation_status="field_verified",
        home=[0.2, 0.2, 0.0],
        seats={"seat-02": [0.8, 0.8, 0.0]},
    )
    digest = hashlib.sha256(
        json.dumps(target, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    target["target_digest"] = digest
    path = tmp_path / "fixture.json"
    path.write_text(json.dumps(target))
    rclpy.init()
    node = rclpy.create_node(
        "synthetic_field_owner",
        parameter_overrides=[
            Parameter("navigation.mission_targets_file", value=str(path)),
            Parameter("navigation.mission_grid_revision", value=grid.revision),
        ],
    )
    peer = rclpy.create_node("synthetic_field_peer")
    calls = []

    def navigate(handle):
        calls.append(handle.request.pose)
        handle.succeed()
        return NavigateToPose.Result()

    nav = ActionServer(peer, NavigateToPose, "navigate_to_pose", execute_callback=navigate)
    cli = ActionClient(peer, DriveMissionTarget, "navigation/drive_mission_target")
    try:
        control = Control()
        b = module.NavigationBridge(node, control, threading.RLock())
        b.map, b.map_revision, b.pose = grid, grid.revision, [0.8, 0.8, 0.0]
        b.hardware_ready = lambda: (True, "")
        b._update_pose = lambda now: None
        permission = MissionPermission(
            mission_id="job",
            lease_id="lease",
            version=3,
            capabilities=["MOBILE_MANIPULATION"],
            granted=True,
            valid_for_s=1.0,
        )
        state = SystemState(epoch="synthetic", operating_state="OPERATIONAL", permissions=[permission])

        def spin():
            at = time.monotonic()
            b.pose_at = b.scan_at = b.odom_at = control.clock.received_at = at
            state.header.stamp = node.get_clock().now().to_msg()
            b.mission_drive.update(state)
            rclpy.spin_once(peer, timeout_sec=0.005)
            rclpy.spin_once(node, timeout_sec=0.005)
            b.tick()

        def wait(future):
            end = time.monotonic() + 4
            while not future.done() and time.monotonic() < end:
                spin()
            assert future.done()
            return future.result()

        assert cli.wait_for_server(timeout_sec=2)
        goal = DriveMissionTarget.Goal(
            target_id="seat-02",
            target_digest=digest,
            mission_id="job",
            lease_id="wrong",
            lease_version=3,
            lease_epoch="synthetic",
        )
        assert not wait(cli.send_goal_async(goal)).accepted and not control.requests
        goal.lease_id = "lease"
        handle = wait(cli.send_goal_async(goal))
        assert handle.accepted
        result = handle.get_result_async()
        if revoke:
            state.permissions[0].granted = False
        else:
            control.state.velocity_rad_s = [0.3] * 4
            end = time.monotonic() + 3
            while b.phase != "stopping" and time.monotonic() < end:
                spin()
            for _ in range(4):
                spin()
            assert b.phase == "stopping" and not result.done()
            control.state.velocity_rad_s = [0.0] * 4
        reply = wait(result)
        assert reply.result.stop_confirmed
        assert reply.result.arrived is (not revoke)
        assert reply.status == (GoalStatus.STATUS_ABORTED if revoke else GoalStatus.STATUS_SUCCEEDED)
        if not revoke:
            assert len(calls) == 1 and calls[0].pose.position.x == 0.8
    finally:
        nav.destroy()
        peer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_web_input_cannot_impersonate_the_internal_navigation_owner():
    from cleany_teleop.web_control import Control

    controller = Control(lambda _: None)
    controller.operator_blocked = lambda: True
    for sequence, action in enumerate(("press", "update", "heartbeat")):
        with pytest.raises(ValueError, match="Mission"):
            controller.request(
                dict(client="navigation-test", sequence=sequence, action=action, navigation_input=True)
            )
    controller.request(dict(client="operator", sequence=1, action="stop"))
    assert controller.owner is None
