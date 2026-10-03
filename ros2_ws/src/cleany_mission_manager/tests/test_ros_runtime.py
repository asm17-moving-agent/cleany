"""Opt-in real ROS service/action transport checks; no physical actuators."""

import os
import time
from pathlib import Path
from uuid import uuid4

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("CLEANY_RUN_ROS_TESTS") != "1",
                                reason="set CLEANY_RUN_ROS_TESTS=1 with an isolated ROS_DOMAIN_ID")


@pytest.fixture
def ros_runtime(tmp_path, request):
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.task import Future
    from tf2_ros import TransformBroadcaster
    from cleany_mission_manager.node import MissionRuntimeNode

    class NavigationFixture(Node):
        def __init__(self):
            super().__init__("navigation_fixture")
            self.goals = []
            self.running = []
            self.hold = False
            self.publish_feedback = True
            group = ReentrantCallbackGroup()
            self.server = ActionServer(self, NavigateToPose, "navigate_to_pose", self.execute,
                                       cancel_callback=lambda _: CancelResponse.ACCEPT,
                                       callback_group=group)
            self.lifecycle_services = [self.create_service(GetState, name + "/get_state", self.state)
                             for name in ("map_server", "amcl", "planner_server",
                                          "controller_server", "bt_navigator")]
            self.odom = self.create_publisher(Odometry, "wheel/odom", 10)
            self.tf = TransformBroadcaster(self)
            self.timer = self.create_timer(.02, self.update, callback_group=group)

        def state(self, _, response):
            response.current_state = State(id=3, label="active")
            return response

        async def execute(self, handle):
            self.goals.append(handle.request.pose)
            future = Future()
            self.running.append((handle, future, time.monotonic()))
            return await future

        def update(self):
            if self.publish_feedback:
                message = Odometry()
                message.header.stamp = self.get_clock().now().to_msg()
                self.odom.publish(message)
                transform = TransformStamped()
                transform.header.stamp = message.header.stamp
                transform.header.frame_id = "map"
                transform.child_frame_id = "base_link"
                transform.transform.rotation.w = 1.0
                self.tf.sendTransform(transform)
            for handle, future, started in list(self.running):
                if handle.is_cancel_requested:
                    handle.canceled()
                elif not self.hold and time.monotonic() - started > .1:
                    handle.succeed()
                else:
                    continue
                future.set_result(NavigateToPose.Result())
                self.running.remove((handle, future, started))

    rclpy.init()
    request.addfinalizer(lambda: rclpy.shutdown() if rclpy.ok() else None)
    targets = Path(__file__).parents[1] / "config/study_cafe_targets.yaml"
    runtime = MissionRuntimeNode(parameter_overrides=[
        Parameter("targets_file", value=str(targets)),
        Parameter("journal_path", value=str(tmp_path / "missions.db")),
        Parameter("odom_topic", value="wheel/odom"),
        Parameter("use_sim_time", value=False),
        Parameter("post_mission", value="return_home"),
        Parameter("input_timeout", value=.3),
        Parameter("tick_hz", value=50.0),
        Parameter("navigation_timeout", value=5.0),
        Parameter("mock_operation_polls", value=1),
    ])
    fixture = NavigationFixture()
    executor = SingleThreadedExecutor()
    executor.add_node(runtime)
    executor.add_node(fixture)

    def spin_until(check, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if check():
                return
        pytest.fail(f"ROS checkpoint timeout: {runtime.runtime.snapshot()}")

    spin_until(lambda: runtime.runtime.readiness()[0])
    yield runtime, fixture, spin_until
    runtime.destroy_node()
    fixture.server.destroy()
    fixture.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


def test_nav2_transport_offer_result_and_duplicate(ros_runtime):
    from cleany_interfaces.srv import GetRuntimeSnapshot, OfferMission
    runtime, nav, spin = ros_runtime
    client = nav.create_client(OfferMission, "mission/offer")
    mid = str(uuid4())
    request = OfferMission.Request(mission_id=mid, mission_type="clean_desk", target_kind="SEAT",
                                   target_id="seat-12", requested_by="integration-test")
    spin(lambda: client.service_is_ready())
    future = client.call_async(request)
    spin(future.done)
    assert future.result().accepted
    spin(lambda: runtime.runtime.last_report is not None)
    assert runtime.runtime.last_report.outcome == "SUCCESS"
    assert runtime.runtime.last_report.navigation_result == "OK"
    assert runtime.runtime.last_report.completed_tasks == ["trash-1", "trash-2"]
    assert len(nav.goals) == 2
    assert nav.goals[0].header.frame_id == "map"
    assert nav.goals[0].pose.position.x == 6.3
    assert nav.goals[1].pose.position.x == 3.0
    assert nav.goals[1].pose.position.y == -1.2
    future = client.call_async(request)
    spin(future.done)
    assert future.result().duplicate
    assert len(nav.goals) == 2
    snapshot = nav.create_client(GetRuntimeSnapshot, "mission/snapshot")
    spin(lambda: snapshot.service_is_ready())
    future = snapshot.call_async(GetRuntimeSnapshot.Request())
    spin(future.done)
    assert '"navigation": "sim"' in future.result().snapshot_json


def test_cancel_waits_for_actual_nav2_result_and_stale_feedback_rejects_offer(ros_runtime):
    from cleany_mission_manager.core.runtime_models import RuntimeRequest
    from cleany_mission_manager.core.runtime_models import RuntimePolicy
    runtime, nav, spin = ros_runtime
    runtime.runtime.policy = RuntimePolicy(post_mission="wait_for_next")
    nav.hold = True
    mid = str(uuid4())
    assert runtime.runtime.offer(RuntimeRequest(mid, "seat-12", "integration-test")).accepted
    spin(lambda: len(nav.goals) == 1)
    assert runtime.runtime.cancel(mid).accepted
    assert runtime.runtime.last_report is None
    spin(lambda: runtime.runtime.last_report is not None)
    assert runtime.runtime.last_report.outcome == "CANCELLED"
    assert not nav.running
    nav.publish_feedback = False
    spin(lambda: not runtime.runtime.readiness()[0])
    assert runtime.runtime.offer(RuntimeRequest(str(uuid4()), "seat-12", "operator")).reason in (
        "ODOMETRY_STALE", "LOCALIZATION_STALE",
    )


def test_lost_action_result_cannot_prove_stopped_or_release_next_mission(ros_runtime):
    from rclpy.task import Future
    from cleany_mission_manager.core.runtime_models import RuntimeRequest, RuntimeState
    runtime, nav, spin = ros_runtime
    nav.hold = True
    mid = str(uuid4())
    assert runtime.runtime.offer(RuntimeRequest(mid, "seat-12", "integration-test")).accepted
    spin(lambda: bool(nav.goals) and runtime.navigator.operation.handle is not None)
    # The action is still executing even though the fixture publishes zero velocity.
    lost_reply = Future()
    lost_reply.set_exception(RuntimeError("GetResult transport lost"))
    runtime.navigator._finished(runtime.navigator.operation, lost_reply)
    assert not runtime.navigator.stopped()
    assert not runtime.runtime.offer(RuntimeRequest(str(uuid4()), "seat-13", "operator")).accepted
    spin(lambda: runtime.runtime.last_report is not None)
    assert runtime.runtime.state == RuntimeState.ERROR
    assert runtime.runtime.last_report.outcome == "FAILED"
    assert runtime.runtime.last_report.needs_human_review
    assert not nav.running
