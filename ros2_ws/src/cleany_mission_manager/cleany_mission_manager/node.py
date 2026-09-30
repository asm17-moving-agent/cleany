"""ROS shell for the FSM: short services and a steady-clock nonblocking tick."""

import json
import time
from math import isfinite

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cleany_interfaces.msg import MissionResult, MissionStatus
from cleany_interfaces.srv import CancelMission, GetRuntimeSnapshot, OfferMission
from cleany_mission_manager.adapters.config import load_targets
from cleany_mission_manager.adapters.nav2 import Nav2Navigator
from cleany_mission_manager.core.journal import MissionJournal
from cleany_mission_manager.core.operations import DeferredPort
from cleany_mission_manager.core.result import FailureCode, ModuleResult
from cleany_mission_manager.core.runtime import MissionRuntime
from cleany_mission_manager.core.runtime_models import RuntimePolicy, RuntimeRequest
from cleany_mission_manager.mocks.desk import MockDesk


class MissionRuntimeNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("mission_runtime", **kwargs)
        defaults = {
            "targets_file": "", "journal_path": "~/.local/state/cleany/missions.db",
            "navigation_backend": "nav2", "post_mission": "return_home",
            "tick_hz": 10.0, "navigation_timeout": 180.0, "cleaning_timeout": 180.0,
            "operation_timeout": 30.0, "cancel_timeout": 5.0,
            "max_actions": 30, "max_skill_retries": 2, "mock_operation_polls": 2,
            "odom_topic": "odom", "input_timeout": 1.5,
            "stopped_linear_speed": 0.03, "stopped_angular_speed": 0.05,
            "navigation_settle_timeout": 3.0,
            "lifecycle_timeout": 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.targets = load_targets(value("targets_file"))
        self.journal = MissionJournal(value("journal_path"))
        self.desk = MockDesk(time.monotonic, polls=value("mock_operation_polls"))
        backend = value("navigation_backend")
        if backend not in ("nav2", "mock"):
            raise ValueError("navigation_backend must be nav2 or mock")
        self.navigator = (Nav2Navigator(self, self.targets, self.journal) if backend == "nav2"
                          else DeferredPort(lambda _: ModuleResult.success(), polls=10))
        self.runtime = MissionRuntime(
            navigator=self.navigator, perception=self.desk.perception, planner=self.desk.planner,
            executor=self.desk.executor, clock=time.monotonic,
            supported_targets=set(self.targets.seats), journal=self.journal,
            on_accept=self.desk.begin, navigation_mode="sim" if backend == "nav2" else "mock",
            policy=RuntimePolicy(**{name: value(name) for name in (
                "post_mission", "navigation_timeout", "cleaning_timeout", "operation_timeout",
                "cancel_timeout", "max_actions", "max_skill_retries",
            )}),
        )
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(MissionStatus, "mission/status", qos)
        self.result_pub = self.create_publisher(MissionResult, "mission/result", qos)
        self.create_service(OfferMission, "mission/offer", self._offer)
        self.create_service(CancelMission, "mission/cancel", self._cancel)
        self.create_service(GetRuntimeSnapshot, "mission/snapshot", self._snapshot)
        self.create_service(Trigger, "mission/reset_error", self._reset)
        # Notification only: the backend producing this signal owns physical stop first.
        self.create_subscription(String, "robot/safety_fault", self._fault, 10)
        self.create_subscription(String, "robot/safety_released", self._released, 10)
        self.last_sequence = -1
        self.last_result = ""
        tick_hz = float(value("tick_hz"))
        if not isfinite(tick_hz) or tick_hz <= 0:
            raise ValueError("tick_hz must be positive")
        self.create_timer(1.0 / tick_hz, self._tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def _admission(response, admission):
        response.accepted = admission.accepted
        response.duplicate = admission.duplicate
        response.reason = admission.reason
        response.report_json = json.dumps(admission.report.to_dict()) if admission.report else ""
        return response

    def _offer(self, request, response):
        return self._admission(response, self.runtime.offer(RuntimeRequest(
            request.mission_id, request.target_id, request.requested_by,
            request.mission_type, request.target_kind,
        )))

    def _cancel(self, request, response):
        return self._admission(response, self.runtime.cancel(request.mission_id))

    def _snapshot(self, request, response):
        response.snapshot_json = json.dumps(self.runtime.snapshot())
        return response

    def _reset(self, request, response):
        result = self.runtime.reset_error()
        response.success, response.message = result.accepted, result.reason
        return response

    def _fault(self, message):
        if message.data not in ("E_STOP", "HARDWARE_ERROR"):
            self.get_logger().warning("Ignoring unknown safety fault code")
            return
        self.runtime.safety_fault(FailureCode(message.data))

    def _released(self, message):
        if message.data in ("E_STOP", "HARDWARE_ERROR"):
            result = self.runtime.safety_released(FailureCode(message.data))
            self.get_logger().info(f"Safety release: {result.accepted} {result.reason}")

    def _tick(self):
        if isinstance(self.navigator, Nav2Navigator):
            self.navigator.maintain()
        self.runtime.tick()
        snapshot = self.runtime.snapshot()
        if snapshot["sequence"] != self.last_sequence:
            request = snapshot["active_request"]
            self.status_pub.publish(MissionStatus(
                robot_id="cleany-01", mission_id=request["mission_id"] if request else "",
                state=snapshot["state"], phase=snapshot["phase"], bt_stage=snapshot["bt_stage"],
                ready=snapshot["ready"], reason=snapshot["reason"], sequence=snapshot["sequence"],
            ))
            self.last_sequence = snapshot["sequence"]
            self.get_logger().info(f"FSM {snapshot['state']} BT {snapshot['bt_stage']}")
        report = self.runtime.last_report
        if report and report.request.mission_id != self.last_result:
            self.result_pub.publish(MissionResult(
                mission_id=report.request.mission_id, outcome=report.outcome,
                failure_code=report.failure_code, summary=report.summary,
                completed_tasks=report.completed_tasks, skipped_tasks=report.skipped_tasks,
                needs_human_review=report.needs_human_review,
                before_observation=report.before_observation,
                after_observation=report.after_observation,
                navigation_result=report.navigation_result, return_result=report.return_result,
                cleaning_mode=report.cleaning_mode, report_json=json.dumps(report.to_dict()),
            ))
            self.last_result = report.request.mission_id

    def destroy_node(self):
        if self.runtime.request:
            self.runtime.cancel(self.runtime.request.mission_id)
        self.journal.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionRuntimeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
