"""짧은 서비스와 steady-clock 비블로킹 tick으로 FSM을 연결하는 ROS 외피입니다."""

import json
from dataclasses import asdict
import time
from math import isfinite
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cleany_interfaces.action import StopMission

from cleany_interfaces.msg import (MissionResult, MissionStatus, DebugBTNode,
                                    RuntimeDebugSnapshot, RuntimeEvent)
from cleany_interfaces.srv import CancelMission, GetRuntimeSnapshot, OfferMission
from cleany_mission_manager.adapters.config import load_targets, validate_manipulation_fixture
from cleany_mission_manager.adapters.nav2 import Nav2Navigator
from cleany_mission_manager.adapters.desk_approach import DeskApproachPort
from cleany_mission_manager.core.staged_navigation import StagedNavigator
from cleany_mission_manager.adapters.manipulation import ROSManipulationTransport
from cleany_mission_manager.adapters.supervision import RosSupervision
from cleany_mission_manager.core.manipulation import ManipulationPort, validate_backend
from cleany_mission_manager.core.journal import MissionJournal
from cleany_mission_manager.core.operations import DeferredPort
from cleany_mission_manager.core.result import FailureCode, ModuleResult
from cleany_mission_manager.core.runtime import MissionRuntime
from cleany_mission_manager.core.runtime_models import RuntimePolicy, RuntimeRequest, SceneObject
from cleany_mission_manager.mocks.desk import MockDesk


class MissionRuntimeNode(Node):
    """journal 기반 임무 상태 머신을 ROS에 연결합니다.

    Subscriptions:
        robot/safety_fault, robot/safety_released (std_msgs/msg/String):
            안전 소유자의 E_STOP 또는 HARDWARE_ERROR 알림. supervision_mode=disabled일
            때만 사용하며 감독 모드에서는 Supervisor가 관리합니다.
        odom (nav_msgs/msg/Odometry): Nav2 백엔드의 정지 근거. odom_topic 기본값입니다.
        navigate_to_pose/_action/status (action_msgs/msg/GoalStatusArray):
            Nav2 모드에서 목표 상태를 대조합니다.

    Publishers:
        mission/status (cleany_interfaces/msg/MissionStatus): 변경된 상태.
        mission/result (cleany_interfaces/msg/MissionResult): 새 종료 결과.
        mission/debug_snapshot (cleany_interfaces/msg/RuntimeDebugSnapshot):
            최대 초당 두 번 추출하는 크기 제한 진단 상태.
        mission/runtime_events (cleany_interfaces/msg/RuntimeEvent): 순서가 있는 이벤트.

    Services:
        mission/offer (cleany_interfaces/srv/OfferMission): 요청 수락 또는 재전달.
        mission/cancel (cleany_interfaces/srv/CancelMission): 취소 요청.
        mission/snapshot (cleany_interfaces/srv/GetRuntimeSnapshot): 상태 조회.
        mission/reset_error (std_srvs/srv/Trigger): 보호 조건을 적용한 복구 요청.
        Nav2 모드에서는 Nav2Navigator로 lifecycle GetState 서비스를 조회합니다.
        action_mock 모드에서는 변경 가능한 기본값 mock/manipulation/get_execution
            (cleany_interfaces/srv/GetManipulationExecution)을 조회합니다.

    Actions:
        mission/stop (cleany_interfaces/action/StopMission): 감독 모드에서 Supervisor
            정지 요청을 받는 Server. 이 Runtime이 자체 작업의 정지를 확인한 경우에만
            stopped=True입니다. stop_action 기본값입니다.
        navigate_to_pose (nav2_msgs/action/NavigateToPose): Nav2 모드의 Client.
        mock/manipulation/execute_skill
            (cleany_interfaces/action/ExecuteManipulationSkill): action_mock 모드의
            변경 가능한 Client 기본값. 기본 책상 백엔드는 프로세스 내부 mock입니다.

    TF:
        Nav2Navigator가 위치 추정 변환을 읽으며 이 노드는 발행하지 않습니다.

    QoS:
        상태, 결과와 debug는 reliable, transient_local, depth 1이며 런타임 이벤트는
        depth 2048을 유지합니다. 주행 odometry는 sensor_data를 사용합니다.

    Notes:
        상태 전이는 steady-clock tick에서 진행합니다. 안전 알림은 발행 측이 이미
        하드웨어를 정지했다는 전제이며 정지 구동기가 아닙니다.
        기본값 supervision_mode=required는 RosSupervision을 추가합니다. 수락에는
        Supervisor 가용성, 이동에는 임대가 필요하며 Nav2 lifecycle 검사를 Supervisor로
        옮깁니다. disabled는 mock과 시뮬레이션용 독립 동작을 유지하며 각 보고서의
        실행 프로파일에 기록합니다.
    """
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
            "desk_approach_enabled": False, "desk_distance_m": 0.75,
            "arrival_position_tolerance": 0.08, "arrival_yaw_tolerance": 0.15,
            "arrival_pose_timeout": 0.5,
            "lifecycle_timeout": 3.0,
            "required_navigation_lifecycle_nodes": [
                "map_server", "amcl", "planner_server", "controller_server",
                "bt_navigator", "collision_monitor",
            ],
            "manipulation_backend": "mock", "manipulation_action": "mock/manipulation/execute_skill",
            "manipulation_query": "mock/manipulation/get_execution",
            "manipulation_initial_mock_safe": False,
            "manipulation_destination": "mock_trash_bin",
            "manipulation_mock_fixture": "",
            "manipulation_mock_snapshot_ids": ["mock-snapshot-001", "mock-snapshot-002",
                                               "mock-snapshot-003", "mock-snapshot-004"],
            "supervision_mode": "required", "supervision_capabilities": ["NAVIGATION"],
            "supervision_heartbeat_timeout": 1.0, "supervision_request_timeout": 1.0,
            "permission_timeout": 3.0, "stop_action": "mission/stop",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.targets = load_targets(value("targets_file"))
        self.journal = MissionJournal(value("journal_path"))
        manipulation_backend = value("manipulation_backend")
        validate_backend(manipulation_backend, self.journal)
        snapshots = tuple(value("manipulation_mock_snapshot_ids")) if manipulation_backend == "action_mock" else ()
        if manipulation_backend == "action_mock" and (len(snapshots) < 4 or len(set(snapshots)) != len(snapshots)):
            raise ValueError("action_mock requires at least four distinct registered snapshot IDs")
        if manipulation_backend == "action_mock":
            fixture = value("manipulation_mock_fixture") or str(Path(get_package_share_directory(
                "cleany_mission_manager")) / "config/manipulation_server_fixture.yaml")
            validate_manipulation_fixture({
                "manipulation_mock_snapshot_ids": snapshots,
                "manipulation_destination": value("manipulation_destination"),
            }, fixture)
        self.desk = MockDesk(
            time.monotonic, polls=value("mock_operation_polls"),
            destination_id=value("manipulation_destination"), snapshot_ids=snapshots,
            objects=(SceneObject("trash-1", wire_object_id=1), SceneObject("trash-2", wire_object_id=2))
                    if manipulation_backend == "action_mock" else None,
        )
        self.manipulation = (ManipulationPort(
            ROSManipulationTransport(self, value("manipulation_action"), value("manipulation_query")),
            self.journal, time.monotonic,
            initial_mock_safe=value("manipulation_initial_mock_safe"),
            on_success=self.desk.manipulation_succeeded,
        ) if manipulation_backend == "action_mock" else self.desk.executor)
        backend = value("navigation_backend")
        if backend not in ("nav2", "mock"):
            raise ValueError("navigation_backend must be nav2 or mock")
        mode = value("supervision_mode")
        if mode not in ("required", "disabled"):
            raise ValueError("supervision_mode must be required or disabled")
        self.supervision = RosSupervision(
            self, list(value("supervision_capabilities")),
            heartbeat_timeout=float(value("supervision_heartbeat_timeout")),
            request_timeout=float(value("supervision_request_timeout")),
        ) if mode == "required" else None
        self.navigator = (Nav2Navigator(self, self.targets, self.journal,
                                        check_lifecycle=self.supervision is None) if backend == "nav2"
                          else DeferredPort(lambda _: ModuleResult.success(), polls=10))
        if value("desk_approach_enabled"):
            if backend != "nav2":
                raise ValueError("desk approach requires Nav2 staging")
            distance = float(value("desk_distance_m"))
            if not isfinite(distance) or distance <= 0:
                raise ValueError("desk_distance_m must be finite and positive")
            navigation = self.navigator
            self.navigator = StagedNavigator(navigation, DeskApproachPort(
                self, self.journal, navigation._base_stopped, distance, self.supervision))
        self.runtime = MissionRuntime(
            navigator=self.navigator, perception=self.desk.perception, planner=self.desk.planner,
            executor=self.manipulation, clock=time.monotonic,
            supported_targets=set(self.targets.seats), journal=self.journal,
            on_accept=self.desk.begin, navigation_mode="sim" if backend == "nav2" else "mock",
            policy=RuntimePolicy(manipulation_destinations=(value("manipulation_destination"),),
                                 **{name: value(name) for name in (
                "post_mission", "navigation_timeout", "cleaning_timeout", "operation_timeout",
                "cancel_timeout", "permission_timeout", "max_actions", "max_skill_retries",
            )}),
            supervision=self.supervision,
        )
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(MissionStatus, "mission/status", qos)
        self.result_pub = self.create_publisher(MissionResult, "mission/result", qos)
        self.create_service(OfferMission, "mission/offer", self._offer)
        self.create_service(CancelMission, "mission/cancel", self._cancel)
        self.create_service(GetRuntimeSnapshot, "mission/snapshot", self._snapshot)
        self.create_service(Trigger, "mission/reset_error", self._reset)
        self.stop_waiters: list[Future] = []
        if self.supervision is None:
            # 알림 전용이며 이 신호를 만드는 백엔드가 먼저 물리적 정지를 담당합니다.
            self.create_subscription(String, "robot/safety_fault", self._fault, 10)
            self.create_subscription(String, "robot/safety_released", self._released, 10)
        else:
            # 독립적인 안전 근거는 Supervisor 권한을 통해서만 Runtime에 전달됩니다.
            # 별도의 비감독 fault 경로를 두면 이 권한과 경쟁하게 됩니다.
            self.stop_server = ActionServer(self, StopMission, value("stop_action"), self._stop_mission,
                                            goal_callback=self._stop_goal,
                                            cancel_callback=lambda _: CancelResponse.REJECT)
        self.debug_pub = self.create_publisher(RuntimeDebugSnapshot, "mission/debug_snapshot", qos)
        self.event_pub = self.create_publisher(RuntimeEvent, "mission/runtime_events",
            QoSProfile(depth=2048, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_debug_at = 0.0
        self.last_debug_event = 0
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

    def _stop_goal(self, request):
        admission = self.runtime.supervisor_stop(request.mission_id, request.reason)
        return GoalResponse.ACCEPT if admission.accepted else GoalResponse.REJECT

    async def _stop_mission(self, goal):
        """이 Runtime이 자체 작업의 정지를 확인한 후에만 정지를 보고합니다."""
        # Runtime 자체 정지 예산으로 응답을 제한하며 더 늦은 정지는 실패로 처리합니다.
        deadline = time.monotonic() + self.runtime.policy.cancel_timeout + 0.5
        while rclpy.ok() and time.monotonic() < deadline:
            if self.runtime.supervisor_stop_confirmed(goal.request.mission_id):
                goal.succeed()
                return StopMission.Result(stopped=True, detail="mission terminal and operations stopped")
            goal.publish_feedback(StopMission.Feedback(state=self.runtime.state.value))
            future = Future()
            self.stop_waiters.append(future)
            await future
        goal.abort()
        return StopMission.Result(stopped=False, detail="stop not confirmed before the Runtime stop deadline")

    def _tick(self):
        if isinstance(self.navigator, (Nav2Navigator, StagedNavigator)):
            self.navigator.maintain()
        self.runtime.tick()
        waiting, self.stop_waiters = self.stop_waiters, []
        for future in waiting:
            if not future.done():
                future.set_result(None)
        try:
            self._publish_debug()
        except Exception as exc:
            # 진단 직렬화가 상태 또는 결과 발행을 막아서는 안 됩니다.
            self.get_logger().warning(f"Debug publication failed: {exc}", throttle_duration_sec=10)
        snapshot = self.runtime.snapshot(include_completed=False)
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

    def _publish_debug(self):
        stamp = self.get_clock().now().to_msg()
        for event in self.runtime.debug_events:
            if event["sequence"] <= self.last_debug_event:
                continue
            message = RuntimeEvent(boot_id=event["boot_id"], sequence=event["sequence"],
                mission_id=event["mission_id"], task_id=event["task_id"],
                execution_id=event["execution_id"], kind=event["kind"],
                data_json=json.dumps(event["data"]))
            message.header.stamp = stamp
            self.event_pub.publish(message)
            self.last_debug_event = event["sequence"]
        if time.monotonic() - self.last_debug_at < 0.5:
            return
        self.last_debug_at = time.monotonic()
        data = self.runtime.debug_snapshot()
        if isinstance(self.manipulation, ManipulationPort):
            data["manipulation_feedback"] = dict(self.manipulation.transport.feedback)
            data["manipulation_details"] = dict(self.manipulation.details)
        data["modules"]["navigation"].update({
            "topics": ["/" + self.get_parameter("odom_topic").value.lstrip("/"), "/tf", "/cmd_vel"],
            "actions": ["navigate_to_pose"] if isinstance(self.navigator, (Nav2Navigator, StagedNavigator)) else [],
        })
        data["modules"]["execution"].update({
            "topics": [], "actions": [self.get_parameter("manipulation_action").value]
                if self.get_parameter("manipulation_backend").value == "action_mock" else [],
        })
        target = data["navigation"]["target"]
        pose = self.targets.home if target == "home" else self.targets.seats.get(target)
        data["navigation"].update({"goal_pose": asdict(pose) if pose else None,
                                   "frame": self.targets.frame_id})
        nav = self.navigator.navigation if isinstance(self.navigator, StagedNavigator) else self.navigator
        if isinstance(nav, Nav2Navigator):
            now = time.monotonic()
            data["navigation"].update({"lifecycle": nav.active_nodes,
                "odom_age": now - nav.odom_received,
                "lifecycle_age": {key: now - stamp for key, stamp in nav.state_received.items()}})
            try:
                measurement = nav.localization_snapshot()
                data["navigation"]["tf_age"] = measurement["age_s"]
                data["navigation"]["localization"] = measurement
            except Exception as exc:
                data["navigation"]["tf_error"] = str(exc)
        message = RuntimeDebugSnapshot(boot_id=data["boot_id"],
            event_sequence=data["event_sequence"], state=data["state"], ready=data["ready"],
            reason=data["reason"], schema_version=1,
            mission_id=data["active_request"]["mission_id"] if data["active_request"] else "",
            bt_nodes=[DebugBTNode(**node) for node in data["bt_nodes"]],
            snapshot_json=json.dumps(data))
        message.header.stamp = stamp
        self.debug_pub.publish(message)

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
