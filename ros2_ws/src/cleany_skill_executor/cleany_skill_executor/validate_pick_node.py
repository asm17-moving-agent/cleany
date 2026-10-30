"""Plan-only detector -> YOLOE-seg -> AnyGrasp -> MoveIt coordinator."""

from __future__ import annotations

from array import array as byte_array
from copy import deepcopy
import json
import math
from pathlib import Path
import threading
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from cleany_interfaces.action import (
    InspectScene,
    SelectReachableGrasp,
    ValidatePick,
)
from cleany_interfaces.msg import GraspCandidate
from cleany_interfaces.srv import PlanGrasp, ObserveObjectReference
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory

from cleany_skill_executor.sensor_scene import SensorSceneMonitor
from cleany_skill_executor.core.target_filter import (
    DEFAULT_DENYLIST,
    select_closest_eligible,
)
from cleany_skill_executor.validation_evidence import write_validation_evidence
from cleany_skill_executor.validation_evidence import write_target_attempt_evidence
from cleany_skill_executor.validation_evidence import (
    filter_overlay,
    image_message_to_pil,
)


class _StageFailure(RuntimeError):
    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code


class _ObjectFailure(_StageFailure):
    """Only a typed object-local failure allows trying another target."""


class ValidatePickNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('validate_pick_server', **kwargs)
        defaults = {
            'action_name': 'grasp/validate_pick',
            'inspection_action_name': 'perception/inspect_scene',
            'grasp_service_name': 'grasp/plan',
            'selection_action_name': 'grasp/select_reachable',
            'required_arm': '',
            'denylist': list(DEFAULT_DENYLIST),
            'maximum_smaller_extent_m': 0.095,
            'workspace_roi_pixels': [-1.0, -1.0, -1.0, -1.0],
            'server_wait_timeout_sec': 10.0,
            'inspection_timeout_sec': 45.0,
            'grasp_timeout_sec': 45.0,
            'selection_timeout_sec': 120.0,
            'action_timeout_sec': 240.0,
            'perception_debug_topic': 'perception/debug_image_latched',
            'grasp_debug_topic': 'grasp/debug_image',
            'validation_debug_topic': 'grasp/validate_pick_debug',
            'evidence_output_directory': '',
            'pin_object_reference': False,
            'reference_cleanup_timeout_sec': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        cleanup_timeout = float(self.get_parameter('reference_cleanup_timeout_sec').value)
        if not math.isfinite(cleanup_timeout) or cleanup_timeout <= 0:
            raise ValueError('reference_cleanup_timeout_sec must be positive and finite')
        required_arm = str(self.get_parameter('required_arm').value)
        if required_arm not in ('', 'left', 'right'):
            raise ValueError('required_arm must be empty, left, or right')
        self._sensor_scene = SensorSceneMonitor(self)
        callback_group = ReentrantCallbackGroup()
        self._inspection = ActionClient(
            self,
            InspectScene,
            str(self.get_parameter('inspection_action_name').value),
            callback_group=callback_group,
        )
        self._grasp = self.create_client(
            PlanGrasp,
            str(self.get_parameter('grasp_service_name').value),
            callback_group=callback_group,
        )
        self._selection = ActionClient(
            self,
            SelectReachableGrasp,
            str(self.get_parameter('selection_action_name').value),
            callback_group=callback_group,
        )
        self._reference = self.create_client(ObserveObjectReference,
            '/perception/observe_object_reference', callback_group=callback_group)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._debug_lock = threading.Lock()
        self._perception_debug: Image | None = None
        self._grasp_debug: Image | None = None
        self.create_subscription(
            Image,
            str(self.get_parameter('perception_debug_topic').value),
            self._on_perception_debug,
            latched,
            callback_group=callback_group,
        )
        self._validation_debug = self.create_publisher(
            Image,
            str(self.get_parameter('validation_debug_topic').value),
            latched,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('grasp_debug_topic').value),
            self._on_grasp_debug,
            latched,
            callback_group=callback_group,
        )
        self._goal_lock = threading.Lock()
        self._goal_active = False
        self._active_downstream_goal: Any | None = None
        self._server = ActionServer(
            self,
            ValidatePick,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=callback_group,
        )

    def destroy_node(self) -> None:
        self._server.destroy()
        super().destroy_node()

    def _on_perception_debug(self, message: Image) -> None:
        with self._debug_lock:
            self._perception_debug = message

    def _on_grasp_debug(self, message: Image) -> None:
        with self._debug_lock:
            self._grasp_debug = message

    def _debug_messages(self) -> tuple[Image | None, Image | None]:
        with self._debug_lock:
            return deepcopy(self._perception_debug), deepcopy(self._grasp_debug)

    def _on_goal(self, _request) -> GoalResponse:
        with self._goal_lock:
            if self._goal_active:
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        if self._active_downstream_goal is not None:
            self._active_downstream_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    @staticmethod
    def _feedback(
        goal_handle,
        stage: int,
        message: str,
        *,
        detection_count: int = 0,
        candidate_count: int = 0,
        candidate_index: int = 0,
        arm: str = '',
    ) -> None:
        update = ValidatePick.Feedback()
        update.stage = stage
        update.message = message
        update.detection_count = detection_count
        update.candidate_count = candidate_count
        update.candidate_index = candidate_index
        update.arm = arm
        goal_handle.publish_feedback(update)

    def _wait_future(self, future, deadline: float, goal_handle, label: str):
        while not future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                future.cancel()
                raise InterruptedError('validate_pick canceled')
            if time.monotonic() >= deadline:
                future.cancel()
                raise RuntimeError(f'{label} timed out')
            time.sleep(0.01)
        if goal_handle is not None and goal_handle.is_cancel_requested:
            raise InterruptedError('validate_pick canceled')
        if time.monotonic() >= deadline:
            raise RuntimeError(f'{label} timed out')
        try:
            value = future.result()
        except Exception as error:
            raise RuntimeError(f'{label} failed: {error}') from error
        if value is None:
            raise RuntimeError(f'{label} returned no result')
        return value

    def _run_action(
        self,
        client,
        request,
        timeout_sec: float,
        goal_handle,
        overall_deadline: float,
        label: str,
        feedback_callback=None,
    ):
        self._check_request(goal_handle, overall_deadline)
        wait_timeout = min(float(self.get_parameter('server_wait_timeout_sec').value),
                           overall_deadline - time.monotonic())
        if not client.wait_for_server(timeout_sec=wait_timeout):
            raise RuntimeError(f'{label} server unavailable')
        self._check_request(goal_handle, overall_deadline)
        deadline = min(overall_deadline, time.monotonic() + timeout_sec)
        sent = client.send_goal_async(request, feedback_callback=feedback_callback)
        downstream = self._wait_future(sent, deadline, goal_handle, f'{label} goal')
        if not downstream.accepted:
            raise RuntimeError(f'{label} goal rejected')
        self._active_downstream_goal = downstream
        try:
            wrapped = self._wait_future(
                downstream.get_result_async(), deadline, goal_handle, label
            )
        except (InterruptedError, RuntimeError):
            downstream.cancel_goal_async()
            raise
        finally:
            self._active_downstream_goal = None
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            raise InterruptedError(f'{label} canceled')
        # Aborted actions carry typed failures (e.g. no reachable grasp).
        # Keep those codes so the coordinator can distinguish object failures
        # from service, state, TF, and planning-scene failures.
        if (wrapped.status == GoalStatus.STATUS_ABORTED
                and not wrapped.result.success
                and getattr(wrapped.result, 'error_code', 0) != 0):
            return wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            detail = str(getattr(wrapped.result, 'message', '')).strip()
            suffix = f': {detail}' if detail else ''
            raise RuntimeError(
                f'{label} ended with status {wrapped.status}{suffix}'
            )
        return wrapped.result

    def _run_grasp_service(
        self,
        request,
        timeout_sec: float,
        goal_handle,
        overall_deadline: float,
    ):
        self._check_request(goal_handle, overall_deadline)
        wait_timeout = min(float(self.get_parameter('server_wait_timeout_sec').value),
                           overall_deadline - time.monotonic())
        if not self._grasp.wait_for_service(timeout_sec=wait_timeout):
            raise RuntimeError('grasp planning service unavailable')
        self._check_request(goal_handle, overall_deadline)
        deadline = min(overall_deadline, time.monotonic() + timeout_sec)
        return self._wait_future(
            self._grasp.call_async(request),
            deadline,
            goal_handle,
            'grasp planning',
        )

    @staticmethod
    def _check_request(goal_handle: Any, deadline: float) -> None:
        if goal_handle.is_cancel_requested:
            raise InterruptedError('validate_pick canceled')
        if time.monotonic() >= deadline:
            raise RuntimeError('Validation deadline exceeded')

    def _wait_sensor_scene(self, goal_handle: Any, deadline: float) -> None:
        try:
            self._check_request(goal_handle, deadline)
            self._sensor_scene.wait(canceled=lambda: (
                goal_handle.is_cancel_requested or time.monotonic() >= deadline))
            self._check_request(goal_handle, deadline)
        except InterruptedError:
            if goal_handle.is_cancel_requested:
                raise
            raise _StageFailure(ValidatePick.Result.ERROR_MOVEIT, 'Sensor scene deadline exceeded')
        except RuntimeError as error:
            raise _StageFailure(ValidatePick.Result.ERROR_MOVEIT, str(error)) from error

    def _clear_reference(self, result: ValidatePick.Result) -> None:
        """Await cleanup before moving to a different target, even after cancellation."""
        if not result.reference_id:
            return
        response = self._wait_future(
            self._reference.call_async(ObserveObjectReference.Request(
                operation=ObserveObjectReference.Request.CLEAR,
                reference_id=result.reference_id)),
            time.monotonic() + float(self.get_parameter('reference_cleanup_timeout_sec').value),
            None, 'clear reference',
        )
        if not response.success:
            raise RuntimeError(f'Reference cleanup failed: {response.message}')
        result.reference_id = ''

    def _validate_object(self, goal_handle, phase1, selected_detection, result,
                         overall_deadline, latencies, decisions, phase1_debug):
        result.selected_detection = deepcopy(selected_detection)
        self._publish_filter_debug(
            phase1_debug,
            phase1.detections,
            decisions,
            int(selected_detection.object_id),
        )

        self._feedback(
            goal_handle,
            ValidatePick.Feedback.STAGE_SEGMENTING,
            f'Segmenting selected object {selected_detection.object_id}: '
            f'{selected_detection.label}',
            detection_count=len(phase1.detections.detections),
        )
        selection_goal = InspectScene.Goal()
        selection_goal.snapshot_id = phase1.detections.snapshot_id
        selection_goal.selected_object_id = selected_detection.object_id
        started = time.monotonic()
        try:
            phase2 = self._run_action(
                self._inspection,
                selection_goal,
                float(self.get_parameter('inspection_timeout_sec').value),
                goal_handle,
                overall_deadline,
                'selected-object inspection',
            )
        except RuntimeError as error:
            raise _StageFailure(result.ERROR_PERCEPTION, str(error)) from error
        finally:
            latencies.append(('segment_reconstruct', time.monotonic() - started))
        if not phase2.success:
            failure = (_ObjectFailure if phase2.error_code in (
                phase2.ERROR_MASK, phase2.ERROR_DEPTH, phase2.ERROR_PLANE,
            ) else _StageFailure)
            raise failure(result.ERROR_PERCEPTION, phase2.message)
        if (len(phase2.objects.objects) != 1
                or phase2.objects.objects[0].object_id != selected_detection.object_id):
            raise _StageFailure(result.ERROR_PERCEPTION, 'Selected-object identity/count mismatch')
        result.selected_object = deepcopy(phase2.objects.objects[0])
        phase2_debug, _ = self._debug_messages()
        if self.get_parameter('pin_object_reference').value:
            if not self._reference.wait_for_service(timeout_sec=5.):
                raise _StageFailure(result.ERROR_PERCEPTION, 'YOLOE-seg reference service unavailable')
            pin = ObserveObjectReference.Request(operation=ObserveObjectReference.Request.PIN,
                source_snapshot_id=phase1.detections.snapshot_id,
                source_object_id=selected_detection.object_id)
            pinned = self._wait_future(self._reference.call_async(pin),
                min(overall_deadline, time.monotonic()+10.), goal_handle, 'pin reference')
            if pinned is None or not pinned.success:
                raise _StageFailure(result.ERROR_PERCEPTION,
                    'Could not pin YOLOE-seg reference' if pinned is None else pinned.message)
            result.reference_id = pinned.reference_id
            if (not pinned.reference_id or pinned.source_snapshot_id != phase1.detections.snapshot_id
                    or pinned.source_object_id != selected_detection.object_id):
                raise _StageFailure(result.ERROR_PERCEPTION, 'Pinned reference identity mismatch')

        self._feedback(
            goal_handle,
            ValidatePick.Feedback.STAGE_GRASP_PLANNING,
            'Running AnyGrasp',
            detection_count=len(phase1.detections.detections),
        )
        request = PlanGrasp.Request()
        request.snapshot_id = phase1.detections.snapshot_id
        request.object_id = selected_detection.object_id
        request.target_cloud = phase2.target_cloud
        request.context_cloud = phase2.context_cloud
        request.target_object = phase2.objects.objects[0]
        request.target_object_header = phase2.objects.header
        started = time.monotonic()
        try:
            grasp = self._run_grasp_service(
                request,
                float(self.get_parameter('grasp_timeout_sec').value),
                goal_handle,
                overall_deadline,
            )
        except RuntimeError as error:
            raise _StageFailure(result.ERROR_GRASP_PLANNING, str(error)) from error
        finally:
            latencies.append(('anygrasp', time.monotonic() - started))
        result.candidates = list(grasp.candidates)
        if not grasp.success:
            failure = (_ObjectFailure if grasp.error_code == grasp.ERROR_NO_GRASP_CANDIDATE
                       else _StageFailure)
            raise failure(result.ERROR_GRASP_PLANNING, grasp.message)
        if not grasp.candidates:
            raise _ObjectFailure(result.ERROR_GRASP_PLANNING, 'No grasp candidates returned')

        self._feedback(
            goal_handle,
            ValidatePick.Feedback.STAGE_MOVEIT,
            'Validating candidates with the configured motion backend (plan-only)',
            detection_count=len(phase1.detections.detections),
            candidate_count=len(grasp.candidates),
        )
        moveit_goal = SelectReachableGrasp.Goal()
        moveit_goal.candidates = list(grasp.candidates)
        moveit_goal.required_arm = str(
            self.get_parameter('required_arm').value
        )

        def moveit_feedback(message) -> None:
            feedback = message.feedback
            self._feedback(
                goal_handle,
                ValidatePick.Feedback.STAGE_MOVEIT,
                feedback.message,
                detection_count=len(phase1.detections.detections),
                candidate_count=len(grasp.candidates),
                candidate_index=feedback.candidate_index,
                arm=feedback.arm,
            )

        started = time.monotonic()
        try:
            selected = self._run_action(
                self._selection,
                moveit_goal,
                float(self.get_parameter('selection_timeout_sec').value),
                goal_handle,
                overall_deadline,
                'MoveIt grasp validation',
                feedback_callback=moveit_feedback,
            )
        except RuntimeError as error:
            raise _StageFailure(result.ERROR_MOVEIT, str(error)) from error
        finally:
            latencies.append(('moveit_total', time.monotonic() - started))
        latencies.extend(zip(selected.latency_stages, selected.latency_seconds, strict=True))
        if not selected.success:
            failure = (_ObjectFailure if selected.error_code == selected.ERROR_NO_REACHABLE_GRASP
                       else _StageFailure)
            raise failure(result.ERROR_MOVEIT, selected.message)
        self._copy_selection_result(result, selected)
        return phase2, phase2_debug, selected

    def _execute(self, goal_handle):
        result = ValidatePick.Result()
        result.selected_candidate_index = -1
        overall_deadline = time.monotonic() + float(
            self.get_parameter('action_timeout_sec').value
        )
        latencies: list[tuple[str, float]] = []
        decisions = ()
        phase1_debug = None
        phase2_debug = None
        attempts: list[dict[str, Any]] = []
        snapshot_id = ""
        terminal = 'abort'
        try:
            self._wait_sensor_scene(goal_handle, overall_deadline)
            self._feedback(
                goal_handle,
                ValidatePick.Feedback.STAGE_DETECTING,
                'Capturing RGB-D and running object detection',
            )
            started = time.monotonic()
            inspection_goal = InspectScene.Goal()
            inspection_goal.query = goal_handle.request.query
            try:
                phase1 = self._run_action(
                    self._inspection,
                    inspection_goal,
                    float(self.get_parameter('inspection_timeout_sec').value),
                    goal_handle,
                    overall_deadline,
                    'inspection detection',
                )
            except RuntimeError as error:
                raise _StageFailure(result.ERROR_PERCEPTION, str(error)) from error
            latencies.append(('detect', time.monotonic() - started))
            if not phase1.success:
                raise _StageFailure(result.ERROR_PERCEPTION, phase1.message)
            result.detections = phase1.detections
            snapshot_id = phase1.detections.snapshot_id
            phase1_debug, _ = self._debug_messages()

            self._feedback(
                goal_handle,
                ValidatePick.Feedback.STAGE_FILTERING,
                'Applying denylist, depth, and expected-size screening',
                detection_count=len(phase1.detections.detections),
            )
            started = time.monotonic()
            filtered = select_closest_eligible(
                phase1.detections.detections,
                denylist=tuple(self.get_parameter('denylist').value),
                workspace_roi_pixels=(
                    () if list(self.get_parameter('workspace_roi_pixels').value) == [-1.0]*4
                    else tuple(self.get_parameter('workspace_roi_pixels').value)
                ),
                maximum_smaller_extent_m=float(
                    self.get_parameter('maximum_smaller_extent_m').value
                ),
            )
            latencies.append(('filter', time.monotonic() - started))
            decisions = filtered.decisions
            attempts = [dict(object_id=item.object_id, label=item.label,
                status='not_checked' if item.eligible else 'excluded',
                detail='Not attempted in this request' if item.eligible else item.detail,
                error_code=0, elapsed_sec=0.0, latency_seconds={}, candidate_count=0)
                for item in decisions]
            if filtered.selected_index is None:
                summary = '; '.join(
                    f'{item.object_id}:{item.detail}' for item in decisions
                )
                raise _StageFailure(
                    result.ERROR_NO_ELIGIBLE_TARGET,
                    f'No eligible target before YOLOE-seg: {summary}',
                )
            last_failure = None
            for index in filtered.eligible_indices:
                if goal_handle.is_cancel_requested:
                    raise InterruptedError('validate_pick canceled')
                if time.monotonic() >= overall_deadline:
                    raise _StageFailure(result.ERROR_INTERNAL, 'Validation deadline exceeded')
                selected_detection = phase1.detections.detections[index]
                record = attempts[index]
                record.update(status='checking', detail='Planning this object')
                result = ValidatePick.Result()
                result.selected_candidate_index = -1
                result.detections = phase1.detections
                object_latencies = []
                started_object = time.monotonic()
                try:
                    # Recheck world freshness before each object's planning attempt.
                    self._wait_sensor_scene(goal_handle, overall_deadline)
                    phase2, phase2_debug, selected = self._validate_object(
                        goal_handle, phase1, selected_detection, result,
                        overall_deadline, object_latencies, decisions, phase1_debug,
                    )
                except _ObjectFailure as error:
                    last_failure = error
                    record.update(status='plan_failed', detail=str(error), error_code=error.code)
                    # A failed clear stops the request; do not silently leak a pinned target.
                    try:
                        self._clear_reference(result)
                    except RuntimeError as cleanup_error:
                        record.update(status='interrupted', detail=str(cleanup_error),
                                      error_code=result.ERROR_PERCEPTION)
                        raise _StageFailure(result.ERROR_PERCEPTION, str(cleanup_error)) from cleanup_error
                    self.get_logger().info(
                        f'Object {selected_detection.object_id} skipped: {error}')
                    self._feedback(goal_handle, ValidatePick.Feedback.STAGE_FILTERING,
                        f'Object {selected_detection.object_id}: {error}; trying next object',
                        detection_count=len(phase1.detections.detections))
                    latencies.extend((f'object_{selected_detection.object_id}/{name}', seconds)
                                     for name, seconds in object_latencies)
                    continue
                except Exception as error:
                    record.update(status='interrupted', detail=str(error), error_code=(
                        error.code if isinstance(error, _StageFailure) else
                        result.ERROR_CANCELED if isinstance(error, InterruptedError) else result.ERROR_INTERNAL))
                    latencies.extend((f'object_{selected_detection.object_id}/{name}', seconds)
                                     for name, seconds in object_latencies)
                    raise
                else:
                    record.update(status='selected', detail='Validated grasp and motion paths')
                    latencies.extend(object_latencies)
                    break
                finally:
                    record['elapsed_sec'] = time.monotonic() - started_object
                    record['latency_seconds'] = dict(object_latencies)
                    record['candidate_count'] = len(result.candidates)
            else:
                assert last_failure is not None
                raise _StageFailure(last_failure.code,
                    'No validated grasp for any eligible object: ' + '; '.join(
                        f"{item['object_id']}:{item['detail']}" for item in attempts
                        if item['status'] == 'plan_failed'))
            result.latency_stages = [name for name, _ in latencies]
            result.latency_seconds = [seconds for _, seconds in latencies]

            evidence_root = str(
                self.get_parameter('evidence_output_directory').value
            ).strip()
            if evidence_root:
                self._feedback(
                    goal_handle,
                    ValidatePick.Feedback.STAGE_RECORDING,
                    'Writing JSON and 3x3 evidence',
                    detection_count=len(phase1.detections.detections),
                    candidate_count=len(result.candidates),
                )
                _, grasp_debug = self._debug_messages()
                try:
                    json_path, grid_path = write_validation_evidence(
                        Path(evidence_root),
                        snapshot_id=phase1.detections.snapshot_id,
                        query=goal_handle.request.query,
                        detections=phase1.detections.detections,
                        decisions=decisions,
                        selected_detection=selected_detection,
                        selected_object=phase2.objects.objects[0],
                        candidates=result.candidates,
                        selection=selected,
                        latencies=latencies,
                        phase1_debug=phase1_debug,
                        phase2_debug=phase2_debug,
                        grasp_debug=grasp_debug,
                        target_attempts=attempts,
                    )
                    result.evidence_json_path = json_path
                    result.evidence_grid_path = grid_path
                except Exception as error:
                    self.get_logger().error(f'Failed to write evidence: {error}')
            result.success = True
            result.error_code = result.ERROR_NONE
            result.message = (
                f'Validated AnyGrasp candidate {result.selected_candidate_index} '
                f'with {result.selected_arm} arm (plan-only)'
            )
            terminal = 'succeed'
        except InterruptedError as error:
            self._set_failure(result, result.ERROR_CANCELED, str(error))
            terminal = 'canceled'
        except _StageFailure as error:
            self._set_failure(result, error.code, str(error))
        except Exception as error:
            self._set_failure(
                result, result.ERROR_INTERNAL, f'Unexpected validation error: {error}'
            )
        finally:
            self._active_downstream_goal = None
            if terminal != 'succeed' and result.reference_id:
                try:
                    self._clear_reference(result)
                except RuntimeError as error:
                    self.get_logger().error(str(error))
                    self._set_failure(result, result.ERROR_PERCEPTION,
                        f'{result.message}; reference cleanup failed: {error}')
                    terminal = 'abort'
            result.latency_stages = [name for name, _ in latencies]
            result.latency_seconds = [seconds for _, seconds in latencies]
            for record in attempts:
                if record['status'] == 'not_checked':
                    record['detail'] = (f'Object {result.selected_detection.object_id} selected first'
                        if result.success else f'Request stopped: {result.message}')
            self.get_logger().info('Target attempts: ' + json.dumps(attempts, ensure_ascii=False))
            evidence_root = str(self.get_parameter('evidence_output_directory').value).strip()
            if evidence_root and snapshot_id:
                try:
                    attempts_path = write_target_attempt_evidence(Path(evidence_root),
                        snapshot_id=snapshot_id, attempts=attempts,
                        success=result.success, error_code=result.error_code, message=result.message)
                    if not result.evidence_json_path:
                        result.evidence_json_path = attempts_path
                except Exception as error:
                    self.get_logger().error(f'Failed to write target attempts: {error}')
            with self._goal_lock:
                self._goal_active = False
        if terminal == 'succeed':
            goal_handle.succeed()
        elif terminal == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    @staticmethod
    def _copy_selection_result(result, selected) -> None:
        fields = (
            'selected_candidate_index', 'selected_arm', 'selected_candidate',
            'pregrasp_joint_state', 'grasp_joint_state',
            'planning_start_state', 'pregrasp_trajectory', 'grasp_trajectory',
            'fk_pregrasp_pose', 'fk_grasp_pose',
            'pregrasp_position_error_m', 'grasp_position_error_m',
            'pregrasp_approach_error_degrees',
            'pregrasp_closing_error_degrees',
            'grasp_approach_error_degrees', 'grasp_closing_error_degrees',
            'trajectory_max_approach_error_degrees',
            'trajectory_max_closing_error_degrees',
            'gripper_joint_position', 'planner_id',
        )
        for field in fields:
            setattr(result, field, deepcopy(getattr(selected, field)))

    def _publish_filter_debug(
        self,
        base_message: Image | None,
        detections,
        decisions,
        selected_object_id: int,
    ) -> None:
        rendered = filter_overlay(
            image_message_to_pil(base_message),
            detections.detections,
            decisions,
            selected_object_id,
        )
        array = rendered.convert('RGB')
        message = Image()
        message.header = detections.header
        message.height = array.height
        message.width = array.width
        message.encoding = 'rgb8'
        message.is_bigendian = False
        message.step = array.width * 3
        message.data = byte_array('B', array.tobytes())
        self._validation_debug.publish(message)

    @staticmethod
    def _set_failure(result, code: int, message: str) -> None:
        result.success = False
        result.error_code = code
        result.message = message
        result.selected_candidate_index = -1
        result.selected_arm = ''
        result.selected_candidate = GraspCandidate()
        result.pregrasp_joint_state = JointState()
        result.grasp_joint_state = JointState()
        result.planning_start_state = JointState()
        result.pregrasp_trajectory = JointTrajectory()
        result.grasp_trajectory = JointTrajectory()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ValidatePickNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
