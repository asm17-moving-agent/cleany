"""BT operations over the existing commissioned physical collection pipeline.

The XML owns order. No automatic target selection, collection loop, controller
activation, fault reset or simulated evidence is used by this adapter.
"""
from copy import deepcopy
import math
import threading
import time

from cleany_interfaces.action import InspectScene, SelectReachableGrasp, ValidatePick
from cleany_interfaces.srv import (CheckManipulation, GetSceneSnapshot,
                                   ObserveObjectReference, ObserveWristTarget, PlanGrasp)
from cleany_skill_executor.collection_pipeline import CollectionPipeline
from cleany_skill_executor.core.collection import (
    contact_verification_required, grasp_identity_verification_required, temporary_return_mode,
)
from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, REQUIRED_JOINT_NAMES, InfrastructureError,
)
from cleany_skill_executor.core.hardware_execution import ExecutionCanceled
from cleany_skill_executor.core.target_filter import select_closest_eligible
from cleany_skill_executor.manipulation.models import Error, ObjectState, Placement
from cleany_skill_executor.validate_pick_node import ValidatePickNode
from rclpy.action import ActionClient

from .action_tracking import TrackedClient, quiescent
from .backend import ExecutionContext, Observation, OperationError
from .joint_feedback import JointFeedback, StationaryWindow


class JetsonOperations(CollectionPipeline):
    def __init__(self, config: dict, **kwargs):
        self._feedback = JointFeedback(REQUIRED_JOINT_NAMES)
        self._feedback_lock = threading.Lock()
        self.execution_context: ExecutionContext | None = None
        self.current_node = ''
        self.deadline = math.inf
        self._stopping = False
        self.submissions, self.handles, self.tracked_clients = [], [], []
        try:
            self._initialize(config, **kwargs)
        except Exception:
            if hasattr(self, '_Node__node'):
                self.destroy_node()
            raise

    def _initialize(self, config: dict, **kwargs) -> None:
        super().__init__(config, **kwargs)
        defaults = {
            'bt_operation_timeout_sec': 180.0, 'bt_selection_timeout_sec': 600.0,
            'bt_stop_timeout_sec': 10.0,
            'bt_verification_timeout_sec': 10.0, 'bt_stop_feedback_max_age_sec': 0.5,
            'bt_stationary_duration_sec': 0.25, 'bt_stationary_samples': 5,
            'bt_stationary_velocity_rad_s': 0.02, 'bt_minimum_confidence': 0.25,
            'bt_maximum_smaller_extent_m': 0.11,
            'bt_arm_return_shortcut_attempts': 8,
            'bt_arm_return_optimization_budget_sec': .5,
            'bt_arm_return_planner_id': 'RRTConnectReturnkConfigDefault',
            'bt_arm_return_planning_trajectory_scaling': 1.0,
            'bt_denylist': ['battery', 'knife', 'broken glass'],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name, value in defaults.items():
            actual = self.get_parameter(name).value
            if isinstance(value, (float, int)) and (
                    not math.isfinite(actual) or actual <= 0):
                raise ValueError(f'{name} must be finite and positive')
        if self.value('bt_minimum_confidence') > 1:
            raise ValueError('bt_minimum_confidence must be at most one')
        if self.value('bt_arm_return_shortcut_attempts') > 32:
            raise ValueError('bt_arm_return_shortcut_attempts must be at most 32')
        if not self.value('bt_arm_return_planner_id').strip():
            raise ValueError('bt_arm_return_planner_id must not be empty')
        if self.value('bt_arm_return_planning_trajectory_scaling') > 1:
            raise ValueError('bt_arm_return_planning_trajectory_scaling must be at most one')
        self._snapshot = self.create_client(GetSceneSnapshot, '/perception/get_scene_snapshot')
        self._inspection = self.track(ActionClient(self, InspectScene, '/perception/inspect_scene'))
        self._selection = self.track(ActionClient(self, SelectReachableGrasp, '/grasp/select_reachable'))
        self._grasp = self.create_client(PlanGrasp, '/grasp/plan')
        self.motion._spin_once = self._spin_guarded
        self.report = {}
        self.before_observation = self.after_observation = self.pregrasp_pose = None

    def value(self, name):
        return self.get_parameter(name).value

    def track(self, client):
        self.tracked_clients[:] = [tracked for tracked in self.tracked_clients if not tracked.destroyed]
        tracked = TrackedClient(self, client)
        self.tracked_clients.append(tracked)
        return tracked

    def create_motion_client(self, action_type, name):
        return self.track(ActionClient(self, action_type, name))

    def on_state(self, message):
        super().on_state(message)
        with self._feedback_lock:
            self._feedback.update(message.name, message.velocity,
                message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                time.monotonic(), positions=message.position,
                maximum_position_interval_sec=float(self.value('bt_stop_feedback_max_age_sec')))

    def quiescent(self):
        return quiescent(list(self.submissions), list(self.handles))

    def confirm_recovery_stationary(self) -> bool:
        """Observe fresh idle feedback without canceling or commanding hardware."""
        if self.real is None or self.real.fault or self.carrying is not None:
            return False
        if not self.quiescent():
            return False
        from types import SimpleNamespace
        confirmed = self._assess_stationary(SimpleNamespace(stop_confirmed=False), cancel=False)
        return bool(confirmed and not self.real.fault and self.carrying is None and self.quiescent())

    def check_execution(self):
        if self._stopping or self.execution_context is None:
            return
        if self.real is not None and self.real.fault:
            raise OperationError(Error.HARDWARE_ERROR, self.real.fault)
        if self.execution_context.abort.is_set():
            raise ExecutionCanceled('Execution halted; awaiting independent stop evidence')
        if time.monotonic() >= self.deadline:
            raise OperationError(Error.TIMEOUT, 'BT operation deadline exceeded')

    def _spin_guarded(self, delay):
        self.check_execution()
        time.sleep(min(delay, 0.01))

    def defer_stop_assessment(self, error: BaseException) -> bool:
        """Cancel a scene/transport failure with live encoders for StopAndAssess.

        Hardware faults and untracked callers retain the independent latched
        stop. A failed cancellation/stationary assessment escalates there too.
        """
        if (not isinstance(error, InfrastructureError) or self.execution_context is None
                or self._stopping or self.real is None or self.real.fault
                or not self.real.guard or self.real.guard.get('state') != 'ready'
                or time.monotonic()-self.real.guard_time > self.real.value(
                    'hardware_guard_max_age_sec')):
            return False
        # Late goal acknowledgements also see this flag in TrackedClient.
        self.execution_context.abort.set()
        for handle, result in list(self.handles):
            if not result.done():
                try:
                    handle.cancel_goal_async()
                except Exception as cancel_error:
                    self.get_logger().error(f'Action cancellation request failed: {cancel_error}')
        self.event('stop_assessment_requested', reason=str(error))
        return True

    def wait(self, future, timeout, *, monitor=False, cancel_on_timeout=True):
        deadline = time.monotonic() + timeout
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        while not future.done():
            self.check_execution()
            if getattr(self, 'current_node', '') == 'SelectArmAndPath':
                self._selection_progress()
            if time.monotonic() >= deadline:
                # Never cancel the response to a submitted Action goal: its
                # late acknowledgement still needs tracking and cancellation.
                if cancel_on_timeout and not any(future is f for f in self.submissions):
                    future.cancel()
                raise OperationError(Error.TIMEOUT, 'ROS operation deadline exceeded')
            if not self._stopping and (monitor or self.carrying is not None):
                if monitor and (self.real is None or not self.real.sensor_scene_frozen):
                    self.scene_monitor.check()
                if self.real is None:
                    self.measured()
                else:
                    self.real.monitor()
            # Sequential collision RPCs should resume on completion; the
            # bounded wait still checks abort, deadlines and motion every 10 ms.
            completed.wait(max(0., min(0.01, deadline-time.monotonic())))
        return future.result()

    def execute(self, node: str, context: ExecutionContext) -> Observation:
        self.execution_context, self.current_node = context, node
        started = time.monotonic()
        timeout_parameter = ('bt_selection_timeout_sec' if node == 'SelectArmAndPath'
                             else 'bt_operation_timeout_sec')
        self.deadline = started + self.value(timeout_parameter)
        self._selection_started, self._selection_updated = started, 0.0
        context.progress = ''
        self._stopping = node in ('StopAndAssess', 'StopAfterRecovery')
        operations = {
            'ValidateGoal': self.validate, 'PrepareTarget': self.prepare,
            'ReconstructTarget': self.reconstruct, 'GenerateGrasp': self.generate,
            'SelectArmAndPath': self.select, 'MoveToPregrasp': self.pregrasp,
            'ApproachObject': self.approach, 'GraspObject': self.grasp,
            'ConfirmGrasp': self.confirm_grasp, 'LiftObject': self.lift,
            'ConfirmHeld': self.confirm_held, 'CarryObject': self.carry,
            'CheckPlacementTarget': self.check_placement, 'OpenGripperAtDestination': self.open,
            'ConfirmRelease': self.confirm_release, 'ReturnArm': self.return_arm,
            'VerifyPlacedObject': self.verify, 'StopAndAssess': self.stop,
            'ReleaseInPlace': self.release_in_place, 'ReturnArmAfterCancel': self.return_arm,
            'StopAfterRecovery': self.stop,
        }
        completed = False
        try:
            self.check_execution()
            observation = operations[node](context)
            completed = observation.success
            return observation
        except ExecutionCanceled as error:
            return Observation(False, Error.CANCELED, str(error))
        except OperationError as error:
            return self._failure(context, error.error, str(error))
        except Exception as error:
            if not isinstance(error, (RuntimeError, ValueError)):
                raise  # Programming failures remain INTERNAL_ERROR/FATAL.
            code = (Error.HARDWARE_ERROR if self.real is not None and self.real.fault else
                Error.TARGET_UNAVAILABLE if node in (
                'PrepareTarget', 'ReconstructTarget', 'GenerateGrasp', 'SelectArmAndPath') else
                Error.GRASP_FAILED if node in ('GraspObject', 'ConfirmGrasp') else
                Error.GRASP_LOST if node == 'ConfirmHeld' else
                Error.VERIFICATION_UNAVAILABLE if node in ('ConfirmRelease', 'VerifyPlacedObject') else
                Error.MOTION_FAILED)
            return self._failure(context, code, str(error))
        finally:
            if self._stopping or (node in ('ConfirmHeld', 'VerifyPlacedObject') and completed):
                self.stop_wrist_camera()
            elif not completed:
                # StopAndAssess must be able to cancel motion immediately. Send
                # camera shutdown now; assess its acknowledgement after stopping.
                self.request_wrist_camera_stop()

    @staticmethod
    def _failure(context, error, message):
        return Observation(False, error, message,
            object_state=(ObjectState.LEFT_GRIPPER if context.release_confirmed else
                          ObjectState.HELD if context.held else None),
            arm_recovered=True if context.arm_recovered else None,
            stop_confirmed=True if context.stop_confirmed else None)

    def validate(self, context):
        temporary = temporary_return_mode(self.config)
        if self.config.get('operation_mode', 'full_collection') != 'full_collection' and not temporary:
            raise OperationError(Error.DESTINATION_UNAVAILABLE,
                'Collection Actions require commissioned full_collection destinations; current profile is hold-only')
        if self.carrying is not None:
            raise OperationError(Error.HARDWARE_ERROR, 'An attached or uncertain payload remains')
        services = (self._snapshot, self._grasp, self.evidence) if grasp_identity_verification_required(
            self.config) else (self._snapshot, self._grasp)
        if not all(c.wait_for_service(timeout_sec=2.0) for c in services):
            raise OperationError(Error.BACKEND_NOT_READY, 'Snapshot, grasp or physical evidence service unavailable')
        if not all(c.wait_for_server(timeout_sec=2.0) for c in (self._inspection, self._selection)):
            raise OperationError(Error.BACKEND_NOT_READY, 'Inspection or motion selection Action unavailable')
        self.scene_monitor.wait()
        self.real.monitor()
        self.scene.restore()
        for client in self.tracked_clients:
            client._close_if_terminal()
            client.reset_completed()
        self.tracked_clients[:] = [client for client in self.tracked_clients if not client.destroyed]
        self.submissions.clear()
        self.handles.clear()
        self.report = {'wrist_checks': 0}
        self.events.clear()
        self.arm_config = None
        self.wrist_reference_id, self.wrist_request = '', None
        self.before_observation = self.after_observation = self.pregrasp_pose = None
        self.pickup_joints = None
        self.pickup_return_pose = self.pickup_arm = None
        self._pickup_departure_paths = self._pickup_departure_arm = None
        self._approach_command_paths = None
        self._approach_command_arm = None
        return Observation(message='Physical control, depth map and evidence ports ready')

    def prepare(self, context):
        goal = context.goal
        response = self.call(self._snapshot, GetSceneSnapshot.Request(snapshot_id=goal.snapshot_id))
        if not response.found:
            raise OperationError(Error.STALE_TARGET, response.message)
        array = response.detections
        matches = [d for d in array.detections if d.object_id == goal.object_id]
        if array.snapshot_id != goal.snapshot_id or len(matches) != 1:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Approved snapshot/object is absent or ambiguous')
        target = matches[0]
        if not target.distance_valid or not math.isfinite(target.distance_m) or target.distance_m <= 0:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Approved target has no valid sensor depth')
        expected = {'collect_trash': 'trash', 'collect_lost_item': 'lost_item'}[goal.skill_name]
        if (target.sorting_category != expected
                or not math.isfinite(target.confidence)
                or not self.value('bt_minimum_confidence') <= target.confidence <= 1):
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Model classification does not match approved skill')
        filtered = select_closest_eligible([target], denylist=tuple(self.value('bt_denylist')),
            maximum_smaller_extent_m=float(self.value('bt_maximum_smaller_extent_m')))
        if filtered.selected_index is None:
            raise OperationError(Error.TARGET_UNAVAILABLE, filtered.decisions[0].detail)
        configured = self.config['label_destinations'].get(target.label,
            self.config['label_destinations'].get('*'))
        if goal.destination_id != configured:
            raise OperationError(Error.DESTINATION_UNAVAILABLE, 'Destination does not match commissioned label policy')
        context.snapshot, context.target = deepcopy(array), deepcopy(target)
        return Observation(message=f'Approved target {goal.snapshot_id}/{goal.object_id}')

    def reconstruct(self, context):
        goal = context.goal
        result = self.run_action(self._inspection, InspectScene.Goal(
            snapshot_id=goal.snapshot_id, selected_object_id=goal.object_id),
            self.value('bt_operation_timeout_sec'))
        if (not result.success or result.objects.snapshot_id != goal.snapshot_id
                or result.objects.header.frame_id != self.config['frame']
                or len(result.objects.objects) != 1
                or result.objects.objects[0].object_id != goal.object_id
                or result.objects.objects[0].label != context.target.label):
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Selected-object reconstruction identity mismatch')
        self._object_radius(result.objects.objects[0])
        context.inspected = result
        pinned = self.call(self.reference, ObserveObjectReference.Request(
            operation=ObserveObjectReference.Request.PIN,
            source_snapshot_id=goal.snapshot_id, source_object_id=goal.object_id))
        if (not pinned.reference_id or pinned.source_snapshot_id != goal.snapshot_id
                or pinned.source_object_id != goal.object_id):
            raise OperationError(Error.STALE_TARGET, 'Pinned object identity mismatch')
        context.verification_id = pinned.reference_id
        return Observation()

    def generate(self, context):
        result = context.inspected
        goal = context.goal
        response = self.call(self._grasp, PlanGrasp.Request(snapshot_id=goal.snapshot_id,
            object_id=goal.object_id, target_cloud=result.target_cloud,
            context_cloud=result.context_cloud, target_object=result.objects.objects[0],
            target_object_header=result.objects.header), self.value('bt_operation_timeout_sec'))
        if not response.candidates or any(c.snapshot_id != goal.snapshot_id or c.object_id != goal.object_id
                                          for c in response.candidates):
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Grasp candidates absent or bound to another object')
        context.grasps = response
        return Observation()

    def select(self, context):
        center = context.inspected.objects.objects[0].obb_pose.position
        arms = [arm for arm in (('left', 'right') if center.y >= 0 else ('right', 'left'))
                if arm in self.config['arms']]
        for arm in arms:
            self._selection_detail = f'arm={arm}: waiting for candidate IK and path selection'
            self._selection_progress(force=True)
            selected = self.run_action(self._selection, SelectReachableGrasp.Goal(
                candidates=context.grasps.candidates, required_arm=arm),
                self.value('bt_selection_timeout_sec'), allow_aborted=True,
                feedback_callback=self._selection_feedback)
            if selected.success:
                break
        else:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'No reachable commissioned arm')
        if (selected.selected_arm != arm or selected.selected_candidate.snapshot_id != context.goal.snapshot_id
                or selected.selected_candidate.object_id != context.goal.object_id):
            raise OperationError(Error.STALE_TARGET, 'Motion selector changed approved object or arm')
        result = ValidatePick.Result(success=True, reference_id=context.verification_id,
            selected_object=context.inspected.objects.objects[0],
            selected_detection=context.target, detections=context.snapshot,
            candidates=context.grasps.candidates)
        ValidatePickNode._copy_selection_result(result, selected)
        context.selected = result
        self.before_execution(result)
        if self.destination != context.goal.destination_id:
            raise OperationError(Error.DESTINATION_UNAVAILABLE, 'Selected-arm destination mismatch')
        self.before_observation = self.after_observation = None
        if self.head_reobservation_required():
            self.before_observation = self.observe(result.reference_id, self.get_clock().now().nanoseconds)
            self._check_object_envelope(result, self.before_observation)
        else:
            self.report.update(target_motion_assumed_stationary=True,
                               head_reobservation_performed=False)
            self.event('stationary_target_assumed', snapshot_id=context.goal.snapshot_id,
                       object_id=context.goal.object_id, frame=self.config['frame'])
        if self.wrist_observation_required():
            self.start_wrist_camera(arm)
        return Observation(selected_arm=arm)

    def _selection_feedback(self, message) -> None:
        update = message.feedback
        self._selection_detail = (
            f'arm={update.arm} candidate={update.candidate_index} '
            f'stage={update.stage}: {update.message}')
        self._selection_progress(force=True)

    def _selection_progress(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._selection_updated < 2.0:
            return
        self._selection_updated = now
        self.execution_context.progress = (
            f'SelectArmAndPath {self._selection_detail}; '
            f'elapsed={now - self._selection_started:.1f}s')

    @staticmethod
    def _object_radius(obj) -> float:
        center, extent = obj.obb_pose.position, obj.obb_size
        extents = (extent.x, extent.y, extent.z)
        radius = math.hypot(*extents) / 2
        if (not all(math.isfinite(v) for v in (center.x, center.y, center.z, *extents, radius))
                or any(v < 0 for v in extents) or radius <= 0):
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Approved object geometry is invalid')
        return radius

    def observe(self, reference_id, after):
        response = super().observe(reference_id, after)
        goal = self.execution_context.goal
        stamp = response.header.stamp.sec * 10**9 + response.header.stamp.nanosec
        if (response.reference_id != reference_id or response.source_snapshot_id != goal.snapshot_id
                or response.source_object_id != goal.object_id
                or not after < stamp <= self.get_clock().now().nanoseconds):
            raise OperationError(Error.STALE_TARGET, 'Reference observation is stale or belongs to another target')
        return response

    def _check_object_envelope(self, selected, observation):
        center = selected.selected_object.obb_pose.position
        radius = self._object_radius(selected.selected_object)
        observed = observation.observed_center
        tolerance = float(self.get_parameter('maximum_target_translation_m').value)
        if (observation.header.frame_id != self.config['frame']
                or not all(math.isfinite(v) for v in (observed.x, observed.y, observed.z, tolerance))
                or tolerance < 0):
            raise OperationError(Error.STALE_TARGET, 'Fresh target geometry, frame or tolerance is invalid')
        approved_xyz = (center.x, center.y, center.z)
        observed_xyz = (observed.x, observed.y, observed.z)
        displacement = math.dist(approved_xyz, observed_xyz)
        allowed = radius + tolerance
        if displacement > allowed:
            approved = ','.join(f'{value:.6f}' for value in approved_xyz)
            fresh = ','.join(f'{value:.6f}' for value in observed_xyz)
            raise OperationError(Error.STALE_TARGET,
                'Fresh target is outside the approved object geometry: '
                f'distance={displacement:.6f}m allowed={allowed:.6f}m '
                f'object_radius={radius:.6f}m margin={tolerance:.6f}m '
                f'approved_center=({approved}) observed_center=({fresh}) '
                f'frame={observation.header.frame_id}')

    def pregrasp(self, context):
        selected = context.selected
        self.before_observation = self.after_observation = None
        if self.head_reobservation_required():
            # An expensive grasp/IK calculation cannot authorize a stale target.
            self.before_observation = self.observe(selected.reference_id, self.get_clock().now().nanoseconds)
            self._check_object_envelope(selected, self.before_observation)
        arrival = self.execute_pregrasp(selected)
        self.report['pregrasp_executed'] = self.config.get('pregrasp_motion_enabled', True)
        if self.head_reobservation_required():
            self.after_observation = self.observe_arrival(selected, self.before_observation, arrival, self.report)
            self._check_object_envelope(selected, self.after_observation)
            before, after = self.before_observation.observed_center, self.after_observation.observed_center
            self.validate_target_translation(selected, self.before_observation, self.after_observation,
                math.dist((before.x, before.y, before.z), (after.x, after.y, after.z)), self.report)
        if self.wrist_observation_required():
            observed = self.after_observation
            request = ObserveWristTarget.Request(operation=ObserveWristTarget.Request.HANDOFF,
                arm=selected.selected_arm, source_snapshot_id=context.goal.snapshot_id,
                source_object_id=context.goal.object_id, label=context.target.label,
                detector_label=context.target.detector_label,
                source_confidence=float(context.target.confidence), after_stamp_ns=arrival)
            request.expected_pose.header = deepcopy(observed.header)
            request.expected_pose.pose.position = deepcopy(observed.observed_center)
            request.expected_pose.pose.orientation.w = 1.0
            request.size = deepcopy(observed.observed_extent)
            response = self.call(self.wrist, request)
            if (not response.reference_id or response.source_snapshot_id != context.goal.snapshot_id
                    or response.source_object_id != context.goal.object_id):
                raise OperationError(Error.STALE_TARGET, 'Wrist handoff identity mismatch')
            request.reference_id = response.reference_id
            request.operation = request.CHECK
            request.after_stamp_ns = response.header.stamp.sec * 10**9 + response.header.stamp.nanosec
            self.wrist_request, self.wrist_reference_id = request, response.reference_id
        return Observation()

    def approach(self, context):
        selected = context.selected
        object_id = 'physical_payload_' + context.goal.execution_id
        if self.config.get('approach_execution_replanning_enabled', False):
            def prepare_pregrasp(reselected):
                context.selected = reselected
                self.pregrasp(context)
            selected, self.pregrasp_pose = self.approach_with_candidate_reselection(
                selected, self.report, object_id=object_id, prepare_pregrasp=prepare_pregrasp)
            context.selected = selected
        else:
            self.scene.begin(selected.selected_candidate, object_id)
            self.scene.allow_contacts_for(selected.selected_arm)
            self.pregrasp_pose = self.pose(selected.selected_arm)
            with self.record_approach_commands(selected.selected_arm):
                if self.config.get('visual_servo_enabled', False):
                    self.visual_approach(selected, self.before_observation, self.after_observation, self.report)
                else:
                    self.approach_target(selected, self.report)
        if not self.config.get('pregrasp_motion_enabled', True):
            self.pregrasp_pose = self.pose(selected.selected_arm)
        if temporary_return_mode(self.config):
            self.capture_pickup(selected.selected_arm)
        return Observation()

    def grasp(self, context):
        context.gripper_engaged = True
        self.grasp_after_ns = self.get_clock().now().nanoseconds
        if not contact_verification_required(self.config):
            self.jaw(context.selected.selected_arm, self.arm_config.closed, 'close_assumed',
                     check_collisions=self.config.get('grasp_collision_check_enabled', True))
            self.report.update(grasp_assumed=True, grasp_verified=False, contact_verification='SKIPPED')
            self.event('grasp_assumed_without_evidence')
        else:
            self.close_until_contact(context.selected, self.report)
        return Observation()

    def confirm_grasp(self, context):
        if contact_verification_required(self.config):
            self.real.wait_for_contact(context.selected.selected_arm, self.grasp_after_ns)
            if grasp_identity_verification_required(self.config):
                self.confirm_wrist_target(self.report, 0)
                self.verify_evidence(context.selected, CheckManipulation.Request.GRASP,
                                     self.grasp_after_ns)
        context.held = True
        self.attach(context.selected, 'physical_payload_' + context.goal.execution_id)
        return Observation(object_state=ObjectState.HELD)

    def lift(self, context):
        arm = context.selected.selected_arm
        lift_collisions = self.config.get('lift_collision_check_enabled', True)
        self.report['lift_collision_checks_enabled'] = lift_collisions
        if not lift_collisions:
            self.event('lift_collision_checks_disabled', arm=arm)
        with self.record_pickup_departure(arm):
            if self.config.get('pregrasp_motion_enabled', True):
                self.retreat_to_pregrasp(arm, self.pregrasp_pose, self.report,
                                        check_collisions=lift_collisions)
            bottom = self.pose(arm)
            top = deepcopy(bottom)
            if (self.config.get('lift_height_correction_enabled', False)
                    and self.config.get('pregrasp_motion_enabled', True)):
                top.position.z = max(top.position.z, self.pregrasp_pose.position.z)
            top.position.z += self.config['lift_height_m']
            self.lift_after_ns = self.get_clock().now().nanoseconds
            self.linear(arm, top, 'lift', check_collisions=lift_collisions)
            if self.config.get('lift_height_correction_enabled', False):
                self.correct_height_endpoint(arm, top, 'lift', self.report,
                                             check_collisions=lift_collisions)
            height = self.pose(arm).position.z - bottom.position.z
            self.report['lift_height_from_fk_m'] = height
        return Observation()

    def confirm_held(self, context):
        if self.wrist_observation_required():
            self.confirm_held_target(context.selected, self.pose(context.selected.selected_arm), self.report)
        if contact_verification_required(self.config):
            if grasp_identity_verification_required(self.config):
                self.verify_evidence(context.selected, CheckManipulation.Request.LIFT, self.lift_after_ns)
                self.report.update(grasp_verified=True,
                                   grasp_evidence='motor_contact_and_fresh_wrist_identity_after_lift')
            else:
                self.real.wait_for_contact(context.selected.selected_arm, self.lift_after_ns)
                self.report.update(grasp_verified=False, grasp_identity_verified=False,
                                   grasp_evidence='motor_contact_only_after_lift')
        return Observation(object_state=ObjectState.HELD)

    def carry(self, context):
        if temporary_return_mode(self.config):
            self.report.update(self.return_to_pickup(context.selected.selected_arm, self.pregrasp_pose))
            return Observation(message='Payload reached the configured pickup release position')
        else:
            self.move_joint_pose(context.selected.selected_arm, self.arm_config.bins[context.goal.destination_id], 'transport')
        return Observation()

    def check_placement(self, context):
        if temporary_return_mode(self.config):
            try:
                self.check_pickup_arrival(context.selected.selected_arm)
            except ValueError as error:
                raise OperationError(Error.DESTINATION_UNAVAILABLE, str(error)) from error
            return Observation()
        self.real.monitor()
        measured = self.measured()
        actual = dict(zip(measured.name, measured.position))
        target = self.arm_config.bins[context.goal.destination_id]
        tolerance = self.real.value('hardware_arrival_tolerance_rad')
        if any(abs(actual[n] - q) > tolerance for n, q in zip(ARM_JOINT_NAMES[context.selected.selected_arm], target)):
            raise OperationError(Error.DESTINATION_UNAVAILABLE, 'Measured arm is outside commissioned release pose')
        return Observation()

    def open(self, context):
        context.release_stamp_ns = self.get_clock().now().nanoseconds
        self.releasing = True
        self.jaw(context.selected.selected_arm, self.arm_config.opened, 'release',
                 check_collisions=self.config.get('gripper_opening_collision_check_enabled', True))
        return Observation()

    def _outcome(self, context, stage, after):
        selected = context.selected
        request = CheckManipulation.Request(stage=stage, arm=selected.selected_arm,
            reference_id=selected.reference_id, wrist_reference_id=self.wrist_reference_id,
            source_snapshot_id=context.goal.snapshot_id, source_object_id=context.goal.object_id,
            destination_id=context.goal.destination_id, after_stamp_ns=after)
        from cleany_skill_executor.core.collection import check_evidence
        deadline = time.monotonic() + self.value('bt_verification_timeout_sec')
        while time.monotonic() < deadline:
            response = self.wait(self.evidence.call_async(request), deadline-time.monotonic())
            if response is not None and response.success:
                try:
                    check_evidence(response, request, self.get_clock().now().nanoseconds,
                                   self.config['evidence_max_age_sec'])
                except RuntimeError:
                    pass  # Fresh negative evidence is not a positive outcome.
                else:
                    return response
            self._spin_guarded(0.01)
        raise OperationError(Error.VERIFICATION_TIMEOUT, 'Independent object/destination outcome not confirmed')

    def confirm_release(self, context):
        if temporary_return_mode(self.config):
            self.report.update(release_verified=False, placement_verified=False, placement_state='ASSUMED')
            self.event('release_assumed_at_pickup', destination_id=context.goal.destination_id)
        else:
            self._outcome(context, CheckManipulation.Request.RELEASE, context.release_stamp_ns)
        context.release_confirmed = True
        if self.carrying is not None:
            self.carrying['release_confirmed'] = True
            self.detach()
        self.scene.restore()
        self.release_grasp_scene_freeze()
        return Observation(object_state=ObjectState.LEFT_GRIPPER)

    def return_arm(self, context):
        if temporary_return_mode(self.config) and context.release_confirmed:
            self.pickup_release_retreat(context.selected.selected_arm, self.pregrasp_pose)
        self.move_arm_return(context.selected.selected_arm, self.arm_config.home,
                             self.arm_config.closed, planned_event='return_planned')
        context.arm_recovered = True
        return Observation(arm_recovered=True)

    def move_arm_return(self, arm, home, gripper_closed, *, planned_event='manual_return_planned'):
        from .arm_return_motion import move_arm_return
        return move_arm_return(self, arm, home, gripper_closed, planned_event=planned_event)

    def verify(self, context):
        placement = Placement.ASSUMED if temporary_return_mode(self.config) else Placement.CONFIRMED
        if placement == Placement.CONFIRMED:
            self._outcome(context, CheckManipulation.Request.PLACEMENT, self.get_clock().now().nanoseconds)
        confirmed = self._assess_stationary(context, cancel=False)
        if not confirmed:
            return Observation(False, Error.STOP_UNCONFIRMED, 'Final physical stop was not observed',
                               placement_state=placement, stop_confirmed=False)
        self._clear_references(context)
        return Observation(placement_state=placement, stop_confirmed=True)

    def release_in_place(self, context):
        if not context.gripper_engaged or context.release_confirmed:
            return Observation(object_state=ObjectState.LEFT_GRIPPER if context.release_confirmed else None)
        self.open(context)
        self.confirm_release(context)
        return Observation(object_state=ObjectState.LEFT_GRIPPER)

    def _clear_references(self, context):
        if self.wrist_request is not None:
            request = deepcopy(self.wrist_request)
            request.operation = request.CLEAR
            self.call(self.wrist, request)
            self.wrist_request, self.wrist_reference_id = None, ''
        if context.verification_id:
            self.call(self.reference, ObserveObjectReference.Request(
                operation=ObserveObjectReference.Request.CLEAR, reference_id=context.verification_id))
            context.verification_id = ''

    def _assess_stationary(self, context, *, cancel):
        started = time.monotonic()
        barrier = self.get_clock().now().nanoseconds
        window = StationaryWindow(barrier_ns=barrier, started_at=started,
            max_age_sec=float(self.value('bt_stop_feedback_max_age_sec')),
            maximum_velocity=float(self.value('bt_stationary_velocity_rad_s')),
            required_samples=int(self.value('bt_stationary_samples')),
            minimum_duration_sec=float(self.value('bt_stationary_duration_sec')))
        canceled = set()
        while time.monotonic() - started < self.value('bt_stop_timeout_sec'):
            if cancel:
                for handle, result in list(self.handles):
                    if not result.done() and id(handle) not in canceled:
                        try:
                            handle.cancel_goal_async()
                        except Exception as error:
                            self.get_logger().error(f'Could not request Action cancellation: {error}')
                            self.real.fail_stop()
                            return False
                        canceled.add(id(handle))
            with self._feedback_lock:
                confirmed = window.observe(self._feedback, now=time.monotonic(),
                    clock_ns=self.get_clock().now().nanoseconds, terminal=self.quiescent())
            if confirmed:
                context.stop_confirmed = True
                return True
            time.sleep(0.01)
        if cancel:
            self.real.fail_stop()
        return False

    def stop(self, context):
        context.abort.set()
        confirmed = self._assess_stationary(context, cancel=True)
        if confirmed and self.carrying is None:
            self.scene.restore()
            self.release_grasp_scene_freeze()
            if not context.gripper_engaged or context.release_confirmed:
                self._clear_references(context)
        return Observation(confirmed, Error.NONE if confirmed else Error.STOP_UNCONFIRMED,
            'Fresh joints and terminal Actions confirmed stop' if confirmed else 'Physical stop unconfirmed',
            stop_confirmed=confirmed)

    def destroy_node(self):
        if hasattr(self, 'scene_monitor'):
            self.scene_monitor.close()
        for client in self.tracked_clients:
            if not client.destroyed:
                client.client.destroy()
                client.destroyed = True
        return super().destroy_node()
