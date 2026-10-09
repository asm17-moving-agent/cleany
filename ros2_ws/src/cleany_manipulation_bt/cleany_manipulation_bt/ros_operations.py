"""Sensor/MoveIt operations for one approved object, on a dedicated ROS worker node."""
from __future__ import annotations

from copy import deepcopy

import math
import time
from typing import Any
from action_msgs.msg import GoalStatus

from cleany_interfaces.srv import GetSceneSnapshot, ObserveWristTarget, RegisterPlacementTarget, VerifyPlacement
from controller_manager_msgs.srv import ListControllers
from cleany_skill_executor.core.grasp_selection import InfrastructureError, REQUIRED_JOINT_NAMES
from cleany_skill_executor.core.nearest_object import ObjectAttempt
from cleany_skill_executor.core.sorting import COLLECTION_SKILLS, bin_release_region, normalize_label
from cleany_skill_executor.collision_geometry_cache import candidate_bounding_radius
from cleany_skill_executor.manipulation.models import Error, ObjectState, Placement
from cleany_skill_executor.nearest_pregrasp_coordinator import LiftRedetectionError
from cleany_skill_executor.pick_operations import GraspProgress
from cleany_skill_executor.seeded_cartesian import CartesianPlanningError
from cleany_skill_executor.sorting_coordinator import SortTarget, SortingCoordinator
import rclpy

from .backend import ExecutionContext, Observation, OperationError
from .joint_feedback import JointFeedback, StationaryWindow
from .target_identity import LOST_ITEM_LABELS, TRASH_LABELS, VERIFICATION_LABELS, label_key


class TrackedClient:
    """Track both pending acceptance and terminal responses, including late goals."""
    def __init__(self, node: MujocoOperations, client: Any) -> None:
        self.node, self.client = node, client

    def __getattr__(self, name: str) -> Any:
        return getattr(self.client, name)

    def send_goal_async(self, goal, **kwargs):
        self.node._guard()
        context = self.node.execution_context
        future = self.client.send_goal_async(goal, **kwargs)
        self.node.submissions.append(future)

        def accepted(response):
            if response.cancelled() or response.exception() is not None:
                return
            handle = response.result()
            if handle.accepted:
                terminal = handle.get_result_async()
                self.node.handles.append((handle, terminal))
                if context.abort.is_set():
                    handle.cancel_goal_async()
        future.add_done_callback(accepted)
        return future


class MujocoOperations(SortingCoordinator):
    """Reuse calibrated sorting motion without invoking run(), pick() or execute_sort()."""
    def __init__(self) -> None:
        self._joint_feedback = JointFeedback(REQUIRED_JOINT_NAMES)
        super().__init__()
        self._controller_retry_enabled = False
        self._require_unique_target_label = True
        self._geometry_association_lift = False
        self.execution_context: ExecutionContext | None = None
        self.current_node = ''
        self.deadline = math.inf
        self._clock_value = 0
        self._clock_progress_wall = time.monotonic()
        self.declare_parameter('bt_feedback_max_age_sec', 2.0)
        self.declare_parameter('bt_operation_timeout_sec', 180.0)
        self.declare_parameter('bt_verification_timeout_sec', 10.0)
        self.declare_parameter('bt_stop_timeout_sec', 10.0)
        self.declare_parameter('bt_stop_feedback_max_age_sec', 0.5)
        self.declare_parameter('bt_stationary_duration_sec', 0.25)
        self.declare_parameter('bt_supported_trash_labels', list(TRASH_LABELS))
        self.declare_parameter('bt_supported_lost_item_labels', list(LOST_ITEM_LABELS))
        self._target_label_key = label_key
        self._tracking_session_id = ''
        self._approved_tracking = None
        self._placement_registration = self.create_client(RegisterPlacementTarget, '/sorting/register_placement_target')
        self._snapshot = self.create_client(GetSceneSnapshot, '/perception/get_scene_snapshot')
        self._controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.submissions: list[Any] = []
        self.handles: list[tuple[Any, Any]] = []
        for name in ('_inspection', '_selection', '_move_group', '_execute_trajectory'):
            setattr(self, name, TrackedClient(self, getattr(self, name)))
        self._grippers = {arm: TrackedClient(self, client) for arm, client in self._grippers.items()}
        self._transport_adapter._spin_once = lambda t: self._spin_once(timeout_sec=t)

    def _on_joints(self, message) -> None:
        super()._on_joints(message)
        self._joint_feedback.update(message.name, message.velocity,
            message.header.stamp.sec * 10**9 + message.header.stamp.nanosec, time.monotonic())

    def quiescent(self) -> bool:
        def valid(future):
            return future.done() and not future.cancelled() and future.exception() is None
        terminals = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED}
        # A response Future can become done before its acceptance callback is
        # dispatched. Until its accepted handle is tracked, there is no stop
        # evidence for that action and no permission to begin a new context.
        return (all(valid(future) and future.result() is not None
                    and (not future.result().accepted
                         or any(handle is future.result() for handle, _ in self.handles))
                    for future in self.submissions)
                and all(valid(future) and future.result() is not None
                        and future.result().status in terminals for _, future in self.handles))

    def _execution_goal(self, *args):
        goal = super()._execution_goal(*args)
        goal.planning_options.replan = False
        return goal

    def _guard(self) -> None:
        if self.execution_context is None or self.current_node == 'StopAndAssess':
            return
        if self.execution_context.abort.is_set():
            raise OperationError(Error.CANCELED, 'Operation halted; awaiting stop assessment')
        if time.monotonic() >= self.deadline:
            raise OperationError(Error.TIMEOUT, 'Operation wall-clock deadline exceeded')
        motion = self.current_node in (
            'MoveToPregrasp', 'ApproachObject', 'GraspObject', 'ConfirmGrasp', 'LiftObject',
            'ConfirmHeld', 'CarryObject', 'CheckPlacementTarget', 'OpenGripperAtDestination', 'ReturnArm',
            'ReleaseInPlace', 'ReturnArmAfterCancel')
        if motion:
            maximum = float(self.get_parameter('bt_feedback_max_age_sec').value)
            now = self.get_clock().now().nanoseconds
            wall = time.monotonic()
            if now != self._clock_value:
                self._clock_value, self._clock_progress_wall = now, wall
            invalid = self._joint_feedback.invalid_joints(wall, now, maximum)
            if invalid or wall - self._clock_progress_wall > maximum:
                # /clock and /joint_states arrive independently. A small newer
                # joint stamp is possible before the next clock callback; large
                # skew, a stopped clock or stale receipt still fails closed.
                raise OperationError(Error.HARDWARE_ERROR,
                    'Joint feedback or simulation clock is stale: '
                    f'invalid_joints={list(invalid)} '
                    f'clock_idle={wall-self._clock_progress_wall:.3f}s')
        if self.current_node in ('LiftObject', 'ConfirmHeld', 'CarryObject', 'CheckPlacementTarget'):
            if self._held_object is None:
                raise OperationError(Error.GRASP_LOST, 'Missing held object')
            try:
                self._require_held_contact(self._held_object)
            except RuntimeError as error:
                if type(error) is not RuntimeError:
                    raise
                raise OperationError(Error.GRASP_LOST, str(error)) from error

    def _spin_once(self, timeout_sec: float) -> None:
        self._guard()
        super()._spin_once(timeout_sec)
        self._guard()

    def _future(self, future, timeout: float, label: str):
        # Keep late action acceptance observable so StopAndAssess can cancel it.
        deadline = min(self.deadline, time.monotonic() + timeout)
        while not future.done() and time.monotonic() < deadline:
            self._spin_once(timeout_sec=0.02)
        self._guard()
        if not future.done():
            raise OperationError(Error.TIMEOUT, f'Timed out waiting for {label}')
        result = future.result()
        if result is None:
            raise OperationError(Error.INTERNAL_ERROR, f'{label} returned no result')
        return result

    def execute(self, node: str, context: ExecutionContext) -> Observation:
        if self.execution_context is not context:
            self.execution_context = context
            self.handles.clear()
            self.submissions.clear()
            self._held_object = None
            self._fixed_release_wrist = {}
            self._wrist_reference = None
            self._lift_completion_stamp_ns = None
        self.current_node = node
        self.deadline = time.monotonic() + float(self.get_parameter('bt_operation_timeout_sec').value)
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
            'ReleaseInPlace': self.release_in_place, 'ReturnArmAfterCancel': self.return_after_cancel,
            'StopAfterRecovery': self.stop_after_recovery,
        }
        try:
            self._guard()
            return operations[node](context)
        except OperationError:
            raise
        except Exception as error:
            # Legacy operations report expected failures with these types.
            # RuntimeError subclasses such as NotImplementedError and
            # RecursionError must not become ordinary motion/grasp failures.
            expected = (type(error) in (RuntimeError, ValueError)
                        or isinstance(error, (InfrastructureError, CartesianPlanningError, LiftRedetectionError)))
            if not expected:
                raise OperationError(Error.INTERNAL_ERROR,
                    f'{node}: {type(error).__name__}: {error}') from error
            code = (Error.BACKEND_NOT_READY if node == 'ValidateGoal' else
                    Error.STALE_TARGET if node in ('MoveToPregrasp', 'ApproachObject') and isinstance(error, ValueError) else
                    Error.GRASP_FAILED if node in ('GraspObject', 'ConfirmGrasp') else
                    Error.GRASP_LOST if node == 'ConfirmHeld' else Error.MOTION_FAILED)
            raise OperationError(code, str(error)) from error

    def validate(self, context: ExecutionContext) -> Observation:
        clients = [self._inspection, self._selection, self._move_group, self._execute_trajectory,
                   *self._grippers.values()]
        deadline = time.monotonic() + float(self.get_parameter('startup_timeout_sec').value)
        while time.monotonic() < deadline:
            self._spin_once(timeout_sec=0.05)
            if (all(client.server_is_ready() for client in clients)
                    and all(client.service_is_ready() for client in
                            (self._snapshot, self._grasp, self._verification, self._placement_registration, self._controllers))
                    and set(REQUIRED_JOINT_NAMES) <= self._joint_positions.keys()):
                break
        else:
            raise OperationError(Error.BACKEND_NOT_READY, 'Perception, motion, verifier or joint state unavailable')
        response = self._future(self._controllers.call_async(ListControllers.Request()), 3., 'controller state')
        required = {'left_arm_controller', 'right_arm_controller',
                    'left_gripper_controller', 'right_gripper_controller'}
        active = {controller.name for controller in response.controller if controller.state == 'active'}
        if not required <= active:
            raise OperationError(Error.BACKEND_NOT_READY, 'Required controllers are not active')
        # Only a quiescent, non-inhibited prior execution can reach this node.
        # Remove a temporary world target left by a failure before gripper
        # closure. Held/unknown records require confirmation and cannot restart.
        self._execution_scene.restore()
        if bool(self.get_parameter('require_sensor_scene').value):
            self._wait_for_sensor_scene(5.)
        self._register_bins()
        self._home = {arm: self._arm_joint_state(arm) for arm in ('left', 'right')}
        if self._wrist_enabled:
            if not self._wrist_client.service_is_ready():
                raise OperationError(Error.BACKEND_NOT_READY, 'Wrist observation unavailable')
            self._switch_camera('head')
        return Observation(message='Sensor, controllers, MoveIt and independent verifier ready')

    def prepare(self, context: ExecutionContext) -> Observation:
        goal = context.goal
        response = self._future(self._snapshot.call_async(
            GetSceneSnapshot.Request(snapshot_id=goal.snapshot_id)), 3., 'snapshot lookup')
        if not response.found:
            raise OperationError(Error.STALE_TARGET, response.message)
        context.snapshot = response.detections
        matches = [d for d in response.detections.detections if d.object_id == goal.object_id]
        if len(matches) != 1 or response.detections.snapshot_id != goal.snapshot_id:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Approved object is absent or ambiguous')
        detection = matches[0]
        expected_category = {skill: category for category, skill in COLLECTION_SKILLS.items()}.get(goal.skill_name)
        if expected_category is None:
            raise OperationError(Error.INVALID_ARGUMENT, 'Unsupported collection skill')
        supported = self.get_parameter('bt_supported_' + expected_category + '_labels').value
        tracked = bool(response.detections.tracking_session_id)
        if tracked:
            if (not response.detections.tracking_epoch or not detection.track_id
                    or detection.tracking_state != 'TRACKED' or not detection.position_valid
                    or response.detections.representative_frame != 'base_link'
                    or sum(d.track_id == detection.track_id for d in response.detections.detections) != 1):
                raise OperationError(Error.TARGET_UNAVAILABLE, 'Approved tracking identity is invalid or ambiguous')
            self._approved_tracking = (response.detections.tracking_session_id,
                                       response.detections.tracking_epoch, detection.track_id)
            self._tracking_session_id = response.detections.tracking_session_id
        else:
            self._approved_tracking, self._tracking_session_id = None, ''
        self._require_unique_target_label = not tracked
        self._geometry_association_lift = tracked
        if (normalize_label(detection.label) not in supported or normalize_label(detection.label) not in VERIFICATION_LABELS
                or (not tracked and sum(label_key(d.label) == label_key(detection.label)
                        for d in response.detections.detections) != 1)):
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Verifier requires a supported type and valid individual identity')
        if not detection.distance_valid:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Target has no valid sensor depth')
        decision = self._policy.classify_model(detection.label, detection.confidence,
                                              detection.sorting_category, detection.sorting_reason)
        if decision.category.value != expected_category:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Target classification does not match approved skill')
        if goal.destination_id != decision.destination or goal.destination_id not in self._bins:
            raise OperationError(Error.DESTINATION_UNAVAILABLE, 'Destination does not match collection policy')
        context.attempt = ObjectAttempt(detection.object_id, detection.label, detection.confidence,
                                        detection.distance_m, detection.sorting_category, detection.sorting_reason)
        return Observation(message=f'Approved {goal.snapshot_id}/{goal.object_id}: {detection.label}')

    def reconstruct(self, context: ExecutionContext) -> Observation:
        context.inspected = self._inspect_selected(context.goal.snapshot_id, context.attempt)
        if context.inspected is None:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Selected-object reconstruction failed or expired')
        obj = context.inspected.objects.objects[0]
        if (context.inspected.objects.snapshot_id != context.goal.snapshot_id
                or obj.object_id != context.goal.object_id or obj.label != context.attempt.label):
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Reconstructed identity mismatch')
        if self._approved_tracking is not None:
            array = context.inspected.objects
            if ((array.tracking_session_id, array.tracking_epoch, obj.track_id) != self._approved_tracking
                    or obj.tracking_state != 'TRACKED'):
                raise OperationError(Error.STALE_TARGET, 'Reconstructed tracking identity mismatch')
        # Pin the independent evaluator before pregrasp or any other motion.
        verification_target = deepcopy(obj)
        verification_target.label = label_key(obj.label)
        response = self._future(self._placement_registration.call_async(RegisterPlacementTarget.Request(
            execution_id=context.goal.execution_id, destination_id=context.goal.destination_id,
            header=context.inspected.objects.header, target=verification_target)), 3., 'individual placement registration')
        if not response.success or not response.verification_id:
            raise OperationError(Error.VERIFICATION_UNAVAILABLE, response.message)
        context.verification_id = response.verification_id
        return Observation(message='Selected object reconstructed and individual verification registered')

    def generate(self, context: ExecutionContext) -> Observation:
        context.grasps = self._plan_grasps(context.inspected, context.attempt)
        if context.grasps is None:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'No grasp candidates')
        return Observation()

    def select(self, context: ExecutionContext) -> Observation:
        center = context.inspected.objects.objects[0].obb_pose.position
        arms = ('left', 'right') if center.y >= 0 else ('right', 'left')
        for arm in arms:
            context.selected = self._select_reachable(context.grasps.candidates, context.attempt, required_arm=arm)
            if context.selected is not None:
                break
        if context.selected is None:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'No collision-valid reachable grasp')
        self.check_payload_fit(context)
        context.target = SortTarget(context.attempt, context.selected)
        context.grasp_progress = GraspProgress()
        return Observation(selected_arm=context.selected.selected_arm)

    def check_payload_fit(self, context: ExecutionContext) -> None:
        cache = self._geometry_cache if self._geometry_subscription is not None else None
        radius = candidate_bounding_radius(context.selected.selected_candidate, cache)
        destination = self._bins[context.goal.destination_id]
        try:
            low, high = bin_release_region(destination.center_xy, destination.outside_size,
                destination.wall, destination.top_z, radius,
                float(self.get_parameter('sorting_release_clearance_m').value),
                float(self.get_parameter('sorting_release_maximum_clearance_m').value),
                float(self.get_parameter('sorting_release_edge_margin_m').value))
            point = self._fixed_release_points[context.goal.destination_id]
            if any(point - .001 < low) or any(point + .001 > high):
                raise ValueError('Configured release point does not fit observed payload')
        except ValueError as error:
            raise OperationError(Error.DESTINATION_UNAVAILABLE, str(error)) from error

    def pregrasp(self, context: ExecutionContext) -> Observation:
        # Revalidate before the first arm motion and again before the approach.
        MujocoOperations.refresh_approved(self, context)
        self._execute_pregrasp(context.selected, context.attempt)
        return Observation()

    def refresh_approved(self, context: ExecutionContext) -> None:
        # Reobserve ONLY the approved label, with existing geometric continuity
        # and ambiguity gates; never use geometry-only automatic reassignment.
        self._switch_camera('head') if self._wrist_enabled else None
        selected, attempt = self._refresh_selected_grasp(context.selected, context.attempt, match_label=True)
        old = self._policy.classify_model(context.attempt.label, context.attempt.confidence,
                                         context.attempt.sorting_category, context.attempt.sorting_reason)
        new = self._policy.classify_model(attempt.label, attempt.confidence,
                                         attempt.sorting_category, attempt.sorting_reason)
        if (new.category != old.category or new.destination != old.destination
                or COLLECTION_SKILLS.get(new.category.value) != context.goal.skill_name
                or new.destination != context.goal.destination_id):
            raise OperationError(Error.STALE_TARGET, 'Approved-object classification changed')
        supported = self.get_parameter('bt_supported_' + new.category.value + '_labels').value
        if normalize_label(attempt.label) not in supported:
            raise OperationError(Error.STALE_TARGET, 'Refreshed type is unsupported')
        context.selected, context.attempt = selected, attempt
        self.check_payload_fit(context)
        context.target = SortTarget(attempt, selected)

    def approach(self, context: ExecutionContext) -> Observation:
        MujocoOperations.refresh_approved(self, context)
        selected, attempt = context.selected, context.attempt
        if self._wrist_enabled:
            self._switch_camera(selected.selected_arm)
            self._wrist_reference = self._observe_wrist(selected, ObserveWristTarget.Request.HANDOFF)
            self._set_head_depth_boost(True)
        self._approach_grasp(selected, attempt, context.grasp_progress)
        return Observation(message='Approved-object continuity and approach confirmed')

    def grasp(self, context: ExecutionContext) -> Observation:
        context.gripper_engaged = True
        self._close_grasp(context.selected, context.attempt, context.grasp_progress, retry=False)
        return Observation()

    def confirm_grasp(self, context: ExecutionContext) -> Observation:
        self._confirm_grasp_contact(context.selected, context.attempt, context.grasp_progress)
        context.held = self._held_object
        context.held.target = context.target
        return Observation(object_state=ObjectState.HELD, message='Settled gripper contact and attachment confirmed')

    def lift(self, context: ExecutionContext) -> Observation:
        self._lift_grasp(context.selected, context.attempt, context.grasp_progress)
        return Observation(object_state=ObjectState.HELD)

    def confirm_held(self, context: ExecutionContext) -> Observation:
        self._confirm_held_grasp(context.selected, context.attempt, context.grasp_progress)
        if self._wrist_enabled:
            self._set_head_depth_boost(False)
        return Observation(object_state=ObjectState.HELD, message='Post-lift observation and contact confirmed')

    def carry(self, context: ExecutionContext) -> Observation:
        self.transport(context.held, context.goal.destination_id)
        return Observation(object_state=ObjectState.HELD)

    def check_placement(self, context: ExecutionContext) -> Observation:
        self.check_release_target(context.held, context.goal.destination_id)
        return Observation()

    def open(self, context: ExecutionContext) -> Observation:
        self.check_release_target(context.held, context.goal.destination_id)
        self.open_at_destination(context.held)
        context.release_stamp_ns = self._release_stamp_ns
        return Observation(message='Gripper opening confirmed by controller and joint feedback')

    def _verify_after(self, context: ExecutionContext, after_ns: int) -> Observation:
        if not context.verification_id:
            return Observation(False, Error.VERIFICATION_UNAVAILABLE, 'No individual placement registration',
                               placement_state=Placement.UNKNOWN)
        request = VerifyPlacement.Request(label=label_key(context.target.attempt.label),
            destination_id=context.goal.destination_id, after_stamp_ns=after_ns,
            verification_id=context.verification_id)
        deadline = time.monotonic() + float(self.get_parameter('bt_verification_timeout_sec').value)
        while time.monotonic() < deadline:
            response = self._future(self._verification.call_async(request), 2., 'independent placement verification')
            if response.success:
                return Observation(message=response.message, placement_state=Placement.CONFIRMED)
            self._spin_once(timeout_sec=0.1)
        return Observation(False, Error.VERIFICATION_TIMEOUT, 'Awaiting settled post-release evidence timed out',
                           placement_state=Placement.UNKNOWN)

    def confirm_release(self, context: ExecutionContext) -> Observation:
        observation = self._verify_after(context, context.release_stamp_ns)
        if not observation.success:
            return observation
        self.finish_release(context.held)
        context.release_confirmed = True
        context.gripper_engaged = False
        return Observation(object_state=ObjectState.LEFT_GRIPPER, placement_state=Placement.CONFIRMED,
                           message=observation.message)

    def return_arm(self, context: ExecutionContext) -> Observation:
        if not context.release_confirmed:
            raise OperationError(Error.PLACEMENT_NOT_CONFIRMED, 'Return requires independently confirmed release')
        self.retreat(context.held, context.goal.destination_id, prefetch=False)
        context.arm_recovered = True
        return Observation(arm_recovered=True)

    def release_in_place(self, context: ExecutionContext) -> Observation:
        """Initial RETURN_ARM policy: open where stopped, without bin verification."""
        arm = context.selected.selected_arm
        engaged = context.gripper_engaged or context.held is not None or self._held_object is not None
        if engaged:
            self._open_gripper(arm)
            self._hold('grasp_settle_sec')
        self._execution_scene.restore()
        self._held_object = None
        self._carry_wrist_reference = {}
        self._fixed_release_wrist = {}
        self._wrist_reference = None
        context.held = None
        context.gripper_engaged = False
        return Observation(object_state=ObjectState.LEFT_GRIPPER if engaged else None,
                           message='Released at cancellation position' if engaged else 'No held object to release')

    def return_after_cancel(self, context: ExecutionContext) -> Observation:
        # Destination release/verification belongs to the normal path. Recovery
        # plans from current feedback to the arm posture captured at startup.
        arm = context.selected.selected_arm
        joints = deepcopy(self._home[arm])
        self._move_to(arm, joints, 'return from cancellation')
        self._verify_feedback(joints)
        context.arm_recovered = True
        return Observation(arm_recovered=True, message='Arm returned after cancellation')

    def stop_after_recovery(self, context: ExecutionContext) -> Observation:
        confirmed = self._assess_stationary(context, cancel=False)
        return Observation(stop_confirmed=confirmed, message='Post-recovery stationary feedback confirmed'
                           if confirmed else 'Post-recovery stop could not be confirmed')

    def verify(self, context: ExecutionContext) -> Observation:
        observation = self._verify_after(context, self.get_clock().now().nanoseconds)
        if not observation.success:
            return observation
        settled = self._assess_stationary(context, cancel=False)
        return Observation(success=settled, error=Error.NONE if settled else Error.STOP_UNCONFIRMED,
                           stop_confirmed=settled, placement_state=Placement.CONFIRMED,
                           message=observation.message if settled else 'Final stationary feedback unavailable')

    def _assess_stationary(self, context: ExecutionContext, *, cancel: bool) -> bool:
        started_at = time.monotonic()
        deadline = started_at + float(self.get_parameter('bt_stop_timeout_sec').value)
        context.stop_confirmed = False
        window = StationaryWindow(
            barrier_ns=self.get_clock().now().nanoseconds, started_at=started_at,
            max_age_sec=float(self.get_parameter('bt_stop_feedback_max_age_sec').value),
            maximum_velocity=float(self.get_parameter('arm_stationary_velocity_rad_s').value),
            required_samples=int(self.get_parameter('arm_stationary_samples').value),
            minimum_duration_sec=float(self.get_parameter('bt_stationary_duration_sec').value))
        canceled: set[int] = set()
        while time.monotonic() < deadline:
            # Assessment bypasses the abort guard, but still has its own deadline.
            rclpy.spin_once(self, timeout_sec=0.02)
            if cancel:
                for handle, terminal in self.handles:
                    if not terminal.done() and id(handle) not in canceled:
                        handle.cancel_goal_async()
                        canceled.add(id(handle))
            wall = time.monotonic()
            if wall >= deadline:
                break
            if window.observe(self._joint_feedback, now=wall,
                              clock_ns=self.get_clock().now().nanoseconds, terminal=self.quiescent()):
                context.stop_confirmed = True
                return True
        return False

    def stop(self, context: ExecutionContext) -> Observation:
        context.abort.set()
        confirmed = self._assess_stationary(context, cancel=True)
        return Observation(stop_confirmed=confirmed, message='Stationary feedback and terminal actions confirmed'
                           if confirmed else 'Stop could not be confirmed from fresh feedback')
