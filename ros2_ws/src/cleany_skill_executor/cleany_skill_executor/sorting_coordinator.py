"""Sensor-driven pick and place with configurable semantic sorting rules."""
from __future__ import annotations

from dataclasses import dataclass, replace
from copy import deepcopy
import json
import math
from pathlib import Path
import time
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from cleany_interfaces.srv import ObserveObjectReference, ObserveWristTarget, VerifyPlacement
from cleany_interfaces.msg import WristTrackingStatus
from cleany_skill_executor.core.carry_guard import CarryGuard, ContactLossGuard, TrackingEvidence
from cleany_skill_executor.core.cartesian import validate_pose_endpoint
from cleany_skill_executor.controller_stop import ControllerStop
from cleany_skill_executor.collision_geometry_cache import candidate_bounding_radius
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import serialize_message
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

from cleany_mujoco_sim.sorting_scene import load_bins, load_shelf_boxes
from cleany_skill_executor.core.can_rgbd import CameraProjection, rotation_matrix_from_quaternion
from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES
from cleany_skill_executor.core.sorting import (
    Category, execute_sort, load_sorting_policy, table_placement_slots,
)
from cleany_skill_executor.core.reobservation import reobservation_centers
from cleany_skill_executor.moveit_adapter import (
    MoveItAdapterConfig, MoveItGraspAdapter,
)
from cleany_skill_executor.nearest_pregrasp_coordinator import (
    LiftRedetectionError, NearestPregraspCoordinator,
)


@dataclass
class SortTarget:
    attempt: object
    selected: object


@dataclass
class HeldObject:
    target: SortTarget
    selected: object
    offset_in_tcp: np.ndarray


def held_bounding_radius(node, held: HeldObject) -> float:
    cache = (node._geometry_cache
             if getattr(node, '_geometry_subscription', None) is not None else None)
    return candidate_bounding_radius(held.selected.selected_candidate, cache)


def rotation(pose: Pose) -> np.ndarray:
    q = pose.orientation
    return rotation_matrix_from_quaternion(q.x, q.y, q.z, q.w)


def object_in_destination(obj, destination) -> bool:
    if destination is None:
        return False
    center, size = obj.obb_pose.position, obj.obb_size
    half_extent = np.abs(rotation(obj.obb_pose)) @ np.array((size.x,size.y,size.z))/2
    return (destination.contains((center.x,center.y,center.z))
            and all(abs(value-destination.center_xy[i])+half_extent[i]
                    < destination.outside_size[i]/2-destination.wall
                    for i,value in enumerate((center.x,center.y))))


class SortingCoordinator(NearestPregraspCoordinator):
    def __init__(self):
        super().__init__()
        skill = Path(get_package_share_directory('cleany_skill_executor'))
        sim = Path(get_package_share_directory('cleany_mujoco_sim'))
        self.declare_parameter('sorting_policy', str(
            skill / 'config' / 'table_sorting_policy.yaml'))
        self.declare_parameter('sorting_bins_config', str(
            sim / 'config' / 'robot_top_bins.yaml'))
        self.declare_parameter('sorting_maximum_objects', 12)
        self.declare_parameter('sorting_empty_confirmations', 2)
        self.declare_parameter('sorting_exit_on_finish', True)
        self.declare_parameter('sorting_test_only_label', '')
        self._wrist_enabled = self.declare_parameter('sorting_use_wrist_camera', False).value
        self._async_carry = self.declare_parameter('sorting_async_carry_monitor', False).value
        if self._async_carry and not self._wrist_enabled:
            raise ValueError('Asynchronous carry monitoring requires the wrist camera')
        self.declare_parameter('sorting_approach_acceleration_scaling', 1.0)
        self.declare_parameter('sorting_return_gripper_position_rad', -0.30)
        self._carry_guard = CarryGuard(
            self.declare_parameter('sorting_tracking_max_capture_age_sec', 12.).value,
            self.declare_parameter('sorting_tracking_max_update_age_sec', 8.).value)
        self._contact_loss_guard = ContactLossGuard(
            self.declare_parameter('sorting_contact_loss_grace_sec', .3).value)
        self.declare_parameter('sorting_joint_feedback_max_age_sec', 1.)
        self._carry_joint_received = {}
        self._controller_stop = ControllerStop(self)
        self._tracking_subscription = self.create_subscription(
            WristTrackingStatus, '/perception/wrist_tracking_status', self._tracking_status, 10)
        self._wrist_client = self.create_client(ObserveWristTarget, '/perception/observe_wrist_target')
        self._camera_client = self.create_client(SetParameters, '/sorting_cameras/set_parameters')
        self._wrist_reference = None
        self.declare_parameter('sorting_release_clearance_m', 0.06)
        self.declare_parameter('sorting_table_release_clearance_m', 0.015)
        self.declare_parameter('sorting_head_reference_refresh_age_sec', 30.0)
        self.declare_parameter('sorting_verification_timeout_sec', 10.0)
        self.declare_parameter('sorting_artifact_directory', '')
        self.declare_parameter('sorting_reobserve_after_lift', True)
        self.declare_parameter('sorting_use_reference_observation', True)
        self.declare_parameter('sorting_reference_timeout_sec', 30.0)
        self.declare_parameter('sorting_reobserve_margin_px', 24.0)
        self.declare_parameter('sorting_reobserve_geometry_padding_m', 0.01)
        self.declare_parameter('sorting_reobserve_max_translation_m', 0.20)
        self.declare_parameter('sorting_reobserve_max_lowering_m', 0.10)
        self.declare_parameter('sorting_reobserve_height_clearance_m', 0.02)
        self.declare_parameter('sorting_held_association_tolerance_m', 0.03)
        self.declare_parameter('sorting_payload_velocity_scaling', 0.01)
        self.declare_parameter('sorting_payload_acceleration_scaling', 0.01)
        self.declare_parameter(
            'sorting_required_categories', ['trash', 'lost_item'])
        self._require_category_coverage = self.declare_parameter('sorting_require_category_coverage', False).value
        # This executable is currently simulation-only. The inspection and
        # motion ports remain sensor-based; the outcome verifier is an oracle.
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('Sorting execution is simulation-only')
        if self.get_parameter('plan_only').value:
            raise RuntimeError('Sorting requires execution, not plan_only')
        self._policy = load_sorting_policy(
            self.get_parameter('sorting_policy').value)
        self._bins = {b.name: b for b in load_bins(
            self.get_parameter('sorting_bins_config').value, include_staging=True)}
        for name in (self._policy.trash_destination,
                     self._policy.lost_item_destination):
            if name not in self._bins:
                raise ValueError(f'Policy destination has no bin: {name}')
        self._verification = self.create_client(
            VerifyPlacement, '/sorting/verify_placement')
        self._reference_client = self.create_client(
            ObserveObjectReference, '/perception/observe_object_reference')
        self._pinned_reference = None
        self._apply_scene = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self._transport_adapter = MoveItGraspAdapter(
            self, config=MoveItAdapterConfig(preserve_scene_attachments=True,
                fk_timeout_sec=float(self.get_parameter('tcp_fk_timeout_sec').value),
                state_validity_timeout_sec=float(self.get_parameter('tcp_fk_timeout_sec').value),
                ik_response_margin_sec=float(self.get_parameter('tcp_fk_timeout_sec').value)),
            spin_once=lambda t: rclpy.spin_once(self, timeout_sec=t))
        self._status = self.create_publisher(
            String, '/sorting/status', QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._completed = []
        self._placed_footprints = []
        self._pending_handoffs = []
        self._current = {}
        self._release_stamp_ns = 0
        self._home = {}
        self._held_object: HeldObject | None = None
        directory = str(self.get_parameter('sorting_artifact_directory').value)
        self._artifact_directory = None
        self._artifact_sequence = 0
        if directory:
            path = Path(directory).expanduser()
            if not path.is_absolute():
                raise ValueError('Sorting artifact directory must be absolute')
            self._artifact_directory = path / f'run-{uuid4().hex}'
            self._artifact_directory.mkdir(parents=True)
            self.get_logger().info(f'Pipeline artifacts: {self._artifact_directory}')

    def _record_pipeline_message(self, stage: str, message: object) -> None:
        if self._artifact_directory is None:
            return
        self._artifact_sequence += 1
        path = self._artifact_directory / f'{self._artifact_sequence:03d}_{stage}.cdr'
        path.write_bytes(serialize_message(message))

    def _attempts(self, detections):
        attempts = super()._attempts(detections)
        label = self.get_parameter('sorting_test_only_label').value
        # Diagnostic isolation never changes the normal empty-workspace gate:
        # other visible objects remain unresolved, so this cannot report clean.
        return tuple(a for a in attempts if not label or a.label == label)

    def _execution_goal(self, arm, joint_state, label):
        goal = super()._execution_goal(arm, joint_state, label)
        if f'{arm}_gripper_joint' in joint_state.name:
            expected = {name for name in REQUIRED_JOINT_NAMES if name.startswith(f'{arm}_')}
            if arm not in ('left', 'right') or set(joint_state.name) != expected or self._held_object is not None:
                raise ValueError('Combined pregrasp requires only the selected empty arm and gripper')
            # One collision-checked six-joint path, dispatched by MoveIt to
            # disjoint arm/gripper controllers. No concurrent blind open command.
            group = 'return_close' if label.startswith('return from ') else 'pregrasp_open'
            goal.request.group_name = f'{arm}_{group}'
        if self._held_object is not None:
            if getattr(self, '_async_carry', False):
                # Do not restart a cancelled payload controller inside MoveIt.
                goal.planning_options.replan = False
            for attribute, parameter in (
                ('max_velocity_scaling_factor', 'sorting_payload_velocity_scaling'),
                ('max_acceleration_scaling_factor', 'sorting_payload_acceleration_scaling'),
            ):
                value = float(self.get_parameter(parameter).value)
                if not math.isfinite(value) or not 0. < value <= 1.:
                    raise ValueError('Payload motion scaling must be finite and in (0, 1]')
                setattr(goal.request, attribute, min(getattr(goal.request, attribute), value))
        return goal

    def _joint_corridor_goal(self, arm, joints, start, target, label, velocity_scaling):
        goal = super()._joint_corridor_goal(arm, joints, start, target, label, velocity_scaling)
        if label == 'refreshed grasp approach':
            value = float(self.get_parameter('sorting_approach_acceleration_scaling').value)
            if not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError('Approach acceleration scaling must be in (0,1]')
            goal.request.max_acceleration_scaling_factor = value
        return goal

    def _tracking_status(self, message):
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        self._carry_guard.update(TrackingEvidence(message.reference_id, message.arm,
            message.source_snapshot_id, message.source_object_id, stamp, message.header.frame_id,
            message.valid, message.visible, message.reason), time.monotonic())

    def _on_joints(self, message):
        super()._on_joints(message)
        if len(message.name) == len(message.position) == len(message.velocity):
            now = time.monotonic()
            for name in message.name:
                self._carry_joint_received[name] = now

    def _stop_guarded_controller(self, result_future):
        held = self._held_object
        if held is None:
            raise RuntimeError('Cannot resolve selected carry arm for cancellation')
        arm = held.selected.selected_arm
        self._controller_stop.stop(arm, result_future)
        # Require newly received low-velocity controller feedback, not cached state.
        previous = self._controller_state_counts[arm]
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            if self._controller_state_counts[arm] == previous:
                continue
            feedback = self._controller_states[arm].feedback.velocities
            maximum = float(self.get_parameter('arm_stationary_velocity_rad_s').value)
            if len(feedback) == 5 and all(math.isfinite(v) and abs(v) <= maximum for v in feedback):
                self.get_logger().error(f'ARM CONTROLLER fresh stationary feedback: arm={arm}')
                return
        raise RuntimeError('Selected arm did not confirm stationary feedback after guard cancellation')

    def _check_motion_guard(self):
        if not getattr(self, '_async_carry', False) or not self._carry_guard.armed:
            return
        self._carry_guard.check(self.get_clock().now().nanoseconds, time.monotonic())
        if self._held_object is None:
            raise RuntimeError('Carry guard armed without a held-object reference')
        arm = self._held_object.selected.selected_arm
        now = time.monotonic()
        maximum_age = float(self.get_parameter('sorting_joint_feedback_max_age_sec').value)
        age = now-self._carry_joint_received.get(f'{arm}_gripper_joint', -math.inf)
        if not math.isfinite(maximum_age) or maximum_age <= 0 or not 0 <= age <= maximum_age:
            raise RuntimeError('Carry gripper feedback stale or invalid')
        healthy = self._gripper_contact_stalled(arm,
            float(self.get_parameter('gripper_open_position_rad').value),
            self._candidate_close_position(self._held_object.selected.selected_candidate),
            allow_closing_motion=True)
        self._contact_loss_guard.check(healthy, now)

    def _execute_linear(self, arm, target, label, **kwargs):
        if (getattr(self, '_direct_vertical_lift', False)
                and label == 'vertical grasp lift' and kwargs.get('joint_target') is None):
            self._transport_adapter.set_current_state(self._feedback_state())
            solution = self._transport_adapter.solve_position_ik(
                arm, tuple(self._pose_position(target)), None)
            if solution is None:
                raise RuntimeError('No IK for direct vertical lift')
            joints = JointState(name=list(solution.names), position=list(solution.positions))
            actual = self._tcp_pose(arm, joints)
            validate_pose_endpoint(self._cartesian_pose(actual), self._cartesian_pose(target),
                float(self.get_parameter('lin_position_tolerance_m').value),
                math.radians(float(self.get_parameter('corridor_orientation_tolerance_deg').value)))
            # Explicit position corridor for the five-axis arm; retain full
            # seeded interpolation, FK, collision and angular-drift checks.
            target = actual
            kwargs['joint_target'] = joints
        return super()._execute_linear(arm, target, label, **kwargs)

    def _hold(self, parameter_name):
        if parameter_name == 'lift_hold_sec' and getattr(self, '_async_carry', False):
            self._check_motion_guard()
            self.get_logger().info('Continuous carry: skipping lift hold; monitoring remains armed')
            return
        super()._hold(parameter_name)

    def _plan_grasps(self, inspected, attempt):
        self._record_pipeline_message('inspection', inspected)
        result = super()._plan_grasps(inspected, attempt)
        if result is not None:
            self._record_pipeline_message('grasps', result)
        return result

    def _select_reachable(self, candidates, attempt, *, required_arm=''):
        result = super()._select_reachable(
            candidates, attempt, required_arm=required_arm)
        if result is not None:
            self._record_pipeline_message('selection', result)
        return result

    def _stage(self, stage, *, error: str | None = None):
        payload = dict(stage=stage, **self._current, completed=self._completed)
        if error is not None:
            payload['error'] = error
        if self._artifact_directory is not None:
            with (self._artifact_directory / 'stages.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(wall_time=time.time(), **payload))+'\n')
        self._status.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'SORTING {json.dumps(payload)}')

    def run(self):
        self._stage('starting')
        self._wait_for_pipeline()
        if not self._verification.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Placement verification port is unavailable')
        if (self.get_parameter('sorting_use_reference_observation').value
                and not self._reference_client.wait_for_service(timeout_sec=10.0)):
            raise RuntimeError('SAM2 reference observation port is unavailable')
        self._register_bins()
        if self._wrist_enabled:
            if not self._wrist_client.wait_for_service(timeout_sec=10):
                raise RuntimeError('Wrist observation service unavailable')
            self._switch_camera('head')
        self._home = {arm: self._arm_joint_state(arm)
                      for arm in ('left', 'right')}
        self._stage('search')
        maximum = int(self.get_parameter('sorting_maximum_objects').value)
        empty_confirmations = 0
        minimum_total_placements = 0
        for _ in range(maximum):
            detected = self._detect_objects()
            self._record_pipeline_message('detections', detected)
            choices = self._attempts(detected.detections.detections)
            # Reconstruct the same fresh snapshot before potentially long IK
            # searches expire its cache. Pick still performs fresh reobservation.
            inspections = {attempt.object_id: self._inspect_selected(
                detected.detections.snapshot_id, attempt) for attempt in choices}
            self._observed_footprints = []
            for inspected in inspections.values():
                if inspected is not None:
                    obj = inspected.objects.objects[0]
                    p,s = obj.obb_pose.position, obj.obb_size
                    self._observed_footprints.append((np.array((p.x,p.y,p.z)),float(np.linalg.norm((s.x,s.y,s.z))/2)))
            work_count = len(detected.detections.detections)
            for attempt in choices:
                observed = inspections[attempt.object_id]
                decision = self._policy.classify_model(attempt.label,attempt.confidence,
                    attempt.sorting_category,attempt.sorting_reason)
                if observed is not None and object_in_destination(observed.objects.objects[0],self._bins.get(decision.destination)):
                    work_count -= 1
            minimum_total_placements = max(minimum_total_placements,len(self._completed)+work_count)
            prepared = None
            unresolved = len(detected.detections.detections) - len(choices)
            for attempt in choices:
                decision = self._policy.classify_model(
                    attempt.label, attempt.confidence,
                    attempt.sorting_category, attempt.sorting_reason)
                self._current = dict(label=attempt.label,
                                     distance_m=attempt.distance_m,
                                     category=decision.category.value,
                                     destination=decision.destination,
                                     reason=decision.reason)
                inspected = inspections[attempt.object_id]
                if inspected is None:
                    unresolved += 1
                    continue
                center = inspected.objects.objects[0].obb_pose.position
                # Already placed items are not picked again, even if a camera
                # still sees them. This uses the perceived position, not GT.
                destination = self._bins.get(decision.destination)
                if object_in_destination(inspected.objects.objects[0],destination):
                    continue
                unresolved += 1
                if decision.category == Category.REVIEW:
                    self._stage('review')
                    continue
                planned = self._plan_grasps(inspected, attempt)
                if planned is None:
                    self._stage('no_grasp')
                    continue
                bin_y = self._bins[decision.destination].center_xy[1]
                arm = ('left' if bin_y > 0
                       else 'right')
                selected = self._select_reachable(
                    planned.candidates, attempt, required_arm=arm)
                staging = self._bins.get('handoff_center')
                if selected is None and staging is not None and not staging.contains((center.x,center.y,center.z)):
                    near_arm = 'left' if center.y >= 0 else 'right'
                    if near_arm != arm:
                        self._stage('try_staging')
                        selected = self._select_reachable(planned.candidates, attempt, required_arm=near_arm)
                        if selected is not None:
                            self._current['final_destination'] = decision.destination
                            self._current['destination'] = staging.name
                            self._current['transfer_kind'] = 'staging'
                            decision = replace(decision, destination=staging.name)
                if selected is None:
                    self._stage('unreachable')
                    continue
                prepared = SortTarget(attempt, selected)
                break
            if prepared is None:
                if unresolved:
                    raise RuntimeError(f'Work area is not clear: {unresolved} unresolved objects')
                if getattr(self, '_pending_handoffs', []):
                    raise RuntimeError('Pending handoff object missing from fresh perception; not clear')
                if len(self._completed) < minimum_total_placements:
                    raise RuntimeError('Previously observed work objects are missing without verified placement')
                empty_confirmations += 1
                self._stage('empty_confirmation')
                if empty_confirmations >= int(self.get_parameter('sorting_empty_confirmations').value):
                    break
                continue
            empty_confirmations = 0
            execute_sort(prepared, decision, self, self._stage)
            if self._current.get('transfer_kind') == 'staging':
                self._pending_handoffs.append(prepared.attempt.label)
                self._stage('handoff_complete')
            else:
                self._completed.append(dict(self._current))
                if prepared.attempt.label in self._pending_handoffs:
                    self._pending_handoffs.remove(prepared.attempt.label)
        else:
            raise RuntimeError('Sorting action/search budget exhausted before clear-workspace confirmation')
        required = (set(self.get_parameter('sorting_required_categories').value)
                    if getattr(self, '_require_category_coverage', False) else set())
        categories = {item['category'] for item in self._completed}
        if not required <= categories:
            self._stage('incomplete')
            raise RuntimeError(
                'Required sorting paths not verified: '
                + ', '.join(sorted(required - categories)))
        self._stage('mission_complete')

    def _execute_pregrasp(self, selected, attempt) -> None:
        arm = selected.selected_arm
        expected = {name for name in REQUIRED_JOINT_NAMES
                    if name.startswith(f'{arm}_') and 'gripper' not in name}
        joints = selected.pregrasp_joint_state
        opening = float(self.get_parameter('gripper_open_position_rad').value)
        if (arm not in ('left', 'right') or len(joints.name) != 5 or set(joints.name) != expected
                or len(joints.position) != 5 or not all(math.isfinite(q) for q in joints.position)
                or not math.isfinite(opening)):
            raise ValueError('Invalid selected-arm pregrasp/open target')
        combined = deepcopy(selected)
        combined.pregrasp_joint_state.name.append(f'{arm}_gripper_joint')
        combined.pregrasp_joint_state.position.append(opening)
        combined.pregrasp_joint_state.velocity = []
        combined.pregrasp_joint_state.effort = []
        # Preserve source selection and the five-joint grasp/retreat targets.
        # Base implementation registers the OBB, forbids target contacts, and
        # requires feedback convergence for ALL six joints before returning.
        self.get_logger().info(f'Pregrasp with simultaneous {arm} gripper opening')
        super()._execute_pregrasp(combined, attempt)

    def pick(self, target: SortTarget) -> HeldObject:
        self._held_object = None
        self._lift_completion_stamp_ns = None
        self._execute_pregrasp(target.selected, target.attempt)
        # The selected gripper opened along the pregrasp trajectory; do not
        # issue another timed opening after arrival.
        if getattr(self, '_wrist_enabled', False):
            selected, attempt = target.selected, target.attempt
            self._wait_arm_stationary(selected.selected_arm)
            selected, attempt = self._ensure_fresh_head_before_wrist(selected, attempt)
            if selected is not target.selected:
                target = SortTarget(attempt, selected)
            self._switch_camera(selected.selected_arm)
            self._wrist_reference = self._observe_wrist(selected, ObserveWristTarget.Request.HANDOFF)
            if getattr(self, '_async_carry', False):
                candidate = selected.selected_candidate
                self._carry_guard.bind(self._wrist_reference.reference_id, selected.selected_arm,
                                       candidate.snapshot_id, candidate.target_object.object_id)
            # Preserve the original head 3D estimate. RGB consistency is not new depth.
            self._execution_scene.allow_contacts_for(selected.selected_arm)
        else:
            selected, attempt = self._refresh_selected_grasp(target.selected, target.attempt)
        if not getattr(self, '_wrist_enabled', False) and self.get_parameter('sorting_use_reference_observation').value:
            candidate = selected.selected_candidate
            self._pinned_reference = self._reference_request(ObserveObjectReference.Request(
                operation=ObserveObjectReference.Request.PIN,
                source_snapshot_id=candidate.snapshot_id,
                source_object_id=candidate.target_object.object_id))
        self._execute_grasp_and_lift(selected, attempt)
        if self._held_object is None:
            raise RuntimeError('No settled grasp attachment reference')
        self._held_object.target = target
        return self._held_object

    def _ensure_fresh_head_before_wrist(self, selected, attempt):
        maximum = float(self.get_parameter('sorting_head_reference_refresh_age_sec').value)
        if not math.isfinite(maximum) or not 0 < maximum <= 30.0:
            raise ValueError('Head reference refresh age must be in (0, 30] seconds')
        def age(selection):
            stamp = selection.selected_candidate.header.stamp
            captured = stamp.sec*10**9 + stamp.nanosec
            now = self.get_clock().now().nanoseconds
            if captured <= 0 or captured > now:
                raise RuntimeError('Invalid head reference timestamp before wrist handoff')
            return (now-captured)/1e9
        if age(selected) <= maximum:
            return selected, attempt
        self.get_logger().info('Refreshing aged head reference before wrist handoff')
        self._switch_camera('head')
        refreshed, fresh_attempt = self._refresh_selected_grasp(selected, attempt)
        if age(refreshed) > maximum:
            raise RuntimeError('Refreshed head observation expired during replanning')
        previous = self._policy.classify_model(attempt.label, attempt.confidence,
            attempt.sorting_category, attempt.sorting_reason)
        current = self._policy.classify_model(fresh_attempt.label, fresh_attempt.confidence,
            fresh_attempt.sorting_category, fresh_attempt.sorting_reason)
        if (current.category == Category.REVIEW or current.category != previous.category
                or current.destination != previous.destination):
            raise RuntimeError('Object classification changed during head reference refresh')
        return refreshed, fresh_attempt

    def _on_lift_motion_complete(self) -> None:
        self._lift_completion_stamp_ns = self.get_clock().now().nanoseconds

    def _on_grasp_contact(self, selected, attempt) -> None:
        # The actual settled TCP, not the requested grasp endpoint, defines the
        # same rigid attachment assumption that MoveIt will use immediately next.
        tcp = self._tcp_pose(selected.selected_arm)
        object_center = (
            selected.selected_candidate.target_object.obb_pose.position)
        if getattr(self, '_wrist_enabled', False):
            self._contact_tcp_quaternion = deepcopy(tcp.orientation)
        offset = rotation(tcp).T @ (
            np.array((object_center.x, object_center.y, object_center.z))
            - np.array(self._pose_position(tcp)))
        self._held_object = HeldObject(SortTarget(attempt, selected), selected, offset)
        if getattr(self, '_async_carry', False):
            self._contact_loss_guard.missing_since = None
            self._carry_guard.arm(self.get_clock().now().nanoseconds, time.monotonic())
            self._check_motion_guard()
            self.get_logger().info('Continuous carry guard armed after stable gripper contact')

    def _verify_lift_height(self, attempt, *, minimum_center_z_m=None):
        if getattr(self, '_wrist_enabled', False):
            held = self._held_object
            if held is None:
                raise RuntimeError('Missing held geometry for wrist check')
            self._require_held_contact(held)
            tcp=self._tcp_pose(held.selected.selected_arm)
            predicted=np.array(self._pose_position(tcp))+rotation(tcp)@held.offset_in_tcp
            if minimum_center_z_m is not None and predicted[2] < minimum_center_z_m:
                raise RuntimeError('Kinematic lift clearance below required height')
            if getattr(self, '_async_carry', False):
                self._check_motion_guard()
                self.get_logger().info('Continuous carry: no stop-and-check wrist RPC after lift')
                return
            result = self._observe_wrist(held.selected, ObserveWristTarget.Request.CHECK, held=True)
            self._require_held_contact(held)
            self.get_logger().info('Wrist RGB + gripper contact consistent after lift; '
                                   'height is kinematic, not an independent depth measurement')
            return result
        use_reference = bool(self.get_parameter('sorting_use_reference_observation').value)
        verify = self._verify_reference_height if use_reference else super()._verify_lift_height
        try:
            inspected = verify(
                attempt, minimum_center_z_m=minimum_center_z_m)
        except LiftRedetectionError:
            if not self.get_parameter('sorting_reobserve_after_lift').value:
                raise
            self._reobserve_held_object(minimum_center_z_m)
            # Exactly one physical recovery move; no recursive motion/retries.
            inspected = verify(
                attempt, minimum_center_z_m=minimum_center_z_m)
        held = self._held_object
        if inspected is None or held is None:
            raise RuntimeError('Sorting requires observed lift and attachment reference')
        self._record_pipeline_message('lift_inspection', inspected)
        frame = held.selected.selected_candidate.header.frame_id
        header = inspected.header if use_reference else inspected.objects.header
        if header.frame_id != frame:
            raise RuntimeError('Held-object observation frame differs from attachment')
        tcp = self._tcp_pose(held.selected.selected_arm)
        predicted = np.array(self._pose_position(tcp)) + rotation(tcp) @ held.offset_in_tcp
        observed = (inspected.observed_center if use_reference
                    else inspected.objects.objects[0].obb_pose.position)
        tolerance = float(self.get_parameter('sorting_held_association_tolerance_m').value)
        if not math.isfinite(tolerance) or tolerance <= 0.:
            raise ValueError('Held-object association tolerance must be finite and positive')
        error = np.linalg.norm(predicted - np.array((observed.x, observed.y, observed.z)))
        if not math.isfinite(error) or error > tolerance:
            raise RuntimeError(f'Observed lifted object does not match held geometry: {error:.4f}m')
        return inspected

    def _reference_request(self, request):
        timeout = float(self.get_parameter('sorting_reference_timeout_sec').value)
        if not math.isfinite(timeout) or timeout <= 0.:
            raise ValueError('Reference service timeout must be finite and positive')
        self._record_pipeline_message('reference_request', request)
        response = self._future(self._reference_client.call_async(request), timeout,
                                'SAM2 reference observation')
        self._record_pipeline_message('reference_response', response)
        if not response.success:
            if request.operation == request.OBSERVE and response.error_code == response.ERROR_MASK:
                raise LiftRedetectionError(response.message)
            raise RuntimeError(f'Reference observation: {response.message}')
        return response

    def _verify_reference_height(self, attempt, *, minimum_center_z_m=None):
        reference, held = self._pinned_reference, self._held_object
        if reference is None or held is None:
            raise RuntimeError('Lift reference was not pinned before grasp')
        minimum = float(self.get_parameter('lift_min_center_z_m').value)
        if not math.isfinite(minimum) or (minimum_center_z_m is not None
                                         and not math.isfinite(minimum_center_z_m)):
            raise ValueError('Observed lift height must be finite')
        if minimum_center_z_m is not None:
            minimum = max(minimum, minimum_center_z_m)
        self._wait_arm_stationary(held.selected.selected_arm)
        after_stamp = self.get_clock().now().nanoseconds
        observed = self._reference_request(ObserveObjectReference.Request(
            operation=ObserveObjectReference.Request.OBSERVE,
            reference_id=reference.reference_id, after_stamp_ns=after_stamp))
        capture = Time.from_msg(observed.header.stamp).nanoseconds
        if (observed.reference_id != reference.reference_id
                or observed.source_snapshot_id != reference.source_snapshot_id
                or observed.source_object_id != reference.source_object_id
                or observed.source_capture_stamp_ns != reference.source_capture_stamp_ns
                or observed.source_label != reference.source_label
                or observed.source_label != attempt.label
                or observed.source_confidence != reference.source_confidence
                or capture <= max(after_stamp, reference.source_capture_stamp_ns)):
            raise RuntimeError('Held observation identity or capture provenance differs from reference')
        center_z = observed.observed_center.z
        self.get_logger().info(
            f'SAM2 reference lift: label={observed.source_label} source_confidence='
            f'{observed.source_confidence:.3f} visible_center_z={center_z:.4f}m '
            f'minimum={minimum:.4f}m depth_points={observed.valid_depth_points}')
        self._require_held_contact(held)
        if not math.isfinite(center_z) or center_z < minimum:
            raise RuntimeError(f'{attempt.label} was not retained after lift: '
                               f'observed surface center {center_z:.4f}m < {minimum:.4f}m')
        return observed

    def _held_center_joints(self, held: HeldObject, center: np.ndarray,
                            tolerance_m: float) -> JointState | None:
        arm = held.selected.selected_arm
        pose = self._tcp_pose(arm)
        self._transport_adapter.set_current_state(self._feedback_state())
        solution = None
        for _ in range(5):
            target = center - rotation(pose) @ held.offset_in_tcp
            solution = self._transport_adapter.solve_position_ik(
                arm, tuple(float(v) for v in target), solution)
            if solution is None:
                self.get_logger().info(f'Held-center IK unavailable: target={target.tolist()}')
                return None
            joints = JointState(name=list(solution.names), position=list(solution.positions))
            pose = self._tcp_pose(arm, joints)
            predicted = np.array(self._pose_position(pose)) + rotation(pose) @ held.offset_in_tcp
            if np.linalg.norm(predicted-center) < tolerance_m:
                valid = self._transport_adapter.state_is_valid(arm, solution)
                if not valid:
                    self.get_logger().info('Held-center IK rejected by collision check')
                return joints if valid else None
        self.get_logger().info(
            f'Held-center fixed-point IK did not converge: error_m={np.linalg.norm(predicted-center):.5f}')
        return None

    def _require_held_contact(self, held: HeldObject) -> None:
        if getattr(self, '_async_carry', False) and self._carry_guard.armed:
            self._check_motion_guard()
            return
        if not self._gripper_contact_stalled(
                held.selected.selected_arm,
                float(self.get_parameter('gripper_open_position_rad').value),
                self._candidate_close_position(held.selected.selected_candidate)):
            raise RuntimeError('Held-object reobservation lost settled gripper contact')

    def _reobserve_held_object(self, minimum_center_z_m: float | None) -> None:
        held, info = self._held_object, self._camera_info
        if held is None or info is None:
            raise RuntimeError('Reobservation needs camera calibration and settled attachment')
        self._require_held_contact(held)
        arm = held.selected.selected_arm
        self._wait_arm_stationary(arm)
        transform = self._tf_buffer.lookup_transform(
            held.selected.selected_candidate.header.frame_id,
            info.header.frame_id, Time()).transform
        q, t = transform.rotation, transform.translation
        camera = CameraProjection(
            float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5]),
            (t.x, t.y, t.z), tuple(rotation_matrix_from_quaternion(q.x, q.y, q.z, q.w).flat))
        tcp = self._tcp_pose(arm)
        center = np.array(self._pose_position(tcp)) + rotation(tcp) @ held.offset_in_tcp
        size = held.selected.selected_candidate.target_object.obb_size
        padding = float(self.get_parameter('sorting_reobserve_geometry_padding_m').value)
        if not math.isfinite(padding) or padding < 0.:
            raise ValueError('Reobservation padding must be finite and nonnegative')
        radius = held_bounding_radius(self, held) + padding
        minimum = max(float(self.get_parameter('lift_min_center_z_m').value),
                      minimum_center_z_m if minimum_center_z_m is not None else -math.inf)
        centers = reobservation_centers(
            camera, info.width, info.height, center, radius,
            minimum_z_m=minimum,
            margin_px=float(self.get_parameter('sorting_reobserve_margin_px').value),
            maximum_translation_m=float(self.get_parameter('sorting_reobserve_max_translation_m').value),
            maximum_lowering_m=float(self.get_parameter('sorting_reobserve_max_lowering_m').value),
            height_clearance_m=float(self.get_parameter('sorting_reobserve_height_clearance_m').value))
        self.get_logger().info(
            f'Held reobservation: predicted_center={center.tolist()} radius={radius:.4f}m '
            f'frustum_candidates={[c.tolist() for c in centers]}')
        if bool(self.get_parameter('require_sensor_scene').value):
            self._wait_for_sensor_scene(float(self.get_parameter('attachment_scene_timeout_sec').value))
        for candidate in centers:
            joints = self._held_center_joints(held, candidate, 0.005)
            if joints is None:
                self.get_logger().info(f'Reobservation IK rejected: {candidate.tolist()}')
                continue
            self._move_to(arm, joints, 'held-object camera reobservation')
            self._verify_feedback(joints)
            self._hold('lift_hold_sec')
            self._require_held_contact(held)
            return
        raise RuntimeError('No collision-valid held-object reobservation pose in camera frustum')

    def _feedback_state(self):
        state = JointState()
        state.name = list(REQUIRED_JOINT_NAMES)
        state.position = [self._joint_positions[n] for n in state.name]
        return state

    def transport(self, held: HeldObject, destination: str):
        self._require_held_contact(held)
        arm = held.selected.selected_arm
        bin_ = self._bins[destination]
        size = held.selected.selected_candidate.target_object.obb_size
        # Bounding sphere accounts for object rotation during transport.
        radius = held_bounding_radius(self, held)
        clearance = self.get_parameter('sorting_release_clearance_m').value
        if bin_.kind == 'table_zone':
            # A tabletop handoff is not a drop into a deep bin. Keep the
            # rotation-safe sphere and >10mm IK tolerance margin, but reduce
            # free fall relative to the legacy bin-release clearance.
            clearance = float(self.get_parameter('sorting_table_release_clearance_m').value)
            if not math.isfinite(clearance) or clearance <= .01:
                raise ValueError('Table release clearance must exceed 10mm IK tolerance')
            # Candidate slots are calibrated station geometry; sizes/occupied
            # footprints come from perception and completed releases, not GT.
            selected_center = None
            slots = [(float(x), bin_.center_xy[1]) for x in np.arange(
                bin_.center_xy[0]-bin_.outside_size[0]/2+radius+.035,
                bin_.center_xy[0]+bin_.outside_size[0]/2-radius-.025, .08)]
            slots.extend(table_placement_slots(bin_.center_xy, bin_.outside_size[:2], float(radius)))
            if destination == 'handoff_center':
                # A handoff must be regraspable by the other arm. Prefer the
                # station center, not its robot-facing edge next to the base.
                slots.sort(key=lambda xy: sum((a-b)**2 for a, b in zip(xy, bin_.center_xy)))
            for x, y in slots:
                self._require_held_contact(held)
                center = np.array((x, y, bin_.top_z+radius+clearance))
                if any(np.linalg.norm(center[:2]-old[:2]) < radius+old_radius+.035
                       for old, old_radius in self._placed_footprints):
                    continue
                source = held.selected.selected_candidate.target_object.obb_pose.position
                if any(np.linalg.norm(old[:2]-np.array((source.x,source.y))) > .03
                       and np.linalg.norm(center[:2]-old[:2]) < radius+old_radius+.02
                       for old, old_radius in self._observed_footprints):
                    continue
                joints = self._held_center_joints(held, center, .01)
                if joints is not None:
                    selected_center = center
                    break
                self.get_logger().info(f'Table placement IK rejected: center={center.tolist()}')
            if selected_center is None:
                raise RuntimeError('No reachable unoccupied table-zone placement slot')
            self._current['placement_center_m'] = selected_center.tolist()
            self._move_to(arm, joints, f'transport to {destination}')
            self._verify_feedback(joints)
            self._require_held_contact(held)
            return
        center = np.array((*bin_.center_xy, bin_.top_z + radius + clearance))
        pose = self._tcp_pose(arm)
        self._transport_adapter.set_current_state(self._feedback_state())
        solution = None
        for _ in range(5):
            tcp_target = center - rotation(pose) @ held.offset_in_tcp
            solution = self._transport_adapter.solve_position_ik(
                arm, tuple(float(v) for v in tcp_target), solution)
            if solution is None:
                raise RuntimeError(f'No transport IK for {destination}')
            joints = JointState(name=list(solution.names),
                                position=list(solution.positions))
            pose = self._tcp_pose(arm, joints)
            actual = (np.array(self._pose_position(pose))
                      + rotation(pose) @ held.offset_in_tcp)
            if np.linalg.norm(actual - center) < 0.015:
                break
        else:
            raise RuntimeError('Transport IK did not center object over bin')
        if not self._transport_adapter.state_is_valid(arm, solution):
            raise RuntimeError('Release configuration is in collision')
        self._move_to(arm, joints, f'transport to {destination}')
        self._verify_feedback(joints)
        self._require_held_contact(held)

    def release(self, held: HeldObject, destination: str):
        arm = held.selected.selected_arm
        pose = self._tcp_pose(arm)
        center = (np.array(self._pose_position(pose))
                  + rotation(pose) @ held.offset_in_tcp)
        bin_ = self._bins[destination]
        size = held.selected.selected_candidate.target_object.obb_size
        radius = held_bounding_radius(self, held)
        if (any(abs(center[i] - bin_.center_xy[i]) + radius
                >= bin_.outside_size[i] / 2 - bin_.wall for i in range(2))
                or center[2] - radius <= bin_.top_z):
            raise RuntimeError('Object is not safely above the bin opening')
        if getattr(self, '_async_carry', False):
            self._check_motion_guard()
            self._carry_guard.disarm()  # Intentional release must not be diagnosed as accidental loss.
            self.get_logger().info('Carry guard disarmed at verified bin opening for intentional release')
        self._open_gripper(arm)
        if bin_.kind == 'table_zone' and destination != 'handoff_center':
            self._placed_footprints.append((center.copy(), float(radius)))
        self._release_stamp_ns = self.get_clock().now().nanoseconds
        self._hold('grasp_settle_sec')
        self._execution_scene.restore()
        self._held_object = None
        if getattr(self, '_wrist_enabled', False) and self._wrist_reference is not None:
            candidate=held.selected.selected_candidate
            request=ObserveWristTarget.Request(operation=ObserveWristTarget.Request.CLEAR,
                arm=arm,reference_id=self._wrist_reference.reference_id,
                source_snapshot_id=candidate.snapshot_id,source_object_id=candidate.target_object.object_id)
            response=self._future(self._wrist_client.call_async(request),3.,'clear wrist reference')
            if not response.success:
                raise RuntimeError(f'Could not clear wrist reference: {response.message}')
            self._wrist_reference=None
        if self._pinned_reference is not None:
            self._reference_request(ObserveObjectReference.Request(
                operation=ObserveObjectReference.Request.CLEAR,
                reference_id=self._pinned_reference.reference_id))
            self._pinned_reference = None

    def retreat(self, held: HeldObject, destination: str):
        arm = held.selected.selected_arm
        if self._held_object is not None:
            raise RuntimeError('Cannot close the returning gripper before release completes')
        joints = deepcopy(self._home[arm])
        closing = float(self.get_parameter('sorting_return_gripper_position_rad').value)
        if not math.isfinite(closing):
            raise ValueError('Return gripper position must be finite')
        joints.name.append(f'{arm}_gripper_joint')
        joints.position.append(closing)
        joints.velocity, joints.effort = [], []
        self.get_logger().info(f'Return with simultaneous {arm} gripper closing')
        self._move_to(arm, joints, f'return from {destination}')
        self._verify_feedback(joints)
        if self._wrist_enabled:
            self._switch_camera('head')

    def _switch_camera(self, camera: str):
        if not self._camera_client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Camera rate control service unavailable')
        request = SetParameters.Request(parameters=[Parameter('active_camera', value=camera).to_parameter_msg()])
        response = self._future(self._camera_client.call_async(request), 3., 'camera source/rate change')
        if len(response.results)!=1 or not response.results[0].successful:
            raise RuntimeError('Camera source/rate change rejected')
        self.get_logger().info(f'Active inspection camera={camera}; head runs at configured background rate during wrist use')

    def _observe_wrist(self, selected, operation, *, held=False):
        candidate = selected.selected_candidate
        obj = candidate.target_object
        request = ObserveWristTarget.Request(operation=operation, arm=selected.selected_arm,
            source_snapshot_id=candidate.snapshot_id, source_object_id=obj.object_id,
            label=obj.label, source_confidence=obj.confidence,
            reference_id=self._wrist_reference.reference_id if self._wrist_reference else '',
            size=deepcopy(obj.obb_size), after_stamp_ns=self.get_clock().now().nanoseconds)
        if operation == ObserveWristTarget.Request.CHECK:
            completed = getattr(self, '_lift_completion_stamp_ns', None)
            if completed is None or not 0 < completed <= request.after_stamp_ns:
                raise RuntimeError('Missing or invalid lift completion freshness barrier')
            # Results captured during the existing stabilization hold are valid
            # post-lift evidence. Do not force another full frame after the hold.
            request.after_stamp_ns = completed
        request.expected_pose.header=deepcopy(candidate.header)
        request.expected_pose.pose=deepcopy(obj.obb_pose)
        if held:
            tcp = self._tcp_pose(selected.selected_arm)
            center = np.array(self._pose_position(tcp))+rotation(tcp)@self._held_object.offset_in_tcp
            request.expected_pose.pose.position.x, request.expected_pose.pose.position.y, request.expected_pose.pose.position.z = map(float,center)
            from cleany_skill_executor.core.wrist_pose import carried_orientation
            request.expected_pose.pose.orientation = carried_orientation(
                self._contact_tcp_quaternion, tcp.orientation, obj.obb_pose.orientation)
        self._record_pipeline_message('wrist_request', request)
        started = time.monotonic()
        response = self._future(self._wrist_client.call_async(request), 30., 'wrist RGB verification')
        self.get_logger().info(f'WRIST RPC operation={operation} elapsed_sec={time.monotonic()-started:.3f} '
                               f'after_ns={request.after_stamp_ns}')
        self._record_pipeline_message('wrist_response', response)
        if (not response.success or response.source_snapshot_id != candidate.snapshot_id
                or response.source_object_id != obj.object_id):
            raise RuntimeError(f'Wrist RGB verification failed: {response.message}')
        stamp=response.header.stamp.sec*10**9+response.header.stamp.nanosec
        if (response.header.frame_id != f'{selected.selected_arm}_wrist_rgb_optical_frame'
                or stamp <= request.after_stamp_ns or not response.reference_id
                or (operation==ObserveWristTarget.Request.CHECK and
                    response.reference_id != request.reference_id)):
            raise RuntimeError('Wrist response source/frame/timestamp mismatch')
        return response

    def verify_placement(self, target: SortTarget, destination: str) -> bool:
        request = VerifyPlacement.Request(
            label=target.attempt.label, destination_id=destination,
            after_stamp_ns=self._release_stamp_ns)
        deadline = time.monotonic() + self.get_parameter(
            'sorting_verification_timeout_sec').value
        while time.monotonic() < deadline:
            response = self._future(self._verification.call_async(request),
                                    2.0, 'placement verification')
            if response.success:
                self.get_logger().info(response.message)
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _register_bins(self):
        # Known collection-station calibration only. Desk and item geometry
        # still comes from the depth sensor, never from the MuJoCo scene.
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        for bin_ in self._bins.values():
            if bin_.kind == 'table_zone':
                continue
            item = CollisionObject(id=bin_.name, operation=CollisionObject.ADD)
            item.header.frame_id = 'base_link'
            for _, size, center in bin_.boxes():
                item.primitives.append(SolidPrimitive(
                    type=SolidPrimitive.BOX, dimensions=list(size)))
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = center
                pose.orientation.w = 1.0
                item.primitive_poses.append(pose)
            scene.world.collision_objects.append(item)
        for name, size, center in load_shelf_boxes(
                self.get_parameter('sorting_bins_config').value):
            item = CollisionObject(id=name, operation=CollisionObject.ADD)
            item.header.frame_id = 'base_link'
            item.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = center
            pose.orientation.w = 1.0
            item.primitive_poses.append(pose)
            scene.world.collision_objects.append(item)
        if not self._apply_scene.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Planning scene is unavailable')
        request = ApplyPlanningScene.Request(scene=scene)
        response = self._future(
            self._apply_scene.call_async(request),
            5.0, 'bin collision scene')
        if not response.success:
            raise RuntimeError('Could not register collection bins')


def main(args=None):
    rclpy.init(args=args)
    node = SortingCoordinator()
    failed = False
    try:
        node.run()
        if not node.get_parameter('sorting_exit_on_finish').value:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        failed = True
        node._stage('failed', error=str(error))
        node.get_logger().fatal(f'SORTING FAILED: {error}')
        try:
            if not node.get_parameter('sorting_exit_on_finish').value:
                rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise SystemExit(1)
