"""Explicit physical collection entrypoint. Default invocation audits config only."""
from __future__ import annotations
import argparse
from collections.abc import Callable
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from cleany_interfaces.action import SelectReachableGrasp
from cleany_interfaces.srv import CheckManipulation
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints, MoveItErrorCodes,
                            JointConstraint, PlanningScene, PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPositionFK, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .core.collection import (RETURN_TO_PICK, PickupReturnPolicy, configuration_issues, load_arm,
                              check_evidence, contact_verification_required,
                              grasp_identity_verification_required, temporary_return_mode)
from .core.cartesian import (
    CartesianPose, bounded_cartesian_step, coordinate_median, estimate_measured_path_pose,
    grasp_capture_metrics, observation_corrected_pose, orientation_error_rad,
    validate_corridor_samples, validate_pose_endpoint, validate_position_endpoint,
)
from .core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution, PlannedPath, PoseValue
from .core.alignment import rotation_from_quaternion, quaternion_from_rotation
from .core.approach_path import (
    ApproachRoute, CartesianPathUnavailable, approach_detours, validate_detour_samples,
)
from .core.hardware_execution import (
    CollisionValidationError, CommandPathValidationError, JointRangeValidationError,
)
from .core.joint_path import executable_joint_path
from .core.height_correction import (
    HeightCorrection, HeightCorrectionPolicy, height_correction_enabled,
    validate_height_correctable_endpoint,
)
from .core.recorded_trajectory import (
    descending_height_time, reversed_command_history, validate_command_history,
)
from .core.fixed_jaw_clearance import closing_clearance
from .core.urdf_fk import UrdfChain
from .fixed_jaw_geometry import load_fixed_jaw_bounds, target_box
from .planning_scene import TargetSceneTransaction, collision_pair_is_allowed
from .pregrasp_pipeline import (
    PregraspPipeline, ReferenceObservationFailure, reference_observation_metrics,
)
from .trajectory_io import path_to_message


def return_hold_mode(config: dict) -> bool:
    return config.get('operation_mode') == 'grasp_lift_return_hold'


def hold_mode(config: dict) -> bool:
    return config.get('operation_mode') in ('grasp_lift_hold', 'grasp_lift_return_hold')


def observation_center(observation) -> tuple[float, float, float]:
    return (
        float(observation.observed_center.x),
        float(observation.observed_center.y),
        float(observation.observed_center.z),
    )


def observation_stamp_ns(observation) -> int:
    return observation.header.stamp.sec*10**9+observation.header.stamp.nanosec


def observation_extent_ratio(left, right) -> float:
    left_values = (
        float(left.observed_extent.x),
        float(left.observed_extent.y),
        float(left.observed_extent.z),
    )
    right_values = (
        float(right.observed_extent.x),
        float(right.observed_extent.y),
        float(right.observed_extent.z),
    )
    if (not all(math.isfinite(value) and value > 0
                for value in (*left_values, *right_values))):
        raise RuntimeError('Visual observation extent is invalid')
    return max(
        max(left_value/right_value, right_value/left_value)
        for left_value, right_value in zip(
            left_values, right_values, strict=True)
    )


class CollectionPipeline(PregraspPipeline):
    """One process owns serial motion ordering; MoveIt only returns plans."""
    def __init__(self, config: dict, **kwargs):
        issues = configuration_issues(config)
        if issues:
            raise ValueError('; '.join(issues))
        super().__init__(mode='hardware', **kwargs)
        if not self.scene_monitor.required:
            raise ValueError('Physical collection requires require_sensor_scene:=true')
        self.config = config
        self.evidence = self.create_client(CheckManipulation, '/manipulation/check_evidence')
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.joint_plan = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.apply_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.carrying = None
        self.payload_future = None
        self.payload_request = None
        self.payload_evidence_time = 0.
        self.wrist_reference_id = ''
        self.wrist_request = None
        self.payload_selected = None
        self.releasing = False
        self.events = []
        self.destination = ''
        self.arm_config = None

    @property
    def hold_mode(self) -> bool:
        return hold_mode(self.config)

    def wrist_observation_required(self) -> bool:
        return bool(self.config.get('wrist_observation_enabled', True))

    def head_reobservation_required(self) -> bool:
        return not self.config.get('assume_stationary_target', False)

    def execute_pregrasp(self, selected):
        if self.config.get('pregrasp_motion_enabled', True):
            return super().execute_pregrasp(selected)
        # Opening the jaw is still required before entering the grasp pose.
        # Do not reuse the selected current->pregrasp or pregrasp->grasp path.
        self.scene_monitor.wait()
        arm = selected.selected_arm
        width = min(self.arm_config.width, float(selected.selected_candidate.required_opening_m)
                    + self.config['pregrasp_opening_margin_m'])
        opening = self.arm_config.opening(width)
        self.jaw(arm, opening, 'open_before_direct_grasp',
                 check_collisions=self.config.get('gripper_opening_collision_check_enabled', True))
        self.motion.set_gripper_position(arm, opening)
        self.event('pregrasp_motion_skipped', arm=arm)
        state = self.real.monitor()
        return state.header.stamp.sec * 10**9 + state.header.stamp.nanosec

    def confirm_wrist_target(self, report: dict, step: int) -> dict:
        """Confirm the fixed head-camera target from a fresh wrist RGB frame."""
        if self.wrist_request is None or not self.wrist_reference_id:
            raise RuntimeError('Wrist target handoff is unavailable')
        request = deepcopy(self.wrist_request)
        request.operation = request.CHECK
        request.reference_id = self.wrist_reference_id
        response = self.call(self.wrist, request, 60.)
        if (response.reference_id != self.wrist_reference_id
                or response.source_snapshot_id != request.source_snapshot_id
                or response.source_object_id != request.source_object_id):
            raise RuntimeError('Wrist tracking returned a different target identity')
        stamp = response.header.stamp.sec*10**9+response.header.stamp.nanosec
        if stamp <= request.after_stamp_ns:
            raise RuntimeError('Wrist tracking did not advance during head-camera occlusion')
        self.wrist_request.after_stamp_ns = stamp
        report['wrist_checks'] = int(report.get('wrist_checks', 0))+1
        result = {
            'reference_id': response.reference_id,
            'stamp_ns': stamp,
            'source_snapshot_id': response.source_snapshot_id,
            'source_object_id': response.source_object_id,
        }
        self.event('visual_wrist_occlusion_confirmed', step=step, **result)
        return result

    def target_translation_limit(self) -> float:
        if self.config.get('visual_servo_enabled', False):
            return float(self.config['visual_servo_maximum_correction_m'])
        return super().target_translation_limit()

    def validate_target_translation(self, selected, before, after,
                                    displacement: float, report: dict) -> None:
        if not self.config.get('visual_servo_enabled', False):
            return super().validate_target_translation(
                selected, before, after, displacement, report)
        if displacement > self.target_translation_limit():
            # Do not stop on a single head-camera surface centroid.  The visual
            # approach will recapture it and either reject a confirmed move or
            # finish from the last stable target once the TCP is in the grasp
            # capture corridor.
            self.event(
                'visual_target_translation_deferred',
                measured_translation_m=displacement,
                soft_limit_m=self.target_translation_limit(),
            )

    def monitor_payload(self):
        if not contact_verification_required(self.config):
            return
        if self.carrying is None or self.releasing:
            return
        from .core.gripper_contact import ContactState
        decision = self.real.contact_decision(self.payload_selected.selected_arm)
        if decision.state == ContactState.CONTACT:
            self.payload_evidence_time = time.monotonic()
        elif decision.state == ContactState.EMPTY:
            raise RuntimeError('Held-object motor contact was lost')
        if time.monotonic()-self.payload_evidence_time > self.config['evidence_max_age_sec']:
            raise RuntimeError('Held-object motor evidence became unknown or stale: '+decision.reason)

    def wait(self, future, timeout, *, monitor=False, cancel_on_timeout=True):
        return super().wait(future,timeout,monitor=monitor or self.carrying is not None,
                            cancel_on_timeout=cancel_on_timeout)

    def event(self, stage: str, **fields):
        item = {'stage': stage, 'stamp_ns': self.get_clock().now().nanoseconds, **fields}
        self.events.append(item)
        self.get_logger().info(json.dumps(item))

    def capture_pickup(self, arm: str) -> None:
        """Save the measured pickup and its release goal before closing the jaw."""
        self.pickup_return_pose = None
        self._pickup_departure_paths = self._pickup_departure_arm = None
        state = self.real.monitor()
        positions = dict(zip(state.name, state.position))
        self.pickup_joints = tuple(positions[name] for name in ARM_JOINT_NAMES[arm])
        if not all(math.isfinite(q) for q in self.pickup_joints):
            raise RuntimeError('Original pickup position requires finite measured joints')
        policy = PickupReturnPolicy.from_config(self.config)
        if policy.height_offset_m > 0:
            pickup = self.cartesian_pose(self.pose(arm))
            self.pickup_return_pose = self.pose_message(CartesianPose(
                (*pickup.position[:2], pickup.position[2] + policy.height_offset_m),
                pickup.orientation))
        self.pickup_arm = arm
        self.event('pickup_pose_captured', arm=arm, joint_positions=self.pickup_joints)

    def return_to_pickup(self, arm: str, pregrasp: Pose) -> dict:
        policy = PickupReturnPolicy.from_config(self.config)
        if policy.height_offset_m > 0:
            target = self.pickup_destination_pose(arm)
            goal = self.cartesian_pose(target)
            details = dict(destination_id=RETURN_TO_PICK, destination_frame='base_link',
                           destination_position_m=goal.position,
                           destination_orientation_xyzw=goal.orientation,
                           return_to_pick_height_offset_m=policy.height_offset_m,
                           pickup_joint_rad=self.pickup_joints)
            self.event('pickup_return_target', arm=arm, **details)
            if policy.reverse_enabled:
                details.update(self.reverse_to_pickup(arm, target, policy))
            else:
                self.linear(arm, target, 'return_to_pickup',
                            validate_command_path=True, check_collisions=True)
            self.check_pickup_arrival(arm)
            return details
        stage = ('lower_to_pregrasp' if self.config.get('pregrasp_motion_enabled', True)
                 else 'lower_to_pickup')
        self.linear(arm, pregrasp, stage)
        self.move_joint_pose(arm, self.pickup_joints, 'return_to_pickup')
        self.check_pickup_arrival(arm)
        return dict(destination_id=RETURN_TO_PICK, destination_joint_rad=self.pickup_joints)

    @contextmanager
    def record_pickup_departure(self, arm: str):
        """Retain the closed-jaw retreat, lift and every successful height correction."""
        if not PickupReturnPolicy.from_config(self.config).reverse_enabled:
            yield
            return
        self._pickup_departure_paths = self._pickup_departure_arm = None
        with self.real.record_arm_commands(arm) as recording:
            yield
        if recording.interrupted:
            raise CommandPathValidationError('Interrupted departure cannot authorize reverse return')
        validate_command_history(recording.trajectories, ARM_JOINT_NAMES[arm])
        self._pickup_departure_paths = tuple(recording.trajectories)
        self._pickup_departure_arm = arm
        self.event('pickup_departure_recorded', arm=arm,
                   trajectories=len(recording.trajectories),
                   command_points=sum(len(path.points) for path in recording.trajectories))

    def reverse_to_pickup(self, arm: str, target: Pose, policy: PickupReturnPolicy) -> dict:
        paths = getattr(self, '_pickup_departure_paths', None)
        if not paths or getattr(self, '_pickup_departure_arm', None) != arm:
            raise CommandPathValidationError('Reverse return has no recorded departure for this arm')
        # Consume once; a failed replay may have moved with the payload attached.
        self._pickup_departure_paths = self._pickup_departure_arm = None
        validate_command_history(paths, ARM_JOINT_NAMES[arm])
        state = self.real.wait_for_settle(ARM_JOINT_NAMES[arm])
        hold = self.real.arm_holds.get(arm)
        if hold is None or any(abs(a-b) > 1e-9 for a, b in zip(
                hold.command, paths[-1].points[-1].positions, strict=True)):
            raise CommandPathValidationError('Reverse return history no longer matches the held command')
        goal = self.cartesian_pose(target)
        self.motion.set_current_state(state)
        def fk(q):
            self.real.monitor()
            value = self.motion.forward_kinematics(arm, JointSolution(ARM_JOINT_NAMES[arm], q))
            return CartesianPose(value.position, value.orientation_xyzw)
        positions = dict(zip(state.name, state.position, strict=True))
        measured_joints = tuple(positions[name] for name in ARM_JOINT_NAMES[arm])
        actual = fk(measured_joints)
        commanded = fk(tuple(hold.command))
        position_tolerance = self.real.value('hardware_tcp_position_tolerance_m')
        orientation_tolerance = math.radians(self.real.value('hardware_tcp_orientation_tolerance_deg'))
        # The next movement starts at measured FK. The saved motor commands
        # supply relative displacement while retaining loaded-arm support.
        # Height correction deliberately makes their absolute TCP differ from
        # the physical goal; that difference is not an arrival failure here.
        residual = tuple(c-a for c, a in zip(commanded.position, actual.position, strict=True))
        reverse = reversed_command_history(paths)
        stop_ns = descending_height_time(reverse,
            lambda q: actual.position[2]+fk(q).position[2]-commanded.position[2],
            goal.position[2], step_rad=self.real.value('hardware_path_step_rad'))
        endpoint = fk(reverse.sample_at(stop_ns).positions)
        predicted = estimate_measured_path_pose(actual, commanded, endpoint)
        validate_pose_endpoint(predicted, goal, position_tolerance, orientation_tolerance)
        details = dict(return_route='reversed_departure',
                       return_start_source='measured',
                       return_measured_start_joint_rad=measured_joints,
                       return_measured_start_position_m=actual.position,
                       return_measured_start_orientation_xyzw=actual.orientation,
                       return_collision_checks_enabled=policy.check_collisions,
                       return_recorded_trajectories=len(paths),
                       return_source_duration_sec=stop_ns/1e9,
                       return_tracking_residual_m=residual,
                       return_command_endpoint_m=endpoint.position)
        self.event('return_to_pickup_replay', arm=arm, **details)
        cursor = 0
        corrections = []
        details['return_height_corrections'] = corrections
        for attempt in range(policy.correction_max_attempts+1):
            self.real.execute_return_segment(arm, reverse, cursor, stop_ns,
                                             check_collisions=policy.check_collisions,
                                             endpoint_pose=goal)
            cursor = stop_ns
            state = self.real.wait_for_settle(ARM_JOINT_NAMES[arm])
            actual = self.cartesian_pose(self.pose(arm))
            validate_pose_endpoint(actual, goal, position_tolerance, orientation_tolerance)
            error = actual.position[2]-goal.position[2]
            sample = dict(attempt=attempt, target_z_m=goal.position[2],
                          actual_z_m=actual.position[2], height_error_m=error)
            corrections.append(sample)
            self.event('pickup_return_height_checked', arm=arm, **sample)
            if abs(error) <= policy.height_tolerance_m+1e-9:
                break
            # A high arrival may continue down the saved path in small bounded
            # steps. Low arrivals, unrelated pose errors and faults stop here.
            if error < 0 or attempt == policy.correction_max_attempts:
                raise ValueError('Measured TCP did not reach the raised pickup release height: '
                                 f'target={goal.position[2]:.6f}m actual={actual.position[2]:.6f}m '
                                 f'error={error:.6f}m tolerance={policy.height_tolerance_m:.6f}m')
            self.motion.set_current_state(state)
            command = reverse.sample_at(cursor).positions
            hold = self.real.arm_holds.get(arm)
            if hold is None or any(abs(a-b) > 1e-9 for a,b in zip(command,hold.command,strict=True)):
                raise CommandPathValidationError('Return height correction lost its held command')
            command_pose = fk(command)
            step = min(error, policy.correction_max_step_m)
            stop_ns = descending_height_time(reverse, lambda q: fk(q).position[2], command_pose.position[2]-step,
                start_ns=cursor, step_rad=self.real.value('hardware_path_step_rad'))
            endpoint = fk(reverse.sample_at(stop_ns).positions)
            predicted = estimate_measured_path_pose(actual, command_pose, endpoint)
            validate_pose_endpoint(predicted, goal, position_tolerance, orientation_tolerance)
            self.event('pickup_return_height_correction', arm=arm, attempt=attempt+1,
                       step_m=step, actual_z_m=actual.position[2], target_z_m=goal.position[2])
        return details

    def pickup_destination_pose(self, arm: str) -> Pose:
        target = getattr(self, 'pickup_return_pose', None)
        if target is None or getattr(self, 'pickup_arm', None) != arm:
            raise ValueError('Raised pickup destination was not captured for this arm')
        self.cartesian_pose(target)  # Reject nonfinite positions/invalid orientations.
        return deepcopy(target)

    def pickup_release_retreat(self, arm: str, pregrasp: Pose) -> None:
        retreat = deepcopy(pregrasp)
        if not self.config.get('pregrasp_motion_enabled', True):
            retreat.position.z += self.config['lift_height_m']
        if PickupReturnPolicy.from_config(self.config).height_offset_m > 0:
            actual = self.cartesian_pose(self.pose(arm))
            retreat.position.z = max(retreat.position.z,
                                     self.pickup_destination_pose(arm).position.z,
                                     actual.position[2])
            target = self.cartesian_pose(retreat)
            if (math.dist(actual.position, target.position) < 1e-9
                    and orientation_error_rad(actual.orientation, target.orientation) < 1e-9):
                self.event('retreat_after_release_already_reached', arm=arm)
                return
        check_collisions = self.config.get('retreat_after_release_collision_check_enabled', True)
        if not check_collisions:
            self.event('retreat_after_release_collision_checks_disabled', arm=arm)
        self.linear(arm, retreat, 'retreat_after_release', check_collisions=check_collisions)

    def check_pickup_arrival(self, arm: str) -> None:
        """Confirm the configured release destination using fresh measured state."""
        policy = PickupReturnPolicy.from_config(self.config)
        if policy.height_offset_m > 0:
            target = self.cartesian_pose(self.pickup_destination_pose(arm))
            self.real.monitor()
            actual = self.cartesian_pose(self.pose(arm))
            validate_pose_endpoint(actual, target,
                                   self.real.value('hardware_tcp_position_tolerance_m'),
                                   math.radians(self.real.value('hardware_tcp_orientation_tolerance_deg')))
            # The general 30mm pose envelope alone would accept the old height
            # for a +20mm goal. Keep release strictly above the pickup height.
            height_error = actual.position[2] - target.position[2]
            if abs(height_error) > policy.height_tolerance_m + 1e-9:
                raise ValueError('Measured TCP did not reach the raised pickup release height: '
                                 f'target={target.position[2]:.6f}m actual={actual.position[2]:.6f}m '
                                 f'error={height_error:.6f}m tolerance={policy.height_tolerance_m:.6f}m')
            self.event('pickup_destination_reached', arm=arm,
                       target_position_m=target.position, actual_position_m=actual.position,
                       height_error_m=height_error)
            return
        target = getattr(self, 'pickup_joints', None)
        names = ARM_JOINT_NAMES[arm]
        if target is None or len(target) != len(names) or not all(math.isfinite(q) for q in target):
            raise ValueError('Original pickup destination was not captured')
        state = self.real.monitor()
        actual = dict(zip(state.name, state.position))
        tolerance = self.real.value('hardware_arrival_tolerance_rad')
        if (not math.isfinite(tolerance) or tolerance <= 0
                or any(name not in actual or not math.isfinite(actual[name])
                       or abs(actual[name] - q) > tolerance for name, q in zip(names, target))):
            raise ValueError('Measured arm did not return to the original pickup position')
        self.event('pickup_destination_reached', arm=arm, joint_positions=target)

    def before_execution(self, selected):
        self._approach_command_paths = None
        self._approach_command_arm = None
        arm = selected.selected_arm
        if (grasp_identity_verification_required(self.config)
                and not self.config.get('wrist_observation_enabled', True)):
            raise RuntimeError('Physical grasp verification requires wrist target observation')
        if arm not in self.config['arms']:
            raise RuntimeError('Selected arm is not commissioned/configured')
        self.arm_config = load_arm(
            self.config['arms'][arm], require_bins=self.config.get('operation_mode', 'full_collection') == 'full_collection'
        )
        full_collection = self.config.get('operation_mode', 'full_collection') == 'full_collection'
        if temporary_return_mode(self.config):
            self.destination = RETURN_TO_PICK
        if full_collection:
            self.destination = self.config['label_destinations'].get(
                selected.selected_object.label,
                self.config['label_destinations'].get('*', ''),
            )
            if not self.destination or self.destination not in self.arm_config.bins:
                raise RuntimeError('No approved destination for selected label')
        expected = self.arm_config.opening(float(selected.selected_candidate.required_opening_m))
        if abs(expected-selected.gripper_joint_position) > 1e-4:
            raise RuntimeError('Selector gripper mapping differs from physical calibration')
        limits = self.real.verify(arm)
        if self.arm_config.home is None:
            measured = self.real.monitor()
            positions = dict(zip(measured.name, measured.position))
            self.arm_config = replace(
                self.arm_config,
                home=tuple(positions[name] for name in ARM_JOINT_NAMES[arm]),
            )
            self.event('startup_home_captured', arm=arm, joint_positions=self.arm_config.home)
        poses = [self.arm_config.home]
        if full_collection:
            poses.append(self.arm_config.bins[self.destination])
        for pose in poses:
            for name,q in zip(ARM_JOINT_NAMES[arm],pose):
                limits[name].check(q)
        for q in (self.arm_config.closed, self.arm_config.opened):
            limits[f'{arm}_gripper_joint'].check(q)
        if contact_verification_required(self.config):
            self.real.closed_contact_position(arm, limits)
        if (grasp_identity_verification_required(self.config)
                and not self.evidence.wait_for_service(timeout_sec=2.)):
            raise RuntimeError('Physical contact/tracking/release evidence provider is unavailable')
        self.event('preflight', arm=arm, destination=self.destination,
                   operation_mode=self.config.get('operation_mode', 'full_collection'))

    def pose(self, arm: str, link: str | None = None):
        state = self.real.monitor()
        request = GetPositionFK.Request()
        request.header.frame_id = 'base_link'
        request.fk_link_names = [link or f'{arm}_grasp_tcp']
        request.robot_state.joint_state = state
        request.robot_state.is_diff = True
        response = self.call(self.fk, request)
        if response.error_code.val != 1 or len(response.pose_stamped) != 1:
            raise RuntimeError('Measured-state FK failed')
        return response.pose_stamped[0].pose

    @staticmethod
    def cartesian_pose(message: Pose) -> CartesianPose:
        return CartesianPose(
            (message.position.x, message.position.y, message.position.z),
            (message.orientation.x, message.orientation.y,
             message.orientation.z, message.orientation.w),
        )

    @staticmethod
    def pose_message(value: CartesianPose) -> Pose:
        message = Pose()
        (message.position.x, message.position.y,
         message.position.z) = value.position
        (message.orientation.x, message.orientation.y,
         message.orientation.z, message.orientation.w) = value.orientation
        return message

    @staticmethod
    def retarget_wrist_roll(trajectory: JointTrajectory, measured,
                            arm: str, target_roll: float | None) -> None:
        """Keep position-only Cartesian IK from choosing an arbitrary tool roll."""
        wrist = f'{arm}_wrist_roll_joint'
        if wrist not in trajectory.joint_names:
            raise RuntimeError('Cartesian response is missing wrist roll')
        positions = dict(zip(measured.name, measured.position))
        if wrist not in positions or not math.isfinite(positions[wrist]):
            raise RuntimeError('Measured wrist roll is unavailable')
        start = float(positions[wrist])
        target_roll = start if target_roll is None else float(target_roll)
        if not math.isfinite(target_roll):
            raise RuntimeError('Cartesian wrist-roll target is not finite')
        if len(trajectory.points) < 2:
            raise RuntimeError('Cartesian response requires at least two points')
        index = trajectory.joint_names.index(wrist)
        denominator = len(trajectory.points)-1
        for number, point in enumerate(trajectory.points):
            values = list(point.positions)
            if len(values) != len(trajectory.joint_names):
                raise RuntimeError('Cartesian response point width mismatch')
            fraction = number/denominator
            values[index] = start+(target_roll-start)*fraction
            point.positions = values

    @staticmethod
    def selected_grasp_roll(selected, arm: str) -> float:
        wrist = f'{arm}_wrist_roll_joint'
        positions = dict(zip(
            selected.grasp_joint_state.name,
            selected.grasp_joint_state.position,
        ))
        if wrist not in positions or not math.isfinite(positions[wrist]):
            raise RuntimeError('Selected grasp wrist roll is unavailable')
        return float(positions[wrist])

    @contextmanager
    def record_approach_commands(self, arm: str):
        if (not self.config.get('retreat_reverse_approach_enabled', False)
                or not self.config.get('pregrasp_motion_enabled', True)):
            yield
            return
        self._approach_command_paths = None
        self._approach_command_arm = None
        with self.real.record_arm_commands(arm) as recording:
            yield
        if recording.interrupted or not recording.trajectories:
            raise CommandPathValidationError('Approach has no complete command history for reverse retreat')
        names = ARM_JOINT_NAMES[arm]
        for trajectory in recording.trajectories:
            trajectory.validate()
            if trajectory.joint_names != names:
                raise CommandPathValidationError('Approach command history belongs to another arm')
        for first, second in zip(recording.trajectories, recording.trajectories[1:]):
            if any(abs(a-b) > 1e-9 for a, b in zip(
                    first.points[-1].positions, second.points[0].positions, strict=True)):
                raise CommandPathValidationError('Approach command history is discontinuous')
        self._approach_command_paths = tuple(recording.trajectories)
        self._approach_command_arm = arm
        self.event('approach_command_path_recorded', arm=arm,
                   trajectories=len(recording.trajectories),
                   command_points=sum(len(path.points) for path in recording.trajectories))

    def retreat_to_pregrasp(self, arm: str, target: Pose, report: dict | None = None, *,
                           check_collisions: bool = True):
        if type(check_collisions) is not bool:
            raise ValueError('check_collisions must be boolean')
        if not self.config.get('retreat_reverse_approach_enabled', False):
            return self.linear(arm, target, 'retreat', check_collisions=check_collisions)
        paths = getattr(self, '_approach_command_paths', None)
        if not paths or getattr(self, '_approach_command_arm', None) != arm:
            raise CommandPathValidationError('Reverse retreat has no recorded approach for this arm')
        measured = self.real.monitor()
        positions = dict(zip(measured.name, measured.position))
        wrist = f'{arm}_wrist_roll_joint'
        if wrist not in positions or not math.isfinite(positions[wrist]):
            raise CommandPathValidationError('Reverse retreat has no finite measured wrist roll')
        roll = float(positions[wrist])
        reversed_paths = [path.reversed_with_roll(wrist, roll) for path in reversed(paths)]
        # Consume once: a failed retreat may have moved with a payload attached.
        self._approach_command_paths = None
        self._approach_command_arm = None
        self.event('retreat_reverse_approach', arm=arm, trajectories=len(reversed_paths),
                   wrist_roll_rad=roll, collision_checks_enabled=check_collisions)
        actual = None
        defer_shortfall = height_correction_enabled(self.config)
        endpoint_options = {'defer_height_shortfall': True} if defer_shortfall else {}
        goal = self.cartesian_pose(target)
        for index, path in enumerate(reversed_paths):
            state = self.real.monitor()
            self.motion.set_current_state(state)
            positions = dict(zip(state.name, state.position, strict=True))
            def fk(q):
                value = self.motion.forward_kinematics(arm, JointSolution(ARM_JOINT_NAMES[arm], tuple(q)))
                return CartesianPose(value.position, value.orientation_xyzw)
            origin = fk(tuple(positions[name] for name in ARM_JOINT_NAMES[arm]))
            command_origin = fk(path.points[0].positions)
            predicted = estimate_measured_path_pose(origin, command_origin, fk(path.points[-1].positions))
            endpoint = goal if index == len(reversed_paths)-1 else predicted
            if index == len(reversed_paths)-1:
                if defer_shortfall:
                    validate_height_correctable_endpoint(predicted, goal,
                        self.real.value('hardware_tcp_position_tolerance_m'),
                        math.radians(self.real.value('hardware_tcp_orientation_tolerance_deg')))
                else:
                    validate_position_endpoint(predicted, goal,
                                               self.real.value('hardware_tcp_position_tolerance_m'))
            self.event('retreat_measured_start', arm=arm, start_source='measured', segment=index,
                       measured_joint_rad=[positions[name] for name in ARM_JOINT_NAMES[arm]],
                       measured_tcp_position_m=origin.position,
                       predicted_endpoint_position_m=predicted.position,
                       physical_target_position_m=endpoint.position)
            actual = self.real.execute_recorded_trajectory(
                arm, path, check_collisions=check_collisions, endpoint_pose=endpoint, **endpoint_options)
        try:
            actual_pose, goal = self.cartesian_pose(self.pose(arm)), self.cartesian_pose(target)
            if defer_shortfall:
                validate_height_correctable_endpoint(actual_pose, goal,
                    self.real.value('hardware_tcp_position_tolerance_m'),
                    math.radians(self.real.value('hardware_tcp_orientation_tolerance_deg')))
            else:
                validate_position_endpoint(actual_pose, goal,
                                           self.real.value('hardware_tcp_position_tolerance_m'))
        except ValueError as error:
            raise RuntimeError(f'retreat: measured pregrasp TCP target not reached: {error}') from error
        if report is not None:
            report.update(retreat_route='reversed_approach', retreat_wrist_roll_rad=roll,
                          retreat_recorded_trajectories=len(reversed_paths))
        self.event('retreat_reverse_approach_completed', arm=arm)
        return actual

    def correct_height_endpoint(self, arm: str, target: Pose, phase: str,
                                report: dict | None = None, *,
                                check_collisions: bool = True) -> None:
        """Correct measured lift shortfall once, without a final height gate."""
        policy = HeightCorrectionPolicy.from_config(self.config)
        if not policy.enabled:
            return
        if phase != 'lift':
            raise ValueError('Height correction is restricted to lift')
        goal = self.cartesian_pose(deepcopy(target))
        position_tolerance = self.real.value('hardware_tcp_position_tolerance_m')
        orientation_tolerance = math.radians(self.real.value('hardware_tcp_orientation_tolerance_deg'))
        history = []
        summary = {'target_z_m': goal.position[2], 'history': history, 'status': 'checking'}
        if report is not None:
            report.setdefault('height_corrections', {})[phase] = summary
        try:
            # Includes fresh feedback, fault, cancellation and payload gates.
            state = self.real.wait_for_settle(ARM_JOINT_NAMES[arm])
            actual = self.cartesian_pose(self.pose(arm))
            validate_height_correctable_endpoint(actual, goal, position_tolerance,
                orientation_tolerance)
            hold = self.real.arm_holds.get(arm)
            if hold is None:
                raise RuntimeError('Height correction requires a completed held arm command')
            self.motion.set_current_state(state)
            fk = self.motion.forward_kinematics(
                arm, JointSolution(ARM_JOINT_NAMES[arm], hold.command))
            # Retain the sent command's support against the tracking residual.
            command = CartesianPose(fk.position, fk.orientation_xyzw)
            correction = HeightCorrection(goal.position[2], command.position[2], policy)
            summary['initial_command_z_m'] = command.position[2]
            sample = {'attempt': 0, 'target_z_m': goal.position[2],
                      'command_z_m': command.position[2], 'actual_z_m': actual.position[2],
                      'error_m': goal.position[2] - actual.position[2],
                      'total_correction_m': 0.}
            history.append(sample)
            summary.update(sample)
            self.event('height_correction_measured', arm=arm, phase=phase, **sample)
            next_z = correction.next_command(actual.position[2])
            if next_z is None:
                summary['status'] = 'not_needed'
            else:
                command_waypoint = CartesianPose(
                    (command.position[0], command.position[1], next_z), command.orientation)
                step = next_z-command.position[2]
                # Physical planning starts at measured FK; motor support remains
                # separate from the requested physical upward increment.
                waypoint = self.pose_message(CartesianPose(
                    (*goal.position[:2], actual.position[2]+step), goal.orientation))
                self.event('height_correction_command', arm=arm, phase=phase,
                           attempt=1, target_z_m=goal.position[2], command_z_m=next_z,
                           actual_z_m=actual.position[2], correction_step_m=step,
                           collision_checks_enabled=check_collisions)
                self.linear(arm, waypoint, phase+'_height_correction',
                            check_orientation=True, check_collisions=check_collisions,
                            command_target=command_waypoint, validate_command_path=True)
                # Record the remaining height error without retrying or gating return.
                final = self.cartesian_pose(self.pose(arm))
                sample = {'attempt': 1, 'target_z_m': goal.position[2],
                          'command_z_m': next_z, 'actual_z_m': final.position[2],
                          'error_m': goal.position[2]-final.position[2],
                          'total_correction_m': step}
                history.append(sample)
                summary.update(sample, status='applied')
            self.event('height_correction_completed', arm=arm, phase=phase, **summary)
        except Exception as error:
            summary.update(status='failed', message=str(error))
            self.event('height_correction_failed', arm=arm, phase=phase, **summary)
            raise

    def _cartesian_trajectory(self, arm: str, measured: JointState,
                              waypoints: tuple[CartesianPose, ...], stage: str,
                              wrist_roll_target: float | None,
                              check_collisions: bool) -> JointTrajectory:
        """Plan in the explicitly supplied physical or motor-command frame."""
        request = GetCartesianPath.Request()
        request.header.frame_id = 'base_link'
        request.group_name = f'{arm}_grasp_arm'
        request.link_name = f'{arm}_grasp_tcp'
        request.start_state.joint_state = deepcopy(measured)
        request.start_state.is_diff = True  # retain payload in planning state
        request.waypoints = [self.pose_message(point) for point in waypoints]
        request.max_step = self.config['cartesian_step_m']
        request.jump_threshold = 2.
        request.revolute_jump_threshold = .15
        request.avoid_collisions = check_collisions
        response = self.call(self.cartesian, request, 30.)
        if not math.isfinite(response.fraction) or not 0 <= response.fraction <= 1:
            raise RuntimeError(f'{stage}: invalid Cartesian path fraction')
        if response.error_code.val != 1 or response.fraction < 1.-1e-6:
            if response.error_code.val in (
                    MoveItErrorCodes.SUCCESS, MoveItErrorCodes.PLANNING_FAILED,
                    MoveItErrorCodes.START_STATE_IN_COLLISION, MoveItErrorCodes.GOAL_IN_COLLISION,
                    MoveItErrorCodes.NO_IK_SOLUTION):
                raise CartesianPathUnavailable(
                    f'{stage}: complete Cartesian path unavailable: '
                    f'code={response.error_code.val} fraction={response.fraction:.6f}',
                    error_code=response.error_code.val, fraction=response.fraction)
            raise RuntimeError(f'{stage}: complete Cartesian path unavailable')
        trajectory = response.solution.joint_trajectory
        if tuple(trajectory.joint_names) != ARM_JOINT_NAMES[arm]:
            raise RuntimeError('Cartesian response joint order mismatch')
        # KDL is deliberately position-only for this five-axis arm.  The
        # unconstrained wrist roll does not move the TCP position, so the
        # Cartesian service may otherwise replace it with an arbitrary value.
        # Restore a continuous roll ending at the already validated grasp roll
        # (or hold the current roll for retreat/lift) before any validation or
        # command is allowed.
        CollectionPipeline.retarget_wrist_roll(
            trajectory, request.start_state.joint_state, arm, wrist_roll_target)
        return trajectory

    def linear(self, arm: str, target: Pose, stage: str,
               wrist_roll_target: float | None = None, *,
               check_orientation: bool = True, route: ApproachRoute | None = None,
               check_collisions: bool = True, validate_command_path: bool = False,
               command_target: CartesianPose | None = None):
        """Plan physical geometry from measured FK, then convert motor support."""
        if type(check_collisions) is not bool:
            raise ValueError('check_collisions must be boolean')
        if command_target is not None and (stage != 'lift_height_correction'
                                          or not isinstance(command_target, CartesianPose)):
            raise ValueError('Explicit command target is restricted to lift height correction')
        defer_shortfall = (stage in ('retreat', 'lift', 'lift_height_correction')
                           and height_correction_enabled(self.config))
        measured = self.real.monitor()
        self.motion.set_current_state(measured)
        names = ARM_JOINT_NAMES[arm]
        positions = dict(zip(measured.name, measured.position, strict=True))
        measured_joints = tuple(positions[name] for name in names)
        def fk(q):
            self.real.monitor()
            value = self.motion.forward_kinematics(arm, JointSolution(names, tuple(q)))
            return CartesianPose(value.position, value.orientation_xyzw)
        start, goal = fk(measured_joints), self.cartesian_pose(target)
        self.event(stage+'_measured_start', arm=arm, start_source='measured',
                   measured_joint_rad=list(measured_joints), measured_tcp_position_m=list(start.position))
        if route is not None and route.waypoints[-1] != goal:
            raise RuntimeError('Approach detour changed the approved grasp target')
        waypoints = (goal,) if route is None else route.waypoints
        def validate(poses, origin):
            if route is None:
                validate_corridor_samples(
                    poses, origin, goal,
                    self.config['cartesian_tolerance_m'],
                    math.radians(15) if check_orientation else None,
                    math.radians(30) if check_orientation else None)
            else:
                validate_detour_samples(
                    poses, origin, route, radius_m=self.config['cartesian_tolerance_m'],
                    maximum_extra_length_m=self.config['approach_collision_max_extra_length_m'])

        # This request always starts from one measured snapshot and uses the
        # unchanged physical destination. Previous command FK cannot replace it.
        trajectory = self._cartesian_trajectory(
            arm, measured, waypoints, stage, wrist_roll_target, check_collisions)
        try:
            nominal = executable_joint_path([tuple(p.positions) for p in trajectory.points])
            nominal_poses = []
            for index, sample in enumerate(nominal.samples(.01, 2, 10000)):
                if check_collisions and not self.motion.state_is_valid(
                        arm, JointSolution(names, sample.positions)):
                    raise CollisionValidationError(
                        f'{stage}: wrist-roll-preserved Cartesian path is in collision: sample={index}',
                        phase='cartesian_path', sample_index=index)
                nominal_poses.append(fk(sample.positions))
            validate(nominal_poses, start)
        except ValueError as error:
            raise CartesianPathUnavailable(
                f'{stage}: Cartesian FK corridor rejected: {error}',
                error_code=MoveItErrorCodes.PLANNING_FAILED, fraction=1.) from error

        # Motor support is an explicit conversion after measured geometry has
        # been checked. Its FK is a command coordinate, never the current TCP.
        # Opening the jaw does not cancel the arm's active position command.
        # Keep its support at release-to-retreat handoff; physical waypoints
        # still originate at measured FK and retain the requested destination.
        hold = (None if stage == 'return_to_pickup' else
                self.real.arm_holds.get(arm))
        command_start = None
        if hold is not None:
            command_start = fk(hold.command)
            if command_target is None:
                command_waypoints = tuple(estimate_measured_path_pose(command_start, start, p)
                                          for p in waypoints)
            else:
                command_waypoints = (command_target,)
            if (command_target is not None or math.dist(start.position, command_start.position) > 1e-9
                    or orientation_error_rad(start.orientation, command_start.orientation) > 1e-9):
                command_state = deepcopy(measured)
                sent = dict(zip(names, hold.command, strict=True))
                command_state.position = [sent.get(name, value) for name, value in
                                          zip(measured.name, measured.position, strict=True)]
                roll = (None if wrist_roll_target is None else wrist_roll_target
                        +hold.command[-1]-measured_joints[-1])
                trajectory = self._cartesian_trajectory(
                    arm, command_state, command_waypoints, stage+'_command_support', roll,
                    check_collisions)
            else:
                trajectory.points[0].positions = list(hold.command)
            self.event('cartesian_command_support', arm=arm, phase=stage,
                       measured_start_position_m=start.position,
                       command_start_position_m=command_start.position,
                       physical_target_position_m=goal.position,
                       command_target_position_m=command_waypoints[-1].position)
        elif command_target is not None:
            raise CommandPathValidationError('Height correction lost its supporting command')
        self.event(stage)
        execution_options = {'endpoint_pose': goal}
        if hold is None:
            execution_options['start_from_measured'] = True
        else:
            execution_options['held_command'] = hold.command
        if stage == 'lift_height_correction':
            execution_options['minimum_duration_sec'] = self.real.value(
                'hardware_height_correction_min_duration_sec')
        if defer_shortfall:
            execution_options['defer_height_shortfall'] = True
        def validate_final_command(samples, actual):
            self.motion.set_current_state(actual)
            current = dict(zip(actual.name, actual.position, strict=True))
            origin = fk(tuple(current[name] for name in names))
            final_poses = []
            for sample in samples:
                pose = fk(sample.positions)
                if command_start is not None:
                    pose = estimate_measured_path_pose(origin, command_start, pose)
                final_poses.append(pose)
            try:
                validate(final_poses, origin)
            except ValueError as error:
                raise CommandPathValidationError(
                    f'{stage}: final held-command FK corridor rejected: {error}') from error
        # Timing changes the shared Hermite slopes. Verify the exact resulting
        # FK corridor for every Cartesian phase, including detours and retreat.
        execution_options['command_path_validator'] = validate_final_command
        self.execute_trajectory(
            arm, trajectory, 'arm', check_orientation=check_orientation,
            check_collisions=check_collisions, **execution_options)
        actual = self.cartesian_pose(self.pose(arm))
        try:
            if defer_shortfall:
                validate_height_correctable_endpoint(actual, goal,
                    self.real.value('hardware_tcp_position_tolerance_m'),
                    math.radians(self.real.value('hardware_tcp_orientation_tolerance_deg'))
                    if check_orientation else math.pi)
            elif check_orientation:
                validate_pose_endpoint(
                    actual, goal,
                    self.real.value('hardware_tcp_position_tolerance_m'),
                    math.radians(self.real.value(
                        'hardware_tcp_orientation_tolerance_deg')),
                )
            else:
                validate_position_endpoint(
                    actual, goal,
                    self.real.value('hardware_tcp_position_tolerance_m'))
        except ValueError as error:
            raise RuntimeError(f'{stage}: measured TCP target not reached: {error}') from error
        # Require the next camera capture to be newer than all endpoint checks.
        return self.real.monitor()

    def move_direct_to_grasp(self, selected, report: dict):
        """Plan from live joints to the selected grasp, without a pregrasp waypoint."""
        arm = selected.selected_arm
        goal = selected.grasp_joint_state
        positions = tuple(float(q) for q in goal.position)
        if (tuple(goal.name) != ARM_JOINT_NAMES[arm]
                or len(positions) != len(ARM_JOINT_NAMES[arm])
                or not all(math.isfinite(q) for q in positions)):
            raise RuntimeError('Direct grasp requires a complete finite selected joint target')
        pose_checks = self.config.get('direct_grasp_pose_checks_enabled', True)
        endpoint_options = {} if pose_checks else {'allow_endpoint_error': True}
        report['direct_grasp_pose_checks_enabled'] = pose_checks
        # The selector's path starts at pregrasp. Prefer a checked direct
        # joint connection from live feedback before requesting a free RRT route.
        checked = None
        if self.config.get('direct_grasp_shortcut_enabled', False):
            self.scene_monitor.wait()
            measured = self.real.monitor()
            self.motion.set_current_state(measured)
            joints = dict(zip(measured.name, measured.position, strict=True))
            start = tuple(float(joints[name]) for name in ARM_JOINT_NAMES[arm])
            checked = self.motion._validated_cubic_path(
                arm, PlannedPath(ARM_JOINT_NAMES[arm], (start, positions)))
        report['direct_grasp_route'] = 'joint_direct' if checked is not None else 'rrt'
        self.event('direct_grasp_route_selected', arm=arm,
                   route=report['direct_grasp_route'])
        if checked is not None:
            self.event('direct_grasp')
            self.execute_trajectory(arm, path_to_message(checked), 'arm', **endpoint_options)
        else:
            self.move_joint_pose(arm, positions, 'direct_grasp', **endpoint_options)
        actual = self.cartesian_pose(self.pose(arm))
        target = self.cartesian_pose(selected.fk_grasp_pose)
        position_error = math.dist(actual.position, target.position)
        orientation_error = math.degrees(orientation_error_rad(actual.orientation, target.orientation))
        if not math.isfinite(position_error):
            raise RuntimeError('Direct grasp requires finite measured and target TCP positions')
        try:
            validate_position_endpoint(
                actual, target, self.real.value('hardware_tcp_position_tolerance_m'))
        except ValueError as error:
            if pose_checks:
                raise RuntimeError(f'direct_grasp: measured TCP target not reached: {error}') from error
            self.event('direct_grasp_position_error_ignored', arm=arm,
                       position_error_m=position_error, orientation_error_deg=orientation_error)
            self.get_logger().warning(
                f'Grasp trial: TCP position error {position_error:.6f}m; proceeding to gripper close')
        report.update(pregrasp_executed=False, direct_grasp_executed=True,
                      direct_grasp_tcp_position_error_m=position_error,
                      direct_grasp_tcp_orientation_error_deg=orientation_error,
                      direct_grasp_execution_metrics=deepcopy(self.real.last_execution_metrics))
        return self.real.monitor()

    def approach_target(self, selected, report: dict):
        """Use corrected selection goals; inspect the physical fixed-finger path."""
        if not self.config.get('pregrasp_motion_enabled', True):
            return self.move_direct_to_grasp(selected, report)
        if not self.config.get('approach_collision_check_enabled', True):
            # Explicitly scoped to this approach; subsequent retreat, lift,
            # gripper commands and joint-space moves keep their default checks.
            report['approach_collision_checks_enabled'] = False
            self.event('approach_collision_checks_disabled', arm=selected.selected_arm)
            if self.config.get('approach_execution_replanning_enabled', False):
                return self.replan_approach_from_arrival(selected, report)
            return self.linear(
                selected.selected_arm, selected.fk_grasp_pose, 'approach',
                self.selected_grasp_roll(selected, selected.selected_arm),
                check_orientation=False, check_collisions=False)
        if not self.config.get('fixed_jaw_clearance_enabled', False):
            return CollectionPipeline._approach_target(self, selected, report)
        arm = selected.selected_arm
        # Hardware verify applies this same controller/MoveIt URDF before each
        # execution. Use serial-chain FK here to avoid an extra ROS FK service
        # call for every gravity-biased command and mandatory hold-bridge sample.
        description = self.description
        meshes = tuple(self.config['fixed_jaw_collision_meshes'])
        key = (description, arm, meshes)
        cache = getattr(self, '_fixed_jaw_geometry_cache', None)
        if cache is None or cache[0] != key:
            cache = (key, load_fixed_jaw_bounds(description, arm, meshes),
                     UrdfChain(description, 'base_link', f'{arm}_grasp_tcp'))
            self._fixed_jaw_geometry_cache = cache
        _, jaw, chain = cache
        corners = target_box(selected.selected_candidate.target_object).corners()
        diagnostics = {'arm': arm, 'checked_samples': 0, 'minimum_gap_m': None,
                       'outcome': 'checking'}
        report['fixed_jaw_approach_clearance'] = diagnostics

        def guard(positions, index, phase):
            # Do not accept a model change while validating an active path.
            if self.description != description:
                raise RuntimeError('Runtime URDF changed during fixed-jaw approach validation')
            position, rotation = chain.pose(positions)
            pose = PoseValue(tuple(position), quaternion_from_rotation(rotation))
            gap = closing_clearance(jaw, corners, [pose])
            diagnostics['checked_samples'] += 1
            if gap is not None:
                old = diagnostics['minimum_gap_m']
                diagnostics['minimum_gap_m'] = gap if old is None else min(old, gap)
            if gap is not None and gap <= 0:
                diagnostics.update(outcome='collision', sample_index=index, phase=phase)
                self.event('fixed_jaw_approach_rejected', **diagnostics)
                raise CollisionValidationError(
                    f'Fixed finger overlaps target on physical approach: '
                    f'gap={gap * 1000:.3f}mm phase={phase} sample={index}',
                    phase=phase, sample_index=index,
                    contacts=({'body_1': f'{arm}_gripper_frame', 'body_2': 'approved_target',
                               'geometry': 'fixed_finger_bounds',
                               'closing_gap_m': gap},))

        previous = getattr(self.real, '_approach_clearance_guard', None)
        if previous is not None:
            raise RuntimeError('Fixed-jaw approach validation is already active')
        self.real._approach_clearance_guard = guard
        try:
            result = self._approach_target(selected, report)
            diagnostics['outcome'] = 'executed'
            return result
        except BaseException:
            if diagnostics['outcome'] == 'checking':
                diagnostics['outcome'] = 'not_executed'
            raise
        finally:
            self.real._approach_clearance_guard = previous
            self.event('fixed_jaw_approach_clearance', **diagnostics)

    def _approach_target(self, selected, report: dict):
        """Retry only unexecuted, rejected paths; retain the selected grasp pose."""
        if self.config.get('approach_execution_replanning_enabled', False):
            return self.replan_approach_from_arrival(selected, report)
        arm, target = selected.selected_arm, selected.fk_grasp_pose
        roll = self.selected_grasp_roll(selected, arm)
        if not self.config.get('approach_collision_replanning_enabled', False):
            return self.linear(arm, target, 'approach', roll, check_orientation=False)
        recovery = {'scene_refreshes': 0, 'detour_attempts': 0, 'failures': [],
                    'selected_route': '', 'outcome': 'planning'}
        report['approach_collision_replanning'] = recovery

        def reject(error) -> None:
            if isinstance(error, CollisionValidationError) and error.motion_started:
                raise error  # Endpoint/communication recovery cannot become a new approach.
            recovery['failures'].append({
                'phase': getattr(error, 'phase', 'cartesian_planning'),
                'sample_index': getattr(error, 'sample_index', None),
                'message': str(error),
                'contacts': list(getattr(error, 'contacts', ())),
            })
            self.event('approach_path_rejected', **recovery['failures'][-1])

        try:
            result = self.linear(arm, target, 'approach', roll, check_orientation=False)
        except (CollisionValidationError, CartesianPathUnavailable) as error:
            reject(error)
            last_error = error
        else:
            recovery.update(selected_route='straight', outcome='executed')
            return result
        for _ in range(self.config['approach_collision_scene_refresh_attempts']):
            after = self.get_clock().now().nanoseconds
            def monitor_while_waiting() -> bool:
                # monitor raises on cancellation, stale encoders or a fault.
                self.real.monitor()
                return False
            self.scene_monitor.wait(after_stamp_ns=after, canceled=monitor_while_waiting)
            self.real.monitor()
            recovery['scene_refreshes'] += 1
            self.event('approach_collision_scene_refreshed', after_stamp_ns=after)
            try:
                result = self.linear(arm, target, 'approach', roll, check_orientation=False)
            except (CollisionValidationError, CartesianPathUnavailable) as error:
                reject(error)
                last_error = error
            else:
                recovery.update(selected_route='straight_after_scene_refresh', outcome='executed')
                return result
        if last_error.entry_blocked:
            recovery['outcome'] = 'entry_blocked'
            self.event('approach_entry_blocked', phase=getattr(last_error, 'phase', 'cartesian_planning'),
                       message=str(last_error))
            raise last_error  # Changing a later waypoint cannot clear this mandatory entry.
        routes = approach_detours(
            self.cartesian_pose(self.pose(arm)), self.cartesian_pose(target),
            offset_m=self.config['approach_collision_detour_offset_m'],
            final_straight_m=self.config['approach_collision_final_straight_m'],
            maximum_extra_length_m=self.config['approach_collision_max_extra_length_m'],
            attempts=self.config['approach_collision_detour_attempts'])
        for route in routes:
            recovery['detour_attempts'] += 1
            self.event('approach_detour_planning', route=route.name,
                       waypoints_m=[point.position for point in route.waypoints])
            try:
                result = self.linear(arm, target, 'approach_detour', roll,
                                     check_orientation=False, route=route)
            except (CollisionValidationError, CartesianPathUnavailable) as error:
                reject(error)
                last_error = error
                if error.entry_blocked:
                    recovery['outcome'] = 'entry_blocked'
                    raise
            else:
                recovery.update(selected_route=route.name, outcome='executed')
                self.event('approach_detour_executed', route=route.name)
                return result
        recovery['outcome'] = 'detours_exhausted' if routes else 'final_entry_or_length_limit'
        raise RuntimeError('Approach collision recovery stopped: '+recovery['outcome']+
                           '; '+str(last_error)) from last_error

    def replan_approach_from_arrival(self, selected, report: dict):
        """Replan from fresh arrival, then preserve the held start without rebiasing."""
        arm = selected.selected_arm
        attempts = report.setdefault('approach_execution_replanning', [])
        for attempt in range(self.config.get('approach_execution_replanning_attempts', 2)):
            self.real.monitor()
            submissions = getattr(self.real, 'motion_submissions', 0)
            item = {'arm': arm, 'candidate_index': int(selected.selected_candidate_index),
                    'original_candidate_index': report.get('approach_original_candidate_index',
                        int(selected.selected_candidate_index)),
                    'start': 'measured', 'outcome': 'planning'}
            attempts.append(item)
            self.event('approach_replanning', **item)
            try:
                result = self.linear(arm, selected.fk_grasp_pose, 'approach',
                    self.selected_grasp_roll(selected, arm), check_orientation=False,
                    check_collisions=self.config.get('approach_collision_check_enabled', True),
                    validate_command_path=True)
            except (JointRangeValidationError, CommandPathValidationError,
                    CollisionValidationError, CartesianPathUnavailable) as error:
                if (getattr(error, 'motion_started', False)
                        or getattr(self.real, 'motion_submissions', 0) != submissions):
                    raise
                self.real.monitor()
                item.update(outcome='rejected', message=str(error))
                self.event('approach_command_rejected', **item)
                last_error = error
                if getattr(error, 'entry_blocked', False):
                    raise
            else:
                item['outcome'] = 'executed'
                self.event('approach_replanned_command_executed', **item)
                return result
        raise last_error

    def approach_with_candidate_reselection(self, selected, report: dict, *, object_id: str,
                                           prepare_pregrasp: Callable[[Any], None]) -> tuple[Any, Pose]:
        """Recover only unsubmitted approaches for the same approved object and arm."""
        remaining = list(selected.candidates)
        original_indices = list(range(len(remaining)))
        maximum = self.config.get('approach_candidate_reselection_attempts', 2)
        report['approach_candidate_reselections'] = 0
        for attempt in range(maximum + 1):
            index = int(selected.selected_candidate_index)
            if not 0 <= index < len(remaining):
                raise CommandPathValidationError('Selected candidate index does not match the candidate set')
            original_index = original_indices[index]
            report['approach_original_candidate_index'] = original_index
            submissions = getattr(self.real, 'motion_submissions', 0)
            try:
                if attempt:
                    prepare_pregrasp(selected)
                    # A completed new pregrasp is the next safe checkpoint.
                    # Failures after a submitted, unfinished pregrasp cannot retry.
                    submissions = getattr(self.real, 'motion_submissions', 0)
                self.scene.begin(selected.selected_candidate, object_id)
                self.scene.allow_contacts_for(selected.selected_arm)
                pregrasp = self.pose(selected.selected_arm)
                with self.record_approach_commands(selected.selected_arm):
                    self.approach_target(selected, report)
            except (JointRangeValidationError, CommandPathValidationError,
                    CollisionValidationError, CartesianPathUnavailable) as error:
                if (attempt == maximum or getattr(error, 'motion_started', False)
                        or getattr(self.real, 'motion_submissions', 0) != submissions
                        or getattr(error, 'entry_blocked', False)):
                    raise
                del remaining[index]
                del original_indices[index]
                if not remaining:
                    raise
                self.real.monitor()
                self.scene.restore()
                self.release_grasp_scene_freeze()
                if self.wrist_request is not None:
                    request = deepcopy(self.wrist_request)
                    request.operation = request.CLEAR
                    self.call(self.wrist, request)
                    self.wrist_request, self.wrist_reference_id = None, ''
                self.event('approach_candidate_reselection', rejected_candidate_index=index,
                    rejected_original_candidate_index=original_index,
                    remaining_original_candidate_indices=list(original_indices),
                    remaining_candidates=len(remaining), reason=str(error), arm=selected.selected_arm)
                selected = self.reselect_approach_candidate(selected, remaining)
                report['approach_candidate_reselections'] += 1
            else:
                report.update(arm=selected.selected_arm,
                              candidate_index=int(selected.selected_candidate_index))
                return selected, pregrasp

    def reselect_approach_candidate(self, previous, candidates):
        from .validate_pick_node import ValidatePickNode

        client = getattr(self, '_selection', None)
        owned = client is None
        if owned:
            client = ActionClient(self, SelectReachableGrasp, '/grasp/select_reachable')
        try:
            result = self.run_action(client, SelectReachableGrasp.Goal(
                candidates=candidates, required_arm=previous.selected_arm),
                self.config.get('approach_candidate_selection_timeout_sec', 60.), allow_aborted=True)
        finally:
            if owned:
                client.destroy()
        if not result.success:
            raise CommandPathValidationError('No remaining approach candidate is reachable: '+result.message)
        approved = previous.selected_candidate
        candidate = result.selected_candidate
        if (result.selected_arm != previous.selected_arm
                or candidate.snapshot_id != approved.snapshot_id or candidate.object_id != approved.object_id
                or candidate.header.frame_id != approved.header.frame_id
                or candidate.target_object != approved.target_object
                or not 0 <= result.selected_candidate_index < len(candidates)):
            raise RuntimeError('Approach reselection changed the approved object, frame or arm')
        selected = deepcopy(previous)
        selected.candidates = list(candidates)
        ValidatePickNode._copy_selection_result(selected, result)
        home = self.arm_config.home
        self.before_execution(selected)
        # The original startup home is not the later candidate's current pregrasp.
        self.arm_config = replace(self.arm_config, home=home)
        return selected

    def prepare_reselected_pregrasp(self, selected, report: dict,
                                   cleanup_request: ObserveWristTarget.Request | None = None) -> None:
        """Repeat the original pregrasp observation/handoff checks after reselection."""
        before = after = None
        if self.head_reobservation_required():
            before = self.observe(selected.reference_id, self.get_clock().now().nanoseconds)
            center = selected.selected_object.obb_pose.position
            size = selected.selected_object.obb_size
            radius = math.hypot(size.x, size.y, size.z) / 2
            observed = before.observed_center
            if math.dist((center.x, center.y, center.z),
                         (observed.x, observed.y, observed.z)) > radius + self.target_translation_limit():
                raise RuntimeError('Reselected target observation is outside the approved object')
        arrival = self.execute_pregrasp(selected)
        report['pregrasp_execution_metrics'] = deepcopy(self.real.last_execution_metrics)
        if before is not None:
            after = self.observe_arrival(selected, before, arrival, report)
            self.validate_target_translation(selected, before, after,
                math.dist(observation_center(before), observation_center(after)), report)
        if self.wrist_observation_required():
            request = ObserveWristTarget.Request(operation=ObserveWristTarget.Request.HANDOFF,
                arm=selected.selected_arm, source_snapshot_id=selected.selected_candidate.snapshot_id,
                source_object_id=selected.selected_candidate.object_id,
                label=after.source_label, source_confidence=float(after.source_confidence),
                after_stamp_ns=arrival)
            request.expected_pose.header = deepcopy(after.header)
            request.expected_pose.pose.position = deepcopy(after.observed_center)
            request.expected_pose.pose.orientation.w = 1.
            request.size = deepcopy(after.observed_extent)
            response = self.call(self.wrist, request, 60.)
            if (not response.reference_id or response.source_snapshot_id != request.source_snapshot_id
                    or response.source_object_id != request.source_object_id):
                raise RuntimeError('Reselected wrist handoff identity mismatch')
            request.reference_id = response.reference_id
            request.operation = request.CHECK
            request.after_stamp_ns = response.header.stamp.sec * 10**9 + response.header.stamp.nanosec
            for _ in range(int(self.get_parameter('wrist_checks').value)):
                observation = self.call(self.wrist, request, 60.)
                stamp = observation.header.stamp.sec * 10**9 + observation.header.stamp.nanosec
                if stamp <= request.after_stamp_ns:
                    raise RuntimeError('Reselected wrist tracking did not advance')
                request.after_stamp_ns = stamp
                report['wrist_checks'] += 1
            self.wrist_request, self.wrist_reference_id = request, request.reference_id
            if cleanup_request is not None:
                for name in request.get_fields_and_field_types():
                    setattr(cleanup_request, name, deepcopy(getattr(request, name)))

    def observe_for_approach(self, reference_id: str, after: int):
        """Only target mask/depth loss can enter the wrist-confirmed fallback.

        Camera timeout, invalid reference/calibration, TF and internal errors
        remain fatal. A mask/depth failure is suspected occlusion, not proof.
        """
        try:
            return self.observe(reference_id, after)
        except ReferenceObservationFailure as error:
            response = error.response
            if (not self.config.get('visual_wrist_occlusion_fallback_enabled', False)
                    or not self.config.get('wrist_observation_enabled', True)
                    or error.error_code not in (response.ERROR_MASK, response.ERROR_DEPTH)
                    or response.reference_id != reference_id):
                raise
            self.event('visual_head_target_unavailable',
                       error_code=error.error_code, message=str(error))
            return None

    def observe_arrival(self, selected, before, arrival, report):
        self._head_occluded_at_arrival = False
        observation = CollectionPipeline.observe_for_approach(
            self, selected.reference_id, arrival)
        if observation is not None:
            return observation
        age = (self.get_clock().now().nanoseconds-observation_stamp_ns(before))/1e9
        if not 0 <= age <= float(self.config['visual_wrist_occlusion_max_age_sec']):
            raise RuntimeError('Head target lost on arrival and the last 3D prior expired')
        self._head_occluded_at_arrival = True
        report['head_observation_after_pregrasp_missing'] = True
        report['reused_head_prior_age_sec'] = age
        self.event('wrist_handoff_using_last_head_prior', prior_age_sec=age)
        # Preserve the actual old capture stamp. HANDOFF still requires a wrist
        # exposure after arrival and confirms identity before any approach move.
        return deepcopy(before)

    def confirm_visual_observation(
        self,
        reference_id: str,
        observation,
        reference_center: tuple[float, float, float],
        previous_center: tuple[float, float, float],
        previous_observation,
    ) -> dict:
        """Median-filter a suspicious surface centroid before it can stop motion."""
        if observation is None:
            return {'accepted': False, 'suspicious': True, 'observation': None,
                    'reason': 'head_target_mask_or_depth_unavailable'}
        soft_limit = float(self.config['visual_servo_maximum_correction_m'])
        hard_limit = float(self.config.get(
            'visual_servo_hard_correction_limit_m', 2.*soft_limit))
        increment_limit = float(self.config.get(
            'visual_observation_increment_limit_m', .01))
        extent_limit = float(self.config.get(
            'visual_observation_extent_ratio_limit', 1.5))
        consistency = float(self.config.get(
            'visual_observation_consistency_m', .008))
        confirmation_frames = int(self.config.get(
            'visual_observation_confirmation_frames', 3))
        raw_center = observation_center(observation)
        raw_correction = math.dist(reference_center, raw_center)
        raw_increment = math.dist(previous_center, raw_center)
        extent_ratio = observation_extent_ratio(
            previous_observation, observation)
        suspicious = (
            raw_correction > soft_limit
            or raw_increment > increment_limit
            or extent_ratio > extent_limit
        )
        observations = [observation]
        if suspicious:
            for _ in range(confirmation_frames-1):
                capture = CollectionPipeline.observe_for_approach(
                    self, reference_id, observation_stamp_ns(observations[-1]))
                if capture is None:
                    return {'accepted': False, 'suspicious': True, 'observation': None,
                            'reason': 'head_target_lost_during_confirmation'}
                observations.append(capture)
        centers = [observation_center(item) for item in observations]
        filtered_center = coordinate_median(centers)
        inliers = sum(
            math.dist(center, filtered_center) <= consistency
            for center in centers
        )
        correction = math.dist(reference_center, filtered_center)
        increment = math.dist(previous_center, filtered_center)
        consistent = inliers >= len(centers)//2+1
        accepted = (
            consistent
            and extent_ratio <= extent_limit
            and correction <= hard_limit
            and (correction <= soft_limit or increment <= increment_limit)
        )
        return {
            'accepted': accepted,
            'suspicious': suspicious,
            'observation': observations[-1],
            'confirmation_frames': len(observations),
            'confirmation_inliers': inliers,
            'raw_center_m': raw_center,
            'filtered_center_m': filtered_center,
            'raw_correction_m': raw_correction,
            'filtered_correction_m': correction,
            'filtered_increment_m': increment,
            'extent_ratio': extent_ratio,
            'soft_limit_m': soft_limit,
            'hard_limit_m': hard_limit,
            'stable_capture_stamp_ns': min(
                observation_stamp_ns(item) for item in observations
                if math.dist(observation_center(item), filtered_center) <= consistency
            ) if accepted else None,
        }

    def visual_approach(self, selected, before_observation, after_observation, report):
        """Reobserve each segment, rejecting one-frame centroid jumps near the jaws."""
        if before_observation is None or after_observation is None:
            raise RuntimeError('Visual approach requires before/after RGB-D observations')
        arm = selected.selected_arm
        reference = observation_center(before_observation)
        planned = self.cartesian_pose(selected.fk_grasp_pose)
        grasp_roll = CollectionPipeline.selected_grasp_roll(selected, arm)
        observation = (None if getattr(self, '_head_occluded_at_arrival', False)
                       else after_observation)
        previous_observation = before_observation
        accepted_center = reference
        hard_limit = float(self.config.get(
            'visual_servo_hard_correction_limit_m',
            2.*float(self.config['visual_servo_maximum_correction_m'])))
        accepted_target = planned
        records = []
        moves = 0
        wrist_fallbacks = 0
        fallback_distance = 0.
        last_head_stamp = observation_stamp_ns(after_observation)
        segment_m = float(self.config.get(
            'visual_approach_segment_m', self.config['visual_approach_step_m']))
        for index in range(int(self.config['visual_approach_max_steps'])):
            decision = CollectionPipeline.confirm_visual_observation(
                self,
                selected.reference_id,
                observation,
                reference,
                accepted_center,
                previous_observation,
            )
            observation = decision['observation']
            current = self.cartesian_pose(self.pose(arm))
            using_wrist_fallback = False
            if not decision['accepted']:
                capture = grasp_capture_metrics(current, accepted_target)
                capture_depth = float(self.config.get(
                    'visual_grasp_capture_depth_m', .045))
                capture_lateral = float(self.config.get(
                    'visual_grasp_capture_lateral_tolerance_m', .012))
                inside_capture = capture.inside(
                    capture_depth,
                    capture_lateral,
                    float(self.config['cartesian_tolerance_m']),
                )
                rejected = {
                    'step': index,
                    **decision,
                    'accepted': False,
                    'observation': (None if observation is None else
                                    reference_observation_metrics(observation)),
                    'capture_distance_m': capture.distance_m,
                    'capture_approach_remaining_m': capture.approach_remaining_m,
                    'capture_lateral_error_m': capture.lateral_error_m,
                    'inside_grasp_capture_corridor': inside_capture,
                }
                records.append(rejected)
                self.event('visual_observation_rejected', **rejected)
                fallback_enabled = bool(self.config.get(
                    'visual_wrist_occlusion_fallback_enabled', False)) and bool(
                        self.config.get('wrist_observation_enabled', True))
                fallback_limit = int(self.config.get(
                    'visual_wrist_occlusion_max_steps', 0))
                if fallback_enabled:
                    entry_distance = float(self.config.get(
                        'visual_wrist_occlusion_entry_distance_m', .06))
                    if capture.distance_m > entry_distance:
                        raise RuntimeError('Head target lost outside the bounded final approach')
                    maximum_age = self.config.get('visual_wrist_occlusion_max_age_sec')
                    if maximum_age is not None:
                        age = (self.get_clock().now().nanoseconds
                               - last_head_stamp)/1e9
                        if not 0 <= age <= float(maximum_age):
                            raise RuntimeError('Last stable 3D target is too old for wrist fallback')
                    if not inside_capture and wrist_fallbacks >= fallback_limit:
                        raise RuntimeError('Wrist-confirmed approach step budget exhausted')
                    wrist = self.confirm_wrist_target(report, index)
                    rejected.update(wrist_occlusion_fallback=True,
                                    wrist_confirmation=wrist)
                    self.event('visual_head_occlusion_fallback', step=index,
                               wrist_fallback_count=wrist_fallbacks,
                               fallback_distance_m=fallback_distance)
                if inside_capture:
                    self.event(
                        'visual_capture_finalized',
                        tcp_remaining_m=capture.distance_m,
                        capture_approach_remaining_m=(
                            capture.approach_remaining_m),
                        capture_lateral_error_m=capture.lateral_error_m,
                        final_motion_skipped=True,
                        target_surface_correction_m=math.dist(
                            reference, accepted_center),
                    )
                    report['visual_approach'] = {
                        'steps': moves,
                        'final_target_surface_correction_m': math.dist(
                            reference, accepted_center),
                        'finalized_from_grasp_capture_corridor': True,
                        'wrist_occlusion_fallbacks': wrist_fallbacks,
                        'wrist_occlusion_distance_m': fallback_distance,
                        'observations': records,
                    }
                    return
                if not fallback_enabled or wrist_fallbacks >= fallback_limit:
                    report['visual_approach'] = {
                        'steps': moves,
                        'wrist_occlusion_fallbacks': wrist_fallbacks,
                        'observations': records,
                    }
                    raise RuntimeError(
                        'Confirmed target observation is outside the visual correction '
                        'limit and the TCP grasp capture corridor')
                wrist_fallbacks += 1
                using_wrist_fallback = True
            else:
                accepted_center = decision['filtered_center_m']
                previous_observation = observation
                last_head_stamp = decision['stable_capture_stamp_ns']
                accepted_target = observation_corrected_pose(
                    planned, reference, accepted_center, hard_limit)
                if getattr(self, 'wrist_request', None) is not None:
                    self.wrist_request.expected_pose.header = deepcopy(
                        observation.header)
                    wrist_center = self.wrist_request.expected_pose.pose.position
                    wrist_center.x, wrist_center.y, wrist_center.z = accepted_center
                    self.wrist_request.size = deepcopy(
                        observation.observed_extent)
            position_error = math.dist(current.position, accepted_target.position)
            orientation_error = orientation_error_rad(
                current.orientation, accepted_target.orientation)
            correction = math.dist(reference, accepted_center)
            capture = grasp_capture_metrics(current, accepted_target)
            capture_depth = float(self.config.get(
                'visual_grasp_capture_depth_m', .045))
            capture_lateral = float(self.config.get(
                'visual_grasp_capture_lateral_tolerance_m', .012))
            inside_capture = capture.inside(
                capture_depth,
                capture_lateral,
                float(self.config['cartesian_tolerance_m']),
            )
            record = {
                'step': index,
                'target_surface_correction_m': correction,
                'tcp_remaining_m': position_error,
                'tcp_orientation_remaining_deg': math.degrees(orientation_error),
                'capture_approach_remaining_m': capture.approach_remaining_m,
                'capture_lateral_error_m': capture.lateral_error_m,
                'inside_grasp_capture_corridor': inside_capture,
                'observation': (None if observation is None else
                                reference_observation_metrics(observation)),
                'filtered_center_m': accepted_center,
                'confirmation_frames': decision.get('confirmation_frames', 0),
                'confirmation_inliers': decision.get('confirmation_inliers', 0),
                'raw_target_surface_correction_m': decision.get('raw_correction_m'),
                'target_surface_increment_m': decision.get('filtered_increment_m'),
                'observation_extent_ratio': decision.get('extent_ratio'),
                'wrist_occlusion_fallback': using_wrist_fallback,
                'segment_limit_m': segment_m,
                'path_resolution_m': float(self.config['cartesian_step_m']),
            }
            records.append(record)
            self.event('visual_approach_feedback', **record)
            if position_error <= self.config['cartesian_tolerance_m']:
                report['visual_approach'] = {
                    'steps': moves,
                    'final_target_surface_correction_m': correction,
                    'finalized_from_grasp_capture_corridor': False,
                    'wrist_occlusion_fallbacks': wrist_fallbacks,
                    'wrist_occlusion_distance_m': fallback_distance,
                    'observations': records,
                }
                return
            if inside_capture:
                self.event(
                    'visual_capture_finalized',
                    tcp_remaining_m=capture.distance_m,
                    capture_approach_remaining_m=(
                        capture.approach_remaining_m),
                    capture_lateral_error_m=capture.lateral_error_m,
                    final_motion_skipped=True,
                    target_surface_correction_m=correction,
                )
                report['visual_approach'] = {
                    'steps': moves,
                    'final_target_surface_correction_m': correction,
                    'finalized_from_grasp_capture_corridor': True,
                    'wrist_occlusion_fallbacks': wrist_fallbacks,
                    'wrist_occlusion_distance_m': fallback_distance,
                    'observations': records,
                }
                return
            motion_segment_m = segment_m
            if using_wrist_fallback:
                maximum_distance = float(self.config.get(
                    'visual_wrist_occlusion_max_distance_m', segment_m*fallback_limit))
                remaining_distance = maximum_distance-fallback_distance
                if remaining_distance <= 1e-9:
                    raise RuntimeError('Wrist-confirmed approach distance budget exhausted')
                motion_segment_m = min(
                    segment_m, remaining_distance,
                    float(self.config.get('visual_wrist_occlusion_step_m', segment_m)))
            waypoint = bounded_cartesian_step(
                current, accepted_target, motion_segment_m)
            measured = self.real.monitor()
            joint_positions = dict(zip(measured.name, measured.position))
            wrist = f'{arm}_wrist_roll_joint'
            if wrist not in joint_positions or not math.isfinite(
                    joint_positions[wrist]):
                raise RuntimeError('Measured wrist roll is unavailable')
            fraction = min(1., motion_segment_m/position_error) if position_error > 0 else 1.
            waypoint_roll = joint_positions[wrist]+(
                grasp_roll-joint_positions[wrist])*fraction
            arrived = self.linear(
                arm, self.pose_message(waypoint), 'visual_approach_step',
                waypoint_roll, check_orientation=False)
            moves += 1
            if using_wrist_fallback:
                # Charge the planned distance even if motor tracking falls short.
                fallback_distance += math.dist(current.position, waypoint.position)
            stamp = arrived.header.stamp.sec*10**9+arrived.header.stamp.nanosec
            if stamp <= 0:
                raise RuntimeError('Visual approach received an unstamped arrival state')
            observation = CollectionPipeline.observe_for_approach(
                self, selected.reference_id, stamp)
        report['visual_approach'] = {
            'steps': moves,
            'wrist_occlusion_fallbacks': wrist_fallbacks,
            'wrist_occlusion_distance_m': fallback_distance,
            'observations': records,
        }
        raise RuntimeError('Visual Cartesian approach exceeded its step budget')

    def move_joint_pose(self, arm: str, positions: tuple[float, ...], stage: str,
                        *, allow_endpoint_error=False):
        request = GetMotionPlan.Request()
        m = request.motion_plan_request
        m.group_name = f'{arm}_grasp_arm'
        m.pipeline_id = 'ompl'
        m.planner_id = 'RRTConnectkConfigDefault'
        m.allowed_planning_time = 5.
        m.num_planning_attempts = 2
        m.max_velocity_scaling_factor = .5
        m.max_acceleration_scaling_factor = .5
        m.start_state.joint_state = self.real.monitor()
        # Selection runs in another process; initialize this execution adapter
        # from the same live state used by the joint planner and endpoint FK.
        self.motion.set_current_state(m.start_state.joint_state)
        m.start_state.is_diff = True
        m.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=q, tolerance_above=1e-4,
                            tolerance_below=1e-4, weight=1.)
            for n,q in zip(ARM_JOINT_NAMES[arm],positions)])]
        response = self.call(self.joint_plan, request, 15.).motion_plan_response
        if response.error_code.val != 1:
            raise RuntimeError(stage+': RRTConnect failed')
        self.event(stage)
        endpoint_options = {'allow_endpoint_error': True} if allow_endpoint_error else {}
        self.execute_trajectory(arm, response.trajectory.joint_trajectory, 'arm', **endpoint_options)

    def jaw(self, arm: str, position: float, stage: str, *, contact_close=False,
            check_collisions=True):
        state = self.real.monitor()
        q = dict(zip(state.name,state.position))
        joint = f'{arm}_gripper_joint'
        trajectory = JointTrajectory(joint_names=[joint], points=[
            JointTrajectoryPoint(positions=[q[joint]]), JointTrajectoryPoint(positions=[position])])
        self.event(stage)
        collision_options = {'check_collisions': False} if not check_collisions else {}
        if contact_close:
            self.real.execute(arm, trajectory, 'gripper', contact_close=True, **collision_options)
        else:
            self.execute_trajectory(arm, trajectory, 'gripper', **collision_options)

    def close_until_contact(self, selected, report=None):
        arm = selected.selected_arm
        after = self.get_clock().now().nanoseconds
        target = self.real.closed_contact_position(arm, self.real.verify(arm))
        self.jaw(arm, target, 'close_for_contact', contact_close=True,
                 check_collisions=self.config.get('grasp_collision_check_enabled', True))
        decision = self.real.wait_for_contact(arm, after)
        report = {} if report is None else report
        # Capture after the motor window completes, so the identity evidence
        # remains fresh while the service checks the same raw criterion.
        identity_required = grasp_identity_verification_required(self.config)
        if identity_required:
            self.confirm_wrist_target(report, 0)
            self.verify_evidence(selected, CheckManipulation.Request.GRASP, after)
        report.update(grasp_contact_confirmed=True, grasp_assumed=False, grasp_verified=False,
                      grasp_identity_verified=identity_required,
                      contact_verification='RAW_FEEDBACK')
        self.event('grasp_confirmed', minimum_residual_ticks=decision.minimum_residual_ticks,
                   median_current_raw=decision.median_current_raw,
                   minimum_load_raw=decision.minimum_load_raw,
                   position_span_ticks=decision.position_span_ticks,
                   window_sec=decision.window_sec)

    def verify_evidence(self, selected, stage: int, after: int):
        request = CheckManipulation.Request(stage=stage, arm=selected.selected_arm,
            reference_id=selected.reference_id, wrist_reference_id=self.wrist_reference_id, source_snapshot_id=selected.selected_candidate.snapshot_id,
            source_object_id=selected.selected_candidate.object_id, after_stamp_ns=after)
        response = self.call(self.evidence, request, 10.)
        check_evidence(response, request, self.get_clock().now().nanoseconds,
                       self.config['evidence_max_age_sec'])
        self.event('evidence', evidence_stage=stage)

    def apply(self, scene):
        response = self.call(self.apply_scene, ApplyPlanningScene.Request(scene=scene))
        if not response.success:
            raise RuntimeError('Payload planning-scene update failed')

    def attach(self, selected, object_id: str):
        """Attach perceived OBB to measured gripper frame via MoveIt's frame transform."""
        arm = selected.selected_arm
        obj = CollisionObject(id=object_id, operation=CollisionObject.ADD)
        obj.header = deepcopy(selected.selected_candidate.header)
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
            dimensions=[getattr(selected.selected_object.obb_size,a) for a in ('x','y','z')])]
        obj.primitive_poses = [deepcopy(selected.selected_object.obb_pose)]
        payload = AttachedCollisionObject(link_name=f'{arm}_gripper_frame', object=obj,
            touch_links=[f'{arm}_gripper_frame',f'{arm}_moving_jaw_link'])
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [payload]
        saved_payload_acm = None
        if not self.config.get('held_object_octomap_collision_check_enabled', True):
            query = GetPlanningScene.Request()
            query.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            saved_payload_acm = deepcopy(self.call(self.get_scene, query).scene.allowed_collision_matrix)
            # An empty ACM in a MoveIt diff means "leave unchanged", so require
            # the registered target matrix that can later be restored exactly.
            if not saved_payload_acm.entry_names:
                raise RuntimeError('Held-object OctoMap exemption requires an active target collision matrix')
            # Allow exactly this attached object against the sensor map. Robot
            # links, other objects and named world obstacles keep their checks.
            scene.allowed_collision_matrix = TargetSceneTransaction._with_allowed_links(
                saved_payload_acm, object_id, {'<octomap>'})
        # MoveIt transfers an ADDed AttachedCollisionObject out of the world.
        # An explicit world REMOVE in the same diff runs after that transfer,
        # reports the now-absent object as a failure, and rejects the service.
        # Mark ownership BEFORE sending: a timeout is an uncertain attachment.
        self.payload_selected = selected
        self.payload_evidence_time = time.monotonic()
        self.carrying = {'object_id':object_id,'arm':arm,'attachment':payload,
                         'grasp_tcp_pose':self.pose(arm), 'release_confirmed':False}
        if saved_payload_acm is not None:
            self.carrying['payload_octomap_saved_acm'] = saved_payload_acm
        self.apply(scene)
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        if saved_payload_acm is not None:
            request.components.components |= PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        confirmed = self.call(self.get_scene,request).scene
        attached = confirmed.robot_state.attached_collision_objects
        payloads = [a for a in attached if a.object.id == object_id]
        if len(payloads) != 1 or payloads[0].link_name != f'{arm}_gripper_frame':
            raise RuntimeError('MoveIt did not confirm payload attachment')
        if (saved_payload_acm is not None and not collision_pair_is_allowed(
                confirmed.allowed_collision_matrix, object_id, '<octomap>')):
            raise RuntimeError('MoveIt did not confirm held-object OctoMap collision permission')
        from rosidl_runtime_py.convert import message_to_ordereddict
        state = self.measured()
        roll = dict(zip(state.name,state.position))[f'{arm}_wrist_roll_joint']
        self.event('attach', object_id=object_id, wrist_roll_rad=roll,
                   attachment=message_to_ordereddict(payloads[0]))
        if saved_payload_acm is not None:
            self.event('held_object_octomap_collision_check_disabled',
                       object_id=object_id, allowed_pair=[object_id, '<octomap>'])

    def confirm_held_target(self, selected, tcp_pose: Pose, report: dict) -> None:
        """Check fresh wrist RGB against a kinematic held-object prior after lift."""
        if self.wrist_request is None or not self.wrist_reference_id or self.carrying is None:
            raise RuntimeError('Post-lift wrist reference is unavailable')
        def rotation(pose):
            q = pose.orientation
            return rotation_from_quaternion((q.x, q.y, q.z, q.w))
        def position(pose):
            p = pose.position
            return np.array((p.x, p.y, p.z))
        original_tcp = self.carrying['grasp_tcp_pose']
        original_object = selected.selected_candidate.target_object.obb_pose
        delta_rotation = rotation(tcp_pose) @ rotation(original_tcp).T
        center = position(tcp_pose) + delta_rotation @ (position(original_object)-position(original_tcp))
        request = deepcopy(self.wrist_request)
        request.operation, request.reference_id = request.CHECK, self.wrist_reference_id
        request.after_stamp_ns = max(request.after_stamp_ns, self.get_clock().now().nanoseconds)
        request.expected_pose.header.frame_id = 'base_link'
        request.expected_pose.header.stamp = self.get_clock().now().to_msg()
        pose = request.expected_pose.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, center)
        q = quaternion_from_rotation(delta_rotation @ rotation(original_object))
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        request.size = deepcopy(selected.selected_candidate.target_object.obb_size)
        response = self.call(self.wrist, request, 60.)
        stamp = response.header.stamp.sec*10**9 + response.header.stamp.nanosec
        if (response.reference_id != self.wrist_reference_id
                or response.source_snapshot_id != request.source_snapshot_id
                or response.source_object_id != request.source_object_id
                or stamp <= request.after_stamp_ns):
            raise RuntimeError('Post-lift wrist observation provenance/freshness failed')
        self.wrist_request = deepcopy(request)
        self.wrist_request.after_stamp_ns = stamp
        report['wrist_confirmed_after_lift'] = True
        report['wrist_checks'] = int(report.get('wrist_checks', 0)) + 1
        self.event('wrist_target_confirmed_after_lift', stamp_ns=stamp,
                   prior_source='held_object_kinematics', measured_depth=False)

    def detach(self, *, assume_released: bool = False):
        # Manual arm return explicitly opens the jaw without object evidence.
        # Normal collection still requires its original release confirmation.
        if not self.carrying or (not self.carrying['release_confirmed'] and not assume_released):
            raise RuntimeError('Cannot detach before physical release evidence')
        item = self.carrying
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
            link_name=f"{item['arm']}_gripper_frame",
            object=CollisionObject(id=item['object_id'],operation=CollisionObject.REMOVE))]
        saved_payload_acm = item.get('payload_octomap_saved_acm')
        if saved_payload_acm is not None:
            scene.allowed_collision_matrix = deepcopy(saved_payload_acm)
        # MoveIt detachment preserves the object's current world transform.
        self.apply(scene)
        if saved_payload_acm is not None:
            query = GetPlanningScene.Request()
            query.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            restored = self.call(self.get_scene, query).scene.allowed_collision_matrix
            if restored != saved_payload_acm:
                raise RuntimeError('MoveIt did not confirm held-object collision permission restoration')
            self.event('held_object_octomap_collision_permission_restored', object_id=item['object_id'])
        self.carrying = None
        if self.payload_future is not None:
            self.payload_future.cancel()
        self.payload_future = None
        self.releasing = False
        self.event('detach')

    def after_pregrasp(self, selected, wrist_request, report,
                       before_observation=None, after_observation=None):
        self.wrist_reference_id = '' if wrist_request is None else wrist_request.reference_id
        self.wrist_request = None if wrist_request is None else deepcopy(wrist_request)
        arm = selected.selected_arm
        object_id = f'physical_payload_{time.monotonic_ns()}'
        try:
            if self.config.get('approach_execution_replanning_enabled', False):
                selected, pregrasp = self.approach_with_candidate_reselection(
                    selected, report, object_id=object_id,
                    prepare_pregrasp=lambda candidate: self.prepare_reselected_pregrasp(
                        candidate, report, wrist_request))
            else:
                self.scene.begin(selected.selected_candidate, object_id)
                self.scene.allow_contacts_for(arm)
                pregrasp = self.pose(arm)
                with self.record_approach_commands(arm):
                    if self.config.get('visual_servo_enabled', False):
                        self.visual_approach(
                            selected, before_observation, after_observation, report)
                    else:
                        self.approach_target(selected, report)
            if not self.config.get('pregrasp_motion_enabled', True):
                pregrasp = self.pose(arm)  # Release destination is now the actual grasp pose.
            if temporary_return_mode(self.config):
                self.capture_pickup(arm)
            if not contact_verification_required(self.config):
                self.jaw(arm, self.arm_config.closed, 'close_assumed',
                         check_collisions=self.config.get('grasp_collision_check_enabled', True))
                report.update(grasp_assumed=True, grasp_verified=False, contact_verification='SKIPPED')
                self.event('grasp_assumed_without_evidence')
            else:
                self.close_until_contact(selected, report)
            report['grasp_executed'] = True
            # Attached geometry is supplied to the continuously updated map.
            self.attach(selected, object_id)
            lift_collisions = self.config.get('lift_collision_check_enabled', True)
            report['lift_collision_checks_enabled'] = lift_collisions
            if not lift_collisions:
                self.event('lift_collision_checks_disabled', arm=arm)
            with self.record_pickup_departure(arm):
                # Retain the closed jaw/payload while reversing the sent approach.
                if self.config.get('pregrasp_motion_enabled', True):
                    self.retreat_to_pregrasp(arm, pregrasp, report,
                                            check_collisions=lift_collisions)
                bottom = self.pose(arm)
                top = deepcopy(bottom)
                if (self.config.get('lift_height_correction_enabled', False)
                        and self.config.get('pregrasp_motion_enabled', True)):
                    top.position.z = max(top.position.z, pregrasp.position.z)
                top.position.z += self.config['lift_height_m']
                before_lift = self.get_clock().now().nanoseconds
                self.linear(arm, top, 'lift', check_collisions=lift_collisions)
                if self.config.get('lift_height_correction_enabled', False):
                    self.correct_height_endpoint(arm, top, 'lift', report,
                                                 check_collisions=lift_collisions)
                actual = self.pose(arm)
                height = actual.position.z-bottom.position.z
                report['lift_height_from_fk_m'] = height
            if self.wrist_observation_required():
                self.confirm_held_target(selected, actual, report)
            if contact_verification_required(self.config):
                if grasp_identity_verification_required(self.config):
                    self.verify_evidence(selected, CheckManipulation.Request.LIFT, before_lift)
                    report.update(grasp_verified=True,
                                  grasp_evidence='motor_contact_and_fresh_wrist_identity_after_lift')
                else:
                    self.real.wait_for_contact(arm, before_lift)
                    report.update(grasp_verified=False, grasp_identity_verified=False,
                                  grasp_evidence='motor_contact_only_after_lift')
            self.stop_wrist_camera()
            if hold_mode(self.config):
                if return_hold_mode(self.config):
                    self.move_joint_pose(arm, self.arm_config.home, 'return_holding_payload')
                    report['returned'] = True
                else:
                    report['returned'] = False
                report.update(payload_held=True,
                              message='Grasp/lift completed; holding payload without transport or release')
                return
            if temporary_return_mode(self.config):
                report.update(self.return_to_pickup(arm, pregrasp))
                self.releasing = True
                self.jaw(arm, self.arm_config.opened, 'release_at_pickup',
                         check_collisions=self.config.get('gripper_opening_collision_check_enabled', True))
                self.carrying['release_confirmed'] = True
                self.detach()
                report.update(released=True, release_verified=False, placement_verified=False,
                              placement_state='ASSUMED',
                              contact_verification=('RAW_FEEDBACK' if contact_verification_required(self.config) else 'SKIPPED'))
                self.pickup_release_retreat(arm, pregrasp)
                self.scene.restore()
                self.move_joint_pose(arm, self.arm_config.home, 'return')
                report['returned'] = True
                return
            self.move_joint_pose(arm, self.arm_config.bins[self.destination], 'transport')
            before_release = self.get_clock().now().nanoseconds
            self.releasing = True
            self.jaw(arm, self.arm_config.opened, 'release',
                     check_collisions=self.config.get('gripper_opening_collision_check_enabled', True))
            self.verify_evidence(selected, CheckManipulation.Request.RELEASE, before_release)
            self.carrying['release_confirmed'] = True
            self.detach()
            report['released'] = True
            # End target ACM/support transaction before returning. Fresh depth
            # remains authoritative; no deletion of an uncertain held object.
            self.scene.restore()
            self.move_joint_pose(arm, self.arm_config.home, 'return')
            report['returned'] = True
        finally:
            if self.carrying is None:
                self.scene.restore()
            elif hold_mode(self.config) and report.get('payload_held'):
                report['payload_preserved_on_success'] = True
            else:
                report['payload_preserved_on_failure'] = True

    def run_collection(self, query: str):
        report = {'success':False,'cycles':[],'empty_confirmations':0}
        last_capture = 0
        try:
            for _ in range(int(self.config.get('maximum_cycles',20))):
                cycle = self.run(query)
                report['cycles'].append(cycle)
                if cycle['success']:
                    if temporary_return_mode(self.config):
                        report.update(success=True, message='Single temporary pick/return/release cycle completed; placement assumed')
                        break
                    if hold_mode(self.config):
                        report.update(success=True,
                                      message='Single grasp/lift/hold cycle completed')
                        break
                    report['empty_confirmations'] = 0
                    continue
                # No eligible target is NOT automatically an empty workspace.
                if (cycle.get('error_code') == 2 and cycle.get('detection_count') == 0
                        and 'cleanup_error' not in cycle):
                    self.real.monitor()
                    capture = cycle.get('capture_stamp_ns',0)
                    if capture <= last_capture:
                        raise RuntimeError('Empty observation did not advance')
                    last_capture = capture
                    report['empty_confirmations'] += 1
                    if report['empty_confirmations'] >= self.config['empty_confirmations']:
                        report.update(success=True,message='Consecutive empty observations confirmed')
                        break
                    continue
                raise RuntimeError(cycle.get('message','Collection failed'))
            else:
                raise RuntimeError('Cycle limit reached without confirmed empty workspace')
        except BaseException as error:
            report['message'] = str(error)
            report['stop'] = self.real.fail_stop()
        report['events'] = self.events
        report['payload_attached_or_uncertain'] = self.carrying is not None
        return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--execute', action='store_true', help='Explicitly request physical collection')
    parser.add_argument('--query', default='')
    parser.add_argument('--output', required=True)
    options, ros_args = parser.parse_known_args(args)
    config = json.loads(Path(options.config).expanduser().read_text())
    issues = configuration_issues(config)
    report = {'mode':'configuration_audit','motion_requested':False,'ready':not issues,'issues':issues}
    node = executor = thread = None
    try:
        if options.execute:
            if issues:
                raise ValueError('; '.join(issues))
            rclpy.init(args=ros_args)
            node = CollectionPipeline(config)
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)
            thread = threading.Thread(target=executor.spin,daemon=True)
            thread.start()
            report = node.run_collection(options.query)
            report['mode'] = 'hardware'
    except BaseException as error:
        report.update(success=False,message=str(error))
    finally:
        path = Path(options.output).expanduser()
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(report,ensure_ascii=False))
        if node:
            node.scene_monitor.close()
        if executor:
            executor.shutdown(timeout_sec=5.)
        if thread:
            thread.join(timeout=5.)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
    raise SystemExit(0 if report.get('success',report.get('ready')) else 1)
