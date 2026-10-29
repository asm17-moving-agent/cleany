"""ROS action server selecting the first MoveIt-reachable grasp pair."""

from __future__ import annotations

import math
import json
import threading
import time
import uuid
from dataclasses import replace
from contextlib import nullcontext
from cleany_grasping.gpu_scheduling import GpuLease
from typing import Callable, Sequence

import rclpy
from cleany_interfaces.action import SelectReachableGrasp
from cleany_interfaces.msg import GraspCandidate
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from moveit_msgs.msg import VisibilityConstraint
from cleany_skill_executor.core.visibility import enclosing_visibility_cone
from cleany_skill_executor.trajectory_io import path_to_message
from geometry_msgs.msg import Pose

from cleany_skill_executor.core.physical_grasp_selection import (
    Candidate,
    EvaluationStage,
    GraspSelectionConfig,
    GraspSelector,
    InfrastructureError,
    PoseValue,
    REQUIRED_JOINT_NAMES,
)
from cleany_skill_executor.core.alignment import tcp_axes
from cleany_skill_executor.core.grasp_direction import GraspDirectionConfig
from cleany_skill_executor.core.fixed_jaw_clearance import (
    ClearanceConfig, closing_clearance, correction_for_clearance,
)
from cleany_skill_executor.fixed_jaw_geometry import load_fixed_jaw_bounds, target_box
from cleany_skill_executor.sensor_scene import SensorSceneMonitor
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter
from cleany_skill_executor.planning_scene import SceneAwarePort, TargetSceneTransaction


STAGE_CONSTANT = {
    EvaluationStage.OPENING: SelectReachableGrasp.Feedback.STAGE_OPENING,
    EvaluationStage.PREGRASP_IK: SelectReachableGrasp.Feedback.STAGE_PREGRASP_IK,
    EvaluationStage.GRASP_IK: SelectReachableGrasp.Feedback.STAGE_GRASP_IK,
    EvaluationStage.STATE_VALIDITY: SelectReachableGrasp.Feedback.STAGE_STATE_VALIDITY,
    EvaluationStage.PLAN_PREGRASP: SelectReachableGrasp.Feedback.STAGE_PLAN_PREGRASP,
    EvaluationStage.PLAN_GRASP: SelectReachableGrasp.Feedback.STAGE_PLAN_GRASP,
    EvaluationStage.POSE_ALIGNMENT: (
        SelectReachableGrasp.Feedback.STAGE_POSE_ALIGNMENT
    ),
    EvaluationStage.TRAJECTORY_ALIGNMENT: (
        SelectReachableGrasp.Feedback.STAGE_TRAJECTORY_ALIGNMENT
    ),
}


from cleany_skill_executor.core.gripper import aperture_centering_offset, approach_offset_for_label
from cleany_skill_executor.collision_geometry_cache import CollisionGeometryCache, subscribe_collision_geometry


class GraspSelectionNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('grasp_selection_server', **kwargs)
        defaults = {
            'action_name': 'grasp/select_reachable',
            'motion_backend': 'moveit',
            'curobo_num_seeds': 32,
            'curobo_ik_primary_seeds': 16,
            'curobo_ik_fallback_seeds': 32,
            'curobo_ik_batch_size': 8,
            'curobo_ik_iterations': 50,
            'curobo_ik_exclude_opposite_arm_cost': True,
            'curobo_ik_retry_failed_candidates': True,
            'curobo_ik_pregrasp_retry_passes': 1,
            'curobo_ik_adaptive_batch_size': False,
            'curobo_ik_progressive_pregrasp': False,
            'curobo_ik_group_gripper_openings': False,
            'curobo_ik_defer_candidate_retries': False,
            'curobo_ik_seeded_local_ik': False,
            'curobo_ik_warmup_enabled': False,
            'curobo_ik_warmup_arms': ['right', 'left'],
            'curobo_ik_warmup_include_aim': True,
            'curobo_ik_warmup_timeout_sec': 180.0,
            'curobo_ik_vlm_warmup_enabled': False,
            'curobo_ik_vlm_warmup_arms': ['right', 'left'],
            'curobo_ik_vlm_warmup_window_sec': 30.0,
            'state_validity_batch_size': 32,
            'curobo_ik_cpu_fallback': False,
            'curobo_planning_retries': 1,
            'use_verified_local_fk': False,
            'curobo_ik_position_threshold_m': 0.001,
            'curobo_joint_tolerance_rad': 0.002,
            'curobo_spheres_per_link': 64,
            'curobo_sphere_budget_per_link': 16,
            'curobo_sphere_model': 'joint_simplified',
            'curobo_sphere_surface_radius_m': 0.0005,
            'curobo_geometry_cache_directory': '',
            'curobo_interpolation_dt': 0.05,
            'curobo_validation_step_rad': 0.025,
            'curobo_maximum_validation_states': 2000,
            'curobo_max_acceleration': 1.0,
            'curobo_max_jerk': 10.0,
            'curobo_use_cuda_graph': True,
            'curobo_collision_margin_m': 0.0001,
            'curobo_collision_hull_max_expansion_m': 0.001,
            'curobo_moveit_shadow_validation': False,
            'joint_state_topic': 'joint_states',
            'planning_frame': 'base_link',
            'joint_state_max_age_sec': 0.5,
            'ik_timeout_sec': 0.15,
            'ik_response_margin_sec': 1.0,
            'fk_timeout_sec': 1.0,
            'state_validity_timeout_sec': 5.0,
            'planning_scene_timeout_sec': 5.0,
            'planning_timeout_sec': 4.0,
            'planning_response_margin_sec': 1.0,
            'planning_attempts': 1,
            'pregrasp_joint_shortcut_enabled': False,
            'cartesian_approach_first': False,
            'approach_joint_shortcut_enabled': False,
            'cartesian_approach_step_m': 0.005,
            'velocity_scaling': 0.1,
            'acceleration_scaling': 0.1,
            'maximum_candidates': 12,
            'stop_after_first_feasible': False,
            'fast_candidate_paths_first': False,
            'deduplicate_candidates': False,
            'prefer_near_wrist_grasp_direction': False,
            'wrist_direction_tie_tolerance_degrees': 5.0,
            'grasp_direction_pair_position_tolerance_m': 1e-5,
            'grasp_direction_pair_axis_tolerance_degrees': 1.0,
            'grasp_direction_pair_score_tolerance': 1e-6,
            'pregrasp_motion_enabled': True,
            'direct_grasp_shortcut_enabled': False,
            'grasp_ik_first': False,
            'grasp_approach_offset_m': 0.0,
            'grasp_lateral_offset_m': 0.0,
            'grasp_use_aperture_centering': False,
            'grasp_aperture_margin_m': 0.008,
            'grasp_fixed_jaw_inner_x_m': 0.008,
            'grasp_fixed_jaw_clearance_m': 0.0,
            'deeper_grasp_labels': [''],
            'deeper_grasp_offsets_m': [0.0],
            'use_observed_collision_geometry': False,
            'collision_geometry_topic': '/grasp/collision_geometry',
            'action_timeout_sec': 120.0,
            'robot_description_startup_timeout_sec': 30.0,
            'pregrasp_offset_m': 0.08,
            'pregrasp_fallback_offset_m': 0.0,
            'planner_id': 'RRTConnectkConfigDefault',
            'aim_tip_offset_m': 0.14,
            'ik_seed_wrist_offsets_degrees': [0.0, -45.0, 45.0, -90.0, 90.0],
            'ik_seed_nominal_postures_rad': [
                0.0, 0.65, 0.80, -0.40, 0.0,
                0.0, 1.10, 1.70, -0.90, 0.0,
            ],
            'maximum_gripper_width_m': 0.10,
            'gripper_safety_margin_m': 0.005,
            'gripper_closed_joint_rad': -0.374532977628,
            'gripper_open_joint_rad': 1.74532919957,
            'approach_tolerance_degrees': 15.0,
            'closing_tolerance_degrees': 30.0,
            'position_tolerance_m': 0.002,
            'pregrasp_position_tolerance_m': 0.0,
            'trajectory_maximum_waypoints': 20,
            'align_wrist_roll': False,
            'pose_refinement_iterations': 0,
            'validate_motion_paths': False,
            'validation_step_rad': 0.025,
            'maximum_validation_states': 2000,
            'approach_corridor_radius_m': 0.01,
            'joint_acceleration_limit': 1.0,
            'require_approach_corridor': False,
            'require_gripper_closure_clearance': False,
            'gripper_sweep_step_rad': 0.05,
            'asymmetric_gripper_enabled': False,
            'fixed_jaw_closing_axis_sign': 1.0,
            'fixed_jaw_tcp_offset_m': 0.0,
            'fixed_jaw_additional_bias_m': 0.0,
            'fixed_jaw_clearance_enabled': False,
            'fixed_jaw_minimum_clearance_m': 0.005,
            'fixed_jaw_maximum_correction_m': 0.010,
            'fixed_jaw_collision_meshes': ['Fixed_Jaw_part2.ply.convex.stl'],
            'support_patch_margin_m': 0.0,
            'require_pregrasp_visibility': False,
            'visibility_camera_frame': 'head_camera_rgb_optical_frame',
            'visibility_camera_max_age_sec': 0.5,
            'visibility_padding_m': 0.003,
            'visibility_cone_sides': 16,
        }
        defaults['gpu_lock_path'] = ''
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        # Empty Python lists otherwise infer BYTE_ARRAY in ROS Humble.
        fallback_name = 'pregrasp_fallback_offsets_m'
        self.declare_parameter(fallback_name, Parameter.Type.DOUBLE_ARRAY)
        if self.get_parameter_or(fallback_name).value is None:
            self.set_parameters([Parameter(fallback_name, Parameter.Type.DOUBLE_ARRAY, [])])
        self._sensor_scene = SensorSceneMonitor(self)
        if self._sensor_scene.required and self.get_parameter("motion_backend").value not in ('moveit', 'curobo_ik_moveit'):
            raise ValueError("Sensor OctoMap requires the MoveIt backend; cuRobo conversion is unavailable")
        self._visibility_tf = None
        if self.get_parameter('require_pregrasp_visibility').value:
            self._visibility_tf = Buffer()
            self._visibility_listener = TransformListener(self._visibility_tf, self)
        self._joint_state: JointState | None = None
        self._joint_state_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._goal_active = False
        self._adapter_lock = threading.RLock()
        self._gpu_lease = GpuLease(str(self.get_parameter('gpu_lock_path').value))
        callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_state_topic').value),
            self._on_joint_state,
            20,
            callback_group=callback_group,
        )

        def wait(duration: float) -> None:
            time.sleep(min(duration, 0.01))

        backend = str(self.get_parameter('motion_backend').value)
        adapter_class = MoveItGraspAdapter
        adapter_options = {}
        if backend in ('curobo', 'curobo_ik_moveit'):
            from cleany_skill_executor.curobo_adapter import CuRoboGraspAdapter
            from cleany_skill_executor.curobo_model import CuRoboConfig

            adapter_class = CuRoboGraspAdapter
            adapter_options['gpu_config'] = CuRoboConfig(**{
                name: self.get_parameter(f'curobo_{name}').value
                for name in CuRoboConfig.__dataclass_fields__
            })
            if backend == 'curobo_ik_moveit':
                from dataclasses import replace
                from cleany_skill_executor.hybrid_adapter import HybridConfig, HybridGraspAdapter

                adapter_class = HybridGraspAdapter
                adapter_options['gpu_config'] = replace(adapter_options['gpu_config'],
                    num_seeds=int(self.get_parameter('curobo_ik_primary_seeds').value))
                adapter_options['hybrid_config'] = HybridConfig(
                    batch_size=int(self.get_parameter('curobo_ik_batch_size').value),
                    ik_iterations=int(self.get_parameter('curobo_ik_iterations').value),
                    exclude_opposite_arm_cost=self.get_parameter('curobo_ik_exclude_opposite_arm_cost').value,
                    retry_failed_candidates=self.get_parameter('curobo_ik_retry_failed_candidates').value,
                    pregrasp_retry_passes=self.get_parameter('curobo_ik_pregrasp_retry_passes').value,
                    adaptive_batch_size=self.get_parameter('curobo_ik_adaptive_batch_size').value,
                    progressive_pregrasp=self.get_parameter('curobo_ik_progressive_pregrasp').value,
                    defer_candidate_ik_retries=self.get_parameter('curobo_ik_defer_candidate_retries').value,
                    seeded_local_ik=self.get_parameter('curobo_ik_seeded_local_ik').value,
                    group_gripper_openings=self.get_parameter('curobo_ik_group_gripper_openings').value,
                    cpu_ik_fallback=self.get_parameter('curobo_ik_cpu_fallback').value,
                    planning_retries=self.get_parameter('curobo_planning_retries').value,
                    fallback_seeds=int(self.get_parameter('curobo_ik_fallback_seeds').value))
        elif backend != 'moveit':
            raise ValueError(f'unknown motion_backend: {backend}')
        self._adapter = adapter_class(
            self,
            MoveItAdapterConfig(
                state_validity_batch_size=int(self.get_parameter('state_validity_batch_size').value),
                **{name: self.get_parameter(name).value for name in (
                    'align_wrist_roll', 'pose_refinement_iterations', 'validate_motion_paths',
                    'validation_step_rad', 'maximum_validation_states',
                    'approach_corridor_radius_m', 'joint_acceleration_limit',
                    'use_verified_local_fk')},
                base_frame=str(self.get_parameter('planning_frame').value),
                ik_timeout_sec=float(self.get_parameter('ik_timeout_sec').value),
                ik_response_margin_sec=float(self.get_parameter('ik_response_margin_sec').value),
                fk_timeout_sec=float(self.get_parameter('fk_timeout_sec').value),
                state_validity_timeout_sec=float(self.get_parameter('state_validity_timeout_sec').value),
                planning_timeout_sec=float(self.get_parameter('planning_timeout_sec').value),
                planning_response_margin_sec=float(self.get_parameter('planning_response_margin_sec').value),
                planning_attempts=int(self.get_parameter('planning_attempts').value),
                pregrasp_joint_shortcut_enabled=bool(
                    self.get_parameter('pregrasp_joint_shortcut_enabled').value),
                cartesian_approach_first=self.get_parameter('cartesian_approach_first').value,
                approach_joint_shortcut_enabled=self.get_parameter('approach_joint_shortcut_enabled').value,
                cartesian_approach_step_m=float(self.get_parameter('cartesian_approach_step_m').value),
                velocity_scaling=float(self.get_parameter('velocity_scaling').value),
                acceleration_scaling=float(self.get_parameter('acceleration_scaling').value),
                planner_id=str(self.get_parameter('planner_id').value),
                aim_tip_offset_m=float(
                    self.get_parameter('aim_tip_offset_m').value
                ),
                ik_seed_wrist_offsets_degrees=tuple(
                    float(value)
                    for value in self.get_parameter(
                        'ik_seed_wrist_offsets_degrees'
                    ).value
                ),
                ik_seed_nominal_postures_rad=self._nominal_postures(),
                position_tolerance_m=float(
                    self.get_parameter('position_tolerance_m').value
                ),
                pregrasp_position_tolerance_m=(float(
                    self.get_parameter('pregrasp_position_tolerance_m').value) or None),
                approach_tolerance_degrees=float(
                    self.get_parameter('approach_tolerance_degrees').value
                ),
                closing_tolerance_degrees=float(
                    self.get_parameter('closing_tolerance_degrees').value
                ),
            ),
            spin_once=wait,
            **adapter_options,
        )
        self.get_logger().info(f'Motion backend: {self._adapter.planner_id}')

        self._description_error = ''
        self._description_ready = False
        self._fixed_jaw_config = None
        self._fixed_jaw_models = {}
        if self.get_parameter('fixed_jaw_clearance_enabled').value:
            if not self.get_parameter('asymmetric_gripper_enabled').value:
                raise ValueError('Fixed-jaw clearance requires the asymmetric gripper model')
            self._fixed_jaw_config = ClearanceConfig(
                float(self.get_parameter('fixed_jaw_minimum_clearance_m').value),
                float(self.get_parameter('fixed_jaw_maximum_correction_m').value))
        prefer_direction = self.get_parameter('prefer_near_wrist_grasp_direction').value
        if prefer_direction and self.get_parameter('planning_frame').value != 'base_link':
            raise ValueError('Grasp direction left preference requires planning_frame=base_link')
        self._requires_description = (backend == 'curobo_ik_moveit'
                                      or self._fixed_jaw_config is not None or prefer_direction)
        self.create_subscription(String, '/robot_description', self._on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=callback_group)
        self._geometry_cache = CollisionGeometryCache()
        use_mesh = self.get_parameter('use_observed_collision_geometry').value
        self._geometry_subscription = subscribe_collision_geometry(self, self._geometry_cache) if use_mesh else None
        self._scene = TargetSceneTransaction(self, spin_once=wait,
            timeout_sec=float(self.get_parameter('planning_scene_timeout_sec').value),
            support_patch_margin_m=float(self.get_parameter('support_patch_margin_m').value),
            geometry_lookup=self._geometry_cache.get if use_mesh else None)
        self._scene_port = SceneAwarePort(self._adapter, self._scene)
        self._selector = GraspSelector(
            self._scene_port,
            GraspSelectionConfig(
                direction_preference=GraspDirectionConfig(
                    enabled=prefer_direction,
                    movement_tie_degrees=float(self.get_parameter(
                        'wrist_direction_tie_tolerance_degrees').value),
                    pair_position_tolerance_m=float(self.get_parameter(
                        'grasp_direction_pair_position_tolerance_m').value),
                    pair_axis_tolerance_degrees=float(self.get_parameter(
                        'grasp_direction_pair_axis_tolerance_degrees').value),
                    pair_score_tolerance=float(self.get_parameter(
                        'grasp_direction_pair_score_tolerance').value)),
                **{name: self.get_parameter(name).value for name in (
                    'require_approach_corridor', 'require_gripper_closure_clearance',
                    'gripper_sweep_step_rad', 'require_pregrasp_visibility',
                    'asymmetric_gripper_enabled', 'stop_after_first_feasible',
                    'deduplicate_candidates', 'fast_candidate_paths_first',
                    'pregrasp_motion_enabled', 'direct_grasp_shortcut_enabled',
                    'grasp_ik_first')},
                pregrasp_offset_m=float(self.get_parameter('pregrasp_offset_m').value),
                pregrasp_fallback_offset_m=float(self.get_parameter('pregrasp_fallback_offset_m').value),
                pregrasp_fallback_offsets_m=tuple(self.get_parameter('pregrasp_fallback_offsets_m').value),
                maximum_candidates=int(self.get_parameter('maximum_candidates').value),
                maximum_gripper_width_m=float(
                    self.get_parameter('maximum_gripper_width_m').value
                ),
                gripper_safety_margin_m=float(
                    self.get_parameter('gripper_safety_margin_m').value
                ),
                gripper_closed_joint_rad=float(
                    self.get_parameter('gripper_closed_joint_rad').value
                ),
                gripper_open_joint_rad=float(
                    self.get_parameter('gripper_open_joint_rad').value
                ),
                approach_tolerance_degrees=float(
                    self.get_parameter('approach_tolerance_degrees').value
                ),
                closing_tolerance_degrees=float(
                    self.get_parameter('closing_tolerance_degrees').value
                ),
                position_tolerance_m=float(
                    self.get_parameter('position_tolerance_m').value
                ),
                pregrasp_position_tolerance_m=(float(
                    self.get_parameter('pregrasp_position_tolerance_m').value) or None),
                trajectory_maximum_waypoints=int(
                    self.get_parameter('trajectory_maximum_waypoints').value
                ),
                fixed_jaw_closing_axis_sign=float(
                    self.get_parameter('fixed_jaw_closing_axis_sign').value
                ),
                fixed_jaw_tcp_offset_m=float(
                    self.get_parameter('fixed_jaw_tcp_offset_m').value
                ),
                fixed_jaw_additional_bias_m=float(
                    self.get_parameter('fixed_jaw_additional_bias_m').value
                ),
            ),
        )
        self._server = ActionServer(
            self,
            SelectReachableGrasp,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=callback_group,
        )
        self._warmup_enabled = (backend == 'curobo_ik_moveit'
                                and self.get_parameter('curobo_ik_warmup_enabled').value)
        self._warmup_ready = not self._warmup_enabled
        self._warmup_error = ''
        self._warmup_description = ''
        self._warmup_service = self.create_service(
            Trigger, '/grasp/warmup_status', self._warmup_status, callback_group=callback_group)
        self._warmup_timer = self.create_timer(1., self._try_warmup, callback_group=callback_group)
        self._vlm_warmup_enabled = (backend == 'curobo_ik_moveit'
            and self.get_parameter('curobo_ik_vlm_warmup_enabled').value)
        self._vlm_warmup_lock = threading.Lock()
        self._vlm_warmup_until = 0.
        self._vlm_warmup_completed: set[str] = set()
        self._vlm_warmup_failed: set[str] = set()
        self._vlm_warmup_preferred_arm = ''
        self._vlm_warmup_description = ''
        self._vlm_warmup_service = self.create_service(SetBool, '/grasp/vlm_warmup',
            self._vlm_warmup_request, callback_group=callback_group)
        self._vlm_warmup_timer = self.create_timer(.1, self._try_vlm_warmup,
            callback_group=callback_group)

    def _on_description(self, message):
        with getattr(self, '_adapter_lock', nullcontext()):
            GraspSelectionNode._update_description(self, message)

    def _update_description(self, message):
        try:
            self._adapter.set_robot_description(message.data)
            if getattr(self, '_fixed_jaw_config', None) is not None:
                meshes = self.get_parameter('fixed_jaw_collision_meshes').value
                self._fixed_jaw_models = {
                    arm: load_fixed_jaw_bounds(message.data, arm, meshes)
                    for arm in ('left', 'right')}
            self._description_error = ''
            self._description_ready = True
            if (hasattr(self, '_vlm_warmup_lock')
                    and self._vlm_warmup_description != message.data):
                with self._vlm_warmup_lock:
                    self._vlm_warmup_completed.clear()
                    self._vlm_warmup_failed.clear()
                    self._vlm_warmup_description = message.data
            if (getattr(self, '_warmup_enabled', False)
                    and self._warmup_description != message.data):
                self._warmup_ready = False
                self._warmup_error = ''
        except (ValueError, TypeError, OSError) as error:
            self._description_error = str(error)
            self._description_ready = False
            self.get_logger().error(f'Invalid robot_description: {error}')

    def _warmup_status(self, _request, response):
        response.success = self._warmup_ready and not self._description_error
        response.message = (self._description_error or self._warmup_error
                            or ('IK warmup ready' if response.success else 'IK warmup pending'))
        return response

    def _vlm_warmup_request(self, request, response):
        if not self._vlm_warmup_enabled:
            response.success, response.message = False, 'VLM IK warmup is disabled'
            return response
        duration = float(self.get_parameter('curobo_ik_vlm_warmup_window_sec').value)
        if not math.isfinite(duration) or duration <= 0:
            response.success, response.message = False, 'Invalid VLM warmup window'
            return response
        with self._vlm_warmup_lock:
            self._vlm_warmup_until = time.monotonic() + duration if request.data else 0.
            if request.data:
                self._vlm_warmup_failed.clear()
        response.success = True
        response.message = 'VLM warmup window opened' if request.data else 'VLM warmup window closed'
        return response

    def _vlm_warmup_active(self):
        with self._vlm_warmup_lock:
            lease = getattr(self, '_gpu_lease', None)
            return (not self._goal_active and time.monotonic() < self._vlm_warmup_until
                    and (lease is None or not lease.foreground_pending()))

    def _remember_warmup_arm(self, arm: str) -> None:
        """Prioritize a known requested/selected arm in the next idle window."""
        lock = getattr(self, '_vlm_warmup_lock', None)
        if arm in ('left', 'right') and lock is not None:
            with lock:
                self._vlm_warmup_preferred_arm = arm

    def _try_vlm_warmup(self):
        if (not self._vlm_warmup_enabled or not self._description_ready
                or not self._vlm_warmup_active()
                or not self._adapter_lock.acquire(blocking=False)):
            return
        began, arm = time.monotonic(), None
        try:
            lease = getattr(self, '_gpu_lease', None)
            admission = lease.acquire(foreground=False, blocking=False) if lease is not None else nullcontext(True)
            with admission as admitted:
                if not admitted:
                    return
                if not self._vlm_warmup_active():
                    return
                arms = self.get_parameter('curobo_ik_vlm_warmup_arms').value
                if not arms or any(value not in ('left', 'right') for value in arms):
                    raise ValueError('VLM warmup arms must be left/right')
                with self._vlm_warmup_lock:
                    preferred = getattr(self, '_vlm_warmup_preferred_arm', '')
                    if preferred in arms:
                        arms = [preferred, *(value for value in arms if value != preferred)]
                    arm = next((value for value in arms if value not in
                        self._vlm_warmup_completed | self._vlm_warmup_failed), None)
                    deadline = self._vlm_warmup_until
                if arm is None:
                    return
                try:
                    state = self._current_joint_state()
                except (TimeoutError, ValueError):
                    return  # No fresh joints yet; never block perception for them.
                self._adapter.set_current_state(state)
                self._adapter.set_deadline(deadline)
                self._adapter.set_progress_callback(
                    lambda message: self.get_logger().info('vlm_warmup: ' + message))
                # One arm per timer callback; a real request takes priority between
                # arms. No auxiliary aim TCP or fallback seed solvers are created.
                self._adapter.warmup([arm], include_aim=False,
                    continue_work=self._vlm_warmup_active)
                with self._vlm_warmup_lock:
                    self._vlm_warmup_completed.add(arm)
                self.get_logger().info(json.dumps({'event': 'vlm_ik_warmup',
                    'arm': arm, 'wall_sec': time.monotonic() - began, 'outcome': 'ready'}))
        except InterruptedError:
            self.get_logger().info('VLM IK warmup yielded; initialized buffers retained')
        except Exception as error:
            with self._vlm_warmup_lock:
                if arm is not None:
                    self._vlm_warmup_failed.add(arm)
                else:
                    self._vlm_warmup_until = 0.
            self.get_logger().warning(f'Optional VLM IK warmup failed: {error}')
        finally:
            self._adapter.set_progress_callback(None)
            self._adapter.set_deadline(math.inf)
            self._adapter_lock.release()

    def _try_warmup(self):
        if (self._warmup_ready or self._warmup_error or not self._description_ready
                or self._goal_active or not self._adapter_lock.acquire(blocking=False)):
            return
        try:
            lease = getattr(self, '_gpu_lease', None)
            admission = lease.acquire(foreground=False, blocking=False) if lease is not None else nullcontext(True)
            with admission as admitted:
                if not admitted:
                    return
                try:
                    state = self._current_joint_state()
                except (TimeoutError, ValueError):
                    return  # Wait for the complete, fresh measured joint snapshot.
                timeout = float(self.get_parameter('curobo_ik_warmup_timeout_sec').value)
                if not math.isfinite(timeout) or timeout <= 0:
                    raise ValueError('IK warmup timeout must be positive')
                self._adapter.set_current_state(state)
                self._adapter.set_deadline(time.monotonic() + timeout)
                self._adapter.set_progress_callback(lambda message: self.get_logger().info('warmup: ' + message))
                began = time.monotonic()
                self._adapter.warmup(self.get_parameter('curobo_ik_warmup_arms').value,
                    include_aim=self.get_parameter('curobo_ik_warmup_include_aim').value,
                    continue_work=lambda: not self._goal_active and
                        (lease is None or not lease.foreground_pending()))
                self._warmup_description = self._adapter._description
                self._warmup_ready = True
                self.get_logger().info('Primary GPU IK warmup complete; results discarded')
                self.get_logger().info(json.dumps({'event': 'startup_ik_warmup',
                    'arms': list(self.get_parameter('curobo_ik_warmup_arms').value),
                    'include_aim': self.get_parameter('curobo_ik_warmup_include_aim').value,
                    'wall_sec': time.monotonic() - began, 'outcome': 'ready'}))
        except InterruptedError as error:
            if lease is not None and lease.foreground_pending():
                self.get_logger().info('IK warmup yielded to foreground GPU work')
            elif self._goal_active:
                self.get_logger().warning(f'IK warmup interrupted by selection cancellation: {error}')
            else:
                self._warmup_error = f'IK warmup failed: {error}'
                self.get_logger().error(self._warmup_error)
        except Exception as error:
            self._warmup_error = f'IK warmup failed: {error}'
            self.get_logger().error(self._warmup_error)
        finally:
            self._adapter.set_progress_callback(None)
            self._adapter.set_deadline(math.inf)
            self._adapter_lock.release()

    def _prepare_fixed_jaw_candidates(
        self, candidates: Sequence[Candidate], messages: Sequence[GraspCandidate],
        required_arm: str, feedback: Callable, canceled: Callable[[], bool],
    ) -> Sequence[Candidate]:
        config = getattr(self, '_fixed_jaw_config', None)
        if config is None:
            return candidates
        if required_arm not in ('', 'left', 'right'):
            raise ValueError('required_arm must be empty, left, or right')
        selector = self._selector
        side = selector._config.fixed_jaw_closing_axis_sign
        arms = (required_arm,) if required_arm else ('left', 'right')
        # Keep the existing score-ranked candidate budget even when geometry
        # rejects candidates before the batched IK call.
        ranked = sorted(candidates, key=lambda c: c.score, reverse=True)[:selector._config.maximum_candidates]
        corrected = []
        for candidate in ranked:
            if canceled():
                raise InterruptedError('Fixed-jaw correction canceled')
            grasp = selector.grasp_tcp_position(candidate)
            pregrasp = selector.pregrasp_position(candidate, selector._config.pregrasp_offset_m, grasp)
            poses = [PoseValue(position, candidate.orientation_xyzw) for position in (pregrasp, grasp)]
            corners = target_box(messages[candidate.source_index].target_object).corners()
            gaps = {arm: closing_clearance(self._fixed_jaw_models[arm], corners, poses, side)
                    for arm in arms}
            base = (selector._config.fixed_jaw_tcp_shift_m(candidate.required_opening_m)
                    if candidate.lateral_offset_m is None else candidate.lateral_offset_m)
            audit = {'candidate': candidate.source_index, 'initial_gap_m': gaps,
                     'minimum_gap_m': config.minimum_m, 'base_lateral_offset_m': base}
            try:
                extra = max(correction_for_clearance(gap, config) for gap in gaps.values())
            except ValueError as error:
                self.get_logger().info('fixed_jaw_clearance=' + json.dumps({
                    **audit, 'accepted': False, 'reason': str(error)}, allow_nan=False))
                feedback(candidate.source_index, required_arm, EvaluationStage.POSE_ALIGNMENT, str(error))
                continue
            corrected.append(replace(candidate, lateral_offset_m=base + side * extra))
            self.get_logger().info('fixed_jaw_clearance=' + json.dumps({
                **audit, 'accepted': True, 'additional_correction_m': extra,
                'lateral_offset_m': base + side * extra}, allow_nan=False))
        return corrected

    def _wait_for_description(self, goal_handle, deadline: float) -> None:
        if not self._requires_description:
            return
        timeout = float(self.get_parameter('robot_description_startup_timeout_sec').value)
        if not math.isfinite(timeout) or timeout <= 0:
            raise InfrastructureError('robot_description startup timeout must be positive')
        limit = min(deadline, time.monotonic() + timeout)
        while not self._description_ready:
            if goal_handle.is_cancel_requested:
                raise InterruptedError('Robot description wait canceled')
            if self._description_error:
                raise InfrastructureError(self._description_error)
            if time.monotonic() >= limit:
                raise InfrastructureError('Runtime robot_description was not received before startup timeout')
            time.sleep(.01)

    def _nominal_postures(self) -> tuple[tuple[float, ...], ...]:
        values = tuple(
            float(value)
            for value in self.get_parameter(
                'ik_seed_nominal_postures_rad'
            ).value
        )
        if len(values) % 5:
            raise ValueError(
                'ik_seed_nominal_postures_rad must contain five values per posture'
            )
        return tuple(
            values[index:index + 5] for index in range(0, len(values), 5)
        )

    def _on_joint_state(self, message: JointState) -> None:
        with self._joint_state_lock:
            self._joint_state = message

    def _on_goal(self, request) -> GoalResponse:
        try:
            if request.required_arm not in ("", "left", "right"):
                raise ValueError("Invalid required_arm")
            self._validate_candidates(
                request.candidates,
                str(self.get_parameter('planning_frame').value),
            )
        except ValueError as error:
            self.get_logger().warning(f'Rejecting invalid grasp goal: {error}')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_active:
                return GoalResponse.REJECT
            self._goal_active = True
        GraspSelectionNode._remember_warmup_arm(self, request.required_arm)
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        self._adapter.cancel_active()
        return CancelResponse.ACCEPT

    @staticmethod
    def _validate_candidates(candidates, planning_frame: str) -> None:
        if not candidates:
            raise ValueError('at least one candidate is required')
        if not planning_frame:
            raise ValueError('planning_frame must not be empty')
        first = candidates[0]
        if not first.snapshot_id or first.object_id == 0 or not first.header.frame_id:
            raise ValueError('candidate snapshot, object, and frame are required')
        if first.target_object.object_id != first.object_id:
            raise ValueError('target OBB object ID does not match candidate')
        if first.header.frame_id != planning_frame:
            raise ValueError(
                'candidate frame must match configured planning_frame '
                f'{planning_frame!r}'
            )
        size = first.target_object.obb_size
        pose = first.target_object.obb_pose
        values = (
            size.x, size.y, size.z,
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values) or min(size.x, size.y, size.z) <= 0:
            raise ValueError('target OBB must have finite positive dimensions and pose')
        norm = math.sqrt(sum(value * value for value in values[-4:]))
        if not math.isclose(norm, 1.0, abs_tol=1e-4):
            raise ValueError('target OBB quaternion must be normalized')
        for candidate in candidates:
            if (
                candidate.snapshot_id != first.snapshot_id
                or candidate.object_id != first.object_id
                or candidate.header.frame_id != first.header.frame_id
                or candidate.target_object != first.target_object
            ):
                raise ValueError('all candidates must share snapshot, object, frame, and OBB')
            candidate_values = (
                candidate.tcp_pose.position.x,
                candidate.tcp_pose.position.y,
                candidate.tcp_pose.position.z,
                candidate.tcp_pose.orientation.x,
                candidate.tcp_pose.orientation.y,
                candidate.tcp_pose.orientation.z,
                candidate.tcp_pose.orientation.w,
                candidate.approach_direction.x,
                candidate.approach_direction.y,
                candidate.approach_direction.z,
                candidate.required_opening_m,
                candidate.grasp_depth_m,
                candidate.score,
            )
            if not all(math.isfinite(value) for value in candidate_values):
                raise ValueError('candidate pose, opening, depth, and score must be finite')
            quaternion_norm = math.sqrt(
                sum(value * value for value in candidate_values[3:7])
            )
            if not math.isclose(quaternion_norm, 1.0, abs_tol=1e-4):
                raise ValueError('candidate TCP quaternion must be normalized')
            approach_norm = math.sqrt(
                sum(value * value for value in candidate_values[7:10])
            )
            if approach_norm <= 1e-9:
                raise ValueError('candidate approach direction must be non-zero')
            requested_approach, _ = tcp_axes(
                tuple(candidate_values[3:7])
            )
            provided_approach = tuple(
                value / approach_norm for value in candidate_values[7:10]
            )
            cosine = sum(
                left * right
                for left, right in zip(
                    requested_approach, provided_approach, strict=True
                )
            )
            if cosine < math.cos(math.radians(0.1)):
                raise ValueError(
                    'candidate approach direction does not match TCP local -Y'
                )
            if candidate.required_opening_m <= 0.0 or candidate.grasp_depth_m < 0.0:
                raise ValueError('candidate opening/depth is invalid')

    def _current_joint_state(self) -> JointState:
        with self._joint_state_lock:
            state = self._joint_state
        if state is None:
            raise ValueError('joint state is incomplete')
        missing = set(REQUIRED_JOINT_NAMES) - set(state.name)
        if missing or len(state.position) != len(state.name):
            raise ValueError(f'joint state is incomplete: {sorted(missing)}')
        stamp_ns = state.header.stamp.sec * 1_000_000_000 + state.header.stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        # Independent /clock and JointState callbacks may arrive out of order.
        # Wait briefly in simulation, then retain the strict nonfuture check.
        deadline = time.monotonic()+.05
        while (self.get_parameter('use_sim_time').value
               and 0 < stamp_ns-now_ns <= 50_000_000 and time.monotonic() < deadline):
            time.sleep(.001)
            now_ns = self.get_clock().now().nanoseconds
        max_age_ns = int(float(self.get_parameter('joint_state_max_age_sec').value) * 1e9)
        if stamp_ns <= 0 or not 0 <= now_ns - stamp_ns <= max_age_ns:
            raise TimeoutError('joint state is stale')
        return state

    def _visibility_constraint(self, candidate) -> VisibilityConstraint:
        frame = str(self.get_parameter('planning_frame').value)
        camera_frame = str(self.get_parameter('visibility_camera_frame').value)
        try:
            transform = self._visibility_tf.lookup_transform(
                frame, camera_frame, Time(), timeout=Duration(seconds=1.0))
        except Exception as error:
            raise InfrastructureError(f'visibility camera TF unavailable: {error}') from error
        stamp = transform.header.stamp.sec*1_000_000_000 + transform.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp)/1e9
        maximum_age = float(self.get_parameter('visibility_camera_max_age_sec').value)
        if (not math.isfinite(maximum_age) or maximum_age <= 0.
                or (stamp != 0 and not 0 <= age <= maximum_age)):
            raise InfrastructureError('visibility camera TF is stale or its age limit is invalid')
        camera = transform.transform.translation
        obj = candidate.target_object
        p, q, size = obj.obb_pose.position, obj.obb_pose.orientation, obj.obb_size
        cone = enclosing_visibility_cone(
            (camera.x, camera.y, camera.z), (p.x, p.y, p.z),
            (size.x, size.y, size.z), (q.x, q.y, q.z, q.w),
            padding_m=float(self.get_parameter('visibility_padding_m').value),
            sides=int(self.get_parameter('visibility_cone_sides').value))
        message = VisibilityConstraint()
        message.target_radius = cone.radius_m
        message.cone_sides = cone.sides
        message.target_pose.header.frame_id = frame
        message.target_pose.pose.position.x, message.target_pose.pose.position.y, message.target_pose.pose.position.z = cone.target
        (message.target_pose.pose.orientation.x, message.target_pose.pose.orientation.y,
         message.target_pose.pose.orientation.z, message.target_pose.pose.orientation.w) = cone.orientation
        message.sensor_pose.header.frame_id = frame
        message.sensor_pose.pose.position.x, message.sensor_pose.pose.position.y, message.sensor_pose.pose.position.z = cone.camera
        message.sensor_pose.pose.orientation.w = 1.
        message.weight = 1.
        self.get_logger().info(f'Pregrasp visibility envelope radius={cone.radius_m:.4f}m camera={camera_frame}')
        return message

    def _execute(self, goal_handle):
        lock = getattr(self, '_adapter_lock', None)
        if lock is None:
            return GraspSelectionNode._execute_locked(self, goal_handle)
        deadline = time.monotonic() + float(self.get_parameter('action_timeout_sec').value)
        try:
            # Description callbacks need the same lock; wait before acquiring it.
            self._wait_for_description(goal_handle, deadline)
            while not lock.acquire(timeout=.01):
                if goal_handle.is_cancel_requested or time.monotonic() >= deadline:
                    raise InterruptedError('IK warmup wait canceled or timed out')
            try:
                lease = getattr(self, '_gpu_lease', None)
                admission = (lease.acquire(continue_work=lambda:
                    not goal_handle.is_cancel_requested and time.monotonic() < deadline)
                    if lease is not None else nullcontext())
                with admission:
                    return GraspSelectionNode._execute_locked(self, goal_handle, deadline)
            finally:
                lock.release()
        except Exception as error:
            result = SelectReachableGrasp.Result()
            result.selected_candidate_index = -1
            code = (result.ERROR_CANCELED if isinstance(error, InterruptedError) else
                    self._infrastructure_error_code(result, str(error)))
            self._set_failure(result, code, str(error))
            with self._goal_lock:
                self._goal_active = False
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

    def _execute_locked(self, goal_handle, deadline=None):
        result = SelectReachableGrasp.Result()
        result.selected_candidate_index = -1
        deadline = deadline if deadline is not None else time.monotonic() + float(
            self.get_parameter('action_timeout_sec').value
        )
        candidate_messages = list(goal_handle.request.candidates)
        scene_started = False
        progress_timer = None
        progress_active = threading.Event()
        terminal_state = 'abort'
        try:
            self._wait_for_description(goal_handle, deadline)
            self._sensor_scene.wait(canceled=lambda: goal_handle.is_cancel_requested or time.monotonic() >= deadline)
            try:
                state = self._current_joint_state()
            except TimeoutError as error:
                self._set_failure(
                    result,
                    result.ERROR_JOINT_STATE_STALE,
                    str(error),
                )
            except ValueError as error:
                self._set_failure(
                    result,
                    result.ERROR_JOINT_STATE_INCOMPLETE,
                    str(error),
                )
            else:
                if self._description_error:
                    raise InfrastructureError(self._description_error)
                self._adapter.set_current_state(state)
                if self._visibility_tf is not None:
                    self._adapter.set_visibility_constraint(self._visibility_constraint(candidate_messages[0]))
                if hasattr(self._adapter, 'set_deadline'):
                    self._adapter.set_deadline(deadline)
                if self._scene.active:
                    self._scene.restore()
                object_id = f'grasp_target_{uuid.uuid4().hex}'
                scene_started = True
                self._scene.begin(candidate_messages[0], object_id)
                self._scene_port.reset()
                self._sensor_scene.wait(after_stamp_ns=self.get_clock().now().nanoseconds,
                    canceled=lambda: goal_handle.is_cancel_requested or time.monotonic() >= deadline)
                candidates = [
                    Candidate(
                        position=(
                            item.tcp_pose.position.x,
                            item.tcp_pose.position.y,
                            item.tcp_pose.position.z,
                        ),
                        approach_direction=(
                            item.approach_direction.x,
                            item.approach_direction.y,
                            item.approach_direction.z,
                        ),
                        score=float(item.score),
                        source_index=index,
                        orientation_xyzw=(
                            item.tcp_pose.orientation.x,
                            item.tcp_pose.orientation.y,
                            item.tcp_pose.orientation.z,
                            item.tcp_pose.orientation.w,
                        ),
                        required_opening_m=float(item.required_opening_m),
                        depth_m=float(item.grasp_depth_m),
                        approach_offset_m=approach_offset_for_label(item.target_object.label,
                            self.get_parameter('grasp_approach_offset_m').value,
                            list(self.get_parameter('deeper_grasp_labels').value),
                            list(self.get_parameter('deeper_grasp_offsets_m').value)),
                        lateral_offset_m=(aperture_centering_offset(float(item.required_opening_m),
                            self.get_parameter('grasp_aperture_margin_m').value,
                            self.get_parameter('grasp_fixed_jaw_inner_x_m').value,
                            self.get_parameter('grasp_fixed_jaw_clearance_m').value)
                            if self.get_parameter('grasp_use_aperture_centering').value else
                            self.get_parameter('grasp_lateral_offset_m').value if not
                            self.get_parameter('asymmetric_gripper_enabled').value else None),
                    )
                    for index, item in enumerate(candidate_messages)
                ]

                def canceled() -> bool:
                    cancel = (
                        goal_handle.is_cancel_requested
                        or time.monotonic() >= deadline
                    )
                    if cancel:
                        self._adapter.cancel_active()
                    return cancel

                latest_stage = [-1, goal_handle.request.required_arm, EvaluationStage.PREGRASP_IK]

                def feedback(index, arm, stage, message, *, log=True) -> None:
                    latest_stage[:] = [index, arm, stage]
                    update = SelectReachableGrasp.Feedback()
                    update.candidate_index = index
                    update.arm = arm
                    update.stage = STAGE_CONSTANT[stage]
                    update.message = message
                    goal_handle.publish_feedback(update)
                    if log:
                        self.get_logger().info(
                            f'candidate={index} arm={arm} '
                            f'stage={stage.value}: {message}')

                if hasattr(self._adapter, 'set_progress_callback'):
                    progress_active.set()

                    def phase_feedback(message):
                        if progress_active.is_set():
                            feedback(*latest_stage, message)

                    def phase_heartbeat():
                        if progress_active.is_set():
                            message = self._adapter.computation_progress()
                            if message:
                                feedback(*latest_stage, message, log=False)

                    self._adapter.set_progress_callback(phase_feedback)
                    progress_timer = self.create_timer(2., phase_heartbeat,
                        callback_group=ReentrantCallbackGroup())

                if hasattr(self._adapter, 'set_ik_diagnostic_callback'):
                    selection_id = str(uuid.uuid4())
                    self._adapter.set_ik_diagnostic_callback(lambda diagnostic:
                        self.get_logger().info('ik_diagnostic=' + json.dumps(
                            {'selection_id': selection_id, **diagnostic}, allow_nan=False)))

                candidates = GraspSelectionNode._prepare_fixed_jaw_candidates(
                    self, candidates, candidate_messages, goal_handle.request.required_arm,
                    feedback, canceled)
                selection = self._selector.select(
                    candidates,
                    required_arm=goal_handle.request.required_arm,
                    cancel_requested=canceled,
                    feedback=feedback,
                )
                statistics = getattr(self._adapter, 'last_ik_statistics', None)
                if statistics is not None:
                    self.get_logger().info('candidate_ik_statistics=' + json.dumps(statistics))
                if selection is None:
                    self._set_failure(
                        result,
                        result.ERROR_NO_REACHABLE_GRASP,
                        ('No candidate-arm pair passed IK, validity, and both plans'
                         if getattr(self._selector, 'pregrasp_motion_enabled', True) else
                         'No candidate-arm pair passed grasp IK, validity, and current-to-grasp plan'),
                    )
                else:
                    result.success = True
                    result.error_code = result.ERROR_NONE
                    self._sensor_scene.check()
                    GraspSelectionNode._remember_warmup_arm(self, selection.arm)
                    result.message = 'Selected reachable grasp (plan-only)'
                    if selection.pregrasp is None:
                        result.message = 'Selected direct grasp (plan-only); pregrasp selection skipped'
                    result.selected_candidate_index = selection.candidate_index
                    result.selected_arm = selection.arm
                    result.selected_candidate = candidate_messages[
                        selection.candidate_index
                    ]
                    result.pregrasp_joint_state = self._joint_message(
                        selection.pregrasp
                    )
                    result.grasp_joint_state = self._joint_message(
                        selection.grasp
                    )
                    result.planning_start_state = state
                    result.pregrasp_trajectory = path_to_message(selection.pregrasp_path)
                    result.grasp_trajectory = path_to_message(selection.grasp_path)
                    self._set_metrics(result, selection)
                    terminal_state = 'succeed'
        except InterruptedError as error:
            self._set_failure(
                result,
                result.ERROR_CANCELED,
                f'Grasp selection canceled or timed out: {error}',
            )
            # ROS only permits CANCELED after an accepted cancellation request.
            terminal_state = 'canceled' if goal_handle.is_cancel_requested else 'abort'
        except InfrastructureError as error:
            message = str(error)
            code = self._infrastructure_error_code(result, message)
            self._set_failure(result, code, message)
        except (ValueError, TypeError) as error:
            self._set_failure(result, result.ERROR_INVALID_INPUT, str(error))
        except Exception as error:
            self._set_failure(
                result,
                result.ERROR_INTERNAL,
                f'Unexpected error: {error}',
            )
        finally:
            progress_active.clear()
            if progress_timer is not None:
                self.destroy_timer(progress_timer)
            if hasattr(self._adapter, 'set_progress_callback'):
                self._adapter.set_progress_callback(None)
                self.get_logger().info('planning_phase_timings=' + json.dumps(
                    self._adapter.computation_timings()))
            if hasattr(self._adapter, 'set_ik_diagnostic_callback'):
                self._adapter.set_ik_diagnostic_callback(None)
            self._adapter.set_visibility_constraint(None)
            if scene_started:
                try:
                    self._scene.restore()
                except Exception as error:
                    self.get_logger().error(
                        f'Failed to restore planning scene: {error}'
                    )
                    self._set_failure(
                        result,
                        result.ERROR_PLANNING_SCENE,
                        f'Failed to restore planning scene: {error}; prior result: {result.message}',
                    )
                    terminal_state = 'abort'
            with self._goal_lock:
                self._goal_active = False

        if terminal_state == 'succeed':
            goal_handle.succeed()
        elif terminal_state == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    @staticmethod
    def _joint_message(solution) -> JointState:
        message = JointState()
        if solution is None:
            return message
        message.name = list(solution.names)
        message.position = list(solution.positions)
        return message

    @staticmethod
    def _pose_message(value) -> Pose:
        message = Pose()
        if value is None:
            return message
        (
            message.position.x,
            message.position.y,
            message.position.z,
        ) = value.position
        (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ) = value.orientation_xyzw
        return message

    def _set_metrics(self, result, selection) -> None:
        pregrasp = selection.pregrasp_alignment
        grasp = selection.grasp_alignment
        trajectory = selection.trajectory_alignment
        result.gripper_joint_position = selection.gripper_joint_position
        result.planner_id = self._adapter.planner_id
        result.latency_stages = [name for name, _ in selection.latencies]
        result.latency_seconds = [value for _, value in selection.latencies]
        if pregrasp is not None:
            result.fk_pregrasp_pose = self._pose_message(pregrasp.pose)
            result.pregrasp_position_error_m = pregrasp.position_error_m
            result.pregrasp_approach_error_degrees = (
                pregrasp.approach_error_degrees
            )
            result.pregrasp_closing_error_degrees = pregrasp.closing_error_degrees
        if grasp is not None:
            result.fk_grasp_pose = self._pose_message(grasp.pose)
            result.grasp_position_error_m = grasp.position_error_m
            result.grasp_approach_error_degrees = grasp.approach_error_degrees
            result.grasp_closing_error_degrees = grasp.closing_error_degrees
        if trajectory is not None:
            result.trajectory_max_approach_error_degrees = (
                trajectory.maximum_approach_error_degrees
            )
            result.trajectory_max_closing_error_degrees = (
                trajectory.maximum_closing_error_degrees
            )

    @staticmethod
    def _set_failure(result, code, message) -> None:
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

    @staticmethod
    def _infrastructure_error_code(result, message: str) -> int:
        planning_scene_terms = (
            'planning-scene',
            'planning scene',
            'target OBB',
            'collision permissions',
        )
        if any(term in message for term in planning_scene_terms):
            return result.ERROR_PLANNING_SCENE
        return result.ERROR_MOVEIT_UNAVAILABLE


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspSelectionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        close = getattr(node._adapter, 'close', None)
        if close is not None:
            close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
