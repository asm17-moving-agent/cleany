"""관측된 시작 상태에서 Cartesian 경로를 계획하고 실행·피드백을 검증합니다."""
from __future__ import annotations

from copy import deepcopy
import math
import time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, MoveItErrorCodes, OrientationConstraint, PositionConstraint, RobotState, RobotTrajectory
from moveit_msgs.srv import GetPositionFK
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from cleany_skill_executor.core.cartesian import (
    CartesianPose, execution_wall_timeout, line_corridor, sampled_time_scale, validate_corridor_samples, validate_pose_endpoint,
)
from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES
from cleany_skill_executor.seeded_cartesian import CartesianPlanningError


class CartesianMotion:
    """실행 순서를 소유하지 않는 ROS 작업 메서드입니다."""

    @staticmethod
    def _pose_position(pose: Pose) -> tuple[float, float, float]:
        return (pose.position.x, pose.position.y, pose.position.z)


    def _tcp_pose(
        self, arm: str, override: JointState | None = None
    ) -> Pose:
        if not self._fk.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('MoveIt FK service is unavailable')
        missing = set(REQUIRED_JOINT_NAMES) - self._joint_positions.keys()
        if missing:
            raise RuntimeError(f'joint feedback is incomplete: {sorted(missing)}')
        state = RobotState()
        state.joint_state.name = list(REQUIRED_JOINT_NAMES)
        positions = {
            name: self._joint_positions[name]
            for name in REQUIRED_JOINT_NAMES
        }
        if override is not None:
            positions.update(
                zip(override.name, override.position, strict=True)
            )
        state.joint_state.position = [positions[name] for name in REQUIRED_JOINT_NAMES]
        request = GetPositionFK.Request()
        request.header.frame_id = 'base_link'
        request.fk_link_names = [f'{arm}_grasp_tcp']
        request.robot_state = state
        response = self._future(
            self._fk.call_async(request),
            float(self.get_parameter('tcp_fk_timeout_sec').value), f'{arm} TCP FK'
        )
        if (
            response.error_code.val != MoveItErrorCodes.SUCCESS
            or len(response.pose_stamped) != 1
        ):
            raise RuntimeError(f'MoveIt could not compute {arm} TCP pose')
        return response.pose_stamped[0].pose


    def _linear_goal(
        self,
        arm: str,
        target: Pose,
        label: str,
        velocity_scaling: float,
    ):
        goal = self._execution_goal(
            arm, self._arm_joint_state(arm), label
        )
        request = goal.request
        request.pipeline_id = str(
            self.get_parameter('pilz_pipeline_id').value
        )
        request.planner_id = str(
            self.get_parameter('pilz_planner_id').value
        )
        request.num_planning_attempts = 1
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = float(
            self.get_parameter('lin_acceleration_scaling').value
        )
        request.goal_constraints = [
            self._pose_constraint(arm, target, label)
        ]
        request.start_state.is_diff = True
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        return goal


    def _arm_joint_state(self, arm: str):
        state = JointState()
        state.name = [
            name
            for name in REQUIRED_JOINT_NAMES
            if name.startswith(f'{arm}_') and 'gripper' not in name
        ]
        state.position = [self._joint_positions[name] for name in state.name]
        return state


    def _joint_corridor_goal(self, arm: str, joints: JointState, start: Pose,
                             target: Pose, label: str, velocity_scaling: float):
        goal = self._execution_goal(arm, joints, label)
        request = goal.request
        request.pipeline_id = 'ompl'
        request.planner_id = 'RRTConnectkConfigDefault'
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = float(
            self.get_parameter('lin_acceleration_scaling').value)
        request.start_state.joint_state.name = list(REQUIRED_JOINT_NAMES)
        request.start_state.joint_state.position = [self._joint_positions[n] for n in REQUIRED_JOINT_NAMES]
        request.start_state.is_diff = True  # 잡은 물체의 장면 부착물을 유지합니다.
        corridor = line_corridor(self._pose_position(start), self._pose_position(target),
                                 float(self.get_parameter('lin_position_tolerance_m').value))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = corridor.center
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = corridor.orientation
        constraint = PositionConstraint(link_name=f'{arm}_grasp_tcp', weight=1.)
        constraint.header.frame_id = 'base_link'
        constraint.constraint_region.primitives = [SolidPrimitive(
            type=SolidPrimitive.CYLINDER, dimensions=[corridor.length, corridor.radius])]
        constraint.constraint_region.primitive_poses = [pose]
        request.path_constraints.position_constraints = [constraint]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        return goal


    @staticmethod
    def _cartesian_pose(pose: Pose) -> CartesianPose:
        p, q = pose.position, pose.orientation
        return CartesianPose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


    def _validate_cartesian_plan(self, arm: str, trajectory, start: Pose,
                                 target: Pose, label: str, *, corridor: bool,
                                 precomputed_samples: list[CartesianPose] | None = None) -> list[CartesianPose]:
        points, names = trajectory.joint_trajectory.points, trajectory.joint_trajectory.joint_names
        if (len(points) < 2 or len(names) != len(set(names)) or
                set(names) != set(self._arm_joint_state(arm).name)):
            raise RuntimeError(f'{label} plan has incomplete arm trajectory')
        for point in points:
            if len(point.positions) != len(names) or not all(math.isfinite(x) for x in point.positions):
                raise RuntimeError(f'{label} plan has invalid joint positions')
        samples = precomputed_samples if precomputed_samples is not None else [self._cartesian_pose(self._tcp_pose(arm, JointState(
            name=list(names), position=list(point.positions))))
                   for point in (points if corridor else [points[-1]])]
        if precomputed_samples is not None and len(samples) != len(points):
            raise RuntimeError('seeded trajectory/FK sample count mismatch')
        position_limit = float(self.get_parameter('lin_position_tolerance_m').value)
        orientation_limit = float(self.get_parameter('lin_orientation_tolerance_rad').value)
        try:
            if corridor:
                lateral, angle = validate_corridor_samples(
                    samples, self._cartesian_pose(start), self._cartesian_pose(target),
                    position_limit, math.radians(float(self.get_parameter(
                        'corridor_orientation_tolerance_deg').value)), orientation_limit)
                self.get_logger().debug(f'{label} FK corridor verified: lateral={lateral:.6f}m '
                                       f'orientation={math.degrees(angle):.3f}deg samples={len(samples)}')
            else:
                # 위치 전용 KDL은 요청한 Pilz 방향을 무시하고도 성공할 수 있습니다.
                # 이런 끝점은 실행하지 않습니다.
                validate_pose_endpoint(samples[-1], self._cartesian_pose(target),
                                       position_limit, orientation_limit)
        except ValueError as error:
            raise CartesianPlanningError(f'{label} plan rejected before execution: {error}') from error
        return samples


    def _slow_corridor_plan(self, trajectory, samples: list[CartesianPose],
                            velocity_scaling: float, *, acceleration_scaling: float | None = None) -> None:
        points = trajectory.joint_trajectory.points
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
        factor = sampled_time_scale(
            samples, times,
            float(self.get_parameter('cartesian_translation_speed_m_s').value) * velocity_scaling,
            float(self.get_parameter('cartesian_rotation_speed_rad_s').value) * velocity_scaling,
            float(self.get_parameter('cartesian_translation_acceleration_m_s2').value) *
            (float(self.get_parameter('lin_acceleration_scaling').value)
             if acceleration_scaling is None else acceleration_scaling),
            float(self.get_parameter('corridor_time_margin').value))
        for point, stamp in zip(points, times, strict=True):
            point.time_from_start = Duration(nanoseconds=round(stamp * factor * 1e9)).to_msg()
            point.velocities = [x/factor for x in point.velocities]
            point.accelerations = [x/factor**2 for x in point.accelerations]
        self.get_logger().debug(f'Corridor uniform time scale={factor:.3f} '
                               f'duration={times[-1]*factor:.3f}s; geometry unchanged')


    def _pose_constraint(
        self, arm: str, target: Pose, label: str
    ) -> Constraints:
        tolerance = float(
            self.get_parameter('lin_position_tolerance_m').value
        )
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [tolerance]
        position = PositionConstraint()
        position.header.frame_id = 'base_link'
        position.link_name = f'{arm}_grasp_tcp'
        position.constraint_region.primitives = [region]
        position.constraint_region.primitive_poses = [deepcopy(target)]
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header.frame_id = 'base_link'
        orientation.link_name = f'{arm}_grasp_tcp'
        orientation.orientation = deepcopy(target.orientation)
        angular = float(
            self.get_parameter('lin_orientation_tolerance_rad').value
        )
        orientation.absolute_x_axis_tolerance = angular
        orientation.absolute_y_axis_tolerance = angular
        orientation.absolute_z_axis_tolerance = angular
        orientation.weight = 1.0
        constraints = Constraints()
        constraints.name = f'pilz_lin_{label}'
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        return constraints


    def _plan_linear_motion(
        self,
        arm: str,
        target: Pose,
        label: str,
        *,
        velocity_scaling: float,
        contact_target: Pose | None = None,
        joint_target: JointState | None = None,
    ) -> RobotTrajectory:
        """제어기 명령 없이 계획 및 검증하며 이 경계에서는 재시도할 수 있습니다."""
        self._wait_arm_stationary(arm)
        start = self._tcp_pose(arm)
        goal = (self._linear_goal(arm, target, label, velocity_scaling)
                if joint_target is None else self._joint_corridor_goal(
                    arm, joint_target, start, target, label, velocity_scaling))
        seeded = getattr(self, '_seeded_cartesian', None) if joint_target is not None else None
        if seeded is not None:
            goal.request.pipeline_id = 'cleany_seeded_cartesian'
            goal.request.planner_id = 'monotone_cubic'
        trace = getattr(self, '_record_pipeline_message', None)
        if trace is not None:
            trace('motion_request', goal)
        precomputed = None
        if seeded is not None:
            result = MoveGroup.Result()
            begin = time.monotonic()
            try:
                result.planned_trajectory, precomputed = seeded.plan(
                    goal.request, self._cartesian_pose(start), self._cartesian_pose(target))
                result.error_code.val = MoveItErrorCodes.SUCCESS
                result.trajectory_start = goal.request.start_state
            except Exception:
                result.error_code.val = MoveItErrorCodes.FAILURE
                raise
            finally:
                result.planning_time = time.monotonic()-begin
                if trace is not None:
                    trace('motion_result', result)
        else:
            plan_handle = self._future(
                self._move_group.send_goal_async(goal), 10.0, f'{label} Cartesian plan response')
            if not plan_handle.accepted:
                raise RuntimeError(f'{label} Cartesian plan was rejected')
            planned = self._future(
                plan_handle.get_result_async(), 60.0, f'{label} Cartesian plan'
            )
            result = planned.result
            if trace is not None:
                trace('motion_result', result)
            if (planned.status != GoalStatus.STATUS_SUCCEEDED or
                    result.error_code.val != MoveItErrorCodes.SUCCESS):
                raise CartesianPlanningError(f'{label} Cartesian planning failed: code={result.error_code.val}')
        samples = self._validate_cartesian_plan(
            arm, result.planned_trajectory, start, target,
            label, corridor=joint_target is not None, precomputed_samples=precomputed)
        if joint_target is not None:
            self._slow_corridor_plan(result.planned_trajectory, samples, velocity_scaling,
                acceleration_scaling=goal.request.max_acceleration_scaling_factor)
        if trace is not None:
            trace('motion_plan', result.planned_trajectory)
        return result.planned_trajectory


    def _execute_linear(
        self, arm: str, target: Pose, label: str, *, velocity_scaling: float,
        contact_target: Pose | None = None, joint_target: JointState | None = None,
    ) -> bool:
        trajectory = self._plan_linear_motion(
            arm, target, label, velocity_scaling=velocity_scaling,
            contact_target=contact_target, joint_target=joint_target)
        last_time = trajectory.joint_trajectory.points[-1].time_from_start
        planned_duration = last_time.sec + last_time.nanosec / 1e9
        execution_timeout = execution_wall_timeout(planned_duration,
            float(self.get_parameter('cartesian_execution_wall_timeout_factor').value),
            float(self.get_parameter('cartesian_execution_wall_timeout_margin_sec').value),
            float(self.get_parameter('cartesian_execution_wall_timeout_minimum_sec').value))
        self.get_logger().debug(f'{label} execution deadline: trajectory={planned_duration:.3f}s '
                               f'wall_timeout={execution_timeout:.3f}s')
        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = trajectory

        execute_handle = self._future(
            self._execute_trajectory.send_goal_async(execute_goal),
            10.0,
            f'{label} execution response',
        )
        if not execute_handle.accepted:
            raise RuntimeError(f'{label} trajectory execution was rejected')
        contact = self._contact_monitor() if contact_target is not None else None
        last_sample = self._controller_state_counts[arm]
        result_future = execute_handle.get_result_async()
        deadline = time.monotonic() + execution_timeout
        while not result_future.done() and time.monotonic() < deadline:
            self._spin_once(timeout_sec=0.02)

            count = self._controller_state_counts[arm]
            if contact is None or count == last_sample:
                continue
            last_sample = count
            sample = self._contact_sample(arm, contact_target)
            if sample is not None and contact.update(sample):
                self._future(
                    execute_handle.cancel_goal_async(),
                    5.0,
                    f'{label} contact cancellation',
                )
                self._future(
                    result_future, 10.0, f'{label} canceled result'
                )
                return True
        if not result_future.done():
            self._future(
                execute_handle.cancel_goal_async(), 5.0, f'{label} timeout cancel'
            )
            self._future(result_future, 10.0, f'{label} timeout terminal result')
            raise RuntimeError(f'{label} trajectory execution timed out')

        wrapped = result_future.result()
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f'{label} trajectory execution failed: '
                f'status={wrapped.status} code={wrapped.result.error_code.val}'
            )
        return False


