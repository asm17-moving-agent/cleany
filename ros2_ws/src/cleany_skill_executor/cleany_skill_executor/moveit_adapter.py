"""MoveIt service/action adapter for plan-only grasp reachability checks."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from cleany_skill_executor.core.grasp_selection import (
    ARM_JOINT_NAMES,
    InfrastructureError,
    JointSolution,
    REQUIRED_JOINT_NAMES,
    directed_axis_error_deg,
    quaternion_axis,
    unsigned_axis_error_deg,
)


# Canonical limits from cleany_description/urdf/dual_arm.xacro.  The wrist
# roll entries are replaced by the configured bounds when seeds are built.
_ARM_JOINT_LIMITS = (
    (-2.16, 2.16),
    (-0.22, 3.37),
    (-0.22, 3.14),
    (-1.65806284946, 1.65806272933),
    (-2.743847297, 2.84120630938),
)

# Low-discrepancy, deterministic coverage of all five joints.  The first two
# attempts remain the candidate-specific position seed and current state.
_DISTRIBUTED_SEED_FRACTIONS = (
    (0.5000, 0.4533, 0.7102, 0.2008, 0.3975),
    (0.1394, 0.4803, 0.4708, 0.9252, 0.1901),
    (0.50, 0.50, 0.50, 0.50, 0.50),
    (0.375, 0.625, 0.875, 0.125, 0.625),
    (0.625, 0.375, 0.625, 0.875, 0.375),
    (0.125, 0.875, 0.375, 0.625, 0.875),
    (0.875, 0.125, 0.750, 0.375, 0.125),
    (0.250, 0.750, 0.125, 0.750, 0.250),
    (0.750, 0.250, 0.250, 0.250, 0.750),
    (0.0625, 0.875, 0.640, 0.163, 0.727),
)


@dataclass(frozen=True, slots=True)
class MoveItAdapterConfig:
    base_frame: str = 'base_link'
    ik_timeout_sec: float = 0.15
    pregrasp_aim_ik_timeout_sec: float = 1.0
    state_validity_timeout_sec: float = 1.0
    fk_timeout_sec: float = 1.0
    pregrasp_position_tolerance_m: float = 0.005
    pregrasp_preferred_approach_tolerance_deg: float = 5.0
    pregrasp_approach_tolerance_deg: float = 15.0
    pregrasp_closing_tolerance_deg: float = 30.0
    grasp_closing_tolerance_deg: float = 30.0
    grasp_closing_sign_invariant: bool = True
    pregrasp_aim_attempts: int = 8
    wrist_roll_lower_rad: float = -2.743847297
    wrist_roll_upper_rad: float = 2.84120630938
    planning_timeout_sec: float = 4.0
    planning_attempts: int = 3
    velocity_scaling: float = 0.08
    acceleration_scaling: float = 0.08
    poll_interval_sec: float = 0.01

    def __post_init__(self) -> None:
        positive = (
            self.ik_timeout_sec,
            self.pregrasp_aim_ik_timeout_sec,
            self.state_validity_timeout_sec,
            self.fk_timeout_sec,
            self.pregrasp_position_tolerance_m,
            self.pregrasp_preferred_approach_tolerance_deg,
            self.pregrasp_approach_tolerance_deg,
            self.pregrasp_closing_tolerance_deg,
            self.grasp_closing_tolerance_deg,
            self.planning_timeout_sec,
            self.velocity_scaling,
            self.acceleration_scaling,
            self.poll_interval_sec,
        )
        if not self.base_frame or not all(
            math.isfinite(value) and value > 0.0 for value in positive
        ):
            raise ValueError('MoveIt adapter limits must be finite and positive')
        if self.planning_attempts <= 0 or self.pregrasp_aim_attempts <= 0:
            raise ValueError('MoveIt attempt counts must be positive')
        if (
            not math.isfinite(self.wrist_roll_lower_rad)
            or not math.isfinite(self.wrist_roll_upper_rad)
            or self.wrist_roll_lower_rad >= self.wrist_roll_upper_rad
        ):
            raise ValueError('wrist roll limits are inconsistent')
        if (
            self.pregrasp_preferred_approach_tolerance_deg
            > self.pregrasp_approach_tolerance_deg
            or self.pregrasp_approach_tolerance_deg > 180.0
            or self.pregrasp_closing_tolerance_deg > 90.0
            or self.grasp_closing_tolerance_deg > 90.0
        ):
            raise ValueError('pregrasp approach tolerances are inconsistent')


class MoveItGraspAdapter:
    """Preserve a complete feedback state in IK, validity, and plan requests."""

    def __init__(
        self,
        node: Any,
        config: MoveItAdapterConfig = MoveItAdapterConfig(),
        *,
        ik_client: Any | None = None,
        fk_client: Any | None = None,
        validity_client: Any | None = None,
        plan_client: Any | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        spin_once: Callable[[float], None] | None = None,
    ) -> None:
        self._node = node
        self._config = config
        self._ik_client = ik_client or node.create_client(GetPositionIK, '/compute_ik')
        self._fk_client = fk_client or node.create_client(GetPositionFK, '/compute_fk')
        self._validity_client = validity_client or node.create_client(
            GetStateValidity, '/check_state_validity'
        )
        self._plan_client = plan_client or ActionClient(node, MoveGroup, '/move_action')
        self._monotonic = monotonic
        self._spin_once = spin_once or self._default_spin
        self._current_state: JointState | None = None
        self._active_goal_handle: Any | None = None

    def _default_spin(self, timeout: float) -> None:
        import rclpy

        rclpy.spin_once(self._node, timeout_sec=timeout)

    def set_current_state(self, state: JointState) -> None:
        positions = dict(zip(state.name, state.position, strict=True))
        missing = set(REQUIRED_JOINT_NAMES) - positions.keys()
        if missing:
            raise ValueError(f'incomplete joint state: {sorted(missing)}')
        canonical = JointState()
        canonical.header = state.header
        canonical.name = list(REQUIRED_JOINT_NAMES)
        canonical.position = [float(positions[name]) for name in canonical.name]
        if state.velocity and len(state.velocity) == len(state.name):
            velocities = dict(zip(state.name, state.velocity, strict=True))
            canonical.velocity = [float(velocities[name]) for name in canonical.name]
        self._current_state = canonical

    def _merged_state(self, solution: JointSolution | None) -> RobotState:
        if self._current_state is None:
            raise InfrastructureError('current joint state is unavailable')
        positions = dict(zip(self._current_state.name, self._current_state.position, strict=True))
        if solution is not None:
            positions.update(zip(solution.names, solution.positions, strict=True))
        result = RobotState()
        result.is_diff = False
        result.joint_state.header = self._current_state.header
        result.joint_state.name = list(REQUIRED_JOINT_NAMES)
        result.joint_state.position = [positions[name] for name in REQUIRED_JOINT_NAMES]
        if self._current_state.velocity:
            velocities = dict(
                zip(self._current_state.name, self._current_state.velocity, strict=True)
            )
            result.joint_state.velocity = [velocities[name] for name in REQUIRED_JOINT_NAMES]
        return result

    def _wait_service(self, client: Any, timeout: float) -> None:
        deadline = self._monotonic() + timeout
        while not client.service_is_ready():
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                raise InfrastructureError('MoveIt service unavailable')
            self._spin_once(min(self._config.poll_interval_sec, remaining))

    def _call(self, client: Any, request: Any, timeout: float) -> Any:
        self._wait_service(client, timeout)
        future = client.call_async(request)
        deadline = self._monotonic() + timeout
        while not future.done():
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                future.cancel()
                raise InfrastructureError('MoveIt service timed out')
            self._spin_once(min(self._config.poll_interval_sec, remaining))
        try:
            response = future.result()
        except Exception as error:
            raise InfrastructureError(f'MoveIt service failed: {error}') from error
        if response is None:
            raise InfrastructureError('MoveIt service returned no response')
        return response

    def solve_position_ik(
        self,
        arm: str,
        position: tuple[float, float, float],
        seed: JointSolution | None,
    ) -> JointSolution | None:
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = f'{arm}_grasp_arm'
        ik.ik_link_name = f'{arm}_grasp_tcp'
        ik.avoid_collisions = True
        ik.robot_state = self._merged_state(seed)
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = self._config.base_frame
        ik.pose_stamped.pose.position.x, ik.pose_stamped.pose.position.y, ik.pose_stamped.pose.position.z = position
        ik.pose_stamped.pose.orientation.w = 1.0
        seconds = int(self._config.ik_timeout_sec)
        ik.timeout.sec = seconds
        ik.timeout.nanosec = int((self._config.ik_timeout_sec - seconds) * 1e9)
        # MoveIt may legitimately consume the full solver timeout before
        # returning NO_IK_SOLUTION. Keep transport overhead outside that
        # algorithm budget so an unreachable pair remains a fallback result,
        # not an infrastructure timeout.
        response = self._call(
            self._ik_client,
            request,
            self._config.ik_timeout_sec + 1.0,
        )
        if response.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
            return None
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            if response.error_code.val in (
                MoveItErrorCodes.INVALID_ROBOT_STATE,
                MoveItErrorCodes.INVALID_GROUP_NAME,
                MoveItErrorCodes.INVALID_LINK_NAME,
            ):
                raise InfrastructureError(f'IK configuration error: {response.error_code.val}')
            return None
        positions = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
                strict=True,
            )
        )
        names = ARM_JOINT_NAMES[arm]
        if not set(names) <= positions.keys():
            raise InfrastructureError('IK response omits arm joints')
        return JointSolution(names, tuple(float(positions[name]) for name in names))

    def solve_aimed_pregrasp_ik(
        self,
        arm: str,
        grasp_position: tuple[float, float, float],
        approach_direction: tuple[float, float, float],
        closing_direction: tuple[float, float, float],
        pregrasp_position: tuple[float, float, float],
        seed: JointSolution | None,
    ) -> tuple[JointSolution, ...]:
        """Return pose-compatible pregrasp solutions in increasing error order."""
        valid: list[
            tuple[float, float, float, JointSolution, float, float]
        ] = []
        evaluated: list[
            tuple[float, float, float, JointSolution, float, float]
        ] = []
        if seed is None:
            seed = self._current_arm_solution(arm)
        attempts = self._config.pregrasp_aim_attempts
        for current_seed in self._aim_seed_solutions(
            arm,
            seed,
            attempts,
            target_position=grasp_position,
        ):
            solution = self._solve_aim_tip_position_ik(
                arm, grasp_position, current_seed
            )
            if solution is None:
                continue
            tcp_error, aim_error, distance_error, angle_deg, closing_error_deg = (
                self._pregrasp_direction_errors(
                    arm,
                    solution,
                    grasp_position,
                    approach_direction,
                    closing_direction,
                    pregrasp_position,
                )
            )
            evaluated.append((
                angle_deg,
                closing_error_deg,
                tcp_error,
                solution,
                aim_error,
                distance_error,
            ))
            if (
                tcp_error <= self._config.pregrasp_position_tolerance_m
                and aim_error <= self._config.pregrasp_position_tolerance_m
                and distance_error <= self._config.pregrasp_position_tolerance_m
                and closing_error_deg <= self._config.pregrasp_closing_tolerance_deg
                and angle_deg <= self._config.pregrasp_approach_tolerance_deg
            ):
                valid.append((
                    angle_deg,
                    closing_error_deg,
                    tcp_error,
                    solution,
                    aim_error,
                    distance_error,
                ))
        ranked = self._unique_ranked_solutions(valid)
        if ranked:
            self._log_pose_result(
                'Accepted pregrasp', min(valid, key=lambda item: item[:3])
            )
        elif evaluated:
            self._log_pose_result(
                'Rejected closest pregrasp',
                min(evaluated, key=lambda item: item[:3]),
            )
        return ranked

    def _aim_seed_solutions(
        self,
        arm: str,
        candidate_seed: JointSolution,
        attempts: int,
        *,
        target_position: tuple[float, float, float] | None = None,
    ) -> tuple[JointSolution, ...]:
        limits = (
            *_ARM_JOINT_LIMITS[:-1],
            (
                self._config.wrist_roll_lower_rad,
                self._config.wrist_roll_upper_rad,
            ),
        )
        unique: list[JointSolution] = []
        seen: set[tuple[int, ...]] = set()

        def add_seed(item: JointSolution) -> None:
            key = tuple(round(value * 1e6) for value in item.positions)
            if key not in seen:
                seen.add(key)
                unique.append(item)

        # The position-only seed already reflects this object's location.
        # Keep its shoulder/elbow solution and vary wrist roll before using
        # target-independent whole-joint samples.
        for location_seed in self._location_biased_seeds(
            candidate_seed,
            limits,
            target_position,
        ):
            add_seed(location_seed)
            if len(unique) >= attempts:
                return tuple(unique[:attempts])
        add_seed(self._current_arm_solution(arm))
        for fractions in _DISTRIBUTED_SEED_FRACTIONS:
            positions = tuple(
                lower + fraction * (upper - lower)
                for fraction, (lower, upper) in zip(
                    fractions, limits, strict=True
                )
            )
            add_seed(JointSolution(ARM_JOINT_NAMES[arm], positions))
            if len(unique) >= attempts:
                return tuple(unique[:attempts])
        sequence_index = 1
        while len(unique) < attempts:
            fractions = tuple(
                self._radical_inverse(sequence_index, base)
                for base in (2, 3, 5, 7, 11)
            )
            positions = tuple(
                lower + fraction * (upper - lower)
                for fraction, (lower, upper) in zip(
                    fractions, limits, strict=True
                )
            )
            add_seed(JointSolution(ARM_JOINT_NAMES[arm], positions))
            sequence_index += 1
        return tuple(unique)

    @staticmethod
    def _location_biased_seeds(
        candidate_seed: JointSolution,
        limits: tuple[tuple[float, float], ...],
        target_position: tuple[float, float, float] | None,
    ) -> tuple[JointSolution, ...]:
        if target_position is None:
            return (candidate_seed,)
        lower, upper = limits[-1]
        midpoint = (lower + upper) / 2.0
        quarter_range = (upper - lower) / 4.0
        lateral = target_position[1]
        side = 1.0 if lateral >= 0.0 else -1.0
        rolls = (
            float(candidate_seed.positions[-1]),
            midpoint + side * quarter_range,
        )
        return tuple(
            MoveItGraspAdapter._with_wrist_roll(candidate_seed, roll)
            for roll in rolls
        )

    @staticmethod
    def _radical_inverse(index: int, base: int) -> float:
        result = 0.0
        denominator = 1.0
        while index:
            index, remainder = divmod(index, base)
            denominator *= base
            result += remainder / denominator
        return result

    def _current_arm_solution(self, arm: str) -> JointSolution:
        if self._current_state is None:
            raise InfrastructureError('current joint state is unavailable')
        positions = dict(
            zip(
                self._current_state.name,
                self._current_state.position,
                strict=True,
            )
        )
        names = ARM_JOINT_NAMES[arm]
        return JointSolution(
            names,
            tuple(float(positions[name]) for name in names),
        )

    def solve_grasp_ik(
        self,
        arm: str,
        grasp_position: tuple[float, float, float],
        approach_direction: tuple[float, float, float],
        closing_direction: tuple[float, float, float],
        seed: JointSolution,
    ) -> tuple[JointSolution, ...]:
        """Project the desired grasp pose onto feasible 5-axis joint states."""
        valid: list[
            tuple[float, float, float, JointSolution, float, float]
        ] = []
        evaluated: list[
            tuple[float, float, float, JointSolution, float, float]
        ] = []
        for roll in self._wrist_roll_seeds(
            seed, self._config.pregrasp_aim_attempts
        ):
            trial_seed = self._with_wrist_roll(seed, roll)
            solution = self.solve_position_ik(arm, grasp_position, trial_seed)
            if solution is None:
                continue
            position_error, approach_error, closing_error = self._grasp_pose_errors(
                arm,
                solution,
                grasp_position,
                approach_direction,
                closing_direction,
            )
            evaluated.append((
                approach_error,
                closing_error,
                position_error,
                solution,
                0.0,
                0.0,
            ))
            if (
                position_error <= self._config.pregrasp_position_tolerance_m
                and approach_error <= self._config.pregrasp_approach_tolerance_deg
                and closing_error <= self._config.grasp_closing_tolerance_deg
            ):
                valid.append((
                    approach_error,
                    closing_error,
                    position_error,
                    solution,
                    0.0,
                    0.0,
                ))
        ranked = self._unique_ranked_solutions(valid)
        if ranked:
            self._log_pose_result(
                'Accepted grasp', min(valid, key=lambda item: item[:3])
            )
        elif evaluated:
            self._log_pose_result(
                'Rejected closest grasp',
                min(evaluated, key=lambda item: item[:3]),
            )
        return ranked

    def _wrist_roll_seeds(
        self, seed: JointSolution, attempts: int
    ) -> tuple[float, ...]:
        original = float(seed.positions[-1])
        if attempts == 1:
            return (original,)
        lower = self._config.wrist_roll_lower_rad
        upper = self._config.wrist_roll_upper_rad
        distributed = tuple(
            lower + index * (upper - lower) / max(1, attempts - 2)
            for index in range(attempts - 1)
        )
        return (min(upper, max(lower, original)), *distributed)

    @staticmethod
    def _with_wrist_roll(seed: JointSolution, roll: float) -> JointSolution:
        positions = (*seed.positions[:-1], float(roll))
        return JointSolution(seed.names, positions)

    @staticmethod
    def _unique_ranked_solutions(
        results: list[tuple[float, float, float, JointSolution, float, float]],
    ) -> tuple[JointSolution, ...]:
        unique: list[JointSolution] = []
        seen: set[tuple[int, ...]] = set()
        for _, _, _, solution, _, _ in sorted(
            results, key=lambda item: (item[0], item[1], item[2])
        ):
            key = tuple(round(value * 1e6) for value in solution.positions)
            if key in seen:
                continue
            seen.add(key)
            unique.append(solution)
        return tuple(unique)

    def _log_pose_result(
        self,
        disposition: str,
        result: tuple[float, float, float, JointSolution, float, float],
    ) -> None:
        if not hasattr(self._node, 'get_logger'):
            return
        (
            angle_deg,
            closing_error_deg,
            tcp_error,
            _,
            aim_error,
            distance_error,
        ) = result
        self._node.get_logger().info(
            f'{disposition}: '
            f'approach_error={angle_deg:.2f}deg '
            f'tcp_error={tcp_error:.4f}m aim_error={aim_error:.4f}m '
            f'distance_error={distance_error:.4f}m '
            f'closing_error={closing_error_deg:.2f}deg'
        )

    def _solve_aim_tip_position_ik(
        self,
        arm: str,
        grasp_position: tuple[float, float, float],
        seed: JointSolution,
    ) -> JointSolution | None:
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = f'{arm}_pregrasp_aim_arm'
        ik.ik_link_name = f'{arm}_pregrasp_aim_tip'
        ik.avoid_collisions = True
        ik.robot_state = self._merged_state(seed)
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = self._config.base_frame
        (
            ik.pose_stamped.pose.position.x,
            ik.pose_stamped.pose.position.y,
            ik.pose_stamped.pose.position.z,
        ) = grasp_position
        ik.pose_stamped.pose.orientation.w = 1.0
        timeout = self._config.pregrasp_aim_ik_timeout_sec
        seconds = int(timeout)
        ik.timeout.sec = seconds
        ik.timeout.nanosec = int((timeout - seconds) * 1e9)
        response = self._call(
            self._ik_client,
            request,
            timeout + 1.0,
        )
        if response.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
            return None
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            if response.error_code.val in (
                MoveItErrorCodes.INVALID_ROBOT_STATE,
                MoveItErrorCodes.INVALID_GROUP_NAME,
                MoveItErrorCodes.INVALID_LINK_NAME,
            ):
                raise InfrastructureError(
                    f'direction-aware IK configuration error: '
                    f'{response.error_code.val}'
                )
            return None
        positions = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
                strict=True,
            )
        )
        names = ARM_JOINT_NAMES[arm]
        if not set(names) <= positions.keys():
            raise InfrastructureError(
                'direction-aware IK response omits arm joints'
            )
        return JointSolution(
            names,
            tuple(float(positions[name]) for name in names),
        )

    def _pregrasp_direction_errors(
        self,
        arm: str,
        solution: JointSolution,
        grasp_position: tuple[float, float, float],
        approach_direction: tuple[float, float, float],
        closing_direction: tuple[float, float, float],
        pregrasp_position: tuple[float, float, float],
    ) -> tuple[float, float, float, float, float]:
        request = GetPositionFK.Request()
        request.header.frame_id = self._config.base_frame
        request.fk_link_names = [
            f'{arm}_grasp_tcp',
            f'{arm}_pregrasp_aim_tip',
        ]
        request.robot_state = self._merged_state(solution)
        response = self._call(
            self._fk_client,
            request,
            self._config.fk_timeout_sec,
        )
        if (
            response.error_code.val != MoveItErrorCodes.SUCCESS
            or len(response.pose_stamped) != 2
        ):
            raise InfrastructureError('MoveIt could not verify pregrasp direction')
        tcp = response.pose_stamped[0].pose.position
        aim = response.pose_stamped[1].pose.position
        tcp_position = (float(tcp.x), float(tcp.y), float(tcp.z))
        aim_position = (float(aim.x), float(aim.y), float(aim.z))

        tcp_error = math.sqrt(
            sum(
                (actual - expected) ** 2
                for actual, expected in zip(
                    tcp_position, pregrasp_position, strict=True
                )
            )
        )
        aim_error = math.sqrt(
            sum(
                (actual - expected) ** 2
                for actual, expected in zip(
                    aim_position, grasp_position, strict=True
                )
            )
        )
        actual = tuple(
            aim_value - tcp_value
            for aim_value, tcp_value in zip(
                aim_position, tcp_position, strict=True
            )
        )
        actual_norm = math.sqrt(sum(value * value for value in actual))
        expected_norm = math.sqrt(
            sum(value * value for value in approach_direction)
        )
        expected_distance = math.sqrt(
            sum(
                (grasp - pregrasp) ** 2
                for grasp, pregrasp in zip(
                    grasp_position, pregrasp_position, strict=True
                )
            )
        )
        if actual_norm <= 1.0e-9 or expected_norm <= 1.0e-9:
            return tcp_error, aim_error, math.inf, 180.0, 90.0
        distance_error = abs(actual_norm - expected_distance)
        cosine = sum(
            actual_value / actual_norm * expected_value / expected_norm
            for actual_value, expected_value in zip(
                actual, approach_direction, strict=True
            )
        )
        angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
        orientation = response.pose_stamped[0].pose.orientation
        actual_closing = quaternion_axis(
            (orientation.x, orientation.y, orientation.z, orientation.w),
            (1.0, 0.0, 0.0),
        )
        closing_error_deg = unsigned_axis_error_deg(actual_closing, closing_direction)
        return tcp_error, aim_error, distance_error, angle_deg, closing_error_deg

    def _grasp_pose_errors(
        self,
        arm: str,
        solution: JointSolution,
        grasp_position: tuple[float, float, float],
        approach_direction: tuple[float, float, float],
        closing_direction: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        request = GetPositionFK.Request()
        request.header.frame_id = self._config.base_frame
        request.fk_link_names = [f'{arm}_grasp_tcp']
        request.robot_state = self._merged_state(solution)
        response = self._call(
            self._fk_client,
            request,
            self._config.fk_timeout_sec,
        )
        if (
            response.error_code.val != MoveItErrorCodes.SUCCESS
            or len(response.pose_stamped) != 1
        ):
            raise InfrastructureError('MoveIt could not verify grasp pose')
        pose = response.pose_stamped[0].pose
        actual_position = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        position_error = math.sqrt(
            sum(
                (actual - expected) ** 2
                for actual, expected in zip(
                    actual_position, grasp_position, strict=True
                )
            )
        )
        quaternion = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        actual_approach = quaternion_axis(quaternion, (0.0, -1.0, 0.0))
        actual_closing = quaternion_axis(quaternion, (1.0, 0.0, 0.0))
        approach_error = directed_axis_error_deg(
            actual_approach, approach_direction
        )
        if self._config.grasp_closing_sign_invariant:
            closing_error = unsigned_axis_error_deg(
                actual_closing, closing_direction
            )
        else:
            closing_error = directed_axis_error_deg(
                actual_closing, closing_direction
            )
        return position_error, approach_error, closing_error

    def state_is_valid(self, arm: str, solution: JointSolution) -> bool:
        request = GetStateValidity.Request()
        request.group_name = f'{arm}_grasp_arm'
        request.robot_state = self._merged_state(solution)
        response = self._call(
            self._validity_client, request, self._config.state_validity_timeout_sec
        )
        return bool(response.valid)

    def plan(
        self,
        arm: str,
        goal: JointSolution,
        start: JointSolution | None,
    ) -> bool:
        wait_timeout = self._config.planning_timeout_sec
        if not self._plan_client.wait_for_server(timeout_sec=wait_timeout):
            raise InfrastructureError('MoveGroup action unavailable')
        action_goal = MoveGroup.Goal()
        action_goal.planning_options.plan_only = True
        action_goal.planning_options.look_around = False
        action_goal.planning_options.replan = False
        action_goal.planning_options.planning_scene_diff.is_diff = True
        action_goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        request = action_goal.request
        request.group_name = f'{arm}_grasp_arm'
        request.num_planning_attempts = self._config.planning_attempts
        request.allowed_planning_time = self._config.planning_timeout_sec
        request.max_velocity_scaling_factor = self._config.velocity_scaling
        request.max_acceleration_scaling_factor = self._config.acceleration_scaling
        request.start_state = self._merged_state(start)
        constraints = Constraints()
        constraints.name = f'{arm}_grasp_joint_goal'
        for name, position in zip(goal.names, goal.positions, strict=True):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 1e-4
            constraint.tolerance_below = 1e-4
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        request.goal_constraints = [constraints]
        future = self._plan_client.send_goal_async(action_goal)
        goal_handle = self._wait_future(future, wait_timeout)
        if goal_handle is None or not goal_handle.accepted:
            raise InfrastructureError('MoveGroup goal rejected')
        self._active_goal_handle = goal_handle
        try:
            wrapped = self._wait_future(goal_handle.get_result_async(), wait_timeout)
        finally:
            self._active_goal_handle = None
        if wrapped is None:
            raise InfrastructureError('MoveGroup action timed out')
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            raise InterruptedError('MoveGroup action canceled')
        return (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.error_code.val == MoveItErrorCodes.SUCCESS
            and bool(wrapped.result.planned_trajectory.joint_trajectory.points)
        )

    def _wait_future(self, future: Any, timeout: float) -> Any | None:
        deadline = self._monotonic() + timeout
        while not future.done():
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                future.cancel()
                return None
            self._spin_once(min(self._config.poll_interval_sec, remaining))
        return future.result()

    def cancel_active(self) -> None:
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
