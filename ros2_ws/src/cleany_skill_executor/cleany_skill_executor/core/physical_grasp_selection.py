"""ROS-independent candidate/arm fallback and alignment orchestration."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import time
from typing import Callable, Protocol, Sequence

from .alignment import quaternion_axis, vector_error_degrees as directed_axis_error_deg
from .grasp_direction import GraspDirectionConfig, grasp_direction_pairs, order_grasp_directions


ARM_JOINT_NAMES = {
    side: tuple(
        f'{side}_{suffix}_joint'
        for suffix in (
            'shoulder_yaw', 'shoulder_pitch', 'elbow_pitch',
            'wrist_pitch', 'wrist_roll',
        )
    )
    for side in ('left', 'right')
}
REQUIRED_JOINT_NAMES = (
    *ARM_JOINT_NAMES['left'], 'left_gripper_joint',
    *ARM_JOINT_NAMES['right'], 'right_gripper_joint',
)


class EvaluationStage(str, Enum):
    OPENING = 'OPENING'
    PREGRASP_IK = 'PREGRASP_IK'
    GRASP_IK = 'GRASP_IK'
    POSE_ALIGNMENT = 'POSE_ALIGNMENT'
    STATE_VALIDITY = 'STATE_VALIDITY'
    PLAN_PREGRASP = 'PLAN_PREGRASP'
    PLAN_GRASP = 'PLAN_GRASP'
    TRAJECTORY_ALIGNMENT = 'TRAJECTORY_ALIGNMENT'


from .grasp_selection import InfrastructureError


@dataclass(frozen=True, slots=True)
class Candidate:
    position: tuple[float, float, float]
    approach_direction: tuple[float, float, float]
    score: float
    source_index: int = 0
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    required_opening_m: float = 0.04
    depth_m: float = 0.0
    approach_offset_m: float = 0.0
    lateral_offset_m: float | None = None

    def __post_init__(self) -> None:
        values = (
            *self.position, *self.approach_direction, self.score,
            *self.orientation_xyzw, self.required_opening_m, self.depth_m,
            self.approach_offset_m, *( () if self.lateral_offset_m is None else (self.lateral_offset_m,) ),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('candidate values must be finite')
        if math.sqrt(sum(value * value for value in self.approach_direction)) <= 1e-9:
            raise ValueError('approach_direction must be non-zero')
        quaternion_norm = math.sqrt(
            sum(value * value for value in self.orientation_xyzw)
        )
        if not math.isclose(quaternion_norm, 1.0, abs_tol=1e-4):
            raise ValueError('candidate quaternion must be normalized')
        if self.required_opening_m <= 0.0 or self.depth_m < 0.0:
            raise ValueError('candidate opening/depth is invalid')


@dataclass(frozen=True, slots=True)
class JointSolution:
    names: tuple[str, ...]
    positions: tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.names) != len(self.positions) or not self.names:
            raise ValueError('joint solution names and positions must match')
        if len(set(self.names)) != len(self.names):
            raise ValueError('joint solution names must be unique')
        if not all(math.isfinite(value) for value in self.positions):
            raise ValueError('joint positions must be finite')


@dataclass(frozen=True, slots=True)
class PoseValue:
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class PoseAlignment:
    solution: JointSolution
    pose: PoseValue
    position_error_m: float
    approach_error_degrees: float
    closing_error_degrees: float

    def within(self, config: 'GraspSelectionConfig', *, pregrasp: bool = False) -> bool:
        tolerance = (config.pregrasp_position_tolerance_m
                     if pregrasp and config.pregrasp_position_tolerance_m is not None
                     else config.position_tolerance_m)
        return (
            self.position_error_m <= tolerance
            and self.approach_error_degrees <= config.approach_tolerance_degrees
            and self.closing_error_degrees <= config.closing_tolerance_degrees
        )


@dataclass(frozen=True, slots=True)
class PlannedPath:
    joint_names: tuple[str, ...]
    points: tuple[tuple[float, ...], ...]
    times: tuple[float, ...] = ()
    velocities: tuple[tuple[float, ...], ...] = ()


@dataclass(frozen=True, slots=True)
class TrajectoryAlignment:
    maximum_approach_error_degrees: float
    maximum_closing_error_degrees: float
    sampled_waypoints: int

    def within(self, config: 'GraspSelectionConfig') -> bool:
        return (
            self.maximum_approach_error_degrees <= config.approach_tolerance_degrees
            and self.maximum_closing_error_degrees <= config.closing_tolerance_degrees
        )


@dataclass(frozen=True, slots=True)
class Selection:
    candidate_index: int
    arm: str
    candidate: Candidate
    pregrasp: JointSolution | None
    grasp: JointSolution
    gripper_joint_position: float = 0.0
    pregrasp_alignment: PoseAlignment | None = None
    grasp_alignment: PoseAlignment | None = None
    trajectory_alignment: TrajectoryAlignment | None = None
    latencies: tuple[tuple[str, float], ...] = ()
    pregrasp_path: PlannedPath | None = None
    grasp_path: PlannedPath | None = None


@dataclass(frozen=True, slots=True)
class GraspSelectionConfig:
    pregrasp_offset_m: float = 0.08
    maximum_candidates: int = 12
    maximum_gripper_width_m: float = 0.10
    gripper_safety_margin_m: float = 0.005
    gripper_closed_joint_rad: float = -0.374532977628
    gripper_open_joint_rad: float = 1.74532919957
    approach_tolerance_degrees: float = 15.0
    closing_tolerance_degrees: float = 30.0
    position_tolerance_m: float = 0.002
    trajectory_maximum_waypoints: int = 20
    require_gripper_closure_clearance: bool = False
    gripper_sweep_step_rad: float = 0.05
    require_approach_corridor: bool = False
    require_pregrasp_visibility: bool = False
    stop_after_first_feasible: bool = False
    pregrasp_motion_enabled: bool = True
    direct_grasp_shortcut_enabled: bool = False
    grasp_ik_first: bool = False
    asymmetric_gripper_enabled: bool = False
    fixed_jaw_closing_axis_sign: float = 1.0
    fixed_jaw_tcp_offset_m: float = 0.0
    fixed_jaw_additional_bias_m: float = 0.0
    pregrasp_position_tolerance_m: float | None = None
    direction_preference: GraspDirectionConfig = GraspDirectionConfig()
    pregrasp_fallback_offset_m: float = 0.0
    pregrasp_fallback_offsets_m: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        positive = (
            self.pregrasp_offset_m, self.maximum_gripper_width_m,
            self.approach_tolerance_degrees,
            self.closing_tolerance_degrees, self.position_tolerance_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('grasp selection limits must be finite and positive')
        if (not math.isfinite(self.pregrasp_fallback_offset_m)
                or not 0.0 <= self.pregrasp_fallback_offset_m < self.pregrasp_offset_m):
            raise ValueError('pregrasp fallback must be zero or shorter than the primary offset')
        previous = self.pregrasp_offset_m
        for offset in self.pregrasp_fallback_offsets_m:
            if not math.isfinite(offset) or not 0.0 < offset < previous:
                raise ValueError('pregrasp fallback offsets must be positive and strictly decreasing')
            previous = offset
        if (self.pregrasp_position_tolerance_m is not None
                and (not math.isfinite(self.pregrasp_position_tolerance_m)
                     or self.pregrasp_position_tolerance_m <= 0.0)):
            raise ValueError('pregrasp position tolerance must be finite and positive')
        if not (math.isfinite(self.gripper_safety_margin_m)
                and 0.0 <= self.gripper_safety_margin_m < self.maximum_gripper_width_m):
            raise ValueError('gripper safety margin must be finite, nonnegative, and smaller than maximum width')
        if not (
            math.isfinite(self.gripper_closed_joint_rad)
            and math.isfinite(self.gripper_open_joint_rad)
            and self.gripper_open_joint_rad > self.gripper_closed_joint_rad
        ):
            raise ValueError('gripper simulation joint range is invalid')
        if self.maximum_candidates <= 0 or self.trajectory_maximum_waypoints <= 0:
            raise ValueError('candidate and waypoint limits must be positive')
        if type(self.asymmetric_gripper_enabled) is not bool:
            raise ValueError('asymmetric gripper enable must be boolean')
        if type(self.stop_after_first_feasible) is not bool:
            raise ValueError('First feasible selection switch must be boolean')
        for name in ('pregrasp_motion_enabled', 'direct_grasp_shortcut_enabled', 'grasp_ik_first'):
            if type(getattr(self, name)) is not bool:
                raise ValueError(name+' must be boolean')
        if self.fixed_jaw_closing_axis_sign not in (-1.0, 1.0):
            raise ValueError('fixed jaw closing-axis sign must be -1 or 1')
        if (not math.isfinite(self.fixed_jaw_tcp_offset_m)
                or self.fixed_jaw_closing_axis_sign*self.fixed_jaw_tcp_offset_m < 0.0
                or abs(self.fixed_jaw_tcp_offset_m) >= self.maximum_gripper_width_m):
            raise ValueError('fixed jaw TCP offset is invalid')
        if (not math.isfinite(self.fixed_jaw_additional_bias_m)
                or not 0.0 <= self.fixed_jaw_additional_bias_m
                < self.maximum_gripper_width_m):
            raise ValueError('fixed jaw additional bias is invalid')

    @property
    def maximum_candidate_opening_m(self) -> float:
        return self.maximum_gripper_width_m - self.gripper_safety_margin_m

    def opening_to_joint_position(self, opening_m: float) -> float:
        if not math.isfinite(opening_m) or opening_m <= 0.0:
            raise ValueError('candidate opening must be finite and positive')
        # ROS float32 transport can move an exact 95 mm value by a few nm.
        if opening_m > self.maximum_candidate_opening_m + 1e-6:
            raise ValueError(
                f'candidate opening {opening_m:.3f} m exceeds safe simulation '
                f'limit {self.maximum_candidate_opening_m:.3f} m'
            )
        # An accepted float32 rounding overshoot must not command beyond open.
        ratio = min(opening_m, self.maximum_candidate_opening_m) / self.maximum_gripper_width_m
        return self.gripper_closed_joint_rad + ratio * (
            self.gripper_open_joint_rad - self.gripper_closed_joint_rad
        )

    def fixed_jaw_tcp_shift_m(self, opening_m: float) -> float:
        """Signed local +X shift that places the fixed jaw at one object face."""
        if not math.isfinite(opening_m) or opening_m <= 0.0:
            raise ValueError('candidate opening must be finite and positive')
        if not self.asymmetric_gripper_enabled:
            return 0.0
        return (
            self.fixed_jaw_closing_axis_sign*(
                opening_m/2.0+self.fixed_jaw_additional_bias_m)
            - self.fixed_jaw_tcp_offset_m
        )


class ReachabilityPort(Protocol):
    def current_tcp_pose(self, arm: str) -> PoseValue:
        """Local FK of the measured joints; no IK, scene update, or remote call."""
        ...

    def set_target_contacts(self, arm: str | None) -> None:
        ...

    def set_gripper_position(self, arm: str, position: float) -> None:
        ...

    def solve_aligned_ik(
        self, arm: str, target: PoseValue, seed: JointSolution | None,
    ) -> PoseAlignment | None:
        ...

    def state_is_valid(self, arm: str, solution: JointSolution) -> bool:
        ...

    def plan(
        self, arm: str, goal: JointSolution, start: JointSolution | None,
    ) -> PlannedPath | bool | None:
        ...

    def trajectory_alignment(
        self,
        arm: str,
        path: PlannedPath,
        orientation_xyzw: tuple[float, float, float, float],
        maximum_waypoints: int,
    ) -> TrajectoryAlignment | None:
        ...

    def wrist_motion_cost(
        self,
        arm: str,
        pregrasp: JointSolution,
        grasp: JointSolution,
    ) -> float | None:
        ...


Feedback = Callable[[int, str, EvaluationStage, str], None]


@dataclass(frozen=True, slots=True)
class PregraspRequest:
    arm: str
    target: PoseValue
    gripper_position: float
    candidate_index: int = field(default=-1, compare=False)


class GraspSelector:
    def __init__(
        self,
        port: ReachabilityPort,
        config: GraspSelectionConfig = GraspSelectionConfig(),
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._port = port
        self._config = config
        self._monotonic = monotonic

    @staticmethod
    def pregrasp_position(
        candidate: Candidate,
        offset_m: float = 0.08,
        grasp_position: tuple[float, float, float] | None = None,
    ) -> tuple[float, float, float]:
        norm = math.sqrt(sum(value * value for value in candidate.approach_direction))
        if not math.isfinite(norm) or norm <= 1e-9:
            raise ValueError('approach_direction must be non-zero')
        return tuple(
            position - direction / norm * (offset_m + (candidate.approach_offset_m if grasp_position is not None else 0.0))
            for position, direction in zip(
                candidate.position if grasp_position is None else grasp_position,
                candidate.approach_direction,
                strict=True,
            )
        )

    def grasp_tcp_position(self, candidate: Candidate) -> tuple[float, float, float]:
        """Translate a symmetric grasp center to the fixed-jaw TCP target."""
        distance = self._config.fixed_jaw_tcp_shift_m(
            candidate.required_opening_m)
        if candidate.lateral_offset_m is not None:
            distance = candidate.lateral_offset_m
        x, y, z, w = candidate.orientation_xyzw
        # First column of the normalized quaternion rotation matrix: TCP local
        # +X, which is the closing axis used by the alignment contract.
        closing_axis = (
            1.0-2.0*(y*y+z*z),
            2.0*(x*y+z*w),
            2.0*(x*z-y*w),
        )
        return tuple(
            position+distance*axis+candidate.approach_offset_m*approach
            for position, axis, approach in zip(
                candidate.position, closing_axis, candidate.approach_direction, strict=True)
        )

    @staticmethod
    def arm_order(candidate: Candidate) -> tuple[str, str]:
        return ('left', 'right') if candidate.position[1] >= 0.0 else ('right', 'left')

    def _evaluation_order(
        self, ranked: Sequence[Candidate], required_arm: str,
        check_canceled: Callable[[], None],
    ) -> list[tuple[Candidate, str]]:
        preference = self._config.direction_preference
        if not preference.enabled:
            return [(candidate, arm)
                    for candidate in ranked[:self._config.maximum_candidates]
                    for arm in ((required_arm,) if required_arm else self.arm_order(candidate))]

        groups = grasp_direction_pairs(ranked, preference)
        current_poses: dict[str, PoseValue] = {}
        attempts: list[tuple[Candidate, str]] = []
        remaining = self._config.maximum_candidates
        for group in groups:
            check_canceled()
            if remaining == 0:
                break
            count = min(len(group), remaining)
            for arm in ((required_arm,) if required_arm else self.arm_order(group[0])):
                ordered = group
                if len(group) == 2:
                    if arm not in current_poses:
                        current_poses[arm] = self._port.current_tcp_pose(arm)
                    ordered = order_grasp_directions(
                        group, current_poses[arm].orientation_xyzw, preference,
                        self._config.fixed_jaw_closing_axis_sign)
                attempts.extend((candidate, arm) for candidate in ordered[:count])
            remaining -= count
        return attempts

    @staticmethod
    def _perfect_alignment(
        solution: JointSolution,
        target: PoseValue,
    ) -> PoseAlignment:
        """Compatibility path for ports implementing the former position IK."""
        return PoseAlignment(solution, target, 0.0, 0.0, 0.0)

    def _solve(
        self,
        arm: str,
        target: PoseValue,
        seed: JointSolution | None,
        *,
        pregrasp: bool = False,
        candidate_index: int = -1,
    ) -> PoseAlignment | None:
        context = getattr(self._port, 'set_ik_context', None)
        if context is not None:
            context(candidate_index, arm, target, pregrasp=pregrasp)
        if pregrasp:
            solve = getattr(self._port, 'solve_pregrasp_ik', None)
            if solve is not None:
                return solve(arm, target, seed)
        solve = getattr(self._port, 'solve_aligned_ik', None)
        if solve is not None:
            return solve(arm, target, seed)
        legacy = getattr(self._port, 'solve_position_ik')
        solution = legacy(arm, target.position, seed)
        return None if solution is None else self._perfect_alignment(solution, target)

    def _ik_pair(
        self, candidate: Candidate, arm: str, pregrasp_target: PoseValue,
        grasp_target: PoseValue, check_canceled: Callable[[], None], feedback: Feedback,
        latencies: list[tuple[str, float]],
    ) -> tuple[PoseAlignment, PoseAlignment, PoseValue] | None:
        """Solve one endpoint, then seed the other endpoint from that solution."""
        index = candidate.source_index
        if not self._config.grasp_ik_first:
            self._port.set_target_contacts(None)
            started = self._monotonic()
            feedback(index, arm, EvaluationStage.PREGRASP_IK, 'evaluating')
            pregrasp = self._solve(arm, pregrasp_target, None, pregrasp=True,
                                   candidate_index=index)
            latencies.append(('pregrasp_ik', self._monotonic() - started))
            check_canceled()
            if pregrasp is None:
                feedback(index, arm, EvaluationStage.PREGRASP_IK, 'no IK solution')
                return
            feedback(index, arm, EvaluationStage.POSE_ALIGNMENT, 'pregrasp')
            if not pregrasp.within(self._config, pregrasp=True):
                feedback(index, arm, EvaluationStage.POSE_ALIGNMENT,
                         self._alignment_failure('pregrasp', pregrasp))
                return
            self._port.set_target_contacts(arm)
            started = self._monotonic()
            feedback(index, arm, EvaluationStage.GRASP_IK, 'evaluating')
            grasp = self._solve(arm, grasp_target, pregrasp.solution, candidate_index=index)
            latencies.append(('grasp_ik', self._monotonic() - started))
            check_canceled()
            if grasp is None:
                feedback(index, arm, EvaluationStage.GRASP_IK, 'no IK solution')
                return
            feedback(index, arm, EvaluationStage.POSE_ALIGNMENT, 'grasp')
            if not grasp.within(self._config):
                feedback(index, arm, EvaluationStage.POSE_ALIGNMENT,
                         self._alignment_failure('grasp', grasp))
                return
            return pregrasp, grasp, pregrasp_target

        self._port.set_target_contacts(arm)
        started = self._monotonic()
        feedback(index, arm, EvaluationStage.GRASP_IK, 'evaluating before pregrasp')
        grasp = self._solve(arm, grasp_target, None, candidate_index=index)
        latencies.append(('grasp_ik', self._monotonic() - started))
        check_canceled()
        if grasp is None:
            feedback(index, arm, EvaluationStage.GRASP_IK, 'no IK solution')
            return
        feedback(index, arm, EvaluationStage.POSE_ALIGNMENT, 'grasp')
        if not grasp.within(self._config):
            feedback(index, arm, EvaluationStage.POSE_ALIGNMENT,
                     self._alignment_failure('grasp', grasp))
            return
        targets = [pregrasp_target]
        fallbacks = self._config.pregrasp_fallback_offsets_m or (
            (self._config.pregrasp_fallback_offset_m,)
            if self._config.pregrasp_fallback_offset_m else ())
        for fallback in fallbacks:
            targets.append(PoseValue(self.pregrasp_position(
                candidate, fallback, grasp_target.position), candidate.orientation_xyzw))
        for attempt, target in enumerate(targets):
            check_canceled()
            self._port.set_target_contacts(None)
            started = self._monotonic()
            offset = math.dist(target.position, grasp_target.position)
            feedback(index, arm, EvaluationStage.PREGRASP_IK,
                     f'evaluating from grasp solution; offset={offset:.3f}m fallback={bool(attempt)}')
            pregrasp = self._solve(arm, target, grasp.solution, pregrasp=True,
                                   candidate_index=index)
            latencies.append(('pregrasp_ik', self._monotonic() - started))
            check_canceled()
            if pregrasp is None:
                feedback(index, arm, EvaluationStage.PREGRASP_IK, 'no IK solution')
                continue
            feedback(index, arm, EvaluationStage.POSE_ALIGNMENT, 'pregrasp')
            if pregrasp.within(self._config, pregrasp=True):
                return pregrasp, grasp, target
            feedback(index, arm, EvaluationStage.POSE_ALIGNMENT,
                     self._alignment_failure('pregrasp', pregrasp))

    def select(
        self,
        candidates: Sequence[Candidate],
        *,
        required_arm: str = "",
        cancel_requested: Callable[[], bool] = lambda: False,
        feedback: Feedback = lambda *_: None,
    ) -> Selection | None:
        if required_arm not in ("", "left", "right"):
            raise ValueError("required_arm must be empty, left, or right")
        ranked = sorted(candidates, key=lambda item: item.score, reverse=True)

        def check_canceled() -> None:
            if cancel_requested():
                raise InterruptedError('grasp selection canceled')

        attempts = self._evaluation_order(ranked, required_arm, check_canceled)
        if not self._config.pregrasp_motion_enabled:
            return self._select_direct(attempts, check_canceled, feedback)

        grasp_first = self._config.grasp_ik_first
        prepare = getattr(self._port, 'prepare_grasps' if grasp_first else 'prepare_pregrasps', None)
        if prepare is not None:
            requests = []
            for candidate, arm in attempts:
                check_canceled()
                try:
                    opening = self._config.opening_to_joint_position(candidate.required_opening_m)
                    position = self.grasp_tcp_position(candidate)
                except ValueError:
                    continue
                target = PoseValue(position if grasp_first else self.pregrasp_position(
                    candidate, self._config.pregrasp_offset_m, position), candidate.orientation_xyzw)
                if not getattr(self._port, 'prepares_pregrasps_lazily', False):
                    feedback(candidate.source_index, arm,
                             EvaluationStage.GRASP_IK if grasp_first else EvaluationStage.PREGRASP_IK,
                             'preparing batched endpoint IK')
                requests.append(PregraspRequest(arm, target, opening, candidate.source_index))
            self._port.set_target_contacts(None)
            prepare(requests)
            check_canceled()

        best_selection: Selection | None = None
        best_wrist_key: tuple[float, float, int] | None = None
        feasible_candidates: set[int] = set()

        def evaluation_passes():
            begin = getattr(self._port, 'begin_candidate_pass', None)
            deferred = begin(False) if begin is not None else False
            yield from attempts
            check_canceled()
            if not grasp_first and deferred and begin(True):
                # The ROS feedback candidate_index is uint32; identify the
                # first retried candidate instead of using a negative sentinel.
                feedback(attempts[0][0].source_index if attempts else 0, required_arm,
                         EvaluationStage.GRASP_IK if grasp_first else EvaluationStage.PREGRASP_IK,
                         'primary candidates exhausted; retrying deferred IK')
                yield from attempts

        for candidate, arm in evaluation_passes():
            if id(candidate) in feasible_candidates:
                continue
            index = candidate.source_index
            opening_started = self._monotonic()
            try:
                gripper_position = self._config.opening_to_joint_position(
                    candidate.required_opening_m
                )
                grasp_position = self.grasp_tcp_position(candidate)
            except ValueError as error:
                feedback(index, '', EvaluationStage.OPENING, str(error))
                continue
            opening_latency = self._monotonic() - opening_started
            tcp_shift = self._config.fixed_jaw_tcp_shift_m(
                candidate.required_opening_m)
            if candidate.lateral_offset_m is not None:
                tcp_shift = candidate.lateral_offset_m
            feedback(
                index, '', EvaluationStage.OPENING,
                f'mapped opening; fixed-jaw TCP shift={tcp_shift:.6f}m',
            )
            pregrasp_target = PoseValue(
                self.pregrasp_position(
                    candidate, self._config.pregrasp_offset_m, grasp_position),
                candidate.orientation_xyzw,
            )
            grasp_target = PoseValue(grasp_position, candidate.orientation_xyzw)
            check_canceled()
            latencies: list[tuple[str, float]] = [('opening', opening_latency)]
            set_gripper = getattr(self._port, 'set_gripper_position', None)
            if set_gripper is not None:
                set_gripper(arm, gripper_position)
            pair = self._ik_pair(
                candidate, arm, pregrasp_target, grasp_target, check_canceled, feedback, latencies)
            if pair is None:
                continue
            pregrasp, grasp, pregrasp_target = pair
            self._port.set_target_contacts(None)
            started = self._monotonic()
            feedback(index, arm, EvaluationStage.STATE_VALIDITY, 'pregrasp')
            if not self._port.state_is_valid(arm, pregrasp.solution):
                continue
            if (self._config.require_pregrasp_visibility
                    and not self._port.pregrasp_is_visible(arm, pregrasp.solution)):
                continue
            self._port.set_target_contacts(arm)
            feedback(index, arm, EvaluationStage.STATE_VALIDITY, 'grasp')
            if not self._port.state_is_valid(arm, grasp.solution):
                continue
            if (self._config.require_gripper_closure_clearance
                    and not self._port.gripper_sweep_is_valid(
                        arm, grasp.solution, gripper_position,
                        self._config.gripper_closed_joint_rad,
                        self._config.gripper_sweep_step_rad)):
                feedback(index, arm, EvaluationStage.STATE_VALIDITY,
                         'gripper closure intersects environment or robot')
                continue
            latencies.append(('state_validity', self._monotonic() - started))
            check_canceled()
            self._port.set_target_contacts(None)
            started = self._monotonic()
            feedback(index, arm, EvaluationStage.PLAN_PREGRASP, 'planning')
            pregrasp_path = self._port.plan(arm, pregrasp.solution, None)
            latencies.append(('plan_pregrasp', self._monotonic() - started))
            check_canceled()
            if not pregrasp_path:
                continue
            self._port.set_target_contacts(arm)
            started = self._monotonic()
            feedback(index, arm, EvaluationStage.PLAN_GRASP, 'planning')
            grasp_path = self._port.plan(arm, grasp.solution, pregrasp.solution)
            latencies.append(('plan_grasp', self._monotonic() - started))
            check_canceled()
            if not grasp_path:
                continue
            if self._config.require_approach_corridor:
                if not isinstance(grasp_path, PlannedPath):
                    raise InfrastructureError('Approach validation requires a trajectory')
                if not self._port.validate_approach_path(
                        arm, grasp_path, pregrasp_target, grasp_target):
                    feedback(index, arm, EvaluationStage.TRAJECTORY_ALIGNMENT,
                             'RRT approach leaves the permitted corridor')
                    continue
            trajectory = TrajectoryAlignment(0.0, 0.0, 0)
            feedback(index, arm, EvaluationStage.TRAJECTORY_ALIGNMENT, 'checking FK samples')
            if isinstance(grasp_path, PlannedPath):
                started = self._monotonic()
                trajectory = self._port.trajectory_alignment(
                    arm,
                    grasp_path,
                    candidate.orientation_xyzw,
                    self._config.trajectory_maximum_waypoints,
                )
                latencies.append(('trajectory_alignment', self._monotonic() - started))
                check_canceled()
                if trajectory is None or not trajectory.within(self._config):
                    continue
            selection = Selection(
                index, arm, candidate, pregrasp.solution, grasp.solution,
                gripper_position, pregrasp, grasp, trajectory,
                tuple(latencies),
                pregrasp_path if isinstance(pregrasp_path, PlannedPath) else None,
                grasp_path if isinstance(grasp_path, PlannedPath) else None,
            )
            if self._config.stop_after_first_feasible:
                return selection
            wrist_cost = getattr(self._port, 'wrist_motion_cost', None)
            if wrist_cost is None:
                return selection
            cost = wrist_cost(arm, pregrasp.solution, grasp.solution)
            if cost is None:
                return selection
            if not math.isfinite(cost) or cost < 0.0:
                raise InfrastructureError('Invalid wrist motion cost')
            feedback(
                index,
                arm,
                EvaluationStage.TRAJECTORY_ALIGNMENT,
                f'feasible; wrist travel={math.degrees(cost):.1f}deg',
            )
            key = (cost, -candidate.score, index)
            if best_wrist_key is None or key < best_wrist_key:
                best_selection = selection
                best_wrist_key = key
            # No later candidate can improve on zero wrist movement.
            if cost <= 1e-9:
                return best_selection
            # A complete pair is enough for this candidate. Preserve the
            # preferred-arm policy while comparing later candidates.
            feasible_candidates.add(id(candidate))

        return best_selection

    @property
    def pregrasp_motion_enabled(self) -> bool:
        return self._config.pregrasp_motion_enabled

    def _select_direct(
        self, attempts: Sequence[tuple[Candidate, str]],
        check_canceled: Callable[[], None], feedback: Feedback,
    ) -> Selection | None:
        """Validate the current-to-grasp route without unused pregrasp prerequisites."""
        for candidate, arm in attempts:
            index = candidate.source_index
            check_canceled()
            started = self._monotonic()
            try:
                opening = self._config.opening_to_joint_position(candidate.required_opening_m)
                target = PoseValue(self.grasp_tcp_position(candidate), candidate.orientation_xyzw)
            except ValueError as error:
                feedback(index, '', EvaluationStage.OPENING, str(error))
                continue
            opening_latency = self._monotonic()-started
            feedback(index, '', EvaluationStage.OPENING, 'direct grasp; pregrasp selection skipped')
            check_canceled()
            latencies = [('opening', opening_latency)]
            set_gripper = getattr(self._port, 'set_gripper_position', None)
            if set_gripper is not None:
                set_gripper(arm, opening)
            self._port.set_target_contacts(arm)
            started = self._monotonic()
            feedback(index, arm, EvaluationStage.GRASP_IK, 'evaluating direct grasp')
            grasp = self._solve(arm, target, None, candidate_index=index)
            latencies.append(('grasp_ik', self._monotonic()-started))
            check_canceled()
            if grasp is None:
                feedback(index, arm, EvaluationStage.GRASP_IK, 'no grasp IK solution')
                continue
            if not grasp.within(self._config):
                feedback(index, arm, EvaluationStage.POSE_ALIGNMENT,
                         self._alignment_failure('grasp', grasp))
                continue
            started = self._monotonic()
            feedback(index, arm, EvaluationStage.STATE_VALIDITY, 'grasp')
            if not self._port.state_is_valid(arm, grasp.solution):
                feedback(index, arm, EvaluationStage.STATE_VALIDITY, 'grasp state invalid')
                continue
            if (self._config.require_gripper_closure_clearance
                    and not self._port.gripper_sweep_is_valid(
                        arm, grasp.solution, opening, self._config.gripper_closed_joint_rad,
                        self._config.gripper_sweep_step_rad)):
                feedback(index, arm, EvaluationStage.STATE_VALIDITY,
                         'gripper closure intersects environment or robot')
                continue
            latencies.append(('state_validity', self._monotonic()-started))
            check_canceled()
            started = self._monotonic()
            feedback(index, arm, EvaluationStage.PLAN_GRASP, 'planning current-to-grasp')
            plan = (getattr(self._port, 'plan_direct_grasp', None)
                    if self._config.direct_grasp_shortcut_enabled else None)
            path = (plan(arm, grasp.solution) if plan is not None
                    else self._port.plan(arm, grasp.solution, None))
            latencies.append(('plan_direct_grasp', self._monotonic()-started))
            check_canceled()
            if not path:
                feedback(index, arm, EvaluationStage.PLAN_GRASP, 'current-to-grasp plan failed')
                continue
            # Intermediate TCP orientation and an approach corridor are not
            # constraints on this joint-space move. Endpoint/path validity is.
            return Selection(index, arm, candidate, None, grasp.solution, opening,
                             grasp_alignment=grasp, latencies=tuple(latencies),
                             grasp_path=path if isinstance(path, PlannedPath) else None)
        return None

    @staticmethod
    def _alignment_failure(label: str, alignment: PoseAlignment) -> str:
        return (
            f'{label} outside tolerance: pos={alignment.position_error_m:.4f}m '
            f'approach={alignment.approach_error_degrees:.1f}deg '
            f'closing={alignment.closing_error_degrees:.1f}deg'
        )



def unsigned_axis_error_deg(actual, expected) -> float:
    return min(directed_axis_error_deg(actual, expected),
               directed_axis_error_deg(actual, tuple(-value for value in expected)))
