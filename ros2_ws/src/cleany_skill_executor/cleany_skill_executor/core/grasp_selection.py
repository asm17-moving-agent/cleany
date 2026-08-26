"""ROS-independent candidate/arm fallback orchestration."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Callable, Protocol, Sequence


ARM_JOINT_NAMES = {
    side: tuple(
        f'{side}_{suffix}_joint'
        for suffix in (
            'shoulder_yaw',
            'shoulder_pitch',
            'elbow_pitch',
            'wrist_pitch',
            'wrist_roll',
        )
    )
    for side in ('left', 'right')
}
REQUIRED_JOINT_NAMES = (
    *ARM_JOINT_NAMES['left'],
    'left_gripper_joint',
    *ARM_JOINT_NAMES['right'],
    'right_gripper_joint',
)


class EvaluationStage(str, Enum):
    PREGRASP_IK = 'PREGRASP_IK'
    GRASP_IK = 'GRASP_IK'
    STATE_VALIDITY = 'STATE_VALIDITY'
    PLAN_PREGRASP = 'PLAN_PREGRASP'
    PLAN_GRASP = 'PLAN_GRASP'


class InfrastructureError(RuntimeError):
    """A transport or MoveIt failure for which pair fallback is unsafe."""


@dataclass(frozen=True, slots=True)
class Candidate:
    position: tuple[float, float, float]
    approach_direction: tuple[float, float, float]
    score: float
    source_index: int = 0
    orientation: tuple[float, float, float, float] | None = None

    def __post_init__(self) -> None:
        values = (*self.position, *self.approach_direction, self.score)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('candidate values must be finite')
        approach = _normalize(self.approach_direction, 'approach_direction')
        quaternion = self.orientation or _quaternion_with_local_minus_y(approach)
        quaternion = _normalize_quaternion(quaternion)
        pose_approach = quaternion_axis(quaternion, (0.0, -1.0, 0.0))
        if directed_axis_error_deg(pose_approach, approach) > 0.1:
            raise ValueError(
                'approach_direction must match tcp_pose local -Y axis'
            )
        object.__setattr__(self, 'approach_direction', approach)
        object.__setattr__(self, 'orientation', quaternion)

    @property
    def closing_direction(self) -> tuple[float, float, float]:
        assert self.orientation is not None
        return quaternion_axis(self.orientation, (1.0, 0.0, 0.0))


def _normalize(values: tuple[float, ...], name: str) -> tuple[float, ...]:
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError(f'{name} must be non-zero and finite')
    return tuple(value / norm for value in values)


def _normalize_quaternion(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    if len(quaternion) != 4 or not all(math.isfinite(value) for value in quaternion):
        raise ValueError('orientation must be a finite quaternion')
    return _normalize(quaternion, 'orientation')  # type: ignore[return-value]


def quaternion_axis(
    quaternion: tuple[float, float, float, float],
    local_axis: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Rotate a local unit axis by an xyzw quaternion."""
    x, y, z, w = _normalize_quaternion(quaternion)
    vx, vy, vz = local_axis
    # Expanded q * v * q^-1.
    return (
        (1 - 2 * (y * y + z * z)) * vx
        + 2 * (x * y - z * w) * vy
        + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx
        + (1 - 2 * (x * x + z * z)) * vy
        + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx
        + 2 * (y * z + x * w) * vy
        + (1 - 2 * (x * x + y * y)) * vz,
    )


def directed_axis_error_deg(
    actual: tuple[float, float, float], expected: tuple[float, float, float]
) -> float:
    left = _normalize(actual, 'actual axis')
    right = _normalize(expected, 'expected axis')
    cosine = max(-1.0, min(1.0, sum(a * b for a, b in zip(left, right))))
    return math.degrees(math.acos(cosine))


def unsigned_axis_error_deg(
    actual: tuple[float, float, float], expected: tuple[float, float, float]
) -> float:
    """Parallel-jaw closing axes are equivalent after a 180 degree flip."""
    return min(
        directed_axis_error_deg(actual, expected),
        directed_axis_error_deg(actual, tuple(-value for value in expected)),
    )


def _quaternion_with_local_minus_y(
    approach: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    """Compatibility orientation for core-only callers without a wire pose."""
    ay = tuple(-value for value in approach)
    reference = (0.0, 0.0, 1.0) if abs(ay[2]) < 0.9 else (1.0, 0.0, 0.0)
    x_axis = _normalize(
        (
            reference[1] * ay[2] - reference[2] * ay[1],
            reference[2] * ay[0] - reference[0] * ay[2],
            reference[0] * ay[1] - reference[1] * ay[0],
        ),
        'closing axis',
    )
    z_axis = (
        x_axis[1] * ay[2] - x_axis[2] * ay[1],
        x_axis[2] * ay[0] - x_axis[0] * ay[2],
        x_axis[0] * ay[1] - x_axis[1] * ay[0],
    )
    # Rotation matrix columns are local +X, +Y, +Z.
    m00, m10, m20 = x_axis
    m01, m11, m21 = ay
    m02, m12, m22 = z_axis
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((m21 - m12) / scale, (m02 - m20) / scale, (m10 - m01) / scale, 0.25 * scale)
    matrix = ((m00, m01, m02), (m10, m11, m12), (m20, m21, m22))
    index = max(range(3), key=lambda item: matrix[item][item])
    i, j, k = ((0, 1, 2), (1, 2, 0), (2, 0, 1))[index]
    scale = math.sqrt(1.0 + matrix[i][i] - matrix[j][j] - matrix[k][k]) * 2.0
    result = [0.0, 0.0, 0.0, (matrix[k][j] - matrix[j][k]) / scale]
    result[i] = 0.25 * scale
    result[j] = (matrix[i][j] + matrix[j][i]) / scale
    result[k] = (matrix[i][k] + matrix[k][i]) / scale
    return tuple(result)  # type: ignore[return-value]


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
class Selection:
    candidate_index: int
    arm: str
    candidate: Candidate
    pregrasp: JointSolution
    grasp: JointSolution


@dataclass(frozen=True, slots=True)
class GraspSelectionConfig:
    pregrasp_offset_m: float = 0.14
    pregrasp_seed_offset_m: float = 0.08
    maximum_candidates: int = 12

    def __post_init__(self) -> None:
        if not math.isfinite(self.pregrasp_offset_m) or self.pregrasp_offset_m <= 0:
            raise ValueError('pregrasp_offset_m must be positive and finite')
        if (
            not math.isfinite(self.pregrasp_seed_offset_m)
            or self.pregrasp_seed_offset_m <= 0
        ):
            raise ValueError('pregrasp_seed_offset_m must be positive and finite')
        if self.maximum_candidates <= 0:
            raise ValueError('maximum_candidates must be positive')


class ReachabilityPort(Protocol):
    def set_target_contacts(self, arm: str | None) -> None:
        ...

    def solve_position_ik(
        self,
        arm: str,
        position: tuple[float, float, float],
        seed: JointSolution | None,
    ) -> JointSolution | None:
        ...

    def solve_aimed_pregrasp_ik(
        self,
        arm: str,
        grasp_position: tuple[float, float, float],
        approach_direction: tuple[float, float, float],
        closing_direction: tuple[float, float, float],
        pregrasp_position: tuple[float, float, float],
        seed: JointSolution | None,
    ) -> tuple[JointSolution, ...]:
        ...

    def solve_grasp_ik(
        self,
        arm: str,
        grasp_position: tuple[float, float, float],
        approach_direction: tuple[float, float, float],
        closing_direction: tuple[float, float, float],
        seed: JointSolution,
    ) -> tuple[JointSolution, ...]:
        ...

    def state_is_valid(self, arm: str, solution: JointSolution) -> bool:
        ...

    def plan(
        self,
        arm: str,
        goal: JointSolution,
        start: JointSolution | None,
    ) -> bool:
        ...


Feedback = Callable[[int, str, EvaluationStage, str], None]


class GraspSelector:
    def __init__(
        self,
        port: ReachabilityPort,
        config: GraspSelectionConfig = GraspSelectionConfig(),
    ) -> None:
        self._port = port
        self._config = config

    @staticmethod
    def pregrasp_position(
        candidate: Candidate,
        offset_m: float = 0.14,
    ) -> tuple[float, float, float]:
        norm = math.sqrt(sum(value * value for value in candidate.approach_direction))
        if not math.isfinite(norm) or norm <= 1e-9:
            raise ValueError('approach_direction must be non-zero')
        return tuple(
            position - direction / norm * offset_m
            for position, direction in zip(
                candidate.position, candidate.approach_direction, strict=True
            )
        )

    @staticmethod
    def arm_order(candidate: Candidate) -> tuple[str, str]:
        return ('left', 'right') if candidate.position[1] >= 0.0 else ('right', 'left')

    def select(
        self,
        candidates: Sequence[Candidate],
        *,
        cancel_requested: Callable[[], bool] = lambda: False,
        feedback: Feedback = lambda *_: None,
    ) -> Selection | None:
        ranked = sorted(candidates, key=lambda item: item.score, reverse=True)[
            : self._config.maximum_candidates
        ]

        def check_canceled() -> None:
            if cancel_requested():
                raise InterruptedError('grasp selection canceled')

        for candidate in ranked:
            pregrasp_position = self.pregrasp_position(
                candidate, self._config.pregrasp_offset_m
            )
            pregrasp_seed_position = self.pregrasp_position(
                candidate, self._config.pregrasp_seed_offset_m
            )
            for arm in self.arm_order(candidate):
                check_canceled()
                index = candidate.source_index
                self._port.set_target_contacts(None)
                feedback(index, arm, EvaluationStage.PREGRASP_IK, 'evaluating')
                pregrasp_seed = self._port.solve_position_ik(
                    arm, pregrasp_seed_position, None
                )
                check_canceled()
                pregrasps = self._port.solve_aimed_pregrasp_ik(
                    arm,
                    candidate.position,
                    candidate.approach_direction,
                    candidate.closing_direction,
                    pregrasp_position,
                    pregrasp_seed,
                )
                check_canceled()
                if not pregrasps:
                    feedback(
                        index,
                        arm,
                        EvaluationStage.PREGRASP_IK,
                        'no direction-aligned IK solution'
                        if pregrasp_seed is not None
                        else 'seed IK failed; current-state fallback also failed',
                    )
                    continue
                for pregrasp_attempt, pregrasp in enumerate(pregrasps, start=1):
                    self._port.set_target_contacts(None)
                    feedback(
                        index,
                        arm,
                        EvaluationStage.STATE_VALIDITY,
                        f'pregrasp solution {pregrasp_attempt}/{len(pregrasps)}',
                    )
                    if not self._port.state_is_valid(arm, pregrasp):
                        check_canceled()
                        continue
                    check_canceled()
                    feedback(
                        index,
                        arm,
                        EvaluationStage.PLAN_PREGRASP,
                        f'planning solution {pregrasp_attempt}/{len(pregrasps)}',
                    )
                    if not self._port.plan(arm, pregrasp, None):
                        check_canceled()
                        continue
                    check_canceled()

                    self._port.set_target_contacts(arm)
                    feedback(index, arm, EvaluationStage.GRASP_IK, 'evaluating')
                    grasps = self._port.solve_grasp_ik(
                        arm,
                        candidate.position,
                        candidate.approach_direction,
                        candidate.closing_direction,
                        pregrasp,
                    )
                    check_canceled()
                    if not grasps:
                        feedback(
                            index,
                            arm,
                            EvaluationStage.GRASP_IK,
                            'no pose-compatible IK solution',
                        )
                        continue
                    for grasp_attempt, grasp in enumerate(grasps, start=1):
                        feedback(
                            index,
                            arm,
                            EvaluationStage.STATE_VALIDITY,
                            f'grasp solution {grasp_attempt}/{len(grasps)}',
                        )
                        if not self._port.state_is_valid(arm, grasp):
                            check_canceled()
                            continue
                        check_canceled()
                        feedback(
                            index,
                            arm,
                            EvaluationStage.PLAN_GRASP,
                            f'planning solution {grasp_attempt}/{len(grasps)}',
                        )
                        if not self._port.plan(arm, grasp, pregrasp):
                            check_canceled()
                            continue
                        check_canceled()
                        return Selection(index, arm, candidate, pregrasp, grasp)
        return None
