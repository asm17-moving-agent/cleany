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

    def __post_init__(self) -> None:
        values = (*self.position, *self.approach_direction, self.score)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('candidate values must be finite')


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
    pregrasp_offset_m: float = 0.08
    maximum_candidates: int = 12

    def __post_init__(self) -> None:
        if not math.isfinite(self.pregrasp_offset_m) or self.pregrasp_offset_m <= 0:
            raise ValueError('pregrasp_offset_m must be positive and finite')
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
        offset_m: float = 0.08,
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
        for candidate in ranked:
            pregrasp_position = self.pregrasp_position(
                candidate, self._config.pregrasp_offset_m
            )
            for arm in self.arm_order(candidate):
                if cancel_requested():
                    raise InterruptedError('grasp selection canceled')
                index = candidate.source_index
                self._port.set_target_contacts(None)
                feedback(index, arm, EvaluationStage.PREGRASP_IK, 'evaluating')
                pregrasp = self._port.solve_position_ik(arm, pregrasp_position, None)
                if pregrasp is None:
                    feedback(index, arm, EvaluationStage.PREGRASP_IK, 'no IK solution')
                    continue
                self._port.set_target_contacts(arm)
                feedback(index, arm, EvaluationStage.GRASP_IK, 'evaluating')
                grasp = self._port.solve_position_ik(arm, candidate.position, pregrasp)
                if grasp is None:
                    feedback(index, arm, EvaluationStage.GRASP_IK, 'no IK solution')
                    continue
                self._port.set_target_contacts(None)
                feedback(index, arm, EvaluationStage.STATE_VALIDITY, 'pregrasp')
                if not self._port.state_is_valid(arm, pregrasp):
                    continue
                self._port.set_target_contacts(arm)
                feedback(index, arm, EvaluationStage.STATE_VALIDITY, 'grasp')
                if not self._port.state_is_valid(arm, grasp):
                    continue
                self._port.set_target_contacts(None)
                feedback(index, arm, EvaluationStage.PLAN_PREGRASP, 'planning')
                if not self._port.plan(arm, pregrasp, None):
                    continue
                self._port.set_target_contacts(arm)
                feedback(index, arm, EvaluationStage.PLAN_GRASP, 'planning')
                if not self._port.plan(arm, grasp, pregrasp):
                    continue
                return Selection(index, arm, candidate, pregrasp, grasp)
        return None
