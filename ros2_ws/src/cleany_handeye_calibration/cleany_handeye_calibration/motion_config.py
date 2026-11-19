"""안전 설정과 ROS에 독립적인 보정 실행 전 검사입니다."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence

from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    RIGHT_ARM_JOINT_NAMES,
    DualArmJointContract,
)
from cleany_handeye_calibration.models import TimedJointSample


CALIBRATION_PLANNING_GROUP = 'left_arm'
CALIBRATION_TIP_LINK = 'left_gripper_frame'
CALIBRATION_BASE_FRAME = 'base_link'
RIGHT_PARK_POSITIONS_RAD = (0.0, 0.0, 0.0, 0.0, 0.0)
NANOSECONDS_PER_SECOND = 1_000_000_000


def _positive_finite(value: float, *, field_name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{field_name} must be positive and finite')
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'{field_name} must be positive and finite'
        ) from error
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f'{field_name} must be positive and finite')
    return number


def _unit_interval(value: float, *, field_name: str) -> float:
    number = _positive_finite(value, field_name=field_name)
    if number > 1.0:
        raise ValueError(f'{field_name} must be in (0, 1]')
    return number


def _positive_integer(value: int, *, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f'{field_name} must be a positive integer')
    return value


def _finite_tuple(
    values: Sequence[float],
    *,
    field_name: str,
    expected_length: int,
) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'{field_name} must contain numeric values'
        ) from error
    if len(result) != expected_length:
        raise ValueError(
            f'{field_name} must contain {expected_length} values'
        )
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{field_name} must contain finite values')
    return result


@dataclass(frozen=True, slots=True)
class StageTimeouts:
    """각 실행 단계에 필요한 단조 시계 기준 시간 예산입니다."""

    ik_sec: float
    state_validity_sec: float
    plan_sec: float
    execute_sec: float
    cancel_sec: float
    settle_sec: float

    def __post_init__(self) -> None:
        for field_name in (
            'ik_sec',
            'state_validity_sec',
            'plan_sec',
            'execute_sec',
            'cancel_sec',
            'settle_sec',
        ):
            object.__setattr__(
                self,
                field_name,
                _positive_finite(
                    getattr(self, field_name),
                    field_name=field_name,
                ),
            )


@dataclass(frozen=True, slots=True)
class MujocoMotionConfig:
    """MuJoCo 안전 기준과 실행별 필수 전제 조건입니다.

    최신성, 오른팔 주차 허용 오차와 각 단계 시간 제한에는 기본값이 없습니다.
    호출자는 공통 시간 제한을 암묵적으로 물려받는 대신 측정에 근거한
    실행 시간 예산을 구체적으로 제공해야 합니다.
    """

    current_state_max_age_sec: float
    right_park_position_tolerance_rad: float
    stage_timeouts: StageTimeouts
    max_velocity_scaling_factor: float = 0.10
    max_acceleration_scaling_factor: float = 0.10
    controller_path_tolerance_rad: float = 0.05
    controller_goal_tolerance_rad: float = 0.01
    settle_position_tolerance_rad: float = 0.015
    settle_velocity_tolerance_rad_s: float = 0.01
    settle_duration_sec: float = 1.0
    planning_attempts: int = 1

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            'current_state_max_age_sec',
            _positive_finite(
                self.current_state_max_age_sec,
                field_name='current_state_max_age_sec',
            ),
        )
        object.__setattr__(
            self,
            'right_park_position_tolerance_rad',
            _positive_finite(
                self.right_park_position_tolerance_rad,
                field_name='right_park_position_tolerance_rad',
            ),
        )
        if not isinstance(self.stage_timeouts, StageTimeouts):
            raise ValueError('stage_timeouts must be StageTimeouts')
        for field_name in (
            'max_velocity_scaling_factor',
            'max_acceleration_scaling_factor',
        ):
            object.__setattr__(
                self,
                field_name,
                _unit_interval(
                    getattr(self, field_name),
                    field_name=field_name,
                ),
            )
        for field_name in (
            'controller_path_tolerance_rad',
            'controller_goal_tolerance_rad',
            'settle_position_tolerance_rad',
            'settle_velocity_tolerance_rad_s',
            'settle_duration_sec',
        ):
            object.__setattr__(
                self,
                field_name,
                _positive_finite(
                    getattr(self, field_name),
                    field_name=field_name,
                ),
            )
        object.__setattr__(
            self,
            'planning_attempts',
            _positive_integer(
                self.planning_attempts,
                field_name='planning_attempts',
            ),
        )
        if (
            self.controller_goal_tolerance_rad
            > self.controller_path_tolerance_rad
        ):
            raise ValueError(
                'controller goal tolerance must not exceed path tolerance'
            )
        if (
            self.settle_position_tolerance_rad
            > self.controller_path_tolerance_rad
        ):
            raise ValueError(
                'settle position tolerance must not exceed controller path '
                'tolerance'
            )
        if self.stage_timeouts.settle_sec < self.settle_duration_sec:
            raise ValueError(
                'settle timeout must be at least the settle duration'
            )

    @property
    def current_state_max_age_ns(self) -> int:
        return round(
            self.current_state_max_age_sec * NANOSECONDS_PER_SECOND
        )


@dataclass(frozen=True, slots=True)
class CalibrationScope:
    """이 절차에서 허용하는 유일한 팔 그룹과 IK 끝점입니다."""

    planning_group: str
    tip_link: str

    def __post_init__(self) -> None:
        validate_calibration_scope(self.planning_group, self.tip_link)


def validate_calibration_scope(
    planning_group: str,
    tip_link: str,
) -> None:
    """어댑터가 ROS에 접속하기 전에 왼팔 이외의 범위를 거부합니다."""

    if planning_group != CALIBRATION_PLANNING_GROUP:
        raise ValueError(
            'calibration planning_group must be exactly '
            f'{CALIBRATION_PLANNING_GROUP!r}, got {planning_group!r}'
        )
    if tip_link != CALIBRATION_TIP_LINK:
        raise ValueError(
            'calibration tip_link must be exactly '
            f'{CALIBRATION_TIP_LINK!r}, got {tip_link!r}'
        )


class StartupFailure(str, Enum):
    """안전 사전 검사의 고정 거부 분류입니다."""

    INCOMPLETE_DUAL_ARM_STATE = 'incomplete_dual_arm_state'
    FUTURE_STATE_STAMP = 'future_state_stamp'
    STALE_CURRENT_STATE = 'stale_current_state'
    RIGHT_ARM_NOT_PARKED = 'right_arm_not_parked'
    RIGHT_ARM_VELOCITY_MISSING = 'right_arm_velocity_missing'
    RIGHT_ARM_MOVING = 'right_arm_moving'


class StartupPreconditionError(RuntimeError):
    """이동 없이 시작을 거부하며 이유를 확인할 수 있습니다."""

    def __init__(
        self,
        failure: StartupFailure,
        message: str,
        *,
        joint_names: Sequence[str] = (),
    ) -> None:
        self.failure = StartupFailure(failure)
        self.joint_names = tuple(joint_names)
        super().__init__(message)


@dataclass(frozen=True, slots=True)
class ValidatedCurrentState:
    """특정 ROS 시각에 최신성과 오른팔 주차 상태를 확인한 완전한 피드백입니다."""

    sample: TimedJointSample
    validated_at_stamp_ns: int
    age_ns: int


def validate_dual_arm_current_state(
    sample: TimedJointSample,
    *,
    now_stamp_ns: int,
    config: MujocoMotionConfig,
    joint_contract: DualArmJointContract = (
        DEFAULT_DUAL_ARM_JOINT_CONTRACT
    ),
    right_park_positions_rad: Sequence[float] = (
        RIGHT_PARK_POSITIONS_RAD
    ),
) -> ValidatedCurrentState:
    """최신성, 전체 피드백과 오른팔의 정지 주차 상태를 확인합니다.

    now_stamp_ns와 표본 stamp는 ROS 데이터 시각입니다. 여기서는 경과 시간 시계를
    조회하지 않으며 단조 시계는 대기 시간 제한에만 사용합니다.
    """

    if not isinstance(sample, TimedJointSample):
        raise ValueError('sample must be a TimedJointSample')
    if not isinstance(config, MujocoMotionConfig):
        raise ValueError('config must be MujocoMotionConfig')
    if not isinstance(joint_contract, DualArmJointContract):
        raise ValueError('joint_contract must be DualArmJointContract')
    if (
        isinstance(now_stamp_ns, bool)
        or not isinstance(now_stamp_ns, int)
        or now_stamp_ns < 0
    ):
        raise ValueError('now_stamp_ns must be a non-negative integer')
    park_positions = _finite_tuple(
        right_park_positions_rad,
        field_name='right_park_positions_rad',
        expected_length=len(RIGHT_ARM_JOINT_NAMES),
    )

    missing = joint_contract.missing_joint_names(sample)
    if missing:
        raise StartupPreconditionError(
            StartupFailure.INCOMPLETE_DUAL_ARM_STATE,
            'current state is missing required feedback joints: '
            f'{list(missing)!r}',
            joint_names=missing,
        )
    if sample.stamp_ns > now_stamp_ns:
        raise StartupPreconditionError(
            StartupFailure.FUTURE_STATE_STAMP,
            'current-state stamp is later than the validation ROS stamp',
        )
    age_ns = now_stamp_ns - sample.stamp_ns
    if age_ns > config.current_state_max_age_ns:
        raise StartupPreconditionError(
            StartupFailure.STALE_CURRENT_STATE,
            'current state is stale: '
            f'age={age_ns} ns, max={config.current_state_max_age_ns} ns',
        )

    positions = dict(
        zip(sample.joint_names, sample.positions_rad, strict=True)
    )
    off_park = tuple(
        name
        for name, expected in zip(
            RIGHT_ARM_JOINT_NAMES,
            park_positions,
            strict=True,
        )
        if abs(positions[name] - expected)
        > config.right_park_position_tolerance_rad
    )
    if off_park:
        raise StartupPreconditionError(
            StartupFailure.RIGHT_ARM_NOT_PARKED,
            'right arm is outside the approved all-zero park tolerance: '
            f'{list(off_park)!r}',
            joint_names=off_park,
        )

    if sample.velocities_rad_s is None:
        raise StartupPreconditionError(
            StartupFailure.RIGHT_ARM_VELOCITY_MISSING,
            'right-arm park validation requires velocity feedback',
            joint_names=RIGHT_ARM_JOINT_NAMES,
        )
    velocities = dict(
        zip(sample.joint_names, sample.velocities_rad_s, strict=True)
    )
    moving = tuple(
        name
        for name in RIGHT_ARM_JOINT_NAMES
        if abs(velocities[name])
        > config.settle_velocity_tolerance_rad_s
    )
    if moving:
        raise StartupPreconditionError(
            StartupFailure.RIGHT_ARM_MOVING,
            'right arm is not stationary at the approved park pose: '
            f'{list(moving)!r}',
            joint_names=moving,
        )

    return ValidatedCurrentState(
        sample=sample,
        validated_at_stamp_ns=now_stamp_ns,
        age_ns=age_ns,
    )
