"""ROS에 독립적인 불변 보정 모델입니다."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence

from cleany_handeye_calibration.transforms import RigidTransform, Vector3


def _non_empty_text(value: str, *, field_name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f'{field_name} must be a non-empty string')
    if value != value.strip():
        raise ValueError(
            f'{field_name} must not contain surrounding whitespace'
        )
    return value


def _finite_tuple(
    values: Sequence[float],
    *,
    field_name: str,
    expected_length: int | None = None,
) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'{field_name} must contain numeric values'
        ) from error
    if expected_length is not None and len(result) != expected_length:
        raise ValueError(
            f'{field_name} must contain {expected_length} values, '
            f'got {len(result)}'
        )
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{field_name} must contain only finite values')
    return result


def _joint_names(values: Sequence[str]) -> tuple[str, ...]:
    names = tuple(
        _non_empty_text(value, field_name='joint name') for value in values
    )
    if not names:
        raise ValueError('joint_names must not be empty')
    if len(set(names)) != len(names):
        raise ValueError('joint_names must not contain duplicates')
    return names


def _stamp_ns(value: int, *, field_name: str = 'stamp_ns') -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f'{field_name} must be a non-negative integer')
    return value


@dataclass(frozen=True, slots=True)
class PositionTarget:
    """frame_id 좌표계의 미터 단위 Cartesian 위치 목표입니다."""

    frame_id: str
    position_m: Vector3

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            'frame_id',
            _non_empty_text(self.frame_id, field_name='frame_id'),
        )
        object.__setattr__(
            self,
            'position_m',
            _finite_tuple(
                self.position_m,
                field_name='position_m',
                expected_length=3,
            ),
        )


@dataclass(frozen=True, slots=True)
class JointPose:
    """순서가 정해진 완전한 관절 위치 벡터이며 단위는 라디안입니다."""

    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...]

    def __post_init__(self) -> None:
        names = _joint_names(self.joint_names)
        positions = _finite_tuple(
            self.positions_rad,
            field_name='positions_rad',
        )
        if len(names) != len(positions):
            raise ValueError(
                'joint_names and positions_rad must have equal lengths'
            )
        object.__setattr__(self, 'joint_names', names)
        object.__setattr__(self, 'positions_rad', positions)


@dataclass(frozen=True, slots=True)
class IkResult:
    """성공 및 실패 데이터가 상호 배타적인 위치 IK 결과입니다."""

    success: bool
    joint_pose: JointPose | None = None
    failure_reason: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.success, bool):
            raise ValueError('success must be a bool')
        if self.success:
            if self.joint_pose is None:
                raise ValueError('successful IK requires joint_pose')
            if self.failure_reason is not None:
                raise ValueError(
                    'successful IK must not include failure_reason'
                )
        else:
            if self.joint_pose is not None:
                raise ValueError('failed IK must not include joint_pose')
            if self.failure_reason is None:
                raise ValueError('failed IK requires failure_reason')
            object.__setattr__(
                self,
                'failure_reason',
                _non_empty_text(
                    self.failure_reason,
                    field_name='failure_reason',
                ),
            )


@dataclass(frozen=True, slots=True)
class TimedJointSample:
    """시각이 있는 관절 피드백이며 위치는 rad, 속도는 rad/s입니다."""

    stamp_ns: int
    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...]
    velocities_rad_s: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        stamp = _stamp_ns(self.stamp_ns)
        names = _joint_names(self.joint_names)
        positions = _finite_tuple(
            self.positions_rad,
            field_name='positions_rad',
        )
        if len(names) != len(positions):
            raise ValueError(
                'joint_names and positions_rad must have equal lengths'
            )
        velocities = None
        if self.velocities_rad_s is not None:
            velocities = _finite_tuple(
                self.velocities_rad_s,
                field_name='velocities_rad_s',
            )
            if len(names) != len(velocities):
                raise ValueError(
                    'joint_names and velocities_rad_s must have equal lengths'
                )
        object.__setattr__(self, 'stamp_ns', stamp)
        object.__setattr__(self, 'joint_names', names)
        object.__setattr__(self, 'positions_rad', positions)
        object.__setattr__(self, 'velocities_rad_s', velocities)


class SampleSplit(str, Enum):
    CALIBRATION = 'calibration'
    HELD_OUT = 'held_out'


@dataclass(frozen=True, slots=True)
class CalibrationSample:
    """eye-in-hand 방정식에 사용하는 동기화된 자세 쌍입니다.

    base_T_gripper는 gripper 좌표를 base 좌표로, camera_T_target은 target 좌표를
    camera 좌표로 변환합니다. solver가 구하는 미지 변환은 gripper_T_camera입니다.
    """

    sample_id: str
    pose_id: str
    split: SampleSplit
    base_T_gripper: RigidTransform
    camera_T_target: RigidTransform

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            'sample_id',
            _non_empty_text(self.sample_id, field_name='sample_id'),
        )
        object.__setattr__(
            self,
            'pose_id',
            _non_empty_text(self.pose_id, field_name='pose_id'),
        )
        try:
            split = SampleSplit(self.split)
        except ValueError as error:
            raise ValueError(
                'split must be calibration or held_out'
            ) from error
        object.__setattr__(self, 'split', split)
        if not isinstance(self.base_T_gripper, RigidTransform):
            raise ValueError('base_T_gripper must be a RigidTransform')
        if not isinstance(self.camera_T_target, RigidTransform):
            raise ValueError('camera_T_target must be a RigidTransform')
