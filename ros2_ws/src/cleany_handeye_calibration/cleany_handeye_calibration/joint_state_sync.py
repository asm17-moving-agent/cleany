"""ROS 시각 기준 관절 피드백 버퍼와 보간입니다.

이 core 모듈은 ROS를 import하지 않습니다. 호출자가 ROS 경계에서
sensor_msgs/msg/JointState의 header stamp를 정수 나노초로 변환한 뒤
TimedJointSample로 전달합니다.
"""

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence

from cleany_handeye_calibration.models import TimedJointSample


ARM_JOINT_SUFFIXES = (
    'shoulder_yaw_joint',
    'shoulder_pitch_joint',
    'elbow_pitch_joint',
    'wrist_pitch_joint',
    'wrist_roll_joint',
)
LEFT_ARM_JOINT_NAMES = tuple(
    f'left_{suffix}' for suffix in ARM_JOINT_SUFFIXES
)
RIGHT_ARM_JOINT_NAMES = tuple(
    f'right_{suffix}' for suffix in ARM_JOINT_SUFFIXES
)
LEFT_FEEDBACK_JOINT_NAMES = LEFT_ARM_JOINT_NAMES + (
    'left_gripper_joint',
)
RIGHT_FEEDBACK_JOINT_NAMES = RIGHT_ARM_JOINT_NAMES + (
    'right_gripper_joint',
)


def _positive_integer(value: int, *, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f'{field_name} must be a positive integer')
    return value


def _joint_name_tuple(
    values: Sequence[str],
    *,
    field_name: str,
) -> tuple[str, ...]:
    names = tuple(values)
    if not names:
        raise ValueError(f'{field_name} must not be empty')
    if any(
        not isinstance(name, str)
        or not name
        or name != name.strip()
        for name in names
    ):
        raise ValueError(
            f'{field_name} must contain non-empty trimmed strings'
        )
    if len(set(names)) != len(names):
        raise ValueError(f'{field_name} must not contain duplicates')
    return names


@dataclass(frozen=True, slots=True)
class DualArmJointContract:
    """서로 겹치지 않는 필수 왼팔 및 오른팔 피드백 관절입니다.

    Cleany 기본값은 양쪽의 팔 관절 다섯 개와 읽기 전용 그리퍼 피드백을 포함하며
    완전한 RobotState의 회전 관절은 총 12개입니다. 추가 관절도 받아 보존합니다.
    수락하는 모든 표본에는 설정된 전체 상태가 있어야 하므로 한쪽의 일부 정보만으로
    피드백 FK를 계산할 수 없습니다.
    """

    left_joint_names: tuple[str, ...] = LEFT_FEEDBACK_JOINT_NAMES
    right_joint_names: tuple[str, ...] = RIGHT_FEEDBACK_JOINT_NAMES

    def __post_init__(self) -> None:
        left = _joint_name_tuple(
            self.left_joint_names,
            field_name='left_joint_names',
        )
        right = _joint_name_tuple(
            self.right_joint_names,
            field_name='right_joint_names',
        )
        overlap = set(left).intersection(right)
        if overlap:
            raise ValueError(
                'left_joint_names and right_joint_names must be disjoint: '
                f'{sorted(overlap)!r}'
            )
        object.__setattr__(self, 'left_joint_names', left)
        object.__setattr__(self, 'right_joint_names', right)

    @property
    def required_joint_names(self) -> tuple[str, ...]:
        return self.left_joint_names + self.right_joint_names

    def missing_joint_names(
        self,
        sample: TimedJointSample,
    ) -> tuple[str, ...]:
        if not isinstance(sample, TimedJointSample):
            raise ValueError('sample must be a TimedJointSample')
        present = set(sample.joint_names)
        return tuple(
            name
            for name in self.required_joint_names
            if name not in present
        )

    def validate(self, sample: TimedJointSample) -> None:
        missing = self.missing_joint_names(sample)
        if missing:
            raise IncompleteDualArmFeedback(
                'joint feedback is missing required dual-arm joints: '
                f'{list(missing)!r}'
            )


DEFAULT_DUAL_ARM_JOINT_CONTRACT = DualArmJointContract()


class IncompleteDualArmFeedback(ValueError):
    """시각이 있는 피드백 표본에 필수 팔 관절이 하나 이상 빠져 있습니다."""


class BufferInsertStatus(str, Enum):
    """관절 피드백 표본이 버퍼에 미친 영향을 나타냅니다."""

    ACCEPTED = 'accepted'
    CLOCK_RESET = 'clock_reset'
    DUPLICATE = 'duplicate'
    OUT_OF_ORDER = 'out_of_order'
    INCOMPLETE = 'incomplete'


@dataclass(frozen=True, slots=True)
class BufferInsertResult:
    """스트림 오류를 예외로 던지지 않는 피드백 입력 시도의 결과입니다."""

    status: BufferInsertStatus
    stamp_ns: int
    missing_joint_names: tuple[str, ...] = ()
    discarded_sample_count: int = 0

    @property
    def accepted(self) -> bool:
        return self.status in {
            BufferInsertStatus.ACCEPTED,
            BufferInsertStatus.CLOCK_RESET,
        }


class InterpolationFailure(str, Enum):
    """영상을 관절 피드백과 짝지을 수 없는 명시적인 이유입니다."""

    EMPTY_BUFFER = 'empty_buffer'
    MISSING_BEFORE = 'missing_before'
    MISSING_AFTER = 'missing_after'
    STALE_BEFORE = 'stale_before'
    STALE_AFTER = 'stale_after'
    INCOMPATIBLE_JOINT_SETS = 'incompatible_joint_sets'


@dataclass(frozen=True, slots=True)
class InterpolatedJointState:
    """영상의 ROS 시각에 해당하는 피드백 위치와 입력 출처입니다."""

    sample: TimedJointSample
    before_stamp_ns: int
    after_stamp_ns: int
    ratio: float

    def __post_init__(self) -> None:
        if not isinstance(self.sample, TimedJointSample):
            raise ValueError('sample must be a TimedJointSample')
        if (
            isinstance(self.before_stamp_ns, bool)
            or not isinstance(self.before_stamp_ns, int)
            or self.before_stamp_ns < 0
        ):
            raise ValueError(
                'before_stamp_ns must be a non-negative integer'
            )
        if (
            isinstance(self.after_stamp_ns, bool)
            or not isinstance(self.after_stamp_ns, int)
            or self.after_stamp_ns < 0
        ):
            raise ValueError(
                'after_stamp_ns must be a non-negative integer'
            )
        if self.before_stamp_ns >= self.after_stamp_ns:
            raise ValueError(
                'interpolation requires two ordered source timestamps'
            )
        if not (
            self.before_stamp_ns
            <= self.sample.stamp_ns
            <= self.after_stamp_ns
        ):
            raise ValueError(
                'interpolated sample stamp must be bracketed by sources'
            )
        try:
            ratio = float(self.ratio)
        except (TypeError, ValueError) as error:
            raise ValueError('ratio must be numeric') from error
        if not math.isfinite(ratio) or not 0.0 <= ratio <= 1.0:
            raise ValueError('ratio must be finite and in [0, 1]')
        expected_ratio = (
            (self.sample.stamp_ns - self.before_stamp_ns)
            / (self.after_stamp_ns - self.before_stamp_ns)
        )
        if not math.isclose(
            ratio,
            expected_ratio,
            rel_tol=0.0,
            abs_tol=1.0e-15,
        ):
            raise ValueError(
                'ratio does not match the source and image timestamps'
            )
        object.__setattr__(self, 'ratio', ratio)

    @property
    def image_stamp_ns(self) -> int:
        return self.sample.stamp_ns


@dataclass(frozen=True, slots=True)
class JointInterpolationResult:
    """영상 시각 하나에 대한 성공 결과 또는 명시적인 표본 거부 이유입니다."""

    interpolation: InterpolatedJointState | None = None
    failure: InterpolationFailure | None = None

    def __post_init__(self) -> None:
        has_interpolation = self.interpolation is not None
        has_failure = self.failure is not None
        if has_interpolation == has_failure:
            raise ValueError(
                'exactly one of interpolation or failure is required'
            )
        if has_interpolation and not isinstance(
            self.interpolation,
            InterpolatedJointState,
        ):
            raise ValueError(
                'interpolation must be an InterpolatedJointState'
            )
        if has_failure:
            try:
                failure = InterpolationFailure(self.failure)
            except ValueError as error:
                raise ValueError('failure is not recognized') from error
            object.__setattr__(self, 'failure', failure)

    @property
    def success(self) -> bool:
        return self.interpolation is not None


class JointStateRingBuffer:
    """완전한 양팔 피드백을 시각순으로 보관하는 크기 제한 버퍼입니다.

    작은 시각 역행은 순서 오류로 거부합니다. clock_reset_threshold_ns 이상
    역행하면 ROS 시계 초기화로 보고 이전 epoch를 버린 뒤 새 표본을 받습니다.
    관절 메시지만으로 전송 지연과 시뮬레이션 시계 초기화를 구분할 수 없으므로
    이 임계값을 명시적으로 지정합니다.
    """

    def __init__(
        self,
        *,
        capacity: int,
        max_sample_distance_ns: int,
        clock_reset_threshold_ns: int,
        joint_contract: DualArmJointContract = (
            DEFAULT_DUAL_ARM_JOINT_CONTRACT
        ),
    ) -> None:
        self._capacity = _positive_integer(capacity, field_name='capacity')
        self._max_sample_distance_ns = _positive_integer(
            max_sample_distance_ns,
            field_name='max_sample_distance_ns',
        )
        self._clock_reset_threshold_ns = _positive_integer(
            clock_reset_threshold_ns,
            field_name='clock_reset_threshold_ns',
        )
        if not isinstance(joint_contract, DualArmJointContract):
            raise ValueError(
                'joint_contract must be a DualArmJointContract'
            )
        self._joint_contract = joint_contract
        self._samples: deque[TimedJointSample] = deque(
            maxlen=self._capacity
        )

    @property
    def samples(self) -> tuple[TimedJointSample, ...]:
        return tuple(self._samples)

    @property
    def joint_contract(self) -> DualArmJointContract:
        return self._joint_contract

    def clear(self) -> None:
        self._samples.clear()

    def add(self, sample: TimedJointSample) -> BufferInsertResult:
        """피드백을 검사해 입력하며 스트림 오류는 예외로 던지지 않습니다."""

        if not isinstance(sample, TimedJointSample):
            raise ValueError('sample must be a TimedJointSample')
        missing = self._joint_contract.missing_joint_names(sample)
        if missing:
            return BufferInsertResult(
                status=BufferInsertStatus.INCOMPLETE,
                stamp_ns=sample.stamp_ns,
                missing_joint_names=missing,
            )

        if not self._samples:
            self._samples.append(sample)
            return BufferInsertResult(
                status=BufferInsertStatus.ACCEPTED,
                stamp_ns=sample.stamp_ns,
            )

        latest_stamp = self._samples[-1].stamp_ns
        if any(
            buffered.stamp_ns == sample.stamp_ns
            for buffered in self._samples
        ):
            return BufferInsertResult(
                status=BufferInsertStatus.DUPLICATE,
                stamp_ns=sample.stamp_ns,
            )
        if sample.stamp_ns < latest_stamp:
            regression_ns = latest_stamp - sample.stamp_ns
            if regression_ns < self._clock_reset_threshold_ns:
                return BufferInsertResult(
                    status=BufferInsertStatus.OUT_OF_ORDER,
                    stamp_ns=sample.stamp_ns,
                )
            discarded_count = len(self._samples)
            self._samples.clear()
            self._samples.append(sample)
            return BufferInsertResult(
                status=BufferInsertStatus.CLOCK_RESET,
                stamp_ns=sample.stamp_ns,
                discarded_sample_count=discarded_count,
            )

        self._samples.append(sample)
        return BufferInsertResult(
            status=BufferInsertStatus.ACCEPTED,
            stamp_ns=sample.stamp_ns,
        )

    def interpolate(self, image_stamp_ns: int) -> JointInterpolationResult:
        """영상 ROS 시각의 양쪽 표본이 모두 있으면 보간합니다."""

        if (
            isinstance(image_stamp_ns, bool)
            or not isinstance(image_stamp_ns, int)
            or image_stamp_ns < 0
        ):
            raise ValueError(
                'image_stamp_ns must be a non-negative integer'
            )
        if not self._samples:
            return JointInterpolationResult(
                failure=InterpolationFailure.EMPTY_BUFFER
            )

        samples = tuple(self._samples)
        stamps = tuple(sample.stamp_ns for sample in samples)
        after_index = bisect_left(stamps, image_stamp_ns)

        if after_index == len(samples):
            return JointInterpolationResult(
                failure=InterpolationFailure.MISSING_AFTER
            )
        if after_index == 0:
            if samples[0].stamp_ns != image_stamp_ns:
                return JointInterpolationResult(
                    failure=InterpolationFailure.MISSING_BEFORE
                )
            if len(samples) < 2:
                return JointInterpolationResult(
                    failure=InterpolationFailure.MISSING_AFTER
                )
            before = samples[0]
            after = samples[1]
        else:
            before = samples[after_index - 1]
            after = samples[after_index]

        before_distance_ns = image_stamp_ns - before.stamp_ns
        after_distance_ns = after.stamp_ns - image_stamp_ns
        if before_distance_ns > self._max_sample_distance_ns:
            return JointInterpolationResult(
                failure=InterpolationFailure.STALE_BEFORE
            )
        if after_distance_ns > self._max_sample_distance_ns:
            return JointInterpolationResult(
                failure=InterpolationFailure.STALE_AFTER
            )
        if set(before.joint_names) != set(after.joint_names):
            return JointInterpolationResult(
                failure=InterpolationFailure.INCOMPATIBLE_JOINT_SETS
            )

        ratio = before_distance_ns / (
            after.stamp_ns - before.stamp_ns
        )
        after_positions = dict(
            zip(after.joint_names, after.positions_rad, strict=True)
        )
        positions = tuple(
            before_position
            + ratio
            * (after_positions[name] - before_position)
            for name, before_position in zip(
                before.joint_names,
                before.positions_rad,
                strict=True,
            )
        )

        velocities = None
        if (
            before.velocities_rad_s is not None
            and after.velocities_rad_s is not None
        ):
            after_velocities = dict(
                zip(
                    after.joint_names,
                    after.velocities_rad_s,
                    strict=True,
                )
            )
            velocities = tuple(
                before_velocity
                + ratio
                * (after_velocities[name] - before_velocity)
                for name, before_velocity in zip(
                    before.joint_names,
                    before.velocities_rad_s,
                    strict=True,
                )
            )

        sample = TimedJointSample(
            stamp_ns=image_stamp_ns,
            joint_names=before.joint_names,
            positions_rad=positions,
            velocities_rad_s=velocities,
        )
        return JointInterpolationResult(
            interpolation=InterpolatedJointState(
                sample=sample,
                before_stamp_ns=before.stamp_ns,
                after_stamp_ns=after.stamp_ns,
                ratio=ratio,
            )
        )
