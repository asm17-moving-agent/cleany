"""왼팔 보정 피드백의 ROS stamp 기반 순수 정착 판정입니다."""

from __future__ import annotations

from enum import Enum
import math
import time
from typing import Callable, Sequence

from cleany_handeye_calibration.joint_state_sync import (
    LEFT_ARM_JOINT_NAMES,
)
from cleany_handeye_calibration.models import JointPose, TimedJointSample
from cleany_handeye_calibration.motion_config import (
    NANOSECONDS_PER_SECOND,
    MujocoMotionConfig,
)


class SettleResetReason(str, Enum):
    """연속 정착 구간이 초기화된 이유입니다."""

    ACTION_NOT_SUCCESSFUL = 'action_not_successful'
    CLOCK_RESET = 'clock_reset'
    INCOMPLETE_FEEDBACK = 'incomplete_feedback'
    VELOCITY_MISSING = 'velocity_missing'
    POSITION_VIOLATION = 'position_violation'
    VELOCITY_VIOLATION = 'velocity_violation'


class SettleWaitStatus(str, Enum):
    """ROS stamp 판정기를 감싸는 시간 제한 대기 상태입니다."""

    NOT_ARMED = 'not_armed'
    WAITING = 'waiting'
    SETTLED = 'settled'
    TIMED_OUT = 'timed_out'


class JointSettleDetector:
    """왼팔 관절의 위치와 속도가 연속적으로 수렴해야 정착을 인정합니다.

    ROS 메시지 stamp로 1초 안정 구간을 계산합니다. ROS 노드, 경과 시간 시계나
    action client를 사용하지 않습니다. action 성공은 판정기를 준비시킬 뿐이며
    후속 피드백 없이 목표가 정착했다고 표시하지 않습니다.
    """

    def __init__(
        self,
        config: MujocoMotionConfig,
        *,
        joint_names: Sequence[str] = LEFT_ARM_JOINT_NAMES,
    ) -> None:
        if not isinstance(config, MujocoMotionConfig):
            raise ValueError('config must be MujocoMotionConfig')
        names = tuple(joint_names)
        if names != LEFT_ARM_JOINT_NAMES:
            raise ValueError(
                'settle detector joint_names must be exactly the five '
                'left-arm joints in canonical order'
            )
        self._config = config
        self._joint_names = names
        self._required_duration_ns = round(
            config.settle_duration_sec * NANOSECONDS_PER_SECOND
        )
        self.reset()

    @property
    def settled(self) -> bool:
        return self._settled

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def stable_since_stamp_ns(self) -> int | None:
        return self._stable_since_stamp_ns

    @property
    def last_reset_reason(self) -> SettleResetReason | None:
        return self._last_reset_reason

    def reset(self) -> None:
        """목표와 모든 시간 상태를 초기화합니다."""

        self._target_positions: dict[str, float] | None = None
        self._armed = False
        self._settled = False
        self._stable_since_stamp_ns: int | None = None
        self._last_stamp_ns: int | None = None
        self._last_reset_reason: SettleResetReason | None = None

    def begin(
        self,
        target: JointPose,
        *,
        action_succeeded: bool,
    ) -> bool:
        """이동 실행에 성공한 뒤에만 새 안정 구간을 준비합니다.

        action 성공만으로 피드백 정착 계약을 만족할 수 없으므로 항상 False를 반환합니다.
        """

        if not isinstance(target, JointPose):
            raise ValueError('target must be a JointPose')
        if not isinstance(action_succeeded, bool):
            raise ValueError('action_succeeded must be a bool')
        if set(target.joint_names) != set(self._joint_names):
            raise ValueError(
                'target must contain exactly the five left-arm joints'
            )

        positions = dict(
            zip(target.joint_names, target.positions_rad, strict=True)
        )
        self.reset()
        self._target_positions = {
            name: positions[name] for name in self._joint_names
        }
        self._armed = action_succeeded
        if not action_succeeded:
            self._last_reset_reason = (
                SettleResetReason.ACTION_NOT_SUCCESSFUL
            )
        return False

    def update(self, sample: TimedJointSample) -> bool:
        """피드백 표본 하나를 반영하고 현재 정착 상태를 반환합니다."""

        if not isinstance(sample, TimedJointSample):
            raise ValueError('sample must be a TimedJointSample')
        if not self._armed or self._target_positions is None:
            return False

        if (
            self._last_stamp_ns is not None
            and sample.stamp_ns < self._last_stamp_ns
        ):
            self._clear_interval(SettleResetReason.CLOCK_RESET)
        self._last_stamp_ns = sample.stamp_ns

        present = set(sample.joint_names)
        if not set(self._joint_names) <= present:
            self._clear_interval(
                SettleResetReason.INCOMPLETE_FEEDBACK
            )
            return False
        if sample.velocities_rad_s is None:
            self._clear_interval(SettleResetReason.VELOCITY_MISSING)
            return False

        positions = dict(
            zip(sample.joint_names, sample.positions_rad, strict=True)
        )
        velocities = dict(
            zip(
                sample.joint_names,
                sample.velocities_rad_s,
                strict=True,
            )
        )
        position_tolerance = (
            self._config.settle_position_tolerance_rad
        )
        if any(
            positions[name]
            < self._target_positions[name] - position_tolerance
            or positions[name]
            > self._target_positions[name] + position_tolerance
            for name in self._joint_names
        ):
            self._clear_interval(SettleResetReason.POSITION_VIOLATION)
            return False
        if any(
            self._exceeds_threshold(
                abs(velocities[name]),
                self._config.settle_velocity_tolerance_rad_s,
            )
            for name in self._joint_names
        ):
            self._clear_interval(SettleResetReason.VELOCITY_VIOLATION)
            return False

        if self._stable_since_stamp_ns is None:
            self._stable_since_stamp_ns = sample.stamp_ns
            self._settled = False
            return False
        self._settled = (
            sample.stamp_ns - self._stable_since_stamp_ns
            >= self._required_duration_ns
        )
        return self._settled

    def _clear_interval(self, reason: SettleResetReason) -> None:
        self._stable_since_stamp_ns = None
        self._settled = False
        self._last_reset_reason = reason

    @staticmethod
    def _exceeds_threshold(value: float, threshold: float) -> bool:
        return value > threshold


class MonotonicSettleMonitor:
    """블로킹 없이 정착 단계에 필요한 경과 시간 제한을 적용합니다.

    관절 안정성은 JointSettleDetector에서 오직 ROS stamp로 판단합니다.
    이 래퍼는 시뮬레이션 시각이 멈춘 경우를 포함해 진전 없는 WAIT_SETTLED 단계를
    종료하는 데만 단조 시계를 사용합니다.
    """

    def __init__(
        self,
        config: MujocoMotionConfig,
        *,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if not isinstance(config, MujocoMotionConfig):
            raise ValueError('config must be MujocoMotionConfig')
        if not callable(monotonic):
            raise ValueError('monotonic must be callable')
        self._config = config
        self._monotonic = monotonic
        self._detector = JointSettleDetector(config)
        self._started_at: float | None = None
        self._status = SettleWaitStatus.NOT_ARMED

    @property
    def detector(self) -> JointSettleDetector:
        return self._detector

    @property
    def status(self) -> SettleWaitStatus:
        return self._status

    @property
    def ready_for_sample(self) -> bool:
        return self._status is SettleWaitStatus.SETTLED

    def begin(
        self,
        target: JointPose,
        *,
        action_succeeded: bool,
    ) -> SettleWaitStatus:
        self._detector.begin(
            target,
            action_succeeded=action_succeeded,
        )
        if not action_succeeded:
            self._started_at = None
            self._status = SettleWaitStatus.NOT_ARMED
            return self._status
        self._started_at = self._now()
        self._status = SettleWaitStatus.WAITING
        return self._status

    def update(self, sample: TimedJointSample) -> SettleWaitStatus:
        if self._status is not SettleWaitStatus.WAITING:
            return self._status
        if self._deadline_expired():
            self._status = SettleWaitStatus.TIMED_OUT
            return self._status
        if self._detector.update(sample):
            self._status = SettleWaitStatus.SETTLED
        return self._status

    def poll(self) -> SettleWaitStatus:
        """피드백이 도착하지 않아도 단계 마감 시각을 확인합니다."""

        if (
            self._status is SettleWaitStatus.WAITING
            and self._deadline_expired()
        ):
            self._status = SettleWaitStatus.TIMED_OUT
        return self._status

    def _deadline_expired(self) -> bool:
        assert self._started_at is not None
        elapsed = self._now() - self._started_at
        if elapsed < 0.0:
            raise RuntimeError('monotonic clock regressed during settle wait')
        return elapsed >= self._config.stage_timeouts.settle_sec

    def _now(self) -> float:
        try:
            now = float(self._monotonic())
        except (TypeError, ValueError) as error:
            raise RuntimeError(
                'monotonic clock returned a non-numeric value'
            ) from error
        if not math.isfinite(now):
            raise RuntimeError(
                'monotonic clock returned a non-finite value'
            )
        return now


SettleDetector = JointSettleDetector
