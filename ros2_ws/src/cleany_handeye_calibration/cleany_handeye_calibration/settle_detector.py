"""Pure ROS-stamp settle gate for left-arm calibration feedback."""

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
    """Why a continuous settle interval was cleared."""

    ACTION_NOT_SUCCESSFUL = 'action_not_successful'
    CLOCK_RESET = 'clock_reset'
    INCOMPLETE_FEEDBACK = 'incomplete_feedback'
    VELOCITY_MISSING = 'velocity_missing'
    POSITION_VIOLATION = 'position_violation'
    VELOCITY_VIOLATION = 'velocity_violation'


class SettleWaitStatus(str, Enum):
    """Bounded wait state around the ROS-stamp detector."""

    NOT_ARMED = 'not_armed'
    WAITING = 'waiting'
    SETTLED = 'settled'
    TIMED_OUT = 'timed_out'


class JointSettleDetector:
    """Require continuous left-joint position and velocity convergence.

    ROS message stamps drive the one-second stability interval.  No ROS node,
    wall clock, or action client is used.  A successful action only arms the
    detector; it never marks the target settled without subsequent feedback.
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
        """Clear the target and all timing state."""

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
        """Arm a fresh interval only after successful motion execution.

        The return value is always ``False`` because action success by itself
        cannot satisfy a feedback settle contract.
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
        """Ingest one feedback sample and report current settle state."""

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
    """Apply the required wall-clock settle-stage timeout without blocking.

    Joint stability remains exclusively ROS-stamp based in
    :class:`JointSettleDetector`.  This wrapper uses a monotonic clock only to
    stop an unproductive WAIT_SETTLED stage, including when simulation time is
    paused.
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
        """Check the stage deadline even when no feedback arrives."""

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
