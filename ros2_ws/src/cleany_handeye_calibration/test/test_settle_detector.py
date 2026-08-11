import math

import pytest

from cleany_handeye_calibration.joint_state_sync import (
    LEFT_ARM_JOINT_NAMES,
)
from cleany_handeye_calibration.models import JointPose, TimedJointSample
from cleany_handeye_calibration.motion_config import (
    MujocoMotionConfig,
    StageTimeouts,
)
from cleany_handeye_calibration.settle_detector import (
    JointSettleDetector,
    MonotonicSettleMonitor,
    SettleResetReason,
    SettleWaitStatus,
)


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def monotonic(self):
        return self.value


def _config() -> MujocoMotionConfig:
    return MujocoMotionConfig(
        current_state_max_age_sec=0.2,
        right_park_position_tolerance_rad=0.02,
        stage_timeouts=StageTimeouts(
            ik_sec=0.5,
            state_validity_sec=0.5,
            plan_sec=2.0,
            execute_sec=3.0,
            cancel_sec=0.5,
            settle_sec=2.0,
        ),
    )


def _target() -> JointPose:
    return JointPose(
        joint_names=LEFT_ARM_JOINT_NAMES,
        positions_rad=(0.1, 0.2, 0.3, 0.4, 0.5),
    )


def _sample(
    stamp_ns: int,
    *,
    position_error: float = 0.0,
    velocity: float = 0.0,
    omit_last_joint: bool = False,
    include_velocity: bool = True,
) -> TimedJointSample:
    names = LEFT_ARM_JOINT_NAMES[
        :-1 if omit_last_joint else len(LEFT_ARM_JOINT_NAMES)
    ]
    target = _target()
    target_positions = dict(
        zip(target.joint_names, target.positions_rad, strict=True)
    )
    return TimedJointSample(
        stamp_ns=stamp_ns,
        joint_names=names,
        positions_rad=tuple(
            target_positions[name] + position_error for name in names
        ),
        velocities_rad_s=(
            tuple(velocity for _ in names) if include_velocity else None
        ),
    )


def test_action_success_alone_does_not_settle():
    detector = JointSettleDetector(_config())

    assert not detector.begin(_target(), action_succeeded=True)
    assert detector.armed
    assert not detector.settled


def test_all_left_joints_must_hold_thresholds_for_one_ros_second():
    detector = JointSettleDetector(_config())
    detector.begin(_target(), action_succeeded=True)

    assert not detector.update(
        _sample(
            10_000_000_000,
            position_error=0.015,
            velocity=0.01,
        )
    )
    assert not detector.update(
        _sample(
            10_999_999_999,
            position_error=-0.015,
            velocity=-0.01,
        )
    )
    assert detector.update(
        _sample(
            11_000_000_000,
            position_error=0.015,
            velocity=0.01,
        )
    )
    assert detector.settled


@pytest.mark.parametrize(
    ('position_error', 'velocity', 'reason'),
    [
        (0.015001, 0.0, SettleResetReason.POSITION_VIOLATION),
        (0.0, 0.010001, SettleResetReason.VELOCITY_VIOLATION),
    ],
)
def test_any_joint_threshold_violation_resets_continuous_interval(
    position_error,
    velocity,
    reason,
):
    detector = JointSettleDetector(_config())
    detector.begin(_target(), action_succeeded=True)
    assert not detector.update(_sample(1_000_000_000))
    assert not detector.update(_sample(1_500_000_000))

    assert not detector.update(
        _sample(
            1_600_000_000,
            position_error=position_error,
            velocity=velocity,
        )
    )
    assert detector.last_reset_reason is reason
    assert detector.stable_since_stamp_ns is None

    assert not detector.update(_sample(1_700_000_000))
    assert not detector.update(_sample(2_600_000_000))
    assert detector.update(_sample(2_700_000_000))


def test_smallest_representable_position_above_bound_is_a_violation():
    detector = JointSettleDetector(_config())
    detector.begin(_target(), action_succeeded=True)
    target_positions = _target().positions_rad
    position_tolerance = _config().settle_position_tolerance_rad
    outside_upper_bound = math.nextafter(
        target_positions[0] + position_tolerance,
        math.inf,
    )
    sample = TimedJointSample(
        stamp_ns=1_000_000_000,
        joint_names=LEFT_ARM_JOINT_NAMES,
        positions_rad=(outside_upper_bound,) + target_positions[1:],
        velocities_rad_s=tuple(0.0 for _ in LEFT_ARM_JOINT_NAMES),
    )

    assert not detector.update(sample)
    assert detector.last_reset_reason is (
        SettleResetReason.POSITION_VIOLATION
    )
    assert detector.stable_since_stamp_ns is None


def test_clock_regression_starts_a_new_ros_time_interval():
    detector = JointSettleDetector(_config())
    detector.begin(_target(), action_succeeded=True)
    detector.update(_sample(5_000_000_000))
    detector.update(_sample(5_900_000_000))

    assert not detector.update(_sample(100_000_000))
    assert detector.last_reset_reason is SettleResetReason.CLOCK_RESET
    assert detector.stable_since_stamp_ns == 100_000_000
    assert not detector.update(_sample(1_099_999_999))
    assert detector.update(_sample(1_100_000_000))


def test_missing_joint_or_velocity_is_a_settle_violation():
    detector = JointSettleDetector(_config())
    detector.begin(_target(), action_succeeded=True)
    detector.update(_sample(1_000_000_000))

    assert not detector.update(
        _sample(1_500_000_000, omit_last_joint=True)
    )
    assert detector.last_reset_reason is (
        SettleResetReason.INCOMPLETE_FEEDBACK
    )

    detector.update(_sample(1_600_000_000))
    assert not detector.update(
        _sample(1_700_000_000, include_velocity=False)
    )
    assert detector.last_reset_reason is SettleResetReason.VELOCITY_MISSING


def test_failed_action_never_arms_detector():
    detector = JointSettleDetector(_config())

    detector.begin(_target(), action_succeeded=False)

    assert not detector.armed
    assert not detector.update(_sample(1_000_000_000))
    assert detector.last_reset_reason is (
        SettleResetReason.ACTION_NOT_SUCCESSFUL
    )


def test_settle_stage_timeout_never_authorizes_sample_acquisition():
    clock = FakeClock()
    monitor = MonotonicSettleMonitor(
        _config(),
        monotonic=clock.monotonic,
    )
    assert monitor.begin(
        _target(),
        action_succeeded=True,
    ) is SettleWaitStatus.WAITING
    assert not monitor.ready_for_sample

    clock.value = 1.999999
    assert monitor.poll() is SettleWaitStatus.WAITING
    clock.value = 2.0
    assert monitor.poll() is SettleWaitStatus.TIMED_OUT
    assert not monitor.ready_for_sample
    assert monitor.update(_sample(3_000_000_000)) is (
        SettleWaitStatus.TIMED_OUT
    )
