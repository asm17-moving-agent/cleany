from dataclasses import FrozenInstanceError

import pytest

from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    RIGHT_ARM_JOINT_NAMES,
)
from cleany_handeye_calibration.models import TimedJointSample
from cleany_handeye_calibration.motion_config import (
    CalibrationScope,
    MujocoMotionConfig,
    StageTimeouts,
    StartupFailure,
    StartupPreconditionError,
    validate_dual_arm_current_state,
)


def _timeouts() -> StageTimeouts:
    return StageTimeouts(
        ik_sec=0.5,
        state_validity_sec=0.5,
        plan_sec=2.0,
        execute_sec=3.0,
        cancel_sec=0.5,
        settle_sec=2.0,
    )


def _config() -> MujocoMotionConfig:
    return MujocoMotionConfig(
        current_state_max_age_sec=0.2,
        right_park_position_tolerance_rad=0.02,
        stage_timeouts=_timeouts(),
    )


def _feedback(
    *,
    stamp_ns: int = 1_000_000_000,
    omit: str | None = None,
    right_position: float = 0.0,
    right_velocity: float = 0.0,
    include_velocity: bool = True,
) -> TimedJointSample:
    names = tuple(
        name
        for name in DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
        if name != omit
    )
    positions = tuple(
        right_position if name in RIGHT_ARM_JOINT_NAMES else 0.1
        for name in names
    )
    velocities = tuple(
        right_velocity if name in RIGHT_ARM_JOINT_NAMES else 0.0
        for name in names
    )
    return TimedJointSample(
        stamp_ns=stamp_ns,
        joint_names=names,
        positions_rad=positions,
        velocities_rad_s=velocities if include_velocity else None,
    )


def test_mujoco_config_requires_explicit_preflight_and_stage_values():
    with pytest.raises(TypeError):
        MujocoMotionConfig()
    with pytest.raises(TypeError):
        StageTimeouts()

    config = _config()

    assert config.max_velocity_scaling_factor == 0.10
    assert config.max_acceleration_scaling_factor == 0.10
    assert config.controller_path_tolerance_rad == 0.05
    assert config.controller_goal_tolerance_rad == 0.01
    assert config.settle_position_tolerance_rad == 0.015
    assert config.settle_velocity_tolerance_rad_s == 0.01
    assert config.settle_duration_sec == 1.0
    with pytest.raises(FrozenInstanceError):
        config.max_velocity_scaling_factor = 1.0


def test_post_execution_settle_tolerance_is_independent_of_goal_region():
    config = MujocoMotionConfig(
        current_state_max_age_sec=0.2,
        right_park_position_tolerance_rad=0.02,
        stage_timeouts=_timeouts(),
        controller_path_tolerance_rad=0.05,
        controller_goal_tolerance_rad=0.01,
        settle_position_tolerance_rad=0.015,
    )

    assert config.settle_position_tolerance_rad == 0.015
    with pytest.raises(ValueError, match='controller path tolerance'):
        MujocoMotionConfig(
            current_state_max_age_sec=0.2,
            right_park_position_tolerance_rad=0.02,
            stage_timeouts=_timeouts(),
            controller_path_tolerance_rad=0.05,
            controller_goal_tolerance_rad=0.01,
            settle_position_tolerance_rad=0.050001,
        )


@pytest.mark.parametrize(
    ('group', 'tip'),
    [
        ('right_arm', 'left_gripper_frame'),
        ('left_arm', 'right_gripper_frame'),
        ('right_arm', 'right_gripper_frame'),
    ],
)
def test_calibration_scope_allowlist_is_exact(group, tip):
    with pytest.raises(ValueError, match='exactly'):
        CalibrationScope(group, tip)

    scope = CalibrationScope('left_arm', 'left_gripper_frame')
    assert scope.planning_group == 'left_arm'
    assert scope.tip_link == 'left_gripper_frame'


def test_startup_accepts_only_complete_fresh_stationary_park_state():
    state = validate_dual_arm_current_state(
        _feedback(),
        now_stamp_ns=1_200_000_000,
        config=_config(),
    )

    assert state.age_ns == 200_000_000
    assert set(state.sample.joint_names) == set(
        DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
    )


def test_startup_rejects_missing_gripper_as_incomplete_full_state():
    with pytest.raises(StartupPreconditionError) as caught:
        validate_dual_arm_current_state(
            _feedback(omit='right_gripper_joint'),
            now_stamp_ns=1_000_000_000,
            config=_config(),
        )

    assert caught.value.failure is (
        StartupFailure.INCOMPLETE_DUAL_ARM_STATE
    )
    assert caught.value.joint_names == ('right_gripper_joint',)


def test_startup_rejects_stale_or_future_ros_state_stamp():
    with pytest.raises(StartupPreconditionError) as stale:
        validate_dual_arm_current_state(
            _feedback(stamp_ns=1_000_000_000),
            now_stamp_ns=1_200_000_001,
            config=_config(),
        )
    assert stale.value.failure is StartupFailure.STALE_CURRENT_STATE

    with pytest.raises(StartupPreconditionError) as future:
        validate_dual_arm_current_state(
            _feedback(stamp_ns=1_000_000_001),
            now_stamp_ns=1_000_000_000,
            config=_config(),
        )
    assert future.value.failure is StartupFailure.FUTURE_STATE_STAMP


def test_startup_rejects_right_arm_off_park_or_moving():
    with pytest.raises(StartupPreconditionError) as off_park:
        validate_dual_arm_current_state(
            _feedback(right_position=0.020001),
            now_stamp_ns=1_000_000_000,
            config=_config(),
        )
    assert off_park.value.failure is StartupFailure.RIGHT_ARM_NOT_PARKED

    with pytest.raises(StartupPreconditionError) as moving:
        validate_dual_arm_current_state(
            _feedback(right_velocity=0.010001),
            now_stamp_ns=1_000_000_000,
            config=_config(),
        )
    assert moving.value.failure is StartupFailure.RIGHT_ARM_MOVING

    with pytest.raises(StartupPreconditionError) as missing_velocity:
        validate_dual_arm_current_state(
            _feedback(include_velocity=False),
            now_stamp_ns=1_000_000_000,
            config=_config(),
        )
    assert missing_velocity.value.failure is (
        StartupFailure.RIGHT_ARM_VELOCITY_MISSING
    )
