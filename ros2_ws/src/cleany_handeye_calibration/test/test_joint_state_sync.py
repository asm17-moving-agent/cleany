from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from cleany_handeye_calibration.joint_state_sync import (
    BufferInsertStatus,
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    DualArmJointContract,
    InterpolatedJointState,
    InterpolationFailure,
    JointStateRingBuffer,
    LEFT_FEEDBACK_JOINT_NAMES,
    RIGHT_ARM_JOINT_NAMES,
    RIGHT_FEEDBACK_JOINT_NAMES,
)
from cleany_handeye_calibration.models import TimedJointSample


FULL_JOINTS = LEFT_FEEDBACK_JOINT_NAMES + RIGHT_FEEDBACK_JOINT_NAMES


def _sample(
    stamp_ns: int,
    *,
    joint_names: tuple[str, ...] = FULL_JOINTS,
    offset: float = 0.0,
    velocities: bool = True,
) -> TimedJointSample:
    return TimedJointSample(
        stamp_ns=stamp_ns,
        joint_names=joint_names,
        positions_rad=tuple(
            offset + index for index in range(len(joint_names))
        ),
        velocities_rad_s=(
            tuple(
                offset + 0.1 * index
                for index in range(len(joint_names))
            )
            if velocities
            else None
        ),
    )


def _buffer(
    *,
    capacity: int = 4,
    max_sample_distance_ns: int = 100,
    clock_reset_threshold_ns: int = 500,
) -> JointStateRingBuffer:
    return JointStateRingBuffer(
        capacity=capacity,
        max_sample_distance_ns=max_sample_distance_ns,
        clock_reset_threshold_ns=clock_reset_threshold_ns,
    )


def test_default_contract_requires_complete_disjoint_dual_arm() -> None:
    contract = DEFAULT_DUAL_ARM_JOINT_CONTRACT

    assert contract.left_joint_names == LEFT_FEEDBACK_JOINT_NAMES
    assert contract.right_joint_names == RIGHT_FEEDBACK_JOINT_NAMES
    assert len(contract.required_joint_names) == 12

    with pytest.raises(ValueError, match='disjoint'):
        DualArmJointContract(
            left_joint_names=('shared_joint',),
            right_joint_names=('shared_joint',),
        )


def test_buffer_is_bounded_and_remains_time_ordered() -> None:
    buffer = _buffer(capacity=2)

    assert buffer.add(_sample(10)).accepted
    assert buffer.add(_sample(20)).accepted
    assert buffer.add(_sample(30)).accepted

    assert tuple(item.stamp_ns for item in buffer.samples) == (20, 30)
    assert (
        buffer.interpolate(15).failure
        is InterpolationFailure.MISSING_BEFORE
    )


def test_incomplete_feedback_is_rejected_without_mutating_buffer() -> None:
    buffer = _buffer()
    complete = _sample(10)
    assert buffer.add(complete).accepted

    partial_names = tuple(
        name for name in FULL_JOINTS if name != RIGHT_ARM_JOINT_NAMES[-1]
    )
    result = buffer.add(_sample(20, joint_names=partial_names))

    assert result.status is BufferInsertStatus.INCOMPLETE
    assert result.missing_joint_names == (RIGHT_ARM_JOINT_NAMES[-1],)
    assert buffer.samples == (complete,)


@pytest.mark.parametrize(
    'missing_gripper',
    ['left_gripper_joint', 'right_gripper_joint'],
)
def test_missing_gripper_feedback_is_also_incomplete(
    missing_gripper: str,
) -> None:
    buffer = _buffer()
    partial_names = tuple(
        name for name in FULL_JOINTS if name != missing_gripper
    )

    result = buffer.add(_sample(100, joint_names=partial_names))

    assert result.status is BufferInsertStatus.INCOMPLETE
    assert result.missing_joint_names == (missing_gripper,)
    assert buffer.samples == ()


def test_interpolation_uses_names_not_message_order_and_keeps_extras() -> None:
    buffer = _buffer()
    state_names = FULL_JOINTS + ('optional_joint',)
    before = _sample(1_000, joint_names=state_names, offset=0.0)
    after_names = tuple(reversed(state_names))
    after_positions_by_name = {
        name: before.positions_rad[index] + 10.0
        for index, name in enumerate(before.joint_names)
    }
    after_velocities_by_name = {
        name: before.velocities_rad_s[index] + 2.0
        for index, name in enumerate(before.joint_names)
    }
    after = TimedJointSample(
        stamp_ns=1_100,
        joint_names=after_names,
        positions_rad=tuple(
            after_positions_by_name[name] for name in after_names
        ),
        velocities_rad_s=tuple(
            after_velocities_by_name[name] for name in after_names
        ),
    )
    assert buffer.add(before).accepted
    assert buffer.add(after).accepted

    result = buffer.interpolate(1_025)

    assert result.success
    interpolation = result.interpolation
    assert interpolation is not None
    assert interpolation.before_stamp_ns == 1_000
    assert interpolation.after_stamp_ns == 1_100
    assert interpolation.image_stamp_ns == 1_025
    assert interpolation.ratio == pytest.approx(0.25)
    assert interpolation.sample.joint_names == state_names
    assert interpolation.sample.joint_names[-1] == 'optional_joint'
    assert interpolation.sample.positions_rad == pytest.approx(
        tuple(value + 2.5 for value in before.positions_rad)
    )
    assert interpolation.sample.velocities_rad_s == pytest.approx(
        tuple(value + 0.5 for value in before.velocities_rad_s)
    )


def test_velocity_is_recorded_only_when_both_sources_provide_it() -> None:
    buffer = _buffer()
    buffer.add(_sample(100, velocities=False))
    buffer.add(_sample(200, offset=1.0))

    result = buffer.interpolate(150)

    assert result.interpolation is not None
    assert result.interpolation.sample.velocities_rad_s is None


@pytest.mark.parametrize(
    ('image_stamp_ns', 'expected_failure'),
    [
        (50, InterpolationFailure.MISSING_BEFORE),
        (250, InterpolationFailure.MISSING_AFTER),
    ],
)
def test_missing_side_rejects_image(
    image_stamp_ns: int,
    expected_failure: InterpolationFailure,
) -> None:
    buffer = _buffer()
    buffer.add(_sample(100))
    buffer.add(_sample(200))

    assert buffer.interpolate(image_stamp_ns).failure is expected_failure


def test_empty_and_single_exact_sample_do_not_claim_interpolation() -> None:
    buffer = _buffer()
    assert (
        buffer.interpolate(100).failure
        is InterpolationFailure.EMPTY_BUFFER
    )
    buffer.add(_sample(100))
    assert (
        buffer.interpolate(100).failure
        is InterpolationFailure.MISSING_AFTER
    )


@pytest.mark.parametrize(
    ('image_stamp_ns', 'expected_failure'),
    [
        (141, InterpolationFailure.STALE_BEFORE),
        (139, InterpolationFailure.STALE_AFTER),
    ],
)
def test_stale_bracketing_sample_rejects_image(
    image_stamp_ns: int,
    expected_failure: InterpolationFailure,
) -> None:
    buffer = _buffer(max_sample_distance_ns=40)
    buffer.add(_sample(100))
    buffer.add(_sample(200))

    assert buffer.interpolate(image_stamp_ns).failure is expected_failure


def test_exact_endpoint_still_requires_a_fresh_second_sample() -> None:
    buffer = _buffer(max_sample_distance_ns=100)
    buffer.add(_sample(100))
    buffer.add(_sample(200, offset=10.0))

    at_first = buffer.interpolate(100).interpolation
    at_last = buffer.interpolate(200).interpolation

    assert at_first is not None
    assert at_first.ratio == 0.0
    assert at_first.sample.positions_rad == _sample(100).positions_rad
    assert at_last is not None
    assert at_last.ratio == 1.0
    assert at_last.sample.positions_rad == _sample(
        200, offset=10.0
    ).positions_rad


def test_incompatible_extra_joint_sets_reject_interpolation() -> None:
    buffer = _buffer()
    before_names = FULL_JOINTS + ('optional_joint',)
    buffer.add(_sample(100, joint_names=before_names))
    buffer.add(_sample(200))

    assert (
        buffer.interpolate(150).failure
        is InterpolationFailure.INCOMPATIBLE_JOINT_SETS
    )


def test_duplicate_and_small_out_of_order_samples_are_rejected() -> None:
    buffer = _buffer(clock_reset_threshold_ns=100)
    earlier = _sample(900)
    latest = _sample(1_000)
    buffer.add(earlier)
    buffer.add(latest)

    duplicate = buffer.add(_sample(900, offset=100.0))
    out_of_order = buffer.add(_sample(950))

    assert duplicate.status is BufferInsertStatus.DUPLICATE
    assert out_of_order.status is BufferInsertStatus.OUT_OF_ORDER
    assert buffer.samples == (earlier, latest)


def test_large_regression_resets_clock_epoch_before_accepting() -> None:
    buffer = _buffer(clock_reset_threshold_ns=100)
    buffer.add(_sample(1_000))
    buffer.add(_sample(1_100))

    result = buffer.add(_sample(50))

    assert result.status is BufferInsertStatus.CLOCK_RESET
    assert result.accepted
    assert result.discarded_sample_count == 2
    assert tuple(item.stamp_ns for item in buffer.samples) == (50,)
    assert (
        buffer.interpolate(1_050).failure
        is InterpolationFailure.MISSING_AFTER
    )


def test_interpolation_is_immutable_and_validates_provenance() -> None:
    sample = _sample(150)
    interpolation = InterpolatedJointState(
        sample=sample,
        before_stamp_ns=100,
        after_stamp_ns=200,
        ratio=0.5,
    )

    with pytest.raises(FrozenInstanceError):
        interpolation.ratio = 0.25
    with pytest.raises(ValueError, match='does not match'):
        InterpolatedJointState(
            sample=sample,
            before_stamp_ns=100,
            after_stamp_ns=200,
            ratio=0.25,
        )


@pytest.mark.parametrize(
    ('keyword', 'value'),
    [
        ('capacity', 0),
        ('max_sample_distance_ns', 0),
        ('clock_reset_threshold_ns', 0),
    ],
)
def test_buffer_requires_explicit_positive_bounds(
    keyword: str,
    value: int,
) -> None:
    arguments = {
        'capacity': 2,
        'max_sample_distance_ns': 10,
        'clock_reset_threshold_ns': 20,
    }
    arguments[keyword] = value

    with pytest.raises(ValueError, match='positive integer'):
        JointStateRingBuffer(**arguments)
