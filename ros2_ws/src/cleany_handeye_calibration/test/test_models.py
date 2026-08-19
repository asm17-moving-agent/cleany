from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from cleany_handeye_calibration.models import (
    CalibrationSample,
    IkResult,
    JointPose,
    PositionTarget,
    SampleSplit,
    TimedJointSample,
)
from cleany_handeye_calibration.transforms import RigidTransform


JOINT_NAMES = (
    'left_shoulder_yaw_joint',
    'left_shoulder_pitch_joint',
    'left_elbow_pitch_joint',
    'left_wrist_pitch_joint',
    'left_wrist_roll_joint',
)


def _joint_pose() -> JointPose:
    return JointPose(
        joint_names=JOINT_NAMES,
        positions_rad=(0.1, 0.2, 0.3, 0.4, 0.5),
    )


def _sample() -> CalibrationSample:
    return CalibrationSample(
        sample_id='sample_001',
        pose_id='calibration_001',
        split=SampleSplit.CALIBRATION,
        base_T_gripper=RigidTransform(
            parent_frame='base_link',
            child_frame='left_gripper_frame',
            rotation_matrix=np.eye(3),
            translation_m=(0.3, 0.2, 0.5),
        ),
        camera_T_target=RigidTransform(
            parent_frame='left_wrist_rgb_optical_frame',
            child_frame='charuco_target',
            rotation_matrix=np.eye(3),
            translation_m=(0.0, 0.0, 0.4),
        ),
    )


def test_position_target_and_joint_pose_normalize_to_immutable_tuples():
    target_values = [0.3, 0.2, 0.5]
    joint_values = [0.1, 0.2, 0.3, 0.4, 0.5]

    target = PositionTarget('base_link', target_values)
    pose = JointPose(JOINT_NAMES, joint_values)
    target_values[0] = 9.0
    joint_values[0] = 9.0

    assert target.position_m == (0.3, 0.2, 0.5)
    assert pose.positions_rad == (0.1, 0.2, 0.3, 0.4, 0.5)
    with pytest.raises(FrozenInstanceError):
        target.frame_id = 'map'


@pytest.mark.parametrize(
    ('joint_names', 'positions'),
    [
        ((), ()),
        (('joint', 'joint'), (0.0, 0.0)),
        (('joint_a', 'joint_b'), (0.0,)),
        (('joint',), (float('nan'),)),
    ],
)
def test_joint_pose_rejects_invalid_joint_vectors(joint_names, positions):
    with pytest.raises(ValueError):
        JointPose(joint_names=joint_names, positions_rad=positions)


def test_ik_result_enforces_success_and_failure_payloads():
    pose = _joint_pose()

    assert IkResult(success=True, joint_pose=pose).joint_pose == pose
    failed = IkResult(success=False, failure_reason='no_solution')
    assert failed.joint_pose is None
    with pytest.raises(ValueError, match='requires joint_pose'):
        IkResult(success=True)
    with pytest.raises(ValueError, match='requires failure_reason'):
        IkResult(success=False)
    with pytest.raises(ValueError, match='must not include joint_pose'):
        IkResult(
            success=False,
            joint_pose=pose,
            failure_reason='no_solution',
        )


def test_timed_joint_sample_validates_timestamp_and_velocity_shape():
    sample = TimedJointSample(
        stamp_ns=123,
        joint_names=JOINT_NAMES,
        positions_rad=(0.1, 0.2, 0.3, 0.4, 0.5),
        velocities_rad_s=(0.0, 0.0, 0.0, 0.0, 0.0),
    )

    assert sample.stamp_ns == 123
    with pytest.raises(ValueError, match='non-negative integer'):
        TimedJointSample(
            stamp_ns=-1,
            joint_names=('joint',),
            positions_rad=(0.0,),
        )
    with pytest.raises(ValueError, match='velocities_rad_s'):
        TimedJointSample(
            stamp_ns=0,
            joint_names=('joint_a', 'joint_b'),
            positions_rad=(0.0, 0.0),
            velocities_rad_s=(0.0,),
        )


def test_calibration_sample_preserves_explicit_transform_directions():
    sample = _sample()

    assert sample.split is SampleSplit.CALIBRATION
    assert sample.base_T_gripper.parent_frame == 'base_link'
    assert sample.base_T_gripper.child_frame == 'left_gripper_frame'
    assert (
        sample.camera_T_target.parent_frame
        == 'left_wrist_rgb_optical_frame'
    )
    assert sample.camera_T_target.child_frame == 'charuco_target'


def test_calibration_sample_accepts_only_known_split():
    sample = _sample()

    held_out = CalibrationSample(
        sample_id=sample.sample_id,
        pose_id=sample.pose_id,
        split='held_out',
        base_T_gripper=sample.base_T_gripper,
        camera_T_target=sample.camera_T_target,
    )

    assert held_out.split is SampleSplit.HELD_OUT
    with pytest.raises(ValueError, match='calibration or held_out'):
        CalibrationSample(
            sample_id=sample.sample_id,
            pose_id=sample.pose_id,
            split='validation',
            base_T_gripper=sample.base_T_gripper,
            camera_T_target=sample.camera_T_target,
        )
