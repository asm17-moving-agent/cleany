import json

import numpy as np
import pytest

from cleany_handeye_calibration.models import (
    CalibrationSample,
    JointPose,
    PositionTarget,
    SampleSplit,
    TimedJointSample,
)
from cleany_handeye_calibration.schema import (
    SAMPLE_RECORD_SCHEMA_VERSION,
    CalibrationSampleRecord,
    sample_record_from_mapping,
    sample_record_to_mapping,
)
from cleany_handeye_calibration.transforms import RigidTransform


JOINT_NAMES = (
    'left_shoulder_yaw_joint',
    'left_shoulder_pitch_joint',
    'left_elbow_pitch_joint',
    'left_wrist_pitch_joint',
    'left_wrist_roll_joint',
)


def _record() -> CalibrationSampleRecord:
    sample = CalibrationSample(
        sample_id='sample_001',
        pose_id='calibration_001',
        split=SampleSplit.CALIBRATION,
        base_T_gripper=RigidTransform.from_rodrigues(
            parent_frame='base_link',
            child_frame='left_gripper_frame',
            translation_m=(0.31, 0.18, 0.52),
            rodrigues_vector=(0.3, -0.1, 0.2),
        ),
        camera_T_target=RigidTransform.from_quaternion_xyzw(
            parent_frame='left_wrist_rgb_optical_frame',
            child_frame='charuco_target',
            translation_m=(0.01, -0.02, 0.45),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        ),
    )
    seed = JointPose(JOINT_NAMES, (0.0, 0.1, 0.2, 0.3, 0.4))
    resolved = JointPose(JOINT_NAMES, (0.1, 0.2, 0.3, 0.4, 0.5))
    return CalibrationSampleRecord(
        sample=sample,
        calibration_arm='left',
        planning_group='left_arm',
        position_target=PositionTarget(
            frame_id='base_link',
            position_m=(0.3, 0.2, 0.5),
        ),
        ik_seed=seed,
        resolved_ik=resolved,
        image_stamp_ns=1_500,
        joint_state_before_stamp_ns=1_000,
        joint_state_after_stamp_ns=2_000,
        joint_interpolation_ratio=0.5,
        interpolated_joints=TimedJointSample(
            stamp_ns=1_500,
            joint_names=JOINT_NAMES,
            positions_rad=(0.1, 0.2, 0.3, 0.4, 0.5),
            velocities_rad_s=(0.0, 0.0, 0.0, 0.0, 0.0),
        ),
        pnp_method='SOLVEPNP_IPPE',
        pnp_reprojection_rmse_px=0.2,
        pnp_ambiguous=False,
        image_path='images/sample_001.png',
    )


def test_sample_record_matches_versioned_units_and_transform_schema():
    mapping = sample_record_to_mapping(_record())

    assert mapping['schema_version'] == SAMPLE_RECORD_SCHEMA_VERSION
    assert mapping['split'] == 'calibration'
    assert mapping['target_position_m'] == [0.3, 0.2, 0.5]
    assert mapping['ik_seed_positions_rad'] == [0.0, 0.1, 0.2, 0.3, 0.4]
    assert mapping['image_stamp_ns'] == 1_500
    assert mapping['base_to_gripper']['parent_frame'] == 'base_link'
    assert (
        mapping['base_to_gripper']['child_frame']
        == 'left_gripper_frame'
    )
    assert mapping['camera_to_target']['quaternion_xyzw'] == [
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    assert mapping['pnp'] == {
        'method': 'SOLVEPNP_IPPE',
        'reprojection_rmse_px': 0.2,
        'ambiguous': False,
    }
    json.dumps(mapping)


def test_sample_record_round_trip_preserves_models_and_values():
    record = _record()

    restored = sample_record_from_mapping(
        json.loads(json.dumps(sample_record_to_mapping(record)))
    )

    assert restored.sample.sample_id == record.sample.sample_id
    assert restored.sample.pose_id == record.sample.pose_id
    assert restored.sample.split == record.sample.split
    np.testing.assert_allclose(
        restored.sample.base_T_gripper.as_homogeneous_matrix(),
        record.sample.base_T_gripper.as_homogeneous_matrix(),
        rtol=0.0,
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        restored.sample.camera_T_target.as_homogeneous_matrix(),
        record.sample.camera_T_target.as_homogeneous_matrix(),
        rtol=0.0,
        atol=1.0e-12,
    )
    assert restored.calibration_arm == record.calibration_arm
    assert restored.planning_group == record.planning_group
    assert restored.position_target == record.position_target
    assert restored.ik_seed == record.ik_seed
    assert restored.resolved_ik == record.resolved_ik
    assert restored.interpolated_joints == record.interpolated_joints
    assert restored.pnp_method == record.pnp_method
    assert restored.pnp_reprojection_rmse_px == (
        record.pnp_reprojection_rmse_px
    )
    assert restored.pnp_ambiguous == record.pnp_ambiguous
    assert restored.image_path == record.image_path


@pytest.mark.parametrize(
    ('changes', 'message'),
    [
        ({'joint_state_after_stamp_ns': 1_000}, 'ordered timestamps'),
        ({'image_stamp_ns': 2_500}, 'must be bracketed'),
        ({'joint_interpolation_ratio': 1.1}, 'must be finite'),
        ({'pnp_reprojection_rmse_px': float('nan')}, 'non-negative'),
    ],
)
def test_sample_record_rejects_invalid_synchronization_or_pnp_fields(
    changes,
    message,
):
    values = {
        field: getattr(_record(), field)
        for field in CalibrationSampleRecord.__dataclass_fields__
    }
    values.update(changes)

    with pytest.raises(ValueError, match=message):
        CalibrationSampleRecord(**values)


def test_sample_record_parser_rejects_unknown_schema_version():
    mapping = sample_record_to_mapping(_record())
    mapping['schema_version'] = 2

    with pytest.raises(ValueError, match='unsupported sample schema_version'):
        sample_record_from_mapping(mapping)
