from dataclasses import replace
import json

import numpy as np
import pytest

from cleany_handeye_calibration.camera_acquisition import (
    CAMERA_D,
    CAMERA_DISTORTION_MODEL,
    CAMERA_FRAME_ID,
    CAMERA_HEIGHT,
    CAMERA_K,
    CAMERA_P,
    CAMERA_R,
    CAMERA_WIDTH,
    CameraInfoFrame,
)
from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    LEFT_ARM_JOINT_NAMES,
)
from cleany_handeye_calibration.models import (
    CalibrationSample,
    JointPose,
    PositionTarget,
    SampleSplit,
    TimedJointSample,
)
from cleany_handeye_calibration.schema import (
    CORNER_POINT_ORDERING,
    SAMPLE_RECORD_SCHEMA_VERSION,
    CalibrationSampleRecord,
    camera_calibration_sha256,
    sample_record_from_mapping,
    sample_record_to_mapping,
)
from cleany_handeye_calibration.target_detector import (
    INNER_CORNER_COUNT,
    analyze_charuco_corners,
)
from cleany_handeye_calibration.transforms import RigidTransform


JOINT_NAMES = DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names


def _detection():
    return analyze_charuco_corners(
        tuple(range(INNER_CORNER_COUNT)),
        tuple(
            (100.0 + (index % 6) * 40.0, 80.0 + (index // 6) * 50.0)
            for index in range(INNER_CORNER_COUNT)
        ),
    )


def _camera_info() -> CameraInfoFrame:
    return CameraInfoFrame(
        stamp_ns=1_500,
        frame_id=CAMERA_FRAME_ID,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        distortion_model=CAMERA_DISTORTION_MODEL,
        d=CAMERA_D,
        k=CAMERA_K,
        r=CAMERA_R,
        p=CAMERA_P,
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
            parent_frame=CAMERA_FRAME_ID,
            child_frame='charuco_target',
            translation_m=(0.01, -0.02, 0.45),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        ),
    )
    seed = JointPose(LEFT_ARM_JOINT_NAMES, (0.0, 0.1, 0.2, 0.3, 0.4))
    resolved = JointPose(
        LEFT_ARM_JOINT_NAMES,
        (0.1, 0.2, 0.3, 0.4, 0.5),
    )
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
            positions_rad=tuple(index / 10.0 for index in range(12)),
            velocities_rad_s=(0.0,) * 12,
        ),
        camera_info=_camera_info(),
        target_detection=_detection(),
        pnp_method='SOLVEPNP_IPPE',
        pnp_reprojection_rmse_px=0.2,
        pnp_ambiguous=False,
        pnp_selected_candidate_index=1,
        image_path='images/sample_001.png',
    )


def test_sample_record_contains_every_reproducible_pnp_and_fk_input():
    mapping = sample_record_to_mapping(_record())

    assert mapping['schema_version'] == SAMPLE_RECORD_SCHEMA_VERSION
    assert mapping['split'] == 'calibration'
    assert mapping['target_position_m'] == [0.3, 0.2, 0.5]
    assert mapping['ik_seed_positions_rad'] == [0.0, 0.1, 0.2, 0.3, 0.4]
    assert mapping['image_stamp_ns'] == 1_500
    assert mapping['camera_info_stamp_ns'] == 1_500
    assert len(mapping['joint_names']) == 12
    assert len(mapping['joint_positions_rad']) == 12
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
    assert mapping['camera_calibration']['K'] == list(CAMERA_K)
    assert mapping['camera_calibration']['D'] == list(CAMERA_D)
    assert mapping['camera_calibration_sha256'] == (
        camera_calibration_sha256(_camera_info())
    )
    assert mapping['target_detection']['point_ordering'] == (
        CORNER_POINT_ORDERING
    )
    assert mapping['target_detection']['corner_ids'] == list(
        range(INNER_CORNER_COUNT)
    )
    assert len(mapping['target_detection']['object_points_m']) == 24
    assert len(mapping['target_detection']['image_points_px']) == 24
    assert mapping['pnp'] == {
        'method': 'SOLVEPNP_IPPE',
        'reprojection_rmse_px': 0.2,
        'ambiguous': False,
        'selected_candidate_index': 1,
    }
    json.dumps(mapping, allow_nan=False)


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
    assert restored.camera_info == record.camera_info
    assert restored.target_detection == record.target_detection
    assert restored.pnp_method == record.pnp_method
    assert restored.pnp_reprojection_rmse_px == (
        record.pnp_reprojection_rmse_px
    )
    assert restored.pnp_ambiguous == record.pnp_ambiguous
    assert restored.pnp_selected_candidate_index == 1
    assert restored.image_path == record.image_path


@pytest.mark.parametrize(
    ('changes', 'message'),
    [
        ({'joint_state_after_stamp_ns': 1_000}, 'ordered timestamps'),
        ({'image_stamp_ns': 2_500}, 'must be bracketed'),
        ({'joint_interpolation_ratio': 0.4}, 'must be finite and match'),
        ({'pnp_reprojection_rmse_px': float('nan')}, 'non-negative'),
        ({'pnp_ambiguous': True}, 'ambiguous PnP'),
        ({'calibration_arm': 'right'}, 'require arm left'),
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


def test_sample_record_requires_exact_canonical_joint_and_camera_contracts():
    record = _record()
    values = {
        field: getattr(record, field)
        for field in CalibrationSampleRecord.__dataclass_fields__
    }
    values['interpolated_joints'] = TimedJointSample(
        stamp_ns=1_500,
        joint_names=JOINT_NAMES[:-1],
        positions_rad=record.interpolated_joints.positions_rad[:-1],
        velocities_rad_s=(0.0,) * 11,
    )
    with pytest.raises(ValueError, match='exactly the canonical 12'):
        CalibrationSampleRecord(**values)

    values['interpolated_joints'] = record.interpolated_joints
    values['camera_info'] = replace(
        record.camera_info,
        k=(1.0,) + CAMERA_K[1:],
    )
    with pytest.raises(ValueError, match='camera_info must match'):
        CalibrationSampleRecord(**values)


def test_sample_artifact_identifiers_cannot_escape_image_or_journal_roots():
    record = _record()
    values = {
        field: getattr(record, field)
        for field in CalibrationSampleRecord.__dataclass_fields__
    }
    values['sample'] = replace(record.sample, sample_id='../escape')
    values['image_path'] = 'images/../escape.png'

    with pytest.raises(ValueError, match='sample_id must contain only'):
        CalibrationSampleRecord(**values)


def test_sample_record_parser_rejects_schema_hash_and_ordering_tampering():
    mapping = sample_record_to_mapping(_record())
    mapping['schema_version'] = 2
    with pytest.raises(ValueError, match='unsupported sample schema_version'):
        sample_record_from_mapping(mapping)

    mapping = sample_record_to_mapping(_record())
    mapping['camera_calibration']['K'][0] += 1.0
    with pytest.raises(ValueError, match='SHA-256 does not match'):
        sample_record_from_mapping(mapping)

    mapping = sample_record_to_mapping(_record())
    mapping['target_detection']['corner_ids'] = list(reversed(
        mapping['target_detection']['corner_ids']
    ))
    with pytest.raises(ValueError, match='ascending canonical order'):
        sample_record_from_mapping(mapping)
