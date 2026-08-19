import json

import numpy as np
import pytest

from cleany_handeye_calibration.experiment_evaluation import (
    MethodSelection,
    NoiseCondition,
    SelectionStatus,
)
from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
)
from cleany_handeye_calibration.models import TimedJointSample
from cleany_handeye_calibration.schema import transform_to_mapping
from cleany_handeye_calibration.solver import HandEyeMethod, HandEyeResult
from cleany_handeye_calibration.timestamp_sensitivity import (
    ContinuousImageObservation,
    ContinuousTrajectoryLog,
    TimestampSensitivityConfig,
    evaluate_timestamp_sensitivity,
    load_continuous_trajectory_log,
)
from cleany_handeye_calibration.transforms import RigidTransform


JOINT_NAMES = DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
TRUTH = RigidTransform.from_rodrigues(
    parent_frame='left_gripper_frame',
    child_frame='left_wrist_rgb_optical_frame',
    translation_m=(0.03, 0.0, 0.08),
    rodrigues_vector=(0.0, 0.0, 0.0),
)


def _trajectory():
    joint_samples = tuple(
        TimedJointSample(
            stamp_ns=stamp,
            joint_names=JOINT_NAMES,
            positions_rad=(stamp / 1000.0,) + (0.0,) * 11,
            velocities_rad_s=(1.0,) + (0.0,) * 11,
        )
        for stamp in (100, 200, 300, 400)
    )
    camera_pose = RigidTransform.from_rodrigues(
        parent_frame='left_wrist_rgb_optical_frame',
        child_frame='charuco_target',
        translation_m=(0.0, 0.0, 0.5),
        rodrigues_vector=(0.1, -0.2, 0.05),
    )
    observations = tuple(
        ContinuousImageObservation(
            f'continuous_{index}',
            f'continuous_pose_{index}',
            stamp,
            camera_pose,
        )
        for index, stamp in enumerate((150, 250, 350), start=1)
    )
    return ContinuousTrajectoryLog(joint_samples, observations)


class RecordingFk:
    def __init__(self):
        self.q = []

    def compute(self, joint_names, positions_rad):
        assert tuple(joint_names) == JOINT_NAMES
        self.q.append(positions_rad[0])
        return RigidTransform.from_rodrigues(
            parent_frame='base_link',
            child_frame='left_gripper_frame',
            translation_m=(positions_rad[0], 0.0, 0.5),
            rodrigues_vector=(positions_rad[0], 0.0, 0.0),
        )


def _selection(status=SelectionStatus.SELECTED):
    if status is SelectionStatus.SELECTED:
        return MethodSelection(
            status,
            HandEyeMethod.PARK,
            'test selection',
            TRUTH,
            NoiseCondition.IDEAL,
            0,
        )
    return MethodSelection(status, None, 'crossed metrics', None, None, None)


def test_timestamp_sensitivity_runs_only_selected_method_for_each_offset():
    calls = []

    def solve_one(samples, method, *, validity_policy):
        calls.append(
            (len(samples), method, validity_policy.max_translation_norm_m)
        )
        return HandEyeResult(
            method,
            'CALIB_HAND_EYE_PARK',
            True,
            TRUTH,
            None,
            None,
            2.0,
        )

    fk = RecordingFk()
    result = evaluate_timestamp_sensitivity(
        _trajectory(),
        fk,
        TimestampSensitivityConfig((-25, 0, 25), 100, 1.0),
        _selection(),
        ground_truth=TRUTH,
        solve_one=solve_one,
    )

    assert not result.review_required
    assert result.selected_method is HandEyeMethod.PARK
    assert [row.offset_ns for row in result.rows] == [-25, 0, 25]
    assert all(row.valid for row in result.rows)
    assert calls == [(3, HandEyeMethod.PARK, 1.0)] * 3
    assert fk.q[3:6] == pytest.approx([0.15, 0.25, 0.35])


def test_crossed_solver_metrics_skip_timestamp_experiment():
    def unexpected(*args, **kwargs):
        raise AssertionError('solver must not run before method selection')

    result = evaluate_timestamp_sensitivity(
        _trajectory(),
        RecordingFk(),
        TimestampSensitivityConfig((0,), 100, 1.0),
        _selection(SelectionStatus.REVIEW_REQUIRED),
        ground_truth=TRUTH,
        solve_one=unexpected,
    )

    assert result.review_required
    assert result.rows == ()


def test_continuous_log_loader_keeps_joint_series_separate_from_images(
    tmp_path,
):
    trajectory = _trajectory()
    rows = []
    for sample in trajectory.joint_samples:
        rows.append(
            {
                'schema_version': 1,
                'kind': 'joint_state',
                'stamp_ns': sample.stamp_ns,
                'joint_names': list(sample.joint_names),
                'positions_rad': list(sample.positions_rad),
                'velocities_rad_s': list(sample.velocities_rad_s),
            }
        )
    for observation in trajectory.image_observations:
        rows.append(
            {
                'schema_version': 1,
                'kind': 'image_observation',
                'sample_id': observation.sample_id,
                'pose_id': observation.pose_id,
                'split': 'calibration',
                'image_stamp_ns': observation.image_stamp_ns,
                'camera_to_target': transform_to_mapping(
                    observation.camera_T_target
                ),
            }
        )
    path = tmp_path / 'continuous.jsonl'
    path.write_text(
        ''.join(json.dumps(row) + '\n' for row in rows),
        encoding='utf-8',
    )

    restored = load_continuous_trajectory_log(path)
    assert restored.joint_samples == trajectory.joint_samples
    assert tuple(
        (item.sample_id, item.pose_id, item.image_stamp_ns)
        for item in restored.image_observations
    ) == tuple(
        (item.sample_id, item.pose_id, item.image_stamp_ns)
        for item in trajectory.image_observations
    )
    for actual, expected in zip(
        restored.image_observations,
        trajectory.image_observations,
        strict=True,
    ):
        np.testing.assert_allclose(
            actual.camera_T_target.as_homogeneous_matrix(),
            expected.camera_T_target.as_homogeneous_matrix(),
            rtol=0.0,
            atol=1.0e-12,
        )

    rows[-1]['unexpected'] = True
    path.write_text(
        ''.join(json.dumps(row) + '\n' for row in rows),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='invalid fields'):
        load_continuous_trajectory_log(path)
