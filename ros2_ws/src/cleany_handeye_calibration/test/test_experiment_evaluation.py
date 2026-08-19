from collections import defaultdict
import json

import numpy as np
import pytest

from cleany_handeye_calibration.experiment_evaluation import (
    EXPECTED_SOLVER_RUN_COUNT,
    ExperimentConfig,
    MethodExperimentSummary,
    NoiseCondition,
    SelectionStatus,
    load_sample_records,
    percentile_linear,
    run_solver_experiment,
    select_method,
    validate_evaluation_records,
)
from cleany_handeye_calibration.dataset_writer import (
    mapping_sha256,
    sha256_bytes,
)
from cleany_handeye_calibration.dataset_validation import _review_candidates
from cleany_handeye_calibration.pnp import PnpResult
from cleany_handeye_calibration.solver import (
    HAND_EYE_METHOD_REGISTRY,
    HandEyeMethod,
    HandEyeResult,
)
from cleany_handeye_calibration.schema import sample_record_to_mapping
from cleany_handeye_calibration.transforms import RigidTransform
from evaluation_test_support import evaluation_records


GROUND_TRUTH = RigidTransform.from_rodrigues(
    parent_frame='left_gripper_frame',
    child_frame='left_wrist_rgb_optical_frame',
    translation_m=(0.03, -0.02, 0.08),
    rodrigues_vector=(0.0, 0.0, 0.0),
)


class RecordingFk:
    def __init__(self):
        self.calls = []

    def compute(self, joint_names, positions_rad):
        self.calls.append((tuple(joint_names), tuple(positions_rad)))
        q0 = float(positions_rad[0])
        return RigidTransform.from_rodrigues(
            parent_frame='base_link',
            child_frame='left_gripper_frame',
            translation_m=(0.3 + q0 * 0.01, 0.18, 0.52),
            rodrigues_vector=(q0, 0.1, -0.05),
        )


class RecordingPnp:
    def __init__(self):
        self.points = []

    def __call__(self, detection, **kwargs):
        del kwargs
        self.points.append(np.asarray(detection.image_points_px))
        pose = RigidTransform.from_rodrigues(
            parent_frame='left_wrist_rgb_optical_frame',
            child_frame='charuco_target',
            translation_m=(0.01, -0.02, 0.45),
            rodrigues_vector=(0.2, -0.1, 0.05),
        )
        return PnpResult(True, 'SOLVEPNP_IPPE', None, None, False, 0, pose, ())


def _deterministic_results(validity_policy):
    assert validity_policy.max_translation_norm_m == 1.0
    results = []
    for index, spec in enumerate(HAND_EYE_METHOD_REGISTRY):
        estimate = RigidTransform.from_rodrigues(
            parent_frame='left_gripper_frame',
            child_frame='left_wrist_rgb_optical_frame',
            translation_m=(0.03 + index * 0.01, -0.02, 0.08),
            rodrigues_vector=(0.0, 0.0, 0.0),
        )
        results.append(
            HandEyeResult(
                spec.method,
                spec.opencv_symbol,
                True,
                estimate,
                None,
                None,
                1.0 + index,
            )
        )
    return tuple(results)


def deterministic_solver(samples, *, validity_policy):
    assert len(samples) == 20
    return _deterministic_results(validity_policy)


def test_exact_150_rows_share_one_noise_bundle_across_all_five_methods():
    fk = RecordingFk()
    pnp = RecordingPnp()
    config = ExperimentConfig(tuple(range(10)), 1.0)

    result = run_solver_experiment(
        evaluation_records(),
        fk,
        config,
        ground_truth=GROUND_TRUTH,
        pnp_solver=pnp,
        solve_all=deterministic_solver,
    )

    assert len(result.rows) == EXPECTED_SOLVER_RUN_COUNT
    assert len(result.pnp_diagnostics) == 750
    grouped = defaultdict(list)
    for row in result.rows:
        grouped[(row.condition, row.random_seed)].append(row)
    assert len(grouped) == 30
    assert all(len(rows) == 5 for rows in grouped.values())
    assert all(
        len({row.noise_bundle_sha256 for row in rows}) == 1
        for rows in grouped.values()
    )
    assert all(
        item.noise_bundle_sha256
        == grouped[(item.condition, item.random_seed)][0].noise_bundle_sha256
        for item in result.pnp_diagnostics
    )
    assert len(fk.calls) == 25 * 30
    assert len(pnp.points) == 25 * 30
    assert result.selection.selected_method.value == 'tsai'
    assert result.selection.candidate_condition is NoiseCondition.IDEAL
    assert result.selection.candidate_seed == 0
    assert result.selection.candidate_transform == GROUND_TRUTH
    candidates = _review_candidates(result)
    assert len(candidates) == 5
    assert candidates[0]['method'] == 'tsai'
    assert candidates[0]['gripper_T_camera']['parent_frame'] == (
        'left_gripper_frame'
    )


def test_explicit_partial_mode_runs_19_plus_4_without_silent_row_drops():
    records = tuple(
        record
        for record in evaluation_records()
        if record.sample.pose_id not in ('calibration_006', 'held_out_001')
    )

    with pytest.raises(ValueError, match='exactly 20 calibration'):
        validate_evaluation_records(records)

    def solve_partial(samples, *, validity_policy):
        assert len(samples) == 19
        return _deterministic_results(validity_policy)

    result = run_solver_experiment(
        records,
        RecordingFk(),
        ExperimentConfig(tuple(range(10)), 1.0),
        ground_truth=GROUND_TRUTH,
        pnp_solver=RecordingPnp(),
        solve_all=solve_partial,
        allow_partial=True,
    )

    assert len(result.rows) == EXPECTED_SOLVER_RUN_COUNT
    assert len(result.pnp_diagnostics) == 23 * 30


def test_partial_mode_still_requires_solver_and_held_out_minimums():
    records = evaluation_records()
    too_few_calibration = records[:2] + records[20:21]
    with pytest.raises(ValueError, match='5..20 calibration'):
        validate_evaluation_records(
            too_few_calibration,
            allow_partial=True,
        )


def test_noise_is_applied_to_pixels_and_feedback_q_before_pnp_and_fk():
    fk = RecordingFk()
    pnp = RecordingPnp()
    records = evaluation_records()

    run_solver_experiment(
        records,
        fk,
        ExperimentConfig(tuple(range(10)), 1.0),
        ground_truth=GROUND_TRUTH,
        pnp_solver=pnp,
        solve_all=deterministic_solver,
    )

    original_q = records[0].interpolated_joints.positions_rad
    original_points = records[0].target_detection.image_points_array()
    # First 250 calls are ideal; nominal starts at call 250.
    assert fk.calls[0][1] == original_q
    np.testing.assert_array_equal(pnp.points[0], original_points)
    assert fk.calls[250][1] != original_q
    assert not np.array_equal(pnp.points[250], original_points)
    # Right arm and grippers are never perturbed by the left-arm noise model.
    assert fk.calls[250][1][5:] == original_q[5:]


def test_same_inputs_and_seeds_reproduce_every_non_runtime_output():
    def run_once():
        return run_solver_experiment(
            evaluation_records(),
            RecordingFk(),
            ExperimentConfig(tuple(range(10, 20)), 1.0),
            ground_truth=GROUND_TRUTH,
            pnp_solver=RecordingPnp(),
            solve_all=deterministic_solver,
        )

    first = run_once()
    second = run_once()
    first_values = tuple(
        (
            row.condition,
            row.random_seed,
            row.method,
            row.noise_bundle_sha256,
            row.valid,
            row.gripper_T_camera,
            row.translation_error_m,
            row.rotation_error_rad,
        )
        for row in first.rows
    )
    second_values = tuple(
        (
            row.condition,
            row.random_seed,
            row.method,
            row.noise_bundle_sha256,
            row.valid,
            row.gripper_T_camera,
            row.translation_error_m,
            row.rotation_error_rad,
        )
        for row in second.rows
    )
    assert first_values == second_values


def test_manual_percentile_matches_numpy_121_linear_definition():
    values = [9.0, 1.0, 5.0, 3.0]
    assert percentile_linear(values, 50.0) == pytest.approx(4.0)
    assert percentile_linear(values, 95.0) == pytest.approx(8.4)
    with pytest.raises(ValueError):
        percentile_linear([], 95.0)


def test_experiment_config_requires_exactly_ten_unique_seeds():
    with pytest.raises(ValueError, match='10 unique'):
        ExperimentConfig(tuple(range(9)), 1.0)
    with pytest.raises(ValueError, match='10 unique'):
        ExperimentConfig((0,) * 10, 1.0)


def test_sample_loader_verifies_committed_row_and_image_hashes(tmp_path):
    images = tmp_path / 'images'
    images.mkdir()
    mappings = []
    for record in evaluation_records():
        image_payload = record.sample.sample_id.encode('ascii')
        (tmp_path / record.image_path).write_bytes(image_payload)
        mapping = sample_record_to_mapping(record)
        mapping['image_sha256'] = sha256_bytes(image_payload)
        mapping['source_image_sha256'] = 'a' * 64
        mapping['record_sha256'] = mapping_sha256(mapping)
        mappings.append(mapping)
    samples = tmp_path / 'samples.jsonl'
    samples.write_text(
        ''.join(
            json.dumps(mapping, sort_keys=True, separators=(',', ':')) + '\n'
            for mapping in mappings
        ),
        encoding='utf-8',
    )

    assert len(load_sample_records(samples)) == 25

    mappings[0]['joint_positions_rad'][0] += 1.0
    samples.write_text(
        ''.join(
            json.dumps(mapping, sort_keys=True, separators=(',', ':')) + '\n'
            for mapping in mappings
        ),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='record SHA-256'):
        load_sample_records(samples)


def test_crossed_accuracy_and_held_out_metrics_require_human_review():
    summaries = []
    accuracy = {
        HandEyeMethod.TSAI: (1.0, 2.0, 1.0, 2.0),
        HandEyeMethod.PARK: (2.0, 1.0, 2.0, 1.0),
    }
    held_out = {
        HandEyeMethod.TSAI: (1.0, 2.0, 1.0, 2.0),
        HandEyeMethod.PARK: (2.0, 1.0, 2.0, 1.0),
    }
    for method in HandEyeMethod:
        a = accuracy.get(method, (3.0, 3.0, 3.0, 3.0))
        h = held_out.get(method, (3.0, 3.0, 3.0, 3.0))
        summaries.append(
            MethodExperimentSummary(
                method,
                30,
                30,
                1.0,
                *a,
                *h,
                1.0,
                1.0,
                (),
            )
        )

    selection = select_method(summaries, (), candidate_seed=0)

    assert selection.status is SelectionStatus.REVIEW_REQUIRED
    assert selection.selected_method is None
    assert 'human review' in selection.reason
