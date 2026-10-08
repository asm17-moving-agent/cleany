from pathlib import Path
import json

import pytest
import yaml

from cleany_handeye_calibration.evaluation_artifacts import (
    load_evaluation_config,
    load_ground_truth,
    validate_dataset_manifest,
    write_evaluation_artifacts,
)
from cleany_handeye_calibration.dataset_writer import mapping_sha256
from cleany_handeye_calibration.experiment_evaluation import (
    ExperimentConfig,
    NOISE_CONDITIONS,
    PnpExperimentDiagnostic,
    SolverExperimentResult,
    SolverExperimentRow,
    select_method,
    summarize_methods,
)
from cleany_handeye_calibration.models import SampleSplit
from cleany_handeye_calibration.schema import transform_to_mapping
from cleany_handeye_calibration.solver import HandEyeMethod
from cleany_handeye_calibration.timestamp_sensitivity import (
    TimestampSensitivityResult,
    TimestampSensitivityRow,
)
from cleany_handeye_calibration.transforms import RigidTransform


TRUTH = RigidTransform.from_rodrigues(
    parent_frame='left_gripper_frame',
    child_frame='left_wrist_rgb_optical_frame',
    translation_m=(0.03, -0.02, 0.08),
    rodrigues_vector=(0.0, 0.0, 0.0),
)


def _result():
    config = ExperimentConfig(tuple(range(10)), 1.0)
    rows = []
    for condition in NOISE_CONDITIONS:
        for seed in config.random_seeds:
            bundle_value = seed + list(NOISE_CONDITIONS).index(condition)
            for index, method in enumerate(HandEyeMethod):
                estimate = RigidTransform.from_rodrigues(
                    parent_frame='left_gripper_frame',
                    child_frame='left_wrist_rgb_optical_frame',
                    translation_m=(0.03 + 0.01 * index, -0.02, 0.08),
                    rodrigues_vector=(0.0, 0.0, 0.0),
                )
                rows.append(
                    SolverExperimentRow(
                        condition.condition,
                        seed,
                        method,
                        f'{bundle_value:064x}',
                        True,
                        None,
                        1.0 + index,
                        estimate,
                        0.01 * index,
                        0.001 * index,
                        0.02 * index,
                        0.03 * index,
                        0.002 * index,
                        0.003 * index,
                    )
                )
    summaries = summarize_methods(rows)
    selection = select_method(summaries, rows, candidate_seed=0)
    pnp_diagnostics = tuple(
        PnpExperimentDiagnostic(
            condition.condition,
            seed,
            f'sample_{sample_index:03d}',
            (
                SampleSplit.CALIBRATION
                if sample_index <= 20
                else SampleSplit.HELD_OUT
            ),
            f'{seed + list(NOISE_CONDITIONS).index(condition):064x}',
            True,
            None,
            False,
            0,
            (),
        )
        for condition in NOISE_CONDITIONS
        for seed in config.random_seeds
        for sample_index in range(1, 26)
    )
    return SolverExperimentResult(
        config,
        tuple(rows),
        pnp_diagnostics,
        summaries,
        selection,
    )


def test_candidate_report_writes_150_csv_rows_without_touching_source(
    tmp_path,
):
    source = tmp_path / 'robot.urdf'
    source.write_text('<robot name="unchanged"/>\n', encoding='utf-8')
    before = source.read_bytes()
    solver = _result()
    timestamp = TimestampSensitivityResult(
        HandEyeMethod.TSAI,
        False,
        None,
        (
            TimestampSensitivityRow(
                HandEyeMethod.TSAI,
                0,
                True,
                None,
                20,
                1.0,
                0.0,
                0.0,
            ),
        ),
    )

    paths = write_evaluation_artifacts(
        tmp_path / 'evaluation',
        solver,
        timestamp,
        input_hashes={'urdf': 'a' * 64},
    )

    assert {path.name for path in paths} == {
        'pnp_candidates.jsonl',
        'solver_results.csv',
        'timestamp_sensitivity.csv',
        'metrics.yaml',
        'candidate_transform.yaml',
    }
    assert len((paths[0]).read_text(encoding='utf-8').splitlines()) == 750
    assert len((paths[1]).read_text(encoding='utf-8').splitlines()) == 151
    report = yaml.safe_load((tmp_path / 'evaluation/metrics.yaml').read_text())
    candidate = yaml.safe_load(
        (tmp_path / 'evaluation/candidate_transform.yaml').read_text()
    )
    assert report['artifact_status'] == 'candidate_not_applied'
    assert report['noise']['solver_run_count'] == 150
    assert candidate['selected_method'] == 'tsai'
    assert candidate['promotion_requires_human_review'] is True
    assert source.read_bytes() == before


def test_config_and_ground_truth_are_explicit_and_template_is_not_runnable(
    tmp_path,
):
    config_path = tmp_path / 'evaluation.yaml'
    config_path.write_text(
        yaml.safe_dump(
            {
                'schema_version': 1,
                'random_seeds': list(range(10)),
                'max_translation_norm_m': 1.0,
                'timestamp': {
                    'offsets_ns': [-10_000_000, 0, 10_000_000],
                    'max_joint_sample_distance_ns': 20_000_000,
                },
            }
        ),
        encoding='utf-8',
    )
    truth_path = tmp_path / 'truth.yaml'
    truth_path.write_text(
        yaml.safe_dump(
            {
                'schema_version': 1,
                'evaluation_only': True,
                'gripper_T_camera': transform_to_mapping(TRUTH),
            }
        ),
        encoding='utf-8',
    )

    config = load_evaluation_config(config_path)
    assert config.solver.random_seeds == tuple(range(10))
    assert config.timestamp.offsets_ns == (-10_000_000, 0, 10_000_000)
    assert load_ground_truth(truth_path) == TRUTH

    scene_manifest = (
        Path(__file__).parents[2]
        / 'cleany_mujoco_sim/config/handeye_scene.yaml'
    )
    scene_truth = load_ground_truth(scene_manifest)
    assert scene_truth.parent_frame == 'left_gripper_frame'
    assert scene_truth.child_frame == 'left_wrist_rgb_optical_frame'

    template = (
        Path(__file__).parents[1] / 'config/evaluation.template.yaml'
    )
    with pytest.raises((TypeError, ValueError)):
        load_evaluation_config(template)


def test_dataset_manifest_hash_and_materialized_urdf_must_match(tmp_path):
    body = {
        'schema_version': 1,
        'source_hashes': {
            'urdf_sha256': 'a' * 64,
            'mjcf_sha256': 'b' * 64,
            'pose_manifest_sha256': 'c' * 64,
        },
        'calibration': {'random_seed': 42},
    }
    value = dict(body)
    value['manifest_sha256'] = mapping_sha256(body)
    path = tmp_path / 'manifest.yaml'
    path.write_text(
        json.dumps(value, sort_keys=True, indent=2) + '\n',
        encoding='utf-8',
    )

    assert validate_dataset_manifest(path, urdf_sha256='a' * 64) == value
    with pytest.raises(ValueError, match='URDF differs'):
        validate_dataset_manifest(path, urdf_sha256='d' * 64)

    value['calibration']['random_seed'] = 43
    path.write_text(json.dumps(value) + '\n', encoding='utf-8')
    with pytest.raises(ValueError, match='manifest SHA-256'):
        validate_dataset_manifest(path, urdf_sha256='a' * 64)
