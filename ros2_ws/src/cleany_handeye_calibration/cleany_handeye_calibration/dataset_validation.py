"""Validate a completed stationary dataset and write a review artifact."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
from typing import Callable, Sequence
from uuid import uuid4

import cv2
import numpy as np

from cleany_handeye_calibration.dataset_writer import (
    DatasetWriter,
    mapping_sha256,
    sha256_file,
)
from cleany_handeye_calibration.evaluation_artifacts import (
    load_ground_truth,
    validate_dataset_manifest,
)
from cleany_handeye_calibration.experiment_evaluation import (
    EXPECTED_SOLVER_RUN_COUNT,
    VALID_RESULT_RATE_MIN,
    ExperimentConfig,
    SolverExperimentResult,
    load_sample_records,
    run_solver_experiment,
)
from cleany_handeye_calibration.multi_pose_runtime import (
    validate_multi_pose_runtime_profile,
)
from cleany_handeye_calibration.offline_fk import UrdfOfflineFk
from cleany_handeye_calibration.pnp import (
    AMBIGUITY_RATIO_MIN,
    PnpResult,
    solve_planar_pnp,
)
from cleany_handeye_calibration.pose_manifest import load_pose_manifest
from cleany_handeye_calibration.schema import (
    CalibrationSampleRecord,
    transform_to_mapping,
)
from cleany_handeye_calibration.single_pose_runtime_config import (
    load_single_pose_runtime_config,
)
from cleany_handeye_calibration.target_detector import (
    CharucoDetection,
    CharucoTargetDetector,
    QUADRANTS,
)
from cleany_handeye_calibration.validation import transform_error_metrics


REPORT_SCHEMA = 'cleany.handeye_dataset_validation/v1'
PnpSolver = Callable[..., PnpResult]


@dataclass(frozen=True, slots=True)
class RenderedSampleValidation:
    sample_count: int
    minimum_corner_count: int
    maximum_corner_count: int
    minimum_reprojection_rmse_px: float
    maximum_reprojection_rmse_px: float
    minimum_candidate_rmse_ratio: float | None


def _selected_rmse(result: PnpResult) -> float:
    if result.selected_candidate_index is None:
        raise ValueError('PnP result has no selected candidate index')
    selected = next(
        (
            candidate
            for candidate in result.candidates
            if candidate.index == result.selected_candidate_index
        ),
        None,
    )
    if (
        selected is None
        or selected.refined_reprojection_rmse_px is None
        or not math.isfinite(selected.refined_reprojection_rmse_px)
    ):
        raise ValueError('selected PnP candidate has no finite RMSE')
    return float(selected.refined_reprojection_rmse_px)


def _candidate_ratio(result: PnpResult) -> float | None:
    values = sorted(
        float(candidate.refined_reprojection_rmse_px)
        for candidate in result.candidates
        if candidate.valid
        and candidate.refined_reprojection_rmse_px is not None
        and math.isfinite(candidate.refined_reprojection_rmse_px)
    )
    if len(values) < 2 or values[0] <= 1.0e-12:
        return None
    return values[1] / values[0]


def _same_detection(
    actual: CharucoDetection,
    recorded: CharucoDetection,
) -> bool:
    return (
        actual.corner_ids == recorded.corner_ids
        and actual.covered_quadrants == recorded.covered_quadrants
        and np.array_equal(
            actual.image_points_array(),
            recorded.image_points_array(),
        )
        and np.array_equal(
            actual.object_points_array(),
            recorded.object_points_array(),
        )
    )


def validate_rendered_samples(
    records: Sequence[CalibrationSampleRecord],
    run_directory: str | Path,
    *,
    detector=None,
    pnp_solver: PnpSolver = solve_planar_pnp,
) -> RenderedSampleValidation:
    """Decode every committed PNG and reproduce detection and clean PnP."""

    values = tuple(records)
    if not values:
        raise ValueError('stationary dataset must not be empty')
    root = Path(run_directory).expanduser().resolve(strict=True)
    target_detector = detector or CharucoTargetDetector()
    corner_counts: list[int] = []
    reprojection_errors: list[float] = []
    candidate_ratios: list[float] = []
    for record in values:
        path = root / record.image_path
        if path.is_symlink() or path.parent.is_symlink():
            raise ValueError(
                f'{record.sample.sample_id}: image path is a symlink'
            )
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        expected_shape = (
            record.camera_info.height,
            record.camera_info.width,
            3,
        )
        if image is None or image.shape != expected_shape:
            raise ValueError(
                f'{record.sample.sample_id}: committed PNG cannot be '
                'decoded with the recorded dimensions'
            )
        detection = target_detector.detect(image)
        if not detection.valid:
            reason = (
                'unknown'
                if detection.failure_reason is None
                else detection.failure_reason.value
            )
            raise ValueError(
                f'{record.sample.sample_id}: saved image ChArUco '
                f're-detection failed: {reason}'
            )
        if detection.covered_quadrants != QUADRANTS:
            raise ValueError(
                f'{record.sample.sample_id}: saved image does not cover '
                'all target quadrants'
            )
        if not _same_detection(detection, record.target_detection):
            raise ValueError(
                f'{record.sample.sample_id}: saved image detection differs '
                'from the committed correspondence record'
            )
        result = pnp_solver(
            detection,
            camera_matrix=np.asarray(record.camera_info.k).reshape(3, 3),
            distortion_coefficients=record.camera_info.d,
            camera_frame=record.camera_info.frame_id,
            target_frame='charuco_target',
        )
        if not result.valid or result.ambiguous:
            reason = (
                'unknown'
                if result.failure_reason is None
                else result.failure_reason.value
            )
            raise ValueError(
                f'{record.sample.sample_id}: clean PnP revalidation '
                f'failed: {reason}'
            )
        if result.camera_T_target is None:
            raise ValueError(
                f'{record.sample.sample_id}: valid PnP has no transform'
            )
        transform_error = transform_error_metrics(
            result.camera_T_target,
            record.sample.camera_T_target,
        )
        if (
            result.selected_candidate_index
            != record.pnp_selected_candidate_index
            or transform_error.translation_error_m > 1.0e-12
            or transform_error.rotation_error_rad > 1.0e-12
        ):
            raise ValueError(
                f'{record.sample.sample_id}: recomputed PnP pose differs '
                'from the committed record'
            )
        rmse = _selected_rmse(result)
        if not math.isclose(
            rmse,
            record.pnp_reprojection_rmse_px,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise ValueError(
                f'{record.sample.sample_id}: recomputed PnP RMSE differs '
                'from the committed record'
            )
        corner_counts.append(len(detection.corner_ids))
        reprojection_errors.append(rmse)
        ratio = _candidate_ratio(result)
        if ratio is not None:
            candidate_ratios.append(ratio)
    return RenderedSampleValidation(
        sample_count=len(values),
        minimum_corner_count=min(corner_counts),
        maximum_corner_count=max(corner_counts),
        minimum_reprojection_rmse_px=min(reprojection_errors),
        maximum_reprojection_rmse_px=max(reprojection_errors),
        minimum_candidate_rmse_ratio=(
            None if not candidate_ratios else min(candidate_ratios)
        ),
    )


def _method_summaries(result: SolverExperimentResult) -> list[dict]:
    return [
        {
            'method': item.method.value,
            'valid_runs': item.valid_runs,
            'total_runs': item.total_runs,
            'valid_result_rate': item.valid_result_rate,
            'translation_median_m': item.translation_median_m,
            'translation_p95_m': item.translation_p95_m,
            'rotation_median_rad': item.rotation_median_rad,
            'rotation_p95_rad': item.rotation_p95_rad,
            'held_out_translation_median_m': (
                item.held_out_translation_median_m
            ),
            'held_out_translation_p95_m': (
                item.held_out_translation_p95_m
            ),
            'held_out_rotation_median_rad': (
                item.held_out_rotation_median_rad
            ),
            'held_out_rotation_p95_rad': item.held_out_rotation_p95_rad,
            'runtime_median_ms': item.runtime_median_ms,
            'runtime_p95_ms': item.runtime_p95_ms,
            'failure_counts': dict(item.failure_counts),
        }
        for item in result.method_summaries
    ]


def _report_mapping(
    *,
    input_hashes: dict[str, str],
    rendered: RenderedSampleValidation,
    solver: SolverExperimentResult,
    calibration_count: int,
    held_out_count: int,
    pose_diversity: dict,
) -> dict:
    selection = solver.selection
    body = {
        'schema_version': REPORT_SCHEMA,
        'artifact_status': 'validation_only_not_applied',
        'dataset_status': 'valid',
        'input_sha256': dict(sorted(input_hashes.items())),
        'stationary_samples': {
            'total': rendered.sample_count,
            'calibration': calibration_count,
            'held_out': held_out_count,
            'all_rows_images_and_hashes_valid': True,
            'all_saved_images_redetected_exactly': True,
            'minimum_charuco_corner_count': rendered.minimum_corner_count,
            'maximum_charuco_corner_count': rendered.maximum_corner_count,
            'minimum_pnp_reprojection_rmse_px': (
                rendered.minimum_reprojection_rmse_px
            ),
            'maximum_pnp_reprojection_rmse_px': (
                rendered.maximum_reprojection_rmse_px
            ),
            'minimum_pnp_candidate_rmse_ratio': (
                rendered.minimum_candidate_rmse_ratio
            ),
        },
        'pose_diversity': pose_diversity,
        'solver_experiment': {
            'solver_run_count': len(solver.rows),
            'pnp_diagnostic_count': len(solver.pnp_diagnostics),
            'random_seeds': list(solver.config.random_seeds),
            'max_translation_norm_m': (
                solver.config.max_translation_norm_m
            ),
            'required_valid_result_rate': VALID_RESULT_RATE_MIN,
            'pnp_ambiguity_ratio_minimum': AMBIGUITY_RATIO_MIN,
            'method_summaries': _method_summaries(solver),
            'selection': {
                'status': selection.status.value,
                'selected_method': (
                    None
                    if selection.selected_method is None
                    else selection.selected_method.value
                ),
                'reason': selection.reason,
                'candidate_condition': (
                    None
                    if selection.candidate_condition is None
                    else selection.candidate_condition.value
                ),
                'candidate_seed': selection.candidate_seed,
                'candidate_transform': (
                    None
                    if selection.candidate_transform is None
                    else transform_to_mapping(
                        selection.candidate_transform
                    )
                ),
                'promotion_requires_human_review': True,
            },
        },
    }
    if len(solver.rows) != EXPECTED_SOLVER_RUN_COUNT:
        raise ValueError('solver experiment did not produce exactly 150 rows')
    body['report_sha256'] = mapping_sha256(body)
    return body


def _atomic_write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.parent.is_symlink() or path.is_symlink():
        raise ValueError('validation output path must not be a symlink')
    payload = (
        json.dumps(
            value,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + '\n'
    ).encode('utf-8')
    temporary = path.with_name(f'.{path.name}.{uuid4().hex}.tmp')
    descriptor = os.open(
        temporary,
        os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_CLOEXEC,
        0o600,
    )
    try:
        offset = 0
        while offset < len(payload):
            offset += os.write(descriptor, payload[offset:])
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    os.replace(temporary, path)
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def validate_dataset(
    *,
    samples_path: str | Path,
    pose_manifest_path: str | Path,
    runtime_config_path: str | Path,
    urdf_path: str | Path,
    ground_truth_path: str | Path,
    max_translation_norm_m: float,
    output_path: str | Path,
) -> Path:
    samples = Path(samples_path).expanduser().resolve(strict=True)
    pose_path = Path(pose_manifest_path).expanduser().resolve(strict=True)
    runtime_path = Path(runtime_config_path).expanduser().resolve(strict=True)
    urdf = Path(urdf_path).expanduser().resolve(strict=True)
    ground_truth = Path(ground_truth_path).expanduser().resolve(strict=True)
    output = Path(output_path).expanduser().resolve()
    runtime = load_single_pose_runtime_config(runtime_path)
    manifest = load_pose_manifest(pose_path)
    validate_multi_pose_runtime_profile(pose_path, manifest, runtime)
    writer = DatasetWriter(
        artifact_root=runtime.artifact_root,
        manifest=runtime.dataset_manifest,
    )
    if writer.samples_path.resolve() != samples:
        raise ValueError(
            'samples path does not belong to the runtime dataset manifest'
        )
    stored = writer.read_samples()
    records = load_sample_records(samples)
    if tuple(item.record for item in stored) != records:
        raise ValueError('dataset writer and evaluation loader rows differ')
    expected_poses = {
        (pose.pose_id, pose.split.value) for pose in manifest.poses
    }
    actual_poses = {
        (record.sample.pose_id, record.sample.split.value)
        for record in records
    }
    if actual_poses != expected_poses:
        raise ValueError(
            'dataset pose IDs or splits differ from the pose manifest'
        )
    validate_dataset_manifest(
        writer.manifest_path,
        urdf_sha256=sha256_file(urdf),
    )
    rendered = validate_rendered_samples(records, writer.run_directory)
    solver = run_solver_experiment(
        records,
        UrdfOfflineFk(urdf),
        ExperimentConfig(
            random_seeds=tuple(range(10)),
            max_translation_norm_m=max_translation_norm_m,
        ),
        ground_truth=load_ground_truth(ground_truth),
    )
    diversity = manifest.selection.diversity
    report = _report_mapping(
        input_hashes={
            'samples_jsonl': sha256_file(samples),
            'dataset_manifest': sha256_file(writer.manifest_path),
            'pose_manifest': sha256_file(pose_path),
            'runtime_config': sha256_file(runtime_path),
            'urdf': sha256_file(urdf),
            'ground_truth': sha256_file(ground_truth),
        },
        rendered=rendered,
        solver=solver,
        calibration_count=sum(
            record.sample.split.value == 'calibration'
            for record in records
        ),
        held_out_count=sum(
            record.sample.split.value == 'held_out'
            for record in records
        ),
        pose_diversity={
            'maximum_axis_parallelism': (
                diversity.maximum_axis_parallelism
            ),
            'rotation_covariance_log_det': (
                diversity.rotation_covariance_log_det
            ),
            'rotation_covariance_rank': diversity.rotation_covariance_rank,
            'nonparallel_axis_pose_ids': list(
                diversity.nonparallel_axis_pose_ids
            ),
        },
    )
    _atomic_write_json(output, report)
    return output


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Validate a complete stationary 20+5 dataset and write a '
            'review-only JSON report.'
        )
    )
    parser.add_argument('--samples', required=True)
    parser.add_argument('--pose-manifest', required=True)
    parser.add_argument('--runtime-config', required=True)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--ground-truth', required=True)
    parser.add_argument('--max-translation-norm-m', required=True, type=float)
    parser.add_argument('--output', required=True)
    return parser


def main(argv=None) -> None:
    try:
        values = vars(_parser().parse_args(argv))
        output = validate_dataset(
            samples_path=values['samples'],
            pose_manifest_path=values['pose_manifest'],
            runtime_config_path=values['runtime_config'],
            urdf_path=values['urdf'],
            ground_truth_path=values['ground_truth'],
            max_translation_norm_m=values['max_translation_norm_m'],
            output_path=values['output'],
        )
    except Exception as error:
        print(
            'hand-eye dataset validation failed: '
            f'{type(error).__name__}: {error}'
        )
        raise SystemExit(2) from error
    print(output)


if __name__ == '__main__':
    main()


__all__ = [
    'REPORT_SCHEMA',
    'RenderedSampleValidation',
    'validate_dataset',
    'validate_rendered_samples',
]
