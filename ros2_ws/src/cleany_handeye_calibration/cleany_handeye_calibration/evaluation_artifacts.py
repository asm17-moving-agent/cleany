"""Strict experiment config and candidate-only CSV/YAML artifacts."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
from typing import Any, Mapping
from uuid import uuid4

import yaml

from cleany_handeye_calibration.experiment_evaluation import (
    EXPERIMENT_SCHEMA_VERSION,
    ExperimentConfig,
    NOISE_CONDITIONS,
    SolverExperimentResult,
)
from cleany_handeye_calibration.schema import (
    transform_from_mapping,
    transform_to_mapping,
)
from cleany_handeye_calibration.timestamp_sensitivity import (
    TimestampSensitivityConfig,
    TimestampSensitivityResult,
)
from cleany_handeye_calibration.transforms import RigidTransform


REPORT_SCHEMA_VERSION = 1
_SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ValueError(f'duplicate YAML key: {key!r}')
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_mapping,
)


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def validate_dataset_manifest(
    path: str | Path,
    *,
    urdf_sha256: str,
) -> Mapping[str, Any]:
    """Verify the committed dataset provenance used by the evaluator."""

    manifest_path = Path(path).expanduser().resolve(strict=True)
    try:
        value = json.loads(manifest_path.read_text(encoding='utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(
            f'dataset manifest is not strict JSON: {error}'
        ) from error
    if not isinstance(value, dict):
        raise ValueError('dataset manifest must be an object')
    stored_hash = value.get('manifest_sha256')
    body = dict(value)
    body.pop('manifest_sha256', None)
    canonical = json.dumps(
        body,
        sort_keys=True,
        separators=(',', ':'),
        ensure_ascii=True,
        allow_nan=False,
    ).encode('ascii')
    if stored_hash != hashlib.sha256(canonical).hexdigest():
        raise ValueError('dataset manifest SHA-256 does not match')
    source_hashes = value.get('source_hashes')
    calibration = value.get('calibration')
    if not isinstance(source_hashes, Mapping) or not isinstance(
        calibration,
        Mapping,
    ):
        raise ValueError('dataset manifest lacks provenance mappings')
    for name in ('urdf_sha256', 'mjcf_sha256', 'pose_manifest_sha256'):
        digest = source_hashes.get(name)
        if (
            not isinstance(digest, str)
            or not _SHA256_PATTERN.fullmatch(digest)
        ):
            raise ValueError(f'dataset manifest {name} is invalid')
    if source_hashes['urdf_sha256'] != urdf_sha256:
        raise ValueError('materialized URDF differs from dataset manifest')
    seed = calibration.get('random_seed')
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError('dataset manifest random seed is invalid')
    return value


@dataclass(frozen=True, slots=True)
class EvaluationRunConfig:
    solver: ExperimentConfig
    timestamp: TimestampSensitivityConfig


def _exact_keys(
    value: Mapping[str, Any],
    expected: set[str],
    name: str,
) -> None:
    if not isinstance(value, Mapping) or set(value) != expected:
        raise ValueError(f'{name} must contain exactly {sorted(expected)}')


def load_evaluation_config(path: str | Path) -> EvaluationRunConfig:
    config_path = Path(path).expanduser().resolve(strict=True)
    try:
        value = yaml.load(
            config_path.read_text(encoding='utf-8'),
            Loader=_UniqueKeyLoader,
        )
    except yaml.YAMLError as error:
        raise ValueError(f'invalid evaluation YAML: {error}') from error
    _exact_keys(
        value,
        {
            'schema_version',
            'random_seeds',
            'max_translation_norm_m',
            'timestamp',
        },
        'evaluation config',
    )
    if value['schema_version'] != EXPERIMENT_SCHEMA_VERSION:
        raise ValueError('unsupported evaluation config schema_version')
    timestamp = value['timestamp']
    _exact_keys(
        timestamp,
        {'offsets_ns', 'max_joint_sample_distance_ns'},
        'timestamp config',
    )
    random_seeds = value['random_seeds']
    offsets = timestamp['offsets_ns']
    if not isinstance(random_seeds, list):
        raise ValueError('random_seeds must be a YAML sequence')
    if not isinstance(offsets, list):
        raise ValueError('timestamp offsets_ns must be a YAML sequence')
    solver = ExperimentConfig(
        random_seeds=tuple(random_seeds),
        max_translation_norm_m=value['max_translation_norm_m'],
    )
    return EvaluationRunConfig(
        solver=solver,
        timestamp=TimestampSensitivityConfig(
            offsets_ns=tuple(offsets),
            max_joint_sample_distance_ns=timestamp[
                'max_joint_sample_distance_ns'
            ],
            max_translation_norm_m=solver.max_translation_norm_m,
        ),
    )


def load_ground_truth(path: str | Path) -> RigidTransform:
    truth_path = Path(path).expanduser().resolve(strict=True)
    try:
        value = yaml.load(
            truth_path.read_text(encoding='utf-8'),
            Loader=_UniqueKeyLoader,
        )
    except yaml.YAMLError as error:
        raise ValueError(f'invalid ground-truth YAML: {error}') from error
    if not isinstance(value, Mapping):
        raise ValueError('ground-truth artifact must be a mapping')
    if 'evaluation_ground_truth' in value:
        evaluation = value['evaluation_ground_truth']
        if not isinstance(evaluation, Mapping):
            raise ValueError('evaluation_ground_truth must be a mapping')
        camera = evaluation.get('camera_transform')
        if not isinstance(camera, Mapping):
            raise ValueError('camera_transform must be a mapping')
        if (
            camera.get('semantics')
            != 'left_gripper_T_left_wrist_rgb_optical'
            or camera.get('evaluation_only') is not True
            or camera.get('allowed_for_solver_input') is not False
            or camera.get('published_to_tf') is not False
        ):
            raise ValueError(
                'scene camera ground truth is not evaluation-only'
            )
        transform = RigidTransform.from_quaternion_xyzw(
            parent_frame=camera.get('parent_frame'),
            child_frame=camera.get('child_frame'),
            translation_m=camera.get('translation_m'),
            quaternion_xyzw=camera.get('quaternion_xyzw'),
        )
    else:
        _exact_keys(
            value,
            {'schema_version', 'evaluation_only', 'gripper_T_camera'},
            'ground-truth artifact',
        )
        if (
            value['schema_version'] != 1
            or value['evaluation_only'] is not True
        ):
            raise ValueError(
                'ground truth must be schema 1 and evaluation_only'
            )
        transform = transform_from_mapping(value['gripper_T_camera'])
    if (
        transform.parent_frame != 'left_gripper_frame'
        or transform.child_frame != 'left_wrist_rgb_optical_frame'
    ):
        raise ValueError('ground truth has the wrong transform direction')
    return transform


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.parent.is_symlink() or path.is_symlink():
        raise ValueError('evaluation output paths must not be symlinks')
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


def _csv_bytes(
    fieldnames: tuple[str, ...],
    rows: list[dict[str, Any]],
) -> bytes:
    stream = io.StringIO(newline='')
    writer = csv.DictWriter(
        stream,
        fieldnames=fieldnames,
        lineterminator='\n',
    )
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode('utf-8')


def _finite_or_blank(value: float | None) -> float | str:
    if value is None:
        return ''
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('report metrics must be finite')
    return result


def _solver_csv(result: SolverExperimentResult) -> bytes:
    fields = (
        'condition',
        'random_seed',
        'method',
        'noise_bundle_sha256',
        'valid',
        'failure_reason',
        'runtime_ms',
        'translation_error_m',
        'rotation_error_rad',
        'held_out_translation_median_m',
        'held_out_translation_p95_m',
        'held_out_rotation_median_rad',
        'held_out_rotation_p95_rad',
        'estimate_translation_m',
        'estimate_quaternion_xyzw',
    )
    rows = []
    for row in result.rows:
        estimate_translation = ''
        estimate_quaternion = ''
        if row.gripper_T_camera is not None:
            estimate_translation = json_list(
                row.gripper_T_camera.translation_m
            )
            estimate_quaternion = json_list(
                row.gripper_T_camera.as_quaternion_xyzw()
            )
        rows.append(
            {
                'condition': row.condition.value,
                'random_seed': row.random_seed,
                'method': row.method.value,
                'noise_bundle_sha256': row.noise_bundle_sha256,
                'valid': str(row.valid).lower(),
                'failure_reason': row.failure_reason or '',
                'runtime_ms': row.runtime_ms,
                'translation_error_m': _finite_or_blank(
                    row.translation_error_m
                ),
                'rotation_error_rad': _finite_or_blank(
                    row.rotation_error_rad
                ),
                'held_out_translation_median_m': _finite_or_blank(
                    row.held_out_translation_median_m
                ),
                'held_out_translation_p95_m': _finite_or_blank(
                    row.held_out_translation_p95_m
                ),
                'held_out_rotation_median_rad': _finite_or_blank(
                    row.held_out_rotation_median_rad
                ),
                'held_out_rotation_p95_rad': _finite_or_blank(
                    row.held_out_rotation_p95_rad
                ),
                'estimate_translation_m': estimate_translation,
                'estimate_quaternion_xyzw': estimate_quaternion,
            }
        )
    return _csv_bytes(fields, rows)


def json_list(values) -> str:
    return json.dumps(list(values), separators=(',', ':'), allow_nan=False)


def _optional_transform(transform: RigidTransform | None):
    return None if transform is None else transform_to_mapping(transform)


def _pnp_jsonl(result: SolverExperimentResult) -> bytes:
    lines = []
    for item in result.pnp_diagnostics:
        value = {
            'schema_version': 1,
            'condition': item.condition.value,
            'random_seed': item.random_seed,
            'sample_id': item.sample_id,
            'split': item.split.value,
            'noise_bundle_sha256': item.noise_bundle_sha256,
            'valid': item.valid,
            'failure_reason': item.failure_reason,
            'ambiguous': item.ambiguous,
            'selected_candidate_index': item.selected_candidate_index,
            'candidates': [
                {
                    'index': candidate.index,
                    'valid': candidate.valid,
                    'failure_reason': (
                        None
                        if candidate.failure_reason is None
                        else candidate.failure_reason.value
                    ),
                    'raw_camera_T_target': _optional_transform(
                        candidate.raw_camera_T_target
                    ),
                    'raw_min_depth_m': candidate.raw_min_depth_m,
                    'raw_reprojection_rmse_px': (
                        candidate.raw_reprojection_rmse_px
                    ),
                    'refined_camera_T_target': _optional_transform(
                        candidate.refined_camera_T_target
                    ),
                    'refined_min_depth_m': candidate.refined_min_depth_m,
                    'refined_reprojection_rmse_px': (
                        candidate.refined_reprojection_rmse_px
                    ),
                }
                for candidate in item.candidates
            ],
        }
        lines.append(
            json.dumps(
                value,
                sort_keys=True,
                separators=(',', ':'),
                ensure_ascii=True,
                allow_nan=False,
            ).encode('ascii')
            + b'\n'
        )
    return b''.join(lines)


def _timestamp_csv(result: TimestampSensitivityResult) -> bytes:
    fields = (
        'method',
        'offset_ns',
        'valid',
        'failure_reason',
        'sample_count',
        'runtime_ms',
        'translation_error_m',
        'rotation_error_rad',
    )
    rows = [
        {
            'method': row.method.value,
            'offset_ns': row.offset_ns,
            'valid': str(row.valid).lower(),
            'failure_reason': row.failure_reason or '',
            'sample_count': row.sample_count,
            'runtime_ms': row.runtime_ms,
            'translation_error_m': _finite_or_blank(
                row.translation_error_m
            ),
            'rotation_error_rad': _finite_or_blank(row.rotation_error_rad),
        }
        for row in result.rows
    ]
    return _csv_bytes(fields, rows)


def _summary_mapping(
    solver: SolverExperimentResult,
    timestamp: TimestampSensitivityResult,
    input_hashes: Mapping[str, str],
) -> dict[str, Any]:
    return {
        'schema_version': REPORT_SCHEMA_VERSION,
        'artifact_status': 'candidate_not_applied',
        'input_sha256': dict(sorted(input_hashes.items())),
        'noise': {
            'generator': 'numpy.PCG64.SeedSequence-v1',
            'random_seeds': list(solver.config.random_seeds),
            'solver_run_count': len(solver.rows),
            'pnp_diagnostic_count': len(solver.pnp_diagnostics),
            'conditions': [
                {
                    'name': item.condition.value,
                    'image_point_sigma_px': item.image_point_sigma_px,
                    'joint_position_sigma_rad': (
                        item.joint_position_sigma_rad
                    ),
                }
                for item in NOISE_CONDITIONS
            ],
        },
        'methods': [
            {
                'method': item.method.value,
                'total_runs': item.total_runs,
                'valid_runs': item.valid_runs,
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
            for item in solver.method_summaries
        ],
        'selection': {
            'status': solver.selection.status.value,
            'selected_method': (
                None
                if solver.selection.selected_method is None
                else solver.selection.selected_method.value
            ),
            'reason': solver.selection.reason,
            'candidate_source': (
                None
                if solver.selection.candidate_condition is None
                else {
                    'condition': solver.selection.candidate_condition.value,
                    'random_seed': solver.selection.candidate_seed,
                }
            ),
        },
        'timestamp_sensitivity': {
            'review_required': timestamp.review_required,
            'selected_method': (
                None
                if timestamp.selected_method is None
                else timestamp.selected_method.value
            ),
            'reason': timestamp.reason,
            'row_count': len(timestamp.rows),
        },
    }


def write_evaluation_artifacts(
    output_directory: str | Path,
    solver: SolverExperimentResult,
    timestamp: TimestampSensitivityResult,
    *,
    input_hashes: Mapping[str, str],
) -> tuple[Path, ...]:
    """Write only candidate artifacts; never mutate URDF or profiles."""

    directory = Path(output_directory).expanduser().resolve()
    if directory.exists() and (
        not directory.is_dir() or directory.is_symlink()
    ):
        raise ValueError('output_directory must be a non-symlink directory')
    directory.mkdir(parents=True, exist_ok=True)
    if len(solver.rows) != 150:
        raise ValueError('solver experiment must contain exactly 150 rows')
    if len(solver.pnp_diagnostics) != 750:
        raise ValueError(
            'solver experiment must contain exactly 750 PnP diagnostics'
        )
    if any(
        not isinstance(key, str)
        or not isinstance(value, str)
        or not _SHA256_PATTERN.fullmatch(value)
        for key, value in input_hashes.items()
    ):
        raise ValueError('input_hashes must contain SHA-256 text values')

    pnp_jsonl = directory / 'pnp_candidates.jsonl'
    solver_csv = directory / 'solver_results.csv'
    timestamp_csv = directory / 'timestamp_sensitivity.csv'
    report_yaml = directory / 'metrics.yaml'
    candidate_yaml = directory / 'candidate_transform.yaml'
    _atomic_write(pnp_jsonl, _pnp_jsonl(solver))
    _atomic_write(solver_csv, _solver_csv(solver))
    _atomic_write(timestamp_csv, _timestamp_csv(timestamp))
    report = yaml.safe_dump(
        _summary_mapping(solver, timestamp, input_hashes),
        sort_keys=False,
        allow_unicode=False,
    ).encode('ascii')
    _atomic_write(report_yaml, report)
    candidate = {
        'schema_version': 1,
        'artifact_status': 'candidate_not_applied',
        'selection_status': solver.selection.status.value,
        'selected_method': (
            None
            if solver.selection.selected_method is None
            else solver.selection.selected_method.value
        ),
        'source': (
            None
            if solver.selection.candidate_condition is None
            else {
                'condition': solver.selection.candidate_condition.value,
                'random_seed': solver.selection.candidate_seed,
            }
        ),
        'gripper_T_camera': (
            None
            if solver.selection.candidate_transform is None
            else transform_to_mapping(solver.selection.candidate_transform)
        ),
        'promotion_requires_human_review': True,
    }
    _atomic_write(
        candidate_yaml,
        yaml.safe_dump(
            candidate,
            sort_keys=False,
            allow_unicode=False,
        ).encode('ascii'),
    )
    return (
        pnp_jsonl,
        solver_csv,
        timestamp_csv,
        report_yaml,
        candidate_yaml,
    )


__all__ = [
    'EvaluationRunConfig',
    'REPORT_SCHEMA_VERSION',
    'load_evaluation_config',
    'load_ground_truth',
    'sha256_file',
    'validate_dataset_manifest',
    'write_evaluation_artifacts',
]
