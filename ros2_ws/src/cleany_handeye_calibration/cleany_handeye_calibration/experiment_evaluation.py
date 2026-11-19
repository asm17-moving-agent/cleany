"""Deterministic 150-run hand-eye accuracy/stability experiment."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from enum import Enum
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from cleany_handeye_calibration.joint_state_sync import LEFT_ARM_JOINT_NAMES
from cleany_handeye_calibration.models import CalibrationSample, SampleSplit
from cleany_handeye_calibration.offline_fk import FeedbackFkPort
from cleany_handeye_calibration.pnp import (
    PnpCandidate,
    PnpResult,
    solve_planar_pnp,
)
from cleany_handeye_calibration.schema import (
    CalibrationSampleRecord,
    sample_record_from_mapping,
)
from cleany_handeye_calibration.solver import (
    HAND_EYE_METHOD_REGISTRY,
    HandEyeFailure,
    HandEyeMethod,
    HandEyeResult,
    HandEyeTransformValidityPolicy,
    solve_all_hand_eye_methods,
)
from cleany_handeye_calibration.target_detector import CharucoDetection
from cleany_handeye_calibration.transforms import RigidTransform
from cleany_handeye_calibration.validation import (
    held_out_base_target_consistency,
    transform_error_metrics,
)


EXPERIMENT_SCHEMA_VERSION = 1
EXPECTED_CALIBRATION_COUNT = 20
EXPECTED_HELD_OUT_COUNT = 5
MINIMUM_PARTIAL_CALIBRATION_COUNT = 5
MINIMUM_PARTIAL_HELD_OUT_COUNT = 1
EXPECTED_METHOD_COUNT = 5
EXPECTED_SEED_COUNT = 10
EXPECTED_SOLVER_RUN_COUNT = 150
VALID_RESULT_RATE_MIN = 0.95
DEGREES_TO_RADIANS = math.pi / 180.0
_SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')


class NoiseCondition(str, Enum):
    IDEAL = 'ideal'
    NOMINAL = 'nominal'
    STRESS = 'stress'


@dataclass(frozen=True, slots=True)
class NoiseConditionSpec:
    condition: NoiseCondition
    image_point_sigma_px: float
    joint_position_sigma_rad: float


NOISE_CONDITIONS = (
    NoiseConditionSpec(NoiseCondition.IDEAL, 0.0, 0.0),
    NoiseConditionSpec(
        NoiseCondition.NOMINAL,
        0.5,
        0.05 * DEGREES_TO_RADIANS,
    ),
    NoiseConditionSpec(
        NoiseCondition.STRESS,
        1.0,
        0.10 * DEGREES_TO_RADIANS,
    ),
)


@dataclass(frozen=True, slots=True)
class ExperimentConfig:
    random_seeds: tuple[int, ...]
    max_translation_norm_m: float

    def __post_init__(self) -> None:
        seeds = tuple(self.random_seeds)
        if (
            len(seeds) != EXPECTED_SEED_COUNT
            or len(set(seeds)) != EXPECTED_SEED_COUNT
            or any(
                isinstance(seed, bool)
                or not isinstance(seed, int)
                or seed < 0
                for seed in seeds
            )
        ):
            raise ValueError('random_seeds must contain 10 unique integers')
        try:
            maximum = float(self.max_translation_norm_m)
        except (TypeError, ValueError) as error:
            raise ValueError(
                'max_translation_norm_m must be positive and finite'
            ) from error
        if not math.isfinite(maximum) or maximum <= 0.0:
            raise ValueError(
                'max_translation_norm_m must be positive and finite'
            )
        object.__setattr__(self, 'random_seeds', seeds)
        object.__setattr__(self, 'max_translation_norm_m', maximum)


@dataclass(frozen=True, slots=True)
class PreparedNoiseDataset:
    condition: NoiseCondition
    random_seed: int
    noise_bundle_sha256: str
    calibration_samples: tuple[CalibrationSample, ...]
    held_out_samples: tuple[CalibrationSample, ...]
    pnp_diagnostics: tuple[PnpExperimentDiagnostic, ...]
    valid: bool
    failure_reason: str | None


@dataclass(frozen=True, slots=True)
class PnpExperimentDiagnostic:
    condition: NoiseCondition
    random_seed: int
    sample_id: str
    split: SampleSplit
    noise_bundle_sha256: str
    valid: bool
    failure_reason: str | None
    ambiguous: bool
    selected_candidate_index: int | None
    candidates: tuple[PnpCandidate, ...]


@dataclass(frozen=True, slots=True)
class SolverExperimentRow:
    condition: NoiseCondition
    random_seed: int
    method: HandEyeMethod
    noise_bundle_sha256: str
    valid: bool
    failure_reason: str | None
    runtime_ms: float
    gripper_T_camera: RigidTransform | None
    translation_error_m: float | None
    rotation_error_rad: float | None
    held_out_translation_median_m: float | None
    held_out_translation_p95_m: float | None
    held_out_rotation_median_rad: float | None
    held_out_rotation_p95_rad: float | None


@dataclass(frozen=True, slots=True)
class MethodExperimentSummary:
    method: HandEyeMethod
    total_runs: int
    valid_runs: int
    valid_result_rate: float
    translation_median_m: float | None
    translation_p95_m: float | None
    rotation_median_rad: float | None
    rotation_p95_rad: float | None
    held_out_translation_median_m: float | None
    held_out_translation_p95_m: float | None
    held_out_rotation_median_rad: float | None
    held_out_rotation_p95_rad: float | None
    runtime_median_ms: float | None
    runtime_p95_ms: float | None
    failure_counts: tuple[tuple[str, int], ...]

    @property
    def accuracy_vector(self) -> tuple[float, ...] | None:
        values = (
            self.translation_median_m,
            self.translation_p95_m,
            self.rotation_median_rad,
            self.rotation_p95_rad,
        )
        if any(value is None for value in values):
            return None
        return tuple(float(value) for value in values if value is not None)

    @property
    def held_out_vector(self) -> tuple[float, ...] | None:
        values = (
            self.held_out_translation_median_m,
            self.held_out_translation_p95_m,
            self.held_out_rotation_median_rad,
            self.held_out_rotation_p95_rad,
        )
        if any(value is None for value in values):
            return None
        return tuple(float(value) for value in values if value is not None)


class SelectionStatus(str, Enum):
    SELECTED = 'selected'
    REVIEW_REQUIRED = 'review_required'


@dataclass(frozen=True, slots=True)
class MethodSelection:
    status: SelectionStatus
    selected_method: HandEyeMethod | None
    reason: str
    candidate_transform: RigidTransform | None
    candidate_condition: NoiseCondition | None
    candidate_seed: int | None


@dataclass(frozen=True, slots=True)
class SolverExperimentResult:
    config: ExperimentConfig
    rows: tuple[SolverExperimentRow, ...]
    pnp_diagnostics: tuple[PnpExperimentDiagnostic, ...]
    method_summaries: tuple[MethodExperimentSummary, ...]
    selection: MethodSelection


PnpSolver = Callable[..., PnpResult]
AllMethodSolver = Callable[..., tuple[HandEyeResult, ...]]


def load_sample_records(
    samples_jsonl: str | Path,
    *,
    allow_partial: bool = False,
) -> tuple[CalibrationSampleRecord, ...]:
    """Load committed rows, optionally accepting an explicit partial set."""

    path = Path(samples_jsonl).expanduser().resolve(strict=True)
    payload = path.read_bytes()
    if payload and not payload.endswith(b'\n'):
        raise ValueError('samples JSONL has a partial final row')
    records: list[CalibrationSampleRecord] = []
    for line_number, line in enumerate(payload.splitlines(), start=1):
        try:
            value = json.loads(line.decode('utf-8'))
            if not isinstance(value, dict):
                raise ValueError('sample row must be an object')
            image_sha = value['image_sha256']
            source_sha = value['source_image_sha256']
            record_sha = value['record_sha256']
            if any(
                not isinstance(digest, str)
                or not _SHA256_PATTERN.fullmatch(digest)
                for digest in (image_sha, source_sha, record_sha)
            ):
                raise ValueError('sample integrity hashes are invalid')
            hash_body = dict(value)
            hash_body.pop('record_sha256')
            if _canonical_sha256(hash_body) != record_sha:
                raise ValueError('record SHA-256 does not match')
            record = sample_record_from_mapping(value)
            image_path = path.parent / record.image_path
            if (
                image_path.parent.is_symlink()
                or image_path.is_symlink()
                or not image_path.is_file()
            ):
                raise ValueError('committed sample image is missing or unsafe')
            actual_image_sha = hashlib.sha256(
                image_path.read_bytes()
            ).hexdigest()
            if actual_image_sha != image_sha:
                raise ValueError('image SHA-256 does not match')
            records.append(record)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
            raise ValueError(
                f'invalid samples JSONL line {line_number}: {error}'
            ) from error
        except KeyError as error:
            raise ValueError(
                f'invalid samples JSONL line {line_number}: '
                f'missing {error.args[0]}'
            ) from error
    validate_evaluation_records(records, allow_partial=allow_partial)
    return tuple(records)


def validate_evaluation_records(
    records: Sequence[CalibrationSampleRecord],
    *,
    allow_partial: bool = False,
) -> tuple[CalibrationSampleRecord, ...]:
    values = tuple(records)
    if any(not isinstance(item, CalibrationSampleRecord) for item in values):
        raise ValueError('evaluation inputs must be CalibrationSampleRecord')
    calibration = tuple(
        item for item in values if item.sample.split is SampleSplit.CALIBRATION
    )
    held_out = tuple(
        item for item in values if item.sample.split is SampleSplit.HELD_OUT
    )
    if not isinstance(allow_partial, bool):
        raise ValueError('allow_partial must be a bool')
    if allow_partial:
        if (
            not MINIMUM_PARTIAL_CALIBRATION_COUNT
            <= len(calibration)
            <= EXPECTED_CALIBRATION_COUNT
            or not MINIMUM_PARTIAL_HELD_OUT_COUNT
            <= len(held_out)
            <= EXPECTED_HELD_OUT_COUNT
            or len(values) != len(calibration) + len(held_out)
        ):
            raise ValueError(
                'partial evaluation requires 5..20 calibration + '
                '1..5 held_out rows'
            )
    elif (
        len(calibration) != EXPECTED_CALIBRATION_COUNT
        or len(held_out) != EXPECTED_HELD_OUT_COUNT
        or len(values)
        != EXPECTED_CALIBRATION_COUNT + EXPECTED_HELD_OUT_COUNT
    ):
        raise ValueError(
            'evaluation requires exactly 20 calibration + 5 held_out rows'
        )
    sample_ids = tuple(item.sample.sample_id for item in values)
    pose_ids = tuple(item.sample.pose_id for item in values)
    if (
        len(set(sample_ids)) != len(values)
        or len(set(pose_ids)) != len(values)
    ):
        raise ValueError(
            'evaluation rows contain duplicate sample or pose IDs'
        )
    return values


def _canonical_sha256(value: Mapping[str, Any]) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(',', ':'),
        allow_nan=False,
        ensure_ascii=True,
    ).encode('ascii')
    return hashlib.sha256(payload).hexdigest()


def _noise_rng(seed: int, condition_index: int) -> np.random.Generator:
    sequence = np.random.SeedSequence((seed, condition_index))
    return np.random.Generator(np.random.PCG64(sequence))


def _noisy_detection(
    record: CalibrationSampleRecord,
    image_noise_px: np.ndarray,
) -> CharucoDetection:
    points = record.target_detection.image_points_array() + image_noise_px
    return CharucoDetection(
        valid=True,
        failure_reason=None,
        corner_ids=record.target_detection.corner_ids,
        image_points_px=tuple(
            (float(point[0]), float(point[1])) for point in points
        ),
        object_points_m=record.target_detection.object_points_m,
        covered_quadrants=record.target_detection.covered_quadrants,
    )


def prepare_noise_dataset(
    records: Sequence[CalibrationSampleRecord],
    condition: NoiseConditionSpec,
    random_seed: int,
    fk_port: FeedbackFkPort,
    *,
    pnp_solver: PnpSolver = solve_planar_pnp,
    allow_partial: bool = False,
) -> PreparedNoiseDataset:
    """Apply pixel/q noise, rerun PnP/FK, and never perturb transforms."""

    values = validate_evaluation_records(
        records,
        allow_partial=allow_partial,
    )
    if condition not in NOISE_CONDITIONS:
        raise ValueError('condition must be one of the three fixed patterns')
    if isinstance(random_seed, bool) or not isinstance(random_seed, int):
        raise ValueError('random_seed must be an integer')
    if fk_port is None:
        raise ValueError('fk_port is required')
    condition_index = NOISE_CONDITIONS.index(condition)
    rng = _noise_rng(random_seed, condition_index)
    bundle_rows: list[dict[str, Any]] = []
    noisy_samples: list[CalibrationSample] = []
    failures: list[str] = []
    pnp_values: list[
        tuple[CalibrationSampleRecord, PnpResult | None, str | None]
    ] = []
    left_indices = tuple(
        values[0].interpolated_joints.joint_names.index(name)
        for name in LEFT_ARM_JOINT_NAMES
    )

    for record in values:
        pnp_result: PnpResult | None = None
        sample_failure: str | None = None
        image_standard = rng.standard_normal(
            (len(record.target_detection.corner_ids), 2)
        )
        joint_standard = rng.standard_normal(len(LEFT_ARM_JOINT_NAMES))
        image_noise = image_standard * condition.image_point_sigma_px
        joint_noise = joint_standard * condition.joint_position_sigma_rad
        bundle_rows.append(
            {
                'sample_id': record.sample.sample_id,
                'image_noise_px': image_noise.tolist(),
                'joint_names': list(LEFT_ARM_JOINT_NAMES),
                'joint_noise_rad': joint_noise.tolist(),
            }
        )
        positions = np.asarray(
            record.interpolated_joints.positions_rad,
            dtype=np.float64,
        ).copy()
        positions[np.asarray(left_indices)] += joint_noise
        try:
            base_T_gripper = fk_port.compute(
                record.interpolated_joints.joint_names,
                tuple(float(value) for value in positions),
            )
            if (
                base_T_gripper.parent_frame != 'base_link'
                or base_T_gripper.child_frame != 'left_gripper_frame'
            ):
                raise ValueError(
                    'offline FK returned the wrong frame direction'
                )
            detection = _noisy_detection(record, image_noise)
            pnp_result = pnp_solver(
                detection,
                camera_matrix=np.asarray(record.camera_info.k).reshape(3, 3),
                distortion_coefficients=record.camera_info.d,
                camera_frame=record.camera_info.frame_id,
                target_frame='charuco_target',
            )
            if not pnp_result.valid or pnp_result.camera_T_target is None:
                reason = (
                    'unknown'
                    if pnp_result.failure_reason is None
                    else pnp_result.failure_reason.value
                )
                raise ValueError(f'PnP failed: {reason}')
            noisy_samples.append(
                CalibrationSample(
                    sample_id=record.sample.sample_id,
                    pose_id=record.sample.pose_id,
                    split=record.sample.split,
                    base_T_gripper=base_T_gripper,
                    camera_T_target=pnp_result.camera_T_target,
                )
            )
        except Exception as error:
            sample_failure = (
                f'{record.sample.sample_id}: '
                f'{type(error).__name__}: {error}'
            )
            failures.append(sample_failure)
        pnp_values.append((record, pnp_result, sample_failure))

    bundle_hash = _canonical_sha256(
        {
            'algorithm': 'numpy.PCG64.SeedSequence-v1',
            'condition': condition.condition.value,
            'image_point_sigma_px': condition.image_point_sigma_px,
            'joint_position_sigma_rad': condition.joint_position_sigma_rad,
            'random_seed': random_seed,
            'samples': bundle_rows,
        }
    )
    diagnostics = tuple(
        PnpExperimentDiagnostic(
            condition=condition.condition,
            random_seed=random_seed,
            sample_id=record.sample.sample_id,
            split=record.sample.split,
            noise_bundle_sha256=bundle_hash,
            valid=(
                pnp is not None
                and pnp.valid
                and pnp.camera_T_target is not None
                and failure is None
            ),
            failure_reason=(
                failure
                if failure is not None
                else (
                    None
                    if pnp is not None and pnp.failure_reason is None
                    else (
                        'PnP failed'
                        if pnp is None
                        else pnp.failure_reason.value
                    )
                )
            ),
            ambiguous=False if pnp is None else pnp.ambiguous,
            selected_candidate_index=(
                None if pnp is None else pnp.selected_candidate_index
            ),
            candidates=() if pnp is None else pnp.candidates,
        )
        for record, pnp, failure in pnp_values
    )
    if failures:
        return PreparedNoiseDataset(
            condition=condition.condition,
            random_seed=random_seed,
            noise_bundle_sha256=bundle_hash,
            calibration_samples=(),
            held_out_samples=(),
            pnp_diagnostics=diagnostics,
            valid=False,
            failure_reason='; '.join(failures),
        )
    return PreparedNoiseDataset(
        condition=condition.condition,
        random_seed=random_seed,
        noise_bundle_sha256=bundle_hash,
        calibration_samples=tuple(
            item
            for item in noisy_samples
            if item.split is SampleSplit.CALIBRATION
        ),
        held_out_samples=tuple(
            item
            for item in noisy_samples
            if item.split is SampleSplit.HELD_OUT
        ),
        pnp_diagnostics=diagnostics,
        valid=True,
        failure_reason=None,
    )


def _invalid_rows(
    prepared: PreparedNoiseDataset,
) -> tuple[SolverExperimentRow, ...]:
    assert prepared.failure_reason is not None
    return tuple(
        SolverExperimentRow(
            condition=prepared.condition,
            random_seed=prepared.random_seed,
            method=spec.method,
            noise_bundle_sha256=prepared.noise_bundle_sha256,
            valid=False,
            failure_reason=(
                f'noise_preparation_failed: {prepared.failure_reason}'
            ),
            runtime_ms=0.0,
            gripper_T_camera=None,
            translation_error_m=None,
            rotation_error_rad=None,
            held_out_translation_median_m=None,
            held_out_translation_p95_m=None,
            held_out_rotation_median_rad=None,
            held_out_rotation_p95_rad=None,
        )
        for spec in HAND_EYE_METHOD_REGISTRY
    )


def _evaluated_row(
    prepared: PreparedNoiseDataset,
    result: HandEyeResult,
    ground_truth: RigidTransform,
) -> SolverExperimentRow:
    if not result.valid or result.gripper_T_camera is None:
        reason = (
            result.failure_reason.value
            if isinstance(result.failure_reason, HandEyeFailure)
            else 'invalid_solver_result'
        )
        if result.failure_detail:
            reason = f'{reason}: {result.failure_detail}'
        return SolverExperimentRow(
            prepared.condition,
            prepared.random_seed,
            result.method,
            prepared.noise_bundle_sha256,
            False,
            reason,
            result.runtime_ms,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        )
    try:
        accuracy = transform_error_metrics(
            result.gripper_T_camera,
            ground_truth,
        )
        held_out = held_out_base_target_consistency(
            prepared.held_out_samples,
            result.gripper_T_camera,
        )
    except ValueError as error:
        return SolverExperimentRow(
            prepared.condition,
            prepared.random_seed,
            result.method,
            prepared.noise_bundle_sha256,
            False,
            f'evaluation_failed: {error}',
            result.runtime_ms,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        )
    return SolverExperimentRow(
        prepared.condition,
        prepared.random_seed,
        result.method,
        prepared.noise_bundle_sha256,
        True,
        None,
        result.runtime_ms,
        result.gripper_T_camera,
        accuracy.translation_error_m,
        accuracy.rotation_error_rad,
        held_out.translation_median_m,
        held_out.translation_p95_m,
        held_out.rotation_median_rad,
        held_out.rotation_p95_rad,
    )


def percentile_linear(values: Sequence[float], percentile: float) -> float:
    """NumPy-1.21-compatible deterministic linear percentile."""

    ordered = sorted(float(value) for value in values)
    if not ordered or any(not math.isfinite(value) for value in ordered):
        raise ValueError('percentile values must be non-empty and finite')
    if not 0.0 <= percentile <= 100.0:
        raise ValueError('percentile must be in [0, 100]')
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    ratio = position - lower
    return ordered[lower] + ratio * (ordered[upper] - ordered[lower])


def _median(values: Sequence[float]) -> float:
    return percentile_linear(values, 50.0)


def summarize_methods(
    rows: Sequence[SolverExperimentRow],
) -> tuple[MethodExperimentSummary, ...]:
    result = []
    for method in HandEyeMethod:
        method_rows = tuple(row for row in rows if row.method is method)
        valid = tuple(row for row in method_rows if row.valid)

        def values(name: str) -> tuple[float, ...]:
            output = tuple(
                float(getattr(row, name))
                for row in valid
                if getattr(row, name) is not None
            )
            if len(output) != len(valid):
                raise ValueError(f'valid rows are missing {name}')
            return output

        def stats(name: str) -> tuple[float | None, float | None]:
            field_values = values(name)
            if not field_values:
                return None, None
            return _median(field_values), percentile_linear(field_values, 95.0)

        translation_median, translation_p95 = stats('translation_error_m')
        rotation_median, rotation_p95 = stats('rotation_error_rad')
        held_translation_medians = values('held_out_translation_median_m')
        held_translation_p95s = values('held_out_translation_p95_m')
        held_rotation_medians = values('held_out_rotation_median_rad')
        held_rotation_p95s = values('held_out_rotation_p95_rad')
        runtimes = tuple(row.runtime_ms for row in valid)
        failures = Counter(
            row.failure_reason or 'unknown'
            for row in method_rows
            if not row.valid
        )
        result.append(
            MethodExperimentSummary(
                method=method,
                total_runs=len(method_rows),
                valid_runs=len(valid),
                valid_result_rate=(
                    0.0 if not method_rows else len(valid) / len(method_rows)
                ),
                translation_median_m=translation_median,
                translation_p95_m=translation_p95,
                rotation_median_rad=rotation_median,
                rotation_p95_rad=rotation_p95,
                held_out_translation_median_m=(
                    None
                    if not held_translation_medians
                    else _median(held_translation_medians)
                ),
                held_out_translation_p95_m=(
                    None
                    if not held_translation_p95s
                    else percentile_linear(held_translation_p95s, 95.0)
                ),
                held_out_rotation_median_rad=(
                    None
                    if not held_rotation_medians
                    else _median(held_rotation_medians)
                ),
                held_out_rotation_p95_rad=(
                    None
                    if not held_rotation_p95s
                    else percentile_linear(held_rotation_p95s, 95.0)
                ),
                runtime_median_ms=None if not runtimes else _median(runtimes),
                runtime_p95_ms=(
                    None if not runtimes else percentile_linear(runtimes, 95.0)
                ),
                failure_counts=tuple(sorted(failures.items())),
            )
        )
    return tuple(result)


def _dominates(left: Sequence[float], right: Sequence[float]) -> bool:
    return all(a <= b for a, b in zip(left, right, strict=True)) and any(
        a < b for a, b in zip(left, right, strict=True)
    )


def _nondominated(
    summaries: Sequence[MethodExperimentSummary],
    vector: Callable[[MethodExperimentSummary], tuple[float, ...] | None],
) -> tuple[MethodExperimentSummary, ...]:
    candidates = tuple(summaries)
    result = []
    for candidate in candidates:
        candidate_vector = vector(candidate)
        if candidate_vector is None:
            continue
        if not any(
            other is not candidate
            and vector(other) is not None
            and _dominates(vector(other) or (), candidate_vector)
            for other in candidates
        ):
            result.append(candidate)
    return tuple(result)


def select_method(
    summaries: Sequence[MethodExperimentSummary],
    rows: Sequence[SolverExperimentRow],
    *,
    candidate_seed: int,
) -> MethodSelection:
    stable = tuple(
        item
        for item in summaries
        if item.valid_result_rate >= VALID_RESULT_RATE_MIN
    )
    if not stable:
        return MethodSelection(
            SelectionStatus.REVIEW_REQUIRED,
            None,
            'no method reached the 0.95 valid-result-rate threshold',
            None,
            None,
            None,
        )
    accuracy_front = _nondominated(stable, lambda item: item.accuracy_vector)
    if len(accuracy_front) == 1:
        winner = accuracy_front[0]
        reason = 'unique accuracy Pareto winner among stable methods'
    else:
        held_front = _nondominated(
            accuracy_front,
            lambda item: item.held_out_vector,
        )
        if len(held_front) != 1:
            methods = ','.join(item.method.value for item in held_front)
            return MethodSelection(
                SelectionStatus.REVIEW_REQUIRED,
                None,
                'accuracy/held-out metrics remain crossed; human review '
                f'is required for: {methods}',
                None,
                None,
                None,
            )
        winner = held_front[0]
        reason = 'held-out consistency resolved the accuracy Pareto tie'
    clean = tuple(
        row
        for row in rows
        if row.method is winner.method
        and row.condition is NoiseCondition.IDEAL
        and row.random_seed == candidate_seed
    )
    if (
        len(clean) != 1
        or not clean[0].valid
        or clean[0].gripper_T_camera is None
    ):
        return MethodSelection(
            SelectionStatus.REVIEW_REQUIRED,
            None,
            'selected method lacks a valid clean ideal candidate transform',
            None,
            None,
            None,
        )
    return MethodSelection(
        SelectionStatus.SELECTED,
        winner.method,
        reason,
        clean[0].gripper_T_camera,
        NoiseCondition.IDEAL,
        candidate_seed,
    )


def run_solver_experiment(
    records: Sequence[CalibrationSampleRecord],
    fk_port: FeedbackFkPort,
    config: ExperimentConfig,
    *,
    ground_truth: RigidTransform,
    pnp_solver: PnpSolver = solve_planar_pnp,
    solve_all: AllMethodSolver = solve_all_hand_eye_methods,
    allow_partial: bool = False,
) -> SolverExperimentResult:
    """Execute exactly 5 methods x 3 conditions x 10 seed bundles."""

    values = validate_evaluation_records(
        records,
        allow_partial=allow_partial,
    )
    if not isinstance(config, ExperimentConfig):
        raise ValueError('config must be ExperimentConfig')
    if not isinstance(ground_truth, RigidTransform):
        raise ValueError('ground_truth must be a RigidTransform')
    policy = HandEyeTransformValidityPolicy(config.max_translation_norm_m)
    rows: list[SolverExperimentRow] = []
    pnp_diagnostics: list[PnpExperimentDiagnostic] = []
    for condition in NOISE_CONDITIONS:
        for random_seed in config.random_seeds:
            prepared = prepare_noise_dataset(
                values,
                condition,
                random_seed,
                fk_port,
                pnp_solver=pnp_solver,
                allow_partial=allow_partial,
            )
            pnp_diagnostics.extend(prepared.pnp_diagnostics)
            if not prepared.valid:
                rows.extend(_invalid_rows(prepared))
                continue
            try:
                method_results = solve_all(
                    prepared.calibration_samples,
                    validity_policy=policy,
                )
            except Exception as error:
                failed = PreparedNoiseDataset(
                    prepared.condition,
                    prepared.random_seed,
                    prepared.noise_bundle_sha256,
                    (),
                    (),
                    prepared.pnp_diagnostics,
                    False,
                    'solver registry/run failed: '
                    f'{type(error).__name__}: {error}',
                )
                rows.extend(_invalid_rows(failed))
                continue
            if (
                len(method_results) != EXPECTED_METHOD_COUNT
                or tuple(item.method for item in method_results)
                != tuple(HandEyeMethod)
            ):
                failed = PreparedNoiseDataset(
                    prepared.condition,
                    prepared.random_seed,
                    prepared.noise_bundle_sha256,
                    (),
                    (),
                    prepared.pnp_diagnostics,
                    False,
                    'solver did not return the exact five-method '
                    'registry order',
                )
                rows.extend(_invalid_rows(failed))
                continue
            rows.extend(
                _evaluated_row(prepared, result, ground_truth)
                for result in method_results
            )
    if len(rows) != EXPECTED_SOLVER_RUN_COUNT:
        raise RuntimeError(
            f'experiment produced {len(rows)} rows, expected 150'
        )
    if len(pnp_diagnostics) != 30 * len(values):
        raise RuntimeError('experiment did not preserve all PnP diagnostics')
    summaries = summarize_methods(rows)
    selection = select_method(
        summaries,
        rows,
        candidate_seed=config.random_seeds[0],
    )
    return SolverExperimentResult(
        config=config,
        rows=tuple(rows),
        pnp_diagnostics=tuple(pnp_diagnostics),
        method_summaries=summaries,
        selection=selection,
    )


__all__ = [
    'EXPERIMENT_SCHEMA_VERSION',
    'EXPECTED_SOLVER_RUN_COUNT',
    'MINIMUM_PARTIAL_CALIBRATION_COUNT',
    'MINIMUM_PARTIAL_HELD_OUT_COUNT',
    'ExperimentConfig',
    'MethodExperimentSummary',
    'MethodSelection',
    'NOISE_CONDITIONS',
    'NoiseCondition',
    'NoiseConditionSpec',
    'PnpExperimentDiagnostic',
    'PreparedNoiseDataset',
    'SelectionStatus',
    'SolverExperimentResult',
    'SolverExperimentRow',
    'load_sample_records',
    'percentile_linear',
    'prepare_noise_dataset',
    'run_solver_experiment',
    'select_method',
    'summarize_methods',
    'validate_evaluation_records',
]
