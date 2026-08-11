"""Selected-method sensitivity to image/joint timestamp offsets."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Callable, Mapping

from cleany_handeye_calibration.experiment_evaluation import (
    MethodSelection,
    SelectionStatus,
)
from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    BufferInsertStatus,
    JointStateRingBuffer,
)
from cleany_handeye_calibration.models import (
    CalibrationSample,
    SampleSplit,
    TimedJointSample,
)
from cleany_handeye_calibration.offline_fk import FeedbackFkPort
from cleany_handeye_calibration.schema import transform_from_mapping
from cleany_handeye_calibration.solver import (
    HandEyeMethod,
    HandEyeResult,
    HandEyeTransformValidityPolicy,
    solve_hand_eye_method,
)
from cleany_handeye_calibration.transforms import RigidTransform
from cleany_handeye_calibration.validation import transform_error_metrics


CONTINUOUS_LOG_SCHEMA_VERSION = 1


@dataclass(frozen=True, slots=True)
class ContinuousImageObservation:
    sample_id: str
    pose_id: str
    image_stamp_ns: int
    camera_T_target: RigidTransform

    def __post_init__(self) -> None:
        for field_name in ('sample_id', 'pose_id'):
            value = getattr(self, field_name)
            if (
                not isinstance(value, str)
                or not value
                or value != value.strip()
            ):
                raise ValueError(
                    f'{field_name} must be non-empty trimmed text'
                )
        if (
            isinstance(self.image_stamp_ns, bool)
            or not isinstance(self.image_stamp_ns, int)
            or self.image_stamp_ns <= 0
        ):
            raise ValueError('image_stamp_ns must be a positive integer')
        if not isinstance(self.camera_T_target, RigidTransform):
            raise ValueError('camera_T_target must be a RigidTransform')
        if (
            self.camera_T_target.parent_frame
            != 'left_wrist_rgb_optical_frame'
            or self.camera_T_target.child_frame != 'charuco_target'
        ):
            raise ValueError('camera_T_target has the wrong frame direction')


@dataclass(frozen=True, slots=True)
class ContinuousTrajectoryLog:
    joint_samples: tuple[TimedJointSample, ...]
    image_observations: tuple[ContinuousImageObservation, ...]

    def __post_init__(self) -> None:
        samples = tuple(self.joint_samples)
        observations = tuple(self.image_observations)
        if len(samples) < 2:
            raise ValueError('continuous log requires at least two joint rows')
        if len(observations) < 3:
            raise ValueError('continuous log requires at least three images')
        if any(not isinstance(item, TimedJointSample) for item in samples):
            raise ValueError('joint_samples must contain TimedJointSample')
        if any(
            not isinstance(item, ContinuousImageObservation)
            for item in observations
        ):
            raise ValueError('image observations have the wrong type')
        stamps = tuple(item.stamp_ns for item in samples)
        if tuple(sorted(stamps)) != stamps or len(set(stamps)) != len(stamps):
            raise ValueError('joint sample stamps must be strictly increasing')
        for item in samples:
            if (
                item.joint_names
                != DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
            ):
                raise ValueError(
                    'continuous joint rows require canonical 12-joint order'
                )
        image_stamps = tuple(item.image_stamp_ns for item in observations)
        if len(set(image_stamps)) != len(image_stamps):
            raise ValueError('continuous image stamps must be unique')
        if len({item.sample_id for item in observations}) != len(observations):
            raise ValueError('continuous sample IDs must be unique')
        object.__setattr__(self, 'joint_samples', samples)
        object.__setattr__(self, 'image_observations', observations)


@dataclass(frozen=True, slots=True)
class TimestampSensitivityConfig:
    offsets_ns: tuple[int, ...]
    max_joint_sample_distance_ns: int
    max_translation_norm_m: float

    def __post_init__(self) -> None:
        offsets = tuple(self.offsets_ns)
        if (
            not offsets
            or 0 not in offsets
            or len(set(offsets)) != len(offsets)
            or any(
                isinstance(value, bool) or not isinstance(value, int)
                for value in offsets
            )
        ):
            raise ValueError(
                'offsets_ns must be unique integers including zero'
            )
        if (
            isinstance(self.max_joint_sample_distance_ns, bool)
            or not isinstance(self.max_joint_sample_distance_ns, int)
            or self.max_joint_sample_distance_ns <= 0
        ):
            raise ValueError('max_joint_sample_distance_ns must be positive')
        maximum = float(self.max_translation_norm_m)
        if not math.isfinite(maximum) or maximum <= 0.0:
            raise ValueError('max_translation_norm_m must be positive/finite')
        object.__setattr__(self, 'offsets_ns', offsets)
        object.__setattr__(self, 'max_translation_norm_m', maximum)


@dataclass(frozen=True, slots=True)
class TimestampSensitivityRow:
    method: HandEyeMethod
    offset_ns: int
    valid: bool
    failure_reason: str | None
    sample_count: int
    runtime_ms: float
    translation_error_m: float | None
    rotation_error_rad: float | None


@dataclass(frozen=True, slots=True)
class TimestampSensitivityResult:
    selected_method: HandEyeMethod | None
    review_required: bool
    reason: str | None
    rows: tuple[TimestampSensitivityRow, ...]


SingleMethodSolver = Callable[..., HandEyeResult]


def _parse_continuous_row(value: Mapping[str, Any]) -> tuple[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError('continuous log row must be an object')
    if value.get('schema_version') != CONTINUOUS_LOG_SCHEMA_VERSION:
        raise ValueError('unsupported continuous-log schema_version')
    kind = value.get('kind')
    if kind == 'joint_state':
        expected = {
            'schema_version',
            'kind',
            'stamp_ns',
            'joint_names',
            'positions_rad',
        }
        optional = {'velocities_rad_s'}
        if not expected.issubset(value) or set(value) - expected - optional:
            raise ValueError('joint-state row has invalid fields')
        return kind, TimedJointSample(
            stamp_ns=value['stamp_ns'],
            joint_names=tuple(value['joint_names']),
            positions_rad=tuple(value['positions_rad']),
            velocities_rad_s=(
                None
                if value.get('velocities_rad_s') is None
                else tuple(value['velocities_rad_s'])
            ),
        )
    if kind == 'image_observation':
        expected = {
            'schema_version',
            'kind',
            'sample_id',
            'pose_id',
            'split',
            'image_stamp_ns',
            'camera_to_target',
        }
        if set(value) != expected:
            raise ValueError('image-observation row has invalid fields')
        if value.get('split') != SampleSplit.CALIBRATION.value:
            raise ValueError(
                'continuous sensitivity observations must be calibration split'
            )
        return kind, ContinuousImageObservation(
            sample_id=value['sample_id'],
            pose_id=value['pose_id'],
            image_stamp_ns=value['image_stamp_ns'],
            camera_T_target=transform_from_mapping(value['camera_to_target']),
        )
    raise ValueError(f'unsupported continuous log row kind: {kind!r}')


def load_continuous_trajectory_log(
    path: str | Path,
) -> ContinuousTrajectoryLog:
    log_path = Path(path).expanduser().resolve(strict=True)
    payload = log_path.read_bytes()
    if payload and not payload.endswith(b'\n'):
        raise ValueError('continuous JSONL has a partial final row')
    joint_samples = []
    observations = []
    for line_number, line in enumerate(payload.splitlines(), start=1):
        try:
            raw = json.loads(line.decode('utf-8'))
            kind, item = _parse_continuous_row(raw)
        except (
            UnicodeDecodeError,
            json.JSONDecodeError,
            KeyError,
            ValueError,
        ) as error:
            raise ValueError(
                f'invalid continuous JSONL line {line_number}: {error}'
            ) from error
        if kind == 'joint_state':
            joint_samples.append(item)
        else:
            observations.append(item)
    return ContinuousTrajectoryLog(tuple(joint_samples), tuple(observations))


def _buffer(
    trajectory: ContinuousTrajectoryLog,
    config: TimestampSensitivityConfig,
) -> JointStateRingBuffer:
    buffer = JointStateRingBuffer(
        capacity=len(trajectory.joint_samples),
        max_sample_distance_ns=config.max_joint_sample_distance_ns,
        clock_reset_threshold_ns=(
            trajectory.joint_samples[-1].stamp_ns
            - trajectory.joint_samples[0].stamp_ns
            + 1
        ),
    )
    for sample in trajectory.joint_samples:
        result = buffer.add(sample)
        if result.status is not BufferInsertStatus.ACCEPTED:
            raise ValueError(
                f'continuous joint row was not accepted: {result.status.value}'
            )
    return buffer


def evaluate_timestamp_sensitivity(
    trajectory: ContinuousTrajectoryLog,
    fk_port: FeedbackFkPort,
    config: TimestampSensitivityConfig,
    selection: MethodSelection,
    *,
    ground_truth: RigidTransform,
    solve_one: SingleMethodSolver = solve_hand_eye_method,
) -> TimestampSensitivityResult:
    """Evaluate configured offsets for the one selected method only."""

    if selection.status is not SelectionStatus.SELECTED:
        return TimestampSensitivityResult(
            selected_method=None,
            review_required=True,
            reason=(
                'solver selection requires human review before timing study'
            ),
            rows=(),
        )
    method = selection.selected_method
    if method is None:
        raise ValueError('selected status lacks selected_method')
    if not isinstance(trajectory, ContinuousTrajectoryLog):
        raise ValueError('trajectory must be ContinuousTrajectoryLog')
    if not isinstance(config, TimestampSensitivityConfig):
        raise ValueError('config must be TimestampSensitivityConfig')
    if not isinstance(ground_truth, RigidTransform):
        raise ValueError('ground_truth must be RigidTransform')
    rows = []
    policy = HandEyeTransformValidityPolicy(config.max_translation_norm_m)
    for offset_ns in config.offsets_ns:
        buffer = _buffer(trajectory, config)
        samples = []
        failure = None
        for observation in trajectory.image_observations:
            shifted_stamp = observation.image_stamp_ns + offset_ns
            if shifted_stamp < 0:
                failure = 'timestamp offset precedes zero'
                break
            interpolation = buffer.interpolate(shifted_stamp)
            if not interpolation.success:
                assert interpolation.failure is not None
                failure = (
                    f'{observation.sample_id}: interpolation '
                    f'{interpolation.failure.value}'
                )
                break
            assert interpolation.interpolation is not None
            joint_state = interpolation.interpolation.sample
            try:
                base_T_gripper = fk_port.compute(
                    joint_state.joint_names,
                    joint_state.positions_rad,
                )
                samples.append(
                    CalibrationSample(
                        sample_id=observation.sample_id,
                        pose_id=observation.pose_id,
                        split=SampleSplit.CALIBRATION,
                        base_T_gripper=base_T_gripper,
                        camera_T_target=observation.camera_T_target,
                    )
                )
            except Exception as error:
                failure = (
                    f'{observation.sample_id}: offline FK failed: '
                    f'{type(error).__name__}: {error}'
                )
                break
        if failure is not None:
            rows.append(
                TimestampSensitivityRow(
                    method,
                    offset_ns,
                    False,
                    failure,
                    len(samples),
                    0.0,
                    None,
                    None,
                )
            )
            continue
        try:
            result = solve_one(samples, method, validity_policy=policy)
        except Exception as error:
            rows.append(
                TimestampSensitivityRow(
                    method,
                    offset_ns,
                    False,
                    f'solver failed: {type(error).__name__}: {error}',
                    len(samples),
                    0.0,
                    None,
                    None,
                )
            )
            continue
        if not result.valid or result.gripper_T_camera is None:
            reason = (
                'invalid_solver_result'
                if result.failure_reason is None
                else result.failure_reason.value
            )
            rows.append(
                TimestampSensitivityRow(
                    method,
                    offset_ns,
                    False,
                    reason,
                    len(samples),
                    result.runtime_ms,
                    None,
                    None,
                )
            )
            continue
        try:
            metrics = transform_error_metrics(
                result.gripper_T_camera,
                ground_truth,
            )
        except ValueError as error:
            rows.append(
                TimestampSensitivityRow(
                    method,
                    offset_ns,
                    False,
                    f'evaluation failed: {error}',
                    len(samples),
                    result.runtime_ms,
                    None,
                    None,
                )
            )
            continue
        rows.append(
            TimestampSensitivityRow(
                method,
                offset_ns,
                True,
                None,
                len(samples),
                result.runtime_ms,
                metrics.translation_error_m,
                metrics.rotation_error_rad,
            )
        )
    return TimestampSensitivityResult(method, False, None, tuple(rows))


__all__ = [
    'CONTINUOUS_LOG_SCHEMA_VERSION',
    'ContinuousImageObservation',
    'ContinuousTrajectoryLog',
    'TimestampSensitivityConfig',
    'TimestampSensitivityResult',
    'TimestampSensitivityRow',
    'evaluate_timestamp_sensitivity',
    'load_continuous_trajectory_log',
]
