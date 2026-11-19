"""Strict JSON configuration for the single-pose ROS executable."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping

from cleany_handeye_calibration.camera_acquisition import (
    DEFAULT_CAMERA_CONTRACT,
)
from cleany_handeye_calibration.dataset_writer import (
    CaptureTiming,
    DatasetManifestV1,
    GitProvenance,
    SoftwareVersions,
    SourceArtifactHashes,
    TargetDatasetContract,
)
from cleany_handeye_calibration.joint_state_sync import (
    LEFT_ARM_JOINT_NAMES,
)
from cleany_handeye_calibration.models import (
    JointPose,
    PositionTarget,
    SampleSplit,
)
from cleany_handeye_calibration.motion_config import (
    MujocoMotionConfig,
    StageTimeouts,
)
from cleany_handeye_calibration.single_pose_orchestrator import (
    JointSoftLimit,
    SinglePoseRequest,
    SinglePoseSafetyProfile,
    SinglePoseTimeouts,
)


SCHEMA_VERSION = 'cleany.single_pose_runtime/v1'


class SinglePoseConfigError(ValueError):
    pass


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SinglePoseConfigError(f'{path} must be an object')
    return value


def _exact_keys(
    value: Mapping[str, Any], required: set[str], path: str
) -> None:
    keys = set(value)
    if keys != required:
        missing = sorted(required - keys)
        extra = sorted(keys - required)
        raise SinglePoseConfigError(
            f'{path} keys mismatch; missing={missing!r}, extra={extra!r}'
        )


def _positive(value: Any, path: str) -> float:
    if isinstance(value, bool):
        raise SinglePoseConfigError(f'{path} must be positive and finite')
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise SinglePoseConfigError(
            f'{path} must be positive and finite'
        ) from error
    if not math.isfinite(number) or number <= 0.0:
        raise SinglePoseConfigError(f'{path} must be positive and finite')
    return number


def _positive_int(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise SinglePoseConfigError(f'{path} must be a positive integer')
    return value


def _tuple_floats(value: Any, size: int, path: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise SinglePoseConfigError(f'{path} must contain {size} values')
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise SinglePoseConfigError(f'{path} must be numeric') from error
    if not all(math.isfinite(item) for item in result):
        raise SinglePoseConfigError(f'{path} must be finite')
    return result


def _json_object_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise SinglePoseConfigError(f'duplicate JSON key: {key!r}')
        result[key] = value
    return result


@dataclass(frozen=True, slots=True)
class ExpectedResolvedPoseEvidence:
    pose: JointPose
    match_tolerance_rad: float
    observed_collision_clearance_m: float

    def __post_init__(self) -> None:
        if self.pose.joint_names != LEFT_ARM_JOINT_NAMES:
            raise ValueError('expected pose must use canonical left joints')
        object.__setattr__(
            self,
            'match_tolerance_rad',
            _positive(self.match_tolerance_rad, 'match_tolerance_rad'),
        )
        object.__setattr__(
            self,
            'observed_collision_clearance_m',
            _positive(
                self.observed_collision_clearance_m,
                'observed_collision_clearance_m',
            ),
        )

    def validate_match(self, pose: JointPose) -> None:
        if pose.joint_names != self.pose.joint_names:
            raise ValueError('resolved joint-name set differs from evidence')
        if any(
            abs(actual - expected) > self.match_tolerance_rad
            for actual, expected in zip(
                pose.positions_rad,
                self.pose.positions_rad,
                strict=True,
            )
        ):
            raise ValueError(
                'resolved joint pose differs from the clearance evidence'
            )


@dataclass(frozen=True, slots=True)
class FeedbackBufferConfig:
    capacity: int
    max_sample_distance_ns: int
    clock_reset_threshold_ns: int
    startup_state_timeout_sec: float
    startup_planning_scene_timeout_sec: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self, 'capacity', _positive_int(self.capacity, 'capacity')
        )
        object.__setattr__(
            self,
            'max_sample_distance_ns',
            _positive_int(
                self.max_sample_distance_ns,
                'max_sample_distance_ns',
            ),
        )
        object.__setattr__(
            self,
            'clock_reset_threshold_ns',
            _positive_int(
                self.clock_reset_threshold_ns,
                'clock_reset_threshold_ns',
            ),
        )
        object.__setattr__(
            self,
            'startup_state_timeout_sec',
            _positive(
                self.startup_state_timeout_sec,
                'startup_state_timeout_sec',
            ),
        )
        object.__setattr__(
            self,
            'startup_planning_scene_timeout_sec',
            _positive(
                self.startup_planning_scene_timeout_sec,
                'startup_planning_scene_timeout_sec',
            ),
        )


@dataclass(frozen=True, slots=True)
class SinglePoseRuntimeConfig:
    artifact_root: Path
    request: SinglePoseRequest
    motion: MujocoMotionConfig
    expected: ExpectedResolvedPoseEvidence
    feedback: FeedbackBufferConfig
    dataset_manifest: DatasetManifestV1

    def __post_init__(self) -> None:
        root = Path(self.artifact_root).expanduser()
        if not root.is_absolute():
            raise ValueError('artifact_root must be an absolute path')
        object.__setattr__(self, 'artifact_root', root.resolve())
        if self.dataset_manifest.run_id == '':
            raise ValueError('dataset manifest run_id must not be empty')
        if (
            self.request.timeouts.resolve_position_ik_sec
            < self.motion.stage_timeouts.ik_sec
            or self.request.timeouts.validate_resolved_pose_sec
            < self.motion.stage_timeouts.state_validity_sec
            or self.request.timeouts.plan_sec
            < self.motion.stage_timeouts.plan_sec
            or self.request.timeouts.execute_sec
            < self.motion.stage_timeouts.execute_sec
            or self.request.timeouts.wait_settled_sec
            < self.motion.stage_timeouts.settle_sec
        ):
            raise ValueError(
                'orchestration timeouts must cover underlying adapter budgets'
            )


def _parse_timeouts(value: Any) -> SinglePoseTimeouts:
    data = _mapping(value, 'orchestration_timeouts_sec')
    names = {
        'resolve_position_ik',
        'validate_resolved_pose',
        'plan',
        'execute',
        'wait_settled',
        'acquire_image',
        'detect_target',
        'compute_feedback_fk',
        'record_sample',
    }
    _exact_keys(data, names, 'orchestration_timeouts_sec')
    return SinglePoseTimeouts(
        **{
            f'{name}_sec': _positive(
                data[name], f'orchestration_timeouts_sec.{name}'
            )
            for name in names
        }
    )


def _parse_motion(value: Any) -> MujocoMotionConfig:
    data = _mapping(value, 'motion')
    required = {
        'current_state_max_age_sec',
        'right_park_position_tolerance_rad',
        'stage_timeouts_sec',
        'max_velocity_scaling_factor',
        'max_acceleration_scaling_factor',
        'controller_path_tolerance_rad',
        'controller_goal_tolerance_rad',
        'settle_position_tolerance_rad',
        'settle_velocity_tolerance_rad_s',
        'settle_duration_sec',
        'planning_attempts',
    }
    _exact_keys(data, required, 'motion')
    timeout_data = _mapping(
        data['stage_timeouts_sec'], 'motion.stage_timeouts_sec'
    )
    timeout_names = {
        'ik', 'state_validity', 'plan', 'execute', 'cancel', 'settle'
    }
    _exact_keys(
        timeout_data, timeout_names, 'motion.stage_timeouts_sec'
    )
    return MujocoMotionConfig(
        current_state_max_age_sec=data['current_state_max_age_sec'],
        right_park_position_tolerance_rad=(
            data['right_park_position_tolerance_rad']
        ),
        stage_timeouts=StageTimeouts(
            **{
                f'{name}_sec': timeout_data[name]
                for name in timeout_names
            }
        ),
        max_velocity_scaling_factor=data['max_velocity_scaling_factor'],
        max_acceleration_scaling_factor=(
            data['max_acceleration_scaling_factor']
        ),
        controller_path_tolerance_rad=(
            data['controller_path_tolerance_rad']
        ),
        controller_goal_tolerance_rad=(
            data['controller_goal_tolerance_rad']
        ),
        settle_position_tolerance_rad=(
            data['settle_position_tolerance_rad']
        ),
        settle_velocity_tolerance_rad_s=(
            data['settle_velocity_tolerance_rad_s']
        ),
        settle_duration_sec=data['settle_duration_sec'],
        planning_attempts=data['planning_attempts'],
    )


def _parse_dataset_manifest(value: Any) -> DatasetManifestV1:
    data = _mapping(value, 'dataset_manifest')
    required = {
        'run_id',
        'git',
        'source_hashes',
        'software_versions',
        'target',
        'timing',
        'calibration_parameters',
        'random_seed',
    }
    _exact_keys(data, required, 'dataset_manifest')
    git = _mapping(data['git'], 'dataset_manifest.git')
    _exact_keys(git, {'commit', 'dirty'}, 'dataset_manifest.git')
    hashes = _mapping(
        data['source_hashes'], 'dataset_manifest.source_hashes'
    )
    _exact_keys(
        hashes,
        {'urdf_sha256', 'mjcf_sha256', 'pose_manifest_sha256'},
        'dataset_manifest.source_hashes',
    )
    software = _mapping(
        data['software_versions'], 'dataset_manifest.software_versions'
    )
    _exact_keys(
        software,
        {
            'ros_distro',
            'moveit',
            'opencv',
            'mujoco',
            'mujoco_ros2_control',
            'vendor',
        },
        'dataset_manifest.software_versions',
    )
    target = _mapping(data['target'], 'dataset_manifest.target')
    _exact_keys(
        target,
        {'board_svg_sha256', 'board_pdf_sha256', 'size_provenance'},
        'dataset_manifest.target',
    )
    timing = _mapping(data['timing'], 'dataset_manifest.timing')
    _exact_keys(
        timing,
        {
            'simulation_timestep_s',
            'controller_update_rate_hz',
            'image_rate_hz',
            'joint_state_rate_hz',
        },
        'dataset_manifest.timing',
    )
    parameters = _mapping(
        data['calibration_parameters'],
        'dataset_manifest.calibration_parameters',
    )
    return DatasetManifestV1(
        run_id=data['run_id'],
        git=GitProvenance(commit=git['commit'], dirty=git['dirty']),
        source_hashes=SourceArtifactHashes(**hashes),
        software=SoftwareVersions(
            ros_distro=software['ros_distro'],
            moveit=software['moveit'],
            opencv=software['opencv'],
            mujoco=software['mujoco'],
            mujoco_ros2_control=software['mujoco_ros2_control'],
            vendor_versions=software['vendor'],
        ),
        camera=DEFAULT_CAMERA_CONTRACT,
        camera_vertical_fov_degrees=93.0,
        target=TargetDatasetContract(**target),
        timing=CaptureTiming(**timing),
        calibration_parameters=parameters,
        random_seed=data['random_seed'],
    )


def load_single_pose_runtime_config(
    path: str | Path,
) -> SinglePoseRuntimeConfig:
    source = Path(path).expanduser().resolve(strict=True)
    try:
        data = json.loads(
            source.read_text(encoding='utf-8'),
            object_pairs_hook=_json_object_pairs,
            parse_constant=lambda value: (_ for _ in ()).throw(
                SinglePoseConfigError(
                    f'non-finite JSON constant is forbidden: {value}'
                )
            ),
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise SinglePoseConfigError(
            f'cannot read single-pose config: {error}'
        ) from error
    root = _mapping(data, 'config')
    required = {
        'schema_version',
        'artifact_root',
        'sample',
        'safety_profile',
        'expected_resolved_evidence',
        'motion',
        'orchestration_timeouts_sec',
        'feedback_buffer',
        'dataset_manifest',
    }
    _exact_keys(root, required, 'config')
    if root['schema_version'] != SCHEMA_VERSION:
        raise SinglePoseConfigError('unsupported schema_version')

    sample = _mapping(root['sample'], 'sample')
    _exact_keys(
        sample,
        {
            'sample_id',
            'pose_id',
            'split',
            'target_position_m',
            'ik_seed_positions_rad',
        },
        'sample',
    )
    safety = _mapping(root['safety_profile'], 'safety_profile')
    _exact_keys(
        safety,
        {'profile_id', 'soft_joint_limits_rad', 'collision_margin_m'},
        'safety_profile',
    )
    limits = _mapping(
        safety['soft_joint_limits_rad'],
        'safety_profile.soft_joint_limits_rad',
    )
    _exact_keys(
        limits,
        set(LEFT_ARM_JOINT_NAMES),
        'safety_profile.soft_joint_limits_rad',
    )
    soft_limits = tuple(
        JointSoftLimit(
            name,
            *_tuple_floats(
                limits[name],
                2,
                f'safety_profile.soft_joint_limits_rad.{name}',
            ),
        )
        for name in LEFT_ARM_JOINT_NAMES
    )
    evidence = _mapping(
        root['expected_resolved_evidence'],
        'expected_resolved_evidence',
    )
    _exact_keys(
        evidence,
        {
            'joint_positions_rad',
            'match_tolerance_rad',
            'collision_clearance_m',
        },
        'expected_resolved_evidence',
    )
    feedback = _mapping(root['feedback_buffer'], 'feedback_buffer')
    _exact_keys(
        feedback,
        {
            'capacity',
            'max_sample_distance_ns',
            'clock_reset_threshold_ns',
            'startup_state_timeout_sec',
            'startup_planning_scene_timeout_sec',
        },
        'feedback_buffer',
    )

    return SinglePoseRuntimeConfig(
        artifact_root=Path(root['artifact_root']),
        request=SinglePoseRequest(
            sample_id=sample['sample_id'],
            pose_id=sample['pose_id'],
            split=SampleSplit(sample['split']),
            target=PositionTarget(
                'base_link',
                _tuple_floats(
                    sample['target_position_m'],
                    3,
                    'sample.target_position_m',
                ),
            ),
            ik_seed=JointPose(
                LEFT_ARM_JOINT_NAMES,
                _tuple_floats(
                    sample['ik_seed_positions_rad'],
                    5,
                    'sample.ik_seed_positions_rad',
                ),
            ),
            timeouts=_parse_timeouts(
                root['orchestration_timeouts_sec']
            ),
            safety_profile=SinglePoseSafetyProfile(
                profile_id=safety['profile_id'],
                soft_joint_limits=soft_limits,
                required_collision_margin_m=safety['collision_margin_m'],
            ),
        ),
        motion=_parse_motion(root['motion']),
        expected=ExpectedResolvedPoseEvidence(
            pose=JointPose(
                LEFT_ARM_JOINT_NAMES,
                _tuple_floats(
                    evidence['joint_positions_rad'],
                    5,
                    'expected_resolved_evidence.joint_positions_rad',
                ),
            ),
            match_tolerance_rad=evidence['match_tolerance_rad'],
            observed_collision_clearance_m=(
                evidence['collision_clearance_m']
            ),
        ),
        feedback=FeedbackBufferConfig(**feedback),
        dataset_manifest=_parse_dataset_manifest(
            root['dataset_manifest']
        ),
    )


__all__ = [
    'ExpectedResolvedPoseEvidence',
    'FeedbackBufferConfig',
    'SCHEMA_VERSION',
    'SinglePoseConfigError',
    'SinglePoseRuntimeConfig',
    'load_single_pose_runtime_config',
]
