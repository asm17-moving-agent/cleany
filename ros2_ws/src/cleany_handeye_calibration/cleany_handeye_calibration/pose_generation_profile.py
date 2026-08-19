"""Strict, explicit configuration for MuJoCo pose-set generation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

import yaml

from cleany_handeye_calibration.joint_state_sync import (
    LEFT_ARM_JOINT_NAMES,
)
from cleany_handeye_calibration.motion_config import (
    MujocoMotionConfig,
    StageTimeouts,
)
from cleany_handeye_calibration.pose_generation import PoseGenerationConfig
from cleany_handeye_calibration.pose_manifest import (
    CartesianBounds,
    ClosedInterval,
    PoseRunConfiguration,
    RequiredStageTimeouts,
    SoftJointLimits,
)
from cleany_handeye_calibration.single_pose_runtime_config import (
    FeedbackBufferConfig,
)


SCHEMA_VERSION = 'cleany.pose_generation_mujoco/v1'


class PoseGenerationProfileError(ValueError):
    pass


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise PoseGenerationProfileError(f'duplicate YAML key: {key!r}')
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_mapping,
)


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PoseGenerationProfileError(f'{path} must be a mapping')
    return value


def _exact_keys(value: Mapping[str, Any], expected: set[str], path: str):
    if set(value) != expected:
        missing = sorted(expected - set(value))
        extra = sorted(set(value) - expected)
        raise PoseGenerationProfileError(
            f'{path} keys mismatch; missing={missing!r}, extra={extra!r}'
        )


def _positive(value: Any, path: str) -> float:
    if isinstance(value, bool):
        raise PoseGenerationProfileError(f'{path} must be positive')
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise PoseGenerationProfileError(f'{path} must be positive') from error
    if not math.isfinite(result) or result <= 0.0:
        raise PoseGenerationProfileError(f'{path} must be positive')
    return result


def _unit_fraction(value: Any, path: str) -> float:
    result = _positive(value, path)
    if result >= 1.0:
        raise PoseGenerationProfileError(f'{path} must be below one')
    return result


def _positive_int(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise PoseGenerationProfileError(f'{path} must be a positive integer')
    return value


def _interval(value: Any, path: str) -> ClosedInterval:
    if not isinstance(value, list) or len(value) != 2:
        raise PoseGenerationProfileError(f'{path} must contain [min, max]')
    return ClosedInterval(value[0], value[1])


@dataclass(frozen=True, slots=True)
class VisibilityProfile:
    minimum_camera_depth_m: float
    image_border_fraction: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            'minimum_camera_depth_m',
            _positive(self.minimum_camera_depth_m, 'minimum_camera_depth_m'),
        )
        object.__setattr__(
            self,
            'image_border_fraction',
            _unit_fraction(
                self.image_border_fraction,
                'image_border_fraction',
            ),
        )


@dataclass(frozen=True, slots=True)
class MujocoPoseGenerationProfile:
    random_seed: int
    candidate_pool_size: int
    max_generation_attempts: int
    multistart_count: int
    log_det_epsilon: float
    target_seed_local_radius_m: tuple[float, float, float]
    seed_sampling_limits: SoftJointLimits
    target_bounds: CartesianBounds
    run_config: PoseRunConfiguration
    motion: MujocoMotionConfig
    visibility: VisibilityProfile
    feedback: FeedbackBufferConfig
    expected_resolved_match_tolerance_rad: float

    def __post_init__(self) -> None:
        if (
            isinstance(self.random_seed, bool)
            or not isinstance(self.random_seed, int)
            or not 0 <= self.random_seed < 2**128
        ):
            raise PoseGenerationProfileError('random_seed is out of range')
        for name in (
            'candidate_pool_size',
            'max_generation_attempts',
            'multistart_count',
        ):
            object.__setattr__(
                self,
                name,
                _positive_int(getattr(self, name), name),
            )
        if self.candidate_pool_size < 25:
            raise PoseGenerationProfileError(
                'candidate_pool_size must contain at least 25 candidates'
            )
        if self.max_generation_attempts < self.candidate_pool_size:
            raise PoseGenerationProfileError(
                'max_generation_attempts must cover the candidate pool'
            )
        object.__setattr__(
            self,
            'log_det_epsilon',
            _positive(self.log_det_epsilon, 'log_det_epsilon'),
        )
        radius = tuple(
            _positive(value, 'target_seed_local_radius_m')
            for value in self.target_seed_local_radius_m
        )
        if len(radius) != 3:
            raise PoseGenerationProfileError(
                'target_seed_local_radius_m must contain three values'
            )
        object.__setattr__(self, 'target_seed_local_radius_m', radius)
        object.__setattr__(
            self,
            'expected_resolved_match_tolerance_rad',
            _positive(
                self.expected_resolved_match_tolerance_rad,
                'expected_resolved_match_tolerance_rad',
            ),
        )
        self.run_config.require_ready()

    def generation_config(
        self,
        reference_rotation_quaternion_xyzw: tuple[float, float, float, float],
    ) -> PoseGenerationConfig:
        return PoseGenerationConfig(
            random_seed=self.random_seed,
            candidate_pool_size=self.candidate_pool_size,
            max_generation_attempts=self.max_generation_attempts,
            log_det_epsilon=self.log_det_epsilon,
            target_position_bounds_m=self.target_bounds,
            run_config=self.run_config,
            reference_rotation_quaternion_xyzw=(
                reference_rotation_quaternion_xyzw
            ),
            multistart_count=self.multistart_count,
            seed_sampling_limits=self.seed_sampling_limits,
        )


def _parse_run_config(value: Any) -> PoseRunConfiguration:
    data = _mapping(value, 'run_config')
    expected = {
        'max_retries',
        'stage_timeouts_sec',
        'right_park_position_tolerance_rad',
        'soft_joint_limits_rad',
        'collision_margin_m',
        'target_position_tolerance_m',
        'duplicate_target_position_tolerance_m',
        'duplicate_ik_seed_tolerance_rad',
        'duplicate_resolved_joint_tolerance_rad',
        'axis_parallelism_tolerance',
        'covariance_rank_tolerance',
    }
    _exact_keys(data, expected, 'run_config')
    timeout_data = _mapping(data['stage_timeouts_sec'], 'stage_timeouts_sec')
    _exact_keys(
        timeout_data,
        set(RequiredStageTimeouts.FIELD_NAMES),
        'stage_timeouts_sec',
    )
    limit_data = _mapping(data['soft_joint_limits_rad'], 'soft_joint_limits')
    _exact_keys(limit_data, set(LEFT_ARM_JOINT_NAMES), 'soft_joint_limits')
    intervals = tuple(
        _interval(limit_data[name], f'soft_joint_limits.{name}')
        for name in LEFT_ARM_JOINT_NAMES
    )
    return PoseRunConfiguration(
        max_retries=data['max_retries'],
        stage_timeouts=RequiredStageTimeouts(**timeout_data),
        right_park_position_tolerance_rad=(
            data['right_park_position_tolerance_rad']
        ),
        soft_joint_limits=SoftJointLimits(
            joint_names=LEFT_ARM_JOINT_NAMES,
            lower_rad=tuple(item.minimum for item in intervals),
            upper_rad=tuple(item.maximum for item in intervals),
        ),
        collision_margin_m=data['collision_margin_m'],
        target_position_tolerance_m=data['target_position_tolerance_m'],
        duplicate_target_position_tolerance_m=(
            data['duplicate_target_position_tolerance_m']
        ),
        duplicate_ik_seed_tolerance_rad=(
            data['duplicate_ik_seed_tolerance_rad']
        ),
        duplicate_resolved_joint_tolerance_rad=(
            data['duplicate_resolved_joint_tolerance_rad']
        ),
        axis_parallelism_tolerance=data['axis_parallelism_tolerance'],
        covariance_rank_tolerance=data['covariance_rank_tolerance'],
    )


def _parse_motion(value: Any, run: PoseRunConfiguration) -> MujocoMotionConfig:
    data = _mapping(value, 'motion')
    expected = {
        'current_state_max_age_sec',
        'max_velocity_scaling_factor',
        'max_acceleration_scaling_factor',
        'controller_path_tolerance_rad',
        'controller_goal_tolerance_rad',
        'settle_position_tolerance_rad',
        'settle_velocity_tolerance_rad_s',
        'settle_duration_sec',
        'planning_attempts',
    }
    _exact_keys(data, expected, 'motion')
    timeout = run.stage_timeouts
    assert run.right_park_position_tolerance_rad is not None
    return MujocoMotionConfig(
        current_state_max_age_sec=data['current_state_max_age_sec'],
        right_park_position_tolerance_rad=(
            run.right_park_position_tolerance_rad
        ),
        stage_timeouts=StageTimeouts(
            ik_sec=timeout.ik_sec,
            state_validity_sec=timeout.state_validity_sec,
            plan_sec=timeout.plan_sec,
            execute_sec=timeout.execute_sec,
            cancel_sec=timeout.cancel_sec,
            settle_sec=timeout.settle_sec,
        ),
        max_velocity_scaling_factor=data['max_velocity_scaling_factor'],
        max_acceleration_scaling_factor=(
            data['max_acceleration_scaling_factor']
        ),
        controller_path_tolerance_rad=data['controller_path_tolerance_rad'],
        controller_goal_tolerance_rad=data['controller_goal_tolerance_rad'],
        settle_position_tolerance_rad=data['settle_position_tolerance_rad'],
        settle_velocity_tolerance_rad_s=(
            data['settle_velocity_tolerance_rad_s']
        ),
        settle_duration_sec=data['settle_duration_sec'],
        planning_attempts=data['planning_attempts'],
    )


def load_mujoco_pose_generation_profile(
    path: str | Path,
) -> MujocoPoseGenerationProfile:
    source = Path(path).expanduser().resolve(strict=True)
    try:
        root = yaml.load(
            source.read_text(encoding='utf-8'),
            Loader=_UniqueKeyLoader,
        )
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise PoseGenerationProfileError(
            f'cannot load profile: {error}'
        ) from error
    data = _mapping(root, 'profile')
    expected = {
        'schema_version',
        'generator',
        'run_config',
        'motion',
        'visibility',
        'feedback_buffer',
        'expected_resolved_match_tolerance_rad',
    }
    _exact_keys(data, expected, 'profile')
    if data['schema_version'] != SCHEMA_VERSION:
        raise PoseGenerationProfileError('unsupported schema_version')
    generator = _mapping(data['generator'], 'generator')
    _exact_keys(
        generator,
        {
            'random_seed',
            'candidate_pool_size',
            'max_generation_attempts',
            'multistart_count',
            'log_det_epsilon',
            'target_seed_local_radius_m',
            'seed_sampling_limits_rad',
            'target_position_bounds_m',
        },
        'generator',
    )
    bounds = _mapping(
        generator['target_position_bounds_m'],
        'target_position_bounds_m',
    )
    _exact_keys(bounds, {'x', 'y', 'z'}, 'target_position_bounds_m')
    sampling_data = _mapping(
        generator['seed_sampling_limits_rad'],
        'seed_sampling_limits_rad',
    )
    _exact_keys(
        sampling_data,
        set(LEFT_ARM_JOINT_NAMES),
        'seed_sampling_limits_rad',
    )
    sampling_intervals = tuple(
        _interval(
            sampling_data[name],
            f'seed_sampling_limits_rad.{name}',
        )
        for name in LEFT_ARM_JOINT_NAMES
    )
    run = _parse_run_config(data['run_config'])
    visibility = _mapping(data['visibility'], 'visibility')
    _exact_keys(
        visibility,
        {'minimum_camera_depth_m', 'image_border_fraction'},
        'visibility',
    )
    feedback = _mapping(data['feedback_buffer'], 'feedback_buffer')
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
    return MujocoPoseGenerationProfile(
        random_seed=generator['random_seed'],
        candidate_pool_size=generator['candidate_pool_size'],
        max_generation_attempts=generator['max_generation_attempts'],
        multistart_count=generator['multistart_count'],
        log_det_epsilon=generator['log_det_epsilon'],
        target_seed_local_radius_m=tuple(
            generator['target_seed_local_radius_m']
        ),
        seed_sampling_limits=SoftJointLimits(
            joint_names=LEFT_ARM_JOINT_NAMES,
            lower_rad=tuple(
                item.minimum for item in sampling_intervals
            ),
            upper_rad=tuple(
                item.maximum for item in sampling_intervals
            ),
        ),
        target_bounds=CartesianBounds(
            x_m=_interval(bounds['x'], 'target_position_bounds_m.x'),
            y_m=_interval(bounds['y'], 'target_position_bounds_m.y'),
            z_m=_interval(bounds['z'], 'target_position_bounds_m.z'),
        ),
        run_config=run,
        motion=_parse_motion(data['motion'], run),
        visibility=VisibilityProfile(**visibility),
        feedback=FeedbackBufferConfig(**feedback),
        expected_resolved_match_tolerance_rad=(
            data['expected_resolved_match_tolerance_rad']
        ),
    )


__all__ = [
    'MujocoPoseGenerationProfile',
    'PoseGenerationProfileError',
    'SCHEMA_VERSION',
    'VisibilityProfile',
    'load_mujoco_pose_generation_profile',
]
