"""Strict materialized pose-manifest schema and motion preflight.

The YAML schema keeps unresolved production decisions as explicit ``null``
values.  Structural loading may be requested for tooling, but the default
loader and :func:`preflight_pose_manifest` reject every unresolved value
before motion.  No safety tolerance is inferred from the URDF or hidden in
this module.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
from typing import Any, ClassVar
from uuid import uuid4

import numpy as np
import yaml

from cleany_handeye_calibration.joint_state_sync import LEFT_ARM_JOINT_NAMES
from cleany_handeye_calibration.models import (
    JointPose,
    PositionTarget,
    SampleSplit,
)
from cleany_handeye_calibration.pose_diversity import (
    RotationDiversity,
    evaluate_rotation_diversity,
    require_rotation_diversity,
    rotation_observations,
)
from cleany_handeye_calibration.transforms import (
    RigidTransform,
    quaternion_xyzw_from_rotation_matrix,
    rotation_matrix_from_quaternion_xyzw,
)


POSE_MANIFEST_SCHEMA_VERSION = 1
CALIBRATION_ARM = 'left'
CALIBRATION_FRAME = 'base_link'
CALIBRATION_TIP_LINK = 'left_gripper_frame'
CALIBRATION_POSE_COUNT = 20
HELD_OUT_POSE_COUNT = 5
TOTAL_POSE_COUNT = CALIBRATION_POSE_COUNT + HELD_OUT_POSE_COUNT
MAX_RETRIES = 3
RANDOM_ENGINE = 'numpy.random.PCG64'
SELECTION_STRATEGY = 'multistart_greedy_one_swap_lexicographic_v1'
_POSE_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')


class PoseManifestError(ValueError):
    """A pose manifest is malformed or unsafe to execute."""


class UnresolvedRunConfiguration(PoseManifestError):
    """Required run values are still explicit ``null`` placeholders."""

    def __init__(self, field_paths: Sequence[str]) -> None:
        self.field_paths = tuple(field_paths)
        super().__init__(
            'pose run configuration is unresolved: '
            + ', '.join(self.field_paths)
        )


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(
    loader: _UniqueKeyLoader,
    node: yaml.nodes.MappingNode,
    deep: bool = False,
) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    result: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise yaml.constructor.ConstructorError(
                'while constructing a mapping',
                node.start_mark,
                f'found duplicate key {key!r}',
                key_node.start_mark,
            )
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _mapping(value: Any, *, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PoseManifestError(f'{path} must be a mapping')
    if any(not isinstance(key, str) for key in value):
        raise PoseManifestError(f'{path} keys must be strings')
    return value


def _exact_fields(
    value: Any,
    *,
    path: str,
    fields: Sequence[str],
) -> Mapping[str, Any]:
    result = _mapping(value, path=path)
    expected = set(fields)
    actual = set(result)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing:
        raise PoseManifestError(f'{path} is missing fields: {missing!r}')
    if unknown:
        raise PoseManifestError(f'{path} has unknown fields: {unknown!r}')
    return result


def _trimmed_text(value: Any, *, path: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
    ):
        raise PoseManifestError(f'{path} must be a non-empty trimmed string')
    return value


def _artifact_id(value: Any, *, path: str) -> str:
    result = _trimmed_text(value, path=path)
    if not _POSE_ID_PATTERN.fullmatch(result):
        raise PoseManifestError(
            f'{path} must contain only letters, digits, dot, underscore, '
            'or hyphen'
        )
    return result


def _finite_float(value: Any, *, path: str) -> float:
    if isinstance(value, bool):
        raise PoseManifestError(f'{path} must be a finite number')
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise PoseManifestError(f'{path} must be a finite number') from error
    if not math.isfinite(result):
        raise PoseManifestError(f'{path} must be a finite number')
    return result


def _positive_float_or_none(value: Any, *, path: str) -> float | None:
    if value is None:
        return None
    result = _finite_float(value, path=path)
    if result <= 0.0:
        raise PoseManifestError(f'{path} must be positive when configured')
    return result


def _finite_tuple(
    value: Any,
    *,
    path: str,
    length: int,
) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise PoseManifestError(f'{path} must contain {length} numbers')
    result = tuple(
        _finite_float(item, path=f'{path}[{index}]')
        for index, item in enumerate(value)
    )
    if len(result) != length:
        raise PoseManifestError(
            f'{path} must contain exactly {length} numbers'
        )
    return result


def _strict_integer(value: Any, *, path: str, minimum: int = 0) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < minimum
    ):
        raise PoseManifestError(
            f'{path} must be an integer greater than or equal to {minimum}'
        )
    return value


def _strict_bool(value: Any, *, path: str) -> bool:
    if not isinstance(value, bool):
        raise PoseManifestError(f'{path} must be a bool')
    return value


@dataclass(frozen=True, slots=True)
class ClosedInterval:
    minimum: float
    maximum: float

    def __post_init__(self) -> None:
        minimum = _finite_float(self.minimum, path='interval minimum')
        maximum = _finite_float(self.maximum, path='interval maximum')
        if minimum >= maximum:
            raise PoseManifestError('interval minimum must be below maximum')
        object.__setattr__(self, 'minimum', minimum)
        object.__setattr__(self, 'maximum', maximum)

    def contains(self, value: float) -> bool:
        number = _finite_float(value, path='interval value')
        return self.minimum <= number <= self.maximum


@dataclass(frozen=True, slots=True)
class CartesianBounds:
    x_m: ClosedInterval
    y_m: ClosedInterval
    z_m: ClosedInterval

    def __post_init__(self) -> None:
        for field_name in ('x_m', 'y_m', 'z_m'):
            if not isinstance(getattr(self, field_name), ClosedInterval):
                raise PoseManifestError(
                    f'{field_name} must be a ClosedInterval'
                )

    def contains(self, position_m: Sequence[float]) -> bool:
        position = _finite_tuple(
            position_m,
            path='target position',
            length=3,
        )
        return all(
            bounds.contains(value)
            for bounds, value in zip(
                (self.x_m, self.y_m, self.z_m),
                position,
                strict=True,
            )
        )


@dataclass(frozen=True, slots=True)
class SoftJointLimits:
    """Explicit calibration bounds for the exact canonical left-arm order."""

    joint_names: tuple[str, ...]
    lower_rad: tuple[float, ...]
    upper_rad: tuple[float, ...]

    def __post_init__(self) -> None:
        names = tuple(self.joint_names)
        if names != LEFT_ARM_JOINT_NAMES:
            raise PoseManifestError(
                'soft joint limits must use the exact canonical five '
                'left-arm joint names in canonical order'
            )
        lower = _finite_tuple(
            self.lower_rad,
            path='soft joint lower limits',
            length=len(names),
        )
        upper = _finite_tuple(
            self.upper_rad,
            path='soft joint upper limits',
            length=len(names),
        )
        for name, low, high in zip(names, lower, upper, strict=True):
            if low >= high:
                raise PoseManifestError(
                    f'soft joint limit for {name} must have min < max'
                )
        object.__setattr__(self, 'joint_names', names)
        object.__setattr__(self, 'lower_rad', lower)
        object.__setattr__(self, 'upper_rad', upper)

    def contains(self, pose: JointPose) -> bool:
        if not isinstance(pose, JointPose):
            raise PoseManifestError('pose must be a JointPose')
        if pose.joint_names != self.joint_names:
            raise PoseManifestError(
                'joint pose does not use the canonical left-arm joint order'
            )
        return all(
            low <= value <= high
            for low, value, high in zip(
                self.lower_rad,
                pose.positions_rad,
                self.upper_rad,
                strict=True,
            )
        )


@dataclass(frozen=True, slots=True)
class RequiredStageTimeouts:
    """All independently measured stage budgets; ``None`` blocks motion."""

    ik_sec: float | None
    state_validity_sec: float | None
    plan_sec: float | None
    execute_sec: float | None
    cancel_sec: float | None
    settle_sec: float | None
    image_acquisition_sec: float | None
    target_detection_sec: float | None
    feedback_fk_sec: float | None
    record_sample_sec: float | None

    FIELD_NAMES: ClassVar[tuple[str, ...]] = (
        'ik_sec',
        'state_validity_sec',
        'plan_sec',
        'execute_sec',
        'cancel_sec',
        'settle_sec',
        'image_acquisition_sec',
        'target_detection_sec',
        'feedback_fk_sec',
        'record_sample_sec',
    )

    def __post_init__(self) -> None:
        for field_name in self.FIELD_NAMES:
            object.__setattr__(
                self,
                field_name,
                _positive_float_or_none(
                    getattr(self, field_name),
                    path=f'stage_timeouts.{field_name}',
                ),
            )

    @property
    def unresolved_fields(self) -> tuple[str, ...]:
        return tuple(
            f'run_config.stage_timeouts_sec.{field_name}'
            for field_name in self.FIELD_NAMES
            if getattr(self, field_name) is None
        )


@dataclass(frozen=True, slots=True)
class PoseRunConfiguration:
    """Safety values that must be materialized before any pose execution."""

    max_retries: int | None
    stage_timeouts: RequiredStageTimeouts
    right_park_position_tolerance_rad: float | None
    soft_joint_limits: SoftJointLimits | None
    collision_margin_m: float | None
    target_position_tolerance_m: float | None
    duplicate_target_position_tolerance_m: float | None
    duplicate_ik_seed_tolerance_rad: float | None
    duplicate_resolved_joint_tolerance_rad: float | None
    axis_parallelism_tolerance: float | None
    covariance_rank_tolerance: float | None

    _OPTIONAL_FLOAT_FIELDS: ClassVar[tuple[str, ...]] = (
        'right_park_position_tolerance_rad',
        'collision_margin_m',
        'target_position_tolerance_m',
        'duplicate_target_position_tolerance_m',
        'duplicate_ik_seed_tolerance_rad',
        'duplicate_resolved_joint_tolerance_rad',
        'axis_parallelism_tolerance',
        'covariance_rank_tolerance',
    )

    def __post_init__(self) -> None:
        if self.max_retries is not None:
            retries = _strict_integer(
                self.max_retries,
                path='run_config.max_retries',
            )
            if retries != MAX_RETRIES:
                raise PoseManifestError(
                    'run_config.max_retries must be exactly 3'
                )
        if not isinstance(self.stage_timeouts, RequiredStageTimeouts):
            raise PoseManifestError(
                'run_config.stage_timeouts must be RequiredStageTimeouts'
            )
        if self.soft_joint_limits is not None and not isinstance(
            self.soft_joint_limits,
            SoftJointLimits,
        ):
            raise PoseManifestError(
                'run_config.soft_joint_limits must be SoftJointLimits or None'
            )
        for field_name in self._OPTIONAL_FLOAT_FIELDS:
            object.__setattr__(
                self,
                field_name,
                _positive_float_or_none(
                    getattr(self, field_name),
                    path=f'run_config.{field_name}',
                ),
            )
        if (
            self.axis_parallelism_tolerance is not None
            and self.axis_parallelism_tolerance >= 1.0
        ):
            raise PoseManifestError(
                'axis_parallelism_tolerance must be in (0, 1)'
            )

    @property
    def unresolved_fields(self) -> tuple[str, ...]:
        fields = list(self.stage_timeouts.unresolved_fields)
        if self.max_retries is None:
            fields.append('run_config.max_retries')
        if self.soft_joint_limits is None:
            fields.append('run_config.soft_joint_limits_rad')
        fields.extend(
            f'run_config.{field_name}'
            for field_name in self._OPTIONAL_FLOAT_FIELDS
            if getattr(self, field_name) is None
        )
        return tuple(fields)

    def require_ready(self) -> PoseRunConfiguration:
        unresolved = self.unresolved_fields
        if unresolved:
            raise UnresolvedRunConfiguration(unresolved)
        return self


@dataclass(frozen=True, slots=True)
class GeneratorRecord:
    random_seed: int
    random_engine: str
    candidate_pool_size: int
    max_generation_attempts: int
    attempts_used: int
    log_det_epsilon: float
    target_position_bounds_m: CartesianBounds

    def __post_init__(self) -> None:
        seed = _strict_integer(self.random_seed, path='generator.random_seed')
        if seed >= 2**128:
            raise PoseManifestError(
                'generator.random_seed must be below 2**128'
            )
        if self.random_engine != RANDOM_ENGINE:
            raise PoseManifestError(
                f'generator.random_engine must be exactly {RANDOM_ENGINE!r}'
            )
        pool_size = _strict_integer(
            self.candidate_pool_size,
            path='generator.candidate_pool_size',
            minimum=TOTAL_POSE_COUNT,
        )
        attempt_cap = _strict_integer(
            self.max_generation_attempts,
            path='generator.max_generation_attempts',
            minimum=pool_size,
        )
        attempts_used = _strict_integer(
            self.attempts_used,
            path='generator.attempts_used',
            minimum=pool_size,
        )
        if attempts_used > attempt_cap:
            raise PoseManifestError(
                'generator.attempts_used must not exceed the attempt cap'
            )
        epsilon = _positive_float_or_none(
            self.log_det_epsilon,
            path='generator.log_det_epsilon',
        )
        if epsilon is None:
            raise PoseManifestError('generator.log_det_epsilon is required')
        if not isinstance(self.target_position_bounds_m, CartesianBounds):
            raise PoseManifestError(
                'generator target bounds must be CartesianBounds'
            )
        object.__setattr__(self, 'random_seed', seed)
        object.__setattr__(self, 'candidate_pool_size', pool_size)
        object.__setattr__(self, 'max_generation_attempts', attempt_cap)
        object.__setattr__(self, 'attempts_used', attempts_used)
        object.__setattr__(self, 'log_det_epsilon', epsilon)


@dataclass(frozen=True, slots=True)
class PoseValidationEvidence:
    target_position_error_m: float
    minimum_collision_distance_m: float
    planning_succeeded: bool
    target_visible: bool
    camera_front: bool

    def __post_init__(self) -> None:
        target_error = _finite_float(
            self.target_position_error_m,
            path='target_position_error_m',
        )
        collision_distance = _finite_float(
            self.minimum_collision_distance_m,
            path='minimum_collision_distance_m',
        )
        if target_error < 0.0:
            raise PoseManifestError(
                'target_position_error_m must be non-negative'
            )
        if collision_distance < 0.0:
            raise PoseManifestError(
                'minimum_collision_distance_m must be non-negative'
            )
        for field_name in (
            'planning_succeeded',
            'target_visible',
            'camera_front',
        ):
            _strict_bool(getattr(self, field_name), path=field_name)
        object.__setattr__(self, 'target_position_error_m', target_error)
        object.__setattr__(
            self,
            'minimum_collision_distance_m',
            collision_distance,
        )


@dataclass(frozen=True, slots=True)
class MaterializedPose:
    pose_id: str
    source_candidate_id: str
    split: SampleSplit
    target: PositionTarget
    ik_seed: JointPose
    resolved_joint_pose: JointPose
    base_T_gripper: RigidTransform
    validation: PoseValidationEvidence

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            'pose_id',
            _artifact_id(self.pose_id, path='pose.id'),
        )
        object.__setattr__(
            self,
            'source_candidate_id',
            _artifact_id(
                self.source_candidate_id,
                path='pose.source_candidate_id',
            ),
        )
        try:
            split = SampleSplit(self.split)
        except ValueError as error:
            raise PoseManifestError(
                'pose.split must be calibration or held_out'
            ) from error
        object.__setattr__(self, 'split', split)
        if not isinstance(self.target, PositionTarget):
            raise PoseManifestError('pose target must be a PositionTarget')
        if self.target.frame_id != CALIBRATION_FRAME:
            raise PoseManifestError('pose target frame must be base_link')
        for field_name in ('ik_seed', 'resolved_joint_pose'):
            pose = getattr(self, field_name)
            if not isinstance(pose, JointPose):
                raise PoseManifestError(f'{field_name} must be a JointPose')
            if pose.joint_names != LEFT_ARM_JOINT_NAMES:
                raise PoseManifestError(
                    f'{field_name} must use the exact canonical left joints'
                )
        if not isinstance(self.base_T_gripper, RigidTransform):
            raise PoseManifestError(
                'base_T_gripper must be a RigidTransform'
            )
        if (
            self.base_T_gripper.parent_frame != CALIBRATION_FRAME
            or self.base_T_gripper.child_frame != CALIBRATION_TIP_LINK
        ):
            raise PoseManifestError(
                'resolved FK must be base_link_T_left_gripper_frame'
            )
        if not isinstance(self.validation, PoseValidationEvidence):
            raise PoseManifestError(
                'pose validation must be PoseValidationEvidence'
            )


@dataclass(frozen=True, slots=True)
class PoseSelectionRecord:
    strategy: str
    reference_rotation_quaternion_xyzw: tuple[float, float, float, float]
    diversity: RotationDiversity

    def __post_init__(self) -> None:
        if self.strategy != SELECTION_STRATEGY:
            raise PoseManifestError(
                f'selection.strategy must be exactly {SELECTION_STRATEGY!r}'
            )
        quaternion = _finite_tuple(
            self.reference_rotation_quaternion_xyzw,
            path='selection.reference_rotation_quaternion_xyzw',
            length=4,
        )
        try:
            rotation = rotation_matrix_from_quaternion_xyzw(quaternion)
        except ValueError as error:
            raise PoseManifestError(
                'selection reference quaternion must have unit norm'
            ) from error
        canonical = quaternion_xyzw_from_rotation_matrix(rotation)
        if not isinstance(self.diversity, RotationDiversity):
            raise PoseManifestError(
                'selection.diversity must be RotationDiversity'
            )
        object.__setattr__(
            self,
            'reference_rotation_quaternion_xyzw',
            canonical,
        )


@dataclass(frozen=True, slots=True)
class PoseManifest:
    schema_version: int
    calibration_arm: str
    frame_id: str
    joint_names: tuple[str, ...]
    generator: GeneratorRecord
    run_config: PoseRunConfiguration
    selection: PoseSelectionRecord
    poses: tuple[MaterializedPose, ...]

    def __post_init__(self) -> None:
        if self.schema_version != POSE_MANIFEST_SCHEMA_VERSION:
            raise PoseManifestError('pose manifest schema_version must be 1')
        if self.calibration_arm != CALIBRATION_ARM:
            raise PoseManifestError('calibration_arm must be exactly left')
        if self.frame_id != CALIBRATION_FRAME:
            raise PoseManifestError('frame_id must be exactly base_link')
        names = tuple(self.joint_names)
        if names != LEFT_ARM_JOINT_NAMES:
            raise PoseManifestError(
                'joint_names must be the exact canonical five left-arm '
                'joint names in canonical order'
            )
        if not isinstance(self.generator, GeneratorRecord):
            raise PoseManifestError('generator must be GeneratorRecord')
        if not isinstance(self.run_config, PoseRunConfiguration):
            raise PoseManifestError('run_config must be PoseRunConfiguration')
        if not isinstance(self.selection, PoseSelectionRecord):
            raise PoseManifestError('selection must be PoseSelectionRecord')
        poses = tuple(self.poses)
        if len(poses) != TOTAL_POSE_COUNT:
            raise PoseManifestError(
                f'pose manifest must contain exactly {TOTAL_POSE_COUNT} poses'
            )
        if any(not isinstance(pose, MaterializedPose) for pose in poses):
            raise PoseManifestError(
                'poses must contain MaterializedPose values'
            )
        pose_ids = tuple(pose.pose_id for pose in poses)
        source_ids = tuple(pose.source_candidate_id for pose in poses)
        if len(set(pose_ids)) != len(pose_ids):
            raise PoseManifestError('pose IDs must be unique')
        if len(set(source_ids)) != len(source_ids):
            raise PoseManifestError('source candidate IDs must be unique')
        calibration_count = sum(
            pose.split is SampleSplit.CALIBRATION for pose in poses
        )
        held_out_count = sum(
            pose.split is SampleSplit.HELD_OUT for pose in poses
        )
        if (
            calibration_count != CALIBRATION_POSE_COUNT
            or held_out_count != HELD_OUT_POSE_COUNT
        ):
            raise PoseManifestError(
                'pose manifest requires exactly 20 calibration and 5 '
                'held-out poses'
            )
        object.__setattr__(self, 'joint_names', names)
        object.__setattr__(self, 'poses', poses)

    def poses_for_split(
        self,
        split: SampleSplit,
    ) -> tuple[MaterializedPose, ...]:
        requested = SampleSplit(split)
        return tuple(pose for pose in self.poses if pose.split is requested)


@dataclass(frozen=True, slots=True)
class ValidatedPoseManifest:
    manifest: PoseManifest
    computed_diversity: RotationDiversity


def _maximum_absolute_difference(
    left: Sequence[float],
    right: Sequence[float],
) -> float:
    return max(
        abs(float(left_value) - float(right_value))
        for left_value, right_value in zip(left, right, strict=True)
    )


def _validate_pose_safety(
    manifest: PoseManifest,
    config: PoseRunConfiguration,
) -> None:
    assert config.soft_joint_limits is not None
    assert config.collision_margin_m is not None
    assert config.target_position_tolerance_m is not None
    for pose in manifest.poses:
        if not manifest.generator.target_position_bounds_m.contains(
            pose.target.position_m
        ):
            raise PoseManifestError(
                f'pose {pose.pose_id!r} target is outside generator bounds'
            )
        if not config.soft_joint_limits.contains(pose.ik_seed):
            raise PoseManifestError(
                f'pose {pose.pose_id!r} IK seed violates soft joint limits'
            )
        if not config.soft_joint_limits.contains(pose.resolved_joint_pose):
            raise PoseManifestError(
                f'pose {pose.pose_id!r} resolved joints violate soft limits'
            )
        expected_target_error = float(
            np.linalg.norm(
                np.asarray(pose.target.position_m, dtype=np.float64)
                - np.asarray(
                    pose.base_T_gripper.translation_m,
                    dtype=np.float64,
                )
            )
        )
        if not math.isclose(
            pose.validation.target_position_error_m,
            expected_target_error,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise PoseManifestError(
                f'pose {pose.pose_id!r} target-position error evidence '
                'does not match resolved FK'
            )
        if expected_target_error > config.target_position_tolerance_m:
            raise PoseManifestError(
                f'pose {pose.pose_id!r} exceeds target-position tolerance'
            )
        if (
            pose.validation.minimum_collision_distance_m
            < config.collision_margin_m
        ):
            raise PoseManifestError(
                f'pose {pose.pose_id!r} violates collision margin'
            )
        if not pose.validation.planning_succeeded:
            raise PoseManifestError(
                f'pose {pose.pose_id!r} lacks planning-success evidence'
            )
        if not pose.validation.target_visible:
            raise PoseManifestError(
                f'pose {pose.pose_id!r} lacks target-visibility evidence'
            )
        if not pose.validation.camera_front:
            raise PoseManifestError(
                f'pose {pose.pose_id!r} target is not in front of the camera'
            )


def _validate_duplicates(
    poses: Sequence[MaterializedPose],
    config: PoseRunConfiguration,
) -> None:
    assert config.duplicate_target_position_tolerance_m is not None
    assert config.duplicate_ik_seed_tolerance_rad is not None
    assert config.duplicate_resolved_joint_tolerance_rad is not None
    for left_index, left in enumerate(poses):
        for right in poses[left_index + 1:]:
            target_distance = float(
                np.linalg.norm(
                    np.asarray(left.target.position_m, dtype=np.float64)
                    - np.asarray(right.target.position_m, dtype=np.float64)
                )
            )
            seed_difference = _maximum_absolute_difference(
                left.ik_seed.positions_rad,
                right.ik_seed.positions_rad,
            )
            resolved_difference = _maximum_absolute_difference(
                left.resolved_joint_pose.positions_rad,
                right.resolved_joint_pose.positions_rad,
            )
            if (
                target_distance
                <= config.duplicate_target_position_tolerance_m
                and seed_difference
                <= config.duplicate_ik_seed_tolerance_rad
            ):
                raise PoseManifestError(
                    f'poses {left.pose_id!r} and {right.pose_id!r} have a '
                    'duplicate target/seed pair'
                )
            if (
                resolved_difference
                <= config.duplicate_resolved_joint_tolerance_rad
            ):
                raise PoseManifestError(
                    f'poses {left.pose_id!r} and {right.pose_id!r} have '
                    'duplicate resolved joints'
                )


def preflight_pose_manifest(manifest: PoseManifest) -> ValidatedPoseManifest:
    """Prove a full pose set is ready before any ROS motion request."""

    if not isinstance(manifest, PoseManifest):
        raise PoseManifestError('manifest must be a PoseManifest')
    config = manifest.run_config.require_ready()
    _validate_pose_safety(manifest, config)
    _validate_duplicates(manifest.poses, config)

    reference_rotation = rotation_matrix_from_quaternion_xyzw(
        manifest.selection.reference_rotation_quaternion_xyzw
    )
    calibration_poses = manifest.poses_for_split(SampleSplit.CALIBRATION)
    observations = rotation_observations(
        [pose.pose_id for pose in calibration_poses],
        [pose.base_T_gripper.rotation_matrix for pose in calibration_poses],
        reference_rotation_matrix=reference_rotation,
    )
    assert config.axis_parallelism_tolerance is not None
    assert config.covariance_rank_tolerance is not None
    computed = evaluate_rotation_diversity(
        observations,
        log_det_epsilon=manifest.generator.log_det_epsilon,
        axis_parallelism_tolerance=config.axis_parallelism_tolerance,
        covariance_rank_tolerance=config.covariance_rank_tolerance,
    )
    require_rotation_diversity(computed)
    recorded = manifest.selection.diversity
    if not math.isclose(
        recorded.maximum_axis_parallelism,
        computed.maximum_axis_parallelism,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise PoseManifestError(
            'recorded maximum axis parallelism does not match resolved FK'
        )
    if not math.isclose(
        recorded.rotation_covariance_log_det,
        computed.rotation_covariance_log_det,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise PoseManifestError(
            'recorded covariance log-det does not match resolved FK'
        )
    if recorded.rotation_covariance_rank != computed.rotation_covariance_rank:
        raise PoseManifestError(
            'recorded covariance rank does not match resolved FK'
        )
    if (
        recorded.nonparallel_axis_pose_ids
        != computed.nonparallel_axis_pose_ids
    ):
        raise PoseManifestError(
            'recorded non-parallel-axis witness does not match resolved FK'
        )
    return ValidatedPoseManifest(
        manifest=manifest,
        computed_diversity=computed,
    )


def _parse_interval(value: Any, *, path: str) -> ClosedInterval:
    values = _finite_tuple(value, path=path, length=2)
    return ClosedInterval(values[0], values[1])


def _parse_cartesian_bounds(value: Any, *, path: str) -> CartesianBounds:
    mapping = _exact_fields(
        value,
        path=path,
        fields=('x', 'y', 'z'),
    )
    return CartesianBounds(
        x_m=_parse_interval(mapping['x'], path=f'{path}.x'),
        y_m=_parse_interval(mapping['y'], path=f'{path}.y'),
        z_m=_parse_interval(mapping['z'], path=f'{path}.z'),
    )


def _parse_soft_joint_limits(
    value: Any,
    *,
    path: str,
) -> SoftJointLimits | None:
    if value is None:
        return None
    mapping = _exact_fields(
        value,
        path=path,
        fields=LEFT_ARM_JOINT_NAMES,
    )
    intervals = tuple(
        _parse_interval(mapping[name], path=f'{path}.{name}')
        for name in LEFT_ARM_JOINT_NAMES
    )
    return SoftJointLimits(
        joint_names=LEFT_ARM_JOINT_NAMES,
        lower_rad=tuple(item.minimum for item in intervals),
        upper_rad=tuple(item.maximum for item in intervals),
    )


def _parse_stage_timeouts(value: Any, *, path: str) -> RequiredStageTimeouts:
    mapping = _exact_fields(
        value,
        path=path,
        fields=RequiredStageTimeouts.FIELD_NAMES,
    )
    return RequiredStageTimeouts(
        **{
            field_name: _positive_float_or_none(
                mapping[field_name],
                path=f'{path}.{field_name}',
            )
            for field_name in RequiredStageTimeouts.FIELD_NAMES
        }
    )


def _parse_run_config(value: Any, *, path: str) -> PoseRunConfiguration:
    fields = (
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
    )
    mapping = _exact_fields(value, path=path, fields=fields)
    max_retries = mapping['max_retries']
    if max_retries is not None:
        max_retries = _strict_integer(
            max_retries,
            path=f'{path}.max_retries',
        )
    return PoseRunConfiguration(
        max_retries=max_retries,
        stage_timeouts=_parse_stage_timeouts(
            mapping['stage_timeouts_sec'],
            path=f'{path}.stage_timeouts_sec',
        ),
        right_park_position_tolerance_rad=_positive_float_or_none(
            mapping['right_park_position_tolerance_rad'],
            path=f'{path}.right_park_position_tolerance_rad',
        ),
        soft_joint_limits=_parse_soft_joint_limits(
            mapping['soft_joint_limits_rad'],
            path=f'{path}.soft_joint_limits_rad',
        ),
        collision_margin_m=_positive_float_or_none(
            mapping['collision_margin_m'],
            path=f'{path}.collision_margin_m',
        ),
        target_position_tolerance_m=_positive_float_or_none(
            mapping['target_position_tolerance_m'],
            path=f'{path}.target_position_tolerance_m',
        ),
        duplicate_target_position_tolerance_m=_positive_float_or_none(
            mapping['duplicate_target_position_tolerance_m'],
            path=f'{path}.duplicate_target_position_tolerance_m',
        ),
        duplicate_ik_seed_tolerance_rad=_positive_float_or_none(
            mapping['duplicate_ik_seed_tolerance_rad'],
            path=f'{path}.duplicate_ik_seed_tolerance_rad',
        ),
        duplicate_resolved_joint_tolerance_rad=_positive_float_or_none(
            mapping['duplicate_resolved_joint_tolerance_rad'],
            path=f'{path}.duplicate_resolved_joint_tolerance_rad',
        ),
        axis_parallelism_tolerance=_positive_float_or_none(
            mapping['axis_parallelism_tolerance'],
            path=f'{path}.axis_parallelism_tolerance',
        ),
        covariance_rank_tolerance=_positive_float_or_none(
            mapping['covariance_rank_tolerance'],
            path=f'{path}.covariance_rank_tolerance',
        ),
    )


def _parse_generator(value: Any, *, path: str) -> GeneratorRecord:
    fields = (
        'random_seed',
        'random_engine',
        'candidate_pool_size',
        'max_generation_attempts',
        'attempts_used',
        'log_det_epsilon',
        'target_position_bounds_m',
    )
    mapping = _exact_fields(value, path=path, fields=fields)
    return GeneratorRecord(
        random_seed=_strict_integer(
            mapping['random_seed'],
            path=f'{path}.random_seed',
        ),
        random_engine=_trimmed_text(
            mapping['random_engine'],
            path=f'{path}.random_engine',
        ),
        candidate_pool_size=_strict_integer(
            mapping['candidate_pool_size'],
            path=f'{path}.candidate_pool_size',
            minimum=TOTAL_POSE_COUNT,
        ),
        max_generation_attempts=_strict_integer(
            mapping['max_generation_attempts'],
            path=f'{path}.max_generation_attempts',
            minimum=TOTAL_POSE_COUNT,
        ),
        attempts_used=_strict_integer(
            mapping['attempts_used'],
            path=f'{path}.attempts_used',
            minimum=TOTAL_POSE_COUNT,
        ),
        log_det_epsilon=_finite_float(
            mapping['log_det_epsilon'],
            path=f'{path}.log_det_epsilon',
        ),
        target_position_bounds_m=_parse_cartesian_bounds(
            mapping['target_position_bounds_m'],
            path=f'{path}.target_position_bounds_m',
        ),
    )


def _parse_transform(value: Any, *, path: str) -> RigidTransform:
    mapping = _exact_fields(
        value,
        path=path,
        fields=('translation_m', 'quaternion_xyzw'),
    )
    quaternion = _finite_tuple(
        mapping['quaternion_xyzw'],
        path=f'{path}.quaternion_xyzw',
        length=4,
    )
    try:
        rotation = rotation_matrix_from_quaternion_xyzw(quaternion)
    except ValueError as error:
        raise PoseManifestError(
            f'{path}.quaternion_xyzw must have unit norm'
        ) from error
    return RigidTransform(
        parent_frame=CALIBRATION_FRAME,
        child_frame=CALIBRATION_TIP_LINK,
        rotation_matrix=tuple(
            tuple(float(value) for value in row) for row in rotation
        ),
        translation_m=_finite_tuple(
            mapping['translation_m'],
            path=f'{path}.translation_m',
            length=3,
        ),
    )


def _parse_pose(value: Any, *, path: str) -> MaterializedPose:
    fields = (
        'id',
        'source_candidate_id',
        'split',
        'target_position_m',
        'ik_seed_positions_rad',
        'resolved',
        'validation',
    )
    mapping = _exact_fields(value, path=path, fields=fields)
    resolved = _exact_fields(
        mapping['resolved'],
        path=f'{path}.resolved',
        fields=('joint_positions_rad', 'base_to_gripper'),
    )
    validation = _exact_fields(
        mapping['validation'],
        path=f'{path}.validation',
        fields=(
            'target_position_error_m',
            'minimum_collision_distance_m',
            'planning_succeeded',
            'target_visible',
            'camera_front',
        ),
    )
    try:
        split = SampleSplit(mapping['split'])
    except (TypeError, ValueError) as error:
        raise PoseManifestError(
            f'{path}.split must be calibration or held_out'
        ) from error
    return MaterializedPose(
        pose_id=_artifact_id(mapping['id'], path=f'{path}.id'),
        source_candidate_id=_artifact_id(
            mapping['source_candidate_id'],
            path=f'{path}.source_candidate_id',
        ),
        split=split,
        target=PositionTarget(
            frame_id=CALIBRATION_FRAME,
            position_m=_finite_tuple(
                mapping['target_position_m'],
                path=f'{path}.target_position_m',
                length=3,
            ),
        ),
        ik_seed=JointPose(
            joint_names=LEFT_ARM_JOINT_NAMES,
            positions_rad=_finite_tuple(
                mapping['ik_seed_positions_rad'],
                path=f'{path}.ik_seed_positions_rad',
                length=len(LEFT_ARM_JOINT_NAMES),
            ),
        ),
        resolved_joint_pose=JointPose(
            joint_names=LEFT_ARM_JOINT_NAMES,
            positions_rad=_finite_tuple(
                resolved['joint_positions_rad'],
                path=f'{path}.resolved.joint_positions_rad',
                length=len(LEFT_ARM_JOINT_NAMES),
            ),
        ),
        base_T_gripper=_parse_transform(
            resolved['base_to_gripper'],
            path=f'{path}.resolved.base_to_gripper',
        ),
        validation=PoseValidationEvidence(
            target_position_error_m=_finite_float(
                validation['target_position_error_m'],
                path=f'{path}.validation.target_position_error_m',
            ),
            minimum_collision_distance_m=_finite_float(
                validation['minimum_collision_distance_m'],
                path=(
                    f'{path}.validation.minimum_collision_distance_m'
                ),
            ),
            planning_succeeded=_strict_bool(
                validation['planning_succeeded'],
                path=f'{path}.validation.planning_succeeded',
            ),
            target_visible=_strict_bool(
                validation['target_visible'],
                path=f'{path}.validation.target_visible',
            ),
            camera_front=_strict_bool(
                validation['camera_front'],
                path=f'{path}.validation.camera_front',
            ),
        ),
    )


def _parse_selection(value: Any, *, path: str) -> PoseSelectionRecord:
    fields = (
        'strategy',
        'reference_rotation_quaternion_xyzw',
        'maximum_axis_parallelism',
        'rotation_covariance_log_det',
        'rotation_covariance_rank',
        'nonparallel_axis_pose_ids',
    )
    mapping = _exact_fields(value, path=path, fields=fields)
    witness_value = mapping['nonparallel_axis_pose_ids']
    if isinstance(witness_value, (str, bytes)) or not isinstance(
        witness_value,
        Sequence,
    ):
        raise PoseManifestError(
            f'{path}.nonparallel_axis_pose_ids must be a sequence'
        )
    witness = tuple(
        _artifact_id(item, path=f'{path}.nonparallel_axis_pose_ids')
        for item in witness_value
    )
    return PoseSelectionRecord(
        strategy=_trimmed_text(mapping['strategy'], path=f'{path}.strategy'),
        reference_rotation_quaternion_xyzw=_finite_tuple(
            mapping['reference_rotation_quaternion_xyzw'],
            path=f'{path}.reference_rotation_quaternion_xyzw',
            length=4,
        ),
        diversity=RotationDiversity(
            maximum_axis_parallelism=_finite_float(
                mapping['maximum_axis_parallelism'],
                path=f'{path}.maximum_axis_parallelism',
            ),
            rotation_covariance_log_det=_finite_float(
                mapping['rotation_covariance_log_det'],
                path=f'{path}.rotation_covariance_log_det',
            ),
            rotation_covariance_rank=_strict_integer(
                mapping['rotation_covariance_rank'],
                path=f'{path}.rotation_covariance_rank',
            ),
            nonparallel_axis_pose_ids=witness,
        ),
    )


def pose_manifest_from_mapping(value: Any) -> PoseManifest:
    fields = (
        'schema_version',
        'calibration_arm',
        'frame_id',
        'joint_names',
        'generator',
        'run_config',
        'selection',
        'poses',
    )
    mapping = _exact_fields(value, path='manifest', fields=fields)
    joint_names_value = mapping['joint_names']
    if isinstance(joint_names_value, (str, bytes)) or not isinstance(
        joint_names_value,
        Sequence,
    ):
        raise PoseManifestError('manifest.joint_names must be a sequence')
    joint_names = tuple(
        _trimmed_text(item, path='manifest.joint_names')
        for item in joint_names_value
    )
    poses_value = mapping['poses']
    if isinstance(poses_value, (str, bytes)) or not isinstance(
        poses_value,
        Sequence,
    ):
        raise PoseManifestError('manifest.poses must be a sequence')
    return PoseManifest(
        schema_version=_strict_integer(
            mapping['schema_version'],
            path='manifest.schema_version',
            minimum=1,
        ),
        calibration_arm=_trimmed_text(
            mapping['calibration_arm'],
            path='manifest.calibration_arm',
        ),
        frame_id=_trimmed_text(
            mapping['frame_id'],
            path='manifest.frame_id',
        ),
        joint_names=joint_names,
        generator=_parse_generator(mapping['generator'], path='generator'),
        run_config=_parse_run_config(mapping['run_config'], path='run_config'),
        selection=_parse_selection(mapping['selection'], path='selection'),
        poses=tuple(
            _parse_pose(item, path=f'poses[{index}]')
            for index, item in enumerate(poses_value)
        ),
    )


def _interval_to_sequence(interval: ClosedInterval) -> list[float]:
    return [interval.minimum, interval.maximum]


def pose_manifest_to_mapping(manifest: PoseManifest) -> dict[str, Any]:
    if not isinstance(manifest, PoseManifest):
        raise PoseManifestError('manifest must be a PoseManifest')
    config = manifest.run_config
    soft_limits = None
    if config.soft_joint_limits is not None:
        soft_limits = {
            name: [low, high]
            for name, low, high in zip(
                config.soft_joint_limits.joint_names,
                config.soft_joint_limits.lower_rad,
                config.soft_joint_limits.upper_rad,
                strict=True,
            )
        }
    return {
        'schema_version': manifest.schema_version,
        'calibration_arm': manifest.calibration_arm,
        'frame_id': manifest.frame_id,
        'joint_names': list(manifest.joint_names),
        'generator': {
            'random_seed': manifest.generator.random_seed,
            'random_engine': manifest.generator.random_engine,
            'candidate_pool_size': manifest.generator.candidate_pool_size,
            'max_generation_attempts': (
                manifest.generator.max_generation_attempts
            ),
            'attempts_used': manifest.generator.attempts_used,
            'log_det_epsilon': manifest.generator.log_det_epsilon,
            'target_position_bounds_m': {
                'x': _interval_to_sequence(
                    manifest.generator.target_position_bounds_m.x_m
                ),
                'y': _interval_to_sequence(
                    manifest.generator.target_position_bounds_m.y_m
                ),
                'z': _interval_to_sequence(
                    manifest.generator.target_position_bounds_m.z_m
                ),
            },
        },
        'run_config': {
            'max_retries': config.max_retries,
            'stage_timeouts_sec': {
                field_name: getattr(config.stage_timeouts, field_name)
                for field_name in RequiredStageTimeouts.FIELD_NAMES
            },
            'right_park_position_tolerance_rad': (
                config.right_park_position_tolerance_rad
            ),
            'soft_joint_limits_rad': soft_limits,
            'collision_margin_m': config.collision_margin_m,
            'target_position_tolerance_m': (
                config.target_position_tolerance_m
            ),
            'duplicate_target_position_tolerance_m': (
                config.duplicate_target_position_tolerance_m
            ),
            'duplicate_ik_seed_tolerance_rad': (
                config.duplicate_ik_seed_tolerance_rad
            ),
            'duplicate_resolved_joint_tolerance_rad': (
                config.duplicate_resolved_joint_tolerance_rad
            ),
            'axis_parallelism_tolerance': (
                config.axis_parallelism_tolerance
            ),
            'covariance_rank_tolerance': (
                config.covariance_rank_tolerance
            ),
        },
        'selection': {
            'strategy': manifest.selection.strategy,
            'reference_rotation_quaternion_xyzw': list(
                manifest.selection.reference_rotation_quaternion_xyzw
            ),
            'maximum_axis_parallelism': (
                manifest.selection.diversity.maximum_axis_parallelism
            ),
            'rotation_covariance_log_det': (
                manifest.selection.diversity.rotation_covariance_log_det
            ),
            'rotation_covariance_rank': (
                manifest.selection.diversity.rotation_covariance_rank
            ),
            'nonparallel_axis_pose_ids': list(
                manifest.selection.diversity.nonparallel_axis_pose_ids
            ),
        },
        'poses': [
            {
                'id': pose.pose_id,
                'source_candidate_id': pose.source_candidate_id,
                'split': pose.split.value,
                'target_position_m': list(pose.target.position_m),
                'ik_seed_positions_rad': list(pose.ik_seed.positions_rad),
                'resolved': {
                    'joint_positions_rad': list(
                        pose.resolved_joint_pose.positions_rad
                    ),
                    'base_to_gripper': {
                        'translation_m': list(
                            pose.base_T_gripper.translation_m
                        ),
                        'quaternion_xyzw': list(
                            quaternion_xyzw_from_rotation_matrix(
                                pose.base_T_gripper.rotation_matrix
                            )
                        ),
                    },
                },
                'validation': {
                    'target_position_error_m': (
                        pose.validation.target_position_error_m
                    ),
                    'minimum_collision_distance_m': (
                        pose.validation.minimum_collision_distance_m
                    ),
                    'planning_succeeded': (
                        pose.validation.planning_succeeded
                    ),
                    'target_visible': pose.validation.target_visible,
                    'camera_front': pose.validation.camera_front,
                },
            }
            for pose in manifest.poses
        ],
    }


def load_pose_manifest(
    path: str | os.PathLike[str],
    *,
    require_preflight: bool = True,
) -> PoseManifest:
    manifest_path = Path(path)
    try:
        raw = yaml.load(
            manifest_path.read_text(encoding='utf-8'),
            Loader=_UniqueKeyLoader,
        )
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise PoseManifestError(
            f'failed to load pose manifest {manifest_path}: {error}'
        ) from error
    manifest = pose_manifest_from_mapping(raw)
    if require_preflight:
        preflight_pose_manifest(manifest)
    return manifest


def write_pose_manifest(
    path: str | os.PathLike[str],
    manifest: PoseManifest,
    *,
    overwrite: bool = False,
) -> None:
    """Atomically materialize a deterministic YAML pose manifest."""

    destination = Path(path)
    if destination.exists() and not overwrite:
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = yaml.safe_dump(
        pose_manifest_to_mapping(manifest),
        sort_keys=False,
        allow_unicode=False,
    ).encode('utf-8')
    temporary = destination.with_name(
        f'.{destination.name}.{uuid4().hex}.tmp'
    )
    try:
        with temporary.open('xb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        if destination.exists() and not overwrite:
            raise FileExistsError(destination)
        os.replace(temporary, destination)
        directory_fd = os.open(destination.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
