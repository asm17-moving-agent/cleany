"""Versioned, JSON/YAML-safe draft schema for calibration samples."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
from typing import Any

from cleany_handeye_calibration.models import (
    CalibrationSample,
    JointPose,
    PositionTarget,
    TimedJointSample,
)
from cleany_handeye_calibration.transforms import RigidTransform


SAMPLE_RECORD_SCHEMA_VERSION = 1


def _non_empty_text(value: str, *, field_name: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f'{field_name} must be a non-empty trimmed string')
    return value


def _stamp_ns(value: int, *, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f'{field_name} must be a non-negative integer')
    return value


@dataclass(frozen=True, slots=True)
class CalibrationSampleRecord:
    """Materialized fields required for one dataset sample record."""

    sample: CalibrationSample
    calibration_arm: str
    planning_group: str
    position_target: PositionTarget
    ik_seed: JointPose
    resolved_ik: JointPose
    image_stamp_ns: int
    joint_state_before_stamp_ns: int
    joint_state_after_stamp_ns: int
    joint_interpolation_ratio: float
    interpolated_joints: TimedJointSample
    pnp_method: str
    pnp_reprojection_rmse_px: float
    pnp_ambiguous: bool
    image_path: str

    def __post_init__(self) -> None:
        if not isinstance(self.sample, CalibrationSample):
            raise ValueError('sample must be a CalibrationSample')
        object.__setattr__(
            self,
            'calibration_arm',
            _non_empty_text(
                self.calibration_arm,
                field_name='calibration_arm',
            ),
        )
        object.__setattr__(
            self,
            'planning_group',
            _non_empty_text(
                self.planning_group,
                field_name='planning_group',
            ),
        )
        if not isinstance(self.position_target, PositionTarget):
            raise ValueError('position_target must be a PositionTarget')
        if not isinstance(self.ik_seed, JointPose):
            raise ValueError('ik_seed must be a JointPose')
        if not isinstance(self.resolved_ik, JointPose):
            raise ValueError('resolved_ik must be a JointPose')
        if self.ik_seed.joint_names != self.resolved_ik.joint_names:
            raise ValueError(
                'ik_seed and resolved_ik must use the same ordered joint names'
            )

        image_stamp = _stamp_ns(
            self.image_stamp_ns,
            field_name='image_stamp_ns',
        )
        before_stamp = _stamp_ns(
            self.joint_state_before_stamp_ns,
            field_name='joint_state_before_stamp_ns',
        )
        after_stamp = _stamp_ns(
            self.joint_state_after_stamp_ns,
            field_name='joint_state_after_stamp_ns',
        )
        if before_stamp >= after_stamp:
            raise ValueError(
                'joint-state interpolation requires two ordered timestamps'
            )
        if not before_stamp <= image_stamp <= after_stamp:
            raise ValueError(
                'image_stamp_ns must be bracketed by joint-state timestamps'
            )
        if not isinstance(self.interpolated_joints, TimedJointSample):
            raise ValueError(
                'interpolated_joints must be a TimedJointSample'
            )
        if self.interpolated_joints.stamp_ns != image_stamp:
            raise ValueError(
                'interpolated_joints stamp must equal image_stamp_ns'
            )

        try:
            ratio = float(self.joint_interpolation_ratio)
        except (TypeError, ValueError) as error:
            raise ValueError(
                'joint_interpolation_ratio must be numeric'
            ) from error
        if not math.isfinite(ratio) or not 0.0 <= ratio <= 1.0:
            raise ValueError(
                'joint_interpolation_ratio must be finite and in [0, 1]'
            )

        pnp_method = _non_empty_text(
            self.pnp_method,
            field_name='pnp_method',
        )
        try:
            rmse = float(self.pnp_reprojection_rmse_px)
        except (TypeError, ValueError) as error:
            raise ValueError(
                'pnp_reprojection_rmse_px must be numeric'
            ) from error
        if not math.isfinite(rmse) or rmse < 0.0:
            raise ValueError(
                'pnp_reprojection_rmse_px must be finite and non-negative'
            )
        if not isinstance(self.pnp_ambiguous, bool):
            raise ValueError('pnp_ambiguous must be a bool')
        image_path = _non_empty_text(
            self.image_path,
            field_name='image_path',
        )

        object.__setattr__(self, 'image_stamp_ns', image_stamp)
        object.__setattr__(
            self,
            'joint_state_before_stamp_ns',
            before_stamp,
        )
        object.__setattr__(
            self,
            'joint_state_after_stamp_ns',
            after_stamp,
        )
        object.__setattr__(self, 'joint_interpolation_ratio', ratio)
        object.__setattr__(self, 'pnp_method', pnp_method)
        object.__setattr__(self, 'pnp_reprojection_rmse_px', rmse)
        object.__setattr__(self, 'image_path', image_path)


def transform_to_mapping(transform: RigidTransform) -> dict[str, Any]:
    if not isinstance(transform, RigidTransform):
        raise ValueError('transform must be a RigidTransform')
    return {
        'parent_frame': transform.parent_frame,
        'child_frame': transform.child_frame,
        'translation_m': list(transform.translation_m),
        'quaternion_xyzw': list(transform.as_quaternion_xyzw()),
    }


def transform_from_mapping(values: Mapping[str, Any]) -> RigidTransform:
    if not isinstance(values, Mapping):
        raise ValueError('transform record must be a mapping')
    try:
        return RigidTransform.from_quaternion_xyzw(
            parent_frame=values['parent_frame'],
            child_frame=values['child_frame'],
            translation_m=values['translation_m'],
            quaternion_xyzw=values['quaternion_xyzw'],
        )
    except KeyError as error:
        raise ValueError(
            f'transform record is missing {error.args[0]}'
        ) from error


def sample_record_to_mapping(
    record: CalibrationSampleRecord,
) -> dict[str, Any]:
    """Return the version-1 dataset record using only builtin containers."""

    if not isinstance(record, CalibrationSampleRecord):
        raise ValueError('record must be a CalibrationSampleRecord')
    velocities = record.interpolated_joints.velocities_rad_s
    return {
        'schema_version': SAMPLE_RECORD_SCHEMA_VERSION,
        'sample_id': record.sample.sample_id,
        'pose_id': record.sample.pose_id,
        'split': record.sample.split.value,
        'calibration_arm': record.calibration_arm,
        'planning_group': record.planning_group,
        'target_frame_id': record.position_target.frame_id,
        'target_position_m': list(record.position_target.position_m),
        'joint_names': list(record.interpolated_joints.joint_names),
        'ik_seed_joint_names': list(record.ik_seed.joint_names),
        'ik_seed_positions_rad': list(record.ik_seed.positions_rad),
        'resolved_ik_positions_rad': list(
            record.resolved_ik.positions_rad
        ),
        'image_stamp_ns': record.image_stamp_ns,
        'joint_state_before_stamp_ns': (
            record.joint_state_before_stamp_ns
        ),
        'joint_state_after_stamp_ns': record.joint_state_after_stamp_ns,
        'joint_interpolation_ratio': record.joint_interpolation_ratio,
        'joint_positions_rad': list(
            record.interpolated_joints.positions_rad
        ),
        'joint_velocities_rad_s': (
            None if velocities is None else list(velocities)
        ),
        'base_to_gripper': transform_to_mapping(
            record.sample.base_T_gripper
        ),
        'camera_to_target': transform_to_mapping(
            record.sample.camera_T_target
        ),
        'pnp': {
            'method': record.pnp_method,
            'reprojection_rmse_px': record.pnp_reprojection_rmse_px,
            'ambiguous': record.pnp_ambiguous,
        },
        'image_path': record.image_path,
    }


def sample_record_from_mapping(
    values: Mapping[str, Any],
) -> CalibrationSampleRecord:
    """Parse and validate one version-1 dataset sample record."""

    if not isinstance(values, Mapping):
        raise ValueError('sample record must be a mapping')
    try:
        schema_version = values['schema_version']
        if schema_version != SAMPLE_RECORD_SCHEMA_VERSION:
            raise ValueError(
                f'unsupported sample schema_version: {schema_version!r}'
            )
        pnp = values['pnp']
        if not isinstance(pnp, Mapping):
            raise ValueError('pnp must be a mapping')
        ik_seed = JointPose(
            joint_names=values['ik_seed_joint_names'],
            positions_rad=values['ik_seed_positions_rad'],
        )
        resolved_ik = JointPose(
            joint_names=values['ik_seed_joint_names'],
            positions_rad=values['resolved_ik_positions_rad'],
        )
        joint_sample = TimedJointSample(
            stamp_ns=values['image_stamp_ns'],
            joint_names=values['joint_names'],
            positions_rad=values['joint_positions_rad'],
            velocities_rad_s=values.get('joint_velocities_rad_s'),
        )
        sample = CalibrationSample(
            sample_id=values['sample_id'],
            pose_id=values['pose_id'],
            split=values['split'],
            base_T_gripper=transform_from_mapping(
                values['base_to_gripper']
            ),
            camera_T_target=transform_from_mapping(
                values['camera_to_target']
            ),
        )
        return CalibrationSampleRecord(
            sample=sample,
            calibration_arm=values['calibration_arm'],
            planning_group=values['planning_group'],
            position_target=PositionTarget(
                frame_id=values['target_frame_id'],
                position_m=values['target_position_m'],
            ),
            ik_seed=ik_seed,
            resolved_ik=resolved_ik,
            image_stamp_ns=values['image_stamp_ns'],
            joint_state_before_stamp_ns=values[
                'joint_state_before_stamp_ns'
            ],
            joint_state_after_stamp_ns=values[
                'joint_state_after_stamp_ns'
            ],
            joint_interpolation_ratio=values[
                'joint_interpolation_ratio'
            ],
            interpolated_joints=joint_sample,
            pnp_method=pnp['method'],
            pnp_reprojection_rmse_px=pnp['reprojection_rmse_px'],
            pnp_ambiguous=pnp['ambiguous'],
            image_path=values['image_path'],
        )
    except KeyError as error:
        raise ValueError(
            f'sample record is missing {error.args[0]}'
        ) from error
