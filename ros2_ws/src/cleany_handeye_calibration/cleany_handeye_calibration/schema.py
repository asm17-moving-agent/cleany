"""Version-1 synchronized hand-eye sample schema.

The row intentionally carries every input needed to repeat PnP and the later
noise experiments.  In particular, it records the exact camera calibration,
ordered ChArUco correspondences, complete dual-arm feedback, interpolation
provenance, and both transforms.  Persistence and image integrity are handled
by :mod:`dataset_writer`.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import PurePosixPath
import re
from typing import Any, Sequence

from cleany_handeye_calibration.camera_acquisition import (
    CAMERA_D,
    CAMERA_DISTORTION_MODEL,
    CAMERA_FRAME_ID,
    CAMERA_HEIGHT,
    CAMERA_K,
    CAMERA_P,
    CAMERA_R,
    CAMERA_WIDTH,
    CameraInfoFrame,
)
from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    LEFT_ARM_JOINT_NAMES,
)
from cleany_handeye_calibration.models import (
    CalibrationSample,
    JointPose,
    PositionTarget,
    TimedJointSample,
)
from cleany_handeye_calibration.target_detector import (
    QUADRANTS,
    CharucoDetection,
    analyze_charuco_corners,
)
from cleany_handeye_calibration.transforms import RigidTransform


SAMPLE_RECORD_SCHEMA_VERSION = 1
CAMERA_CALIBRATION_HASH_SCHEMA = 'cleany-camera-calibration-v1'
CORNER_POINT_ORDERING = 'charuco_corner_id_ascending'
_ARTIFACT_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')


def _non_empty_text(value: str, *, field_name: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f'{field_name} must be a non-empty trimmed string')
    return value


def _stamp_ns(value: int, *, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f'{field_name} must be a non-negative integer')
    return value


def _non_negative_integer(value: int, *, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f'{field_name} must be a non-negative integer')
    return value


def _points_equal(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
    *,
    tolerance: float,
) -> bool:
    if len(left) != len(right):
        return False
    return all(
        len(left_point) == len(right_point)
        and all(
            math.isclose(
                float(left_value),
                float(right_value),
                rel_tol=0.0,
                abs_tol=tolerance,
            )
            for left_value, right_value in zip(
                left_point,
                right_point,
                strict=True,
            )
        )
        for left_point, right_point in zip(left, right, strict=True)
    )


def camera_calibration_to_mapping(
    camera_info: CameraInfoFrame,
) -> dict[str, Any]:
    """Return the exact geometry fields used by PnP."""

    if not isinstance(camera_info, CameraInfoFrame):
        raise ValueError('camera_info must be a CameraInfoFrame')
    return {
        'hash_schema': CAMERA_CALIBRATION_HASH_SCHEMA,
        'frame_id': camera_info.frame_id,
        'width': camera_info.width,
        'height': camera_info.height,
        'distortion_model': camera_info.distortion_model,
        'K': list(camera_info.k),
        'D': list(camera_info.d),
        'R': list(camera_info.r),
        'P': list(camera_info.p),
    }


def camera_calibration_sha256(camera_info: CameraInfoFrame) -> str:
    """Hash a canonical JSON representation of the PnP camera inputs."""

    payload = json.dumps(
        camera_calibration_to_mapping(camera_info),
        sort_keys=True,
        separators=(',', ':'),
        ensure_ascii=True,
        allow_nan=False,
    ).encode('ascii')
    return hashlib.sha256(payload).hexdigest()


def _validate_fixed_camera_contract(camera_info: CameraInfoFrame) -> None:
    if camera_info.stamp_ns == 0:
        raise ValueError('camera_info stamp must be nonzero')
    actual = (
        camera_info.frame_id,
        camera_info.width,
        camera_info.height,
        camera_info.distortion_model,
        camera_info.k,
        camera_info.d,
        camera_info.r,
        camera_info.p,
    )
    expected = (
        CAMERA_FRAME_ID,
        CAMERA_WIDTH,
        CAMERA_HEIGHT,
        CAMERA_DISTORTION_MODEL,
        CAMERA_K,
        CAMERA_D,
        CAMERA_R,
        CAMERA_P,
    )
    if actual != expected:
        raise ValueError(
            'camera_info must match the fixed wrist-camera contract exactly'
        )


def _validate_detection(detection: CharucoDetection) -> None:
    if not isinstance(detection, CharucoDetection) or not detection.valid:
        raise ValueError('target_detection must be a valid ChArUco detection')
    validated = analyze_charuco_corners(
        detection.corner_ids,
        detection.image_points_px,
    )
    if not validated.valid:
        raise ValueError('target_detection violates the ChArUco contract')
    if detection.corner_ids != validated.corner_ids:
        raise ValueError(
            'target_detection corner IDs must use ascending canonical order'
        )
    if detection.covered_quadrants != QUADRANTS:
        raise ValueError(
            'target_detection must cover all four target quadrants'
        )
    if not _points_equal(
        detection.image_points_px,
        validated.image_points_px,
        tolerance=0.0,
    ):
        raise ValueError('target_detection image points are inconsistent')
    if not _points_equal(
        detection.object_points_m,
        validated.object_points_m,
        tolerance=1.0e-12,
    ):
        raise ValueError(
            'target_detection object points do not match canonical corner IDs'
        )


def _validate_image_path(sample_id: str, image_path: str) -> str:
    if not _ARTIFACT_ID_PATTERN.fullmatch(sample_id):
        raise ValueError(
            'sample_id must contain only letters, digits, dot, underscore, '
            'or hyphen for artifact storage'
        )
    path_text = _non_empty_text(image_path, field_name='image_path')
    path = PurePosixPath(path_text)
    if path.is_absolute() or '..' in path.parts:
        raise ValueError('image_path must be a safe relative POSIX path')
    expected = PurePosixPath('images') / f'{sample_id}.png'
    if path != expected:
        raise ValueError(f'image_path must be {expected.as_posix()!r}')
    return path.as_posix()


@dataclass(frozen=True, slots=True)
class CalibrationSampleRecord:
    """One fully materialized, synchronized sample before persistence."""

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
    camera_info: CameraInfoFrame
    target_detection: CharucoDetection
    pnp_method: str
    pnp_reprojection_rmse_px: float
    pnp_ambiguous: bool
    pnp_selected_candidate_index: int
    image_path: str

    def __post_init__(self) -> None:
        if not isinstance(self.sample, CalibrationSample):
            raise ValueError('sample must be a CalibrationSample')
        calibration_arm = _non_empty_text(
            self.calibration_arm,
            field_name='calibration_arm',
        )
        planning_group = _non_empty_text(
            self.planning_group,
            field_name='planning_group',
        )
        if calibration_arm != 'left' or planning_group != 'left_arm':
            raise ValueError(
                'calibration samples require arm left and group left_arm'
            )
        if not isinstance(self.position_target, PositionTarget):
            raise ValueError('position_target must be a PositionTarget')
        if self.position_target.frame_id != 'base_link':
            raise ValueError('position_target must use base_link')
        if not isinstance(self.ik_seed, JointPose):
            raise ValueError('ik_seed must be a JointPose')
        if not isinstance(self.resolved_ik, JointPose):
            raise ValueError('resolved_ik must be a JointPose')
        if (
            self.ik_seed.joint_names != LEFT_ARM_JOINT_NAMES
            or self.resolved_ik.joint_names != LEFT_ARM_JOINT_NAMES
        ):
            raise ValueError(
                'ik_seed and resolved_ik must use the canonical five '
                'left-arm joints'
            )

        image_stamp = _stamp_ns(
            self.image_stamp_ns,
            field_name='image_stamp_ns',
        )
        if image_stamp == 0:
            raise ValueError('image_stamp_ns must be nonzero')
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
        required_joints = (
            DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
        )
        if self.interpolated_joints.joint_names != required_joints:
            raise ValueError(
                'interpolated_joints must contain exactly the canonical 12 '
                'dual-arm/gripper joints in canonical order'
            )

        try:
            ratio = float(self.joint_interpolation_ratio)
        except (TypeError, ValueError) as error:
            raise ValueError(
                'joint_interpolation_ratio must be numeric'
            ) from error
        expected_ratio = (
            (image_stamp - before_stamp) / (after_stamp - before_stamp)
        )
        if (
            not math.isfinite(ratio)
            or not 0.0 <= ratio <= 1.0
            or not math.isclose(
                ratio,
                expected_ratio,
                rel_tol=0.0,
                abs_tol=1.0e-15,
            )
        ):
            raise ValueError(
                'joint_interpolation_ratio must be finite and match the '
                'source timestamps'
            )

        if not isinstance(self.camera_info, CameraInfoFrame):
            raise ValueError('camera_info must be a CameraInfoFrame')
        _validate_fixed_camera_contract(self.camera_info)
        if self.camera_info.stamp_ns != image_stamp:
            raise ValueError(
                'camera_info stamp must equal image_stamp_ns exactly'
            )
        _validate_detection(self.target_detection)

        pnp_method = _non_empty_text(
            self.pnp_method,
            field_name='pnp_method',
        )
        if pnp_method != 'SOLVEPNP_IPPE':
            raise ValueError('pnp_method must be SOLVEPNP_IPPE')
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
        if self.pnp_ambiguous:
            raise ValueError('ambiguous PnP results cannot be recorded')
        selected_index = _non_negative_integer(
            self.pnp_selected_candidate_index,
            field_name='pnp_selected_candidate_index',
        )
        image_path = _validate_image_path(
            self.sample.sample_id,
            self.image_path,
        )

        base_to_gripper = self.sample.base_T_gripper
        if (
            base_to_gripper.parent_frame != 'base_link'
            or base_to_gripper.child_frame != 'left_gripper_frame'
        ):
            raise ValueError(
                'base_to_gripper must be base_link -> left_gripper_frame'
            )
        camera_to_target = self.sample.camera_T_target
        if (
            camera_to_target.parent_frame != CAMERA_FRAME_ID
            or camera_to_target.child_frame != 'charuco_target'
        ):
            raise ValueError(
                'camera_to_target must be wrist optical frame -> '
                'charuco_target'
            )

        object.__setattr__(self, 'calibration_arm', calibration_arm)
        object.__setattr__(self, 'planning_group', planning_group)
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
        object.__setattr__(
            self,
            'pnp_selected_candidate_index',
            selected_index,
        )
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
    """Return the version-1 row using only JSON-compatible values."""

    if not isinstance(record, CalibrationSampleRecord):
        raise ValueError('record must be a CalibrationSampleRecord')
    velocities = record.interpolated_joints.velocities_rad_s
    calibration = camera_calibration_to_mapping(record.camera_info)
    calibration_hash = camera_calibration_sha256(record.camera_info)
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
        'camera_info_stamp_ns': record.camera_info.stamp_ns,
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
        'camera_calibration': calibration,
        'camera_calibration_sha256': calibration_hash,
        'target_detection': {
            'point_ordering': CORNER_POINT_ORDERING,
            'corner_ids': list(record.target_detection.corner_ids),
            'object_points_m': [
                list(point)
                for point in record.target_detection.object_points_m
            ],
            'image_points_px': [
                list(point)
                for point in record.target_detection.image_points_px
            ],
            'covered_quadrants': list(
                record.target_detection.covered_quadrants
            ),
        },
        'pnp': {
            'method': record.pnp_method,
            'reprojection_rmse_px': record.pnp_reprojection_rmse_px,
            'ambiguous': record.pnp_ambiguous,
            'selected_candidate_index': (
                record.pnp_selected_candidate_index
            ),
        },
        'image_path': record.image_path,
    }


def _camera_info_from_mapping(
    values: Mapping[str, Any],
    *,
    stamp_ns: int,
) -> CameraInfoFrame:
    if not isinstance(values, Mapping):
        raise ValueError('camera_calibration must be a mapping')
    try:
        if values['hash_schema'] != CAMERA_CALIBRATION_HASH_SCHEMA:
            raise ValueError('unsupported camera calibration hash schema')
        return CameraInfoFrame(
            stamp_ns=stamp_ns,
            frame_id=values['frame_id'],
            width=values['width'],
            height=values['height'],
            distortion_model=values['distortion_model'],
            d=values['D'],
            k=values['K'],
            r=values['R'],
            p=values['P'],
        )
    except KeyError as error:
        raise ValueError(
            f'camera_calibration is missing {error.args[0]}'
        ) from error


def sample_record_from_mapping(
    values: Mapping[str, Any],
) -> CalibrationSampleRecord:
    """Parse and validate one complete version-1 dataset row."""

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
        detection_values = values['target_detection']
        if not isinstance(detection_values, Mapping):
            raise ValueError('target_detection must be a mapping')
        if detection_values['point_ordering'] != CORNER_POINT_ORDERING:
            raise ValueError('unsupported target point ordering')
        image_stamp_ns = values['image_stamp_ns']
        if values['camera_info_stamp_ns'] != image_stamp_ns:
            raise ValueError(
                'camera_info_stamp_ns must equal image_stamp_ns exactly'
            )
        camera_info = _camera_info_from_mapping(
            values['camera_calibration'],
            stamp_ns=image_stamp_ns,
        )
        if values['camera_calibration_sha256'] != (
            camera_calibration_sha256(camera_info)
        ):
            raise ValueError('camera calibration SHA-256 does not match')
        ik_seed = JointPose(
            joint_names=values['ik_seed_joint_names'],
            positions_rad=values['ik_seed_positions_rad'],
        )
        resolved_ik = JointPose(
            joint_names=values['ik_seed_joint_names'],
            positions_rad=values['resolved_ik_positions_rad'],
        )
        joint_sample = TimedJointSample(
            stamp_ns=image_stamp_ns,
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
        detection = CharucoDetection(
            valid=True,
            failure_reason=None,
            corner_ids=tuple(detection_values['corner_ids']),
            image_points_px=tuple(
                tuple(point)
                for point in detection_values['image_points_px']
            ),
            object_points_m=tuple(
                tuple(point)
                for point in detection_values['object_points_m']
            ),
            covered_quadrants=tuple(
                detection_values['covered_quadrants']
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
            image_stamp_ns=image_stamp_ns,
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
            camera_info=camera_info,
            target_detection=detection,
            pnp_method=pnp['method'],
            pnp_reprojection_rmse_px=pnp['reprojection_rmse_px'],
            pnp_ambiguous=pnp['ambiguous'],
            pnp_selected_candidate_index=pnp[
                'selected_candidate_index'
            ],
            image_path=values['image_path'],
        )
    except KeyError as error:
        raise ValueError(
            f'sample record is missing {error.args[0]}'
        ) from error
