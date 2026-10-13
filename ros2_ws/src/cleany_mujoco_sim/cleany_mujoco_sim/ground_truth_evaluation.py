from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence


class GroundTruthError(ValueError):
    """Raised for invalid or frame-incompatible evaluation transforms."""


@dataclass(frozen=True)
class RigidTransform:
    parent_frame: str
    child_frame: str
    translation_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class TransformError:
    translation_m: float
    rotation_deg: float


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise GroundTruthError(f'{label} must be a mapping')
    return value


def _vector(value: Any, length: int, label: str) -> tuple[float, ...]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != length
        or any(
            isinstance(item, bool)
            or not isinstance(item, (int, float))
            or not math.isfinite(float(item))
            for item in value
        )
    ):
        raise GroundTruthError(f'{label} must contain {length} finite numbers')
    return tuple(float(item) for item in value)


def _validated_transform(transform: RigidTransform) -> RigidTransform:
    if not transform.parent_frame or not transform.child_frame:
        raise GroundTruthError('transform frames must be non-empty')
    quaternion_norm = math.sqrt(
        sum(component * component for component in transform.quaternion_xyzw)
    )
    if not math.isclose(quaternion_norm, 1.0, abs_tol=1.0e-9):
        raise GroundTruthError('transform quaternion must be normalized')
    return transform


def camera_ground_truth(manifest_data: Mapping[str, Any]) -> RigidTransform:
    """Return simulation-only GT without importing calibration solver code."""

    evaluation = _mapping(
        manifest_data.get('evaluation_ground_truth'),
        'evaluation_ground_truth',
    )
    camera = _mapping(evaluation.get('camera_transform'), 'camera_transform')
    if camera.get('semantics') != 'left_gripper_T_left_wrist_rgb_optical':
        raise GroundTruthError('camera GT semantics are invalid')
    if camera.get('evaluation_only') is not True:
        raise GroundTruthError('camera GT must be evaluation_only')
    if camera.get('allowed_for_solver_input') is not False:
        raise GroundTruthError('camera GT cannot be solver input')
    if camera.get('published_to_tf') is not False:
        raise GroundTruthError('camera GT cannot be published to TF')

    translation = _vector(camera.get('translation_m'), 3, 'translation_m')
    quaternion = _vector(
        camera.get('quaternion_xyzw'), 4, 'quaternion_xyzw'
    )
    return _validated_transform(
        RigidTransform(
            parent_frame=str(camera.get('parent_frame', '')),
            child_frame=str(camera.get('child_frame', '')),
            translation_m=(translation[0], translation[1], translation[2]),
            quaternion_xyzw=(
                quaternion[0],
                quaternion[1],
                quaternion[2],
                quaternion[3],
            ),
        )
    )


def evaluate_transform(
    candidate: RigidTransform,
    ground_truth: RigidTransform,
) -> TransformError:
    """Evaluate a candidate transform; this function has no ROS publishers."""

    candidate = _validated_transform(candidate)
    ground_truth = _validated_transform(ground_truth)
    if (
        candidate.parent_frame != ground_truth.parent_frame
        or candidate.child_frame != ground_truth.child_frame
    ):
        raise GroundTruthError(
            'candidate and ground-truth transform frames do not match'
        )
    translation_error = math.sqrt(
        sum(
            (candidate_value - truth_value) ** 2
            for candidate_value, truth_value in zip(
                candidate.translation_m,
                ground_truth.translation_m,
                strict=True,
            )
        )
    )
    quaternion_dot = abs(
        sum(
            candidate_value * truth_value
            for candidate_value, truth_value in zip(
                candidate.quaternion_xyzw,
                ground_truth.quaternion_xyzw,
                strict=True,
            )
        )
    )
    quaternion_dot = min(1.0, max(-1.0, quaternion_dot))
    rotation_error = math.degrees(2.0 * math.acos(quaternion_dot))
    return TransformError(
        translation_m=translation_error,
        rotation_deg=rotation_error,
    )
