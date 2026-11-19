"""좌표계를 명시하는 강체 변환 기본 연산입니다.

RigidTransform(parent_frame, child_frame, ...)는 parent_T_child를 뜻하며
child_frame의 좌표를 parent_frame으로 변환합니다. 수치를 튜플로 저장하므로
동일한 NumPy 배열을 참조해 동결된 변환을 변경할 수 없습니다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence, TypeAlias

import cv2
import numpy as np


Vector3: TypeAlias = tuple[float, float, float]
QuaternionXyzw: TypeAlias = tuple[float, float, float, float]
RotationMatrix: TypeAlias = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]

ROTATION_ORTHOGONALITY_TOLERANCE = 1.0e-6
ROTATION_DETERMINANT_TOLERANCE = 1.0e-6
QUATERNION_NORM_TOLERANCE = 1.0e-6
HOMOGENEOUS_ROW_TOLERANCE = 1.0e-12


def _validate_frame_id(frame_id: str, *, field_name: str) -> str:
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError(f'{field_name} must be a non-empty string')
    if frame_id != frame_id.strip():
        raise ValueError(
            f'{field_name} must not contain surrounding whitespace'
        )
    return frame_id


def _finite_array(
    values: Sequence[float] | Sequence[Sequence[float]] | np.ndarray,
    *,
    shape: tuple[int, ...],
    field_name: str,
) -> np.ndarray:
    try:
        array = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'{field_name} must contain numeric values'
        ) from error
    if array.shape != shape:
        raise ValueError(
            f'{field_name} must have shape {shape}, got {array.shape}'
        )
    if not np.all(np.isfinite(array)):
        raise ValueError(f'{field_name} must contain only finite values')
    return array


def validate_rotation_matrix(
    rotation_matrix: Sequence[Sequence[float]] | np.ndarray,
) -> np.ndarray:
    """올바른 3차원 회전 행렬인지 검증한 float64 복사본을 반환합니다."""

    rotation = _finite_array(
        rotation_matrix,
        shape=(3, 3),
        field_name='rotation_matrix',
    )
    orthogonality_error = float(
        np.linalg.norm(rotation.T @ rotation - np.eye(3), ord='fro')
    )
    if orthogonality_error > ROTATION_ORTHOGONALITY_TOLERANCE:
        raise ValueError(
            'rotation_matrix is not orthonormal: '
            f'error={orthogonality_error:.9g}'
        )
    determinant = float(np.linalg.det(rotation))
    if abs(determinant - 1.0) > ROTATION_DETERMINANT_TOLERANCE:
        raise ValueError(
            'rotation_matrix must be a proper rotation: '
            f'determinant={determinant:.9g}'
        )
    return np.array(rotation, dtype=np.float64, copy=True)


def rotation_matrix_from_quaternion_xyzw(
    quaternion_xyzw: Sequence[float] | np.ndarray,
) -> np.ndarray:
    """ROS xyzw 순서의 단위 quaternion을 행렬로 변환합니다."""

    quaternion = _finite_array(
        quaternion_xyzw,
        shape=(4,),
        field_name='quaternion_xyzw',
    )
    norm = float(np.linalg.norm(quaternion))
    if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise ValueError(
            'quaternion_xyzw must have unit norm: '
            f'norm={norm:.9g}'
        )
    x, y, z, w = quaternion / norm
    rotation = np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )
    return validate_rotation_matrix(rotation)


def quaternion_xyzw_from_rotation_matrix(
    rotation_matrix: Sequence[Sequence[float]] | np.ndarray,
) -> QuaternionXyzw:
    """올바른 회전 행렬을 부호가 정규화된 단위 quaternion으로 변환합니다."""

    rotation = validate_rotation_matrix(rotation_matrix)
    trace = float(np.trace(rotation))

    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = 2.0 * math.sqrt(
            max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        )
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
        w = (rotation[2, 1] - rotation[1, 2]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = 2.0 * math.sqrt(
            max(0.0, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        )
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
        w = (rotation[0, 2] - rotation[2, 0]) / scale
    else:
        scale = 2.0 * math.sqrt(
            max(0.0, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
        )
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
        w = (rotation[1, 0] - rotation[0, 1]) / scale

    quaternion = np.array((x, y, z, w), dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)

    # q와 -q는 같은 회전입니다. 부호를 정규화해 w가 0인 180도 회전까지
    # 직렬화 결과를 결정적으로 유지합니다.
    if quaternion[3] < 0.0 or (
        abs(quaternion[3]) <= np.finfo(np.float64).eps
        and next(
            (value for value in quaternion[:3] if abs(value) > 1.0e-15),
            0.0,
        )
        < 0.0
    ):
        quaternion *= -1.0
    return (
        float(quaternion[0]),
        float(quaternion[1]),
        float(quaternion[2]),
        float(quaternion[3]),
    )


def rotation_matrix_from_rodrigues(
    rodrigues_vector: Sequence[float] | np.ndarray,
) -> np.ndarray:
    """원소 세 개의 Rodrigues 회전 벡터를 행렬로 변환합니다."""

    vector = _finite_array(
        rodrigues_vector,
        shape=(3,),
        field_name='rodrigues_vector',
    )
    rotation, _ = cv2.Rodrigues(vector.reshape(3, 1))
    return validate_rotation_matrix(rotation)


def rodrigues_from_rotation_matrix(
    rotation_matrix: Sequence[Sequence[float]] | np.ndarray,
) -> Vector3:
    """올바른 회전 행렬을 Rodrigues 회전 벡터로 변환합니다."""

    rotation = validate_rotation_matrix(rotation_matrix)
    vector, _ = cv2.Rodrigues(rotation)
    flat_vector = vector.reshape(3)
    return (
        float(flat_vector[0]),
        float(flat_vector[1]),
        float(flat_vector[2]),
    )


@dataclass(frozen=True, slots=True)
class RigidTransform:
    """미터 단위의 불변 parent_T_child 변환입니다."""

    parent_frame: str
    child_frame: str
    rotation_matrix: RotationMatrix
    translation_m: Vector3

    def __post_init__(self) -> None:
        parent_frame = _validate_frame_id(
            self.parent_frame,
            field_name='parent_frame',
        )
        child_frame = _validate_frame_id(
            self.child_frame,
            field_name='child_frame',
        )
        rotation = validate_rotation_matrix(self.rotation_matrix)
        translation = _finite_array(
            self.translation_m,
            shape=(3,),
            field_name='translation_m',
        )

        object.__setattr__(self, 'parent_frame', parent_frame)
        object.__setattr__(self, 'child_frame', child_frame)
        object.__setattr__(
            self,
            'rotation_matrix',
            tuple(
                tuple(float(value) for value in row)
                for row in rotation
            ),
        )
        object.__setattr__(
            self,
            'translation_m',
            tuple(float(value) for value in translation),
        )

    @classmethod
    def identity(cls, frame_id: str) -> RigidTransform:
        return cls(
            parent_frame=frame_id,
            child_frame=frame_id,
            rotation_matrix=np.eye(3),
            translation_m=(0.0, 0.0, 0.0),
        )

    @classmethod
    def from_quaternion_xyzw(
        cls,
        *,
        parent_frame: str,
        child_frame: str,
        translation_m: Sequence[float] | np.ndarray,
        quaternion_xyzw: Sequence[float] | np.ndarray,
    ) -> RigidTransform:
        return cls(
            parent_frame=parent_frame,
            child_frame=child_frame,
            rotation_matrix=rotation_matrix_from_quaternion_xyzw(
                quaternion_xyzw
            ),
            translation_m=translation_m,
        )

    @classmethod
    def from_rodrigues(
        cls,
        *,
        parent_frame: str,
        child_frame: str,
        translation_m: Sequence[float] | np.ndarray,
        rodrigues_vector: Sequence[float] | np.ndarray,
    ) -> RigidTransform:
        return cls(
            parent_frame=parent_frame,
            child_frame=child_frame,
            rotation_matrix=rotation_matrix_from_rodrigues(
                rodrigues_vector
            ),
            translation_m=translation_m,
        )

    @classmethod
    def from_homogeneous_matrix(
        cls,
        *,
        parent_frame: str,
        child_frame: str,
        matrix: Sequence[Sequence[float]] | np.ndarray,
    ) -> RigidTransform:
        homogeneous = _finite_array(
            matrix,
            shape=(4, 4),
            field_name='homogeneous_matrix',
        )
        if not np.allclose(
            homogeneous[3],
            np.array((0.0, 0.0, 0.0, 1.0)),
            rtol=0.0,
            atol=HOMOGENEOUS_ROW_TOLERANCE,
        ):
            raise ValueError(
                'homogeneous_matrix bottom row must be [0, 0, 0, 1]'
            )
        return cls(
            parent_frame=parent_frame,
            child_frame=child_frame,
            rotation_matrix=homogeneous[:3, :3],
            translation_m=homogeneous[:3, 3],
        )

    def rotation_array(self) -> np.ndarray:
        return np.asarray(self.rotation_matrix, dtype=np.float64)

    def translation_array(self) -> np.ndarray:
        return np.asarray(self.translation_m, dtype=np.float64)

    def as_homogeneous_matrix(self) -> np.ndarray:
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = self.rotation_array()
        matrix[:3, 3] = self.translation_array()
        return matrix

    def as_quaternion_xyzw(self) -> QuaternionXyzw:
        return quaternion_xyzw_from_rotation_matrix(self.rotation_matrix)

    def as_rodrigues_vector(self) -> Vector3:
        return rodrigues_from_rotation_matrix(self.rotation_matrix)

    def inverse(self) -> RigidTransform:
        rotation = self.rotation_array()
        inverse_rotation = rotation.T
        inverse_translation = -inverse_rotation @ self.translation_array()
        return RigidTransform(
            parent_frame=self.child_frame,
            child_frame=self.parent_frame,
            rotation_matrix=inverse_rotation,
            translation_m=inverse_translation,
        )

    def compose(self, child_transform: RigidTransform) -> RigidTransform:
        """parent_T_child @ child_T_descendant를 합성합니다."""

        if not isinstance(child_transform, RigidTransform):
            raise TypeError('child_transform must be a RigidTransform')
        if self.child_frame != child_transform.parent_frame:
            raise ValueError(
                'cannot compose disconnected transforms: '
                f'{self.parent_frame}_T_{self.child_frame} and '
                f'{child_transform.parent_frame}_T_'
                f'{child_transform.child_frame}'
            )
        rotation = self.rotation_array() @ child_transform.rotation_array()
        translation = (
            self.rotation_array() @ child_transform.translation_array()
            + self.translation_array()
        )
        return RigidTransform(
            parent_frame=self.parent_frame,
            child_frame=child_transform.child_frame,
            rotation_matrix=rotation,
            translation_m=translation,
        )

    def __matmul__(self, child_transform: RigidTransform) -> RigidTransform:
        return self.compose(child_transform)
