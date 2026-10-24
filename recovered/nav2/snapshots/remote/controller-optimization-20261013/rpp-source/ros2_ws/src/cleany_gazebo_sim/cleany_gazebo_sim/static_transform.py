from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def _finite_vector(
    values: object, expected_size: int, parameter_name: str
) -> tuple[float, ...]:
    if (
        not isinstance(values, Sequence)
        or isinstance(values, (str, bytes))
        or len(values) != expected_size
    ):
        raise ValueError(
            f'{parameter_name} must contain exactly {expected_size} values'
        )
    converted = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in converted):
        raise ValueError(f'{parameter_name} must contain only finite values')
    return converted


def _frame_id(value: object, parameter_name: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f'{parameter_name} must be a string')
    frame_id = value.strip()
    if not frame_id:
        raise ValueError(f'{parameter_name} must not be empty')
    if frame_id.startswith('/'):
        raise ValueError(f'{parameter_name} must not start with /')
    return frame_id


@dataclass(frozen=True)
class StaticTransformSpec:
    parent_frame_id: str
    child_frame_id: str
    translation: Vector3
    rotation_xyzw: Quaternion

    @classmethod
    def from_values(
        cls,
        parent_frame_id: str,
        child_frame_id: str,
        translation: Sequence[float],
        rotation_xyzw: Sequence[float],
    ) -> StaticTransformSpec:
        parent = _frame_id(parent_frame_id, 'parent_frame_id')
        child = _frame_id(child_frame_id, 'child_frame_id')
        if parent == child:
            raise ValueError('parent_frame_id and child_frame_id must differ')

        translation_values = _finite_vector(
            translation, 3, 'translation'
        )
        rotation_values = _finite_vector(
            rotation_xyzw, 4, 'rotation_xyzw'
        )
        quaternion_norm = math.sqrt(
            sum(value * value for value in rotation_values)
        )
        if not math.isclose(
            quaternion_norm, 1.0, rel_tol=1e-6, abs_tol=1e-6
        ):
            raise ValueError('rotation_xyzw must be a normalized quaternion')

        return cls(
            parent_frame_id=parent,
            child_frame_id=child,
            translation=(
                translation_values[0],
                translation_values[1],
                translation_values[2],
            ),
            rotation_xyzw=(
                rotation_values[0],
                rotation_values[1],
                rotation_values[2],
                rotation_values[3],
            ),
        )
