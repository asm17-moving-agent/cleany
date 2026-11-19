"""Small URDF chain evaluator used by offline noise experiments."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Mapping, Protocol, Sequence
import xml.etree.ElementTree as ET

import numpy as np

from cleany_handeye_calibration.transforms import RigidTransform


class FeedbackFkPort(Protocol):
    """Compute feedback FK after a caller has perturbed joint positions."""

    def compute(
        self,
        joint_names: Sequence[str],
        positions_rad: Sequence[float],
    ) -> RigidTransform: ...


def _vector(
    text: str | None,
    *,
    default: tuple[float, float, float],
    field_name: str,
) -> np.ndarray:
    if text is None:
        return np.asarray(default, dtype=np.float64)
    try:
        values = np.asarray(
            tuple(float(value) for value in text.split()),
            dtype=np.float64,
        )
    except (TypeError, ValueError) as error:
        raise ValueError(f'{field_name} must contain three numbers') from error
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError(f'{field_name} must contain three finite numbers')
    return values


def _rotation_x(angle: float) -> np.ndarray:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        ((1.0, 0.0, 0.0), (0.0, cosine, -sine), (0.0, sine, cosine)),
        dtype=np.float64,
    )


def _rotation_y(angle: float) -> np.ndarray:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        ((cosine, 0.0, sine), (0.0, 1.0, 0.0), (-sine, 0.0, cosine)),
        dtype=np.float64,
    )


def _rotation_z(angle: float) -> np.ndarray:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        ((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)),
        dtype=np.float64,
    )


def _origin_matrix(element: ET.Element | None, joint_name: str) -> np.ndarray:
    if element is None:
        xyz = np.zeros(3, dtype=np.float64)
        rpy = np.zeros(3, dtype=np.float64)
    else:
        xyz = _vector(
            element.attrib.get('xyz'),
            default=(0.0, 0.0, 0.0),
            field_name=f'{joint_name} origin xyz',
        )
        rpy = _vector(
            element.attrib.get('rpy'),
            default=(0.0, 0.0, 0.0),
            field_name=f'{joint_name} origin rpy',
        )
    result = np.eye(4, dtype=np.float64)
    # URDF fixed-axis roll, pitch, yaw: Rz(yaw) Ry(pitch) Rx(roll).
    result[:3, :3] = (
        _rotation_z(float(rpy[2]))
        @ _rotation_y(float(rpy[1]))
        @ _rotation_x(float(rpy[0]))
    )
    result[:3, 3] = xyz
    return result


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError('URDF joint axis must be finite and nonzero')
    x, y, z = axis / norm
    skew = np.array(
        ((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)),
        dtype=np.float64,
    )
    rotation = (
        np.eye(3, dtype=np.float64)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    return result


def _axis_translation(axis: np.ndarray, distance: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError('URDF joint axis must be finite and nonzero')
    result = np.eye(4, dtype=np.float64)
    result[:3, 3] = axis / norm * distance
    return result


@dataclass(frozen=True, slots=True)
class _UrdfJoint:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    origin: np.ndarray
    axis: np.ndarray | None


class UrdfOfflineFk:
    """Evaluate one serial URDF chain without ROS or planned-state inputs."""

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        base_frame: str = 'base_link',
        tip_frame: str = 'left_gripper_frame',
    ) -> None:
        path = Path(urdf_path).expanduser().resolve(strict=True)
        if not path.is_file():
            raise ValueError('urdf_path must name a regular file')
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as error:
            raise ValueError(f'invalid URDF XML: {error}') from error
        if root.tag != 'robot':
            raise ValueError('URDF root element must be robot')
        if not isinstance(base_frame, str) or not base_frame:
            raise ValueError('base_frame is required')
        if not isinstance(tip_frame, str) or not tip_frame:
            raise ValueError('tip_frame is required')
        joints_by_child: dict[str, _UrdfJoint] = {}
        for element in root.findall('joint'):
            name = element.attrib.get('name', '')
            joint_type = element.attrib.get('type', '')
            parent = element.find('parent')
            child = element.find('child')
            if not name or parent is None or child is None:
                raise ValueError('every URDF joint requires name/parent/child')
            parent_link = parent.attrib.get('link', '')
            child_link = child.attrib.get('link', '')
            if not parent_link or not child_link:
                raise ValueError(f'{name} has an empty parent or child link')
            if child_link in joints_by_child:
                raise ValueError(
                    f'URDF has multiple parents for {child_link}'
                )
            axis = None
            if joint_type in {'revolute', 'continuous', 'prismatic'}:
                axis_element = element.find('axis')
                axis = _vector(
                    (
                        None
                        if axis_element is None
                        else axis_element.attrib.get('xyz')
                    ),
                    default=(1.0, 0.0, 0.0),
                    field_name=f'{name} axis',
                )
            elif joint_type != 'fixed':
                raise ValueError(f'{name} has unsupported type {joint_type!r}')
            joints_by_child[child_link] = _UrdfJoint(
                name=name,
                joint_type=joint_type,
                parent_link=parent_link,
                child_link=child_link,
                origin=_origin_matrix(element.find('origin'), name),
                axis=axis,
            )
        chain_reversed: list[_UrdfJoint] = []
        link = tip_frame
        seen = {link}
        while link != base_frame:
            joint = joints_by_child.get(link)
            if joint is None:
                raise ValueError(
                    f'no URDF chain from {base_frame} to {tip_frame}'
                )
            chain_reversed.append(joint)
            link = joint.parent_link
            if link in seen:
                raise ValueError('URDF kinematic chain contains a cycle')
            seen.add(link)
        self._path = path
        self._base_frame = base_frame
        self._tip_frame = tip_frame
        self._chain = tuple(reversed(chain_reversed))
        self._moving_names = tuple(
            joint.name
            for joint in self._chain
            if joint.joint_type != 'fixed'
        )

    @property
    def moving_joint_names(self) -> tuple[str, ...]:
        return self._moving_names

    def compute(
        self,
        joint_names: Sequence[str],
        positions_rad: Sequence[float],
    ) -> RigidTransform:
        names = tuple(joint_names)
        try:
            positions = tuple(float(value) for value in positions_rad)
        except (TypeError, ValueError) as error:
            raise ValueError('positions_rad must be numeric') from error
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError(
                'joint names/positions must be unique and aligned'
            )
        if not all(math.isfinite(value) for value in positions):
            raise ValueError('positions_rad must be finite')
        values: Mapping[str, float] = dict(
            zip(names, positions, strict=True)
        )
        missing = tuple(
            name for name in self._moving_names if name not in values
        )
        if missing:
            raise ValueError(f'feedback state is missing FK joints: {missing}')

        matrix = np.eye(4, dtype=np.float64)
        for joint in self._chain:
            matrix = matrix @ joint.origin
            if joint.joint_type == 'fixed':
                continue
            assert joint.axis is not None
            position = values[joint.name]
            if joint.joint_type in {'revolute', 'continuous'}:
                matrix = matrix @ _axis_rotation(joint.axis, position)
            else:
                matrix = matrix @ _axis_translation(joint.axis, position)
        return RigidTransform.from_homogeneous_matrix(
            parent_frame=self._base_frame,
            child_frame=self._tip_frame,
            matrix=matrix,
        )


__all__ = ['FeedbackFkPort', 'UrdfOfflineFk']
