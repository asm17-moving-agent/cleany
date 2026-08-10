from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml


SCHEMA_VERSION = 'cleany.moveit_collision_objects/v1'


class CollisionSceneError(ValueError):
    """Raised when collision-object configuration is unsafe or ambiguous."""


@dataclass(frozen=True)
class PoseSpec:
    translation_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class PrimitiveSpec:
    type: str
    dimensions_m: tuple[float, float, float]
    pose: PoseSpec


@dataclass(frozen=True)
class CollisionObjectSpec:
    id: str
    primitives: tuple[PrimitiveSpec, ...]


@dataclass(frozen=True)
class CollisionSceneSpec:
    planning_frame: str
    apply_service: str
    objects: tuple[CollisionObjectSpec, ...]


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CollisionSceneError(f'{label} must be a mapping')
    return value


def _vector(value: Any, length: int, label: str) -> tuple[float, ...]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != length
        or any(isinstance(item, bool) for item in value)
    ):
        raise CollisionSceneError(f'{label} must contain {length} numbers')
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise CollisionSceneError(f'{label} must contain numbers') from error
    if not all(math.isfinite(item) for item in result):
        raise CollisionSceneError(f'{label} must contain finite numbers')
    return result


def load_collision_scene(path: Path) -> CollisionSceneSpec:
    try:
        raw = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise CollisionSceneError(f'cannot read {path}: {error}') from error
    data = _mapping(raw, 'collision scene')
    if data.get('schema_version') != SCHEMA_VERSION:
        raise CollisionSceneError(
            f'unsupported schema_version: {data.get("schema_version")!r}'
        )
    planning_frame = data.get('planning_frame')
    apply_service = data.get('apply_service')
    if not isinstance(planning_frame, str) or not planning_frame:
        raise CollisionSceneError('planning_frame must be a non-empty string')
    if not isinstance(apply_service, str) or not apply_service.startswith('/'):
        raise CollisionSceneError('apply_service must be an absolute ROS name')
    raw_objects = data.get('objects')
    if not isinstance(raw_objects, list) or not raw_objects:
        raise CollisionSceneError('objects must be a non-empty list')

    objects: list[CollisionObjectSpec] = []
    object_ids: set[str] = set()
    for object_index, raw_object in enumerate(raw_objects):
        object_data = _mapping(raw_object, f'objects[{object_index}]')
        object_id = object_data.get('id')
        if not isinstance(object_id, str) or not object_id:
            raise CollisionSceneError(
                f'objects[{object_index}].id must be non-empty'
            )
        if object_id in object_ids:
            raise CollisionSceneError(f'duplicate collision object: {object_id}')
        object_ids.add(object_id)
        raw_primitives = object_data.get('primitives')
        if not isinstance(raw_primitives, list) or not raw_primitives:
            raise CollisionSceneError(
                f'{object_id}.primitives must be a non-empty list'
            )
        primitives: list[PrimitiveSpec] = []
        for primitive_index, raw_primitive in enumerate(raw_primitives):
            primitive_data = _mapping(
                raw_primitive,
                f'{object_id}.primitives[{primitive_index}]',
            )
            primitive_type = primitive_data.get('type')
            if primitive_type != 'box':
                raise CollisionSceneError(
                    f'{object_id} supports only box primitives'
                )
            dimensions = _vector(
                primitive_data.get('dimensions_m'),
                3,
                f'{object_id}.dimensions_m',
            )
            if any(dimension <= 0.0 for dimension in dimensions):
                raise CollisionSceneError(
                    f'{object_id} dimensions must be positive'
                )
            pose_data = _mapping(
                primitive_data.get('pose'),
                f'{object_id}.pose',
            )
            translation = _vector(
                pose_data.get('translation_m'),
                3,
                f'{object_id}.translation_m',
            )
            quaternion = _vector(
                pose_data.get('quaternion_xyzw'),
                4,
                f'{object_id}.quaternion_xyzw',
            )
            quaternion_norm = math.sqrt(sum(value * value for value in quaternion))
            if not math.isclose(quaternion_norm, 1.0, abs_tol=1.0e-9):
                raise CollisionSceneError(
                    f'{object_id} quaternion must be normalized'
                )
            primitives.append(
                PrimitiveSpec(
                    type=primitive_type,
                    dimensions_m=(dimensions[0], dimensions[1], dimensions[2]),
                    pose=PoseSpec(
                        translation_m=(
                            translation[0],
                            translation[1],
                            translation[2],
                        ),
                        quaternion_xyzw=(
                            quaternion[0],
                            quaternion[1],
                            quaternion[2],
                            quaternion[3],
                        ),
                    ),
                )
            )
        objects.append(
            CollisionObjectSpec(id=object_id, primitives=tuple(primitives))
        )
    return CollisionSceneSpec(
        planning_frame=planning_frame,
        apply_service=apply_service,
        objects=tuple(objects),
    )


def build_planning_scene(spec: CollisionSceneSpec):
    """Build a MoveIt diff; every primitive pose is explicit in base_link."""

    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    for configured_object in spec.objects:
        collision_object = CollisionObject()
        collision_object.header.frame_id = spec.planning_frame
        collision_object.id = configured_object.id
        collision_object.operation = CollisionObject.ADD
        for configured_primitive in configured_object.primitives:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(configured_primitive.dimensions_m)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (
                configured_primitive.pose.translation_m
            )
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ) = configured_primitive.pose.quaternion_xyzw
            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)
        scene.world.collision_objects.append(collision_object)
    return scene
