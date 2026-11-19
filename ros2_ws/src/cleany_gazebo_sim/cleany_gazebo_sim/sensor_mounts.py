"""LiDAR mount profiles and sensor TF configuration shared by simulation launches."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping
import yaml
from cleany_gazebo_sim.static_transform import StaticTransformSpec

SCHEMA_VERSION = 1

@dataclass(frozen=True)
class LidarMountProfile:
    name: str
    description: str
    transform: StaticTransformSpec


def _require_mapping(value: object, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f'{context} must be a mapping')
    return value


def load_mount_profiles(path: Path) -> dict[str, LidarMountProfile]:
    raw = _require_mapping(
        yaml.safe_load(path.read_text(encoding='utf-8')), 'profile config'
    )
    if raw.get('schema_version') != SCHEMA_VERSION:
        raise ValueError(
            f'unsupported profile schema: {raw.get("schema_version")!r}'
        )
    parent = raw.get('parent_frame_id')
    child = raw.get('lidar_frame_id')
    entries = _require_mapping(raw.get('profiles'), 'profiles')
    if len(entries) < 3:
        raise ValueError('at least three LiDAR mount profiles are required')

    profiles: dict[str, LidarMountProfile] = {}
    transforms: set[tuple[float, ...]] = set()
    for name, untyped_values in entries.items():
        if (
            not isinstance(name, str)
            or not name
            or not name.replace('_', '').isalnum()
        ):
            raise ValueError(f'invalid profile name: {name!r}')
        values = _require_mapping(untyped_values, f'profile {name!r}')
        description = values.get('description')
        if not isinstance(description, str) or not description.strip():
            raise ValueError(f'profile {name!r} requires a description')
        transform = StaticTransformSpec.from_values(
            parent_frame_id=parent,
            child_frame_id=child,
            translation=values.get('translation', ()),
            rotation_xyzw=values.get('rotation_xyzw', ()),
        )
        signature = (*transform.translation, *transform.rotation_xyzw)
        if signature in transforms:
            raise ValueError(f'profile {name!r} duplicates another transform')
        transforms.add(signature)
        profiles[name] = LidarMountProfile(
            name, description.strip(), transform
        )
    return profiles


def write_sensor_tf_config(profile: LidarMountProfile, path: Path) -> None:
    parameters = {
        'gazebo_sensor_tf_publisher': {
            'ros__parameters': {
                'parent_frame_id': profile.transform.parent_frame_id,
                'lidar_frame_id': profile.transform.child_frame_id,
                'lidar_translation': list(profile.transform.translation),
                'lidar_rotation_xyzw': list(profile.transform.rotation_xyzw),
                'imu_frame_id': 'imu_link',
                'imu_translation': [0.0, 0.0, 0.0],
                'imu_rotation_xyzw': [0.0, 0.0, 0.0, 1.0],
            }
        }
    }
    path.write_text(
        yaml.safe_dump(parameters, sort_keys=False), encoding='utf-8'
    )
