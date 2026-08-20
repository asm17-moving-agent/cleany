from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
from math import isfinite
from pathlib import Path
import shutil
from typing import Any, Mapping, Sequence
from xml.etree import ElementTree

import yaml

from cleany_gazebo_sim.static_transform import StaticTransformSpec
from cleany_gazebo_sim.world.generator import materialize_mecanum_wheel_world


SCHEMA_VERSION = 1
_WORLD_FILENAMES = {
    'fortress': 'cleany_mecanum_fortress.sdf',
    'harmonic': 'cleany_mecanum_harmonic.sdf',
}
_REQUIRED_METRICS = (
    'ate_rmse_m',
    'rpe_translation_rmse_m',
    'rpe_rotation_rmse_rad',
    'map_coverage_ratio',
    'valid_scan_ratio',
    'mean_scan_rate_hz',
    'real_time_factor',
)


@dataclass(frozen=True)
class LidarMountProfile:
    name: str
    description: str
    transform: StaticTransformSpec


@dataclass(frozen=True)
class EvaluationArtifacts:
    profile: str
    simulator: str
    world_path: Path
    sensor_config_path: Path
    result_template_path: Path
    manifest_path: Path


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


def _write_profile_world(
    template_path: Path, profile: LidarMountProfile, path: Path
) -> None:
    expanded_path = materialize_mecanum_wheel_world(template_path)
    root = ElementTree.parse(expanded_path).getroot()
    model = root.find("./world/model[@name='cleany_mecanum']")
    if model is None:
        raise ValueError('world is missing the cleany_mecanum model')
    mount = model.find("joint[@name='lidar_mount']")
    if mount is None or mount.find('pose') is None:
        raise ValueError('world is missing the lidar_mount pose')
    if mount.findtext('parent') != profile.transform.parent_frame_id:
        raise ValueError('world lidar parent does not match the profile')
    if mount.findtext('child') != profile.transform.child_frame_id:
        raise ValueError('world lidar child does not match the profile')

    translation = profile.transform.translation
    # The candidate profiles currently remain level. StaticTransformSpec
    # validates the quaternion, while arbitrary quaternion-to-RPY conversion is
    # intentionally deferred until a tilted candidate is required.
    if profile.transform.rotation_xyzw != (0.0, 0.0, 0.0, 1.0):
        raise ValueError(
            'world materialization currently supports level mounts only'
        )
    pose = mount.find('pose')
    assert pose is not None
    pose.text = ' '.join(
        str(value) for value in (*translation, 0, 0, 0)
    )
    ElementTree.ElementTree(root).write(
        path, encoding='unicode', xml_declaration=True
    )


def _result_template(
    profile: LidarMountProfile, simulator: str
) -> dict[str, Any]:
    return {
        'schema_version': SCHEMA_VERSION,
        'status': 'not_run',
        'profile': profile.name,
        'simulator': simulator,
        'route_id': None,
        'trial_index': None,
        'duration_sec': None,
        'metrics': {metric: None for metric in _REQUIRED_METRICS},
        'qualitative': {
            'map_artifact': None,
            'trajectory_artifact': None,
            'observations': [],
        },
    }


def materialize_evaluation(
    *,
    package_root: Path,
    profiles_path: Path,
    profile_name: str,
    simulator: str,
    output_dir: Path,
) -> EvaluationArtifacts:
    if simulator not in _WORLD_FILENAMES:
        raise ValueError(f'unsupported simulator profile: {simulator!r}')
    profiles = load_mount_profiles(profiles_path)
    try:
        profile = profiles[profile_name]
    except KeyError as error:
        raise ValueError(
            f'unknown LiDAR mount profile: {profile_name!r}'
        ) from error

    output_dir.mkdir(parents=True, exist_ok=False)
    world_path = output_dir / 'world.sdf'
    sensor_config_path = output_dir / 'sensor_tf.yaml'
    result_template_path = output_dir / 'result.json'
    manifest_path = output_dir / 'manifest.json'
    try:
        _write_profile_world(
            package_root / 'worlds' / _WORLD_FILENAMES[simulator],
            profile,
            world_path,
        )
        write_sensor_tf_config(profile, sensor_config_path)
        result_template_path.write_text(
            json.dumps(_result_template(profile, simulator), indent=2) + '\n',
            encoding='utf-8',
        )
        manifest = {
            'schema_version': SCHEMA_VERSION,
            'created_at': datetime.now(timezone.utc).isoformat(),
            'profile': profile.name,
            'description': profile.description,
            'simulator': simulator,
            'transform': asdict(profile.transform),
            'artifacts': {
                'world': world_path.name,
                'sensor_config': sensor_config_path.name,
                'result': result_template_path.name,
            },
        }
        manifest_path.write_text(
            json.dumps(manifest, indent=2) + '\n', encoding='utf-8'
        )
    except Exception:
        shutil.rmtree(output_dir)
        raise
    return EvaluationArtifacts(
        profile.name,
        simulator,
        world_path,
        sensor_config_path,
        result_template_path,
        manifest_path,
    )


def validate_result(
    result: Mapping[str, Any], manifest: Mapping[str, Any]
) -> None:
    if result.get('schema_version') != SCHEMA_VERSION:
        raise ValueError('result schema_version is invalid')
    if result.get('status') not in ('completed', 'failed', 'inconclusive'):
        raise ValueError(
            'result status must be completed, failed, or inconclusive'
        )
    for field in ('profile', 'simulator'):
        if result.get(field) != manifest.get(field):
            raise ValueError(f'result {field} does not match the run manifest')
    if not isinstance(result.get('route_id'), str) or not result['route_id']:
        raise ValueError('route_id must be a non-empty string')
    trial_index = result.get('trial_index')
    if (
        not isinstance(trial_index, int)
        or isinstance(trial_index, bool)
        or trial_index < 0
    ):
        raise ValueError('trial_index must be a non-negative integer')
    duration = result.get('duration_sec')
    if (
        not isinstance(duration, (int, float))
        or not isfinite(duration)
        or duration <= 0
    ):
        raise ValueError('duration_sec must be finite and greater than zero')
    metrics = _require_mapping(result.get('metrics'), 'metrics')
    for name in _REQUIRED_METRICS:
        value = metrics.get(name)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ValueError(f'metric {name} must be numeric')
        if not isfinite(value) or value < 0:
            raise ValueError(f'metric {name} must be finite and non-negative')
    for ratio in ('map_coverage_ratio', 'valid_scan_ratio'):
        if metrics[ratio] > 1.0:
            raise ValueError(f'metric {ratio} must be between zero and one')
    qualitative = _require_mapping(result.get('qualitative'), 'qualitative')
    observations = qualitative.get('observations')
    if not isinstance(observations, list) or not all(
        isinstance(value, str) for value in observations
    ):
        raise ValueError('qualitative observations must be a list of strings')


def record_result(run_dir: Path, input_path: Path) -> Path:
    manifest = _require_mapping(
        json.loads((run_dir / 'manifest.json').read_text(encoding='utf-8')),
        'manifest',
    )
    result = _require_mapping(
        json.loads(input_path.read_text(encoding='utf-8')), 'result'
    )
    validate_result(result, manifest)
    output_path = run_dir / 'result.json'
    output_path.write_text(
        json.dumps(result, indent=2) + '\n', encoding='utf-8'
    )
    return output_path


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Prepare and record Gazebo LiDAR SLAM experiments.'
    )
    subparsers = parser.add_subparsers(dest='command', required=True)
    prepare = subparsers.add_parser('prepare')
    prepare.add_argument('--package-root', type=Path, required=True)
    prepare.add_argument('--profiles', type=Path, required=True)
    prepare.add_argument('--profile', required=True)
    prepare.add_argument(
        '--simulator', choices=tuple(_WORLD_FILENAMES), default='fortress'
    )
    prepare.add_argument('--output', type=Path, required=True)
    record = subparsers.add_parser('record')
    record.add_argument('--run-dir', type=Path, required=True)
    record.add_argument('--input', type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    arguments = _parser().parse_args(argv)
    if arguments.command == 'prepare':
        artifacts = materialize_evaluation(
            package_root=arguments.package_root,
            profiles_path=arguments.profiles,
            profile_name=arguments.profile,
            simulator=arguments.simulator,
            output_dir=arguments.output,
        )
        print(artifacts.manifest_path)
    else:
        print(record_result(arguments.run_dir, arguments.input))


if __name__ == '__main__':
    main()
