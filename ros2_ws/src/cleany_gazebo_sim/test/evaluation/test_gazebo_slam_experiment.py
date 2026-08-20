from __future__ import annotations

import json
from pathlib import Path
from xml.etree import ElementTree

import pytest
import yaml

from cleany_gazebo_sim.gazebo_slam_experiment import (
    load_mount_profiles,
    materialize_evaluation,
    record_result,
    validate_result,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
PROFILES_PATH = PACKAGE_ROOT / 'config' / 'lidar_mount_profiles.yaml'
BRIDGE_CONFIG_ROOT = PACKAGE_ROOT / 'config' / 'bridge'


def test_mount_profiles_define_distinct_candidate_transforms() -> None:
    profiles = load_mount_profiles(PROFILES_PATH)

    assert set(profiles) == {
        'floor_16p5cm', 'floor_26cm', 'floor_45cm', 'floor_70cm'
    }
    assert profiles['floor_16p5cm'].transform.translation == (
        0.16, 0.0, -0.215
    )
    assert profiles['floor_26cm'].transform.translation == (
        0.16, 0.0, -0.12
    )
    assert profiles['floor_45cm'].transform.translation == (
        0.16, 0.0, 0.07
    )
    assert profiles['floor_70cm'].transform.translation == (
        0.16, 0.0, 0.32
    )
    translations = {
        profile.transform.translation for profile in profiles.values()
    }
    assert len(translations) == 4
    assert all(
        profile.transform.parent_frame_id == 'base_link'
        and profile.transform.child_frame_id == 'lidar_link'
        for profile in profiles.values()
    )


def test_shared_lidar_bridge_has_one_stable_scan_topic() -> None:
    entries = yaml.safe_load(
        (BRIDGE_CONFIG_ROOT / 'lidar_bridge_harmonic.yaml').read_text(
            encoding='utf-8'
        )
    )
    assert len(entries) == 1
    assert entries[0]['ros_topic_name'] == '/scan'
    assert entries[0]['gz_topic_name'] == '/model/cleany_mecanum/lidar/scan'


def test_profile_loader_rejects_duplicate_transforms(tmp_path: Path) -> None:
    config = yaml.safe_load(PROFILES_PATH.read_text(encoding='utf-8'))
    config['profiles']['floor_70cm']['translation'] = config['profiles'][
        'floor_26cm'
    ]['translation']
    path = tmp_path / 'profiles.yaml'
    path.write_text(yaml.safe_dump(config), encoding='utf-8')

    with pytest.raises(ValueError, match='duplicates'):
        load_mount_profiles(path)


@pytest.mark.parametrize('simulator', ('fortress', 'harmonic'))
@pytest.mark.parametrize(
    'profile_name',
    (
        'floor_16p5cm',
        'floor_26cm',
        'floor_45cm',
        'floor_70cm',
    ),
)
def test_materialized_world_and_tf_match_profile(
    tmp_path: Path, simulator: str, profile_name: str
) -> None:
    output_dir = tmp_path / f'{simulator}-{profile_name}'
    artifacts = materialize_evaluation(
        package_root=PACKAGE_ROOT,
        profiles_path=PROFILES_PATH,
        profile_name=profile_name,
        simulator=simulator,
        output_dir=output_dir,
    )
    profile = load_mount_profiles(PROFILES_PATH)[profile_name]

    root = ElementTree.parse(artifacts.world_path).getroot()
    mount = root.find(
        "./world/model[@name='cleany_mecanum']/joint[@name='lidar_mount']"
    )
    assert mount is not None
    assert mount.findtext('parent') == profile.transform.parent_frame_id
    assert mount.findtext('child') == profile.transform.child_frame_id
    pose = [float(value) for value in mount.findtext('pose', '').split()]
    assert tuple(pose[:3]) == profile.transform.translation
    assert pose[3:] == [0.0, 0.0, 0.0]

    tf_config = yaml.safe_load(
        artifacts.sensor_config_path.read_text(encoding='utf-8')
    )['gazebo_sensor_tf_publisher']['ros__parameters']
    assert (
        tuple(tf_config['lidar_translation'])
        == profile.transform.translation
    )
    assert tuple(tf_config['lidar_rotation_xyzw']) == (
        profile.transform.rotation_xyzw
    )
    assert tf_config['imu_frame_id'] == 'imu_link'

    manifest = json.loads(artifacts.manifest_path.read_text(encoding='utf-8'))
    template = json.loads(
        artifacts.result_template_path.read_text(encoding='utf-8')
    )
    assert manifest['profile'] == profile_name
    assert manifest['simulator'] == simulator
    assert template['status'] == 'not_run'
    assert all(value is None for value in template['metrics'].values())


def _completed_result() -> dict[str, object]:
    return {
        'schema_version': 1,
        'status': 'completed',
        'profile': 'floor_26cm',
        'simulator': 'fortress',
        'route_id': 'office-loop-v1',
        'trial_index': 0,
        'duration_sec': 120.0,
        'metrics': {
            'ate_rmse_m': 0.12,
            'rpe_translation_rmse_m': 0.03,
            'rpe_rotation_rmse_rad': 0.02,
            'map_coverage_ratio': 0.91,
            'valid_scan_ratio': 0.98,
            'mean_scan_rate_hz': 5.5,
            'real_time_factor': 0.95,
        },
        'qualitative': {
            'map_artifact': 'map.pgm',
            'trajectory_artifact': 'trajectory.csv',
            'observations': ['minor occlusion behind the chassis'],
        },
    }


def test_result_validation_and_recording(tmp_path: Path) -> None:
    run_dir = tmp_path / 'run'
    artifacts = materialize_evaluation(
        package_root=PACKAGE_ROOT,
        profiles_path=PROFILES_PATH,
        profile_name='floor_26cm',
        simulator='fortress',
        output_dir=run_dir,
    )
    input_path = tmp_path / 'measured.json'
    input_path.write_text(json.dumps(_completed_result()), encoding='utf-8')

    output_path = record_result(run_dir, input_path)

    assert output_path == artifacts.result_template_path
    assert json.loads(output_path.read_text(encoding='utf-8'))[
        'status'
    ] == 'completed'


@pytest.mark.parametrize(
    ('field', 'value', 'message'),
    [
        ('profile', 'floor_70cm', 'does not match'),
        ('duration_sec', 0.0, 'greater than zero'),
    ],
)
def test_result_validation_rejects_incomparable_runs(
    field: str, value: object, message: str
) -> None:
    result = _completed_result()
    result[field] = value

    with pytest.raises(ValueError, match=message):
        validate_result(
            result,
            {
                'schema_version': 1,
                'profile': 'floor_26cm',
                'simulator': 'fortress',
            },
        )


def test_result_validation_rejects_invalid_ratio() -> None:
    result = _completed_result()
    result['metrics']['valid_scan_ratio'] = 1.01  # type: ignore[index]

    with pytest.raises(ValueError, match='between zero and one'):
        validate_result(
            result,
            {
                'schema_version': 1,
                'profile': 'floor_26cm',
                'simulator': 'fortress',
            },
        )


def test_launch_profiles_accept_materialized_sensor_config() -> None:
    for launch_name in (
        'gazebo_fortress.launch.py',
        'gazebo_harmonic.launch.py',
    ):
        launch = (PACKAGE_ROOT / 'launch' / launch_name).read_text(
            encoding='utf-8'
        )
        assert "DeclareLaunchArgument(\n        'sensor_config'" in launch
        sensor_node = launch.split(
            "executable='gazebo_sensor_tf_publisher'", 1
        )[1]
        assert "LaunchConfiguration('sensor_config')" in sensor_node
