from pathlib import Path
import pytest
import yaml
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch.substitutions import LaunchConfiguration
from cleany_gazebo_sim.sensor_mounts import load_mount_profiles, write_sensor_tf_config
from cleany_gazebo_sim.static_transform import StaticTransformSpec
from cleany_gazebo_sim.launch_helpers.robot_nodes import robot_runtime_nodes

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROFILES_PATH = PACKAGE_ROOT/'config/lidar_mount_profiles.yaml'

def test_profile_loader_rejects_duplicate_transforms(tmp_path: Path) -> None:
    config = yaml.safe_load(PROFILES_PATH.read_text(encoding='utf-8'))
    config['profiles']['floor_70cm']['translation'] = config['profiles'][
        'floor_26cm'
    ]['translation']
    path = tmp_path / 'profiles.yaml'
    path.write_text(yaml.safe_dump(config), encoding='utf-8')

    with pytest.raises(ValueError, match='duplicates'):
        load_mount_profiles(path)


def _launch_configurations(value) -> set[str]:
    """Names of every LaunchConfiguration reachable from node parameters."""
    if isinstance(value, LaunchConfiguration):
        return {''.join(part.text for part in value.variable_name)}
    if isinstance(value, dict):
        value = list(value.values())
    elif hasattr(value, 'param_file'):
        value = value.param_file
    if isinstance(value, (list, tuple)):
        return set().union(*map(_launch_configurations, value))
    return set()


def test_launch_profiles_accept_materialized_sensor_config() -> None:
    for launch_name in (
        'gazebo_fortress.launch.py',
        'gazebo_harmonic.launch.py',
    ):
        description = get_launch_description_from_python_launch_file(
            str(PACKAGE_ROOT / 'launch' / launch_name)
        )
        assert 'sensor_config' in {
            argument.name for argument in description.get_launch_arguments()
        }
    sensor_node = next(
        node for node in robot_runtime_nodes(PACKAGE_ROOT / 'config' / 'base.yaml')
        if node.node_executable == 'gazebo_sensor_tf_publisher'
    )
    assert 'sensor_config' in _launch_configurations(
        sensor_node._Node__parameters
    )


def test_materialized_mount_retains_transform_and_frames(tmp_path):
    for name, profile in load_mount_profiles(PROFILES_PATH).items():
        output = tmp_path/(name+'.yaml')
        write_sensor_tf_config(profile, output)
        params = yaml.safe_load(output.read_text())['gazebo_sensor_tf_publisher']['ros__parameters']
        restored = StaticTransformSpec.from_values(
            parent_frame_id=params['parent_frame_id'], child_frame_id=params['lidar_frame_id'],
            translation=params['lidar_translation'], rotation_xyzw=params['lidar_rotation_xyzw'])
        assert restored == profile.transform
