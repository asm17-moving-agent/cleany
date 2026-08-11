from pathlib import Path
from xml.etree import ElementTree

import pytest
import yaml

from cleany_gazebo_sim.static_transform import StaticTransformSpec


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BASE_CONFIG_PATH = PACKAGE_ROOT / 'config' / 'base.yaml'
WORLD_PATHS = (
    PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_prototype.sdf',
    PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_harmonic.sdf',
)
LAUNCH_PATHS = (
    PACKAGE_ROOT / 'launch' / 'gazebo_sim.launch.py',
    PACKAGE_ROOT / 'launch' / 'gazebo_harmonic.launch.py',
)
SETUP_PATH = PACKAGE_ROOT / 'setup.py'


def test_static_transform_spec_accepts_sensor_mount():
    transform = StaticTransformSpec.from_values(
        parent_frame_id='base_link',
        child_frame_id='lidar_link',
        translation=(0.16, 0.0, -0.12),
        rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    assert transform.translation == (0.16, 0.0, -0.12)
    assert transform.rotation_xyzw == (0.0, 0.0, 0.0, 1.0)


@pytest.mark.parametrize(
    ('child_frame_id', 'translation', 'rotation_xyzw'),
    [
        ('base_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ('/imu_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ('imu_link', (0.0, float('nan'), 0.0), (0.0, 0.0, 0.0, 1.0)),
        ('imu_link', (0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ('imu_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
    ],
)
def test_static_transform_spec_rejects_invalid_values(
    child_frame_id, translation, rotation_xyzw
):
    with pytest.raises(ValueError):
        StaticTransformSpec.from_values(
            parent_frame_id='base_link',
            child_frame_id=child_frame_id,
            translation=translation,
            rotation_xyzw=rotation_xyzw,
        )


def test_sensor_tf_config_matches_both_gazebo_worlds():
    config = yaml.safe_load(BASE_CONFIG_PATH.read_text(encoding='utf-8'))
    parameters = config['gazebo_sensor_tf_publisher']['ros__parameters']

    assert parameters['parent_frame_id'] == 'base_link'
    assert parameters['lidar_frame_id'] == 'lidar_link'
    assert parameters['lidar_12cm_frame_id'] == 'lidar_12cm_link'
    assert parameters['lidar_45cm_frame_id'] == 'lidar_45cm_link'
    assert parameters['lidar_70cm_frame_id'] == 'lidar_70cm_link'
    assert parameters['imu_frame_id'] == 'imu_link'
    assert parameters['lidar_rotation_xyzw'] == [0.0, 0.0, 0.0, 1.0]
    assert parameters['imu_rotation_xyzw'] == [0.0, 0.0, 0.0, 1.0]

    for world_path in WORLD_PATHS:
        root = ElementTree.parse(world_path).getroot()
        model = root.find("./world/model[@name='cleany_mecanum']")
        assert model is not None
        for sensor_name in (
            'lidar_12cm', 'lidar', 'lidar_45cm', 'lidar_70cm', 'imu'
        ):
            mount = model.find(f"joint[@name='{sensor_name}_mount']")
            assert mount is not None
            assert mount.findtext('parent') == parameters['parent_frame_id']
            assert mount.findtext('child') == parameters[
                f'{sensor_name}_frame_id'
            ]
            pose_text = mount.findtext('pose')
            assert pose_text is not None
            pose = [float(value) for value in pose_text.split()]
            assert pose[:3] == parameters[f'{sensor_name}_translation']
            assert pose[3:] == [0.0, 0.0, 0.0]


def test_both_launch_profiles_publish_static_sensor_frames():
    for launch_path in LAUNCH_PATHS:
        launch = launch_path.read_text(encoding='utf-8')
        assert "executable='gazebo_sensor_tf_publisher'" in launch
        assert "name='gazebo_sensor_tf_publisher'" in launch

    setup = SETUP_PATH.read_text(encoding='utf-8')
    assert 'gazebo_sensor_tf_publisher = ' in setup
