from pathlib import Path
from xml.etree import ElementTree

from cleany_gazebo_sim.world.generator import materialize_mecanum_wheel_world


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_fortress.sdf'
FILENAME_PREFIX = 'ignition-gazebo-'
NAME_PREFIX = 'ignition::gazebo::systems::'
WHEEL_JOINTS = {
    'front_left_joint': 'front_left_wheel_joint',
    'front_right_joint': 'front_right_wheel_joint',
    'back_left_joint': 'rear_left_wheel_joint',
    'back_right_joint': 'rear_right_wheel_joint',
}


def _model() -> ElementTree.Element:
    root = ElementTree.parse(WORLD_PATH).getroot()
    model = root.find("./world/model[@name='cleany_mecanum']")
    assert model is not None
    return model


def test_world_has_required_systems_and_robot_contract() -> None:
    root = ElementTree.parse(WORLD_PATH).getroot()
    model = _model()
    plugins = root.findall('.//plugin')
    filenames = {plugin.get('filename', '') for plugin in plugins}
    names = {plugin.get('name', '') for plugin in plugins}

    assert any(name == f'{NAME_PREFIX}Sensors' for name in names)
    assert any(name == f'{NAME_PREFIX}Imu' for name in names)
    assert any(name == f'{NAME_PREFIX}MecanumDrive' for name in names)
    assert any(name == f'{NAME_PREFIX}OdometryPublisher' for name in names)
    assert all(
        filename.startswith(FILENAME_PREFIX)
        for filename in filenames
        if filename.startswith(FILENAME_PREFIX)
    )

    drive = model.find(f"plugin[@name='{NAME_PREFIX}MecanumDrive']")
    ground_truth = model.find(
        f"plugin[@name='{NAME_PREFIX}OdometryPublisher']"
    )
    assert drive is not None
    assert ground_truth is not None
    for element, joint_name in WHEEL_JOINTS.items():
        assert drive.findtext(element) == joint_name
        assert model.find(f"joint[@name='{joint_name}']") is not None
    assert drive.find('odom_topic') is None
    assert ground_truth.findtext('odom_topic') == (
        '/model/cleany_mecanum/ground_truth'
    )
    assert ground_truth.find('odom_covariance_topic') is None


def test_world_exposes_lidar_imu_and_camera_sensor_contract() -> None:
    model = _model()
    lidar = model.find("link[@name='lidar_link']/sensor[@name='rplidar_a1']")
    imu = model.find("link[@name='imu_link']/sensor[@name='base_imu']")
    assert lidar is not None
    assert lidar.get('type') == 'gpu_lidar'
    assert lidar.findtext('topic') == '/model/cleany_mecanum/lidar/scan'
    assert lidar.findtext('lidar/range/min') == '0.15'
    assert lidar.findtext('lidar/range/max') == '12.0'
    assert lidar.findtext('lidar/scan/horizontal/samples') == '360'
    assert imu is not None
    assert imu.get('type') == 'imu'
    assert imu.findtext('topic') == '/model/cleany_mecanum/imu'
    assert imu.findtext('gz_frame_id') == 'imu_link'

    camera_frames = {
        sensor.get('name'): sensor.findtext('gz_frame_id')
        for sensor in model.findall('.//sensor')
        if sensor.get('type') in {'camera', 'depth_camera'}
    }
    assert camera_frames == {
        'head_realsense_rgb': 'head_camera_rgb_optical_frame',
        'head_realsense_depth': 'head_camera_depth_optical_frame',
        'left_wrist_rgb': 'left_wrist_rgb_optical_frame',
        'right_wrist_rgb': 'right_wrist_rgb_optical_frame',
    }


def test_materialized_world_removes_arm_controllers_and_keeps_wheels(
) -> None:
    generated = materialize_mecanum_wheel_world(WORLD_PATH)
    root = ElementTree.parse(generated).getroot()
    model = root.find("./world/model[@name='cleany_mecanum']")
    assert model is not None
    assert not any(
        plugin.get('name', '').endswith('JointPositionController')
        for plugin in model.findall('plugin')
    )
    assert all(
        model.find(f"joint[@name='{joint_name}']") is not None
        for joint_name in WHEEL_JOINTS.values()
    )
    roller_visuals = [
        visual
        for visual in model.findall('link/visual')
        if '_roller_' in visual.get('name', '')
    ]
    assert len(roller_visuals) == 48
