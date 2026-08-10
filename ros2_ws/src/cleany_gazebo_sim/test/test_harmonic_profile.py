from pathlib import Path
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_harmonic.sdf'
FORTRESS_WORLD_PATH = PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_prototype.sdf'
BRIDGE_PATH = PACKAGE_ROOT / 'config' / 'bridge_harmonic.yaml'
LIDAR_BRIDGE_PATH = PACKAGE_ROOT / 'config' / 'lidar_bridge_harmonic.yaml'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'gazebo_harmonic.launch.py'


def test_harmonic_world_uses_gz_sim_and_ogre2():
    root = ElementTree.parse(WORLD_PATH).getroot()
    plugins = root.findall('.//plugin')
    filenames = {plugin.attrib['filename'] for plugin in plugins}
    names = {plugin.attrib['name'] for plugin in plugins}

    assert 'gz-sim-sensors-system' in filenames
    assert 'gz-sim-imu-system' in filenames
    assert 'gz-sim-mecanum-drive-system' in filenames
    assert 'gz::sim::systems::Sensors' in names
    assert 'gz::sim::systems::Imu' in names
    assert all(not filename.startswith('ignition-gazebo-') for filename in filenames)

    sensor_system = root.find(
        ".//plugin[@name='gz::sim::systems::Sensors']"
    )
    assert sensor_system is not None
    assert sensor_system.findtext('render_engine') == 'ogre2'


def test_harmonic_world_preserves_fortress_robot_structure():
    fortress_root = ElementTree.parse(FORTRESS_WORLD_PATH).getroot()
    harmonic_root = ElementTree.parse(WORLD_PATH).getroot()
    fortress_model = fortress_root.find("./world/model[@name='cleany_mecanum']")
    harmonic_model = harmonic_root.find("./world/model[@name='cleany_mecanum']")
    assert fortress_model is not None
    assert harmonic_model is not None

    fortress_links = {link.attrib['name'] for link in fortress_model.findall('link')}
    harmonic_links = {link.attrib['name'] for link in harmonic_model.findall('link')}
    fortress_joints = {joint.attrib['name'] for joint in fortress_model.findall('joint')}
    harmonic_joints = {joint.attrib['name'] for joint in harmonic_model.findall('joint')}

    assert harmonic_links == fortress_links
    assert harmonic_joints == fortress_joints
    assert {
        uri.text for uri in fortress_model.findall('.//mesh/uri')
    } == {
        uri.text for uri in harmonic_model.findall('.//mesh/uri')
    }


def test_harmonic_rendering_sensors_are_lazy():
    root = ElementTree.parse(WORLD_PATH).getroot()
    rendering_sensors = [
        sensor
        for sensor in root.findall('.//sensor')
        if sensor.attrib['type'] in {'camera', 'depth_camera', 'gpu_lidar'}
    ]

    assert {sensor.attrib['name'] for sensor in rendering_sensors} == {
        'head_realsense_rgb',
        'head_realsense_depth',
        'left_wrist_rgb',
        'right_wrist_rgb',
        'rplidar_a1',
    }
    assert all(sensor.findtext('always_on') != 'true' for sensor in rendering_sensors)


def test_harmonic_world_contains_rplidar_candidate():
    root = ElementTree.parse(WORLD_PATH).getroot()
    model = root.find("./world/model[@name='cleany_mecanum']")
    assert model is not None

    mount = model.find("joint[@name='lidar_mount']")
    sensor = model.find("link[@name='lidar_link']/sensor[@name='rplidar_a1']")
    assert mount is not None
    assert mount.findtext('parent') == 'base_link'
    assert mount.findtext('pose') == '0.32 0 -0.18 0 0 0'
    assert sensor is not None
    assert sensor.attrib['type'] == 'gpu_lidar'
    assert sensor.findtext('topic') == '/model/cleany_mecanum/lidar/scan'
    assert sensor.findtext('update_rate') == '5.5'
    assert sensor.findtext('lidar/scan/horizontal/samples') == '360'
    assert sensor.findtext('lidar/range/min') == '0.15'
    assert sensor.findtext('lidar/range/max') == '12.0'


def test_harmonic_world_contains_base_imu():
    root = ElementTree.parse(WORLD_PATH).getroot()
    model = root.find("./world/model[@name='cleany_mecanum']")
    assert model is not None

    mount = model.find("joint[@name='imu_mount']")
    sensor = model.find("link[@name='imu_link']/sensor[@name='base_imu']")
    assert mount is not None
    assert mount.findtext('parent') == 'base_link'
    assert mount.findtext('child') == 'imu_link'
    assert mount.findtext('pose') == '0 0 0 0 0 0'
    assert sensor is not None
    assert sensor.attrib['type'] == 'imu'
    assert sensor.findtext('topic') == '/model/cleany_mecanum/imu'
    assert sensor.findtext('always_on') == 'true'
    assert sensor.findtext('update_rate') == '50'
    assert sensor.findtext('gz_frame_id') == 'imu_link'
    assert sensor.find('imu') is not None
    assert sensor.find('.//noise') is None


def test_harmonic_launch_and_bridges_are_version_isolated():
    launch = LAUNCH_PATH.read_text(encoding='utf-8')
    bridge = BRIDGE_PATH.read_text(encoding='utf-8')
    lidar_bridge = LIDAR_BRIDGE_PATH.read_text(encoding='utf-8')

    assert "'cleany_mecanum_harmonic.sdf'" in launch
    assert "'bridge_harmonic.yaml'" in launch
    assert "'GZ_SIM_RESOURCE_PATH'" in launch
    assert "'--render-engine-server'" in launch
    assert "'--render-engine-gui'" in launch
    assert 'gz.msgs.' in bridge
    assert 'ignition.msgs.' not in bridge
    assert 'gz.msgs.LaserScan' in lidar_bridge
    assert 'ros_topic_name: "/imu/data"' in bridge
    assert 'gz_topic_name: "/model/cleany_mecanum/imu"' in bridge
    assert 'ros_type_name: "sensor_msgs/msg/Imu"' in bridge
    assert 'gz_type_name: "gz.msgs.IMU"' in bridge
