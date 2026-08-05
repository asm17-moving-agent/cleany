from pathlib import Path
from xml.etree import ElementTree

from cleany_gazebo_sim.world_generator import materialize_mecanum_wheel_world

WORLD_PATH = (
    Path(__file__).resolve().parents[1] / 'worlds' / 'cleany_mecanum_prototype.sdf'
)
BRIDGE_PATH = Path(__file__).resolve().parents[1] / 'config' / 'bridge.yaml'
LIDAR_BRIDGE_PATH = (
    Path(__file__).resolve().parents[1] / 'config' / 'lidar_bridge.yaml'
)
DESCRIPTION_SHARE = Path(__file__).resolve().parents[2] / 'cleany_description'


def test_world_contains_dual_arm_joint_skeleton():
    root = ElementTree.parse(WORLD_PATH).getroot()
    model = root.find("./world/model[@name='cleany_mecanum']")
    assert model is not None

    links = {link.attrib['name'] for link in model.findall('link')}
    joints = {joint.attrib['name'] for joint in model.findall('joint')}

    assert {
        'left_arm_base',
        'left_moving_jaw',
        'right_arm_base',
        'right_moving_jaw',
    } <= links
    assert {
        'left_shoulder_yaw_joint',
        'left_shoulder_pitch_joint',
        'left_elbow_pitch_joint',
        'left_wrist_pitch_joint',
        'left_wrist_roll_joint',
        'left_gripper_joint',
        'right_shoulder_yaw_joint',
        'right_shoulder_pitch_joint',
        'right_elbow_pitch_joint',
        'right_wrist_pitch_joint',
        'right_wrist_roll_joint',
        'right_gripper_joint',
    } <= joints
    assert {
        'left_rotation_l_joint',
        'left_pitch_joint',
        'left_elbow_joint',
        'left_jaw_joint',
        'right_rotation_r_joint',
        'right_pitch_joint',
        'right_elbow_joint',
        'right_jaw_joint',
    }.isdisjoint(joints)

    expected_mounts = {
        'left_arm_mount': '0.09 0.11 0.395 0 0 3.14159265',
        'right_arm_mount': '0.09 -0.11 0.395 0 0 0',
    }
    for joint_name, expected_pose in expected_mounts.items():
        joint = model.find(f"joint[@name='{joint_name}']")
        assert joint is not None
        assert joint.findtext('pose') == expected_pose

    for side in ('left', 'right'):
        shoulder_yaw = model.find(
            f"joint[@name='{side}_shoulder_yaw_joint']"
        )
        assert shoulder_yaw is not None
        assert shoulder_yaw.findtext('axis/xyz') == '0 1 0'


def test_world_uses_canonical_wheel_names_and_positive_x_front():
    root = ElementTree.parse(WORLD_PATH).getroot()
    model = root.find("./world/model[@name='cleany_mecanum']")
    assert model is not None

    wheel_joints = {
        'rear_left_wheel_joint',
        'rear_right_wheel_joint',
        'front_left_wheel_joint',
        'front_right_wheel_joint',
    }
    joints = {joint.attrib['name'] for joint in model.findall('joint')}
    assert wheel_joints <= joints
    assert {'left_wheel_joint', 'right_wheel_joint'}.isdisjoint(joints)
    for joint_name in wheel_joints:
        axis = model.find(f"joint[@name='{joint_name}']/axis/xyz")
        limit = model.find(f"joint[@name='{joint_name}']/axis/limit")
        assert axis is not None
        assert limit is not None
        assert axis.text == '0 1 0'
        assert axis.attrib['expressed_in'] == 'base_link'
        assert limit.findtext('lower') == '-1000000'
        assert limit.findtext('upper') == '1000000'
        assert limit.findtext('effort') == '1000000'

    expected_friction_directions = {
        'rear_left_wheel': '1 1 0',
        'rear_right_wheel': '1 -1 0',
        'front_left_wheel': '1 -1 0',
        'front_right_wheel': '1 1 0',
    }
    for link_name, expected_direction in expected_friction_directions.items():
        link = model.find(f"link[@name='{link_name}']")
        assert link is not None
        collision = link.find("collision[@name='mecanum_contact']")
        assert collision is not None
        assert collision.findtext('geometry/sphere/radius') == '0.0635'
        assert collision.findtext('surface/friction/ode/mu') == '1.0'
        assert collision.findtext('surface/friction/ode/mu2') == '0.0'
        friction_direction = collision.find('surface/friction/ode/fdir1')
        assert friction_direction is not None
        assert friction_direction.text == expected_direction

    plugin = model.find(
        "plugin[@name='ignition::gazebo::systems::MecanumDrive']"
    )
    assert plugin is not None
    assert plugin.findtext('front_left_joint') == 'front_left_wheel_joint'
    assert plugin.findtext('front_right_joint') == 'front_right_wheel_joint'
    assert plugin.findtext('back_left_joint') == 'rear_left_wheel_joint'
    assert plugin.findtext('back_right_joint') == 'rear_right_wheel_joint'

    top_mount = model.find("joint[@name='top_base_mount']")
    platform_visual = model.find(
        "link[@name='base_link']/visual[@name='raskog_body_visual']"
    )
    assert top_mount is not None
    assert platform_visual is not None
    assert top_mount.findtext('pose') == '0 0 0.35 0 0 3.14159265'
    assert platform_visual.findtext('pose') == (
        '0.01668 -0.502 0.515 1.5708 0 3.14159265'
    )


def test_world_reuses_description_mesh_resource_uris():
    root = ElementTree.parse(WORLD_PATH).getroot()
    mesh_uris = {uri.text for uri in root.findall('.//mesh/uri')}

    assert 'model://meshes/raskogbody.stl' in mesh_uris
    assert 'model://meshes/Base.stl' in mesh_uris
    assert 'model://meshes/Moving_Jaw.stl' in mesh_uris
    assert 'model://meshes/topbase1.stl' in mesh_uris
    assert 'model://meshes/topbase2.stl' in mesh_uris
    assert all(
        uri is not None
        and uri.startswith('model://meshes/')
        and (DESCRIPTION_SHARE / uri.removeprefix('model://')).is_file()
        for uri in mesh_uris
    )


def test_world_contains_mujoco_top_base_arm_support():
    root = ElementTree.parse(WORLD_PATH).getroot()
    model = root.find("./world/model[@name='cleany_mecanum']")
    assert model is not None

    support = model.find("link[@name='top_base_link']")
    support_joint = model.find("joint[@name='top_base_mount']")
    assert support is not None
    assert support_joint is not None
    assert support.findtext('inertial/mass') == '0.2542'
    assert support_joint.findtext('parent') == 'base_link'


def test_world_contains_three_mujoco_camera_modules():
    root = ElementTree.parse(WORLD_PATH).getroot()
    camera_names = {
        sensor.attrib['name']
        for sensor in root.findall(".//sensor")
        if sensor.attrib['type'] in {'camera', 'depth_camera'}
    }
    assert camera_names == {
        'head_realsense_rgb',
        'head_realsense_depth',
        'left_wrist_rgb',
        'right_wrist_rgb',
    }
    assert root.find(".//link[@name='head_camera_link']") is not None
    assert root.find(".//link[@name='left_wrist_camera_link']") is not None
    assert root.find(".//link[@name='right_wrist_camera_link']") is not None

    head_pan = root.find(".//link[@name='head_pan_link']")
    head_tilt = root.find(".//link[@name='head_tilt_link']")
    assert head_pan is not None
    assert head_tilt is not None
    assert {
        visual.findtext('geometry/mesh/uri') for visual in head_pan.findall('visual')
    } == {
        'model://meshes/tophead1.stl',
        'model://meshes/tophead4.stl',
    }
    assert {
        visual.findtext('geometry/mesh/uri') for visual in head_tilt.findall('visual')
    } == {
        'model://meshes/tophead5.stl',
        'model://meshes/tophead6.stl',
    }

    for wrist_camera in ('left_wrist_camera_link', 'right_wrist_camera_link'):
        wrist_link = root.find(f".//link[@name='{wrist_camera}']")
        assert wrist_link is not None
        assert {
            visual.findtext('geometry/mesh/uri')
            for visual in wrist_link.findall('visual')
        } == {
            'model://meshes/XLeRobot_camera1.stl',
            'model://meshes/XLeRobot_camera2.stl',
        }

    head_mount = root.find(".//joint[@name='head_camera_mount']")
    assert head_mount is not None
    assert head_mount.findtext('pose') == '0.025 0 0.03 0 0 0'

    expected_wrist_mount_poses = {
        'left_wrist_camera_mount': '0 -0.022 0.05 1.5708 0 -1.5708',
        'right_wrist_camera_mount': '0 -0.021 0.05 1.5708 0 -1.5708',
    }
    for mount_name, expected_pose in expected_wrist_mount_poses.items():
        mount = root.find(f".//joint[@name='{mount_name}']")
        assert mount is not None
        assert mount.findtext('pose') == expected_pose

    for sensor_name in ('left_wrist_rgb', 'right_wrist_rgb'):
        sensor = root.find(f".//sensor[@name='{sensor_name}']")
        assert sensor is not None
        assert sensor.findtext('pose') == (
            '-0.00833 0.01494 0.003872 0 0.007614 -0.436597'
        )

    expected_optical_frames = {
        'head_realsense_rgb': 'head_camera_rgb_optical_frame',
        'head_realsense_depth': 'head_camera_depth_optical_frame',
        'left_wrist_rgb': 'left_wrist_rgb_optical_frame',
        'right_wrist_rgb': 'right_wrist_rgb_optical_frame',
    }
    for sensor_name, frame_id in expected_optical_frames.items():
        sensor = root.find(f".//sensor[@name='{sensor_name}']")
        assert sensor is not None
        assert sensor.findtext('gz_frame_id') == frame_id


def test_world_contains_gpu_lidar():
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
    assert sensor.findtext('always_on') == 'true'
    assert sensor.findtext('update_rate') == '5.5'
    assert sensor.findtext('lidar/scan/horizontal/samples') == '360'
    assert sensor.findtext('lidar/range/min') == '0.15'
    assert sensor.findtext('lidar/range/max') == '12.0'


def test_world_contains_base_imu():
    root = ElementTree.parse(WORLD_PATH).getroot()
    world = root.find('./world')
    model = root.find("./world/model[@name='cleany_mecanum']")
    assert world is not None
    assert model is not None

    imu_system = world.find(
        "plugin[@name='ignition::gazebo::systems::Imu']"
    )
    mount = model.find("joint[@name='imu_mount']")
    sensor = model.find("link[@name='imu_link']/sensor[@name='base_imu']")
    assert imu_system is not None
    assert imu_system.attrib['filename'] == 'ignition-gazebo-imu-system'
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


def test_fortress_bridges_publish_gpu_lidar_scan():
    bridge = BRIDGE_PATH.read_text(encoding='utf-8')
    lidar_bridge = LIDAR_BRIDGE_PATH.read_text(encoding='utf-8')

    for config in (bridge, lidar_bridge):
        assert 'ros_topic_name: "/scan"' in config
        assert 'gz_topic_name: "/model/cleany_mecanum/lidar/scan"' in config
        assert 'ignition.msgs.LaserScan' in config


def test_fortress_bridge_publishes_base_imu():
    bridge = BRIDGE_PATH.read_text(encoding='utf-8')

    assert 'ros_topic_name: "/imu/data"' in bridge
    assert 'gz_topic_name: "/model/cleany_mecanum/imu"' in bridge
    assert 'ros_type_name: "sensor_msgs/msg/Imu"' in bridge
    assert 'gz_type_name: "ignition.msgs.IMU"' in bridge


def test_arm_links_use_extended_description_geometry_and_inertia():
    root = ElementTree.parse(WORLD_PATH).getroot()
    model = root.find("./world/model[@name='cleany_mecanum']")
    assert model is not None

    for arm in ('left', 'right'):
        base = model.find(f"link[@name='{arm}_arm_base']")
        upper_arm = model.find(f"link[@name='{arm}_upper_arm']")
        lower_arm = model.find(f"link[@name='{arm}_lower_arm']")
        moving_jaw = model.find(f"link[@name='{arm}_moving_jaw']")
        assert base is not None
        assert upper_arm is not None
        assert lower_arm is not None
        assert moving_jaw is not None

        assert base.findtext('inertial/mass') == '0.147'
        assert base.findtext('inertial/inertia/ixz') == '0.00000497151'
        assert upper_arm.findtext('inertial/mass') == '0.1185'
        assert upper_arm.findtext('inertial/inertia/ixx') == (
            '0.000344882383932'
        )
        assert upper_arm.findtext('collision/geometry/mesh/uri') == (
            'model://meshes/Upper_Arm_5cm.stl'
        )
        assert {
            uri.text for uri in upper_arm.findall('visual/geometry/mesh/uri')
        } == {
            'model://meshes/Upper_Arm_5cm_Sockets.stl',
            'model://meshes/Upper_Arm_5cm_Carbon.stl',
            'model://meshes/Upper_Arm_Motor.stl',
        }

        assert lower_arm.findtext('inertial/mass') == '0.093'
        assert lower_arm.findtext('inertial/inertia/izz') == (
            '0.0000182344857592'
        )
        assert lower_arm.findtext('collision/geometry/mesh/uri') == (
            'model://meshes/Lower_Arm_5cm.stl'
        )
        assert {
            uri.text for uri in lower_arm.findall('visual/geometry/mesh/uri')
        } == {
            'model://meshes/Lower_Arm_5cm_Sockets.stl',
            'model://meshes/Lower_Arm_5cm_Carbon.stl',
            'model://meshes/Lower_Arm_Motor.stl',
        }

        elbow = model.find(f"joint[@name='{arm}_elbow_pitch_joint']")
        wrist = model.find(f"joint[@name='{arm}_wrist_pitch_joint']")
        assert elbow is not None
        assert wrist is not None
        assert elbow.findtext('pose') == '0 0.16257 0.028 -1.5708 0 0'
        assert wrist.findtext('pose') == '0 0.0052 0.1849 -1.5708 0 0'

        collision_uris = {
            uri.text for uri in moving_jaw.findall('collision/geometry/mesh/uri')
        }
        assert collision_uris == {
            'model://meshes/Moving_Jaw_part1.ply.convex.stl',
            'model://meshes/Moving_Jaw_part2.ply.convex.stl',
            'model://meshes/Moving_Jaw_part3.ply.convex.stl',
        }
        assert moving_jaw.findtext('collision/surface/friction/ode/mu') == '3.0'


def test_generated_world_uses_fixed_mecanum_roller_visuals(tmp_path):
    generated_world = materialize_mecanum_wheel_world(WORLD_PATH)
    copied_world = tmp_path / generated_world.name
    copied_world.write_text(generated_world.read_text(encoding='utf-8'), encoding='utf-8')

    root = ElementTree.parse(copied_world).getroot()
    model = root.find("./world/model[@name='cleany_mecanum']")
    assert model is not None

    roller_links = [
        link for link in model.findall('link') if '_roller_' in link.attrib['name']
    ]
    roller_joints = [
        joint for joint in model.findall('joint') if '_roller_' in joint.attrib['name']
    ]
    roller_visuals = [
        visual
        for visual in model.findall('link/visual')
        if '_roller_' in visual.attrib['name']
    ]
    assert not roller_links
    assert not roller_joints
    assert len(roller_visuals) == 48
    assert all(
        visual.findtext('geometry/capsule/radius') == '0.008'
        and visual.findtext('geometry/capsule/length') == '0.03'
        for visual in roller_visuals
    )

    for wheel_name in (
        'rear_left_wheel',
        'rear_right_wheel',
        'front_left_wheel',
        'front_right_wheel',
    ):
        wheel = model.find(f"link[@name='{wheel_name}']")
        assert wheel is not None
        assert len(
            [
                visual
                for visual in wheel.findall('visual')
                if '_roller_' in visual.attrib['name']
            ]
        ) == 12
