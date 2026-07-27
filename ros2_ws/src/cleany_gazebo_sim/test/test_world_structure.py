from pathlib import Path
from xml.etree import ElementTree

from cleany_gazebo_sim.world_generator import materialize_articulated_roller_world

WORLD_PATH = (
    Path(__file__).resolve().parents[1] / 'worlds' / 'cleany_mecanum_prototype.sdf'
)


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
        'left_rotation_l_joint',
        'left_jaw_joint',
        'right_rotation_r_joint',
        'right_jaw_joint',
    } <= joints


def test_world_reuses_mujoco_mesh_resource_uris():
    root = ElementTree.parse(WORLD_PATH).getroot()
    mesh_uris = {uri.text for uri in root.findall('.//mesh/uri')}

    assert 'model://assets/raskogbody.stl' in mesh_uris
    assert 'model://assets/Base.stl' in mesh_uris
    assert 'model://assets/Moving_Jaw.stl' in mesh_uris
    assert 'model://assets/topbase1.stl' in mesh_uris
    assert 'model://assets/topbase2.stl' in mesh_uris


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
        sensor.attrib['name'] for sensor in root.findall(".//sensor")
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
        'model://assets/tophead1.stl',
        'model://assets/tophead4.stl',
    }
    assert {
        visual.findtext('geometry/mesh/uri') for visual in head_tilt.findall('visual')
    } == {
        'model://assets/tophead5.stl',
        'model://assets/tophead6.stl',
    }

    for wrist_camera in ('left_wrist_camera_link', 'right_wrist_camera_link'):
        wrist_link = root.find(f".//link[@name='{wrist_camera}']")
        assert wrist_link is not None
        assert {
            visual.findtext('geometry/mesh/uri')
            for visual in wrist_link.findall('visual')
        } == {
            'model://assets/XLeRobot_camera1.stl',
            'model://assets/XLeRobot_camera2.stl',
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


def test_arm_links_use_mujoco_inertia_and_convex_collision_meshes():
    root = ElementTree.parse(WORLD_PATH).getroot()
    model = root.find("./world/model[@name='cleany_mecanum']")
    assert model is not None

    for arm in ('left', 'right'):
        base = model.find(f"link[@name='{arm}_arm_base']")
        upper_arm = model.find(f"link[@name='{arm}_upper_arm']")
        moving_jaw = model.find(f"link[@name='{arm}_moving_jaw']")
        assert base is not None
        assert upper_arm is not None
        assert moving_jaw is not None

        assert base.findtext('inertial/mass') == '0.147'
        assert base.findtext('inertial/inertia/ixz') == '0.00000497151'
        assert upper_arm.findtext('inertial/mass') == '0.103'
        assert upper_arm.findtext('collision/geometry/mesh/uri') == (
            'model://assets/Upper_Arm.stl.convex.stl'
        )

        collision_uris = {
            uri.text for uri in moving_jaw.findall('collision/geometry/mesh/uri')
        }
        assert collision_uris == {
            'model://assets/Moving_Jaw_part1.ply.convex.stl',
            'model://assets/Moving_Jaw_part2.ply.convex.stl',
            'model://assets/Moving_Jaw_part3.ply.convex.stl',
        }
        assert moving_jaw.findtext('collision/surface/friction/ode/mu') == '3.0'


def test_generated_world_contains_48_articulated_mecanum_rollers(tmp_path):
    generated_world = materialize_articulated_roller_world(WORLD_PATH)
    copied_world = tmp_path / generated_world.name
    copied_world.write_text(generated_world.read_text(encoding='utf-8'), encoding='utf-8')

    root = ElementTree.parse(copied_world).getroot()
    model = root.find("./world/model[@name='cleany_mecanum']")
    assert model is not None

    roller_links = [link for link in model.findall('link') if '_roller_' in link.attrib['name']]
    roller_joints = [joint for joint in model.findall('joint') if '_roller_' in joint.attrib['name']]
    assert len(roller_links) == 48
    assert len(roller_joints) == 48
