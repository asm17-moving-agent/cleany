from pathlib import Path
import json
import shutil
from xml.etree import ElementTree as ET

import numpy as np
import pytest

from cleany_gazebo_sim.world.cad_frame import freeze
from cleany_gazebo_sim.world.generator import materialize_mecanum_wheel_world
from cleany_navigation.urdf_geometry import rotation


def test_description_identity_tracks_geometry_bytes_and_expanded_origins(tmp_path):
    from cleany_gazebo_sim.world.cad_frame import description_identity
    mesh = tmp_path / 'meshes' / 'part.stl'
    mesh.parent.mkdir()
    mesh.write_bytes(b'first mesh revision')
    robot = ET.fromstring('''<robot><link name="base_link"><visual>
      <origin xyz="0 0 0"/><geometry>
      <mesh filename="package://cleany_description/meshes/part.stl"/>
      </geometry></visual></link></robot>''')
    first = description_identity(robot, tmp_path)
    assert description_identity(robot, tmp_path) == first
    mesh.write_bytes(b'second mesh revision')
    second = description_identity(robot, tmp_path)
    assert first['sha256'] != second['sha256']
    assert first['expanded_urdf_sha256'] == second['expanded_urdf_sha256']
    robot.find('.//origin').set('xyz', '0 0 1')
    third = description_identity(robot, tmp_path)
    assert third['sha256'] != second['sha256']
    assert third['mesh_sha256'] == second['mesh_sha256']

    installed = tmp_path / 'install'
    installed.mkdir()
    (installed / 'meshes').symlink_to(mesh.parent, target_is_directory=True)
    assert description_identity(robot, installed) == third


def test_park_rotation_follows_joint_axis_and_preserves_translation():
    robot = ET.fromstring('''<robot><joint name="arm" type="revolute">
      <parent link="base_link"/><child link="arm_link"/>
      <origin xyz="1 2 3" rpy="0 0 0"/><axis xyz="0 0 1"/>
      <limit lower="-2" upper="2"/></joint></robot>''')
    transforms = freeze(robot, {'arm': np.pi/2})
    np.testing.assert_allclose(transforms['arm_link'][:3, 3], [1, 2, 3])
    np.testing.assert_allclose(transforms['arm_link'][:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    assert robot.find('joint').get('type') == 'fixed'


def test_park_rejects_out_of_limit_angle():
    robot = ET.fromstring('''<robot><joint name="arm" type="revolute">
      <parent link="base_link"/><child link="arm_link"/>
      <origin/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint></robot>''')
    with pytest.raises(ValueError, match='outside limits'):
        freeze(robot, {'arm': 2})


@pytest.mark.skipif(not (shutil.which('gz') or shutil.which('ign')), reason='Gazebo sdformat conversion required')
def test_cad_conversion_preserves_drive_sensor_and_envelope_contract(tmp_path):
    pytest.importorskip('xacro')
    package = Path(__file__).resolve().parents[1]
    world = materialize_mecanum_wheel_world(
        package/'worlds/cleany_mecanum_fortress.sdf', target_path=tmp_path/'world.sdf', robot_model='cad_frame')
    model = ET.parse(world).getroot().find("world/model[@name='cleany_mecanum']")
    assert len(model.findall('link')) == 6
    assert len(model.findall('joint')) == 5
    drive = next(p for p in model.findall('plugin') if p.get('name').endswith('MecanumDrive'))
    assert float(drive.findtext('wheelbase')) == pytest.approx(0.35)
    assert float(drive.findtext('wheel_separation')) == pytest.approx(0.6038)
    assert len(model.findall('.//collision[@name="mecanum_contact"]')) == 4
    for contact in model.findall('.//collision[@name="mecanum_contact"]'):
        assert contact.find('geometry/sphere') is None
        assert float(contact.findtext('geometry/cylinder/length')) == pytest.approx(.051)
        assert float(contact.findtext('geometry/cylinder/radius')) == pytest.approx(.0635)
    assert model.find("joint[@name='head_pan_joint']").get('type') == 'revolute'
    assert model.find("link[@name='head_pan_link']/sensor[@name='head_realsense_depth']") is not None
    assert len(model.findall('.//sensor')) == 4
    assert not any('Upper_Arm_5cm_Sockets.stl' in uri.text for uri in model.findall('.//mesh/uri'))
    assert not any('wrist' in n.get('name', '') for n in model.findall('frame'))
    names = {n.get('name') for n in model if n.tag in ('link', 'frame', 'joint')}
    for sensor in model.findall('.//sensor'):
        assert sensor.find('pose').get('relative_to') in names
    for uri in model.findall('.//mesh/uri'):
        assert Path(uri.text).is_file()
    meta = json.loads(world.with_suffix('.model.json').read_text())
    low, high = np.array(meta['collision_bounds'])
    assert max(abs(low[0]), abs(high[0])) < 0.48
    assert max(abs(low[1]), abs(high[1])) < 0.40
    assert high[1]-low[1] > 0.60
    assert high[1]-low[1] == pytest.approx(.65480175, abs=1e-6)
    angle = np.deg2rad(20)
    np.testing.assert_allclose(meta['depth_camera_tf']['rotation_rpy'], [0, angle, 0], atol=1e-9)
    # 독립적으로 지정한 CAD 혼 축과 유지되는 차체 원점입니다.
    pan_pivot = np.array([.0874177404372793-.000025, -.0000074623620002+.000027, .9893865235624802-.284497])
    tilt_pivot = pan_pivot + [.00001, 0, 0]
    camera_offset = np.array([.03125, .0169, .0323960780542311])
    expected = tilt_pivot + rotation([0,angle,0]) @ camera_offset
    np.testing.assert_allclose(meta['depth_camera_tf']['pan_pivot'], pan_pivot, atol=1e-9)
    np.testing.assert_allclose(meta['depth_camera_tf']['translation'], expected, atol=1e-9)
    assert meta['depth_camera_tf']['translation'][2] < tilt_pivot[2] + camera_offset[2]

    # URDF에서 얻은 TF 메타데이터와 별도로 변환된 SDF를 해석합니다.
    entities = {e.get('name'): e for e in model if e.tag in ('link', 'joint', 'frame')}

    def local_pose(pose):
        transform = np.eye(4)
        if pose is not None:
            values = np.fromstring(pose.text, sep=' ')
            transform[:3, 3] = values[:3]
            transform[:3, :3] = rotation(values[3:])
        return transform

    def base_pose(name):
        transform = np.eye(4)
        while name != 'base_link':
            entity = entities[name]
            pose = entity.find('pose')
            transform = local_pose(pose) @ transform
            name = (pose.get('relative_to') if pose is not None else None) or entity.get('attached_to')
        return transform

    for name in ('head_realsense_rgb', 'head_realsense_depth'):
        pose = model.find(f".//sensor[@name='{name}']/pose")
        camera = base_pose(pose.get('relative_to')) @ local_pose(pose)
        # SDFormat은 URDF 변환 중 일부 고정 좌표계 RPY 값을 반올림합니다.
        np.testing.assert_allclose(camera[:3, 3], expected, atol=1e-6)
        np.testing.assert_allclose(camera[:3, :3], rotation([0, angle, 0]), atol=1e-5)


def test_navigation_geometry_preserves_offset_and_separates_margins():
    from cleany_gazebo_sim.world.cad_frame import navigation_geometry
    result = navigation_geometry(dict(collision_bounds=[[-.2, -.3, -.4], [.3, .35, .9]],
                                     navigation_margins=dict(stop=.03, planning_padding=.02, slowdown=.2)))
    np.testing.assert_allclose(result['center'], [.05, .025])
    np.testing.assert_allclose(result['half_size'], [.25, .325])
    np.testing.assert_allclose(result['physical'][0], [-.2, -.3])
    np.testing.assert_allclose(result['stop'][2], [.33, .38])
    np.testing.assert_allclose(result['slow'][0], [-.4, -.5])
    assert result['planning_padding'] == .02


def test_rotated_sphere_does_not_inflate_geometry_bounds():
    from cleany_gazebo_sim.world.cad_frame import collision_bounds
    robot = ET.fromstring('''<robot><link name="base_link"><collision>
      <origin xyz=".1 .2 .3" rpy=".7 .8 .9"/>
      <geometry><sphere radius=".05"/></geometry></collision></link></robot>''')
    np.testing.assert_allclose(collision_bounds(robot, {'base_link':np.eye(4)}),
                               [[.05,.15,.25],[.15,.25,.35]])


def test_simulation_limit_override_is_explicit_and_bounded():
    source = '''<robot><joint name="wrist" type="revolute">
      <parent link="base_link"/><child link="wrist_link"/>
      <origin/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint></robot>'''
    with pytest.raises(ValueError, match='outside limits'):
        freeze(ET.fromstring(source), {'wrist': -2})
    transforms = freeze(ET.fromstring(source), {'wrist': -2}, {'wrist': [-2, 1]})
    np.testing.assert_allclose(transforms['wrist_link'][:3, :3]@[1, 0, 0],
                               [np.cos(-2), np.sin(-2), 0])
    with pytest.raises(ValueError, match='outside limits'):
        freeze(ET.fromstring(source), {'wrist': -2.1}, {'wrist': [-2, 1]})
    with pytest.raises(ValueError, match='parked joint'):
        freeze(ET.fromstring(source), {'wrist': 0}, {'typo': [-2, 1]})


@pytest.mark.parametrize('pitch', [np.pi/2, -np.pi/2, np.pi/2-1e-12])
def test_rpy_round_trip_at_optical_axis_gimbal_lock(pitch):
    from cleany_navigation.urdf_geometry import rpy
    expected = rotation([.3, pitch, -.7])
    np.testing.assert_allclose(rotation(rpy(expected)), expected, atol=2e-12)
