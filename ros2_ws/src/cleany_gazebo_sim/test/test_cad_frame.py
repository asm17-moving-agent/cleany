from pathlib import Path
import json
import shutil
from xml.etree import ElementTree as ET

import numpy as np
import pytest

from cleany_gazebo_sim.world.cad_frame import freeze
from cleany_gazebo_sim.world.generator import materialize_mecanum_wheel_world


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


@pytest.mark.skipif(shutil.which('gz') is None, reason='Gazebo sdformat conversion required')
def test_cad_conversion_preserves_drive_sensor_and_envelope_contract(tmp_path):
    pytest.importorskip('xacro')
    package = Path(__file__).resolve().parents[1]
    world = materialize_mecanum_wheel_world(
        package/'worlds/cleany_mecanum_harmonic.sdf', target_path=tmp_path/'world.sdf', robot_model='cad_frame')
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
    assert len(model.findall('.//sensor')) == 6
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
    assert high[1]-low[1] == pytest.approx(.6548, abs=1e-6)
    angle = np.deg2rad(30)
    np.testing.assert_allclose(meta['depth_camera_tf']['rotation_rpy'], [0, angle, 0], atol=1e-9)
    expected = [0.102+0.025*np.cos(angle)+0.03*np.sin(angle), -0.002,
                0.77115-0.025*np.sin(angle)+0.03*np.cos(angle)]
    np.testing.assert_allclose(meta['depth_camera_tf']['translation'], expected, atol=1e-9)
    assert meta['depth_camera_tf']['translation'][2] < 0.80115


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
