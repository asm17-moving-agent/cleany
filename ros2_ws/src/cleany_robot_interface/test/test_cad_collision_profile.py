"""The calibrated profile selects only verified collision geometry."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

PREFIX = 'package://cleany_description/meshes/cad_runtime_collision/'


def rigid_member(name):
    return ET.fromstring(
        f'<collision name="{name}"><origin xyz="0 0 0" rpy="0 0 0"/>'
        f'<geometry><mesh filename="{PREFIX}{"a" * 64}.glb"/></geometry></collision>')


@pytest.mark.parametrize('scope', ['', 'cad_self__', 'cad_world__'])
def test_rigid_group_owns_only_original_link_and_scope(tmp_path, scope):
    from cleany_robot_interface.rigid_collision import apply_rigid_collision_groups, member_digest

    robot = ET.fromstring('<robot><link name="wrist"><visual/></link><link name="finger"/></robot>')
    wrist, finger = robot.findall('link')
    sources = [rigid_member(scope + name) for name in ('plate', 'servo')]
    wrist.extend(sources)
    finger.append(rigid_member('pad'))
    entry = {
        'name': 'palm', 'link': 'wrist', 'members': [s.get('name') for s in sources],
        'source_sha256': member_digest(sources, tmp_path),
        'shapes': [{'type': 'box', 'xyz': [0, 0, .1], 'rpy': [0, 0, 0], 'size': [.1, .2, .3]}],
    }
    finger_xml = ET.tostring(finger)
    assert apply_rigid_collision_groups(robot, [entry], tmp_path) == set()
    body, = wrist.findall('collision')
    assert body.get('name') == scope + 'rigid_fit_palm_0'
    assert list(map(float, body.find('origin').get('xyz').split())) == [0, 0, .1]
    assert list(map(float, body.find('geometry/box').get('size').split())) == [.1, .2, .3]
    assert wrist.find('visual') is not None and ET.tostring(finger) == finger_xml


def test_rigid_fit_rejects_mixed_scopes_and_changed_member_pose(tmp_path):
    from cleany_robot_interface.rigid_collision import apply_rigid_collision_groups, member_digest

    robot = ET.fromstring('<robot><link name="wrist"/></robot>')
    sources = [rigid_member('plate'), rigid_member('cad_self__rod')]
    link = robot.find('link')
    link.extend(sources)
    entry = {'name': 'palm', 'link': 'wrist', 'members': [s.get('name') for s in sources],
             'source_sha256': member_digest(sources, tmp_path), 'shapes': [{'type': 'box'}]}
    with pytest.raises(ValueError, match='mixes collision scopes'):
        apply_rigid_collision_groups(robot, [entry], tmp_path)
    entry.update(members=['plate'], source_sha256=member_digest(sources[:1], tmp_path))
    sources[0].find('origin').set('xyz', '0.001 0 0')
    with pytest.raises(ValueError, match='Stale'):
        apply_rigid_collision_groups(robot, [entry], tmp_path)
    entry['members'] = ['missing']
    with pytest.raises(ValueError, match='same link'):
        apply_rigid_collision_groups(robot, [entry], tmp_path)
    assert link.findall('collision') == sources


def test_rigid_member_digest_binds_the_contents_of_a_named_stl(tmp_path):
    from cleany_robot_interface.rigid_collision import member_digest

    link_shape = rigid_member('drive_rod')
    link_shape.find('geometry/mesh').set('filename', 'package://cleany_description/meshes/rod.stl')
    path = tmp_path / 'ros2_ws/src/cleany_description/meshes/rod.stl'
    path.parent.mkdir(parents=True)
    path.write_bytes(b'previous rod')
    original = member_digest([link_shape], tmp_path)
    path.write_bytes(b'revised rod')
    assert member_digest([link_shape], tmp_path) != original


def test_commissioned_profile_reproduces_installed_collision_model(tmp_path):
    from cleany_robot_interface.state_profile import prepare_state_profile

    repository = Path(__file__).resolve().parents[4]
    profile = repository / 'configs/robot/cad_grasp.json'
    model = repository / 'ros2_ws/src/cleany_description/urdf/cleany.urdf'
    prepared = prepare_state_profile(profile, repository, tmp_path / 'prepared')
    assert prepared.model.read_bytes() == model.read_bytes()
    root = ET.parse(model)
    for side in ('left', 'right'):
        for joint, tube in (('shoulder_pitch', 'UpperTube'), ('elbow_pitch', 'LowerTube')):
            link = root.find(f'link[@name="cad_{side}_{joint}"]')
            matches = [shape for shape in link.findall('collision') if tube in shape.get('name', '')]
            assert len(matches) == 1
            assert matches[0].find('geometry/box') is not None
            assert not matches[0].get('name').startswith(('cad_self__', 'cad_world__'))
