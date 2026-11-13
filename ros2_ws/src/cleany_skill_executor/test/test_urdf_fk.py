import numpy as np
import pytest
from cleany_skill_executor.core.urdf_fk import UrdfChain
from cleany_skill_executor.core.alignment import rotation_from_quaternion


def test_runtime_urdf_chain_composes_origin_rotation_and_joint_motion():
    xml = '''<robot name="test"><joint name="q" type="revolute">
      <parent link="base"/><child link="arm"/><origin xyz="1 0 0"/>
      <axis xyz="0 0 1"/></joint><joint name="tool" type="fixed">
      <parent link="arm"/><child link="tcp"/><origin xyz="1 0 0"/></joint></robot>'''
    chain = UrdfChain(xml,'base','tcp')
    p,r = chain.pose({'q':np.pi/2})
    np.testing.assert_allclose(p,[1,1,0],atol=1e-12)
    np.testing.assert_allclose(r[:,0],[0,1,0],atol=1e-12)
    with pytest.raises(ValueError):
        UrdfChain(xml,'missing','tcp')


@pytest.mark.parametrize('axis', ((1., 0., 0.), (0., -2., 0.), (1., -2., 3.)))
@pytest.mark.parametrize('angle', (0., 1e-10, -0.7, np.pi, 2*np.pi))
def test_axis_rotation_matches_quaternion_and_preserves_axis(axis, angle):
    from cleany_skill_executor.core.alignment import rotation_from_quaternion
    from cleany_skill_executor.core.urdf_fk import axis_rotation

    values = np.array(axis)
    values.setflags(write=False)
    unit = values / np.linalg.norm(values)
    quaternion = (*tuple(unit*np.sin(angle/2)), float(np.cos(angle/2)))
    expected = rotation_from_quaternion(quaternion)
    np.testing.assert_allclose(axis_rotation(values, angle), expected, atol=2e-15)
    np.testing.assert_array_equal(values, axis)


def test_chain_handles_prismatic_continuous_fixed_origins_and_invalid_positions():
    xml = """<robot>
      <joint name="fixed" type="fixed"><parent link="base"/><child link="a"/>
        <origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/></joint>
      <joint name="slide" type="prismatic"><parent link="a"/><child link="b"/>
        <axis xyz="2 0 0"/></joint>
      <joint name="turn" type="continuous"><parent link="b"/><child link="c"/>
        <axis xyz="0 0 -3"/></joint>
      <joint name="tip" type="fixed"><parent link="c"/><child link="tcp"/>
        <origin xyz="0.2 0 0"/></joint></robot>"""
    chain = UrdfChain(xml, 'base', 'tcp')
    position, rotation = chain.pose({'slide': 0.4, 'turn': np.pi/2})
    np.testing.assert_allclose(position, (1.2, 0.4, 0.), atol=2e-15)
    np.testing.assert_allclose(rotation, np.eye(3), atol=2e-15)
    with pytest.raises(ValueError, match='Nonfinite'):
        chain.pose({'slide': np.nan, 'turn': 0.})
    with pytest.raises(ValueError, match='Nonfinite'):
        chain.pose({'slide': 0., 'turn': np.inf})


@pytest.mark.parametrize('kind', ['revolute', 'continuous', 'prismatic'])
def test_runtime_fk_reuses_unit_axes_and_matches_quaternion_reference(monkeypatch, kind):
    import cleany_skill_executor.core.urdf_fk as module
    xml = f'''<robot name="test"><joint name="q" type="{kind}">
      <parent link="base"/><child link="arm"/><origin xyz=".3 .1 -.2" rpy="0 0 .4"/>
      <axis xyz="1 2 3"/></joint><joint name="tool" type="fixed">
      <parent link="arm"/><child link="tcp"/><origin xyz=".05 .07 .09"/></joint></robot>'''
    chain = UrdfChain(xml, 'base', 'tcp')
    axis = np.array((1., 2., 3.)) / np.sqrt(14.)
    origin_rotation = rotation_from_quaternion((0., 0., np.sin(.2), np.cos(.2)))
    def unexpected_normalization(*args):
        pytest.fail('Pose evaluation must reuse the fixed URDF unit axes')
    monkeypatch.setattr(module, 'axis_rotation', unexpected_normalization)
    for value in (-2., 0., 1.3, -2.):
        position, rotation = chain.pose({'q': value})
        translation = np.array((.3, .1, -.2))
        expected_rotation = origin_rotation
        if kind == 'prismatic':
            translation = translation+origin_rotation@axis*value
        else:
            motion = rotation_from_quaternion((*axis*np.sin(value/2), np.cos(value/2)))
            expected_rotation = origin_rotation@motion
        expected_position = translation+expected_rotation@np.array((.05, .07, .09))
        np.testing.assert_allclose(position, expected_position, atol=2e-15, rtol=0.)
        np.testing.assert_allclose(rotation, expected_rotation, atol=2e-15, rtol=0.)
    with pytest.raises(ValueError, match='Nonfinite'):
        chain.pose({'q': float('nan')})
