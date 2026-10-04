from pathlib import Path
from xml.etree import ElementTree as ET
import pytest
from cleany_gazebo_sim.world.sdf import materialize_friction_frames, NAMESPACES


@pytest.mark.parametrize('harmonic,prefix', [(False, 'ignition'), (True, 'gz')])
def test_external_namespace_alias_preserves_body_fixed_friction(tmp_path: Path, harmonic, prefix):
    source = tmp_path/'world.sdf'
    text = '<sdf xmlns:ns0="http://ignitionrobotics.org/schema"><world><model><link><collision><surface><friction><ode><fdir1 ns0:expressed_in="base_link">1 -1 0</fdir1></ode></friction></surface></collision></link></model></world></sdf>'
    source.write_text(text)
    result = materialize_friction_frames(source, harmonic=harmonic)
    assert source.read_text() == text
    # The external physics reader uses this literal qualified attribute name.
    assert f'{prefix}:expressed_in="base_link"' in result.read_text()
    direction = ET.parse(result).find('.//fdir1')
    assert direction.text == '1 -1 0'
    assert direction.get('{'+NAMESPACES[prefix]+'}expressed_in') == 'base_link'


def test_physics_override_preserves_source_geometry_and_target_factor(tmp_path):
    source = tmp_path/'world.sdf'
    original = ('<sdf><world name="test"><physics name="p" type="ignored">'
                '<max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor>'
                '</physics><model name="desk"><pose>1 2 3 0 0 0</pose>'
                '<link name="body"><collision name="box"><geometry><box>'
                '<size>1 2 3</size></box></geometry></collision></link></model></world></sdf>')
    source.write_text(original)
    assert materialize_friction_frames(source) == source
    result = materialize_friction_frames(source, max_step_size=.002)
    assert result != source and source.read_text() == original
    actual, expected = ET.parse(result), ET.fromstring(original)
    assert actual.findtext('./world/physics/max_step_size') == '0.002'
    assert actual.findtext('./world/physics/real_time_factor') == '1.0'
    actual.find('./world/physics/max_step_size').text = '0.001'
    assert ET.tostring(actual.getroot()) == ET.tostring(expected)


@pytest.mark.parametrize('step', [0., -.001, .02, float('inf'), float('nan')])
def test_invalid_physics_override_fails_before_launch(tmp_path, step):
    source = tmp_path/'world.sdf'
    source.write_text('<sdf><world><physics /></world></sdf>')
    with pytest.raises(ValueError):
        materialize_friction_frames(source, max_step_size=step)


def test_requested_rtf_does_not_change_integration_or_geometry(tmp_path):
    source = tmp_path / 'world.sdf'
    original = ('<sdf><world><physics><max_step_size>0.001</max_step_size>'
                '<real_time_factor>1.0</real_time_factor></physics>'
                '<model name="obstacle"><static>true</static></model></world></sdf>')
    source.write_text(original)
    result = materialize_friction_frames(source, real_time_factor=2.0)
    actual = ET.parse(result)
    assert source.read_text() == original
    assert actual.findtext('./world/physics/real_time_factor') == '2.0'
    actual.find('./world/physics/real_time_factor').text = '1.0'
    assert ET.tostring(actual.getroot()) == ET.tostring(ET.fromstring(original))


@pytest.mark.parametrize('factor', [0., -1., float('inf'), float('nan')])
def test_invalid_requested_rtf_fails_before_launch(tmp_path, factor):
    source = tmp_path / 'world.sdf'
    source.write_text('<sdf><world><physics /></world></sdf>')
    with pytest.raises(ValueError):
        materialize_friction_frames(source, real_time_factor=factor)
