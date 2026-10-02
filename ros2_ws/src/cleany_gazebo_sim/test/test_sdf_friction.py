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
