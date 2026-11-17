"""Exercise calibrated coaxial joint scales without hardware feedback."""
import math
import numpy as np
import pytest

from cleany_skill_executor.core.urdf_fk import UrdfChain


def description(source='motor', multiplier='-.04', offset='.1'):
    return f'''<robot name="scaled"><joint name="motor" type="revolute">
      <parent link="base"/><child link="intermediate"/><axis xyz="0 0 1"/>
      </joint><joint name="correction" type="revolute">
      <parent link="intermediate"/><child link="arm"/><axis xyz="0 0 1"/>
      <mimic joint="{source}" multiplier="{multiplier}" offset="{offset}"/>
      </joint><joint name="tip" type="fixed"><parent link="arm"/><child link="tip"/>
      <origin xyz=".4 0 0"/></joint></robot>'''


@pytest.mark.parametrize('value', [-2.4,0.,1.7])
def test_affine_mimic_uses_actuator_position_once(value):
    chain = UrdfChain(description(), 'base', 'tip')
    position, result = chain.pose({'motor':value})
    angle = .96*value+.1
    np.testing.assert_allclose(position,[.4*math.cos(angle),.4*math.sin(angle),0],atol=1e-15)
    np.testing.assert_allclose(result[:,0],[math.cos(angle),math.sin(angle),0],atol=1e-15)


@pytest.mark.parametrize('source,multiplier,offset', [('missing','1','0'),('correction','1','0'),
                                                   ('motor','nan','0'),('motor','1','inf')])
def test_invalid_or_cyclic_mimic_cannot_produce_a_pose(source,multiplier,offset):
    with pytest.raises(ValueError,match='mimic'):
        UrdfChain(description(source,multiplier,offset),'base','tip')
