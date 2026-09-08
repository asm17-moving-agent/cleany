import numpy as np
import pytest
from cleany_skill_executor.core.urdf_fk import UrdfChain


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
