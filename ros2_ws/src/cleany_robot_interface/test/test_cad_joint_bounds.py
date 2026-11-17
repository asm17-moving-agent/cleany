import math
import xml.etree.ElementTree as ET

import pytest

from cleany_robot_interface.cad_joint_bounds import apply_cad_joint_bounds

SOURCE = '''<robot name="source"><joint name="pitch" type="revolute">
<limit lower="-1" upper="2" effort="4" velocity="3"/></joint></robot>'''
CAD = '''<robot name="cad"><joint name="cad_pitch" type="continuous">
<parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
<origin xyz="1 2 3" rpy="0 0 0"/></joint>
<joint name="cam" type="continuous"/></robot>'''


@pytest.mark.parametrize('scale,expected', [(2., (-1.5, 4.5)), (-2., (-3.5, 2.5))])
def test_transports_both_endpoints_without_changing_geometry_or_linkages(scale, expected):
    result = ET.fromstring(apply_cad_joint_bounds(CAD, SOURCE, {
        'cad_pitch': {'source_joint': 'pitch', 'scale': scale, 'offset_rad': .5}}))
    joint = result.find("joint[@name='cad_pitch']")
    limit = joint.find('limit')
    assert tuple(float(limit.get(k)) for k in ('lower', 'upper')) == expected
    assert float(limit.get('velocity')) == 6.
    joint.remove(limit)
    joint.set('type', 'continuous')
    assert ET.tostring(result) == ET.tostring(ET.fromstring(CAD))


@pytest.mark.parametrize('scale,offset', [(0, 0), (math.nan, 0), (1, math.inf)])
def test_rejects_invalid_mapping(scale, offset):
    with pytest.raises(ValueError, match='coordinate mapping'):
        apply_cad_joint_bounds(CAD, SOURCE, {
            'cad_pitch': {'source_joint': 'pitch', 'scale': scale, 'offset_rad': offset}})


def test_rejects_missing_source_instead_of_using_nominal_limits():
    with pytest.raises(ValueError, match='bounded operational joint'):
        apply_cad_joint_bounds(CAD, SOURCE, {
            'cad_pitch': {'source_joint': 'missing', 'scale': 1, 'offset_rad': 0}})
