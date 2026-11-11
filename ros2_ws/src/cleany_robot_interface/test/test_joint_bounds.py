import math
import xml.etree.ElementTree as ET

import pytest

from cleany_robot_interface.core import JointMapping
from cleany_robot_interface.joint_bounds import apply_joint_bounds, calibrated_joint_bounds


def test_calibrated_bounds_use_present_position_coordinates_and_both_directions():
    joints = [JointMapping('left_pitch', 2, 2825, 3104, -1, 0, 1180, 855, 3256),
              JointMapping('left_wrist', 4, 777, 2008, 1, .1, 1533, 811, 3226)]
    bounds = calibrated_joint_bounds(joints, ['left_pitch', 'left_wrist'])
    assert bounds['left_pitch'] == pytest.approx((-.23316507975861744, 3.449922791954807))
    assert bounds['left_wrist'] == pytest.approx((-1.7361750030991124, 1.968388599644711))
    assert calibrated_joint_bounds(joints, []) == {}


@pytest.mark.parametrize('selected', [['unknown'], ['pitch', 'pitch'], 'pitch', None])
def test_calibrated_bounds_reject_invalid_selection(selected):
    joint = JointMapping('pitch', 2, 2825, 2048, 1, 0, 0, 500, 3500)
    with pytest.raises(ValueError, match='unique calibrated arm joints'):
        calibrated_joint_bounds([joint], selected)


MODEL = '''<robot name="test"><joint name="pitch" type="revolute">
<parent link="base"/><child link="arm"/><origin xyz="1 2 3" rpy="0 1 0"/>
<axis xyz="-1 0 0"/><limit lower="-0.22" upper="3.37" velocity="1" effort="2"/>
</joint><joint name="fixed" type="fixed"><parent link="arm"/><child link="camera"/>
<origin xyz="0 0 0.1"/></joint></robot>'''


def test_applying_bounds_changes_only_selected_lower_and_upper():
    changed = ET.fromstring(apply_joint_bounds(MODEL, {'pitch': [-.233, 3.45]}))
    limit = changed.find("joint[@name='pitch']/limit")
    assert float(limit.get('lower')) == -.233 and float(limit.get('upper')) == 3.45
    limit.set('lower', '-0.22')
    limit.set('upper', '3.37')
    assert ET.tostring(changed) == ET.tostring(ET.fromstring(MODEL))


@pytest.mark.parametrize('bounds', [
    {'pitch': [1, -1]}, {'pitch': [math.nan, 1]}, {'pitch': [-1, math.inf]},
    {'pitch': [0]}, {'pitch': 'invalid'}, {'missing': [0, 1]}, {'fixed': [0, 1]},
])
def test_invalid_bounds_or_joint_rejected(bounds):
    with pytest.raises(ValueError):
        apply_joint_bounds(MODEL, bounds)
