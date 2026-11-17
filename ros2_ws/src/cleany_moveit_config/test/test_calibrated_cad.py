"""Keep established assembly permissions without exempting other CAD contacts."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from cleany_description.cad_coordinates import ATTRIBUTE
from cleany_moveit_config.calibrated_cad import cad_runtime_configuration


def test_existing_permissions_follow_cad_owners_and_symmetric_pads_only():
    source = Path(__file__).parents[2]/'cleany_description/config/geometry/freecad_kinematics.json'
    geometry = json.loads(source.read_text())
    metadata = {'schema':'cleany.calibrated_cad/v1','grippers':[
        {'side':g['hand'],'source_joint':g['hand']+'_gripper_joint',
         'scale':-1.,'offset_rad':0.,'linkage':g} for g in geometry['gripper_linkages']]}
    model = ET.Element('robot',name='cleany',**{ATTRIBUTE:json.dumps(metadata)})
    for name in ('base_link','top_base_link','head_pan_link','cad_head_pan','cad_head_tilt',
                 'cad_left_gripper_tip_L','cad_left_gripper_tip_R',
                 'cad_left_gripper_driving_L','cad_left_gripper_driving_R'):
        ET.SubElement(model,'link',name=name)
    semantic = '''<robot name="cleany"><disable_collisions link1="top_base_link"
        link2="head_pan_link" reason="Adjacent"/></robot>'''
    semantic, environment, detector = cad_runtime_configuration(ET.tostring(model,encoding='unicode'),semantic)
    pairs = {frozenset((item.get('link1'),item.get('link2')))
             for item in ET.fromstring(semantic).findall('disable_collisions')}
    assert frozenset(('base_link','cad_head_pan')) in pairs
    assert frozenset(('cad_left_gripper_tip_L','cad_left_gripper_tip_R')) in pairs
    assert frozenset(('base_link','cad_head_tilt')) not in pairs
    assert frozenset(('cad_left_gripper_driving_L','cad_left_gripper_driving_R')) not in pairs
    assert len(ET.fromstring(semantic).findall('passive_joint')) == 16
    assert detector == 'cleany/CalibratedCADFCL'
    assert json.loads(environment['CLEANY_CAD_LINKAGE_JSON']) == metadata
