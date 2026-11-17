"""Connect the verified CAD geometry and nonlinear closure to physical MoveIt."""
import xml.etree.ElementTree as ET

from cleany_description.cad_coordinates import ATTRIBUTE, CadCoordinates

from .model_semantics import restrict_semantics


def cad_runtime_configuration(description: str, semantic: str):
    cad = CadCoordinates.from_urdf(description)
    if cad is None:
        return None
    # Carry existing mechanical-interface permissions onto their CAD owners.
    # Compatibility aliases have no geometry, so retaining only their names
    # would silently lose the established adjacency permissions.
    aliases = {'base_link':'base_link', 'top_base_link':'base_link',
               'head_pan_link':'cad_head_pan', 'head_tilt_link':'cad_head_tilt'}
    for side in ('left','right'):
        for public, native in (('shoulder_yaw_link','shoulder_yaw'),
                               ('upper_arm_link','shoulder_pitch'),
                               ('lower_arm_link','elbow_pitch'),
                               ('wrist_pitch_link','wrist_pitch'),
                               ('gripper_frame','wrist_roll'),
                               ('moving_jaw_link','gripper_cam')):
            aliases[side+'_'+public] = 'cad_'+side+'_'+native
    source = ET.fromstring(semantic)
    permissions = []
    for item in source.findall('disable_collisions'):
        a,b = (aliases.get(item.get(key),item.get(key)) for key in ('link1','link2'))
        if a != b:
            permissions.append((a,b,item.get('reason','')))
    semantic, _ = restrict_semantics(description, semantic)
    root = ET.fromstring(semantic)
    links = {link.get('name') for link in ET.fromstring(description).findall('link')}
    existing_pairs = {frozenset((item.get('link1'),item.get('link2')))
                      for item in root.findall('disable_collisions')}
    for a,b,reason in permissions:
        if a in links and b in links and frozenset((a,b)) not in existing_pairs:
            ET.SubElement(root,'disable_collisions',link1=a,link2=b,reason=reason)
    for side in ('left','right'):
        a,b = (f'cad_{side}_gripper_tip_{finger}' for finger in ('L','R'))
        if a in links and b in links:
            # Replace the legacy fixed/moving-jaw contact permission with the
            # symmetric pair. Public motor bounds prohibit closing past this stop.
            ET.SubElement(root,'disable_collisions',link1=a,link2=b,
                          reason='Calibrated closed pad contact')
    existing = {item.get('name') for item in root.findall('passive_joint')}
    for name in cad.dependent_names:
        if name not in existing:
            ET.SubElement(root, 'passive_joint', name=name)
    return (ET.tostring(root, encoding='unicode'),
            {'CLEANY_CAD_LINKAGE_JSON': ET.fromstring(description).get(ATTRIBUTE)},
            'cleany/CalibratedCADFCL')
