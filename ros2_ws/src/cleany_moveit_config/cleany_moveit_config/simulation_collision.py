"""Explicit simulation-only SRDF overrides; keep URDF geometry for self filtering."""
import xml.etree.ElementTree as ET


def ignore_simulation_mast(urdf: str, srdf: str, *, use_sim_time: bool) -> str:
    if not use_sim_time:
        raise ValueError('Ignoring the mast requires simulation time')
    robot = ET.fromstring(urdf)
    semantic = ET.fromstring(srdf)
    names = {link.get('name') for link in robot.findall('link')}
    if 'top_base_link' not in names:
        raise ValueError('Simulation mast override requires top_base_link')
    existing = {frozenset((entry.get('link1'), entry.get('link2')))
                for entry in semantic.findall('disable_collisions')}
    for name in sorted(names - {'top_base_link'}):
        if frozenset(('top_base_link', name)) not in existing:
            ET.SubElement(semantic, 'disable_collisions', {
                'link1': 'top_base_link', 'link2': name,
                'reason': 'SimulationOnlyMastOverride',
            })
    return ET.tostring(semantic, encoding='unicode')
