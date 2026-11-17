"""Keep planning groups consistent with a physically reduced robot model."""

import xml.etree.ElementTree as ET


def restrict_semantics(description: str, semantic: str) -> tuple[str, set[str]]:
    model = ET.fromstring(description)
    links = {link.get('name') for link in model.findall('link')}
    joints = {joint.get('name') for joint in model.findall('joint')}
    root = ET.fromstring(semantic)
    removed = set()
    while True:
        previous = len(removed)
        for group in list(root.findall('group')):
            invalid = any(chain.get('base_link') not in links or chain.get('tip_link') not in links
                          for chain in group.findall('chain'))
            invalid |= any(j.get('name') not in joints for j in group.findall('joint'))
            invalid |= any(g.get('name') in removed for g in group.findall('group'))
            if invalid:
                removed.add(group.get('name'))
                root.remove(group)
        if len(removed) == previous:
            break
    groups = {group.get('name') for group in root.findall('group')}
    for item in list(root):
        if (item.tag == 'group_state' and item.get('group') not in groups
                or item.tag == 'disable_collisions'
                and (item.get('link1') not in links or item.get('link2') not in links)):
            root.remove(item)
    return ET.tostring(root, encoding='unicode'), groups
