"""Opt-in grouping of simple, immutable static geometry in a temporary world."""
from copy import deepcopy
from math import atan2, cos, hypot, isfinite, sin
from pathlib import Path
from tempfile import mkdtemp
from xml.etree import ElementTree as ET

import numpy as np

from .sdf import write_world

GROUP_NAME = 'cleany_static_world_group'


def _matrix(pose: ET.Element | None) -> np.ndarray:
    values = [float(v) for v in pose.text.split()] if pose is not None and pose.text and pose.text.strip() else [0.] * 6
    if len(values) != 6 or not all(isfinite(v) for v in values):
        raise ValueError('static scene requires finite six-component poses')
    x, y, z, r, p, a = values
    cr, sr, cp, sp, ca, sa = cos(r), sin(r), cos(p), sin(p), cos(a), sin(a)
    result = np.eye(4)
    result[:3, 3] = [x, y, z]
    result[:3, :3] = [[ca*cp, ca*sp*sr-sa*cr, ca*sp*cr+sa*sr],
                      [sa*cp, sa*sp*sr+ca*cr, sa*sp*cr-ca*sr],
                      [-sp, cp*sr, cp*cr]]
    return result


def _pose_text(matrix: np.ndarray) -> str:
    horizontal = hypot(matrix[0, 0], matrix[1, 0])
    pitch = atan2(-matrix[2, 0], horizontal)
    if horizontal > 1e-12:
        roll, yaw = atan2(matrix[2, 1], matrix[2, 2]), atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll, yaw = 0., atan2(-matrix[0, 1], matrix[1, 1])
    return ' '.join(format(float(v), '.17g') for v in [*matrix[:3, 3], roll, pitch, yaw])


def materialize_static_scene(source: Path) -> Path:
    """Keep shapes and world poses; group only unreferenced, plain static models.

    Model/link identities change in this opt-in copy. Stateful entities, explicit
    frame references and directional friction remain in their original models.
    """
    tree = ET.parse(source)
    world = tree.find('./world')
    if world is None:
        raise ValueError('static scene requires an SDF world')
    if any(model.get('name') == GROUP_NAME for model in world.findall('model')):
        raise ValueError('reserved static scene group name already exists')
    references = set()
    for element in world.iter():
        for key, value in element.attrib.items():
            if key in {'relative_to', 'attached_to'} or key.endswith('expressed_in'):
                references.add(value.split('::')[0])
        if element.tag in {'parent', 'child'} and element.text:
            references.add(element.text.strip().split('::')[0])
    group = ET.Element('model', name=GROUP_NAME)
    ET.SubElement(group, 'static').text = 'true'
    destination = ET.SubElement(group, 'link', name='geometry')
    count = 0
    for model in world.findall('model'):
        if model.findtext('static', '').strip().lower() not in {'true', '1'}:
            continue
        links = model.findall('link')
        if (len(links) != 1 or model.get('name') in references
                or links[0].get('name') in references
                or set(model.attrib) - {'name'}
                or set(links[0].attrib) - {'name'}):
            continue
        link = links[0]
        if any(child.tag not in {'static', 'pose', 'link'} for child in model):
            continue
        if any(child.tag not in {'pose', 'inertial', 'collision', 'visual'} for child in link):
            continue
        if any(model.findall('.//' + tag) for tag in ['plugin', 'sensor', 'joint', 'include', 'model', 'frame', 'fdir1']):
            continue
        # Without an explicit direction, unequal friction coefficients still
        # depend on the backend's link-frame choice; leave that model intact.
        if any(friction.findtext('mu', '1').strip() != friction.findtext('mu2', '1').strip()
               for friction in model.findall('.//surface/friction/ode')):
            continue
        if any(pose.attrib for pose in model.findall('.//pose')):
            continue
        geometry = [child for child in link if child.tag in {'collision', 'visual'}]
        if not geometry:
            continue
        prefix = _matrix(model.find('pose')) @ _matrix(link.find('pose'))
        for index, element in enumerate(geometry):
            copied = deepcopy(element)
            copied.set('name', f'static_{count}_{index}_{element.get("name", element.tag)}')
            full = prefix @ _matrix(element.find('pose'))
            pose = copied.find('pose')
            if pose is None:
                pose = ET.SubElement(copied, 'pose')
            pose.text = _pose_text(full)
            if not np.allclose(_matrix(pose), full, atol=1e-12, rtol=0):
                raise ValueError('static scene pose cannot be represented faithfully')
            destination.append(copied)
        world.remove(model)
        count += 1
    if not count:
        return source
    world.append(group)
    target = Path(mkdtemp(prefix='cleany-static-scene-')) / source.name
    write_world(tree, target)
    return target
