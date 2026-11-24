"""Materialize offline collision fits on their original rigid URDF link."""
import hashlib
import json
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET


ASSET_DIR = 'meshes/cad_runtime_collision'


def member_digest(members: list[ET.Element], repository: Path) -> str:
    """Bind a fit to its input meshes, scales and poses, independent of XML whitespace."""
    records = []
    for item in members:
        mesh = item.find('geometry/mesh')
        if mesh is None:
            raise ValueError('A rigid fit must identify mesh collision members')
        origin = item.find('origin')
        uri = mesh.attrib['filename']
        prefix = 'package://cleany_description/'
        if not uri.startswith(prefix):
            raise ValueError(f'Unexpected collision input URI: {uri}')
        relative = Path(uri.removeprefix(prefix))
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError(f'Invalid collision asset path: {relative}')
        record = [item.attrib['name'], dict(mesh.attrib), dict(origin.attrib) if origin is not None else {}]
        if not re.fullmatch(r'[0-9a-f]{64}\.glb', relative.name):
            asset = repository / 'ros2_ws/src/cleany_description' / relative
            record.append(hashlib.sha256(asset.read_bytes()).hexdigest())
        records.append(record)
    payload = json.dumps(sorted(records, key=lambda row: row[0]), sort_keys=True, separators=(',', ':'))
    return hashlib.sha256(payload.encode()).hexdigest()


def _numbers(value: list, count: int, positive: bool = False) -> str:
    if not isinstance(value, list) or len(value) != count or not all(
        type(x) in (float, int) and math.isfinite(x) and (not positive or x > 0) for x in value
    ):
        raise ValueError('Invalid fitted collision shape parameters')
    return ' '.join(format(x, '.17g') for x in value)


def apply_rigid_collision_groups(tree: ET.Element, groups: list, repository: Path) -> set[Path]:
    """Replace fitted geometry on one link, preserving its original collision scope."""
    links = {link.attrib['name']: link for link in tree.findall('link')}
    assets = set()
    for entry in groups:
        link = links[entry['link']]
        shapes = {shape.attrib['name']: shape for shape in link.findall('collision')}
        names = entry['members']
        if not names or len(names) != len(set(names)) or any(n not in shapes for n in names):
            raise ValueError('Rigid fit members must share the same link and collision scope')
        scopes = {next((p for p in ('cad_self__', 'cad_world__') if n.startswith(p)), '')
                  for n in names}
        if len(scopes) != 1:
            raise ValueError('Rigid fit mixes collision scopes')
        scope = scopes.pop()
        members = [shapes[n] for n in names]
        if member_digest(members, repository) != entry['source_sha256'] or not entry['shapes']:
            raise ValueError('Stale or empty rigid collision fit')
        replacements = []
        for i, body in enumerate(entry['shapes']):
            kind = body['type']
            prefix = 'cad_enclosure__' if kind == 'hull' else ''
            name = f"{scope}{prefix}rigid_fit_{entry['name']}_{i}"
            if name in shapes:
                raise ValueError(f'Duplicate fitted collision name: {name}')
            element = ET.Element('collision', name=name)
            ET.SubElement(element, 'origin', xyz=_numbers(body['xyz'], 3), rpy=_numbers(body['rpy'], 3))
            geometry = ET.SubElement(element, 'geometry')
            if kind == 'box':
                ET.SubElement(geometry, 'box', size=_numbers(body['size'], 3, True))
            elif kind == 'cylinder':
                ET.SubElement(geometry, 'cylinder', radius=_numbers([body['radius']], 1, True),
                              length=_numbers([body['length']], 1, True))
            elif kind in ('hull', 'surface'):
                digest = body['sha256']
                if not re.fullmatch(r'[0-9a-f]{64}', digest):
                    raise ValueError('Invalid fitted mesh digest')
                relative = f'{ASSET_DIR}/{digest}.glb'
                path = repository / 'ros2_ws/src/cleany_description' / relative
                if path not in assets and hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                    raise ValueError(f'Fitted collision mesh digest mismatch: {path}')
                assets.add(path)
                ET.SubElement(geometry, 'mesh', filename=f'package://cleany_description/{relative}')
            else:
                raise ValueError(f'Unsupported rigid collision shape: {kind}')
            replacements.append(element)
        index = min(list(link).index(member) for member in members)
        for member in members:
            link.remove(member)
        for offset, element in enumerate(replacements):
            link.insert(index + offset, element)
    return assets
