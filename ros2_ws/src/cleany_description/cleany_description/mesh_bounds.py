"""Bounds for the verified binary STL and identity-node GLB CAD assets."""
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


def mesh_bounds(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = path.read_bytes()
    if path.suffix.lower() == '.glb':
        magic, version, length = struct.unpack_from('<III', data)
        size, kind = struct.unpack_from('<II', data, 12)
        if magic != 0x46546C67 or version != 2 or length != len(data) or kind != 0x4E4F534A:
            raise ValueError('Invalid binary glTF CAD asset')
        scene = json.loads(data[20:20+size])
        if any(any(k in node for k in ('matrix', 'translation', 'rotation', 'scale'))
               for node in scene.get('nodes', [])):
            raise ValueError('CAD mesh bounds require identity GLB nodes')
        accessors = [scene['accessors'][p['attributes']['POSITION']]
                     for m in scene['meshes'] for p in m['primitives']]
        lower = np.min([a['min'] for a in accessors], axis=0)
        upper = np.max([a['max'] for a in accessors], axis=0)
    elif path.suffix.lower() == '.dae':
        root = ET.fromstring(data)
        ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
        unit = root.find('c:asset/c:unit', ns)
        scale = float(unit.get('meter', '1')) if unit is not None else 1.
        arrays = []
        for mesh in root.findall('.//c:geometry/c:mesh', ns):
            for position in mesh.findall('c:vertices/c:input[@semantic="POSITION"]', ns):
                source = mesh.find('c:source[@id="'+position.get('source').removeprefix('#')+'"]', ns)
                arrays.append(np.fromstring(source.find('c:float_array', ns).text, sep=' ').reshape(-1,3))
        vertices = np.concatenate(arrays)*scale
        lower, upper = vertices.min(axis=0), vertices.max(axis=0)
    else:
        count = int.from_bytes(data[80:84], 'little')
        if len(data) != 84+50*count or count == 0:
            raise ValueError('Expected a nonempty binary STL CAD asset')
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3,3)), ('attribute', '<u2')])
        vertices = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1,3)
        lower, upper = vertices.min(axis=0), vertices.max(axis=0)
    if lower.shape != (3,) or not np.isfinite([lower, upper]).all() or np.any(lower > upper):
        raise ValueError('Invalid CAD bounds')
    return lower, upper
