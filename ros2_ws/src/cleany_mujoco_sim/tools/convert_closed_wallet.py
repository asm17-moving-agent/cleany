"""Print the normalized wallet OBJ from the pinned, licensed source GLB.

Usage: python3 tools/convert_closed_wallet.py /path/to/source.glb
Only material mat20 (brown wallet) is retained; bills/cards are omitted.
No files are overwritten by this utility.
"""
from pathlib import Path
import hashlib
import json
import struct
import sys

import numpy as np


def convert(path: Path) -> str:
    raw = path.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == 'bc58f3dd4b241162d1c11fdbe92e8ba0d8826eb1b042d5fd4a0f97e418fecd99'
    assert raw[:4] == b'glTF'
    length = struct.unpack_from('<I', raw, 12)[0]
    scene = json.loads(raw[20:20+length])
    binary = raw[28+length:]
    assert scene['nodes'] == [{'name': 'group771970811', 'mesh': 0}]

    def accessor(index: int) -> np.ndarray:
        a = scene['accessors'][index]
        view = scene['bufferViews'][a['bufferView']]
        assert 'sparse' not in a
        dtype = {5126: '<f4', 5123: '<u2'}[a['componentType']]
        width = {'VEC3': 3, 'SCALAR': 1}[a['type']]
        itemsize = np.dtype(dtype).itemsize
        return np.ndarray((a['count'], width), dtype=dtype, buffer=binary,
            offset=view.get('byteOffset', 0)+a.get('byteOffset', 0),
            strides=(view.get('byteStride', width*itemsize), itemsize))

    material = next(i for i, m in enumerate(scene['materials']) if m['name'] == 'mat20')
    primitives = [p for p in scene['meshes'][0]['primitives'] if p['material'] == material]
    assert len(primitives) == 1 and primitives[0]['mode'] == 4
    primitive = primitives[0]
    vertices = accessor(primitive['attributes']['POSITION']).astype(float)
    faces = accessor(primitive['indices']).reshape(-1, 3)
    # Align principal extent axes: long edge X, short edge Y, thickness Z.
    _, axes = np.linalg.eigh(np.cov(vertices.T))
    axes = axes[:, ::-1]
    if np.linalg.det(axes) < 0:
        axes[:, 2] *= -1
    vertices = vertices @ axes
    # The shell contains an interior spread. Clip at its spine and fold the
    # negative-X half through 180 degrees about Y, above the positive half.
    vertices[:, 0] -= (vertices[:, 0].min()+vertices[:, 0].max())/2
    hinge_z = (vertices[:, 2].min()+vertices[:, 2].max())/2
    folded_vertices, folded_faces = [], []
    for face in faces:
        for side in (-1, 1):
            polygon = []
            triangle = vertices[face]
            for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
                inside_a, inside_b = side*a[0] >= 0, side*b[0] >= 0
                if inside_a:
                    polygon.append(a.copy())
                if inside_a != inside_b:
                    polygon.append(a+(b-a)*(-a[0]/(b[0]-a[0])))
            if len(polygon) < 3:
                continue
            polygon = np.array(polygon)
            if side == -1:
                polygon[:, 0] *= -1
                polygon[:, 2] = 2*hinge_z-polygon[:, 2]
            first = len(folded_vertices)
            folded_vertices.extend(polygon)
            for i in range(1, len(polygon)-1):
                if np.linalg.norm(np.cross(polygon[i]-polygon[0], polygon[i+1]-polygon[0])) > 1e-12:
                    folded_faces.append((first, first+i, first+i+1))
    vertices, faces = np.array(folded_vertices), np.array(folded_faces)
    vertices -= vertices.min(axis=0)
    vertices *= np.array([.110, .085, .030]) / np.ptp(vertices, axis=0)
    vertices -= [.055, .0425, 0.]
    lines = ['# A Wallet by senior design, CC-BY-3.0',
             '# https://poly.pizza/m/898ojWnA41a',
             '# Brown shell only; bills/cards removed; folded 180deg at spine; 110x85x30mm.',
             '# Source SHA256: '+hashlib.sha256(raw).hexdigest()]
    lines.extend('v '+' '.join(f'{v:.10f}' for v in row) for row in vertices)
    lines.extend('f '+' '.join(str(int(v)+1) for v in row) for row in faces)
    return '\n'.join(lines)+'\n'


if __name__ == '__main__':
    print(convert(Path(sys.argv[1])), end='')
