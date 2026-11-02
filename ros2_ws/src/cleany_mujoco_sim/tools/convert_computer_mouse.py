"""Convert the pinned CreativeTrio CC0 mouse GLB to MuJoCo OBJ/PNG assets.

Usage: python3 tools/convert_computer_mouse.py source.glb output_directory
Requires NumPy; refuses to overwrite existing assets.
"""
from pathlib import Path
import hashlib
import json
import struct
import sys

import numpy as np


def convert(source: Path, destination: Path) -> None:
    raw = source.read_bytes()
    if hashlib.sha256(raw).hexdigest() != '75ed200f645b00c86ab4580aecf9d8c72af924e306c4c987bb8426e6d043b8ed':
        raise ValueError('Unexpected source GLB')
    length = struct.unpack_from('<I', raw, 12)[0]
    scene = json.loads(raw[20:20+length])
    binary = raw[28+length:]

    def accessor(index: int) -> np.ndarray:
        a = scene['accessors'][index]
        v = scene['bufferViews'][a['bufferView']]
        dtype = {5126: '<f4', 5123: '<u2'}[a['componentType']]
        width = {'VEC3': 3, 'VEC2': 2, 'SCALAR': 1}[a['type']]
        size = np.dtype(dtype).itemsize
        return np.ndarray((a['count'], width), dtype=dtype, buffer=binary,
            offset=v.get('byteOffset', 0)+a.get('byteOffset', 0),
            strides=(v.get('byteStride', width*size), size))

    primitive = scene['meshes'][0]['primitives'][0]
    # Source is Y-up, long axis Z. Cyclic permutation retains winding.
    vertices = accessor(primitive['attributes']['POSITION'])[:, [2, 0, 1]].astype(float)
    vertices -= vertices.min(axis=0)
    vertices *= np.array([.110, .065, .035]) / np.ptp(vertices, axis=0)
    vertices -= [.055, .0325, 0.]
    uv = accessor(primitive['attributes']['TEXCOORD_0']).copy()
    uv[:, 1] = 1-uv[:, 1]
    faces = accessor(primitive['indices']).reshape(-1, 3)
    lines = ['# Computer Mouse by CreativeTrio, CC0', '# https://poly.pizza/m/V2Ebx3pvo4',
             '# Y-up to Z-up; normalized to 110x65x35mm; original UV texture retained.']
    lines += ['v '+' '.join(f'{v:.10f}' for v in row) for row in vertices]
    lines += ['vt '+' '.join(f'{v:.10f}' for v in row) for row in uv]
    lines += ['f '+' '.join(f'{int(v)+1}/{int(v)+1}' for v in row) for row in faces]
    image = scene['images'][0]
    if image['mimeType'] != 'image/png':
        raise ValueError('Expected PNG texture')
    view = scene['bufferViews'][image['bufferView']]
    texture = binary[view.get('byteOffset', 0):view.get('byteOffset', 0)+view['byteLength']]
    obj_path = destination / 'study_cafe_mouse.obj'
    texture_path = destination / 'study_cafe_mouse.png'
    if obj_path.exists() or texture_path.exists():
        raise FileExistsError('Refusing to overwrite mouse assets')
    obj_path.write_text('\n'.join(lines)+'\n', encoding='utf-8')
    texture_path.write_bytes(texture)


if __name__ == '__main__':
    convert(Path(sys.argv[1]), Path(sys.argv[2]))
