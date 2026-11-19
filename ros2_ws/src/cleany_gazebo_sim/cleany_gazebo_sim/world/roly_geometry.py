"""Photo-reconstructed ROLY surfaces; dimensions come from the furniture config.

Smooth normals and UVs are kept in COLLADA, shared by all 48 static instances.
No product photograph, downloaded CAD, or Blender dependency is embedded.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
import hashlib
import json
import os
from math import cos, sin, pi, sqrt
from pathlib import Path
from xml.etree import ElementTree as ET


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def unit(a):
    length = sqrt(sum(x * x for x in a)) or 1.0
    return tuple(x / length for x in a)


@dataclass
class Mesh:
    vertices: list = field(default_factory=list)
    faces: list = field(default_factory=list)
    uv: list = field(default_factory=list)

    def grid(self, rows, cols, point, uv=None, wrap=False):
        start = len(self.vertices)
        for i in range(rows):
            for j in range(cols):
                self.vertices.append(point(i, j))
                self.uv.append(
                    uv(i, j) if uv else (j / max(cols - 1, 1), i / max(rows - 1, 1))
                )
        for i in range(rows - 1):
            for j in range(cols if wrap else cols - 1):
                a = start + i * cols + j
                b = start + i * cols + (j + 1) % cols
                self.faces.extend([(a, b, b + cols), (a, b + cols, a + cols)])

    def sweep(self, centers, widths, depths, sides=16, exponent=0.6):
        axes = []
        closed = sum(x * x for x in sub(centers[0], centers[-1])) < 1e-12
        for i in range(len(centers)):
            tangent = (
                unit(
                    sub(
                        centers[(i + 1) % (len(centers) - 1)],
                        centers[(i - 1) % (len(centers) - 1)],
                    )
                )
                if closed
                else unit(
                    sub(centers[min(i + 1, len(centers) - 1)], centers[max(i - 1, 0)])
                )
            )
            if axes:
                previous = axes[-1][0]
                dot = sum(a * b for a, b in zip(previous, tangent))
                first = unit(tuple(previous[k] - dot * tangent[k] for k in range(3)))
            else:
                guide = (0, 1, 0) if abs(tangent[1]) < 0.9 else (1, 0, 0)
                first = unit(cross(tangent, guide))
            second = unit(cross(tangent, first))
            axes.append((first, second))

        def point(i, j):
            angle = 2 * pi * j / sides
            # Rounded rectangular injection-moulded section, not round tubing.
            a = (1 if cos(angle) >= 0 else -1) * abs(cos(angle)) ** exponent * depths[i]
            b = (1 if sin(angle) >= 0 else -1) * abs(sin(angle)) ** exponent * widths[i]
            return tuple(
                centers[i][k] + a * axes[i][0][k] + b * axes[i][1][k] for k in range(3)
            )

        start = len(self.vertices)
        self.grid(len(centers), sides, point, wrap=True)
        self.faces += [(start, start + j + 1, start + j) for j in range(1, sides - 1)]
        end = start + (len(centers) - 1) * sides
        self.faces += [(end, end + j, end + j + 1) for j in range(1, sides - 1)]

    def cushion(self, center, size, rotate=0, fabric=False):
        # Concentric crown rings give the cushion a soft waterfall front edge.
        rings = [
            (-0.5, 0.06),
            (-0.47, 0.85),
            (-0.35, 0.98),
            (-0.08, 1),
            (0.23, 0.98),
            (0.42, 0.89),
            (0.49, 0.68),
            (0.50, 0.35),
            (0.50, 0.02),
        ]

        def point(i, j):
            z, r = rings[i]
            t = 2 * pi * j / 64
            x = (1 if cos(t) >= 0 else -1) * abs(cos(t)) ** 0.42 * r * size[0] / 2
            y = (1 if sin(t) >= 0 else -1) * abs(sin(t)) ** 0.42 * r * size[1] / 2
            if fabric:
                # Gently dished sitting surface and a slightly lower front lip.
                z -= 0.10 * (1 - r * r) if i > 4 else 0
                z -= 0.09 * max(x / (size[0] / 2), 0) if i > 3 else 0
            return (
                center[0] + x * cos(rotate) - y * sin(rotate),
                center[1] + x * sin(rotate) + y * cos(rotate),
                center[2] + z * size[2],
            )

        start = len(self.vertices)
        self.grid(
            len(rings),
            64,
            point,
            lambda i, j: (
                (point(i, j)[0] - center[0]) / size[0] * 2,
                (point(i, j)[1] - center[1]) / size[1] * 2,
            ),
            wrap=True,
        )
        end = start + (len(rings) - 1) * 64
        self.faces += [(start, start + j + 1, start + j) for j in range(1, 63)]
        self.faces += [(end, end + j, end + j + 1) for j in range(1, 63)]

    def cylinder(self, center, radius, length, axis=(0, 0, 1)):
        radii = [0.85, 1, 1, 0.85]
        centers = [
            tuple(center[k] + axis[k] * length * t for k in range(3))
            for t in (-0.5, -0.4, 0.4, 0.5)
        ]
        self.sweep(
            centers,
            [radius * r for r in radii],
            [radius * r for r in radii],
            32,
            exponent=1.0,
        )

    def write(self, path):
        faces = [
            f
            for f in self.faces
            if sum(
                x * x
                for x in cross(
                    sub(self.vertices[f[1]], self.vertices[f[0]]),
                    sub(self.vertices[f[2]], self.vertices[f[0]]),
                )
            )
            > 1e-20
        ]
        normals = [[0.0, 0.0, 0.0] for _ in self.vertices]
        for face in faces:
            a, b, c = [self.vertices[i] for i in face]
            n = cross(sub(b, a), sub(c, a))
            for i in face:
                for k in range(3):
                    normals[i][k] += n[k]
        normals = [unit(n) for n in normals]
        root = ET.Element(
            'COLLADA',
            xmlns='http://www.collada.org/2005/11/COLLADASchema',
            version='1.4.1',
        )
        asset = ET.SubElement(root, 'asset')
        ET.SubElement(asset, 'unit', name='meter', meter='1')
        ET.SubElement(asset, 'up_axis').text = 'Z_UP'
        mesh = ET.SubElement(
            ET.SubElement(
                ET.SubElement(root, 'library_geometries'), 'geometry', id='shape'
            ),
            'mesh',
        )
        for name, values, params in [
            ('positions', self.vertices, 'XYZ'),
            ('normals', normals, 'XYZ'),
            ('uv', self.uv, 'ST'),
        ]:
            source = ET.SubElement(mesh, 'source', id=name)
            ET.SubElement(
                source,
                'float_array',
                id=name + '-array',
                count=str(len(values) * len(params)),
            ).text = ' '.join(f'{x:.7g}' for row in values for x in row)
            acc = ET.SubElement(
                ET.SubElement(source, 'technique_common'),
                'accessor',
                source='#' + name + '-array',
                count=str(len(values)),
                stride=str(len(params)),
            )
            for param in params:
                ET.SubElement(acc, 'param', name=param, type='float')
        ET.SubElement(
            ET.SubElement(mesh, 'vertices', id='vertices'),
            'input',
            semantic='POSITION',
            source='#positions',
        )
        triangles = ET.SubElement(mesh, 'triangles', count=str(len(faces)))
        for offset, (semantic, source) in enumerate(
            [('VERTEX', 'vertices'), ('NORMAL', 'normals'), ('TEXCOORD', 'uv')]
        ):
            ET.SubElement(
                triangles,
                'input',
                semantic=semantic,
                source='#' + source,
                offset=str(offset),
            )
        ET.SubElement(triangles, 'p').text = ' '.join(
            f'{i} {i} {i}' for f in faces for i in f
        )
        scene = ET.SubElement(
            ET.SubElement(root, 'library_visual_scenes'), 'visual_scene', id='Scene'
        )
        ET.SubElement(
            ET.SubElement(scene, 'node', id='object'), 'instance_geometry', url='#shape'
        )
        ET.SubElement(
            ET.SubElement(root, 'scene'), 'instance_visual_scene', url='#Scene'
        )
        temporary = path.with_suffix(f'.{os.getpid()}.tmp')
        ET.ElementTree(root).write(temporary, encoding='utf-8', xml_declaration=True)
        temporary.replace(path)


def curve(points, steps=6):
    result = []
    padded = [points[0], *points, points[-1]]
    for i in range(1, len(padded) - 2):
        a, b, c, d = padded[i - 1 : i + 3]
        for j in range(steps):
            t = j / steps
            result.append(
                tuple(
                    0.5
                    * (
                        (2 * b[k])
                        + (-a[k] + c[k]) * t
                        + (2 * a[k] - 5 * b[k] + 4 * c[k] - d[k]) * t * t
                        + (-a[k] + 3 * b[k] - 3 * c[k] + d[k]) * t * t * t
                    )
                    for k in range(3)
                )
            )
    return [*result, points[-1]]


def back_point(config, t, u):
    bx, by, bz = config['back_origin_m']
    depth, width, height = config['back_size_m']
    half = width * (0.5 - 0.05 * sin(pi * t) + 0.005 * t)
    x = bx + depth * (0.58 * sin(pi * t) - 0.72 * t + 0.27 * u * u)
    z = bz + height * t - 0.014 * abs(u) ** 10 * t**8
    return (x, by + u * half, z)


@lru_cache(maxsize=8)
def make_meshes(config_json: str):
    c = json.loads(config_json)
    meshes = {
        key: Mesh() for key in ('frame', 'seat', 'mesh', 'metal', 'tire', 'lumbar')
    }
    frame, seat, back, metal, tire, lumbar = [meshes[k] for k in meshes]
    depth, width, thickness = c['seat_size_m']
    top = c['seat_top_m']
    bottom = top - thickness
    seat.cushion(
        (0.025, 0, top - thickness / 2), (depth, width, thickness), fabric=True
    )
    frame.cushion((0.015, 0, bottom - 0.008), (depth * 0.98, width * 0.98, 0.025))
    back.grid(
        33,
        33,
        lambda i, j: back_point(c, i / 32, 2 * j / 32 - 1),
        lambda i, j: (j / 32 * 3, i / 32 * 3),
    )
    # Reverse face gives the rear an explicit normal without relying on renderer
    # double-sided defaults. A 2 mm thickness preserves the stretched textile.
    back.grid(
        33,
        33,
        lambda i, j: (lambda p: (p[0] - 0.002, p[1], p[2]))(
            back_point(c, i / 32, 1 - 2 * j / 32)
        ),
        lambda i, j: ((1 - j / 32) * 3, i / 32 * 3),
    )
    outline = (
        [back_point(c, i / 40, -1) for i in range(41)]
        + [back_point(c, 1, -1 + 2 * j / 32) for j in range(1, 33)]
        + [back_point(c, 1 - i / 40, 1) for i in range(1, 41)]
        + [back_point(c, 0, 1 - 2 * j / 32) for j in range(1, 33)]
    )
    frame.sweep(outline, [0.009] * len(outline), [0.012] * len(outline), 16)
    # Rear lumbar bridge and softly rounded pad, visible from behind.
    lumbar_path = curve(
        [(-0.265, -0.19, 0.61), (-0.245, 0, 0.605), (-0.265, 0.19, 0.61)]
    )
    frame.sweep(lumbar_path, [0.014] * len(lumbar_path), [0.016] * len(lumbar_path))
    pad = Mesh()
    pad.cushion((0, 0, 0), (0.10, 0.33, 0.032))
    start = len(lumbar.vertices)
    lumbar.vertices.extend((-0.263 - z, y, 0.604 + x) for x, y, z in pad.vertices)
    lumbar.faces.extend(tuple(start + i for i in face) for face in pad.faces)
    lumbar.uv.extend(pad.uv)
    # Continuous side cradle joins front seat rail to the back and fixed arm.
    for side in (-1, 1):
        y = side * c['arm_half_width_m']
        cradle = curve(
            [
                (0.19, side * 0.19, bottom - 0.008),
                (0.09, side * 0.16, bottom - 0.082),
                (-0.035, side * 0.09, bottom - 0.112),
                (-0.21, side * 0.18, bottom - 0.066),
                (-0.28, side * 0.245, bottom + 0.004),
                (-0.225, y, c['arm_top_m'] - 0.115),
            ]
        )
        frame.sweep(cradle, [0.015] * len(cradle), [0.022] * len(cradle))
        # Moulded arm stem widens into the fixed, shallow saddle pad.
        stem = curve(
            [
                (-0.225, y, c['arm_top_m'] - 0.12),
                (-0.205, y, c['arm_top_m'] - 0.074),
                (-0.18, y, c['arm_top_m'] - 0.028),
            ]
        )
        widths = [0.014 + 0.022 * (i / (len(stem) - 1)) ** 3 for i in range(len(stem))]
        frame.sweep(stem, widths, [0.018] * len(stem))
        frame.cushion(
            (-0.13, y, c['arm_top_m'] - 0.013),
            (c['arm_length_m'], c['arm_pad_width_m'], c['arm_pad_thickness_m']),
        )
        # Rear connection and small adjustment paddle below the seat.
        metal.cushion((0.08, side * 0.19, bottom - 0.028), (0.085, 0.043, 0.018))
    rear_bridge = curve(
        [
            back_point(c, 0, -1),
            (-0.255, -0.21, bottom + 0.01),
            (-0.285, -0.12, bottom - 0.025),
            (-0.29, 0, bottom - 0.03),
            (-0.285, 0.12, bottom - 0.025),
            (-0.255, 0.21, bottom + 0.01),
            back_point(c, 0, 1),
        ]
    )
    frame.sweep(rear_bridge, [0.019] * len(rear_bridge), [0.022] * len(rear_bridge))
    frame.cushion((-0.045, 0, bottom - 0.094), (0.22, 0.17, 0.09))
    # Exposed underside ribs of the tilt mechanism.
    for y in (-0.075, -0.045, -0.015, 0.015, 0.045, 0.075):
        frame.sweep(
            [(0.01, y, bottom - 0.083), (0.08, y, bottom - 0.032)],
            [0.006] * 2,
            [0.012] * 2,
            8,
        )
    hub, tip, radius = (
        c['base_hub_height_m'],
        c['base_tip_height_m'],
        c['base_radius_m'],
    )
    metal.cylinder(
        (0, 0, (hub + bottom - 0.13) / 2), c['column_radius_m'], bottom - 0.13 - hub
    )
    frame.cylinder((0, 0, 0.12), c['column_radius_m'] * 1.45, 0.12)
    for i in range(5):
        a = 2 * pi * i / 5
        co, si = cos(a), sin(a)
        points = curve(
            [
                (0, 0, hub),
                (0.12 * co, 0.12 * si, hub - 0.026),
                (0.28 * co, 0.28 * si, tip + 0.015),
                (radius * co, radius * si, tip),
            ]
        )
        frame.sweep(
            points,
            [0.034 - 0.021 * j / (len(points) - 1) for j in range(len(points))],
            [0.023 - 0.009 * j / (len(points) - 1) for j in range(len(points))],
        )
        wheel = c['caster_radius_m']
        x, y = radius * co, radius * si
        frame.cylinder((x, y, (tip + wheel) / 2), 0.010, tip - wheel)
        axis = (-si, co, 0)
        for side in (-1, 1):
            off = side * (c['caster_width_m'] + c['caster_gap_m']) / 2
            center = (x + axis[0] * off, y + axis[1] * off, wheel)
            tire.cylinder(center, wheel, c['caster_width_m'], axis)
            cap = tuple(
                center[k] + axis[k] * side * c['caster_width_m'] * 0.46
                for k in range(3)
            )
            frame.cylinder(cap, wheel * 0.74, 0.002, axis)
    return meshes


def add_visuals(link, config, output: Path):
    output.mkdir(parents=True, exist_ok=True)
    config_json = json.dumps(config.values, sort_keys=True)
    # Source hash prevents stale shapes after changes to the reconstruction.
    digest = hashlib.sha256(
        config_json.encode() + Path(__file__).read_bytes()
    ).hexdigest()[:16]
    textures = config.mesh_dir.parents[1] / 'materials/roly'
    for name, mesh in make_meshes(config_json).items():
        asset = output / f'roly-{digest}-{name}.dae'
        if not asset.exists():
            mesh.write(asset)
        visual = ET.SubElement(link, 'visual', name=f'roly_{name}')
        ET.SubElement(visual, 'visibility_flags').text = '0x01'
        ET.SubElement(
            ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh'), 'uri'
        ).text = asset.resolve().as_uri()
        material = ET.SubElement(visual, 'material')
        color = config.color('mesh' if name == 'lumbar' else name)
        ET.SubElement(material, 'ambient').text = color
        ET.SubElement(material, 'diffuse').text = color
        ET.SubElement(material, 'specular').text = '.08 .08 .08 1'
        metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
        ET.SubElement(metal, 'roughness').text = (
            '.92' if name in ('seat', 'mesh', 'lumbar') else '.52'
        )
        ET.SubElement(metal, 'metalness').text = '.35' if name == 'metal' else '0'
        if name in ('seat', 'mesh'):
            ET.SubElement(metal, 'albedo_map').text = (
                (textures / ('back.png' if name == 'mesh' else 'seat.png'))
                .resolve()
                .as_uri()
            )
