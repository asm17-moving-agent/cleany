"""Deterministic, metre-scale tabletop geometry; no downloaded image/mesh assets."""
from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET


Vertex = tuple[float, float, float]
Triangle = tuple[int, int, int]
MeshData = tuple[list[Vertex], list[Triangle]]


def values(numbers: Iterable[float]) -> str:
    return ' '.join(f'{float(v):.10g}' for v in numbers)


@dataclass(frozen=True)
class TabletopShape:
    kind: str
    parameters: Mapping[str, float]

    def number(self, name: str) -> float:
        return self.parameters[name]


def parse_shape(kind: str, raw: object, collision: str, size: tuple[float, ...]) -> TabletopShape | None:
    if kind not in ('paper_cup', 'lego_brick', 'crumpled_tissue'):
        return None
    if not isinstance(raw, Mapping):
        raise ValueError(f'{kind} requires geometry parameters')
    keys = {
        'paper_cup': ('bottom_diameter_m', 'wall_thickness_m', 'segments'),
        'lego_brick': ('stud_pitch_m', 'stud_diameter_m', 'stud_height_m', 'studs_x', 'studs_y'),
        'crumpled_tissue': ('latitude_segments', 'longitude_segments', 'wrinkle_amplitude'),
    }[kind]
    try:
        parameters = {key: float(raw[key]) for key in keys}
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f'{kind} requires numeric geometry parameters: {keys}') from error
    if not all(math.isfinite(v) and v > 0 for v in parameters.values()):
        raise ValueError(f'{kind} geometry parameters must be finite and positive')
    shape = TabletopShape(kind, parameters)
    integer_keys = {'segments', 'studs_x', 'studs_y', 'latitude_segments', 'longitude_segments'} & parameters.keys()
    if any(not parameters[key].is_integer() or parameters[key] > 128 for key in integer_keys):
        raise ValueError('Geometry counts must be positive integers no greater than 128')
    if kind == 'paper_cup':
        bottom, wall = parameters['bottom_diameter_m'], parameters['wall_thickness_m']
        if collision != 'compound' or len(size) != 2 or not 2*wall < bottom < size[0] or wall >= size[1]:
            raise ValueError('Paper cup needs a hollow tapered compound collision')
        if parameters['segments'] < 12:
            raise ValueError('Paper cup needs at least 12 wall segments')
    elif kind == 'lego_brick':
        if collision != 'compound' or len(size) != 3:
            raise ValueError('LEGO brick needs body dimensions and compound collision')
        pitch, diameter = parameters['stud_pitch_m'], parameters['stud_diameter_m']
        if diameter >= pitch or any((parameters[f'studs_{axis}']-1)*pitch+diameter > size[i]
                                   for i, axis in enumerate(('x', 'y'))):
            raise ValueError('LEGO studs do not fit the brick body')
    elif (collision != 'mesh' or len(size) != 3 or parameters['wrinkle_amplitude'] >= .4
          or parameters['latitude_segments'] < 4 or parameters['longitude_segments'] < 8):
        raise ValueError('Tissue needs bounded wrinkles and a three-dimensional mesh')
    return shape


def _mesh(asset: ET.Element, name: str, vertices: Sequence[Vertex],
          faces: Sequence[Triangle]) -> None:
    ET.SubElement(asset, 'mesh', name=name,
        vertex=values(v for point in vertices for v in point),
        face=' '.join(str(i) for face in faces for i in face))


def cup_vertices(size: tuple[float, ...], shape: TabletopShape) -> MeshData:
    radius, height = size[0]/2, size[1]
    bottom, wall = shape.number('bottom_diameter_m')/2, shape.number('wall_thickness_m')
    count = int(shape.number('segments'))
    rings = ((bottom, 0.), (radius, height), (radius-wall, height), (bottom-wall, wall))
    vertices = [(r*math.cos(2*math.pi*i/count), r*math.sin(2*math.pi*i/count), z)
                for r, z in rings for i in range(count)]
    faces = []
    for ring in range(4):
        other = (ring+1) % 4
        for i in range(count):
            j = (i+1) % count
            a, b, c, d = ring*count+i, ring*count+j, other*count+j, other*count+i
            faces.extend(((a, b, c), (a, c, d)))
    return vertices, faces


def tissue_vertices(size: tuple[float, ...], shape: TabletopShape) -> MeshData:
    lat, lon = int(shape.number('latitude_segments')), int(shape.number('longitude_segments'))
    amplitude = shape.number('wrinkle_amplitude')
    vertices = [(0., 0., 1.)]
    for row in range(1, lat):
        theta = math.pi*row/lat
        for column in range(lon):
            phi = 2*math.pi*column/lon + .13*math.sin(row*2.1)
            r = 1 + amplitude*(.65*math.sin(7*phi+row*2.3)+.35*math.cos(11*phi-row*1.7))
            vertices.append((r*math.sin(theta)*math.cos(phi), r*math.sin(theta)*math.sin(phi),
                             r*math.cos(theta)))
    vertices.append((0., 0., -1.))
    low = [min(v[i] for v in vertices) for i in range(3)]
    high = [max(v[i] for v in vertices) for i in range(3)]
    vertices = [tuple((v[i]-low[i])/(high[i]-low[i])*size[i]-(size[i]/2 if i < 2 else 0.)
                      for i in range(3)) for v in vertices]
    faces = []
    for column in range(lon):
        faces.append((0, 1+column, 1+(column+1) % lon))
    for row in range(lat-2):
        for column in range(lon):
            a, b = 1+row*lon+column, 1+row*lon+(column+1) % lon
            faces.extend(((a, a+lon, b), (b, a+lon, b+lon)))
    for column in range(lon):
        faces.append((len(vertices)-1, 1+(lat-2)*lon+(column+1) % lon, 1+(lat-2)*lon+column))
    return vertices, faces


def add_shape_assets(asset: ET.Element, name: str, size: tuple[float, ...], shape: TabletopShape) -> None:
    if shape.kind == 'paper_cup':
        vertices, faces = cup_vertices(size, shape)
        _mesh(asset, f'{name}_mesh', vertices, faces)
        count = int(shape.number('segments'))
        for i in range(count):
            # Each convex wall panel is separate, preserving the open interior.
            panel = [vertices[ring*count+j] for ring in range(4) for j in (i, (i+1) % count)]
            _mesh(asset, f'{name}_wall_mesh_{i}', panel,
                ((0,1,3),(0,3,2),(2,3,5),(2,5,4),(4,5,7),(4,7,6),
                 (6,7,1),(6,1,0),(0,2,4),(0,4,6),(1,7,5),(1,5,3)))
    elif shape.kind == 'crumpled_tissue':
        _mesh(asset, f'{name}_mesh', *tissue_vertices(size, shape))


def add_shape_geoms(body: ET.Element, name: str, size: tuple[float, ...],
                    rgba: tuple[float, ...], mass: float, shape: TabletopShape) -> None:
    visual = dict(rgba=values(rgba), contype='0', conaffinity='0', density='0', group='2')
    collision = dict(rgba='0 0 0 0', contype='1', conaffinity='1', condim='6',
                     friction='1.5 0.08 0.02', group='3')
    if shape.kind == 'paper_cup':
        count = int(shape.number('segments'))
        wall, bottom = shape.number('wall_thickness_m'), shape.number('bottom_diameter_m')/2
        ET.SubElement(body, 'geom', name=f'{name}_visual', type='mesh', mesh=f'{name}_mesh', **visual)
        ET.SubElement(body, 'geom', name=f'{name}_bottom_visual', type='cylinder',
                      size=values((bottom, wall/2)), pos=values((0,0,wall/2)), **visual)
        ET.SubElement(body, 'geom', name=f'{name}_collision', type='cylinder',
                      size=values((bottom, wall/2)), pos=values((0,0,wall/2)), mass=str(mass*.2), **collision)
        for i in range(count):
            ET.SubElement(body, 'geom', name=f'{name}_wall_{i}_collision', type='mesh',
                          mesh=f'{name}_wall_mesh_{i}', mass=str(mass*.8/count), **collision)
    elif shape.kind == 'lego_brick':
        height, pitch = size[2], shape.number('stud_pitch_m')
        nx, ny = int(shape.number('studs_x')), int(shape.number('studs_y'))
        for suffix, attributes in (('visual', visual), ('collision', {**collision, 'mass': str(mass*.9)})):
            ET.SubElement(body, 'geom', name=f'{name}_{suffix}', type='box', size=values(v/2 for v in size),
                          pos=values((0,0,height/2)), **attributes)
        for i in range(nx):
            for j in range(ny):
                pos = values(((i-(nx-1)/2)*pitch, (j-(ny-1)/2)*pitch, height+shape.number('stud_height_m')/2))
                stud_size = values((shape.number('stud_diameter_m')/2, shape.number('stud_height_m')/2))
                for suffix, attributes in (('visual', visual), ('collision', {**collision, 'mass': str(mass*.1/(nx*ny))})):
                    ET.SubElement(body, 'geom', name=f'{name}_stud_{i}_{j}_{suffix}', type='cylinder',
                                  pos=pos, size=stud_size, **attributes)
    else:
        ET.SubElement(body, 'geom', name=f'{name}_visual', type='mesh', mesh=f'{name}_mesh', **visual)
        # MuJoCo uses the convex hull for contact; the rendered surface retains its folds.
        ET.SubElement(body, 'geom', name=f'{name}_collision', type='mesh', mesh=f'{name}_mesh',
                      mass=str(mass), **collision)
