"""Materialize open collection bins in a dedicated sorting simulation.

The base-relative locations describe the configured collection fixture, not
perception ground truth for the items being sorted. No target welds exist.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


@dataclass(frozen=True)
class BinGeometry:
    name: str
    label: str
    center_xy: tuple[float, float]
    bottom_z: float
    outside_size: tuple[float, float, float]
    wall: float
    rgba: tuple[float, float, float, float]
    kind: str = 'bin'
    rim_rgba: tuple[float, float, float, float] | None = None

    @property
    def top_z(self) -> float:
        return self.bottom_z + self.outside_size[2]

    def boxes(self):
        """Five collision boxes (full extents, base-frame centers), no lid."""
        x, y = self.center_xy
        sx, sy, sz = self.outside_size
        t = self.wall
        yield 'floor', (sx, sy, t), (x, y, self.bottom_z + t / 2)
        for sign, suffix in ((-1, 'negative'), (1, 'positive')):
            yield f'x_{suffix}', (t, sy, sz), (
                x + sign * (sx - t) / 2, y, self.bottom_z + sz / 2,
            )
            yield f'y_{suffix}', (sx - 2 * t, t, sz), (
                x, y + sign * (sy - t) / 2, self.bottom_z + sz / 2,
            )

    def contains(self, point, *, margin: float = 0.0) -> bool:
        x, y, z = point
        sx, sy, _ = self.outside_size
        return (
            all(math.isfinite(v) for v in point)
            and abs(x - self.center_xy[0]) < sx / 2 - self.wall - margin
            and abs(y - self.center_xy[1]) < sy / 2 - self.wall - margin
            and self.bottom_z + self.wall <= z < self.top_z
        )


def load_bins(path: str | Path) -> tuple[BinGeometry, ...]:
    raw = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if raw.get('schema_version') != 1 or raw.get('frame_id') != 'base_link':
        raise ValueError('Bins require schema_version=1 and base_link frame')
    if 'rear_shelf' in raw or 'staging_area' in raw:
        raise ValueError('Unsupported collection fixture; use chassis bins and internal_tray')
    bins = []
    items = list(raw['bins'])
    if len(items) != 2:
        raise ValueError('Two distinct collection bins are required')
    for item in items:
        geometry = BinGeometry(
            name=item['id'], label=item['label'],
            center_xy=tuple(float(v) for v in item['center_xy_m']),
            bottom_z=float(item['bottom_z_m']),
            outside_size=tuple(float(v) for v in item['outside_size_m']),
            wall=float(item['wall_thickness_m']),
            rgba=tuple(float(v) for v in item['rgba']),
            kind=str(item.get('kind', 'bin')),
            rim_rgba=(tuple(float(v) for v in item['rim_rgba'])
                      if 'rim_rgba' in item else None),
        )
        if (len(geometry.center_xy) != 2 or len(geometry.outside_size) != 3
                or len(geometry.rgba) != 4 or not geometry.name.isidentifier()
                or not all(math.isfinite(v) for v in (
                    *geometry.center_xy, geometry.bottom_z,
                    *geometry.outside_size, geometry.wall, *geometry.rgba))
                or geometry.kind != 'bin'
                or geometry.wall <= 0.0
                or min(geometry.outside_size) <= 2 * geometry.wall
                or not all(0 <= v <= 1 for v in geometry.rgba)):
            raise ValueError('Invalid collection bin geometry')
        if geometry.rim_rgba is not None and (
                geometry.kind != 'bin' or len(geometry.rim_rgba) != 4
                or not all(math.isfinite(v) and 0 <= v <= 1 for v in geometry.rim_rgba)):
            raise ValueError('Invalid collection bin rim color')
        bins.append(geometry)
    if len({b.name for b in bins}) != len(items):
        raise ValueError('Distinct collection identifiers are required')
    return tuple(bins)


def add_sorting_bins(model_text: str, config: str | Path) -> str:
    root = ET.fromstring(model_text)
    chassis = root.find("./worldbody/body[@name='chassis']")
    if chassis is None:
        raise ValueError('Sorting fixture requires the Cleany chassis')
    raw = yaml.safe_load(Path(config).read_text(encoding='utf-8'))
    ignore_mast = raw.get('simulation_ignore_mast_collision', False)
    if not isinstance(ignore_mast, bool):
        raise ValueError('simulation_ignore_mast_collision must be a boolean')
    if ignore_mast:
        # Only the fixed mast, never its articulated camera children or the arms.
        mast = root.find(".//body[@name='top_base_link']")
        if mast is not None:
            for geom in mast.findall('geom'):
                geom.set('contype', '0')
                geom.set('conaffinity', '0')
    parent = chassis
    if 'internal_tray' in raw:
        _add_internal_tray(parent, raw['internal_tray'], load_bins(config))
    for bin_ in load_bins(config):
        body = ET.SubElement(parent, 'body', name=bin_.name)
        for suffix, size, center in bin_.boxes():
            trimmed_wall = bin_.rim_rgba is not None and suffix != 'floor'
            ET.SubElement(body, 'geom', {
                'name': f'{bin_.name}_{suffix}', 'type': 'box',
                'size': ' '.join(str(v / 2) for v in size),
                'pos': ' '.join(str(v) for v in center),
                'rgba': ('0 0 0 0' if trimmed_wall
                         else ' '.join(str(v) for v in bin_.rgba)),
                'contype': '1', 'conaffinity': '1',
                'friction': '1 0.005 0.0001', 'condim': '4',
            })
            if trimmed_wall:
                ET.SubElement(body, 'geom', {
                    'name': f'{bin_.name}_{suffix}_shell', 'type': 'box',
                    'size': f'{size[0]/2} {size[1]/2} {(size[2]-bin_.wall)/2}',
                    'pos': f'{center[0]} {center[1]} {center[2]-bin_.wall/2}',
                    'rgba': ' '.join(str(v) for v in bin_.rgba),
                    'contype': '0', 'conaffinity': '0', 'mass': '0',
                })
        if bin_.rim_rgba is not None:
            _add_bin_rim(body, bin_)
    return ET.tostring(root, encoding='unicode')


def load_shelf_boxes(path: str | Path) -> tuple:
    """Return shared fixture boxes for planning, in base_link coordinates."""
    raw = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    parent = ET.Element('body')
    if 'internal_tray' in raw:
        _add_internal_tray(parent, raw['internal_tray'], load_bins(path))
    return tuple((geom.get('name'),
                  tuple(2*float(v) for v in geom.get('size').split()),
                  tuple(float(v) for v in geom.get('pos').split()))
                 for geom in parent.findall('geom'))


def _add_internal_tray(parent: ET.Element, config: dict,
                       bins: tuple[BinGeometry, ...]) -> None:
    """Chassis-mounted support plate; structural fasteners are not modeled."""
    x, y = (float(v) for v in config['center_xy_m'])
    sx, sy = (float(v) for v in config['size_xy_m'])
    top = float(config['top_z_m'])
    thickness = float(config['board_thickness_m'])
    color = tuple(float(v) for v in config['board_rgba'])
    if (not all(math.isfinite(v) for v in (x, y, sx, sy, top, thickness))
            or min(sx, sy, thickness) <= 0 or len(color) != 4
            or not all(math.isfinite(v) and 0 <= v <= 1 for v in color)):
        raise ValueError('Invalid internal tray geometry')
    for bin_ in bins:
        if (bin_.kind != 'bin' or abs(bin_.bottom_z-top) > 1e-8
                or abs(bin_.center_xy[0]-x)+bin_.outside_size[0]/2 > sx/2
                or abs(bin_.center_xy[1]-y)+bin_.outside_size[1]/2 > sy/2):
            raise ValueError('Internal tray must support the complete bin footprint')
    ET.SubElement(parent, 'geom', {
        'name': 'internal_collection_tray', 'type': 'box',
        'size': f'{sx/2} {sy/2} {thickness/2}',
        'pos': f'{x} {y} {top-thickness/2}',
        'rgba': ' '.join(str(v) for v in color),
        'contype': '1', 'conaffinity': '1', 'condim': '4',
        'friction': '1 0.005 0.0001',
    })


def _add_bin_rim(body: ET.Element, bin_: BinGeometry) -> None:
    """Rounded color trim inside the existing wall envelope; visual only."""
    x, y = bin_.center_xy
    sx, sy, _ = bin_.outside_size
    radius = bin_.wall / 2
    z = bin_.top_z - radius
    dx, dy = sx / 2 - radius, sy / 2 - radius
    corners = ((x-dx, y-dy, z), (x+dx, y-dy, z),
               (x+dx, y+dy, z), (x-dx, y+dy, z))
    for index, start in enumerate(corners):
        end = corners[(index + 1) % 4]
        ET.SubElement(body, 'geom', {
            'name': f'{bin_.name}_rim_{index}', 'type': 'capsule',
            'size': str(radius),
            'fromto': ' '.join(str(v) for v in (*start, *end)),
            'rgba': ' '.join(str(v) for v in bin_.rim_rgba),
            'contype': '0', 'conaffinity': '0', 'mass': '0',
        })
