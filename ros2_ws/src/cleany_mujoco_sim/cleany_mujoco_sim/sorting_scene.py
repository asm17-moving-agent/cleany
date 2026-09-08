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
        if self.kind == 'table_zone':
            return self.bottom_z
        return self.bottom_z + self.outside_size[2]

    def boxes(self):
        """Five collision boxes (full extents, base-frame centers), no lid."""
        if self.kind == 'table_zone':
            return  # Painted regions have no collision geometry.
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
            and self.bottom_z + self.wall <= z < (
                self.bottom_z + self.outside_size[2] if self.kind == 'table_zone' else self.top_z)
        )


def load_bins(path: str | Path, *, include_staging: bool = False) -> tuple[BinGeometry, ...]:
    raw = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if raw.get('schema_version') != 1 or raw.get('frame_id') != 'base_link':
        raise ValueError('Bins require schema_version=1 and base_link frame')
    bins = []
    items = list(raw['bins'])
    if len(items) != 2:
        raise ValueError('Two distinct collection bins are required')
    if include_staging and 'staging_area' in raw:
        if raw['staging_area'].get('kind') != 'table_zone':
            raise ValueError('Staging must be a non-colliding table zone')
        items.append(raw['staging_area'])
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
                or geometry.kind not in ('bin', 'table_zone')
                or geometry.wall < 0.0
                or (geometry.kind == 'bin' and geometry.wall == 0)
                or (geometry.kind == 'table_zone' and geometry.wall != 0)
                or min(geometry.outside_size) <= 2 * geometry.wall
                or not all(0 <= v <= 1 for v in geometry.rgba)):
            raise ValueError('Invalid collection bin geometry')
        if geometry.rim_rgba is not None and (
                geometry.kind != 'bin' or len(geometry.rim_rgba) != 4
                or not all(math.isfinite(v) and 0 <= v <= 1 for v in geometry.rim_rgba)):
            raise ValueError('Invalid collection bin rim color')
        bins.append(geometry)
    if len({b.name for b in bins}) != len(items):
        raise ValueError('Distinct collection/staging identifiers are required')
    return tuple(bins)


def add_sorting_bins(model_text: str, config: str | Path) -> str:
    root = ET.fromstring(model_text)
    chassis = root.find("./worldbody/body[@name='chassis']")
    if chassis is None:
        raise ValueError('Sorting fixture requires the Cleany chassis')
    raw = yaml.safe_load(Path(config).read_text(encoding='utf-8'))
    parent = chassis
    if 'rear_shelf' in raw:
        # Copy only the initial frame transform: furniture must not follow the robot.
        parent = ET.SubElement(root.find('worldbody'), 'body', {
            'name': 'rear_collection_station',
            **{key: chassis.get(key) for key in ('pos', 'quat', 'euler', 'axisangle', 'xyaxes', 'zaxis')
               if chassis.get(key) is not None},
        })
        _add_rear_shelf(parent, raw['rear_shelf'], load_bins(config))
    for bin_ in load_bins(config):
        body = ET.SubElement(parent, 'body', name=bin_.name)
        if bin_.kind == 'table_zone':
            ET.SubElement(body, 'geom', {
                'name': f'{bin_.name}_paint', 'type': 'box',
                'size': f'{bin_.outside_size[0]/2} {bin_.outside_size[1]/2} 0.0001',
                'pos': f'{bin_.center_xy[0]} {bin_.center_xy[1]} {bin_.bottom_z+0.00015}',
                'rgba': ' '.join(str(v) for v in bin_.rgba),
                'contype': '0', 'conaffinity': '0', 'mass': '0',
            })
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
    """Return the same known shelf boxes for planning, in the initial base frame."""
    raw = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if 'rear_shelf' not in raw:
        return ()
    parent = ET.Element('body')
    _add_rear_shelf(parent, raw['rear_shelf'], load_bins(path))
    return tuple((geom.get('name'),
                  tuple(2*float(v) for v in geom.get('size').split()),
                  tuple(float(v) for v in geom.get('pos').split()))
                 for geom in parent.findall('geom'))


def _add_rear_shelf(parent: ET.Element, config: dict, bins: tuple[BinGeometry, ...]) -> None:
    """Static two-tier furniture in the initial base frame; all pieces collide."""
    x, y = (float(v) for v in config['center_xy_m'])
    sx, sy = (float(v) for v in config['size_xy_m'])
    top, floor, thickness, leg = (float(config[key]) for key in
        ('top_z_m', 'floor_z_m', 'board_thickness_m', 'leg_width_m'))
    board_color = tuple(float(v) for v in config['board_rgba'])
    frame_color = tuple(float(v) for v in config['frame_rgba'])
    if (not all(math.isfinite(v) for v in (x, y, sx, sy, top, floor, thickness, leg))
            or min(thickness, leg) <= 0 or min(sx, sy) <= 2*leg
            or top-floor <= 5*thickness
            or any(len(color) != 4 or not all(math.isfinite(v) and 0 <= v <= 1 for v in color)
                   for color in (board_color, frame_color))):
        raise ValueError('Invalid rear shelf geometry')
    for bin_ in bins:
        if (bin_.kind != 'bin' or abs(bin_.bottom_z-top) > 1e-8
                or abs(bin_.center_xy[0]-x)+bin_.outside_size[0]/2 > sx/2
                or abs(bin_.center_xy[1]-y)+bin_.outside_size[1]/2 > sy/2):
            raise ValueError('Rear shelf must support the complete bin footprint')

    def box(name: str, size: tuple, center: tuple, color: tuple) -> None:
        ET.SubElement(parent, 'geom', {
            'name': f'rear_shelf_{name}', 'type': 'box',
            'size': ' '.join(str(v/2) for v in size),
            'pos': ' '.join(str(v) for v in center),
            'rgba': ' '.join(str(v) for v in color),
            'contype': '1', 'conaffinity': '1', 'condim': '4',
            'friction': '1 0.005 0.0001',
        })
    box('top', (sx, sy, thickness), (x, y, top-thickness/2), board_color)
    box('lower', (sx, sy, thickness), (x, y, floor+3*thickness), board_color)
    height = top-thickness-floor
    for i, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        box(f'leg_{i}', (leg, leg, height),
            (x+dx*(sx-leg)/2, y+dy*(sy-leg)/2, floor+height/2), frame_color)


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
