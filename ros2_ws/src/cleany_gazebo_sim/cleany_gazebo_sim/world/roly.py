"""Local, static ROLY-inspired furniture with explicit estimated dimensions."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin, sqrt
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml


@dataclass(frozen=True)
class RolyConfig:
    values: dict[str, object]
    mesh_dir: Path

    def number(self, key: str) -> float:
        return float(self.values[key])

    def vector(self, key: str) -> tuple[float, ...]:
        return tuple(float(x) for x in self.values[key])

    def color(self, key: str) -> str:
        return ' '.join(map(str, self.values['colors'][key]))


def load_roly_config(path: Path) -> RolyConfig:
    raw = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(raw, dict) or raw.get('schema_version') != 1:
        raise ValueError('unsupported ROLY config schema')
    vectors = ('seat_size_m', 'back_size_m', 'back_origin_m')
    numbers = (
        'seat_top_m',
        'arm_top_m',
        'arm_half_width_m',
        'arm_length_m',
        'arm_pad_width_m',
        'arm_pad_thickness_m',
        'frame_radius_m',
        'base_radius_m',
        'base_hub_height_m',
        'base_tip_height_m',
        'base_spoke_width_m',
        'base_spoke_height_m',
        'column_radius_m',
        'caster_radius_m',
        'caster_width_m',
        'caster_gap_m',
    )
    try:
        for key in vectors:
            value = tuple(float(x) for x in raw[key])
            if len(value) != 3 or not all(isfinite(x) for x in value):
                raise ValueError(key)
            if key != 'back_origin_m' and min(value) <= 0:
                raise ValueError(key)
        for key in numbers:
            if not isfinite(float(raw[key])) or float(raw[key]) <= 0:
                raise ValueError(key)
        for key in ('frame', 'mesh', 'seat', 'metal', 'tire'):
            color = raw['colors'][key]
            if len(color) != 4 or not all(0 <= float(x) <= 1 for x in color):
                raise ValueError(key)
        if (
            not raw['base_tip_height_m']
            < raw['base_hub_height_m']
            < (raw['seat_top_m'] - raw['seat_size_m'][2])
            < raw['arm_top_m']
        ):
            raise ValueError('vertical dimensions')
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f'invalid ROLY dimensions/colors: {error}') from error
    # Resolve package assets independently of a custom config's directory.
    mesh_dir = Path(__file__).resolve().parents[2] / 'models/roly_p1g210m'
    if not mesh_dir.is_dir():
        from ament_index_python.packages import get_package_share_directory

        mesh_dir = (
            Path(get_package_share_directory('cleany_gazebo_sim'))
            / 'models/roly_p1g210m'
        )
    if not all(
        (mesh_dir.parents[1] / 'materials/roly' / name).is_file()
        for name in ('seat.png', 'back.png')
    ):
        raise ValueError(f'ROLY meshes missing: {mesh_dir}')
    return RolyConfig(raw, mesh_dir)


def add_roly_chair(
    world: ET.Element,
    name: str,
    pose: tuple[float, ...],
    config: RolyConfig,
    mesh_cache_dir: Path | None = None,
) -> ET.Element:
    # Imported lazily so the existing generator can select this model too.
    from cleany_gazebo_sim.world.generator import (
        _add_box_part,
        _add_cylinder_part,
        _add_box_collision,
    )

    c = config
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = ' '.join(map(str, pose))
    link = ET.SubElement(model, 'link', name='body')
    frame, dark = c.color('frame'), c.color('metal')

    def rod(label, start, end, radius, color=frame):
        delta = tuple(b - a for a, b in zip(start, end))
        length = sqrt(sum(x * x for x in delta))
        midpoint = tuple((a + b) / 2 for a, b in zip(start, end))
        _add_cylinder_part(
            link,
            label,
            radius,
            length,
            (
                *midpoint,
                0.0,
                atan2(hypot(*delta[:2]), delta[2]),
                atan2(delta[1], delta[0]),
            ),
            color,
        )

    hub, tip = c.number('base_hub_height_m'), c.number('base_tip_height_m')
    radius, wheel = c.number('base_radius_m'), c.number('caster_radius_m')
    for i in range(5):
        angle = 2 * pi * i / 5
        x, y = radius * cos(angle), radius * sin(angle)
        length = hypot(radius, hub - tip)
        _add_box_part(
            link,
            f'spoke_{i}',
            (length, c.number('base_spoke_width_m'), c.number('base_spoke_height_m')),
            (x / 2, y / 2, (hub + tip) / 2, 0.0, atan2(hub - tip, radius), angle),
            frame,
        )
        rod(f'caster_stem_{i}', (x, y, wheel), (x, y, tip), 0.012)
        for side in (-1, 1):
            offset = side * (c.number('caster_width_m') + c.number('caster_gap_m')) / 2
            _add_cylinder_part(
                link,
                f'caster_{i}_{side}',
                wheel,
                c.number('caster_width_m'),
                (
                    x - offset * sin(angle),
                    y + offset * cos(angle),
                    wheel,
                    pi / 2,
                    0.0,
                    angle,
                ),
                c.color('tire'),
            )
    depth, width, thickness = c.vector('seat_size_m')
    top = c.number('seat_top_m')
    bottom = top - thickness
    rod(
        'center_column',
        (0.0, 0.0, hub),
        (0.0, 0.0, bottom),
        c.number('column_radius_m'),
        dark,
    )
    rod(
        'column_cover',
        (0.0, 0.0, 0.06),
        (0.0, 0.0, hub),
        c.number('column_radius_m') * 1.5,
    )
    _add_box_collision(
        link,
        'seat_collision',
        (depth, width, thickness),
        (0.02, 0.0, top - thickness / 2),
    )
    origin = c.vector('back_origin_m')
    back = c.vector('back_size_m')
    # Broad collision panels follow the bow; fine weave is not simulated optically.
    for i in range(5):
        t = (i + 0.5) / 5
        x = origin[0] + back[0] * (0.58 * sin(pi * t) - 0.72 * t)
        _add_box_collision(
            link,
            f'back_panel_{i}',
            (0.05, back[1] * (1.0 - 0.10 * sin(pi * t)), back[2] / 5),
            (x, 0.0, origin[2] + back[2] * t),
        )
    for side in (-1, 1):
        y = side * c.number('arm_half_width_m')
        points = [
            (0.19, side * 0.19, bottom - 0.008),
            (0.0, side * 0.09, bottom - 0.112),
            (-0.28, side * 0.245, bottom + 0.004),
            (-0.225, y, c.number('arm_top_m') - 0.12),
            (-0.18, y, c.number('arm_top_m') - 0.028),
        ]
        for i, (a, b) in enumerate(zip(points, points[1:])):
            rod(f'cradle_{side}_{i}', a, b, 0.022)
        _add_box_part(
            link,
            f'arm_pad_{side}',
            (
                c.number('arm_length_m'),
                c.number('arm_pad_width_m'),
                c.number('arm_pad_thickness_m'),
            ),
            (
                -0.13,
                y,
                c.number('arm_top_m') - c.number('arm_pad_thickness_m') / 2,
                0.0,
                0.0,
                0.0,
            ),
            frame,
        )
    _add_box_collision(
        link, 'tilt_housing', (0.22, 0.17, 0.09), (-0.045, 0.0, bottom - 0.094)
    )
    # Thin back-rim/stem visuals are already covered by the broad back panels
    # and caster/spoke contact shapes. Avoid redundant static physics shapes.
    for collision in list(link.findall('collision')):
        if collision.get('name', '').startswith(('back_rim_', 'caster_stem_')):
            link.remove(collision)
    # Detailed smooth surfaces are independent of the simple physics proxies.
    for visual in list(link.findall('visual')):
        link.remove(visual)
    from cleany_gazebo_sim.world.roly_geometry import add_visuals
    from tempfile import gettempdir

    add_visuals(link, c, mesh_cache_dir or Path(gettempdir()) / 'cleany_roly_meshes')
    return model
