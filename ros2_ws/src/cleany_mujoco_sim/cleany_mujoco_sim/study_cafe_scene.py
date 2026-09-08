from __future__ import annotations

import html
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import yaml

from cleany_mujoco_sim.study_cafe_lighting import apply_study_cafe_lighting
from cleany_mujoco_sim.tabletop_shapes import (
    TabletopShape, add_shape_assets, add_shape_geoms, parse_shape,
)


STUDY_CAFE_ENVIRONMENT_TOKEN = '@CLEANY_STUDY_CAFE_ENVIRONMENT@'
STUDY_CAFE_ASSET_DIR_TOKEN = '@CLEANY_STUDY_CAFE_ASSET_DIR@'
SCHEMA_VERSION = 1
_TABLETOP_TOP_Z_M = 0.72
_COLLISION_ATTRIBUTES = {
    'contype': '1',
    'conaffinity': '1',
    'condim': '4',
    'friction': '1 0.005 0.0001',
    # Group 2 is enabled in MuJoCo's default viewer. These primitives are
    # intentionally both the visual and the static collision geometry.
    'group': '2',
}


@dataclass(frozen=True)
class SceneLayout:
    ambient_rgba: tuple[float, float, float, float]
    background_rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class RoomLayout:
    inside_size_m: tuple[float, float]
    wall_thickness_m: float
    wall_height_m: float
    wall_rgba: tuple[float, float, float, float]
    wall_roughness: float


@dataclass(frozen=True)
class DeskRowLayout:
    desk_y_offset_m: float
    chair_y_offset_from_desk_m: float


@dataclass(frozen=True)
class DeskLayout:
    x_positions_m: tuple[float, ...]
    row_pair_centers_y_m: tuple[float, ...]
    rows: tuple[DeskRowLayout, ...]
    partition_center_z_m: float
    monitor_y_offset_from_desk_center_m: float


@dataclass(frozen=True)
class TabletopObjectLayout:
    name: str
    xy_offset_from_desk_center_m: tuple[float, float]
    yaw_rad: float
    mass_kg: float
    rgba: tuple[float, float, float, float]
    visual_type: str
    collision_type: str
    collision_size_m: tuple[float, ...]
    shape: TabletopShape | None


@dataclass(frozen=True)
class StudyCafeLayout:
    scene: SceneLayout
    robot_spawn_pose: tuple[float, float, float, float, float, float]
    robot_center_rearward_offset_m: float
    tabletop_objects: tuple[TabletopObjectLayout, ...]
    room: RoomLayout
    desks: DeskLayout


@dataclass(frozen=True)
class DeskStation:
    index: int
    x: float
    desk_y: float
    chair_y: float
    front_sign: float


def _mapping(value: object, context: str) -> Mapping[object, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f'{context} must be a mapping')
    return value


def _numbers(
    value: object,
    context: str,
    *,
    length: int | None = None,
) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f'{context} must be a number sequence')
    try:
        numbers = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{context} must contain only numbers') from error
    if length is not None and len(numbers) != length:
        raise ValueError(f'{context} must contain {length} values')
    if not numbers or not all(math.isfinite(number) for number in numbers):
        raise ValueError(f'{context} must contain finite numbers')
    return numbers


def _positive(value: object, context: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{context} must be a number') from error
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f'{context} must be positive')
    return number


def _tabletop_objects(
    value: object,
) -> tuple[TabletopObjectLayout, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError('mujoco.tabletop_objects must be a sequence')
    objects: list[TabletopObjectLayout] = []
    for index, item in enumerate(value):
        context = f'mujoco.tabletop_objects[{index}]'
        raw = _mapping(item, context)
        name = raw.get('name')
        if not isinstance(name, str) or not name.isidentifier():
            raise ValueError(f'{context}.name must be an identifier')
        yaw_rad = float(raw.get('yaw_rad', 0.0))
        if not math.isfinite(yaw_rad):
            raise ValueError(f'{context}.yaw_rad must be finite')
        rgba = _numbers(raw.get('rgba'), f'{context}.rgba', length=4)
        if any(value < 0.0 or value > 1.0 for value in rgba):
            raise ValueError(f'{context}.rgba values must be within [0, 1]')
        collision = _mapping(raw.get('collision'), f'{context}.collision')
        visual_type = raw.get('visual_type', 'mesh')
        if visual_type not in {'mesh', 'primitive', 'eraser', 'paper_cup', 'lego_brick', 'crumpled_tissue'}:
            raise ValueError(
                f'{context}.visual_type is unsupported'
            )
        collision_type = collision.get('type')
        if collision_type not in {'box', 'cylinder', 'compound', 'mesh'}:
            raise ValueError(
                f'{context}.collision.type is unsupported'
            )
        expected_size_count = 2 if collision_type == 'cylinder' or visual_type == 'paper_cup' else 3
        collision_size = _numbers(
            collision.get('size_m'),
            f'{context}.collision.size_m',
            length=expected_size_count,
        )
        if any(size <= 0.0 for size in collision_size):
            raise ValueError(f'{context}.collision.size_m must be positive')
        shape = parse_shape(visual_type, raw.get('geometry'), collision_type, collision_size)
        if (shape is None and collision_type in ('compound', 'mesh')
                and not (visual_type == 'mesh' and collision_type == 'mesh')):
            raise ValueError(f'{context} compound/mesh collision requires procedural geometry')
        objects.append(
            TabletopObjectLayout(
                name=name,
                xy_offset_from_desk_center_m=_numbers(
                    raw.get('xy_offset_from_desk_center_m'),
                    f'{context}.xy_offset_from_desk_center_m',
                    length=2,
                ),
                yaw_rad=yaw_rad,
                mass_kg=_positive(raw.get('mass_kg'), f'{context}.mass_kg'),
                rgba=rgba,
                visual_type=visual_type,
                collision_type=collision_type,
                collision_size_m=collision_size,
                shape=shape,
            )
        )
    names = [item.name for item in objects]
    if set(names) != {'cup', 'lego', 'tissue', 'wallet'}:
        raise ValueError(
            'mujoco.tabletop_objects must define cup, lego, tissue, wallet'
        )
    if len(names) != len(set(names)):
        raise ValueError('mujoco.tabletop_objects names must be unique')
    return tuple(objects)


def load_study_cafe_layout(path: Path) -> StudyCafeLayout:
    raw = _mapping(
        yaml.safe_load(path.read_text(encoding='utf-8')),
        'layout config',
    )
    if raw.get('schema_version') != SCHEMA_VERSION:
        raise ValueError(
            'unsupported study cafe layout schema: '
            f'{raw.get("schema_version")!r}'
        )

    scene_raw = _mapping(raw.get('scene'), 'scene')
    robot_raw = _mapping(raw.get('robot'), 'robot')
    mujoco_raw = _mapping(raw.get('mujoco'), 'mujoco')
    room_raw = _mapping(raw.get('room'), 'room')
    desks_raw = _mapping(raw.get('desk_layout'), 'desk_layout')
    inside_size = _numbers(
        room_raw.get('inside_size_m'),
        'room.inside_size_m',
        length=2,
    )
    if any(number <= 0.0 for number in inside_size):
        raise ValueError('room.inside_size_m values must be positive')
    wall_roughness = float(room_raw.get('wall_roughness', -1.0))
    if not math.isfinite(wall_roughness) or not 0.0 <= wall_roughness <= 1.0:
        raise ValueError('room.wall_roughness must be within [0, 1]')

    rows_raw = desks_raw.get('rows')
    if isinstance(rows_raw, (str, bytes)) or not isinstance(
        rows_raw, Sequence
    ):
        raise ValueError('desk_layout.rows must be a sequence')
    rows = tuple(
        DeskRowLayout(
            desk_y_offset_m=float(
                _mapping(row, f'desk_layout.rows[{index}]').get(
                    'desk_y_offset_m'
                )
            ),
            chair_y_offset_from_desk_m=float(
                _mapping(row, f'desk_layout.rows[{index}]').get(
                    'chair_y_offset_from_desk_m'
                )
            ),
        )
        for index, row in enumerate(rows_raw)
    )
    if len(rows) != 2 or not all(
        math.isfinite(value)
        for row in rows
        for value in (
            row.desk_y_offset_m,
            row.chair_y_offset_from_desk_m,
        )
    ):
        raise ValueError('desk_layout.rows must contain two finite row pairs')

    x_positions = _numbers(
        desks_raw.get('x_positions_m'),
        'desk_layout.x_positions_m',
    )
    if len(x_positions) < 2 or tuple(sorted(x_positions)) != x_positions:
        raise ValueError(
            'desk_layout.x_positions_m must be strictly increasing'
        )
    if len(set(x_positions)) != len(x_positions):
        raise ValueError('desk_layout.x_positions_m must be unique')
    row_pair_centers = _numbers(
        desks_raw.get('row_pair_centers_y_m'),
        'desk_layout.row_pair_centers_y_m',
    )
    if len(set(row_pair_centers)) != len(row_pair_centers):
        raise ValueError('desk row-pair centers must be unique')

    return StudyCafeLayout(
        scene=SceneLayout(
            ambient_rgba=_numbers(
                scene_raw.get('ambient_rgba'),
                'scene.ambient_rgba',
                length=4,
            ),
            background_rgba=_numbers(
                scene_raw.get('background_rgba'),
                'scene.background_rgba',
                length=4,
            ),
        ),
        robot_spawn_pose=_numbers(
            robot_raw.get('spawn_pose'),
            'robot.spawn_pose',
            length=6,
        ),
        robot_center_rearward_offset_m=_positive(
            mujoco_raw.get('robot_center_rearward_offset_m'),
            'mujoco.robot_center_rearward_offset_m',
        ),
        tabletop_objects=_tabletop_objects(
            mujoco_raw.get('tabletop_objects')
        ),
        room=RoomLayout(
            inside_size_m=(inside_size[0], inside_size[1]),
            wall_thickness_m=_positive(
                room_raw.get('wall_thickness_m'),
                'room.wall_thickness_m',
            ),
            wall_height_m=_positive(
                room_raw.get('wall_height_m'),
                'room.wall_height_m',
            ),
            wall_rgba=_numbers(
                room_raw.get('wall_rgba'),
                'room.wall_rgba',
                length=4,
            ),
            wall_roughness=wall_roughness,
        ),
        desks=DeskLayout(
            x_positions_m=x_positions,
            row_pair_centers_y_m=row_pair_centers,
            rows=rows,
            partition_center_z_m=_positive(
                desks_raw.get('partition_center_z_m'),
                'desk_layout.partition_center_z_m',
            ),
            monitor_y_offset_from_desk_center_m=_positive(
                desks_raw.get('monitor_y_offset_from_desk_center_m'),
                'desk_layout.monitor_y_offset_from_desk_center_m',
            ),
        ),
    )


def _values(values: Sequence[float]) -> str:
    return ' '.join(f'{value:.12g}' for value in values)


def _quaternion(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _add_body(
    parent: ET.Element,
    name: str,
    position: tuple[float, float, float],
    *,
    yaw: float = 0.0,
) -> ET.Element:
    attributes = {'name': name, 'pos': _values(position)}
    if yaw != 0.0:
        attributes['quat'] = _values(_quaternion(0.0, 0.0, yaw))
    return ET.SubElement(parent, 'body', attributes)


def _add_geom(
    body: ET.Element,
    name: str,
    geom_type: str,
    size: tuple[float, ...],
    position: tuple[float, float, float],
    rgba: tuple[float, float, float, float],
    *,
    roll: float = 0.0,
    collision: bool = True,
) -> None:
    attributes = {
        'name': name,
        'type': geom_type,
        'size': _values(size),
        'pos': _values(position),
        'rgba': _values(rgba),
    }
    if roll != 0.0:
        attributes['quat'] = _values(_quaternion(roll, 0.0, 0.0))
    if collision:
        attributes.update(_COLLISION_ATTRIBUTES)
    else:
        attributes.update(
            {'contype': '0', 'conaffinity': '0', 'group': '2'}
        )
    ET.SubElement(body, 'geom', attributes)


def _add_box(
    body: ET.Element,
    name: str,
    full_size: tuple[float, float, float],
    position: tuple[float, float, float],
    rgba: tuple[float, float, float, float],
    *,
    roll: float = 0.0,
    collision: bool = True,
) -> None:
    _add_geom(
        body,
        name,
        'box',
        tuple(value / 2.0 for value in full_size),
        position,
        rgba,
        roll=roll,
        collision=collision,
    )


def _add_cylinder(
    body: ET.Element,
    name: str,
    radius: float,
    full_length: float,
    position: tuple[float, float, float],
    rgba: tuple[float, float, float, float],
    *,
    roll: float = 0.0,
) -> None:
    _add_geom(
        body,
        name,
        'cylinder',
        (radius, full_length / 2.0),
        position,
        rgba,
        roll=roll,
    )


def _add_desk(
    parent: ET.Element,
    name: str,
    x: float,
    y: float,
    front_sign: float,
) -> None:
    body = _add_body(parent, name, (x, y, 0.0))
    color = (0.92, 0.93, 0.94, 1.0)
    corner_radius = 0.06
    half_depth = 0.385
    _add_box(
        body,
        f'{name}__tabletop_back',
        (1.2, 0.77 - corner_radius, 0.04),
        (0.0, -front_sign * corner_radius / 2.0, 0.70),
        color,
    )
    _add_box(
        body,
        f'{name}__tabletop_front_center',
        (1.2 - 2.0 * corner_radius, corner_radius, 0.04),
        (0.0, front_sign * (half_depth - corner_radius / 2.0), 0.70),
        color,
    )
    for side_name, corner_x in (
        ('left', -0.60 + corner_radius),
        ('right', 0.60 - corner_radius),
    ):
        _add_cylinder(
            body,
            f'{name}__tabletop_front_{side_name}_corner',
            corner_radius,
            0.04,
            (
                corner_x,
                front_sign * (half_depth - corner_radius),
                0.70,
            ),
            color,
        )
    for support_name, support_x in (('left', -0.52), ('right', 0.52)):
        for side_name, bottom_y in (
            ('front', half_depth - 0.08),
            ('back', -half_depth + 0.08),
        ):
            top_y = 0.035 if bottom_y > 0.0 else -0.035
            delta_y = top_y - bottom_y
            delta_z = 0.67 - 0.02
            _add_box(
                body,
                f'{name}__{support_name}_{side_name}_leg',
                (0.045, 0.045, math.hypot(delta_y, delta_z)),
                (support_x, (bottom_y + top_y) / 2.0, 0.345),
                color,
                roll=math.atan2(-delta_y, delta_z),
            )
    _add_box(
        body,
        f'{name}__upper_crossbar',
        (0.82, 0.045, 0.045),
        (0.0, 0.0, 0.62),
        color,
    )


def _add_partition(
    parent: ET.Element,
    name: str,
    x: float,
    y: float,
    z: float,
) -> None:
    body = _add_body(parent, name, (x, y, z))
    color = (0.78, 0.80, 0.82, 1.0)
    width, thickness, height, radius = 1.2, 0.025, 0.72, 0.05
    _add_box(
        body,
        f'{name}__partition_center',
        (width - 2.0 * radius, thickness, height),
        (0.0, 0.0, 0.0),
        color,
    )
    _add_box(
        body,
        f'{name}__partition_middle',
        (width, thickness, height - 2.0 * radius),
        (0.0, 0.0, 0.0),
        color,
    )
    for horizontal_name, corner_x in (
        ('left', -width / 2.0 + radius),
        ('right', width / 2.0 - radius),
    ):
        for vertical_name, corner_z in (
            ('bottom', -height / 2.0 + radius),
            ('top', height / 2.0 - radius),
        ):
            _add_cylinder(
                body,
                f'{name}__partition_{vertical_name}_{horizontal_name}_corner',
                radius,
                thickness,
                (corner_x, 0.0, corner_z),
                color,
                roll=math.pi / 2.0,
            )


def _add_monitor(
    parent: ET.Element,
    name: str,
    x: float,
    y: float,
    front_sign: float,
) -> None:
    body = _add_body(parent, name, (x, y, 0.0))
    bezel = (0.025, 0.025, 0.03, 1.0)
    screen = (0.008, 0.010, 0.014, 1.0)
    _add_box(
        body,
        f'{name}__monitor_panel',
        (0.62, 0.035, 0.36),
        (0.0, 0.0, 1.0),
        bezel,
    )
    _add_box(
        body,
        f'{name}__monitor_stem',
        (0.035, 0.035, 0.12),
        (0.0, 0.0, 0.79),
        bezel,
    )
    _add_box(
        body,
        f'{name}__monitor_base',
        (0.24, 0.16, 0.02),
        (0.0, front_sign * 0.04, 0.73),
        bezel,
    )
    _add_box(
        body,
        f'{name}__monitor_screen_visual',
        (0.598, 0.002, 0.336),
        (0.0, front_sign * 0.0185, 1.0),
        screen,
        collision=False,
    )


def _add_chair(
    parent: ET.Element,
    name: str,
    chair_x: float,
    chair_y: float,
    desk_x: float,
    desk_y: float,
) -> None:
    yaw = math.atan2(desk_y - chair_y, desk_x - chair_x)
    body = _add_body(parent, name, (chair_x, chair_y, 0.0), yaw=yaw)
    dark = (0.10, 0.11, 0.12, 1.0)
    fabric = (0.32, 0.34, 0.36, 1.0)
    _add_cylinder(
        body,
        f'{name}__caster_base',
        0.32,
        0.06,
        (0.0, 0.0, 0.05),
        dark,
    )
    _add_cylinder(
        body,
        f'{name}__center_column',
        0.045,
        0.34,
        (-0.02, 0.0, 0.22),
        dark,
    )
    _add_box(
        body,
        f'{name}__seat',
        (0.52, 0.55, 0.08),
        (-0.03, 0.0, 0.42),
        fabric,
    )
    _add_box(
        body,
        f'{name}__backrest',
        (0.10, 0.48, 0.50),
        (-0.35, 0.0, 0.73),
        fabric,
    )


def _desk_stations(layout: StudyCafeLayout) -> tuple[DeskStation, ...]:
    stations: list[DeskStation] = []
    index = 1
    for pair_center in layout.desks.row_pair_centers_y_m:
        for row in layout.desks.rows:
            desk_y = pair_center + row.desk_y_offset_m
            chair_offset = row.chair_y_offset_from_desk_m
            front_sign = 1.0 if chair_offset > 0.0 else -1.0
            for desk_x in layout.desks.x_positions_m:
                stations.append(
                    DeskStation(
                        index=index,
                        x=desk_x,
                        desk_y=desk_y,
                        chair_y=desk_y + chair_offset,
                        front_sign=front_sign,
                    )
                )
                index += 1
    return tuple(stations)


def _robot_station(layout: StudyCafeLayout) -> DeskStation:
    spawn_x, spawn_y = layout.robot_spawn_pose[:2]
    return min(
        _desk_stations(layout),
        key=lambda station: (
            (station.x - spawn_x) ** 2 + (station.chair_y - spawn_y) ** 2
        ),
    )


def _add_tabletop_object(
    parent: ET.Element,
    station: DeskStation,
    item: TabletopObjectLayout,
) -> None:
    offset_x, offset_y = item.xy_offset_from_desk_center_m
    body_name = f'study_cafe_{item.name}'
    body = _add_body(
        parent,
        body_name,
        (
            station.x + offset_x,
            station.desk_y + offset_y,
            _TABLETOP_TOP_Z_M,
        ),
        yaw=item.yaw_rad,
    )
    ET.SubElement(body, 'freejoint', {'name': f'{body_name}_freejoint'})
    if item.shape is not None:
        add_shape_geoms(body, body_name, item.collision_size_m, item.rgba, item.mass_kg, item.shape)
        return
    if item.collision_type in ('box', 'mesh'):
        geom_size = tuple(size / 2.0 for size in item.collision_size_m)
        height = item.collision_size_m[2]
    else:
        diameter, height = item.collision_size_m
        geom_size = (diameter / 2.0, height / 2.0)
    visual_attributes = {
        'name': f'{body_name}_visual',
        'rgba': _values(item.rgba),
        'contype': '0',
        'conaffinity': '0',
        'density': '0',
        'group': '2',
    }
    if item.visual_type == 'mesh':
        visual_attributes.update(
            {'type': 'mesh', 'mesh': f'{body_name}_mesh'}
        )
        ET.SubElement(body, 'geom', visual_attributes)
    elif item.visual_type == 'eraser':
        visual_attributes.update(
            {
                'type': 'box',
                'size': _values(geom_size),
                'pos': _values((0.0, 0.0, height / 2.0)),
            }
        )
        ET.SubElement(body, 'geom', visual_attributes)
        # A slightly oversized, visual-only paper sleeve covers the middle
        # of the eraser while leaving both coloured rubber ends exposed.
        ET.SubElement(
            body,
            'geom',
            {
                'name': f'{body_name}_sleeve_visual',
                'type': 'box',
                'size': _values(
                    (
                        geom_size[0] * 0.58,
                        geom_size[1] + 0.0006,
                        geom_size[2] + 0.0006,
                    )
                ),
                'pos': _values((0.0, 0.0, height / 2.0)),
                'rgba': '0.12 0.25 0.68 1',
                'contype': '0',
                'conaffinity': '0',
                'density': '0',
                'group': '2',
            },
        )
    else:
        visual_attributes.update(
            {
                'type': item.collision_type,
                'size': _values(geom_size),
                'pos': _values((0.0, 0.0, height / 2.0)),
            }
        )
        ET.SubElement(body, 'geom', visual_attributes)
    ET.SubElement(
        body,
        'geom',
        {
            'name': f'{body_name}_collision',
            'type': item.collision_type,
            **({'mesh': f'{body_name}_mesh'} if item.collision_type == 'mesh' else {
                'size': _values(geom_size),
                'pos': _values((0.0, 0.0, height / 2.0)),
            }),
            'rgba': '0 0 0 0',
            'contype': '1',
            'conaffinity': '1',
            'condim': '6',
            'friction': '1.5 0.08 0.02',
            'mass': f'{item.mass_kg:.12g}',
            'group': '3',
        },
    )


def build_study_cafe_environment(layout: StudyCafeLayout) -> str:
    environment = ET.Element('environment')
    room_width, room_depth = layout.room.inside_size_m
    thickness = layout.room.wall_thickness_m
    height = layout.room.wall_height_m
    half_width, half_depth = room_width / 2.0, room_depth / 2.0
    wall_z = height / 2.0
    walls = (
        (
            'wall_north',
            (0.0, half_depth + thickness / 2.0, wall_z),
            (room_width + thickness, thickness, height),
        ),
        (
            'wall_south',
            (0.0, -half_depth - thickness / 2.0, wall_z),
            (room_width + thickness, thickness, height),
        ),
        (
            'wall_east',
            (half_width + thickness / 2.0, 0.0, wall_z),
            (thickness, room_depth, height),
        ),
        (
            'wall_west',
            (-half_width - thickness / 2.0, 0.0, wall_z),
            (thickness, room_depth, height),
        ),
    )
    for name, position, size in walls:
        body = _add_body(environment, name, position)
        _add_box(
            body,
            f'{name}__body',
            size,
            (0.0, 0.0, 0.0),
            layout.room.wall_rgba,
        )

    partition_index = 1
    for pair_center in layout.desks.row_pair_centers_y_m:
        for desk_x in layout.desks.x_positions_m:
            _add_partition(
                environment,
                f'desk_partition_{partition_index:02d}',
                desk_x,
                pair_center,
                layout.desks.partition_center_z_m,
            )
            partition_index += 1

    robot_station = _robot_station(layout)
    for station in _desk_stations(layout):
        _add_desk(
            environment,
            f'demo_desk_{station.index:02d}',
            station.x,
            station.desk_y,
            station.front_sign,
        )
        _add_monitor(
            environment,
            f'desk_monitor_{station.index:02d}',
            station.x,
            station.desk_y
            - station.front_sign
            * layout.desks.monitor_y_offset_from_desk_center_m,
            station.front_sign,
        )
        if station.index != robot_station.index:
            _add_chair(
                environment,
                f'office_chair_{station.index:02d}',
                station.x,
                station.chair_y,
                station.x,
                station.desk_y,
            )
    for item in layout.tabletop_objects:
        _add_tabletop_object(environment, robot_station, item)

    ET.indent(environment, space='    ')
    return '\n'.join(
        ET.tostring(child, encoding='unicode') for child in environment
    )


def apply_study_cafe_layout(
    scene_text: str,
    model_text: str,
    layout_path: Path,
    asset_directory: Path,
) -> tuple[str, str]:
    if scene_text.count(STUDY_CAFE_ENVIRONMENT_TOKEN) != 1:
        raise ValueError(
            'Study-cafe scene template must contain exactly one '
            f'{STUDY_CAFE_ENVIRONMENT_TOKEN}'
        )
    layout = load_study_cafe_layout(layout_path)
    mesh_items = tuple(
        item for item in layout.tabletop_objects if item.visual_type == 'mesh'
    )
    if scene_text.count(STUDY_CAFE_ASSET_DIR_TOKEN) != len(mesh_items):
        raise ValueError(
            'Study-cafe scene template asset reference count must match '
            'the configured mesh objects'
        )
    for item in mesh_items:
        asset_path = asset_directory / f'study_cafe_{item.name}.obj'
        if not asset_path.is_file():
            raise FileNotFoundError(
                f'Study-cafe tabletop asset not found: {asset_path}'
            )
    scene_text = scene_text.replace(
        STUDY_CAFE_ASSET_DIR_TOKEN,
        html.escape(str(asset_directory.resolve()), quote=True),
    )
    scene_text = scene_text.replace(
        STUDY_CAFE_ENVIRONMENT_TOKEN,
        build_study_cafe_environment(layout),
    )

    scene_root = ET.fromstring(scene_text)
    assets = scene_root.find('asset')
    if assets is None:
        raise ValueError('Study-cafe template requires an asset section')
    for item in layout.tabletop_objects:
        if item.shape is not None:
            add_shape_assets(assets, f'study_cafe_{item.name}', item.collision_size_m, item.shape)
    # Apply each explicitly configured jaw contact policy to every convex part
    # of that object, not just the bottom of the hollow paper cup or LEGO body.
    contacts = scene_root.find('contact')
    if contacts is not None:
        for pair in list(contacts):
            for item in layout.tabletop_objects:
                prefix = f'study_cafe_{item.name}'
                if pair.get('geom1') != f'{prefix}_collision':
                    continue
                for geom in scene_root.findall(f"./worldbody/body[@name='{prefix}']/geom"):
                    name = geom.get('name', '')
                    if name.endswith('_collision') and name != pair.get('geom1'):
                        attributes = dict(pair.attrib)
                        attributes.update(name=f'{pair.get("name")}_{name}', geom1=name)
                        ET.SubElement(contacts, 'pair', attributes)
    root = ET.fromstring(model_text)
    apply_study_cafe_lighting(scene_root, root, asset_directory.parent / 'config' / 'study_cafe_lighting.yaml')
    scene_text = ET.tostring(scene_root, encoding='unicode')
    chassis = root.find("./worldbody/body[@name='chassis']")
    if chassis is None:
        raise ValueError('Cleany MJCF is missing the chassis body')
    _, _, z, roll, pitch, _ = layout.robot_spawn_pose
    station = _robot_station(layout)
    yaw = math.atan2(
        station.desk_y - station.chair_y,
        0.0,
    )
    chassis.set(
        'pos',
        _values(
            (
                station.x,
                station.chair_y
                + station.front_sign
                * layout.robot_center_rearward_offset_m,
                z,
            )
        ),
    )
    chassis.set('quat', _values(_quaternion(roll, pitch, yaw)))
    chassis.attrib.pop('euler', None)
    ET.indent(root, space='  ')
    return scene_text, ET.tostring(root, encoding='unicode')
