from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Mapping, Sequence

import yaml


SCHEMA_VERSION = 1


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
class StudyCafeLayout:
    scene: SceneLayout
    robot_spawn_pose: tuple[float, float, float, float, float, float]
    room: RoomLayout
    desks: DeskLayout


def _mapping(value: object, context: str) -> Mapping[object, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f'{context} must be a mapping')
    return value


def _numbers(
    value: object, context: str, *, length: int | None = None
) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f'{context} must be a number sequence')
    try:
        numbers = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{context} must contain only numbers') from error
    if length is not None and len(numbers) != length:
        raise ValueError(f'{context} must contain {length} values')
    if not numbers or not all(isfinite(number) for number in numbers):
        raise ValueError(f'{context} must contain finite numbers')
    return numbers


def _positive(value: object, context: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{context} must be a number') from error
    if not isfinite(number) or number <= 0.0:
        raise ValueError(f'{context} must be positive')
    return number


def load_study_cafe_layout(path: Path) -> StudyCafeLayout:
    raw = _mapping(
        yaml.safe_load(path.read_text(encoding='utf-8')), 'layout config'
    )
    if raw.get('schema_version') != SCHEMA_VERSION:
        raise ValueError(
            f'unsupported study cafe layout schema: '
            f'{raw.get("schema_version")!r}'
        )

    scene_raw = _mapping(raw.get('scene'), 'scene')
    robot_raw = _mapping(raw.get('robot'), 'robot')
    room_raw = _mapping(raw.get('room'), 'room')
    desks_raw = _mapping(raw.get('desk_layout'), 'desk_layout')

    inside_size = _numbers(
        room_raw.get('inside_size_m'), 'room.inside_size_m', length=2
    )
    if any(number <= 0.0 for number in inside_size):
        raise ValueError('room.inside_size_m values must be positive')
    wall_roughness = float(room_raw.get('wall_roughness', -1.0))
    if not isfinite(wall_roughness) or not 0.0 <= wall_roughness <= 1.0:
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
        isfinite(value)
        for row in rows
        for value in (
            row.desk_y_offset_m,
            row.chair_y_offset_from_desk_m,
        )
    ):
        raise ValueError('desk_layout.rows must contain two finite row pairs')

    x_positions = _numbers(
        desks_raw.get('x_positions_m'), 'desk_layout.x_positions_m'
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
            robot_raw.get('spawn_pose'), 'robot.spawn_pose', length=6
        ),
        room=RoomLayout(
            inside_size_m=(inside_size[0], inside_size[1]),
            wall_thickness_m=_positive(
                room_raw.get('wall_thickness_m'), 'room.wall_thickness_m'
            ),
            wall_height_m=_positive(
                room_raw.get('wall_height_m'), 'room.wall_height_m'
            ),
            wall_rgba=_numbers(
                room_raw.get('wall_rgba'), 'room.wall_rgba', length=4
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
