from pathlib import Path
from xml.etree import ElementTree

import pytest

from cleany_gazebo_sim.world.generator import materialize_study_cafe_world
from cleany_gazebo_sim.world.layout import load_study_cafe_layout


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROBOT_WORLD = PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_harmonic.sdf'
FORTRESS_ROBOT_WORLD = (
    PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_fortress.sdf'
)
LAYOUT_CONFIG = (
    PACKAGE_ROOT / 'config' / 'study_cafe' / 'study_cafe_layout.yaml'
)


def _world(tmp_path: Path) -> ElementTree.Element:
    generated = materialize_study_cafe_world(
        ROBOT_WORLD,
        tmp_path / 'study_cafe.sdf',
        layout_path=LAYOUT_CONFIG,
    )
    world = ElementTree.parse(generated).getroot().find(
        "world[@name='cleany_study_cafe']"
    )
    assert world is not None
    return world


def test_study_cafe_materializes_expected_scenario_entities(
    tmp_path: Path,
) -> None:
    world = _world(tmp_path)
    names = {model.get('name', '') for model in world.findall('model')}

    assert 'cleany_mecanum' in names
    assert {'wall_north', 'wall_south', 'wall_east', 'wall_west'} <= names
    assert len([name for name in names if name.startswith('demo_desk_')]) == 48
    assert len(
        [name for name in names if name.startswith('office_chair_')]
    ) == 48
    assert len(
        [name for name in names if name.startswith('desk_partition_')]
    ) == 24
    assert len(
        [name for name in names if name.startswith('desk_monitor_')]
    ) == 48


def test_study_cafe_materializes_for_fortress(tmp_path: Path) -> None:
    generated = materialize_study_cafe_world(
        FORTRESS_ROBOT_WORLD,
        tmp_path / 'study_cafe_fortress.sdf',
        simulator='fortress',
        layout_path=LAYOUT_CONFIG,
    )
    world = ElementTree.parse(generated).getroot().find(
        "world[@name='cleany_study_cafe']"
    )
    assert world is not None
    assert world.find("model[@name='cleany_mecanum']") is not None


def test_study_cafe_applies_bounded_physics_override(tmp_path: Path) -> None:
    generated = materialize_study_cafe_world(
        ROBOT_WORLD,
        tmp_path / 'accelerated.sdf',
        max_step_size=0.003,
        real_time_factor=2.0,
        layout_path=LAYOUT_CONFIG,
    )
    world = ElementTree.parse(generated).getroot().find('world')
    assert world is not None
    assert world.findtext('physics/max_step_size') == '0.003'
    assert world.findtext('physics/real_time_factor') == '2.0'


def test_robot_visual_is_excluded_from_its_lidar(tmp_path: Path) -> None:
    robot = _world(tmp_path).find("model[@name='cleany_mecanum']")
    assert robot is not None
    visuals = robot.findall('.//visual')
    lidars = robot.findall(".//sensor[@type='gpu_lidar']/lidar")
    assert visuals
    assert len(lidars) == 1
    assert all(
        visual.findtext('visibility_flags') == '0x02'
        for visual in visuals
    )
    assert all(lidar.findtext('visibility_mask') == '0x01' for lidar in lidars)


def test_study_cafe_materializes_one_selected_lidar_pose(tmp_path: Path) -> None:
    generated = materialize_study_cafe_world(
        ROBOT_WORLD,
        tmp_path / 'lidar_70cm.sdf',
        layout_path=LAYOUT_CONFIG,
        lidar_translation=(0.16, 0.0, 0.32),
    )
    mount = ElementTree.parse(generated).getroot().find(
        "./world/model[@name='cleany_mecanum']/joint[@name='lidar_mount']"
    )
    assert mount is not None
    assert mount.findtext('child') == 'lidar_link'
    assert mount.findtext('pose') == '0.16 0.0 0.32 0.0 0.0 0.0'


def test_study_cafe_layout_schema_is_validated(tmp_path: Path) -> None:
    layout = load_study_cafe_layout(LAYOUT_CONFIG)
    assert layout.room.inside_size_m == (12.26, 10.94)
    assert len(layout.desks.x_positions_m) == 8
    assert len(layout.desks.row_pair_centers_y_m) == 3
    assert len(layout.desks.rows) == 2

    invalid = tmp_path / 'invalid_layout.yaml'
    invalid.write_text('schema_version: 999\n', encoding='utf-8')
    with pytest.raises(ValueError, match='unsupported study cafe layout'):
        load_study_cafe_layout(invalid)
