from math import cos, hypot, sin
from pathlib import Path
from xml.etree import ElementTree

import pytest

from cleany_gazebo_sim.world.generator import materialize_study_cafe_world
from cleany_gazebo_sim.world.layout import load_study_cafe_layout


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
ROBOT_WORLD = PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_harmonic.sdf'
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


def test_study_cafe_matches_demo_room_and_seat_count(
    tmp_path: Path,
) -> None:
    world = _world(tmp_path)
    assert world.findtext(
        "model[@name='cleany_mecanum']/pose"
    ) == '-1.865 -4.705 0.38 0.0 0.0 1.5708'
    assert tuple(map(float, world.findtext(
        "model[@name='ground_plane']/link/visual/geometry/plane/size", ''
    ).split())) == (12.42, 11.10)
    assert world.findtext("model[@name='wall_south']/pose") == (
        '0 -5.55 1.25 0 0 0'
    )
    assert world.findtext(
        "model[@name='wall_east']/link/collision/geometry/box/size"
    ) == '0.16 10.94 2.5'
    assert tuple(map(float, world.findtext('scene/ambient', '').split())) == (
        0.65, 0.65, 0.65, 1.0
    )
    assert tuple(map(
        float, world.findtext('scene/background', '').split()
    )) == (0.82, 0.83, 0.84, 1.0)

    names = {model.get('name', '') for model in world.findall('model')}
    assert len(
        [name for name in names if name.startswith('demo_desk_')]
    ) == 48
    assert len(
        [name for name in names if name.startswith('office_chair_')]
    ) == 48
    assert len(
        [name for name in names if name.startswith('desk_partition_')]
    ) == 24
    assert len(
        [name for name in names if name.startswith('desk_monitor_')]
    ) == 48
    assert not any(name.startswith('ceiling_') for name in names)


def test_study_cafe_supports_bounded_accelerated_physics(
    tmp_path: Path,
) -> None:
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


def test_robot_is_visible_in_gui_but_excluded_from_lidar(
    tmp_path: Path,
) -> None:
    world = _world(tmp_path)
    robot = world.find("model[@name='cleany_mecanum']")
    assert robot is not None
    visuals = robot.findall('.//visual')
    lidars = [
        sensor.find('lidar')
        for sensor in robot.findall(".//sensor[@type='gpu_lidar']")
    ]
    assert visuals
    assert len(lidars) == 1
    assert all(
        visual.findtext('visibility_flags') == '0x02'
        for visual in visuals
    )
    assert all(
        lidar is not None
        and lidar.findtext('visibility_mask') == '0x01'
        for lidar in lidars
    )


def test_office_chair_uses_fuel_visual_and_primitive_collisions(
    tmp_path: Path,
) -> None:
    world = _world(tmp_path)
    chair = world.find("model[@name='office_chair_01']")
    assert chair is not None
    assert chair.findtext('link/visual/geometry/mesh/uri') == (
        'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
        'OfficeChairGrey/1/files/meshes/OfficeChairGrey.obj'
    )
    assert chair.findtext('link/visual/geometry/mesh/scale') == '0.9 0.9 0.9'
    collisions = chair.findall('link/collision')
    assert {
        collision.get('name') for collision in collisions
    } == {
        'caster_base_collision',
        'center_column_collision',
        'seat_collision',
        'backrest_collision',
    }
    assert all(
        collision.find('geometry/mesh') is None
        for collision in collisions
    )


def test_desks_have_white_72_cm_top_and_a_frame_legs(
    tmp_path: Path,
) -> None:
    world = _world(tmp_path)
    desk = world.find("model[@name='demo_desk_01']/link")
    assert desk is not None
    assert desk.findtext(
        "collision[@name='tabletop_back_collision']/geometry/box/size"
    ) == '1.2 0.71 0.04'
    assert desk.findtext(
        "collision[@name='tabletop_back_collision']/pose"
    ) == '0.0 -0.03 0.7 0.0 0.0 0.0'
    assert desk.findtext(
        "visual[@name='tabletop_back_visual']/material/diffuse"
    ) == '0.92 0.93 0.94 1'

    names = {
        collision.get('name') for collision in desk.findall('collision')
    }
    assert names == {
        'tabletop_back_collision',
        'tabletop_front_center_collision',
        'tabletop_front_left_corner_collision',
        'tabletop_front_right_corner_collision',
        'left_front_leg_collision',
        'left_back_leg_collision',
        'right_front_leg_collision',
        'right_back_leg_collision',
        'upper_crossbar_collision',
    }
    leg_poses = [
        collision.findtext('pose', '').split()
        for collision in desk.findall('collision')
        if '_leg_collision' in collision.get('name', '')
    ]
    assert len(leg_poses) == 4
    assert all(abs(float(pose[3])) > 0.2 for pose in leg_poses)
    assert sorted({abs(float(pose[0])) for pose in leg_poses}) == [0.52]

    corner_collisions = [
        collision for collision in desk.findall('collision')
        if 'tabletop_front_' in collision.get('name', '')
        and collision.find('geometry/cylinder') is not None
    ]
    assert len(corner_collisions) == 2
    assert {
        collision.findtext('geometry/cylinder/radius')
        for collision in corner_collisions
    } == {'0.06'}
    assert {
        tuple(round(float(value), 2) for value in collision.findtext(
            'pose', ''
        ).split()[:3])
        for collision in corner_collisions
    } == {(-0.54, 0.33, 0.70), (0.54, 0.33, 0.70)}

    opposite_desk = world.find("model[@name='demo_desk_09']/link")
    assert opposite_desk is not None
    assert {
        round(float(collision.findtext('pose', '').split()[1]), 2)
        for collision in opposite_desk.findall('collision')
        if 'tabletop_front_' in collision.get('name', '')
        and collision.find('geometry/cylinder') is not None
    } == {-0.33}


def test_partitions_span_30_cm_above_floor_to_30_cm_above_desk(
    tmp_path: Path,
) -> None:
    world = _world(tmp_path)
    partition = world.find("model[@name='desk_partition_01']")
    assert partition is not None
    assert partition.findtext('pose') == '-5.53 3.17 0.66 0.0 0.0 0.0'
    collisions = partition.findall('link/collision')
    assert {
        collision.get('name') for collision in collisions
    } == {
        'partition_center_collision',
        'partition_middle_collision',
        'partition_bottom_left_corner_collision',
        'partition_top_left_corner_collision',
        'partition_bottom_right_corner_collision',
        'partition_top_right_corner_collision',
    }
    corners = [
        collision for collision in collisions
        if collision.find('geometry/cylinder') is not None
    ]
    assert len(corners) == 4
    assert all(
        collision.findtext('geometry/cylinder/radius') == '0.05'
        and collision.findtext('geometry/cylinder/length') == '0.025'
        and round(float(collision.findtext('pose', '').split()[3]), 5)
        == round(3.141592653589793 / 2.0, 5)
        for collision in corners
    )
    center_z = float(partition.findtext('pose', '').split()[2])
    height = 0.72
    assert round(center_z - height / 2.0, 2) == 0.30
    assert round(center_z + height / 2.0, 2) == 1.02


def test_each_desk_has_27_inch_monitor_facing_its_chair(
    tmp_path: Path,
) -> None:
    world = _world(tmp_path)
    monitors = [
        model for model in world.findall('model')
        if model.get('name', '').startswith('desk_monitor_')
    ]
    assert len(monitors) == 48

    first = world.find("model[@name='desk_monitor_01']")
    assert first is not None
    link = first.find('link')
    assert link is not None
    assert {
        collision.get('name') for collision in link.findall('collision')
    } == {
        'monitor_panel_collision',
        'monitor_stem_collision',
        'monitor_base_collision',
    }
    assert link.findtext(
        "visual[@name='monitor_panel_visual']/geometry/box/size"
    ) == '0.62 0.035 0.36'
    screen_size = tuple(map(float, link.findtext(
        "visual[@name='monitor_screen_visual']/geometry/box/size", ''
    ).split()))
    assert round(hypot(screen_size[0], screen_size[2]) / 0.0254, 1) == 27.0
    assert link.findtext(
        "visual[@name='monitor_panel_visual']/material/diffuse"
    ) == '0.025 0.025 0.03 1'

    for monitor in monitors:
        suffix = monitor.get('name', '').removeprefix('desk_monitor_')
        desk = world.find(f"model[@name='demo_desk_{suffix}']")
        chair = world.find(f"model[@name='office_chair_{suffix}']")
        assert desk is not None
        assert chair is not None
        monitor_y = float(monitor.findtext('pose', '').split()[1])
        desk_y = float(desk.findtext('pose', '').split()[1])
        chair_y = float(chair.findtext('pose', '').split()[1])
        screen_y = float(monitor.findtext(
            "link/visual[@name='monitor_screen_visual']/pose", ''
        ).split()[1])
        front_sign = 1.0 if screen_y > 0 else -1.0
        opposite_edge_y = desk_y - front_sign * 0.385
        assert round(abs(monitor_y - opposite_edge_y), 2) == 0.10
        assert screen_y * (chair_y - monitor_y) > 0.0


def test_walls_use_white_matte_gypsum_material(tmp_path: Path) -> None:
    world = _world(tmp_path)
    for name in ('wall_north', 'wall_south', 'wall_east', 'wall_west'):
        wall = world.find(f"model[@name='{name}']")
        assert wall is not None
        assert tuple(map(float, wall.findtext(
            'link/visual/material/ambient', ''
        ).split())) == (0.97, 0.97, 0.96, 1.0)
        assert tuple(map(float, wall.findtext(
            'link/visual/material/diffuse', ''
        ).split())) == (0.97, 0.97, 0.96, 1.0)
        assert wall.findtext('link/visual/material/specular') == (
            '0.03 0.03 0.03 1'
        )
        assert wall.findtext(
            'link/visual/material/pbr/metal/roughness'
        ) == '0.92'
        assert wall.findtext(
            'link/visual/material/pbr/metal/metalness'
        ) == '0.0'


def test_desks_follow_3_2_3_wall_touching_layout(tmp_path: Path) -> None:
    world = _world(tmp_path)
    desks = [
        model for model in world.findall('model')
        if model.get('name', '').startswith('demo_desk_')
    ]
    points = [
        tuple(map(float, desk.findtext('pose', '').split()[:2]))
        for desk in desks
    ]
    xs = sorted({point[0] for point in points})
    ys = sorted({round(point[1], 3) for point in points})
    assert xs == [-5.53, -4.33, -3.13, -0.6, 0.6, 3.13, 4.33, 5.53]
    assert ys == [-3.555, -2.785, -0.385, 0.385, 2.785, 3.555]

    # The first and last desk sides touch the side-wall faces.
    assert round(xs[0] - 0.6, 2) == -6.13
    assert round(xs[-1] + 0.6, 2) == 6.13
    # Desk blocks are 3-2-3 with measured 1.33 m clear horizontal aisles.
    clear_aisles = [
        round(xs[index + 1] - xs[index] - 1.2, 2)
        for index in (2, 4)
    ]
    assert clear_aisles == [1.33, 1.33]
    # Each two-row block touches back-to-back.
    assert [round(ys[index + 1] - ys[index], 2) for index in (0, 2, 4)] == [
        0.77, 0.77, 0.77
    ]
    clear_row_gaps = [
        round(ys[index + 1] - ys[index] - 0.77, 2)
        for index in (1, 3)
    ]
    assert clear_row_gaps == [1.63, 1.63]
    # Measured edge-to-wall clearance at the north and south rows is 1.53 m.
    assert round(5.47 - (ys[-1] + 0.385), 2) == 1.53
    assert round((ys[0] - 0.385) - (-5.47), 2) == 1.53


def test_chairs_overlap_tabletop_edge_by_23_cm(
    tmp_path: Path,
) -> None:
    world = _world(tmp_path)
    chairs = [
        model for model in world.findall('model')
        if model.get('name', '').startswith('office_chair_')
    ]
    chair_ys = sorted({
        round(float(chair.findtext('pose', '').split()[1]), 2)
        for chair in chairs
    })
    assert chair_ys == [-3.94, -2.4, -0.77, 0.77, 2.4, 3.94]

    for chair in chairs:
        suffix = chair.get('name', '').removeprefix('office_chair_')
        desk = world.find(f"model[@name='demo_desk_{suffix}']")
        assert desk is not None
        chair_y = float(chair.findtext('pose', '').split()[1])
        desk_y = float(desk.findtext('pose', '').split()[1])
        # Tabletop half-depth is 0.385 m. The chair seat front reaches
        # 0.23 m forward from its origin after accounting for its -0.03 m
        # local offset and 0.52 m front-to-back collision size.
        edge_to_seat_front = abs(chair_y - desk_y) - 0.385 - 0.23
        assert round(edge_to_seat_front, 2) == -0.23

    # Backrests extend 0.40 m away from the chair origin.
    chair_back_extent = 0.40
    cross_aisles = (
        chair_ys[2] - chair_back_extent
        - (chair_ys[1] + chair_back_extent),
        chair_ys[4] - chair_back_extent
        - (chair_ys[3] + chair_back_extent),
    )
    perimeter_aisles = (
        chair_ys[0] - chair_back_extent - (-5.47),
        5.47 - (chair_ys[-1] + chair_back_extent),
    )
    assert tuple(round(width, 2) for width in cross_aisles) == (0.83, 0.83)
    assert tuple(round(width, 2) for width in perimeter_aisles) == (1.13, 1.13)


def test_every_chair_faces_its_assigned_desk(tmp_path: Path) -> None:
    world = _world(tmp_path)
    chairs = [
        model for model in world.findall('model')
        if model.get('name', '').startswith('office_chair_')
    ]
    for chair in chairs:
        suffix = chair.get('name', '').removeprefix('office_chair_')
        desk = world.find(f"model[@name='demo_desk_{suffix}']")
        assert desk is not None
        x, y, _, _, _, yaw = map(
            float, chair.findtext('pose', '').split()
        )
        desk_x, desk_y = map(
            float, desk.findtext('pose', '').split()[:2]
        )
        dx = desk_x - x
        dy = desk_y - y
        distance = hypot(dx, dy)
        alignment = cos(yaw) * dx / distance + sin(yaw) * dy / distance
        assert alignment > 0.999


def test_study_cafe_launch_is_harmonic_gui_profile() -> None:
    source = (
        PACKAGE_ROOT / 'launch' / 'gazebo_study_cafe.launch.py'
    ).read_text(encoding='utf-8')
    assert "'gazebo_harmonic.launch.py'" in source
    assert 'declare_sensor_profile_argument' in source
    assert "'sensor_profile': LaunchConfiguration('sensor_profile')" in source
    assert "default_value=''" in source
    assert "default_value='false'" in source
    assert "SetEnvironmentVariable('QT_SCALE_FACTOR', '1.0')" in source
    assert "'physics_max_step_size'" in source
    assert "'physics_real_time_factor'" in source
    assert "'layout_config'" in source
    assert "'study_cafe_layout.yaml'" in source


def test_study_cafe_layout_is_human_readable_and_validated(
    tmp_path: Path,
) -> None:
    layout = load_study_cafe_layout(LAYOUT_CONFIG)
    assert layout.room.inside_size_m == (12.26, 10.94)
    assert layout.desks.x_positions_m == (
        -5.53, -4.33, -3.13, -0.60, 0.60, 3.13, 4.33, 5.53
    )
    assert layout.desks.row_pair_centers_y_m == (3.17, 0.0, -3.17)
    assert len(layout.desks.rows) == 2

    invalid = tmp_path / 'invalid_layout.yaml'
    invalid.write_text('schema_version: 999\n', encoding='utf-8')
    with pytest.raises(ValueError, match='unsupported study cafe layout'):
        load_study_cafe_layout(invalid)
