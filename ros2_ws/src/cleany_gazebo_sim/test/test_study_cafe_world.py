from math import cos, hypot, sin
from pathlib import Path
from xml.etree import ElementTree

from cleany_gazebo_sim.world_generator import materialize_study_cafe_world


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROBOT_WORLD = PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_harmonic.sdf'


def test_study_cafe_has_open_room_and_furniture(tmp_path: Path) -> None:
    generated = materialize_study_cafe_world(
        ROBOT_WORLD, tmp_path / 'study_cafe.sdf'
    )
    world = ElementTree.parse(generated).getroot().find(
        "world[@name='cleany_study_cafe']"
    )
    assert world is not None
    assert world.findtext(
        "model[@name='cleany_mecanum']/pose"
    ) == '0.0 -2.7 0.38 0.0 0.0 1.5708'
    assert world.findtext(
        "model[@name='ground_plane']/link/visual/geometry/plane/size"
    ) == '18 10.5'
    assert world.findtext(
        "model[@name='ground_plane']/pose"
    ) == '0 1.75 0 0 0 0'
    assert world.findtext("model[@name='wall_south']/pose") == (
        '0 -3.5 1.25 0 0 0'
    )
    assert world.findtext(
        "model[@name='wall_east']/link/collision/geometry/box/size"
    ) == '0.16 10.5 2.5'
    names = {model.get('name', '') for model in world.findall('model')}
    assert len([name for name in names if name.startswith('adj_table_')]) == 5
    assert len(
        [name for name in names if name.startswith('standard_table_')]
    ) == 6
    assert len(
        [name for name in names if name.startswith('wooden_chair_')]
    ) == 26
    assert len(
        [name for name in names if name.startswith('square_shelf_')]
    ) == 6
    assert len([name for name in names if name.startswith('planter_')]) == 6


def test_robot_is_visible_in_gui_but_excluded_from_lidar(
    tmp_path: Path,
) -> None:
    root = ElementTree.parse(
        materialize_study_cafe_world(
            ROBOT_WORLD, tmp_path / 'masked_robot.sdf'
        )
    ).getroot()
    robot = root.find("./world/model[@name='cleany_mecanum']")
    assert robot is not None
    visuals = robot.findall('.//visual')
    lidars = [
        sensor.find('lidar')
        for sensor in robot.findall(".//sensor[@type='gpu_lidar']")
    ]
    assert visuals
    assert len(lidars) == 4
    assert all(
        visual.findtext('visibility_flags') == '0x02'
        for visual in visuals
    )
    assert all(
        lidar is not None
        and lidar.findtext('visibility_mask') == '0x01'
        for lidar in lidars
    )


def test_fuel_furniture_uses_mesh_visual_and_box_collision(
    tmp_path: Path,
) -> None:
    world = ElementTree.parse(
        materialize_study_cafe_world(ROBOT_WORLD, tmp_path / 'cafe.sdf')
    ).getroot().find('world')
    assert world is not None
    for prefix in ('adj_table_', 'wooden_chair_', 'square_shelf_'):
        furniture = next(
            model
            for model in world.findall('model')
            if model.get('name', '').startswith(prefix)
        )
        uri = furniture.findtext('link/visual/geometry/mesh/uri')
        assert uri is not None and uri.startswith(
            'https://fuel.gazebosim.org/'
        )
        collisions = furniture.findall('link/collision')
        assert collisions
        assert all(
            collision.find('geometry/mesh') is None
            for collision in collisions
        )


def test_study_tables_have_72_cm_top_height(tmp_path: Path) -> None:
    world = ElementTree.parse(
        materialize_study_cafe_world(ROBOT_WORLD, tmp_path / 'cafe.sdf')
    ).getroot().find('world')
    assert world is not None

    adj_table = world.find("model[@name='adj_table_01']")
    standard_table = world.find("model[@name='standard_table_01']")
    assert adj_table is not None
    assert standard_table is not None
    assert adj_table.findtext(
        "link/collision[@name='tabletop_collision']/geometry/box/size"
    ) == '1.6 0.82 0.04'
    assert adj_table.findtext(
        "link/collision[@name='tabletop_collision']/pose"
    ) == '0.0 0.0 0.7 0 0 0'
    z_scale = float(
        adj_table.findtext('link/visual/geometry/mesh/scale', '').split()[2]
    )
    assert abs(0.802432 * z_scale - 0.72) < 1e-6
    assert standard_table.findtext(
        "link/visual[@name='wood_top']/pose"
    ) == '0 0 0.70 0 0 0'
    assert standard_table.findtext(
        "link/visual[@name='wood_top']/geometry/box/size"
    ) == '1.5 0.8 0.04'


def test_tables_and_chairs_use_part_level_primitive_collisions(
    tmp_path: Path,
) -> None:
    world = ElementTree.parse(
        materialize_study_cafe_world(ROBOT_WORLD, tmp_path / 'cafe.sdf')
    ).getroot().find('world')
    assert world is not None
    adj_table = world.find("model[@name='adj_table_01']/link")
    standard_table = world.find("model[@name='standard_table_01']/link")
    chair = world.find("model[@name='wooden_chair_01']/link")
    assert adj_table is not None
    assert standard_table is not None
    assert chair is not None

    for table in (adj_table, standard_table):
        names = {
            collision.get('name') for collision in table.findall('collision')
        }
        assert names == {
            'tabletop_collision',
            'front_left_leg_collision',
            'front_right_leg_collision',
            'back_left_leg_collision',
            'back_right_leg_collision',
        }
    chair_names = {
        collision.get('name') for collision in chair.findall('collision')
    }
    assert chair_names == {
        'seat_collision',
        'backrest_collision',
        'front_left_leg_collision',
        'front_right_leg_collision',
        'back_left_leg_collision',
        'back_right_leg_collision',
    }


def test_every_chair_faces_its_nearest_table(tmp_path: Path) -> None:
    world = ElementTree.parse(
        materialize_study_cafe_world(ROBOT_WORLD, tmp_path / 'cafe.sdf')
    ).getroot().find('world')
    assert world is not None
    tables = [
        model
        for model in world.findall('model')
        if model.get('name', '').startswith(
            ('adj_table_', 'standard_table_')
        )
    ]
    chairs = [
        model
        for model in world.findall('model')
        if model.get('name', '').startswith('wooden_chair_')
    ]

    table_points = [
        tuple(map(float, table.findtext('pose', '').split()[:2]))
        for table in tables
    ]
    for chair in chairs:
        x, y, _, _, _, yaw = map(
            float, chair.findtext('pose', '').split()
        )
        table_x, table_y = min(
            table_points,
            key=lambda point: hypot(point[0] - x, point[1] - y),
        )
        dx = table_x - x
        dy = table_y - y
        distance = hypot(dx, dy)
        alignment = cos(yaw) * dx / distance + sin(yaw) * dy / distance
        assert alignment > 0.999


def test_study_cafe_launch_is_harmonic_gui_profile() -> None:
    source = (
        PACKAGE_ROOT / 'launch' / 'gazebo_study_cafe.launch.py'
    ).read_text(encoding='utf-8')
    assert "'gazebo_harmonic.launch.py'" in source
    assert "'navigation_bridge_harmonic.yaml'" in source
    assert "default_value='false'" in source
    assert "SetEnvironmentVariable('QT_SCALE_FACTOR', '1.0')" in source
