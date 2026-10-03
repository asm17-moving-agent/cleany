from copy import deepcopy
import json
from math import hypot
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import yaml

from cleany_gazebo_sim.world.facility_generator import materialize_facility_world
from cleany_gazebo_sim.world.facility_layout import load_facility_layout
from cleany_gazebo_sim.world.generator import materialize_study_cafe_world
from cleany_gazebo_sim.world.roly import load_roly_config

PACKAGE = Path(__file__).resolve().parents[1]
TEMPLATE = PACKAGE / 'worlds/cleany_mecanum_fortress.sdf'
LAYOUT = PACKAGE / 'config/facility_18f/facility_layout.yaml'
ROLY = PACKAGE / 'config/furniture/roly_p1g210m.yaml'


@pytest.fixture(scope='module')
def generated(tmp_path_factory):
    path = materialize_facility_world(
        TEMPLATE, tmp_path_factory.mktemp('facility') / 'world.sdf', robot_model='legacy'
    )
    return path, ET.parse(path).getroot().find('world')


def test_default_facility_uses_canonical_cad_with_drive_and_sensors(tmp_path):
    path = materialize_facility_world(TEMPLATE, tmp_path / 'cad.sdf')
    robot = ET.parse(path).getroot().find("world/model[@name='cleany_mecanum']")
    meshes = [mesh.text for mesh in robot.findall('.//mesh/uri')]
    assert any('/meshes/cad_frame/' in mesh for mesh in meshes)
    assert not any('raskog' in mesh.lower() for mesh in meshes)
    assert robot.find("joint[@name='head_pan_joint']") is not None
    for wheel in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        assert robot.find(f"joint[@name='{wheel}_wheel_joint']") is not None
    assert robot.find("plugin[@name='ignition::gazebo::systems::MecanumDrive']") is not None
    assert {'gpu_lidar', 'imu', 'camera', 'depth_camera'} <= {
        sensor.get('type') for sensor in robot.findall('.//sensor')
    }
    metadata = json.loads(path.with_suffix('.model.json').read_text())
    assert metadata['source_commit'] == '71f8d6c'
    assert metadata['collision_bounds'][1][1] > 0.3
    assert len(json.loads(path.with_suffix('.seats.json').read_text())) == 82


def test_full_facility_preserves_dhub_and_exposes_82_seat_ids(generated, tmp_path):
    path, world = generated
    old = (
        ET.parse(materialize_study_cafe_world(TEMPLATE, tmp_path / 'legacy.sdf'))
        .getroot()
        .find('world')
    )
    assert world.get('name') == 'cleany_facility_18f'
    assert world.find("model[@name='wall_south']") is None
    for i in range(1, 49):
        for prefix in ('demo_desk', 'desk_monitor', 'office_chair'):
            query = f"model[@name='{prefix}_{i:02d}']/pose"
            assert world.findtext(query) == old.findtext(query)
    seats = json.loads(path.with_suffix('.seats.json').read_text())
    assert len(seats) == 82
    assert len({seat['seat_id'] for seat in seats}) == 82
    assert {seat['seat_id'] for seat in seats} >= {
        'A41',
        'A44',
        'M11',
        'M36',
        '01',
        '48',
    }
    assert len(world.findall("model/link/visual[@name='roly_frame']")) == 48
    assert len(world.findall("model/link/visual[@name='office_chair_visual']")) == 34


def test_coordinate_anchors_and_round_trip():
    layout = load_facility_layout(LAYOUT)
    assert layout.world([576, 8]) == pytest.approx((-6.13, 5.47))
    assert layout.world([976, 364.93311582381725]) == pytest.approx((6.13, -5.47))
    for point in ((0, 0), (-20, -12), (5, 3)):
        assert layout.world(layout.map_point(point)) == pytest.approx(point)


def test_ground_table_and_open_glass_doors_have_physical_geometry(generated):
    _, world = generated
    assert not any('central_obstacle' in m.get('name') for m in world.findall('model'))
    table = world.find("model[@name='facility_ground_table']")
    top = table.find("link/collision[@name='solid_body_collision']")
    z = float(top.findtext('pose').split()[2])
    thickness = float(top.findtext('geometry/box/size').split()[2])
    assert z + thickness / 2 == pytest.approx(0.8)
    assert z - thickness / 2 == pytest.approx(0)
    assert len(table.findall('link/collision')) == 1
    assert not any('stair_guard' in m.get('name') for m in world.findall('model'))
    layout = load_facility_layout(LAYOUT)
    for door in layout.raw['doors']:
        if door.get('material') != 'glass':
            continue
        leaf = world.find(f"model[@name='facility_door_{door['id']}']")
        assert leaf.find("link/collision[@name='glass_collision']") is not None
        assert float(leaf.findtext("link/visual[@name='glass_visual']/transparency")) > 0
        a, b = (layout.world(p) for p in door['points'])
        opening_center = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        leaf_xy = tuple(map(float, leaf.findtext('pose').split()[:2]))
        # A 90 degree open leaf is away from the center of the doorway.
        assert hypot(*(x - y for x, y in zip(opening_center, leaf_xy))) > 0.45
    for wall in layout.segments:
        if wall.material == 'glass':
            model = world.find(f"model[@name='facility_wall_{wall.name}']")
            assert model.find("link/collision[@name='glass_collision']") is not None


def test_ceiling_has_downward_faces_and_solid_collision(generated):
    path, world = generated
    ceiling = world.find("model[@name='facility_ceiling']")
    assert float(ceiling.findtext('pose').split()[2]) == pytest.approx(2.7)
    assert ceiling.find('link/collision/geometry/box') is not None
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    mesh = ET.parse(path.with_suffix('.ceiling.dae'))
    vertices = list(map(float, mesh.find(".//c:source[@id='positions']/c:float_array", ns).text.split()))
    indices = list(map(int, mesh.find('.//c:triangles/c:p', ns).text.split()))[::2]
    for i in range(0, len(indices), 3):
        a, b, c = [vertices[j * 3:j * 3 + 3] for j in indices[i:i + 3]]
        assert (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) < 0


def point_segment_distance(point, segment):
    a, b = segment.start, segment.end
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(
        0,
        min(1, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / (dx * dx + dy * dy)),
    )
    return hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def test_doors_and_representative_route_are_clear_of_walls():
    layout = load_facility_layout(LAYOUT)
    half = layout.raw['wall']['thickness_m'] / 2
    for door in layout.raw['doors']:
        a, b = (layout.world(p) for p in door['points'])
        center = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        assert (
            min(point_segment_distance(center, wall) for wall in layout.segments) - half
            > 0.27
        ), door['id']
    for a, b in zip(layout.raw['route_xy'], layout.raw['route_xy'][1:]):
        n = max(1, int(hypot(b[0] - a[0], b[1] - a[1]) / 0.03))
        for i in range(n + 1):
            point = (a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
            assert (
                min(point_segment_distance(point, wall) for wall in layout.segments)
                - half
                > 0.32
            ), point


def test_roly_has_discrete_casters_arms_and_local_visual_meshes(generated):
    _, world = generated
    chair = world.find("model[@name='office_chair_01']")
    names = {x.get('name') for x in chair.findall('link/collision')}
    assert 'caster_base_collision' not in names
    assert len(chair.findall('link/visual')) <= 6
    assert len([name for name in names if name.startswith('spoke_')]) == 5
    assert (
        len(
            [
                name
                for name in names
                if name.startswith('caster_') and not name.startswith('caster_stem')
            ]
        )
        == 10
    )
    assert {'arm_pad_-1_collision', 'arm_pad_1_collision'} <= names
    assert all(uri.text.startswith('file://') for uri in chair.findall('.//mesh/uri'))
    assert (
        world.find("model[@name='cleany_mecanum']//sensor[@type='gpu_lidar']")
        is not None
    )
    assert load_roly_config(ROLY).number('arm_top_m') < 0.68  # existing desk underside


@pytest.mark.parametrize('mutation', ['zero', 'nan', 'duplicate', 'arc'])
def test_invalid_facility_geometry_rejected(tmp_path, mutation):
    raw = deepcopy(yaml.safe_load(LAYOUT.read_text()))
    if mutation == 'zero':
        raw['walls'][0]['points'][1] = raw['walls'][0]['points'][0]
    elif mutation == 'nan':
        raw['transform']['units_per_meter'] = float('nan')
    elif mutation == 'duplicate':
        raw['walls'].append(raw['walls'][0])
    else:
        raw['arcs'][0]['segments'] = 0
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError):
        load_facility_layout(path)


def test_roly_bad_dimensions_rejected(tmp_path):
    raw = yaml.safe_load(ROLY.read_text())
    raw['caster_radius_m'] = -1
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match='ROLY'):
        load_roly_config(path)


def test_legacy_chair_profile_and_lidar_override(tmp_path):
    path = materialize_facility_world(
        TEMPLATE,
        tmp_path / 'legacy.sdf',
        chair_model='legacy',
        robot_model='legacy',
        lidar_translation=(0.16, 0.0, 0.32),
    )
    world = ET.parse(path).getroot().find('world')
    assert len(world.findall("model/link/visual[@name='office_chair_visual']")) == 82
    assert (
        world.findtext("model[@name='cleany_mecanum']/joint[@name='lidar_mount']/pose")
        == '0.16 0.0 0.32 0.0 0.0 0.0'
    )


def test_joint_state_transport_is_independent_of_world_name(generated):
    _, world = generated
    plugin = world.find(
        "model[@name='cleany_mecanum']/plugin[@name='ignition::gazebo::systems::JointStatePublisher']"
    )
    rows = yaml.safe_load((PACKAGE / 'config/bridge/core_bridge.yaml').read_text())
    bridge = next(row for row in rows if row['ros_topic_name'] == '/joint_states')
    assert (
        plugin.findtext('topic')
        == bridge['gz_topic_name']
        == '/model/cleany_mecanum/joint_state'
    )


def test_batched_visuals_keep_material_colors_and_world_local_assets(generated):
    path, world = generated
    chair = world.find("model[@name='office_chair_01']")
    colors = {
        visual.findtext('material/diffuse') for visual in chair.findall('link/visual')
    }
    assert load_roly_config(ROLY).color('seat') in colors
    assert load_roly_config(ROLY).color('mesh') in colors
    from urllib.parse import unquote, urlparse

    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    for uri in chair.findall('link/visual/geometry/mesh/uri'):
        asset = Path(unquote(urlparse(uri.text).path))
        assert asset.parent == path.parent / 'roly_meshes'
        root = ET.parse(asset).getroot()
        positions = list(
            map(
                float,
                root.find(
                    ".//c:source[@id='positions']/c:float_array", ns
                ).text.split(),
            )
        )
        normals = list(
            map(
                float,
                root.find(".//c:source[@id='normals']/c:float_array", ns).text.split(),
            )
        )
        assert len(positions) == len(normals) > 0
        assert all(__import__('math').isfinite(v) for v in positions + normals)
        assert int(root.find('.//c:triangles', ns).get('count')) > 0
    for texture in chair.findall('.//albedo_map'):
        assert Path(unquote(urlparse(texture.text).path)).is_file()
    # 48 seats share the same six resources, so geometry is not duplicated.
    assert (
        len(
            {
                uri.text
                for uri in world.findall('model/link/visual/geometry/mesh/uri')
                if 'roly-' in uri.text
            }
        )
        == 6
    )
