from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import yaml

from cleany_mujoco_sim.sorting_scene import add_sorting_bins, load_bins, load_shelf_boxes


CONFIG = Path(__file__).parents[1] / 'config' / 'robot_top_bins.yaml'


@pytest.mark.parametrize('enabled', [True, False])
def test_simulation_mast_override_preserves_other_contacts(tmp_path, enabled):
    raw = yaml.safe_load(CONFIG.read_text())
    raw['simulation_ignore_mast_collision'] = enabled
    config = tmp_path / 'bins.yaml'
    config.write_text(yaml.safe_dump(raw))
    source = ('<mujoco><worldbody><body name="chassis"><body name="top_base_link">'
              '<geom name="mast" contype="1" conaffinity="1"/>'
              '<body name="head_tilt_link"><geom name="head" contype="1" conaffinity="1"/>'
              '</body></body><body name="arm"><geom name="arm" contype="1" conaffinity="1"/>'
              '</body></body></worldbody></mujoco>')
    result = ET.fromstring(add_sorting_bins(source, config))
    assert result.find(".//geom[@name='mast']").get('contype') == ('0' if enabled else '1')
    assert result.find(".//geom[@name='mast']").get('conaffinity') == ('0' if enabled else '1')
    for name in ('head', 'arm', 'lost_items_left_floor', 'trash_right_floor'):
        assert result.find(f".//geom[@name='{name}']").get('contype') == '1'


def test_current_simulation_disables_mast_but_canonical_model_keeps_it():
    from cleany_mujoco_sim.scene_loader import materialize_control_scene
    root = Path(__file__).parents[1]
    model = mujoco.MjModel.from_xml_path(str(materialize_control_scene(
        root/'scenes/study_cafe_grasp_execution.xml.in', sorting_bins_config=CONFIG)))
    mast = model.geom('top_base_link_collision_0').id
    assert model.geom_contype[mast] == model.geom_conaffinity[mast] == 0
    assert model.geom_contype[model.geom('lost_items_left_floor').id] == 1
    canonical = mujoco.MjModel.from_xml_path(str(root.parent/'cleany_description/mjcf/cleany.xml'))
    assert canonical.geom_contype[canonical.geom('top_base_link_collision_0').id] == 1


def test_nominal_sorting_wrist_transforms_match_both_mujoco_camera_mounts():
    config = yaml.safe_load((CONFIG.parent / 'sorting_wrist_cameras.yaml').read_text())
    model = mujoco.MjModel.from_xml_path(str(
        Path(__file__).parents[2] / 'cleany_description/mjcf/cleany.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert config['source'] == 'simulation_nominal_cad'
    for arm, body in (('left', 'Fixed_Jaw'), ('right', 'Fixed_Jaw_2')):
        mount = config[arm]
        parent = model.body(body).id
        child = model.site(mount['child_frame']).id
        r_parent = data.xmat[parent].reshape(3, 3)
        translation = r_parent.T @ (data.site_xpos[child]-data.xpos[parent])
        rotation = r_parent.T @ data.site_xmat[child].reshape(3, 3)
        qx, qy, qz, qw = mount['quaternion_xyzw']
        configured_rotation = np.empty(9)
        mujoco.mju_quat2Mat(configured_rotation, np.array([qw, qx, qy, qz]))
        assert mount['parent_frame'] == f'{arm}_gripper_frame'
        np.testing.assert_allclose(translation, mount['translation_m'], atol=1e-8)
        np.testing.assert_allclose(rotation, configured_rotation.reshape(3, 3), atol=1e-8)


def test_bin_layout_is_left_lost_right_trash_with_open_tops():
    bins = load_bins(CONFIG)
    assert bins[0].name == 'lost_items_left' and bins[0].center_xy[1] > 0
    assert bins[1].name == 'trash_right' and bins[1].center_xy[1] < 0
    for bin_ in bins:
        assert len(list(bin_.boxes())) == 5
        assert bin_.contains((*bin_.center_xy, bin_.bottom_z + 0.03))
        assert not bin_.contains((*bin_.center_xy, bin_.top_z + 0.01))


@pytest.mark.parametrize('config', [CONFIG])
@pytest.mark.parametrize('index', [0, 1])
def test_bins_physically_retain_released_object_without_weld(index, config):
    bin_ = load_bins(config)[index]
    root = ET.fromstring(add_sorting_bins(
        '<mujoco><option timestep="0.002"/>'
        '<worldbody><body name="chassis"/></worldbody></mujoco>', config,
    ))
    body = ET.SubElement(root.find('worldbody'), 'body', {
        'name': 'dropped_item',
        'pos': f'{bin_.center_xy[0]} {bin_.center_xy[1]} {bin_.top_z + .1}',
    })
    ET.SubElement(body, 'freejoint')
    ET.SubElement(body, 'geom', {
        'type': 'box', 'size': '.02 .02 .02', 'mass': '.05',
    })
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    assert model.neq == 0
    for _ in range(1500):
        mujoco.mj_step(model, data)
    assert bin_.contains(data.xpos[model.body('dropped_item').id])
    assert np.linalg.norm(data.qvel) < 0.01


def test_robot_rear_bins_have_no_table_markings():
    assert not (CONFIG.parent / 'table_sorting_zones.yaml').exists()
    assert not (CONFIG.parent / 'sorting_bins.yaml').exists()
    config = CONFIG.parent / 'robot_top_bins.yaml'
    bins = load_bins(config)
    root = ET.fromstring(add_sorting_bins(
        '<mujoco><worldbody><body name="chassis"/></worldbody></mujoco>', config))
    assert len(root.findall('.//geom')) == 27
    collision_geoms = [g for g in root.findall('.//geom') if g.get('contype') == '1']
    assert len(collision_geoms) == 11
    rims = [g for g in root.findall('.//geom') if '_rim_' in g.get('name', '')]
    assert len(rims) == 8
    assert all(g.get('mass') == '0' and g.get('conaffinity') == '0' for g in rims)
    assert [b.name for b in bins] == ['lost_items_left', 'trash_right']
    for bin_ in bins:
        assert root.find(f'./worldbody/body[@name="chassis"]/body[@name="{bin_.name}"]') is not None
        assert bin_.bottom_z == .22
        assert bin_.top_z < .38
        assert bin_.center_xy[0] - bin_.outside_size[0] / 2 > -.18
        assert bin_.center_xy[0] + bin_.outside_size[0] / 2 < .036
        assert abs(bin_.center_xy[1]) + bin_.outside_size[1] / 2 < .23
    assert not any('quarter' in g.get('name', '') or 'paint' in g.get('name', '')
                   for g in root.findall('.//geom'))


def test_internal_tray_follows_chassis_and_matches_planning_boxes():
    config = CONFIG.parent / 'robot_top_bins.yaml'
    root = ET.fromstring(add_sorting_bins(
        '<mujoco><worldbody><body name="chassis" pos="1 2 .38" quat="0 0 0 1"/></worldbody></mujoco>', config))
    assert root.find('./worldbody/body[@name="rear_collection_station"]') is None
    station = root.find('./worldbody/body[@name="chassis"]')
    assert station.get('pos') == '1 2 .38'
    assert station.get('quat') == '0 0 0 1'
    assert station.find('joint') is None
    assert len(station.findall('body')) == 2
    assert len(station.findall('geom')) == 1
    boxes = load_shelf_boxes(config)
    assert len(boxes) == 1
    for geom, (name, size, center) in zip(station.findall('geom'), boxes):
        assert name == geom.get('name')
        np.testing.assert_allclose(size, 2*np.fromstring(geom.get('size'), sep=' '))
        np.testing.assert_allclose(center, np.fromstring(geom.get('pos'), sep=' '))


def test_internal_tray_rejects_unsupported_bins(tmp_path):
    raw = yaml.safe_load((CONFIG.parent / 'robot_top_bins.yaml').read_text())
    raw['internal_tray']['top_z_m'] = .15
    config = tmp_path / 'bins.yaml'
    config.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match='support'):
        add_sorting_bins('<mujoco><worldbody><body name="chassis"/></worldbody></mujoco>', config)


def test_internal_fixtures_clear_chassis_box_collisions():
    """Check frame rails/electronics, not arbitrary arm poses or CAD meshes."""
    root = ET.parse(Path(__file__).parents[2] /
                    'cleany_description/urdf/physical_properties.xacro')
    fixtures = list(load_shelf_boxes(CONFIG))
    for bin_ in load_bins(CONFIG):
        fixtures.extend(bin_.boxes())
    checked = 0
    for collision in root.findall('.//collision'):
        box = collision.find('geometry/box')
        if box is None:
            continue
        origin = collision.find('origin')
        # This macro also contains articulated link geometry; use chassis frame IDs.
        name = collision.get('name', '')
        if not name.startswith('mjcf_geom_'):
            continue
        index = int(name.split('_')[2])
        if not 27 <= index <= 47:
            continue
        center = np.fromstring(origin.get('xyz'), sep=' ')
        half = np.fromstring(box.get('size'), sep=' ') / 2
        for fixture_name, size, pos in fixtures:
            overlap = half + np.array(size)/2 - np.abs(center-np.array(pos))
            assert not np.all(overlap > 1e-8), (name, fixture_name, overlap)
        checked += 1
    assert checked >= 19


def test_internal_and_external_support_are_mutually_exclusive(tmp_path):
    raw = yaml.safe_load(CONFIG.read_text())
    raw['rear_shelf'] = {}
    config = tmp_path / 'bins.yaml'
    config.write_text(yaml.safe_dump(raw))
    for operation in (load_shelf_boxes, lambda path: add_sorting_bins(
            '<mujoco><worldbody><body name="chassis"/></worldbody></mujoco>', path)):
        with pytest.raises(ValueError, match='either'):
            operation(config)


@pytest.mark.parametrize('color', [[1, 0], [float('nan'), 0, 0, 1], [2, 0, 0, 1]])
def test_invalid_rim_color_rejected(tmp_path, color):
    raw = yaml.safe_load(CONFIG.read_text())
    raw['bins'][0]['rim_rgba'] = color
    config = tmp_path / 'bins.yaml'
    config.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match='rim color'):
        load_bins(config)
