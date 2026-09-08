from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import yaml

from cleany_mujoco_sim.sorting_scene import add_sorting_bins, load_bins, load_shelf_boxes


CONFIG = Path(__file__).parents[1] / 'config' / 'robot_top_bins.yaml'


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
    assert len(root.findall('.//geom')) == 32
    collision_geoms = [g for g in root.findall('.//geom') if g.get('contype') == '1']
    assert len(collision_geoms) == 16
    rims = [g for g in root.findall('.//geom') if '_rim_' in g.get('name', '')]
    assert len(rims) == 8
    assert all(g.get('mass') == '0' and g.get('conaffinity') == '0' for g in rims)
    assert [b.name for b in bins] == ['lost_items_left', 'trash_right']
    for bin_ in bins:
        assert root.find(f'./worldbody/body[@name="rear_collection_station"]/body[@name="{bin_.name}"]') is not None
        assert bin_.bottom_z == .18
        assert bin_.top_z < .38
        assert bin_.center_xy[0] + bin_.outside_size[0] / 2 <= -.22
        assert abs(bin_.center_xy[1]) + bin_.outside_size[1] / 2 < .32
    assert not any('quarter' in g.get('name', '') or 'paint' in g.get('name', '')
                   for g in root.findall('.//geom'))


def test_rear_shelf_is_world_fixed_and_starts_in_base_frame():
    config = CONFIG.parent / 'robot_top_bins.yaml'
    root = ET.fromstring(add_sorting_bins(
        '<mujoco><worldbody><body name="chassis" pos="1 2 .38" quat="0 0 0 1"/></worldbody></mujoco>', config))
    station = root.find('./worldbody/body[@name="rear_collection_station"]')
    assert station.get('pos') == '1 2 .38'
    assert station.get('quat') == '0 0 0 1'
    assert station.find('joint') is None
    assert root.find('./worldbody/body[@name="chassis"]/body') is None
    assert len(station.findall('geom')) == 6
    boxes = load_shelf_boxes(config)
    assert len(boxes) == 6
    for geom, (name, size, center) in zip(station.findall('geom'), boxes):
        assert name == geom.get('name')
        np.testing.assert_allclose(size, 2*np.fromstring(geom.get('size'), sep=' '))
        np.testing.assert_allclose(center, np.fromstring(geom.get('pos'), sep=' '))


def test_rear_shelf_rejects_unsupported_bins(tmp_path):
    raw = yaml.safe_load((CONFIG.parent / 'robot_top_bins.yaml').read_text())
    raw['rear_shelf']['top_z_m'] = .15
    config = tmp_path / 'bins.yaml'
    config.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match='support'):
        add_sorting_bins('<mujoco><worldbody><body name="chassis"/></worldbody></mujoco>', config)


@pytest.mark.parametrize('color', [[1, 0], [float('nan'), 0, 0, 1], [2, 0, 0, 1]])
def test_invalid_rim_color_rejected(tmp_path, color):
    raw = yaml.safe_load(CONFIG.read_text())
    raw['bins'][0]['rim_rgba'] = color
    config = tmp_path / 'bins.yaml'
    config.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match='rim color'):
        load_bins(config)
