from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from cleany_mujoco_sim.scene_loader import materialize_scene
from cleany_mujoco_sim.study_cafe_scene import load_study_cafe_layout
from cleany_mujoco_sim.tabletop_shapes import (
    add_shape_assets, add_shape_geoms, cup_vertices, parse_shape, tissue_vertices,
)


ROOT = Path(__file__).parents[1]


@pytest.fixture(scope='module')
def model():
    return mujoco.MjModel.from_xml_path(str(materialize_scene(ROOT/'scenes/study_cafe.xml.in')))


def test_cup_is_physically_open_not_a_solid_collision_cylinder(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    cup = model.body('study_cafe_cup').id
    origin = data.xpos[cup]+np.array((0., 0., .12))
    hit = np.array([-1], dtype=np.int32)
    # mj_ray ignores alpha=0 even for a selected collision group. Expose only
    # for this geometry diagnostic, and restore the shared fixture immediately.
    alpha = model.geom_rgba[:,3].copy()
    try:
        model.geom_rgba[:,3] = 1.
        distance = mujoco.mj_ray(model, data, origin, np.array((0.,0.,-1.)),
                                np.array((0,0,0,1,0,0), dtype=np.uint8), True, -1, hit)
    finally:
        model.geom_rgba[:,3] = alpha
    assert hit[0] == model.geom('study_cafe_cup_collision').id
    assert distance == pytest.approx(.12-.000504, abs=1e-7)
    assert model.body_mass[cup] == pytest.approx(.008)


def test_cup_has_eighty_percent_external_dimensions():
    item = next(i for i in load_study_cafe_layout(ROOT/'config/study_cafe_layout.yaml').tabletop_objects
                if i.name == 'cup')
    vertices, _ = cup_vertices(item.collision_size_m, item.shape)
    np.testing.assert_allclose(np.ptp(np.asarray(vertices), axis=0), (.0612, .0612, .0684), atol=1e-9)
    assert item.shape.number('bottom_diameter_m') == pytest.approx(.0396)
    assert item.shape.number('wall_thickness_m') == pytest.approx(.000504)


def test_cup_support_clearance_preserves_visual_and_upper_gripping_surface():
    item = next(i for i in load_study_cafe_layout(ROOT/'config/study_cafe_layout.yaml').tabletop_objects
                if i.name == 'cup')
    legacy_parameters = dict(item.shape.parameters)
    legacy_parameters.pop('collision_bottom_clearance_m')
    legacy = parse_shape('paper_cup', legacy_parameters, 'compound', item.collision_size_m)
    visual = cup_vertices(item.collision_size_m, item.shape)
    assert visual == cup_vertices(item.collision_size_m, legacy)
    assert visual == cup_vertices(item.collision_size_m, legacy, collision=True)
    collision, _ = cup_vertices(item.collision_size_m, item.shape, collision=True)
    count = int(item.shape.number('segments'))
    # The other three rings (rim and inner wall) are untouched.
    assert collision[count:] == visual[0][count:]
    clearance = item.shape.number('collision_bottom_clearance_m')
    assert 0 < clearance < item.shape.number('wall_thickness_m')
    assert min(v[2] for v in collision) == pytest.approx(clearance)


def test_cup_collision_has_no_gaps_at_gripping_height(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    cup = model.body('study_cafe_cup').id
    origin = data.xpos[cup]+np.array((0., 0., .035))
    hit = np.array([-1], dtype=np.int32)
    alpha = model.geom_rgba[:, 3].copy()
    try:
        model.geom_rgba[:, 3] = 1.
        for angle in np.linspace(0., 2*np.pi, 128, endpoint=False):
            direction = np.array((np.cos(angle), np.sin(angle), 0.))
            distance = mujoco.mj_ray(model, data, origin, direction,
                                    np.array((0, 0, 0, 1, 0, 0), dtype=np.uint8), True, -1, hit)
            assert .02 < distance < .03
            assert model.geom_bodyid[hit[0]] == cup
    finally:
        model.geom_rgba[:, 3] = alpha


@pytest.mark.parametrize('clearance', [-.0001, .001, float('nan'), float('inf'), None])
def test_invalid_cup_support_clearance_is_rejected(clearance):
    raw = dict(bottom_diameter_m=.0396, wall_thickness_m=.000504, segments=32,
               collision_bottom_clearance_m=clearance)
    with pytest.raises(ValueError):
        parse_shape('paper_cup', raw, 'compound', (.0612, .0684))


def test_cup_stays_supported_with_fewer_floor_contacts():
    item = next(i for i in load_study_cafe_layout(ROOT/'config/study_cafe_layout.yaml').tabletop_objects
                if i.name == 'cup')
    counts = []
    for clearance in (0., item.shape.number('collision_bottom_clearance_m')):
        shape = parse_shape('paper_cup', {**item.shape.parameters,
                            'collision_bottom_clearance_m': clearance}, 'compound', item.collision_size_m)
        root = ET.Element('mujoco')
        ET.SubElement(root, 'option', timestep='.001', integrator='implicitfast',
                      iterations='100', noslip_iterations='20')
        # Match the scene's inherited contact stiffness (not MuJoCo's softer default).
        ET.SubElement(ET.SubElement(root, 'default'), 'geom', solref='.005 1')
        assets = ET.SubElement(root, 'asset')
        add_shape_assets(assets, 'cup', item.collision_size_m, shape)
        world = ET.SubElement(root, 'worldbody')
        ET.SubElement(world, 'geom', name='floor', type='plane', size='1 1 .1')
        body = ET.SubElement(world, 'body', name='cup', pos='0 0 .001')
        ET.SubElement(body, 'freejoint')
        add_shape_geoms(body, 'cup', item.collision_size_m, (1., 1., 1., 1.), .008, shape)
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        data = mujoco.MjData(model)
        mujoco.mj_step(model, data, nstep=1000)
        position = data.xpos[model.body('cup').id].copy()
        mujoco.mj_step(model, data, nstep=1000)
        assert abs(data.xpos[model.body('cup').id, 2]) < .0001
        np.testing.assert_allclose(data.xpos[model.body('cup').id], position, atol=1e-5)
        assert data.xmat[model.body('cup').id, 8] > .9999
        counts.append(data.ncon)
        if clearance:
            support = {model.geom('floor').id, model.geom('cup_collision').id}
            assert all({c.geom1, c.geom2} == support for c in data.contact)
    assert 0 < counts[1] < counts[0]/3


def test_lego_body_and_all_eight_studs_have_enlarged_scale(model):
    body = model.geom('study_cafe_lego_collision').id
    np.testing.assert_allclose(2*model.geom_size[body], (.0636, .0316, .02112), atol=1e-9)
    visual = model.geom('study_cafe_lego_visual').id
    np.testing.assert_allclose(model.geom_size[visual], model.geom_size[body])
    for i in range(4):
        for j in range(2):
            stud = model.geom(f'study_cafe_lego_stud_{i}_{j}_collision').id
            assert model.geom_contype[stud] == 1
            np.testing.assert_allclose(model.geom_pos[stud], ((i-1.5)*.016, (j-.5)*.016, .0231))
            np.testing.assert_allclose(model.geom_size[stud][:2], (.0048, .00198))
            visual = model.geom(f'study_cafe_lego_stud_{i}_{j}_visual').id
            np.testing.assert_allclose(model.geom_pos[visual], model.geom_pos[stud])
            np.testing.assert_allclose(model.geom_size[visual], model.geom_size[stud])
    assert model.body_mass[model.body('study_cafe_lego').id] == pytest.approx(.0023)


def test_tissue_mesh_is_deterministic_and_has_configured_external_size():
    item = next(i for i in load_study_cafe_layout(ROOT/'config/study_cafe_layout.yaml').tabletop_objects if i.name == 'tissue')
    first = tissue_vertices(item.collision_size_m, item.shape)
    assert first == tissue_vertices(item.collision_size_m, item.shape)
    vertices, faces = np.asarray(first[0]), np.asarray(first[1])
    np.testing.assert_allclose(np.ptp(vertices, axis=0), (.06,.05,.045), atol=1e-10)
    assert vertices[:,2].min() == 0.
    assert np.isfinite(vertices).all() and faces.min() == 0 and faces.max() == len(vertices)-1
    # Nondegenerate triangles; outward oriented closed surface has positive volume.
    a, b, c = (vertices[faces[:,i]] for i in range(3))
    assert np.all(np.linalg.norm(np.cross(b-a,c-a), axis=1) > 1e-10)
    assert np.einsum('ij,ij->i',a,np.cross(b,c)).sum()/6 > 0


@pytest.mark.parametrize('key,value', [('stud_height_m', 0), ('stud_pitch_m', float('nan')),
                                      ('stud_height_m', None),
                                      ('studs_x', 4.5), ('stud_diameter_m', .02)])
def test_invalid_lego_geometry_is_rejected(key, value):
    raw = dict(stud_height_m=.0018, stud_pitch_m=.008, studs_x=4, studs_y=2, stud_diameter_m=.0048)
    raw[key] = value
    with pytest.raises(ValueError):
        parse_shape('lego_brick', raw, 'compound', (.0318,.0158,.0096))


def test_missing_shape_parameters_have_a_clear_validation_error():
    with pytest.raises(ValueError, match='paper_cup requires numeric geometry parameters'):
        parse_shape('paper_cup', {}, 'compound', (.085, .095))


def test_old_mug_and_phone_assets_are_not_loaded_in_active_scene():
    scene = materialize_scene(ROOT/'scenes/study_cafe.xml.in').read_text()
    assert 'study_cafe_cup.obj' not in scene and 'study_cafe_phone.obj' not in scene
    assert 'study_cafe_mouse.obj' in scene
