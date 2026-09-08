from pathlib import Path

import mujoco
import numpy as np
import pytest

from cleany_mujoco_sim.scene_loader import materialize_scene
from cleany_mujoco_sim.study_cafe_scene import load_study_cafe_layout
from cleany_mujoco_sim.tabletop_shapes import parse_shape, tissue_vertices


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
    assert distance == pytest.approx(.12-.0007, abs=1e-7)
    assert model.body_mass[cup] == pytest.approx(.008)


def test_lego_body_and_all_eight_physical_studs_have_real_scale(model):
    body = model.geom('study_cafe_lego_collision').id
    np.testing.assert_allclose(2*model.geom_size[body], (.0318, .0158, .0096), atol=1e-9)
    for i in range(4):
        for j in range(2):
            stud = model.geom(f'study_cafe_lego_stud_{i}_{j}_collision').id
            assert model.geom_contype[stud] == 1
            np.testing.assert_allclose(model.geom_pos[stud], ((i-1.5)*.008, (j-.5)*.008, .0105))
            np.testing.assert_allclose(model.geom_size[stud][:2], (.0024, .0009))
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
    assert 'study_cafe_wallet.obj' in scene
