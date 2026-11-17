"""Verify explicit runtime covers for registered wrist solids."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from cleany_skill_executor import collision_sphere_model as spheres
from cleany_skill_executor.curobo_model import CuRoboConfig


@pytest.fixture
def registered_model(tmp_path, monkeypatch):
    mesh = tmp_path/'mesh.stl'
    mesh.write_bytes(b'known mesh')
    xml = ('<robot><link name="left_gripper_frame"><collision name="registered_camera">'
           '<geometry><mesh filename="mesh.stl"/></geometry></collision></link>'
           '<link name="right_gripper_frame"><collision><geometry>'
           '<mesh filename="mesh.stl"/></geometry></collision></link></robot>')
    resolve = lambda _: mesh
    signatures = spheres.collision_signatures(xml, {'left_gripper_frame', 'right_gripper_frame'}, resolve)
    asset = dict(schema=2, model='joint_simplified', padding_m=.0005, joint_covers=[],
                 coverage_certificate_sha256='unchanged-certificate', links={
        name: dict(collision_sha256=signature, spheres=[dict(center=[0., 0., 0.], radius=.1)])
        for name, signature in signatures.items()})
    asset['links']['left_gripper_frame']['collision_sha256'] = 'old-v8-geometry'
    path = tmp_path/'asset.json'; path.write_text(json.dumps(asset))
    monkeypatch.setattr(spheres, 'MODEL_PATH', path)
    vertices = np.array([[x,y,z] for x in [-.02,.02] for y in [-.01,.01] for z in [-.03,.03]])
    bodies = [NS(owner='left_gripper_frame', vertices=vertices+np.array([i*.08,0.,0.]))
              for i in range(3)]
    model = NS(xml=xml, config=CuRoboConfig(sphere_model='joint_simplified_registered',
               sphere_budget_per_link=2), bodies=bodies,
               spheres={name: [dict(center=[0.,0.,0.], radius=.2)] for name in signatures},
               ignore={name: [] for name in signatures},
               sphere_reduction=dict(links={name: dict(before=64,after=16) for name in signatures}))
    return model, resolve


def test_registered_wrist_encloses_every_current_solid_and_keeps_other_certificates(registered_model):
    model, resolve = registered_model
    bodies = model.bodies
    spheres.apply_sphere_model(model, resolve=resolve)
    assert model.bodies is bodies
    assert model.spheres['right_gripper_frame'] == [dict(center=[0.,0.,0.], radius=.1)]
    assert len(model.spheres['left_gripper_frame']) == 2
    for body in bodies:
        assert any(np.max(np.linalg.norm(body.vertices-ball['center'],axis=1)) <
                   ball['radius'] for ball in model.spheres['left_gripper_frame'])
    assert model.sphere_reduction['runtime_solid_covers']['left_gripper_frame']['method'] == 'enclosing_convex_body_balls'


@pytest.mark.parametrize('change', ['other_link', 'no_registration', 'invalid_solid', 'no_solids'])
def test_registered_mode_does_not_silently_accept_other_geometry(registered_model, change):
    model, resolve = registered_model
    if change == 'other_link':
        model.xml = model.xml.replace('<collision><geometry>',
                                      '<collision><origin xyz="1 0 0"/><geometry>')
    elif change == 'no_registration':
        model.xml = model.xml.replace('registered_camera', 'camera')
    elif change == 'invalid_solid':
        model.bodies[0].vertices[0,0] = np.nan
    else:
        model.bodies = []
    before = deepcopy(model.spheres)
    with pytest.raises(ValueError):
        spheres.apply_sphere_model(model, resolve=resolve)
    assert model.spheres == before
