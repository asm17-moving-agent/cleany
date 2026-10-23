from dataclasses import replace
from types import SimpleNamespace
import json

import numpy as np
import pytest

from cleany_skill_executor.curobo_collision import ConvexBody
from cleany_skill_executor.curobo_model import CuRoboConfig
from cleany_skill_executor.robot_geometry_cache import load_model, model_cache_path, save_model


def model():
    return SimpleNamespace(limits={'joint': (-1., 1.)}, velocity_limits={'joint': 1.},
        spheres={'link': [{'center': [0., 0., 0.], 'radius': .1}]},
        collision_links=['link'], ignore={'link': []}, sphere_reduction={'after': 1},
        bodies=[ConvexBody('link', np.ones((4, 3)), np.ones((4, 4)))])


def test_complete_cache_round_trip_and_corruption_fallback(tmp_path):
    path = tmp_path / 'model.npz'
    original, restored = model(), SimpleNamespace()
    save_model(path, original)
    assert load_model(path, restored)
    for field in ('limits', 'velocity_limits', 'spheres', 'collision_links', 'ignore', 'sphere_reduction'):
        assert getattr(original, field) == getattr(restored, field)
    np.testing.assert_array_equal(restored.bodies[0].planes, original.bodies[0].planes)
    assert not restored.bodies[0].vertices.flags.writeable
    with np.load(path, allow_pickle=False) as stored:
        values = {key: stored[key] for key in stored.files}
    metadata = json.loads(str(values['metadata']))
    metadata['spheres']['link'][0]['radius'] = .001
    values['metadata'] = json.dumps(metadata)
    np.savez(path, **values)
    untouched = SimpleNamespace()
    assert not load_model(path, untouched) and not vars(untouched)
    path.write_bytes(b'broken')
    assert not load_model(path, untouched)
    blocker = tmp_path / 'file'
    blocker.write_text('not a directory')
    save_model(blocker / 'model.npz', original)  # Cache failure is optional.
    assert not load_model(blocker / 'model.npz', untouched)


def test_complete_cache_key_tracks_geometry_but_not_solver_tuning(tmp_path, monkeypatch):
    monkeypatch.setattr('importlib.metadata.version', lambda _: 'test-v1')
    mesh = tmp_path / 'mesh.stl'
    mesh.write_bytes(b'original')
    urdf = f'<robot><link name="arm"><collision><geometry><mesh filename="{mesh}"/></geometry></collision></link></robot>'
    config = CuRoboConfig(geometry_cache_directory=str(tmp_path))
    first = model_cache_path(urdf, '<robot/>', config)
    assert first == model_cache_path(urdf, '<robot/>', replace(config, num_seeds=16))
    assert first != model_cache_path(urdf.replace('arm', 'other'), '<robot/>', config)
    assert first != model_cache_path(urdf, '<robot name="changed"/>', config)
    assert first != model_cache_path(urdf, '<robot/>', replace(config, sphere_budget_per_link=0))
    assert first != model_cache_path(urdf, '<robot/>', replace(config, collision_hull_max_expansion_m=0))
    mesh.write_bytes(b'changed!')
    assert first != model_cache_path(urdf, '<robot/>', config)
    assert model_cache_path(urdf, '', replace(config, geometry_cache_directory='')) is None
