from dataclasses import replace
import hashlib
import json
import numpy as np
import pytest

from cleany_skill_executor.geometry_cache import fitted_geometry
from cleany_skill_executor.curobo_model import CuRoboConfig, RobotModel


def test_legacy_surface_point_cache_is_not_reused(tmp_path):
    from cleany_skill_executor.geometry_cache import _array_digest
    mesh = tmp_path / 'mesh.stl'
    mesh.write_bytes(b'mesh')
    digest = hashlib.sha256(mesh.read_bytes())
    digest.update(json.dumps([2, (1., 1., 1.), 4, .002, 'v1', 'convex_hull/subdivide']).encode())
    old = (np.ones((4, 3)), np.full(4, .002), np.ones((8, 3)))
    np.savez(tmp_path / (digest.hexdigest() + '.npz'), centers=old[0], radii=old[1],
             vertices=old[2], data_sha256=_array_digest(old))
    values, hit = fitted_geometry(mesh, (1., 1., 1.), 4, .002, 'v1', str(tmp_path),
                                 lambda: (old[0], np.full(4, .03), old[2]))
    assert not hit
    np.testing.assert_array_equal(values[1], np.full(4, .03))


def test_cache_invalidates_mesh_contents_scale_settings_and_version(tmp_path):
    mesh = tmp_path/'mesh.stl'
    mesh.write_bytes(b'first')
    calls = []
    def fit():
        calls.append(1)
        return np.ones((4,3)), np.ones(4), np.ones((8,3))
    def load(scale=(1.,1.,1.), count=4, version='v1'):
        return fitted_geometry(mesh, scale, count, .002, version, str(tmp_path/'cache'), fit)
    first, hit = load()
    assert not hit and not first[0].flags.writeable
    assert load()[1] and len(calls) == 1
    # Even a change of equal byte length must invalidate cached shapes.
    mesh.write_bytes(b'other')
    assert not load()[1]
    assert not load(scale=(2.,1.,1.))[1]
    assert not load(count=8)[1]
    assert not load(version='v2')[1]
    assert len(calls) == 5


def test_corrupt_or_unwritable_cache_falls_back_to_mesh_fitting(tmp_path):
    mesh = tmp_path/'mesh.stl'; mesh.write_bytes(b'mesh')
    fit = lambda: (np.ones((4,3)), np.ones(4), np.ones((8,3)))
    args = (mesh, (1.,1.,1.), 4, .002, 'v1', str(tmp_path/'cache'), fit)
    fitted_geometry(*args)
    next((tmp_path/'cache').glob('*.npz')).write_bytes(b'corrupt')
    assert not fitted_geometry(*args)[1]
    assert fitted_geometry(*args)[1]
    file = tmp_path/'unwritable'; file.write_bytes(b'file')
    assert not fitted_geometry(*args[:5], str(file), fit)[1]


def test_geometry_sharing_keeps_solver_tuning_separate():
    model = RobotModel.__new__(RobotModel)
    model.config = CuRoboConfig(num_seeds=16)
    model.spheres = {'link': [{'radius': .02}]}
    fallback = model.with_config(replace(model.config, num_seeds=32))
    assert fallback.spheres is model.spheres
    assert model.config.num_seeds == 16 and fallback.config.num_seeds == 32
    with pytest.raises(ValueError, match='identical sphere'):
        model.with_config(replace(model.config, spheres_per_link=32))
    with pytest.raises(ValueError, match='identical sphere'):
        model.with_config(replace(model.config, sphere_budget_per_link=0))
    with pytest.raises(ValueError, match='identical hull'):
        model.with_config(replace(model.config, collision_hull_max_expansion_m=0))
    with pytest.raises(ValueError, match='identical sphere models'):
        model.with_config(replace(model.config, sphere_model='generated'))
