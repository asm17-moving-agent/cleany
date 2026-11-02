"""Verify wrist optical registration and visible rays without an OpenGL context."""
from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np
import pytest


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(PACKAGE / 'mjcf/cleany.xml'))


@pytest.fixture(scope='module')
def registration() -> np.ndarray:
    config = json.loads((PACKAGE / 'config/geometry/wrist_mount_v8.json').read_text())
    return np.asarray(config['legacy_jaw_from_camera_cv_approximate'])


@pytest.mark.parametrize('side', ('left', 'right'))
@pytest.mark.parametrize('pose', [
    (0., 0., 0., 0., 0.), (.4, 1., .8, .2, -.7), (-.4, 2., 1.6, -.7, .8),
], ids=['zero', 'bent', 'rolled'])
def test_canonical_wrist_optics_follow_v8_registration(
    model: mujoco.MjModel, registration: np.ndarray, side: str, pose: tuple[float, ...],
) -> None:
    """Keep rendered pixels and optical coordinates at the mounted lens."""
    expected = registration
    data = mujoco.MjData(model)
    jaw = model.body('Fixed_Jaw' if side == 'left' else 'Fixed_Jaw_2').id
    optical_name = f'{side}_wrist_rgb_optical_frame'
    site, body = model.site(optical_name).id, model.body(optical_name).id
    camera = model.camera(f'{side}_wrist_rgb').id
    joint_suffixes = ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch',
                      'wrist_pitch', 'wrist_roll')
    for suffix, value in zip(joint_suffixes, pose):
        data.qpos[model.joint(f'{side}_{suffix}_joint').qposadr[0]] = value
    mujoco.mj_forward(model, data)
    rotation = data.xmat[jaw].reshape(3, 3)
    expected_position = data.xpos[jaw] + rotation @ expected[:3, 3]
    expected_rotation = rotation @ expected[:3, :3]
    for position in (data.cam_xpos[camera], data.site_xpos[site], data.xpos[body]):
        np.testing.assert_allclose(position, expected_position, atol=1e-12)
    for actual in (data.site_xmat[site].reshape(3, 3),
                   data.xmat[body].reshape(3, 3),
                   data.cam_xmat[camera].reshape(3, 3) @ np.diag([1., -1., -1.])):
        np.testing.assert_allclose(actual, expected_rotation, atol=1e-12)


@pytest.mark.parametrize('side', ('left', 'right'))
def test_wrist_mount_does_not_occlude_camera_frustum(
    model: mujoco.MjModel, side: str,
) -> None:
    """Trace visible rays beyond the near plane, including the image centre."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = model.camera(f'{side}_wrist_rgb').id
    rotation = data.cam_xmat[camera].reshape(3, 3)
    near = model.vis.map.znear * model.stat.extent
    half_height = np.tan(np.deg2rad(model.cam_fovy[camera] / 2))
    geomgroup = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
    geom_id = np.array([-1], dtype=np.int32)
    for x in np.linspace(-.9, .9, 9):
        for y in np.linspace(-.9, .9, 7):
            direction = rotation @ [x * half_height * 4 / 3, y * half_height, -1.]
            origin = data.cam_xpos[camera] + near * direction
            direction /= np.linalg.norm(direction)
            distance = mujoco.mj_ray(model, data, origin, direction,
                                     geomgroup, 1, -1, geom_id)
            if distance >= 0:
                name = model.geom(int(geom_id[0])).name
                assert not name.startswith(f'{side}_wrist_mount_v8_'), (x, y, name)
