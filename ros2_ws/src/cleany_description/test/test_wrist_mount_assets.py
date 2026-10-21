"""Validate portable wrist-mount assets and their canonical attachment frame."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def triangles(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    assert len(raw) == 84 + 50 * count, path
    dtype = np.dtype([
        ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2'),
    ])
    result = np.frombuffer(raw, offset=84, dtype=dtype)['vertices'].astype(float)
    assert np.isfinite(result).all(), path
    return result


def verify(package: Path) -> dict:
    """Check units, measured datums, source hashes and standalone frame placement."""
    config = json.loads((package / 'config/geometry/wrist_mount_v8.json').read_text())
    asset_dir = package / 'meshes/wrist_mount_v8'
    for folder, key in ((asset_dir, 'combined_mesh_sha256'),
                        (asset_dir / 'components', 'source_component_sha256')):
        for name, expected in config[key].items():
            assert Path(name).name == name
            assert hashlib.sha256((folder / name).read_bytes()).hexdigest() == expected, name

    combined = triangles(asset_dir / 'mount_legacy_jaw_m.stl')
    millimetres = triangles(asset_dir / 'mount_legacy_jaw_mm.stl')
    np.testing.assert_allclose(combined, millimetres / 1000, atol=5e-9, rtol=0)
    assert len(combined) == 120

    jaw_from_body = np.array(config['legacy_jaw_from_camera_geometry_body'])
    body_from_cv = np.array(config['camera_geometry_body_from_camera_cv'])
    jaw_from_cv = np.array(config['legacy_jaw_from_camera_cv_approximate'])
    for transform in (jaw_from_body, body_from_cv, jaw_from_cv):
        np.testing.assert_allclose(transform[3], [0, 0, 0, 1], atol=1e-12)
        rotation = transform[:3, :3]
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(rotation), 1, atol=1e-12)
    np.testing.assert_allclose(jaw_from_body @ body_from_cv, jaw_from_cv, atol=1e-12)

    def in_jaw(points: np.ndarray) -> np.ndarray:
        return points @ jaw_from_body[:3, :3].T + jaw_from_body[:3, 3]

    xml_path = package / 'mjcf/wrist_mount_v8.xml'
    xml = ET.parse(xml_path)
    mesh_relative = xml.find('compiler').get('meshdir')
    assert mesh_relative == '../meshes/wrist_mount_v8'
    mesh_dir = xml_path.parent / mesh_relative
    mesh_paths = {}
    for mesh in xml.findall('./asset/mesh'):
        relative = Path(mesh.get('file'))
        assert not relative.is_absolute(), relative
        assert relative.parent == Path('components'), relative
        assert relative.name in config['source_component_sha256'], relative
        path = (mesh_dir / relative).resolve()
        assert path.is_file(), path
        mesh_paths[mesh.get('name')] = path
    purple = [geom for geom in xml.findall('.//geom')
              if geom.get('type') == 'mesh'
              and np.allclose(np.fromstring(geom.get('rgba'), sep=' '), [.38, .08, .66, 1])]
    reconstructed = np.concatenate([in_jaw(triangles(mesh_paths[g.get('mesh')])) for g in purple])
    np.testing.assert_allclose(combined, reconstructed, atol=5e-9, rtol=0)
    assert len(purple) == 10

    measured = config['confirmed_user_measurements_mm']
    panel = triangles(asset_dir / 'components/case_back.stl')
    extent = np.ptp(panel, axis=(0, 1)) * 1000
    np.testing.assert_allclose(extent[:2], [measured['wrist_plate_width'],
                                         measured['wrist_plate_height']], atol=5e-6)
    np.testing.assert_allclose(panel[:, :, 1].max() * 1000,
                               measured['wrist_plate_top_to_lens_center'], atol=5e-6)
    foot = in_jaw(triangles(asset_dir / 'components/foot.stl'))
    np.testing.assert_allclose((jaw_from_cv[2, 3] - foot[:, :, 2].min()) * 1000,
                               measured['wrist_mount_contact_to_lens_vertical_height'], atol=5e-6)

    canonical = ET.parse(package / 'mjcf/cleany.xml')
    for name in ('Fixed_Jaw', 'Fixed_Jaw_2'):
        body = canonical.find(f'.//body[@name="{name}"]')
        geom = next(g for g in body.findall('geom') if g.get('mesh') == 'canonical_Fixed_Jaw')
        np.testing.assert_array_equal(np.fromstring(geom.get('pos', '0 0 0'), sep=' '), [0, 0, 0])
        np.testing.assert_array_equal(np.fromstring(geom.get('quat', '1 0 0 0'), sep=' '), [1, 0, 0, 0])
    expected_jaw = config['validation']['target_fixed_jaw_mesh_sha256']
    assert hashlib.sha256((package / 'meshes/Fixed_Jaw.stl').read_bytes()).hexdigest() == expected_jaw
    physical = ET.parse(package / 'urdf/physical_properties.xacro')
    for side, index in (('left', 70), ('right', 102)):
        origin = physical.find(f'.//visual[@name="mjcf_geom_{index}_0_visual"]/origin')
        saved = config['target_urdf_fixed_jaw_visual_origins'][side]
        xyz = np.fromstring(origin.get('xyz'), sep=' ')
        rpy = np.fromstring(origin.get('rpy'), sep=' ')
        np.testing.assert_allclose(xyz, saved['xyz_m'], atol=1e-15)
        np.testing.assert_allclose(rpy, saved['rpy_rad'], atol=1e-15)
        assert np.linalg.norm(xyz) < 1.3e-6
        assert np.linalg.norm(rpy) < 3e-6

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    assert model.njnt == 0
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    site = model.site('approximate_wrist_camera_cv').id
    np.testing.assert_allclose(data.site_xpos[site], jaw_from_cv[:3, 3], atol=1e-12)
    np.testing.assert_allclose(data.site_xmat[site].reshape(3, 3), jaw_from_cv[:3, :3], atol=1e-12)
    camera = model.camera('registered_wrist_rgb').id
    cv_rotation = data.cam_xmat[camera].reshape(3, 3) @ np.diag([1., -1., -1.])
    np.testing.assert_allclose(cv_rotation, jaw_from_cv[:3, :3], atol=1e-12)
    return {'status': 'passed', 'mujoco_version': mujoco.__version__,
            'asset_hashes': len(config['source_component_sha256']) + len(config['combined_mesh_sha256']),
            'purple_components': len(purple), 'purple_triangles': len(combined),
            'checks': ['binary STL integrity and hashes', 'm/mm equivalence',
                       'component-to-merged geometry transform', 'four user-measured datums',
                       'canonical fixed-jaw attachment and mesh identity',
                       'portable MJCF references and compilation', 'camera and optical-site transforms'],
            'scope': 'Geometry and file portability; no hardware, sensor calibration or robot dynamics validation.'}



def test_wrist_mount_assets() -> None:
    verify(Path(__file__).resolve().parents[1])
