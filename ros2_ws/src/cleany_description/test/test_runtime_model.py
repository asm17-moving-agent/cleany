"""Installed runtime geometry and controller-free endpoint contracts."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from cleany_description.cad_coordinates import CadCoordinates
from cleany_description.mesh_bounds import mesh_bounds

PACKAGE = Path(__file__).parents[1]


def test_canonical_cad_hash_geometry_and_physical_stroke():
    path = PACKAGE/'urdf/cleany.urdf'
    manifest = json.loads((PACKAGE/'config/geometry/runtime_model.json').read_text())
    assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest['sha256']
    root = ET.parse(path).getroot()
    assert root.find('ros2_control') is None
    motors = [j for j in root.findall('joint') if j.get('type') == 'revolute' and j.find('mimic') is None]
    assert len(motors) == 14
    cad = CadCoordinates.from_urdf(path.read_text())
    assert len(cad.dependent_names) == 16
    for g in cad.config['grippers']:
        endpoint = g['endpoint_mapping']
        assert endpoint['motor_open_rad']-endpoint['motor_closed_rad'] == pytest.approx(2.7918450339518667)
        assert g['scale']*endpoint['motor_closed_rad']+g['offset_rad'] == pytest.approx(0.)
    assert len(root.findall('.//visual')) == 310
    for side in ('left', 'right'):
        for finger in ('L', 'R'):
            link = root.find(f"link[@name='cad_{side}_gripper_driving_{finger}']")
            assert any(c.get('name').startswith('cad_self__') for c in link.findall('collision'))
            assert any(c.get('name').startswith('cad_world__') for c in link.findall('collision'))


def test_every_runtime_mesh_exists_and_has_finite_bounds():
    meshes = {m.get('filename') for m in ET.parse(PACKAGE/'urdf/cleany.urdf').findall('.//mesh')}
    for name in meshes:
        assert name.startswith('package://cleany_description/')
        path = PACKAGE/name.split('package://cleany_description/',1)[1]
        lower, upper = mesh_bounds(path)
        assert (upper >= lower).all()
