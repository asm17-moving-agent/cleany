"""The body-geometry consumer accepts the same articulated CAD as planning."""
import json
from pathlib import Path

import numpy as np

from cleany_description.runtime_state import initial_positions
from cleany_navigation.body_geometry import load_body_boxes


def test_body_geometry_accepts_cad_passives_mimics_and_all_mesh_formats(tmp_path):
    package = Path(__file__).parents[2]/'ros2_ws/src/cleany_description'
    model = package/'urdf/cleany.urdf'
    profile = tmp_path/'park.json'
    profile.write_text(json.dumps({'park_joints': initial_positions(model.read_text()), 'dynamic_pan': False}))
    boxes = load_body_boxes(package, profile)
    assert any(box.name == 'upper_body/merged' for box in boxes)
    assert all(np.isfinite(box.corners()).all() for box in boxes)
    assert len(boxes) > 1
