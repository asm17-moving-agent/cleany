"""The body-geometry consumer accepts the same articulated CAD as planning."""
import json
from pathlib import Path

import numpy as np
import pytest

from cleany_description.runtime_state import initial_positions
from cleany_navigation.body_geometry import load_body_boxes
from cleany_navigation.urdf_geometry import load_navigation_model


def test_body_geometry_accepts_cad_passives_mimics_and_all_mesh_formats(tmp_path):
    package = Path(__file__).parents[2]/'ros2_ws/src/cleany_description'
    model = package/'urdf/cleany.urdf'
    profile = tmp_path/'park.json'
    profile.write_text(json.dumps({'park_joints': initial_positions(model.read_text()), 'dynamic_pan': False}))
    boxes = load_body_boxes(package, profile)
    assert any(box.name == 'upper_body/merged' for box in boxes)
    assert all(np.isfinite(box.corners()).all() for box in boxes)
    assert len(boxes) > 1


def test_simulation_geometry_is_explicit_and_rejected_on_physical_path():
    packages = Path(__file__).parents[2] / 'ros2_ws/src'
    description = packages / 'cleany_description'
    profile = packages / 'cleany_gazebo_sim/config/cad_frame.yaml'
    with pytest.raises(ValueError, match='requires simulation'):
        load_body_boxes(description, profile)
    boxes = load_body_boxes(description, profile, allow_simulation_limits=True)
    assert boxes and all(np.isfinite(box.corners()).all() for box in boxes)
    assert load_navigation_model(description, {}, simulation=False).get('cleany_cad_calibration')
    assert load_navigation_model(description, {}, simulation=True).get('cleany_cad_calibration')
    for variant in ('cad_final', 'jetson_20261009'):
        with pytest.raises(ValueError, match='requires simulation'):
            load_navigation_model(description, {'model_profile': variant}, simulation=False)
