"""Check the rendered wrist pose against the shared lens registration."""
import json
from copy import deepcopy
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest
import yaml

from cleany_gazebo_sim.world.cad_frame import adapt_cad_frame
from cleany_gazebo_sim.world.generator import materialize_mecanum_wheel_world
from cleany_navigation.urdf_geometry import rotation


PACKAGE = Path(__file__).resolve().parents[1]
DESCRIPTION = PACKAGE.parent / 'cleany_description'


def _transform(pose: ET.Element) -> np.ndarray:
    values = np.fromstring(pose.text, sep=' ')
    result = np.eye(4)
    result[:3, 3] = values[:3]
    result[:3, :3] = rotation(values[3:])
    return result


@pytest.fixture(scope='module')
def legacy_model(tmp_path_factory: pytest.TempPathFactory) -> ET.Element:
    directory = tmp_path_factory.mktemp('wrist-world')
    world = materialize_mecanum_wheel_world(
        PACKAGE / 'worlds/cleany_mecanum_fortress.sdf', target_path=directory/'world.sdf')
    return ET.parse(world).find("world/model[@name='cleany_mecanum']")


@pytest.mark.parametrize('model_name', ('legacy', 'cad_frame_with_arms'))
def test_wrist_sensors_and_optical_frames_follow_v8_lens(
    tmp_path: Path, legacy_model: ET.Element, model_name: str,
) -> None:
    model = deepcopy(legacy_model)
    jaw_suffix = 'fixed_jaw'
    if model_name == 'cad_frame_with_arms':
        profile = yaml.safe_load((PACKAGE / 'config/cad_frame.yaml').read_text())
        profile['simulation_excluded_link_roots'] = []
        config = tmp_path / 'with_arms.yaml'
        config.write_text(yaml.safe_dump(profile))
        model, _ = adapt_cad_frame(model, DESCRIPTION, config)
        jaw_suffix = 'gripper_frame'
    registration = json.loads((DESCRIPTION / 'config/geometry/wrist_mount_v8.json').read_text())
    expected = np.asarray(registration['legacy_jaw_from_camera_cv_approximate'])
    for side in ('left', 'right'):
        parent = f'{side}_{jaw_suffix}'
        sensor = model.find(f".//sensor[@name='{side}_wrist_rgb']")
        pose = sensor.find('pose')
        assert pose.get('relative_to') == parent
        actual = _transform(pose)
        np.testing.assert_allclose(actual[:3, 3], expected[:3, 3], atol=1e-12)
        # Compare viewing, image-right and image-down directions independently.
        np.testing.assert_allclose(actual[:3, 0], expected[:3, 2], atol=1e-12)
        np.testing.assert_allclose(-actual[:3, 1], expected[:3, 0], atol=1e-12)
        np.testing.assert_allclose(-actual[:3, 2], expected[:3, 1], atol=1e-12)
        optical_name = sensor.findtext('gz_frame_id')
        frames = model.findall(f"frame[@name='{optical_name}']")
        assert len(frames) == 1
        assert frames[0].get('attached_to') == parent
        np.testing.assert_allclose(_transform(frames[0].find('pose')), expected, atol=1e-12)
        visuals = [v for v in model.findall('.//visual')
                   if f'{side}_wrist_mount_v8_' in v.get('name', '')]
        assert len(visuals) == 6
        assert not any('XLeRobot_camera' in uri.text
                       for uri in model.findall('.//visual/geometry/mesh/uri'))
        for visual in visuals:
            mesh = visual.find('geometry/mesh/uri')
            if mesh is not None:
                assert Path(mesh.text).is_file()
