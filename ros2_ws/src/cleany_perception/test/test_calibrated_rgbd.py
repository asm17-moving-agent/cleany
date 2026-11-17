import numpy as np
import pytest
import importlib.util
from pathlib import Path

from cleany_perception.calibrated_rgbd import align_depth


@pytest.mark.parametrize('native', [True, False])
def test_native_depth_is_projected_using_measured_rgb_focal_length(native):
    depth = np.ones((1, 3), dtype=np.float32)
    depth_k = np.eye(3)
    rgb_k = np.diag([2., 2., 1.])
    result = align_depth(depth, depth_k, rgb_k, np.eye(4), (5, 1), native=native)
    np.testing.assert_array_equal(result, [[1, 0, 1, 0, 1]])


@pytest.mark.parametrize('native', [True, False])
def test_registration_uses_extrinsics_and_keeps_nearest_surface(native):
    depth = np.array([[1., 2., np.nan, 0.]])
    transform = np.eye(4)
    transform[0, 3] = 2.
    # Both visible points project to pixel 2: x'=x+tx/Z. Nearer Z wins.
    result = align_depth(depth, np.eye(3), np.eye(3), transform, (4, 1), native=native)
    np.testing.assert_array_equal(result, [[0, 0, 1, 0]])


@pytest.mark.parametrize('native', [True, False])
def test_points_behind_rgb_camera_are_not_published(native):
    transform = np.eye(4)
    transform[2, 3] = -2.
    result = align_depth(np.ones((2, 2)), np.eye(3), np.eye(3), transform, (2, 2), native=native)
    assert not result.any()


@pytest.mark.parametrize(('explicit', 'serial', 'calibrated'), [
    (True, 'other-device', True), (False, '261822075013', True),
    (False, 'other-device', False), (False, '', False),
])
def test_launch_selects_calibration_for_the_requested_camera(monkeypatch, tmp_path, explicit, serial, calibrated):
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location('realsense_calibration_launch',
        Path(__file__).parents[1] / 'launch/realsense.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    package = Path(__file__).parents[1]
    bundled_model = package / 'config/jetson_head_intrinsics.json'
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(package))
    model = tmp_path / 'explicit.json' if explicit else bundled_model
    if explicit:
        model.write_text('{}')
    nodes = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: nodes.append(kwargs))
    context = LaunchContext()
    context.launch_configurations.update(rgb_intrinsics_file=str(model) if explicit else '', serial_no=serial,
        config_file='test.yaml', tf_topic='/tf', tf_static_topic='/tf_static')
    module.setup(context)
    if not calibrated:
        assert [node['executable'] for node in nodes] == ['realsense2_camera_node']
        assert ('~/aligned_depth_to_color/image_raw', '/camera/depth/image_raw') in nodes[0]['remappings']
        return
    assert [n['executable'] for n in nodes] == ['realsense2_camera_node', 'calibrated_rgbd']
    assert ('~/color/image_raw', '/camera/sdk/color/image_raw') in nodes[0]['remappings']
    assert {'align_depth.enable': False} in nodes[0]['parameters']
    assert nodes[1]['parameters'] == [{'intrinsics_file': str(model)}]
