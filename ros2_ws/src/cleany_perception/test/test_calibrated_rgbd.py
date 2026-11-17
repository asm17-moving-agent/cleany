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


@pytest.mark.parametrize('explicit', [True, False])
def test_launch_separates_sdk_topics_from_calibrated_outputs(monkeypatch, tmp_path, explicit):
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location('realsense_calibration_launch',
        Path(__file__).parents[1] / 'launch/realsense.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    device_dir = tmp_path / 'cleany'
    device_dir.mkdir()
    host_model = device_dir / 'head-rgb-intrinsics.json'
    host_model.write_text('{}')
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(Path(__file__).parents[1]))
    model = tmp_path / 'explicit.json' if explicit else host_model
    model.write_text('{}')
    nodes = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: nodes.append(kwargs))
    context = LaunchContext()
    context.launch_configurations.update(rgb_intrinsics_file=str(model) if explicit else '', serial_no='test',
        config_file='test.yaml', tf_topic='/tf', tf_static_topic='/tf_static')
    module.setup(context)
    assert [n['executable'] for n in nodes] == ['realsense2_camera_node', 'calibrated_rgbd']
    assert ('~/color/image_raw', '/camera/sdk/color/image_raw') in nodes[0]['remappings']
    assert {'align_depth.enable': False} in nodes[0]['parameters']
    assert nodes[1]['parameters'] == [{'intrinsics_file': str(model)}]
