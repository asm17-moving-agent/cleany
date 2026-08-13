from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _parameters(filename: str, node: str = '/**') -> dict:
    document = yaml.safe_load((PACKAGE_ROOT / 'config' / filename).read_text())
    return document[node]['ros__parameters']


def test_d435_config_enforces_aligned_synchronized_profiles() -> None:
    parameters = _parameters('jetson_d435.yaml')

    assert parameters['depth_module.depth_profile'] == '640x480x15'
    assert parameters['rgb_camera.color_profile'] == '640x480x15'
    assert parameters['enable_sync'] is True
    assert parameters['align_depth.enable'] is True
    assert parameters['enable_infra1'] is False
    assert parameters['enable_infra2'] is False
    assert parameters['pointcloud__neon_.enable'] is True
    assert parameters['pointcloud__neon_.stream_filter'] == 2


def test_perception_uses_aligned_color_optical_inputs() -> None:
    parameters = _parameters('perception_d435.yaml', 'perception_inspector')

    assert parameters['color_image_topic'].endswith('/color/image_raw')
    assert parameters['color_info_topic'].endswith('/color/camera_info')
    assert '/aligned_depth_to_color/' in parameters['depth_image_topic']
    assert '/aligned_depth_to_color/' in parameters['depth_info_topic']
    assert parameters['target_frame'] == 'camera_color_optical_frame'
    assert parameters['depth_16u_scale_m'] == 0.001
