from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter

from cleany_perception.inspection_node import InspectionNode
from cleany_perception.model_runtime import (
    load_model_profile, resolve_device, resolve_model_assets,
)


def test_shared_profile_selects_small_yoloe_and_tiny_sam2():
    profile = load_model_profile(
        Path(__file__).resolve().parents[1] / 'config/yoloe_s_sam2_tiny.yaml'
    )
    assert profile['detector_type'] == 'yoloe'
    assert profile['segmenter_type'] == 'sam2'
    assert profile['yoloe_model_path'] == 'yoloe/yoloe-26s-seg.pt'
    assert profile['sam2_checkpoint'] == 'sam2/sam2.1_t.pt'
    assert profile['preload_models'] is True
    assert profile['yoloe_image_size'] == 640
    assert profile['minimum_detection_confidence'] == 0.25


def test_flash_lite_profile_uses_cloud_detection_and_local_tiny_masks():
    profile = load_model_profile(
        Path(__file__).resolve().parents[1] / 'config/gemini_flash_lite_sam2_tiny.yaml')
    assert profile['detector_type'] == 'gemini'
    assert profile['gemini_model'] == 'gemini-3.1-flash-lite'
    assert profile['segmenter_type'] == 'sam2'
    assert profile['sam2_checkpoint'] == 'sam2/sam2.1_t.pt'
    assert profile['preload_models'] is True
    for label in ('cup', 'wallet', 'crumpled tissue', 'lego brick'):
        assert label in profile['default_query']


@pytest.mark.parametrize('requested,available,expected', [
    ('auto', False, 'cpu'), ('auto', True, 'cuda:0'),
    ('cpu', True, 'cpu'), ('cuda:1', True, 'cuda:1'),
])
def test_device_resolution(requested, available, expected):
    assert resolve_device(requested, available) == expected


@pytest.mark.parametrize('requested', ['cuda', 'cuda:0', 'other', ''])
def test_no_silent_device_fallback(requested):
    with pytest.raises(ValueError):
        resolve_device(requested, False)


@pytest.mark.parametrize('detector_type', ['gemini', 'yoloe'])
def test_learned_detectors_initialize_reference_and_wrist_services(monkeypatch, detector_type):
    import cleany_perception.inspection_node as module
    events = []

    class Tracker:
        def __init__(self, *args):
            events.append('tracker_created')

    monkeypatch.setattr(module, 'Sam2ReferenceTracker', Tracker)
    rclpy.init(args=[])
    node = None
    try:
        node = InspectionNode(detector=object(), segmenter=object(), transformer=object(),
            parameter_overrides=[
                Parameter('detector_type', value=detector_type),
                Parameter('segmenter_type', value='sam2'),
                Parameter('enable_reference_observation', value=True),
                Parameter('enable_wrist_observation', value=True),
                Parameter('wrist_continuous_tracking', value=False),
            ])
        assert events == ['tracker_created']
        assert node._reference_service is not None
        assert node._wrist_service is not None
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_relative_assets_resolve_against_model_directory(tmp_path):
    (tmp_path / 'yoloe').mkdir()
    (tmp_path / 'yoloe/small.pt').write_bytes(b'checkpoint')
    (tmp_path / 'yoloe/mobileclip2_b.ts').write_bytes(b'encoder')
    resolved = resolve_model_assets({
        'yoloe_model_path': 'yoloe/small.pt',
        'yoloe_text_encoder_directory': 'yoloe',
    }, str(tmp_path), 'yoloe')
    assert resolved['yoloe_model_path'] == str(tmp_path / 'yoloe/small.pt')


def test_missing_assets_fail_without_download_or_substitution(tmp_path):
    with pytest.raises(ValueError, match='not found'):
        resolve_model_assets({
            'sam2_checkpoint': 'missing.pt', 'sam2_model_config': 'config',
        }, str(tmp_path), 'sam2')
    assert list(tmp_path.iterdir()) == []


def test_model_preload_precedes_action_advertisement(monkeypatch):
    import cleany_perception.inspection_node as module
    events = []

    class Adapter:
        def prepare(self):
            events.append('prepare')

    class Server:
        def __init__(self, *args, **kwargs):
            events.append('action_server')

        def destroy(self):
            pass

    monkeypatch.setattr(module, 'ActionServer', Server)
    rclpy.init(args=[])
    node = None
    try:
        node = InspectionNode(
            detector=Adapter(), segmenter=Adapter(), transformer=object(),
            parameter_overrides=[Parameter('preload_models', value=True)],
        )
        assert events == ['prepare', 'prepare', 'action_server']
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_failed_preload_never_advertises_action(monkeypatch):
    import cleany_perception.inspection_node as module

    class BrokenAdapter:
        def prepare(self):
            raise RuntimeError('model load failure')

    monkeypatch.setattr(module, 'ActionServer',
                        lambda *a, **kw: pytest.fail('Action became ready'))
    rclpy.init(args=[])
    try:
        with pytest.raises(RuntimeError, match='model load failure'):
            InspectionNode(
                detector=BrokenAdapter(), segmenter=BrokenAdapter(),
                transformer=object(),
                parameter_overrides=[Parameter('preload_models', value=True)],
            )
    finally:
        rclpy.shutdown()
