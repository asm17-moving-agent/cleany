from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter

from cleany_perception.inspection_node import InspectionNode
from cleany_perception.model_runtime import (
    load_model_profile, resolve_device, resolve_model_assets,
)


def test_shared_profile_uses_yoloe_instance_masks():
    profile = load_model_profile(
        Path(__file__).resolve().parents[1] / 'config/yoloe_seg.yaml'
    )
    assert profile['detector_type'] == 'yoloe'
    assert profile['segmenter_type'] == 'yoloe_seg'
    assert profile['yoloe_model_path'] == 'yoloe/yoloe-26s-seg.pt'
    assert profile['preload_models'] is True
    assert profile['yoloe_image_size'] == 640
    assert profile['minimum_detection_confidence'] == 0.25


def test_flash_lite_profile_classifies_local_yoloe_masks():
    profile = load_model_profile(
        Path(__file__).resolve().parents[1] / 'config/yoloe_seg_gemini.yaml')
    assert profile['detector_type'] == 'yoloe_gemini'
    assert profile['gemini_model'] == 'gemini-3.1-flash-lite'
    assert profile['segmenter_type'] == 'yoloe_seg'
    assert profile['preload_models'] is True
    for label in ('cup', 'computer mouse', 'crumpled tissue', 'lego brick'):
        assert label not in profile['default_query'].lower()


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
            'yoloe_model_path': 'missing.pt', 'yoloe_text_encoder_directory': 'yoloe',
        }, str(tmp_path), 'yoloe')
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
