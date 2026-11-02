"""Cloud nodes keep active failures visible and tolerate shutdown conversion races."""
from types import SimpleNamespace

import pytest

from cleany_perception import depth_scene_node, scene_cloud_receipt_node
from rclpy.executors import ExternalShutdownException


@pytest.mark.parametrize('module,node_name', [
    (depth_scene_node, 'DepthSceneNode'),
    (scene_cloud_receipt_node, 'SceneCloudReceiptNode'),
])
@pytest.mark.parametrize('active,error', [
    (True, RuntimeError('cloud conversion failed')),
    (False, RuntimeError('cloud conversion interrupted')),
    (False, ExternalShutdownException()),
])
def test_shutdown_handling_preserves_active_runtime_errors(monkeypatch, module, node_name, active, error):
    events = []
    monkeypatch.setattr(module, node_name, lambda: SimpleNamespace(
        destroy_node=lambda: events.append('destroyed')))
    monkeypatch.setattr(module.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: active)
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: events.append('shutdown'))

    def spin(node):
        raise error

    monkeypatch.setattr(module.rclpy, 'spin', spin)
    if active:
        with pytest.raises(RuntimeError, match='cloud conversion failed'):
            module.main()
    else:
        module.main()
    assert events == (['destroyed', 'shutdown'] if active else ['destroyed'])
