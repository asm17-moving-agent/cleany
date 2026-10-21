import pytest
from cleany_skill_executor.core.sensor_scene import validate_sensor_scene


def test_populated_fresh_scene_passes():
    validate_sensor_scene(0.5, 2.0, 'OcTree', 500)

@pytest.mark.parametrize('age', [-0.1, 2.1, float('inf'), float('nan')])
def test_missing_or_stale_camera_fails_closed(age):
    with pytest.raises(RuntimeError, match='fresh'):
        validate_sensor_scene(age, 2., 'OcTree', 500)

@pytest.mark.parametrize('tree_id,size', [
    ('', 0), ('OcTree', 0), ('other', 100),
])
def test_missing_map_fails_closed(tree_id, size):
    with pytest.raises(RuntimeError, match='OctoMap'):
        validate_sensor_scene(0., 2., tree_id, size)


@pytest.mark.parametrize('capture_age,map_age,snapshot_age,map_size,accepted', [
    (2.002, .033, .033, 500, True),
    (3., .033, .033, 500, True),
    (3.001, .033, .033, 500, False),
    (-.001, .033, .033, 500, False),
    (.5, 3.001, .033, 500, False),
    (.5, .033, 3.001, 500, False),
    (.5, .033, .033, 0, False),
])
def test_operational_scene_budget_checks_capture_and_map_separately(
        monkeypatch, capture_age, map_age, snapshot_age, map_size, accepted):
    from pathlib import Path
    import threading
    from types import SimpleNamespace as NS
    import yaml
    from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError
    from cleany_skill_executor import sensor_scene

    values = yaml.safe_load((Path(__file__).resolve().parents[4]
        /'ros2_ws/src/cleany_skill_executor/test/data/hardware_motion_monitor_fixture.yaml').read_text())['pregrasp_pipeline']['ros__parameters']
    monitor = sensor_scene.SensorSceneMonitor.__new__(sensor_scene.SensorSceneMonitor)
    monitor.required, monitor.maximum_age = True, values['sensor_scene_maximum_age_sec']
    monitor.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000)))
    monitor._lock = threading.Lock()
    monitor._capture_ns = 100_000_000_000-round(capture_age*1e9)
    monitor._map_received, monitor._snapshot_received = 100-map_age, 100-snapshot_age
    monitor._map_id, monitor._map_bytes = 'OcTree', map_size
    monkeypatch.setattr(sensor_scene.time, 'monotonic', lambda: 100.)
    if accepted:
        monitor.check()
    else:
        with pytest.raises(InfrastructureError, match='Sensor planning scene unavailable'):
            monitor.check()
