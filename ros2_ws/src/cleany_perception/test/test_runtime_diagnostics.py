from pathlib import Path

import numpy as np
import pytest
from PIL import Image as PilImage

from cleany_perception.runtime_diagnostics import (
    RuntimeMonitor,
    SnapshotArtifactStore,
)


class _Clock:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self) -> float:
        return self.value


def test_runtime_monitor_reports_stages_total_and_memory_delta():
    clock = _Clock()
    memory_reports = iter(
        (
            {
                'rss_bytes': 100,
                'process_peak_rss_bytes': 120,
                'system_total_bytes': 1000,
                'system_available_bytes': 600,
            },
            {'rss_bytes': 110, 'process_peak_rss_bytes': 130},
            {
                'rss_bytes': 160,
                'process_peak_rss_bytes': 180,
                'cuda_allocated_bytes': 20,
                'cuda_reserved_bytes': 30,
                'cuda_peak_allocated_bytes': 40,
                'cuda_peak_reserved_bytes': 50,
                'cuda_total_bytes': 200,
                'system_total_bytes': 1000,
                'system_available_bytes': 400,
            },
        )
    )
    monitor = RuntimeMonitor(
        True,
        'selection',
        True,
        clock=clock,
        memory_reader=lambda _cuda, _reset: next(memory_reports),
    )
    monitor.begin_stage('sam2')
    clock.value = 1.25
    monitor.begin_stage('reconstruction_3d')
    clock.value = 2.0

    report = monitor.finish(
        success=False,
        error_code=6,
        message='plane failed',
        snapshot_id='rgbd-123-000001',
        selected_object_id=2,
    )

    assert report is not None
    assert report['timing_seconds']['sam2'] == pytest.approx(1.25)
    assert report['timing_seconds']['reconstruction_3d'] == pytest.approx(0.75)
    assert report['timing_seconds']['total'] == pytest.approx(2.0)
    assert report['memory']['rss_delta_bytes'] == 60
    assert report['memory']['rss_end_percent_of_system'] == 16.0
    assert report['memory']['system_used_end_percent'] == 60.0
    assert report['memory']['cuda_peak_allocated_bytes'] == 40
    assert report['memory']['cuda_peak_allocated_percent_of_total'] == 20.0


def test_disabled_runtime_monitor_returns_no_report():
    monitor = RuntimeMonitor(False, 'detection', False)

    assert monitor.finish(
        success=True,
        error_code=0,
        message='ok',
        snapshot_id='',
        selected_object_id=0,
    ) is None


def test_runtime_monitor_ignores_memory_reader_failure():
    def failing_memory_reader(_include_cuda, _reset_cuda_peak):
        raise RuntimeError('CUDA diagnostics unavailable')

    monitor = RuntimeMonitor(
        True,
        'selection',
        True,
        memory_reader=failing_memory_reader,
    )

    report = monitor.finish(
        success=True,
        error_code=0,
        message='ok',
        snapshot_id='rgbd-123-000001',
        selected_object_id=1,
    )

    assert report is not None
    assert report['memory']['rss_start_bytes'] is None
    assert report['memory']['cuda_peak_allocated_bytes'] is None


def test_snapshot_artifact_store_writes_image_and_json(tmp_path):
    store = SnapshotArtifactStore(tmp_path)
    rgb = np.zeros((12, 16, 3), dtype=np.uint8)
    rgb[:, :] = (10, 20, 30)

    image_path = store.save_rgb(
        'rgbd-123-000001',
        'detections.png',
        rgb,
    )
    json_path = store.save_json(
        'rgbd-123-000001',
        'detection-metrics.json',
        {'success': True},
    )

    assert image_path == tmp_path / 'rgbd-123-000001' / 'detections.png'
    assert np.asarray(PilImage.open(image_path)).shape == (12, 16, 3)
    assert '"success": true' in json_path.read_text(encoding='utf-8')


@pytest.mark.parametrize('snapshot_id', ('../outside', '/tmp/outside', ''))
def test_snapshot_artifact_store_rejects_unsafe_snapshot_id(
    tmp_path: Path,
    snapshot_id: str,
):
    store = SnapshotArtifactStore(tmp_path)

    with pytest.raises(ValueError):
        store.save_json(snapshot_id, 'metrics.json', {})
