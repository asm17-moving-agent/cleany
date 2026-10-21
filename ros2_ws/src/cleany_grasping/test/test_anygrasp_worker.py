from pathlib import Path
import numpy as np
import pytest

from cleany_grasping.anygrasp_worker import AnyGraspWorkerPredictor
from cleany_grasping.anygrasp_adapter import ModelUnavailableError
from cleany_grasping.core.models import PointCloud


def test_spawned_worker_reuses_then_releases_and_restarts_with_cpu_results(tmp_path, monkeypatch):
    # Exercise real spawn/Pipe/termination with a CPU-only SDK double.
    (tmp_path/'gsnet.py').write_text('''
import sys
from contextlib import nullcontext
from types import SimpleNamespace
import numpy as np
sys.modules['torch'] = SimpleNamespace(inference_mode=nullcontext)
def check_license(path): return True
class Detector:
    def get_grasp(self, points, options):
        return SimpleNamespace(rotation_matrices=np.array([np.eye(3)]),
            translations=np.array([[0.,0.,.5]]), widths=[.04], depths=[.02], scores=[.9])
def create_detector(options): return Detector()
''')
    monkeypatch.syspath_prepend(str(tmp_path))
    checkpoint = tmp_path/'checkpoint.tar'; checkpoint.touch()
    license = tmp_path/'license/test.lic'; license.parent.mkdir(); license.touch()
    worker = AnyGraspWorkerPredictor(checkpoint_path=str(checkpoint), license_path=str(license),
        idle_timeout_sec=120., request_timeout_sec=15., adaptive_sampling_enabled=True,
        adaptive_maximum_points=3, adaptive_minimum_region_points=1)
    cloud = PointCloud(np.zeros((4,3)), np.zeros((4,3)))
    try:
        result = worker.predict(cloud, cloud, np.array([-1.,1.]*3))
        pid = worker.worker_pid
        worker.predict_full_resolution(cloud, cloud, np.array([-1.,1.]*3))
        assert worker.worker_pid == pid and not worker.release_if_idle()
        assert worker.should_retry_full_resolution(cloud)
        worker._last_used -= 121.
        assert worker.release_if_idle() and worker.worker_pid is None
        assert result[0].translation.tolist() == [0.,0.,.5]
        assert worker.predict(cloud, cloud, np.array([-1.,1.]*3))[0].score == .9
        assert worker.worker_pid != pid
    finally:
        worker.close()
    assert worker.worker_pid is None


def test_failed_startup_is_reported_and_child_is_cleaned_up(tmp_path):
    worker = AnyGraspWorkerPredictor(checkpoint_path=str(tmp_path/'missing'),
        license_path=str(tmp_path/'license/missing'), request_timeout_sec=15.)
    with pytest.raises(ModelUnavailableError, match='configured'):
        worker.prepare()
    assert worker.worker_pid is None


@pytest.mark.parametrize('options', [{'idle_timeout_sec': -1.}, {'request_timeout_sec': 0.},
                                    {'idle_timeout_sec': float('nan')}])
def test_invalid_worker_timeouts_are_rejected_before_spawn(options):
    with pytest.raises(ValueError, match='timeout'):
        AnyGraspWorkerPredictor(checkpoint_path='', license_path='', **options)
