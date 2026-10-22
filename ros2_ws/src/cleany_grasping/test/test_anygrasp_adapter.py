from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from cleany_grasping.anygrasp_adapter import (
    AnyGraspPredictor,
    ModelUnavailableError,
)
from cleany_grasping.core.models import PointCloud
import cleany_grasping.anygrasp_adapter as adapter_module


class FakeDetector:
    def __init__(self) -> None:
        self.points = None
        self.options = None

    def get_grasp(self, points, options):
        self.points = points
        self.options = options
        return SimpleNamespace(
            rotation_matrices=np.asarray([np.eye(3)]),
            translations=np.asarray([[0.0, 0.0, 0.5]]),
            widths=np.asarray([0.04]),
            depths=np.asarray([0.02]),
            scores=np.asarray([0.9]),
        )


def _files(tmp_path):
    checkpoint = tmp_path / 'checkpoint.tar'
    checkpoint.write_bytes(b'checkpoint')
    license_dir = tmp_path / 'license'
    license_dir.mkdir()
    license_file = license_dir / 'test.lic'
    license_file.write_bytes(b'license')
    return checkpoint, license_file


def _cloud() -> PointCloud:
    points = np.asarray(
        [(-0.1, 0.0, 0.5), (0.0, 0.0, 0.5), (0.2, 0.0, 0.5)]
    )
    return PointCloud(points, np.zeros_like(points))


def test_uses_new_sdk_detector_and_region_steering(
    tmp_path, monkeypatch
) -> None:
    checkpoint, license_file = _files(tmp_path)
    detector = FakeDetector()
    observed = {}

    def check_license(path):
        observed['license'] = path
        return True

    def create_detector(configuration):
        observed['configuration'] = configuration
        observed['cwd'] = str(Path.cwd())
        return detector

    monkeypatch.setitem(
        sys.modules,
        'gsnet',
        SimpleNamespace(
            check_license=check_license,
            create_detector=create_detector,
        ),
    )
    predictor = AnyGraspPredictor(
        str(checkpoint),
        str(license_file),
        maximum_gripper_width_m=0.08,
        gripper_height_m=0.025,
    )

    result = predictor.predict(
        _cloud(),
        _cloud(),
        np.asarray((-0.05, 0.05, -0.1, 0.1, 0.4, 0.6)),
    )

    assert observed['license'] == str(license_file.parent)
    assert observed['cwd'] == str(tmp_path)
    assert observed['configuration'].max_gripper_width == 0.08
    assert observed['configuration'].gripper_height == 0.025
    assert detector.points.dtype == np.float32
    assert detector.options['region_steering'].tolist() == [False, True, False]
    assert len(result) == 1
    assert result[0].score == pytest.approx(0.9)


def test_rejects_failed_license_validation(tmp_path, monkeypatch) -> None:
    checkpoint, license_file = _files(tmp_path)
    monkeypatch.setitem(
        sys.modules,
        'gsnet',
        SimpleNamespace(
            check_license=lambda _: False,
            create_detector=lambda _: FakeDetector(),
        ),
    )
    predictor = AnyGraspPredictor(str(checkpoint), str(license_file))

    with pytest.raises(ModelUnavailableError, match='validation failed'):
        predictor.predict(_cloud(), _cloud(), np.asarray((-1.0, 1.0) * 3))


def test_requires_sdk_license_directory_name(tmp_path) -> None:
    checkpoint = tmp_path / 'checkpoint.tar'
    checkpoint.write_bytes(b'checkpoint')
    license_dir = tmp_path / 'credentials'
    license_dir.mkdir()
    license_file = license_dir / 'test.lic'
    license_file.write_bytes(b'license')
    predictor = AnyGraspPredictor(str(checkpoint), str(license_file))

    with pytest.raises(ModelUnavailableError, match='directory named license'):
        predictor.predict(_cloud(), _cloud(), np.asarray((-1.0, 1.0) * 3))


@pytest.mark.parametrize('interval, expected_collections', [(1, 8), (4, 2), (16, 0)])
def test_periodic_gc_keeps_returning_cuda_cache(
    monkeypatch, interval, expected_collections
) -> None:
    events = []
    monkeypatch.setattr(adapter_module.gc, 'collect', lambda: events.append('gc'))
    automatic_gc_enabled = adapter_module.gc.isenabled()
    cuda = SimpleNamespace(
        is_available=lambda: True,
        synchronize=lambda: None,
        memory_reserved=lambda: 1024 * 2**20,
        memory_allocated=lambda: 128 * 2**20,
        empty_cache=lambda: events.append('cache'),
    )
    predictor = AnyGraspPredictor('', '', cuda_gc_interval_calls=interval)
    for _ in range(8):
        predictor._release_unused_cuda_cache(SimpleNamespace(cuda=cuda))
    assert events.count('gc') == expected_collections
    assert events.count('cache') == 8
    assert predictor._cuda_gc_calls_since_collection == 8 % interval
    assert adapter_module.gc.isenabled() == automatic_gc_enabled


def test_gc_waits_for_cache_pressure_then_collects_before_returning_cache(monkeypatch):
    events = []
    reserved = 128 * 2**20
    monkeypatch.setattr(adapter_module.gc, 'collect', lambda: events.append('gc'))
    cuda = SimpleNamespace(
        is_available=lambda: True,
        synchronize=lambda: None,
        memory_reserved=lambda: reserved,
        memory_allocated=lambda: 128 * 2**20,
        empty_cache=lambda: events.append('cache'),
    )
    predictor = AnyGraspPredictor('', '', cuda_gc_interval_calls=2)
    for _ in range(3):
        predictor._release_unused_cuda_cache(SimpleNamespace(cuda=cuda))
    assert events == []
    reserved = 1024 * 2**20
    predictor._release_unused_cuda_cache(SimpleNamespace(cuda=cuda))
    assert events == ['gc', 'cache']
    assert predictor._cuda_gc_calls_since_collection == 0


@pytest.mark.parametrize('interval', [0, -1, 1.5, True])
def test_invalid_gc_interval_is_rejected_before_loading_model(interval):
    with pytest.raises(ValueError, match='positive integer'):
        AnyGraspPredictor('', '', cuda_gc_interval_calls=interval)


def test_approach_steering_reaches_sdk_without_removing_collision_checks(monkeypatch):
    predictor=AnyGraspPredictor('', '', approach_direction=(0.,0.,2.), approach_threshold_rad=np.pi/3)
    predictor._model=FakeDetector()
    monkeypatch.setattr(adapter_module, '_torch_runtime', lambda: None)
    predictor.predict(_cloud(),_cloud(),np.array([-1.,1.]*3))
    np.testing.assert_allclose(predictor._model.options['approach_steering'],[0.,0.,1.])
    assert predictor._model.options['approach_thresh']==pytest.approx(np.pi/3)
    assert predictor._model.options['collision_detection']


def test_target_roi_keeps_context_obstacles_and_falls_back_from_empty_mask():
    target=PointCloud(np.array([[0.,0.,.5]]),np.zeros((1,3)))
    context=PointCloud(np.array([[0.,0.,.5],[.005,0.,.5],[.2,0.,.5]]),np.zeros((3,3)))
    predictor=AnyGraspPredictor('', '',region_steering_mode='target_voxels')
    points,mask=predictor._prepare_input(target,context,np.array([-1.,1.]*3))
    assert len(points)==3 and mask.tolist()==[True,True,False]
    far=PointCloud(np.array([[10.,0.,.5]]),np.zeros((1,3)))
    _,mask=predictor._prepare_input(far,context,np.array([-1.,1.]*3))
    assert mask.all()


@pytest.mark.parametrize('shape,count,context_count,expected',[
    ((.1,.1,.1),500,15000,4000),
    ((.1,.1,.001),500,15000,8000),
    ((.1,.1,.1),100,15000,8000),
    ((.1,.1,.1),500,25000,8000),
])
def test_geometry_budget_preserves_thin_sparse_and_high_context_targets(shape,count,context_count,expected):
    rng=np.random.default_rng(12)
    target=PointCloud(rng.uniform(-.5,.5,(count,3))*shape,np.zeros((count,3)))
    context=PointCloud(np.zeros((context_count,3)),np.zeros((context_count,3)))
    predictor=AnyGraspPredictor('', '',adaptive_point_policy='target_geometry')
    assert predictor._sampling_limit(target,context)==expected


def test_geometry_budget_retry_tracks_the_selected_low_budget():
    rng=np.random.default_rng(13)
    target=PointCloud(rng.uniform(-.1,.1,(500,3)),np.zeros((500,3)))
    context=PointCloud(rng.uniform(-1.,1.,(6000,3)),np.zeros((6000,3)))
    predictor=AnyGraspPredictor('', '',adaptive_sampling_enabled=True,adaptive_point_policy='target_geometry')
    points,mask=predictor._prepare_input(target,context,np.array([-1.,1.]*3))
    sampled,_=predictor._adaptive_sample(points,mask)
    assert len(sampled)==4000 and predictor.should_retry_full_resolution(context)


@pytest.mark.parametrize('options',[
    {'approach_direction':(0.,0.,0.)},{'approach_direction':(1.,float('nan'),0.)},
    {'approach_threshold_rad':0.},{'region_steering_mode':'missing'},
    {'target_region_voxel_m':float('inf')},{'adaptive_point_policy':'missing'},
    {'adaptive_point_policy':'target_geometry','adaptive_low_maximum_points':9000},
])
def test_invalid_geometry_controls_are_rejected_before_model_load(options):
    with pytest.raises(ValueError):
        AnyGraspPredictor('', '',**options)
