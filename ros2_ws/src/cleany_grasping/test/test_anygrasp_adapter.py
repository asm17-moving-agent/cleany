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
        predictor.predict(_cloud(), np.asarray((-1.0, 1.0) * 3))


def test_requires_sdk_license_directory_name(tmp_path) -> None:
    checkpoint = tmp_path / 'checkpoint.tar'
    checkpoint.write_bytes(b'checkpoint')
    license_dir = tmp_path / 'credentials'
    license_dir.mkdir()
    license_file = license_dir / 'test.lic'
    license_file.write_bytes(b'license')
    predictor = AnyGraspPredictor(str(checkpoint), str(license_file))

    with pytest.raises(ModelUnavailableError, match='directory named license'):
        predictor.predict(_cloud(), np.asarray((-1.0, 1.0) * 3))
