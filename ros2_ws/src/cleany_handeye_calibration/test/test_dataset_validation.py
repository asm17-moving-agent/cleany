from pathlib import Path

import cv2
import numpy as np
import pytest

from cleany_handeye_calibration.dataset_validation import (
    validate_rendered_samples,
)
from cleany_handeye_calibration.pnp import PnpCandidate, PnpResult
from cleany_handeye_calibration.target_detector import (
    analyze_charuco_corners,
)
from evaluation_test_support import evaluation_records


class FixedDetector:
    def __init__(self, detection):
        self._detection = detection

    def detect(self, image):
        assert image.shape == (480, 640, 3)
        return self._detection


def _write_image(root: Path, image_path: str) -> None:
    path = root / image_path
    path.parent.mkdir(parents=True, exist_ok=True)
    assert cv2.imwrite(
        str(path),
        np.full((480, 640, 3), 255, dtype=np.uint8),
    )


def test_saved_image_validation_reproduces_detection_and_pnp(tmp_path):
    record = evaluation_records()[0]
    _write_image(tmp_path, record.image_path)

    def solve(detection, **kwargs):
        del detection, kwargs
        transform = record.sample.camera_T_target
        candidates = (
            PnpCandidate(
                0,
                True,
                None,
                transform,
                0.4,
                record.pnp_reprojection_rmse_px,
                transform,
                0.4,
                record.pnp_reprojection_rmse_px,
            ),
            PnpCandidate(
                1,
                True,
                None,
                transform,
                0.4,
                record.pnp_reprojection_rmse_px * 20.0,
                transform,
                0.4,
                record.pnp_reprojection_rmse_px * 20.0,
            ),
        )
        return PnpResult(
            True,
            'SOLVEPNP_IPPE',
            None,
            None,
            False,
            record.pnp_selected_candidate_index,
            transform,
            candidates,
        )

    summary = validate_rendered_samples(
        (record,),
        tmp_path,
        detector=FixedDetector(record.target_detection),
        pnp_solver=solve,
    )

    assert summary.sample_count == 1
    assert summary.minimum_corner_count == 24
    assert summary.maximum_corner_count == 24
    assert summary.minimum_reprojection_rmse_px == (
        record.pnp_reprojection_rmse_px
    )
    assert summary.minimum_candidate_rmse_ratio == pytest.approx(20.0)


def test_saved_image_validation_rejects_detection_drift(tmp_path):
    record = evaluation_records()[0]
    _write_image(tmp_path, record.image_path)
    changed = analyze_charuco_corners(
        record.target_detection.corner_ids,
        tuple(
            (x + 1.0, y)
            for x, y in record.target_detection.image_points_px
        ),
    )

    assert changed.valid
    with pytest.raises(ValueError, match='detection differs'):
        validate_rendered_samples(
            (record,),
            tmp_path,
            detector=FixedDetector(changed),
        )
