from __future__ import annotations

import importlib.util
from pathlib import Path
from types import ModuleType
import unittest

import numpy as np


def _load_module() -> ModuleType:
    path = Path(__file__).with_name('sam2_smoke.py')
    spec = importlib.util.spec_from_file_location('sam2_smoke', path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


smoke = _load_module()


class Sam2SmokeTest(unittest.TestCase):
    def test_synthetic_image_contains_central_prompt_object(self) -> None:
        image = smoke.synthetic_image(640, 480)
        box = smoke.central_box(640, 480)

        self.assertEqual(image.shape, (480, 640, 3))
        self.assertEqual(image.dtype, np.uint8)
        self.assertEqual(box, (160, 120, 480, 360))
        self.assertTrue((image[120:360, 160:480] == (220, 80, 40)).all())

    def test_central_box_rejects_too_small_image(self) -> None:
        with self.assertRaises(ValueError):
            smoke.central_box(3, 480)

    def test_select_mask_accepts_predictor_batch_shape(self) -> None:
        predicted = np.zeros((1, 20, 30), dtype=np.bool_)
        predicted[0, 2:8, 4:10] = True

        mask, score = smoke.select_mask(predicted, [0.91], (20, 30))

        self.assertEqual(mask.shape, (20, 30))
        self.assertEqual(np.count_nonzero(mask), 36)
        self.assertAlmostEqual(score, 0.91)

    def test_select_mask_rejects_mismatched_resolution(self) -> None:
        with self.assertRaisesRegex(ValueError, 'does not match image'):
            smoke.select_mask(
                np.zeros((1, 5, 5), dtype=np.bool_),
                [0.5],
                (20, 30),
            )


if __name__ == '__main__':
    unittest.main()
