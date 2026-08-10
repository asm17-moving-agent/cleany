from __future__ import annotations

from collections.abc import Callable, Sequence
from contextlib import nullcontext
from pathlib import Path
from typing import Any

import numpy as np

from cleany_perception.core.models import (
    Detection2D,
    FailureKind,
    InspectionFailure,
    ObjectMask,
    RgbArray,
)


PredictorFactory = Callable[[str, str, str], Any]


class Sam2Segmenter:
    def __init__(
        self,
        model_config: str,
        checkpoint_path: str,
        device: str = 'cuda',
        predictor_factory: PredictorFactory | None = None,
    ) -> None:
        if not device:
            raise ValueError('SAM2 device must not be empty')
        self._model_config = model_config
        self._checkpoint_path = checkpoint_path
        self._device = device
        self._predictor_factory = predictor_factory
        self._predictor = None

    def segment(
        self,
        rgb: RgbArray,
        detections: Sequence[Detection2D],
    ) -> Sequence[ObjectMask]:
        if not detections:
            return ()
        predictor = self._get_predictor()
        try:
            inference_context = nullcontext()
            if self._predictor_factory is None:
                import torch

                inference_context = torch.inference_mode()
            with inference_context:
                predictor.set_image(np.asarray(rgb))
                masks = []
                for detection in detections:
                    box = np.array(
                        [
                            detection.bbox.x_min,
                            detection.bbox.y_min,
                            detection.bbox.x_max,
                            detection.bbox.y_max,
                        ],
                        dtype=np.float32,
                    )
                    predicted_masks, scores, _logits = predictor.predict(
                        box=box,
                        multimask_output=False,
                    )
                    mask_array = np.asarray(predicted_masks)
                    score_array = np.asarray(
                        scores,
                        dtype=np.float64,
                    ).reshape(-1)
                    if mask_array.ndim == 2:
                        selected_mask = mask_array
                    elif mask_array.ndim == 3 and mask_array.shape[0] >= 1:
                        selected_mask = mask_array[0]
                    elif (
                        mask_array.ndim == 4
                        and mask_array.shape[:2] == (1, 1)
                    ):
                        selected_mask = mask_array[0, 0]
                    else:
                        raise ValueError(
                            'Unexpected SAM2 mask shape: '
                            f'{mask_array.shape}'
                        )
                    if selected_mask.shape != rgb.shape[:2]:
                        raise ValueError(
                            'SAM2 mask does not match the RGB image'
                        )
                    score = float(score_array[0]) if score_array.size else 0.0
                    masks.append(
                        ObjectMask(
                            detection=detection,
                            mask=selected_mask.astype(np.bool_),
                            score=score,
                        )
                    )
            return tuple(masks)
        except InspectionFailure:
            raise
        except Exception as error:
            raise InspectionFailure(
                FailureKind.MASK,
                f'SAM2 inference failed: {error}',
            ) from error

    def _get_predictor(self):
        if self._predictor is not None:
            return self._predictor
        if not self._model_config:
            raise InspectionFailure(
                FailureKind.MASK,
                'SAM2 model config parameter is empty',
            )
        if not self._checkpoint_path:
            raise InspectionFailure(
                FailureKind.MASK,
                'SAM2 checkpoint parameter is empty',
            )
        if not Path(self._checkpoint_path).is_file():
            raise InspectionFailure(
                FailureKind.MASK,
                f'SAM2 checkpoint not found: {self._checkpoint_path}',
            )
        try:
            if self._predictor_factory is not None:
                predictor = self._predictor_factory(
                    self._model_config,
                    self._checkpoint_path,
                    self._device,
                )
            else:
                from sam2.build_sam import build_sam2
                from sam2.sam2_image_predictor import SAM2ImagePredictor

                predictor = SAM2ImagePredictor(
                    build_sam2(
                        self._model_config,
                        self._checkpoint_path,
                        device=self._device,
                    )
                )
        except InspectionFailure:
            raise
        except ImportError as error:
            raise InspectionFailure(
                FailureKind.MASK,
                'SAM2 is not installed',
            ) from error
        except Exception as error:
            raise InspectionFailure(
                FailureKind.MASK,
                f'Failed to load SAM2: {error}',
            ) from error
        self._predictor = predictor
        return predictor
