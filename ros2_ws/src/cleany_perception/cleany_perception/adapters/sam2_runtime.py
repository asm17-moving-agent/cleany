from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import threading
from typing import Any

from cleany_perception.core.models import FailureKind, InspectionFailure


class Sam2SharedPredictors:
    """One SAM2 video model shared by image segmentation and tracking."""

    def __init__(
        self,
        model_config: str,
        checkpoint_path: str,
        device: str,
        *,
        model_factory: Callable[[str, str, str], Any] | None = None,
        image_predictor_factory: Callable[[Any], Any] | None = None,
    ) -> None:
        if not device:
            raise ValueError('SAM2 device must not be empty')
        self._model_config = model_config
        self._checkpoint_path = checkpoint_path
        self._device = device
        self._model_factory = model_factory
        self._image_predictor_factory = image_predictor_factory
        self._video_predictor = None
        self._image_predictor = None
        self.lock = threading.RLock()

    def video_predictor(self):
        with self.lock:
            if self._video_predictor is not None:
                return self._video_predictor
            self._validate_assets()
            try:
                if self._model_factory is not None:
                    predictor = self._model_factory(
                        self._model_config,
                        self._checkpoint_path,
                        self._device,
                    )
                else:
                    from sam2.build_sam import build_sam2_video_predictor

                    predictor = build_sam2_video_predictor(
                        self._model_config,
                        self._checkpoint_path,
                        device=self._device,
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
                    f'Failed to load shared SAM2 model: {error}',
                ) from error
            self._video_predictor = predictor
            return predictor

    def image_predictor(self):
        with self.lock:
            if self._image_predictor is not None:
                return self._image_predictor
            model = self.video_predictor()
            try:
                if self._image_predictor_factory is not None:
                    predictor = self._image_predictor_factory(model)
                else:
                    from sam2.sam2_image_predictor import SAM2ImagePredictor

                    predictor = SAM2ImagePredictor(model)
            except ImportError as error:
                raise InspectionFailure(
                    FailureKind.MASK,
                    'SAM2 is not installed',
                ) from error
            except Exception as error:
                raise InspectionFailure(
                    FailureKind.MASK,
                    f'Failed to create shared SAM2 image predictor: {error}',
                ) from error
            self._image_predictor = predictor
            return predictor

    def _validate_assets(self) -> None:
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
