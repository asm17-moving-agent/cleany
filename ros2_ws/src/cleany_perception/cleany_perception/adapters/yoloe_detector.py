from __future__ import annotations

import os
import threading
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from cleany_perception.core.models import (
    BoundingBox2D,
    Detection2D,
    FailureKind,
    InspectionFailure,
    RgbArray,
)


ModelFactory = Callable[[str], Any]

_TEXT_ENCODER_FILENAME = 'mobileclip2_b.ts'
_MODEL_SETUP_LOCK = threading.Lock()


def _numpy(value: Any) -> np.ndarray:
    if hasattr(value, 'detach'):
        value = value.detach()
    if hasattr(value, 'cpu'):
        value = value.cpu()
    return np.asarray(value)


class YoloeDetector:
    """Ultralytics YOLOE text-prompt adapter returning bbox results only."""

    def __init__(
        self,
        model_path: str,
        classes: Sequence[str],
        *,
        device: str = 'cuda',
        image_size: int = 640,
        confidence_threshold: float = 0.25,
        iou_threshold: float = 0.5,
        maximum_detections: int = 10,
        text_encoder_directory: str = '',
        model_factory: ModelFactory | None = None,
    ) -> None:
        normalized_classes = tuple(item.strip() for item in classes)
        if not model_path:
            raise ValueError('YOLOE model path must not be empty')
        if not normalized_classes or any(
            not item for item in normalized_classes
        ):
            raise ValueError('YOLOE classes must contain non-empty labels')
        if len(set(normalized_classes)) != len(normalized_classes):
            raise ValueError('YOLOE classes must be unique')
        if not device:
            raise ValueError('YOLOE device must not be empty')
        if image_size <= 0:
            raise ValueError('YOLOE image size must be positive')
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError('YOLOE confidence threshold must be in [0, 1]')
        if not 0.0 <= iou_threshold <= 1.0:
            raise ValueError('YOLOE IoU threshold must be in [0, 1]')
        if maximum_detections <= 0:
            raise ValueError('YOLOE maximum detections must be positive')

        self._model_path = str(Path(model_path).expanduser().resolve())
        self._classes = normalized_classes
        self._device = device
        self._image_size = image_size
        self._confidence_threshold = confidence_threshold
        self._iou_threshold = iou_threshold
        self._maximum_detections = maximum_detections
        self._text_encoder_directory = str(
            Path(text_encoder_directory).expanduser().resolve()
        )
        self._model_factory = model_factory
        self._model = None

    def prepare(self) -> None:
        """Load checkpoint and text prompts before accepting requests."""
        self._get_model()

    def detect(
        self,
        rgb: RgbArray,
        query: str,
    ) -> Sequence[Detection2D]:
        del query
        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                'YOLOE detector requires an HxWx3 uint8 RGB image',
            )

        model = self._get_model()
        try:
            results = model.predict(
                # Ultralytics interprets NumPy images as BGR; our port is RGB.
                np.ascontiguousarray(image[..., ::-1]),
                device=self._device,
                imgsz=self._image_size,
                conf=self._confidence_threshold,
                iou=self._iou_threshold,
                max_det=self._maximum_detections,
                verbose=False,
            )
        except InspectionFailure:
            raise
        except Exception as error:
            raise InspectionFailure(
                FailureKind.DETECTOR_API,
                f'YOLOE inference failed: {error}',
            ) from error

        try:
            if len(results) != 1:
                raise ValueError('YOLOE must return one result per snapshot')
            result = results[0]
            boxes = getattr(result, 'boxes', None)
            if boxes is None:
                return ()
            names = getattr(result, 'names', None) or model.names
            height, width = image.shape[:2]
            detections = []
            for box in boxes:
                coordinates = _numpy(box.xyxy).reshape(-1, 4)[0]
                class_id = int(_numpy(box.cls).reshape(-1)[0])
                confidence = float(_numpy(box.conf).reshape(-1)[0])
                if isinstance(names, dict):
                    label = names[class_id]
                else:
                    label = names[class_id]
                x_min, y_min, x_max, y_max = (
                    float(value) for value in coordinates
                )
                x_min = min(max(x_min, 0.0), float(width))
                y_min = min(max(y_min, 0.0), float(height))
                x_max = min(max(x_max, 0.0), float(width))
                y_max = min(max(y_max, 0.0), float(height))
                detections.append(
                    Detection2D(
                        label=str(label).strip(),
                        confidence=confidence,
                        bbox=BoundingBox2D(
                            x_min=x_min,
                            y_min=y_min,
                            x_max=x_max,
                            y_max=y_max,
                        ),
                    )
                )
            return tuple(detections)
        except InspectionFailure:
            raise
        except (IndexError, KeyError, TypeError, ValueError) as error:
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                f'YOLOE returned invalid detections: {error}',
            ) from error

    def _get_model(self):
        if self._model is not None:
            return self._model
        if not Path(self._model_path).is_file():
            raise InspectionFailure(
                FailureKind.DETECTOR_API,
                f'YOLOE checkpoint not found: {self._model_path}',
            )
        encoder_directory = Path(self._text_encoder_directory)
        encoder_path = encoder_directory / _TEXT_ENCODER_FILENAME
        if not encoder_path.is_file():
            raise InspectionFailure(
                FailureKind.DETECTOR_API,
                f'YOLOE text encoder not found: {encoder_path}',
            )

        try:
            factory = self._model_factory
            if factory is None:
                from ultralytics import YOLOE

                factory = YOLOE
            # Ultralytics resolves mobileclip2_b.ts relative to the process
            # working directory. Restrict that global change to serialized
            # one-time model setup and restore it immediately afterwards.
            with _MODEL_SETUP_LOCK:
                original_directory = Path.cwd()
                try:
                    os.chdir(encoder_directory)
                    model = factory(self._model_path)
                    model.to(self._device)
                    model.set_classes(list(self._classes))
                finally:
                    os.chdir(original_directory)
        except InspectionFailure:
            raise
        except ImportError as error:
            raise InspectionFailure(
                FailureKind.DETECTOR_API,
                'Ultralytics YOLOE is not installed',
            ) from error
        except Exception as error:
            raise InspectionFailure(
                FailureKind.DETECTOR_API,
                f'Failed to load YOLOE: {error}',
            ) from error
        self._model = model
        return model
