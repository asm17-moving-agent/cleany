from __future__ import annotations

import os
import threading
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any
from contextlib import nullcontext

import numpy as np

from cleany_perception.model_contract import (
    check_segmentation_shapes, ordered_names, read_model_manifest,
)

from cleany_perception.core.models import (
    BoundingBox2D,
    Detection2D,
    FailureKind,
    InspectionFailure,
    ObjectMask,
    RgbArray,
)


ModelFactory = Callable[..., Any]

_TEXT_ENCODER_FILENAME = 'mobileclip2_b.ts'
_MODEL_SETUP_LOCK = threading.Lock()


class YoloeModelPool:
    """One inspector's validated backends, with serialized setup/inference."""
    def __init__(self) -> None:
        self.models: dict[tuple, Any] = {}
        self.lock = threading.RLock()


def _numpy(value: Any) -> np.ndarray:
    if hasattr(value, 'detach'):
        value = value.detach()
    if hasattr(value, 'cpu'):
        value = value.cpu()
    return np.asarray(value)


class YoloeDetector:
    """Ultralytics YOLOE adapter with optional instance masks."""

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
        require_masks: bool = False,
        class_confidence_thresholds: Sequence[float] = (),
        allow_unvalidated_engine: bool = False,
        model_pool: YoloeModelPool | None = None,
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
        if (class_confidence_thresholds
                and len(class_confidence_thresholds) != len(normalized_classes)):
            raise ValueError(
                'YOLOE class confidence thresholds must match classes'
            )
        if any(not confidence_threshold <= value <= 1.0
               for value in class_confidence_thresholds):
            raise ValueError(
                'YOLOE class confidence thresholds must be in '
                '[global threshold, 1]'
            )

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
        self._require_masks = require_masks
        self._class_confidence_thresholds = dict(zip(
            normalized_classes, class_confidence_thresholds
        ))
        self._model = None
        self._is_engine = Path(self._model_path).suffix == '.engine'
        self._allow_unvalidated_engine = allow_unvalidated_engine
        self._model_pool = model_pool

    def _predict(self, model: Any, image: np.ndarray) -> Any:
        options = dict(
            device=self._device, imgsz=self._image_size,
            conf=self._confidence_threshold, iou=self._iou_threshold,
            max_det=self._maximum_detections, verbose=False, rect=False,
        )
        if self._require_masks:
            options['retina_masks'] = True
        with self._model_pool.lock if self._model_pool is not None else nullcontext():
            return model.predict(np.ascontiguousarray(image[..., ::-1]), **options)

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
            # Ultralytics restores boxes and retina masks to original pixels.
            results = self._predict(model, image)
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
            height, width = image.shape[:2]
            mask_data = None
            if self._require_masks and len(boxes):
                masks = getattr(result, 'masks', None)
                if masks is None:
                    raise ValueError('YOLOE segmentation result has no masks')
                mask_data = _numpy(masks.data)
                if mask_data.shape != (len(boxes), height, width):
                    raise ValueError('YOLOE mask count or image shape differs from boxes')
            names = getattr(result, 'names', None) or model.names
            if ordered_names(names) != list(self._classes):
                raise ValueError('YOLOE result class order differs from profile')
            detections = []
            for index, box in enumerate(boxes):
                coordinates = _numpy(box.xyxy).reshape(-1, 4)[0]
                class_id = int(_numpy(box.cls).reshape(-1)[0])
                confidence = float(_numpy(box.conf).reshape(-1)[0])
                if isinstance(names, dict):
                    label = names[class_id]
                else:
                    label = names[class_id]
                minimum = self._class_confidence_thresholds.get(
                    label, self._confidence_threshold
                )
                if confidence < minimum:
                    continue
                x_min, y_min, x_max, y_max = (
                    float(value) for value in coordinates
                )
                x_min = min(max(x_min, 0.0), float(width))
                y_min = min(max(y_min, 0.0), float(height))
                x_max = min(max(x_max, 0.0), float(width))
                y_max = min(max(y_max, 0.0), float(height))
                mask = None
                if mask_data is not None:
                    mask = np.array(mask_data[index] > 0.5, dtype=np.bool_, copy=True)
                    mask.flags.writeable = False
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
                        segmentation_mask=mask,
                    )
                )
            return tuple(detections)
        except InspectionFailure:
            raise
        except (AttributeError, IndexError, KeyError, TypeError, ValueError) as error:
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                f'YOLOE returned invalid detections: {error}',
            ) from error

    def _get_model(self) -> Any:
        if self._model is not None:
            return self._model
        if self._model_pool is None:
            return self._load_model()
        # Per-call thresholds/NMS are wrapper settings, not backend identity.
        key = (self._model_path, self._classes, self._device, self._image_size,
               self._require_masks, self._text_encoder_directory,
               self._allow_unvalidated_engine, self._model_factory)
        with self._model_pool.lock:
            if key not in self._model_pool.models:
                self._model_pool.models[key] = self._load_model()
            self._model = self._model_pool.models[key]
            return self._model

    def _load_model(self) -> Any:
        if not Path(self._model_path).is_file():
            raise InspectionFailure(
                FailureKind.DETECTOR_API,
                f'YOLOE checkpoint not found: {self._model_path}',
            )
        if self._is_engine:
            return self._load_engine()
        # Prepared PT already contains the exact export text embeddings.
        if (Path(self._model_path).parent / 'manifest.json').is_file():
            return self._load_prepared_checkpoint()
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

    def _load_prepared_checkpoint(self) -> Any:
        try:
            read_model_manifest(Path(self._model_path), self._classes, self._image_size)
            factory = self._model_factory
            if factory is None:
                from ultralytics import YOLOE
                factory = YOLOE
            model = factory(self._model_path)
            model.to(self._device)
            if model.task != 'segment' or ordered_names(model.names) != list(self._classes):
                raise ValueError('Prepared checkpoint task or classes differ from profile')
            self._model = model
            return model
        except Exception as error:
            raise InspectionFailure(FailureKind.DETECTOR_API,
                                    f'Failed to load prepared YOLOE: {error}') from error

    def _load_engine(self) -> Any:
        try:
            if not self._require_masks:
                raise ValueError('TensorRT profile requires segmentation masks')
            manifest = read_model_manifest(
                Path(self._model_path), self._classes, self._image_size,
                allow_unvalidated=self._allow_unvalidated_engine,
            )
            factory = self._model_factory
            if factory is None:
                from ultralytics import YOLO
                factory = YOLO
            model = factory(self._model_path, task='segment')
            # Initialize the lazy backend once, exercising its segmentation postprocessor.
            self._predict(model, np.zeros((self._image_size, self._image_size, 3),
                                          dtype=np.uint8))
            backend = model.predictor.model
            if (model.task != 'segment' or ordered_names(model.names) != list(self._classes)
                    or backend.dynamic):
                raise ValueError('TensorRT task, class order or static input differs')
            shapes = {name: list(binding.shape) for name, binding in backend.bindings.items()}
            inputs = {'images': shapes.pop('images')}
            check_segmentation_shapes(inputs, shapes, self._image_size, len(self._classes))
            if inputs != manifest['inputs'] or shapes != manifest['outputs']:
                raise ValueError('TensorRT bindings differ from manifest')
            self._model = model
            return model
        except Exception as error:
            raise InspectionFailure(FailureKind.DETECTOR_API,
                                    f'Failed to load YOLOE TensorRT: {error}') from error


class YoloeMaskSegmenter:
    """Use masks from the matching YOLOE detection, without a second model call."""

    def prepare(self) -> None:
        pass

    def segment(self, rgb: RgbArray, detections: Sequence[Detection2D]) -> Sequence[ObjectMask]:
        shape = np.asarray(rgb).shape[:2]
        masks = []
        for detection in detections:
            mask = detection.segmentation_mask
            if mask is None or mask.shape != shape or not np.any(mask):
                raise InspectionFailure(FailureKind.MASK,
                                        'YOLOE detection has no valid full-resolution mask')
            masks.append(ObjectMask(detection, mask, detection.confidence))
        return tuple(masks)
