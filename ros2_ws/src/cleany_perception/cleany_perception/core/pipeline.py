from __future__ import annotations

import math
from collections.abc import Callable, Sequence

import numpy as np

from cleany_perception.core.geometry import (
    deproject_masked_depth,
    fit_plane_ransac,
    reconstruct_supported_obb,
    transform_box,
    transform_plane_normal,
)
from cleany_perception.core.models import (
    Detection2D,
    FailureKind,
    InspectedObject,
    InspectionFailure,
    InspectionOutput,
    InspectionStage,
    ObjectMask,
    PipelineConfig,
    RgbdSnapshot,
    RigidTransform,
)
from cleany_perception.core.ports import (
    DetectorPort,
    SegmenterPort,
    TransformPort,
)


ProgressCallback = Callable[[InspectionStage, int, int, str], None]
CancelCallback = Callable[[], bool]


class InspectionPipeline:
    def __init__(
        self,
        detector: DetectorPort,
        segmenter: SegmenterPort,
        transformer: TransformPort,
        target_frame: str,
        config: PipelineConfig | None = None,
    ) -> None:
        if not target_frame:
            raise ValueError('Inspection target frame must not be empty')
        self._detector = detector
        self._segmenter = segmenter
        self._transformer = transformer
        self._target_frame = target_frame
        self._config = config or PipelineConfig()

    def inspect(
        self,
        snapshot: RgbdSnapshot,
        query: str,
        progress: ProgressCallback | None = None,
        cancelled: CancelCallback | None = None,
        capture_transform: RigidTransform | None = None,
    ) -> InspectionOutput:
        report = progress or (
            lambda _stage, _detected, _objects, _message: None
        )
        is_cancelled = cancelled or (lambda: False)
        self._raise_if_cancelled(is_cancelled)

        report(InspectionStage.DETECTING, 0, 0, 'Detecting objects')
        detections = self._detect(snapshot, query)
        self._raise_if_cancelled(is_cancelled)
        if not detections:
            return InspectionOutput(
                objects=(),
                detections=(),
                masks=(),
                target_frame=self._target_frame,
                plane=None,
            )

        report(
            InspectionStage.SEGMENTING,
            len(detections),
            0,
            'Segmenting detections',
        )
        masks = self._segment(snapshot, detections)
        self._raise_if_cancelled(is_cancelled)

        report(
            InspectionStage.RECONSTRUCTING,
            len(detections),
            0,
            'Reconstructing 3D objects',
        )
        plane, camera_boxes = self._reconstruct(snapshot, detections, masks)
        self._raise_if_cancelled(is_cancelled)

        report(
            InspectionStage.TRANSFORMING,
            len(detections),
            len(camera_boxes),
            f'Transforming objects to {self._target_frame}',
        )
        transform = capture_transform
        if transform is None:
            try:
                transform = self._transformer.lookup(
                    self._target_frame,
                    snapshot.source_frame,
                    snapshot.stamp_ns,
                )
            except InspectionFailure:
                raise
            except Exception as error:
                raise InspectionFailure(
                    FailureKind.TF,
                    f'Failed to look up capture transform: {error}',
                ) from error

        normal_in_target = transform_plane_normal(plane, transform)
        cosine_limit = math.cos(
            math.radians(self._config.maximum_plane_tilt_degrees)
        )
        if float(normal_in_target @ np.array((0.0, 0.0, 1.0))) < cosine_limit:
            raise InspectionFailure(
                FailureKind.PLANE,
                'Support plane exceeds the configured base-frame tilt',
            )

        objects = tuple(
            InspectedObject(
                label=detection.label,
                confidence=detection.confidence,
                box=transform_box(box, transform),
            )
            for detection, box in zip(detections, camera_boxes)
        )
        return InspectionOutput(
            objects=objects,
            detections=detections,
            masks=masks,
            target_frame=self._target_frame,
            plane=plane,
        )

    def _detect(
        self,
        snapshot: RgbdSnapshot,
        query: str,
    ) -> tuple[Detection2D, ...]:
        try:
            raw_detections = tuple(self._detector.detect(snapshot.rgb, query))
        except InspectionFailure:
            raise
        except Exception as error:
            raise InspectionFailure(
                FailureKind.DETECTOR_API,
                f'Detector request failed: {error}',
            ) from error
        detections: list[Detection2D] = []
        for detection in raw_detections:
            if not isinstance(detection, Detection2D):
                raise InspectionFailure(
                    FailureKind.DETECTOR_RESPONSE,
                    'Detector returned an unsupported detection object',
                )
            if detection.bbox.x_max > snapshot.intrinsics.width:
                raise InspectionFailure(
                    FailureKind.DETECTOR_RESPONSE,
                    'Detector bounding box exceeds image width',
                )
            if detection.bbox.y_max > snapshot.intrinsics.height:
                raise InspectionFailure(
                    FailureKind.DETECTOR_RESPONSE,
                    'Detector bounding box exceeds image height',
                )
            if (
                detection.confidence
                < self._config.minimum_detection_confidence
            ):
                continue
            detections.append(detection)
        detections.sort(key=lambda item: item.confidence, reverse=True)
        return tuple(detections[: self._config.maximum_detections])

    def _segment(
        self,
        snapshot: RgbdSnapshot,
        detections: Sequence[Detection2D],
    ) -> tuple[ObjectMask, ...]:
        try:
            masks = tuple(self._segmenter.segment(snapshot.rgb, detections))
        except InspectionFailure:
            raise
        except Exception as error:
            raise InspectionFailure(
                FailureKind.MASK,
                f'Segmenter request failed: {error}',
            ) from error
        if len(masks) != len(detections):
            raise InspectionFailure(
                FailureKind.MASK,
                'Segmenter did not return one mask per detection',
            )
        for detection, object_mask in zip(detections, masks):
            if not isinstance(object_mask, ObjectMask):
                raise InspectionFailure(
                    FailureKind.MASK,
                    'Segmenter returned an unsupported mask object',
                )
            if object_mask.detection != detection:
                raise InspectionFailure(
                    FailureKind.MASK,
                    'Segmenter changed detection ordering',
                )
            if object_mask.mask.shape != snapshot.depth_m.shape:
                raise InspectionFailure(
                    FailureKind.MASK,
                    'Segmenter mask shape does not match depth image',
                )
            if not np.any(object_mask.mask):
                raise InspectionFailure(
                    FailureKind.MASK,
                    'Segmenter returned an empty mask',
                )
        return masks

    def _reconstruct(
        self,
        snapshot: RgbdSnapshot,
        detections: Sequence[Detection2D],
        masks: Sequence[ObjectMask],
    ):
        support_selection = self._support_selection(
            snapshot.depth_m.shape,
            detections,
            masks,
        )
        support_points = deproject_masked_depth(
            snapshot.depth_m,
            snapshot.intrinsics,
            support_selection,
            self._config.minimum_depth_m,
            self._config.maximum_depth_m,
            self._config.support_sample_stride,
        )
        try:
            plane = fit_plane_ransac(
                support_points,
                self._config.plane_ransac_iterations,
                self._config.plane_distance_threshold_m,
                self._config.plane_minimum_inliers,
                self._config.plane_minimum_inlier_ratio,
            )
        except ValueError as error:
            raise InspectionFailure(FailureKind.PLANE, str(error)) from error

        boxes = []
        for object_mask in masks:
            points = deproject_masked_depth(
                snapshot.depth_m,
                snapshot.intrinsics,
                object_mask.mask,
                self._config.minimum_depth_m,
                self._config.maximum_depth_m,
            )
            if points.shape[0] < self._config.minimum_object_points:
                raise InspectionFailure(
                    FailureKind.DEPTH,
                    'Not enough valid depth for '
                    f'{object_mask.detection.label}',
                )
            try:
                boxes.append(
                    reconstruct_supported_obb(
                        points,
                        plane,
                        self._config.minimum_object_height_m,
                        self._config.minimum_obb_extent_m,
                        self._config.minimum_object_points,
                    )
                )
            except ValueError as error:
                raise InspectionFailure(
                    FailureKind.DEPTH,
                    'Failed to reconstruct '
                    f'{object_mask.detection.label}: {error}',
                ) from error
        return plane, tuple(boxes)

    def _support_selection(
        self,
        shape: tuple[int, int],
        detections: Sequence[Detection2D],
        masks: Sequence[ObjectMask],
    ) -> np.ndarray:
        height, width = shape
        selection = np.zeros(shape, dtype=np.bool_)
        margin = self._config.support_margin_pixels
        for detection in detections:
            x_min = max(0, math.floor(detection.bbox.x_min) - margin)
            y_min = max(0, math.floor(detection.bbox.y_min) - margin)
            x_max = min(width, math.ceil(detection.bbox.x_max) + margin)
            y_max = min(height, math.ceil(detection.bbox.y_max) + margin)
            selection[y_min:y_max, x_min:x_max] = True
        for object_mask in masks:
            selection &= ~object_mask.mask
        return selection

    @staticmethod
    def _raise_if_cancelled(cancelled: CancelCallback) -> None:
        if cancelled():
            raise InspectionFailure(
                FailureKind.CANCELLED,
                'Inspection was cancelled',
            )
