from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import replace

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
from cleany_perception.core.reconstruction_cache import ReconstructionCache


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
        """Detect, segment and reconstruct one RGB-D snapshot.

        Args:
            capture_transform: Optional source-to-target transform at capture time.
                If omitted, look it up at snapshot.stamp_ns through the transform port.

        Returns:
            Objects in target_frame, image-space detections and masks, and the
            support plane in the snapshot's source frame. No detections yields an
            empty result with no plane.

        Raises:
            InspectionFailure: A processing stage fails or cancellation is observed
                between stages; in-flight port calls are not interrupted.
        """
        report = progress or (
            lambda _stage, _detected, _objects, _message: None
        )
        is_cancelled = cancelled or (lambda: False)
        detections = self.detect(
            snapshot,
            query,
            progress=report,
            cancelled=is_cancelled,
        )
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

    def detect(
        self,
        snapshot: RgbdSnapshot,
        query: str,
        progress: ProgressCallback | None = None,
        cancelled: CancelCallback | None = None,
        *,
        capture_transform: RigidTransform | None = None,
        geometry_cache: ReconstructionCache | None = None,
    ) -> tuple[Detection2D, ...]:
        report = progress or (
            lambda _stage, _detected, _objects, _message: None
        )
        is_cancelled = cancelled or (lambda: False)
        self._raise_if_cancelled(is_cancelled)
        report(InspectionStage.DETECTING, 0, 0, 'Detecting objects')
        def preprocess(raw_detections):
            self.estimate_detection_footprints(
                snapshot, self._validate_detections(snapshot, raw_detections),
                capture_transform, cancelled=is_cancelled, geometry_cache=geometry_cache)

        detections = self._detect(snapshot, query,
            preprocess=preprocess if capture_transform is not None and geometry_cache is not None else None)
        self._raise_if_cancelled(is_cancelled)
        return detections

    def inspect_selected(
        self,
        snapshot: RgbdSnapshot,
        all_detections: Sequence[Detection2D],
        selected_detection: Detection2D,
        capture_transform: RigidTransform,
        progress: ProgressCallback | None = None,
        cancelled: CancelCallback | None = None,
        *,
        geometry_cache: ReconstructionCache | None = None,
    ) -> InspectionOutput:
        """Reconstruct one selected detection using its original capture transform.

        All detection boxes are excluded when choosing support-plane pixels.
        The output uses the same frame conventions as inspect().

        Raises:
            ValueError: The selection is absent from all_detections.
            InspectionFailure: Reconstruction, tilt validation or cancellation fails.
        """
        if selected_detection not in all_detections:
            raise ValueError('Selected detection must belong to the snapshot')
        report = progress or (
            lambda _stage, _detected, _objects, _message: None
        )
        is_cancelled = cancelled or (lambda: False)
        self._raise_if_cancelled(is_cancelled)
        detection_count = len(all_detections)

        report(
            InspectionStage.SEGMENTING,
            detection_count,
            0,
            f'Segmenting selected object: {selected_detection.label}',
        )
        masks = self._segment(snapshot, (selected_detection,))
        self._raise_if_cancelled(is_cancelled)

        report(
            InspectionStage.RECONSTRUCTING,
            detection_count,
            0,
            'Reconstructing selected 3D object',
        )
        plane, camera_boxes = self._reconstruct(
            snapshot,
            all_detections,
            masks,
            exclude_detection_boxes=True,
            geometry_cache=geometry_cache,
        )
        self._raise_if_cancelled(is_cancelled)

        report(
            InspectionStage.TRANSFORMING,
            detection_count,
            1,
            f'Transforming selected object to {self._target_frame}',
        )
        normal_in_target = transform_plane_normal(
            plane,
            capture_transform,
        )
        cosine_limit = math.cos(
            math.radians(self._config.maximum_plane_tilt_degrees)
        )
        if float(normal_in_target @ np.array((0.0, 0.0, 1.0))) < cosine_limit:
            raise InspectionFailure(
                FailureKind.PLANE,
                'Support plane exceeds the configured base-frame tilt',
            )
        inspected = InspectedObject(
            label=selected_detection.label,
            confidence=selected_detection.confidence,
            box=transform_box(camera_boxes[0], capture_transform),
        )
        return InspectionOutput(
            objects=(inspected,),
            detections=(selected_detection,),
            masks=masks,
            target_frame=self._target_frame,
            plane=plane,
        )

    def estimate_detection_footprints(
        self,
        snapshot: RgbdSnapshot,
        detections: Sequence[Detection2D],
        capture_transform: RigidTransform,
        cancelled: CancelCallback | None = None,
        geometry_cache: ReconstructionCache | None = None,
    ) -> tuple[Detection2D, ...]:
        """Estimate support-plane extents without another segmentation call.

        Image-axis bbox lengths mix footprint, object height and camera tilt.
        Reuse detector instance masks when available; otherwise use bbox depth
        above the fitted support plane for this preliminary estimate. Selected
        inspection and grasp opening checks remain separate requirements.
        Unmeasurable extents stay zero instead of substituting image dimensions.
        """
        is_cancelled = cancelled or (lambda: False)
        estimated = []
        for detection in detections:
            self._raise_if_cancelled(is_cancelled)
            measured = replace(detection, estimated_width_m=0.0, estimated_height_m=0.0)
            mask = detection.segmentation_mask
            if mask is None:
                mask = np.zeros(snapshot.depth_m.shape, dtype=np.bool_)
                bbox = detection.bbox
                mask[math.floor(bbox.y_min):math.ceil(bbox.y_max),
                     math.floor(bbox.x_min):math.ceil(bbox.x_max)] = True
            try:
                if mask.shape != snapshot.depth_m.shape or not np.any(mask):
                    raise ValueError('Detection mask does not cover valid image pixels')
                plane, boxes = self._reconstruct(snapshot, detections,
                    (ObjectMask(detection, mask, detection.confidence),),
                    exclude_detection_boxes=True, geometry_cache=geometry_cache)
                normal = transform_plane_normal(plane, capture_transform)
                if float(normal @ np.array((0.0, 0.0, 1.0))) < math.cos(
                        math.radians(self._config.maximum_plane_tilt_degrees)):
                    raise ValueError('Support plane exceeds the configured base-frame tilt')
                measured = replace(measured,
                    estimated_width_m=float(boxes[0].size[0]),
                    estimated_height_m=float(boxes[0].size[1]))
            except (InspectionFailure, ValueError):
                # A bad depth/plane estimate for one object must not hide other
                # detections. Zero extents make its size eligibility explicit.
                pass
            self._raise_if_cancelled(is_cancelled)
            estimated.append(measured)
        return tuple(estimated)

    def _detect(
        self,
        snapshot: RgbdSnapshot,
        query: str,
        preprocess: Callable | None = None,
    ) -> tuple[Detection2D, ...]:
        try:
            concurrent = getattr(self._detector, 'detect_with_preprocessing', None)
            raw_detections = tuple(concurrent(snapshot.rgb, query, preprocess)
                if preprocess is not None and callable(concurrent)
                else self._detector.detect(snapshot.rgb, query))
        except InspectionFailure:
            raise
        except Exception as error:
            raise InspectionFailure(
                FailureKind.DETECTOR_API,
                f'Detector request failed: {error}',
            ) from error
        return self._validate_detections(snapshot, raw_detections)

    def _validate_detections(self, snapshot, raw_detections) -> tuple[Detection2D, ...]:
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
        exclude_detection_boxes: bool = False,
        geometry_cache: ReconstructionCache | None = None,
    ):
        support_selection = self._support_selection(
            snapshot.depth_m.shape,
            detections,
            masks,
            exclude_detection_boxes,
        )
        if geometry_cache is not None:
            geometry_cache.bind(snapshot, self._config)
        support_key = ReconstructionCache.array_key(support_selection)

        def estimate_plane():
            support_points = deproject_masked_depth(
                snapshot.depth_m, snapshot.intrinsics, support_selection,
                self._config.minimum_depth_m, self._config.maximum_depth_m,
                self._config.support_sample_stride)
            return fit_plane_ransac(support_points,
                self._config.plane_ransac_iterations,
                self._config.plane_distance_threshold_m,
                self._config.plane_minimum_inliers,
                self._config.plane_minimum_inlier_ratio)

        try:
            plane = (geometry_cache.get_or_compute(('plane', support_key), estimate_plane)
                     if geometry_cache is not None else estimate_plane())
        except ValueError as error:
            raise InspectionFailure(FailureKind.PLANE, str(error)) from error

        boxes = []
        for object_mask in masks:
            def estimate_box():
                points = deproject_masked_depth(
                    snapshot.depth_m, snapshot.intrinsics, object_mask.mask,
                    self._config.minimum_depth_m, self._config.maximum_depth_m)
                if points.shape[0] < self._config.minimum_object_points:
                    raise InspectionFailure(FailureKind.DEPTH,
                        f'Not enough valid depth for {object_mask.detection.label}')
                return reconstruct_supported_obb(points, plane,
                    self._config.minimum_object_height_m,
                    self._config.minimum_obb_extent_m,
                    self._config.minimum_object_points)
            try:
                boxes.append(geometry_cache.get_or_compute(
                    ('box', support_key, ReconstructionCache.array_key(object_mask.mask)), estimate_box)
                    if geometry_cache is not None else estimate_box())
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
        exclude_detection_boxes: bool = False,
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
        if exclude_detection_boxes:
            for detection in detections:
                x_min = max(0, math.floor(detection.bbox.x_min))
                y_min = max(0, math.floor(detection.bbox.y_min))
                x_max = min(width, math.ceil(detection.bbox.x_max))
                y_max = min(height, math.ceil(detection.bbox.y_max))
                selection[y_min:y_max, x_min:x_max] = False
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
