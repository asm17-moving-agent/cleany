from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from cleany_perception.core.models import (
    BoundingBox2D,
    FailureKind,
    InspectionFailure,
    InspectionStage,
    ObjectMask,
    PipelineConfig,
    CameraIntrinsics,
    Detection2D,
    RgbdSnapshot,
    RigidTransform,
)
from cleany_perception.core.pipeline import InspectionPipeline


class _Detector:
    def __init__(self, detections) -> None:
        self.detections = detections
        self.queries = []

    def detect(self, _rgb, query):
        self.queries.append(query)
        return self.detections


class _Segmenter:
    def __init__(self, mask) -> None:
        self.mask = mask
        self.calls = 0
        self.received = []

    def segment(self, _rgb, detections):
        self.calls += 1
        self.received.append(tuple(detections))
        return tuple(
            ObjectMask(detection=item, mask=self.mask, score=0.99)
            for item in detections
        )


class _Transformer:
    def __init__(self, transform) -> None:
        self.transform = transform
        self.calls = []

    def lookup(self, target_frame, source_frame, stamp_ns):
        self.calls.append((target_frame, source_frame, stamp_ns))
        return self.transform


class _FailingTransformer:
    def lookup(self, _target_frame, _source_frame, _stamp_ns):
        raise RuntimeError('transform unavailable')


def _pipeline(
    synthetic_scene,
    detector=None,
    segmenter=None,
    transformer=None,
):
    return InspectionPipeline(
        detector=detector or _Detector([synthetic_scene['detection']]),
        segmenter=segmenter or _Segmenter(synthetic_scene['mask']),
        transformer=transformer or _Transformer(synthetic_scene['transform']),
        target_frame='base_link',
        config=PipelineConfig(
            plane_distance_threshold_m=0.001,
            plane_minimum_inliers=100,
            plane_minimum_inlier_ratio=0.9,
        ),
    )


def test_pipeline_reports_stages_and_reconstructs_object(synthetic_scene):
    stages = []
    transformer = _Transformer(synthetic_scene['transform'])
    pipeline = _pipeline(synthetic_scene, transformer=transformer)

    output = pipeline.inspect(
        synthetic_scene['snapshot'],
        'find box',
        progress=lambda stage, _detections, _objects, _message: stages.append(
            stage
        ),
    )

    assert stages == [
        InspectionStage.DETECTING,
        InspectionStage.SEGMENTING,
        InspectionStage.RECONSTRUCTING,
        InspectionStage.TRANSFORMING,
    ]
    assert len(output.objects) == 1
    assert output.objects[0].label == 'box'
    assert np.linalg.norm(
        output.objects[0].box.center - synthetic_scene['expected_center']
    ) <= 0.005
    assert transformer.calls == [
        ('base_link', 'camera_optical_frame', 1_500_000_000)
    ]


def test_pipeline_returns_successful_empty_output_without_tf(synthetic_scene):
    transformer = _Transformer(synthetic_scene['transform'])
    segmenter = _Segmenter(synthetic_scene['mask'])
    pipeline = _pipeline(
        synthetic_scene,
        detector=_Detector([]),
        segmenter=segmenter,
        transformer=transformer,
    )

    output = pipeline.inspect(synthetic_scene['snapshot'], 'nothing')

    assert output.objects == ()
    assert segmenter.calls == 0
    assert transformer.calls == []


@pytest.mark.parametrize('has_mask', [False, True])
@pytest.mark.parametrize('short_width_m', [.040, .140])
def test_footprint_of_diagonal_object_does_not_use_image_bbox(
        synthetic_scene, has_mask, short_width_m):
    intrinsics = CameraIntrinsics(320, 240, 500., 500., 159.5, 119.5)
    rows, columns = np.indices((240, 320))
    x = (columns-intrinsics.cx)*.6/intrinsics.fx
    y = (rows-intrinsics.cy)*.6/intrinsics.fy
    long_axis = (x+y)/np.sqrt(2.)
    short_axis = (x-y)/np.sqrt(2.)
    mask = (np.abs(long_axis) <= .080) & (np.abs(short_axis) <= short_width_m/2)
    depth = np.full(mask.shape, .7, dtype=np.float32)
    depth[mask] = .6
    snapshot = RgbdSnapshot(np.zeros((*mask.shape, 3), dtype=np.uint8),
        depth, intrinsics, 1, 'camera_optical_frame')
    ys, xs = np.nonzero(mask)
    detection = Detection2D('box', .9,
        BoundingBox2D(float(xs.min()), float(ys.min()),
                      float(xs.max()+1), float(ys.max()+1)),
        segmentation_mask=mask if has_mask else None)
    image_bbox_smaller = min(xs.max()-xs.min()+1, ys.max()-ys.min()+1)*.6/500.
    assert image_bbox_smaller > .11
    segmenter = _Segmenter(mask)
    pipeline = _pipeline(synthetic_scene, segmenter=segmenter)
    transform = RigidTransform(np.array((0., 0., .7)), np.diag((1., -1., -1.)))
    measured, = pipeline.estimate_detection_footprints(snapshot, (detection,), transform)
    assert sorted((measured.estimated_width_m, measured.estimated_height_m)) == pytest.approx(
        [short_width_m, .160], abs=.002)
    assert (min(measured.estimated_width_m, measured.estimated_height_m) <= .11
            ) == (short_width_m <= .11)
    assert segmenter.calls == 0
    assert measured.bbox == detection.bbox


@pytest.mark.parametrize('failure', ['depth', 'plane', 'tilt'])
def test_unmeasurable_footprint_does_not_substitute_image_bbox(synthetic_scene, failure):
    snapshot = synthetic_scene['snapshot']
    detection = replace(synthetic_scene['detection'],
        segmentation_mask=synthetic_scene['mask'],
        estimated_width_m=.040, estimated_height_m=.160)
    transform = synthetic_scene['transform']
    if failure == 'depth':
        snapshot.depth_m[synthetic_scene['mask']] = np.nan
    elif failure == 'plane':
        snapshot.depth_m[~synthetic_scene['mask']] = np.nan
    else:
        transform = RigidTransform(np.zeros(3), np.eye(3))
    pipeline = _pipeline(synthetic_scene)
    measured, = pipeline.estimate_detection_footprints(snapshot, (detection,), transform)
    assert measured.estimated_width_m == 0. and measured.estimated_height_m == 0.


def test_bad_footprint_does_not_hide_another_object_and_cancel_remains_effective(synthetic_scene):
    good = replace(synthetic_scene['detection'], segmentation_mask=synthetic_scene['mask'])
    bad = replace(good, label='bad', segmentation_mask=np.zeros_like(synthetic_scene['mask']))
    pipeline = _pipeline(synthetic_scene)
    measured = pipeline.estimate_detection_footprints(synthetic_scene['snapshot'],
        (bad, good), synthetic_scene['transform'])
    assert measured[0].estimated_width_m == 0.
    assert measured[1].estimated_width_m > 0.
    with pytest.raises(InspectionFailure) as failed:
        pipeline.estimate_detection_footprints(synthetic_scene['snapshot'],
            (good,), synthetic_scene['transform'], cancelled=lambda: True)
    assert failed.value.kind == FailureKind.CANCELLED


def test_pipeline_maps_invalid_mask_to_mask_failure(synthetic_scene):
    empty_mask = np.zeros_like(synthetic_scene['mask'])
    pipeline = _pipeline(
        synthetic_scene,
        segmenter=_Segmenter(empty_mask),
    )

    with pytest.raises(InspectionFailure) as raised:
        pipeline.inspect(synthetic_scene['snapshot'], 'find box')

    assert raised.value.kind == FailureKind.MASK


def test_pipeline_maps_missing_object_depth_to_depth_failure(synthetic_scene):
    snapshot = synthetic_scene['snapshot']
    snapshot.depth_m[synthetic_scene['mask']] = np.nan
    pipeline = _pipeline(synthetic_scene)

    with pytest.raises(InspectionFailure) as raised:
        pipeline.inspect(snapshot, 'find box')

    assert raised.value.kind == FailureKind.DEPTH


def test_pipeline_rejects_support_plane_tilt_in_target_frame(synthetic_scene):
    identity_transform = type(synthetic_scene['transform'])(
        translation=np.zeros(3),
        rotation=np.eye(3),
    )
    pipeline = _pipeline(
        synthetic_scene,
        transformer=_Transformer(identity_transform),
    )

    with pytest.raises(InspectionFailure) as raised:
        pipeline.inspect(synthetic_scene['snapshot'], 'find box')

    assert raised.value.kind == FailureKind.PLANE


def test_pipeline_checks_cancel_before_detector(synthetic_scene):
    detector = _Detector([synthetic_scene['detection']])
    pipeline = _pipeline(synthetic_scene, detector=detector)

    with pytest.raises(InspectionFailure) as raised:
        pipeline.inspect(
            synthetic_scene['snapshot'],
            'find box',
            cancelled=lambda: True,
        )

    assert raised.value.kind == FailureKind.CANCELLED
    assert detector.queries == []


def test_pipeline_maps_transform_exception_to_tf_failure(synthetic_scene):
    pipeline = _pipeline(
        synthetic_scene,
        transformer=_FailingTransformer(),
    )

    with pytest.raises(InspectionFailure) as raised:
        pipeline.inspect(synthetic_scene['snapshot'], 'find box')

    assert raised.value.kind == FailureKind.TF


def test_pipeline_uses_transform_captured_before_inference(synthetic_scene):
    pipeline = _pipeline(
        synthetic_scene,
        transformer=_FailingTransformer(),
    )

    output = pipeline.inspect(
        synthetic_scene['snapshot'],
        'find box',
        capture_transform=synthetic_scene['transform'],
    )

    assert len(output.objects) == 1
    assert output.objects[0].label == 'box'


def test_detect_only_does_not_run_segmentation_or_tf(synthetic_scene):
    segmenter = _Segmenter(synthetic_scene['mask'])
    transformer = _Transformer(synthetic_scene['transform'])
    pipeline = _pipeline(
        synthetic_scene,
        segmenter=segmenter,
        transformer=transformer,
    )
    stages = []

    detections = pipeline.detect(
        synthetic_scene['snapshot'],
        'find box',
        progress=lambda stage, _detections, _objects, _message: stages.append(
            stage
        ),
    )

    assert detections == (synthetic_scene['detection'],)
    assert stages == [InspectionStage.DETECTING]
    assert segmenter.calls == 0
    assert transformer.calls == []


def test_inspect_selected_segments_and_reconstructs_only_selected(
    synthetic_scene,
):
    other = type(synthetic_scene['detection'])(
        label='can',
        confidence=0.99,
        bbox=synthetic_scene['detection'].bbox,
    )
    segmenter = _Segmenter(synthetic_scene['mask'])
    transformer = _FailingTransformer()
    pipeline = _pipeline(
        synthetic_scene,
        segmenter=segmenter,
        transformer=transformer,
    )
    stages = []

    output = pipeline.inspect_selected(
        synthetic_scene['snapshot'],
        (other, synthetic_scene['detection']),
        synthetic_scene['detection'],
        synthetic_scene['transform'],
        progress=lambda stage, _detections, _objects, _message: stages.append(
            stage
        ),
    )

    assert segmenter.received == [(synthetic_scene['detection'],)]
    assert [item.label for item in output.objects] == ['box']
    assert stages == [
        InspectionStage.SEGMENTING,
        InspectionStage.RECONSTRUCTING,
        InspectionStage.TRANSFORMING,
    ]


def test_selected_inspection_excludes_every_detection_box_from_plane(
    synthetic_scene,
):
    other = type(synthetic_scene['detection'])(
        label='can',
        confidence=0.99,
        bbox=BoundingBox2D(
            x_min=210.0,
            y_min=100.0,
            x_max=250.0,
            y_max=140.0,
        ),
    )
    selected = synthetic_scene['detection']
    selected_mask = ObjectMask(
        detection=selected,
        mask=synthetic_scene['mask'],
        score=0.99,
    )
    pipeline = _pipeline(synthetic_scene)

    support = pipeline._support_selection(
        synthetic_scene['snapshot'].depth_m.shape,
        (selected, other),
        (selected_mask,),
        exclude_detection_boxes=True,
    )

    assert not support[100:140, 135:185].any()
    assert not support[100:140, 210:250].any()
    assert support[95:100, 135:185].all()


def test_detect_only_returns_empty_without_downstream_work(synthetic_scene):
    segmenter = _Segmenter(synthetic_scene['mask'])
    transformer = _Transformer(synthetic_scene['transform'])
    pipeline = _pipeline(
        synthetic_scene,
        detector=_Detector([]),
        segmenter=segmenter,
        transformer=transformer,
    )

    assert pipeline.detect(synthetic_scene['snapshot'], 'nothing') == ()
    assert segmenter.calls == 0
    assert transformer.calls == []
