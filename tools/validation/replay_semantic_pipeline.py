#!/usr/bin/env python3
"""Replay saved RGB-D through production detection, semantics and wire contracts.

Detection runs in the vision container. Classification runs with the installed
native ROS overlay and a Gemini key in its environment. No ROS node is created.
Camera-frame geometry and a synthetic destination policy do not validate robot
TF, wrist observations, motion planning, or physical execution.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, fields
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

import numpy as np


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def detect(args: argparse.Namespace) -> None:
    from PIL import Image
    from cleany_perception.adapters.yoloe_detector import YoloeDetector
    from cleany_perception.core.models import (
        CameraIntrinsics, InspectionFailure, ObjectMask, PipelineConfig, RgbdSnapshot,
    )
    from cleany_perception.core.pipeline import InspectionPipeline

    args.output.mkdir(parents=True, exist_ok=False)
    parameters = json.loads((args.capture / 'parameters.json').read_text())
    metadata = json.loads((args.capture / 'capture.json').read_text())
    config = PipelineConfig(**{f.name: parameters[f.name] for f in fields(PipelineConfig)})
    model = args.model_path or Path(parameters['yoloe_model_path'])
    detector = YoloeDetector(str(model), parameters['yoloe_classes'], device='cuda',
        image_size=parameters['yoloe_image_size'], confidence_threshold=config.minimum_detection_confidence,
        iou_threshold=parameters['yoloe_iou_threshold'], maximum_detections=config.maximum_detections,
        text_encoder_directory=parameters['yoloe_text_encoder_directory'], require_masks=True,
        class_confidence_thresholds=parameters['yoloe_class_confidence_thresholds'] or ())
    detector.prepare()
    pipeline = InspectionPipeline(detector, None, None, metadata['source_frame'], config)
    intrinsics = CameraIntrinsics(**metadata['intrinsics'])
    frames = []
    with np.load(args.capture / 'captures.npz', allow_pickle=False) as archive:
        for index in args.indices:
            snapshot = RgbdSnapshot(archive['rgb'][index], archive['depth'][index], intrinsics,
                int(archive['stamps'][index]), metadata['source_frame'])
            began = time.monotonic()
            detections = pipeline.detect(snapshot, '')
            elapsed = time.monotonic() - began
            if not detections:
                raise RuntimeError(f'No detected object in replay frame {index}')
            rows, masks = [], []
            for detection in detections:
                mask = detection.segmentation_mask
                assert mask is not None and mask.any()
                row = dict(label=detection.label, detector_label=detection.detector_label,
                    confidence=float(detection.confidence), bbox=asdict(detection.bbox))
                try:
                    _, boxes = pipeline._reconstruct(snapshot, detections,
                        (ObjectMask(detection, mask, detection.confidence),), True)
                    row['geometry'] = dict(center_m=boxes[0].center.tolist(), size_m=boxes[0].size.tolist())
                except InspectionFailure as error:
                    row['geometry_error'] = dict(kind=error.kind.value, message=str(error))
                valid = snapshot.depth_m[mask]
                valid = valid[np.isfinite(valid) & (valid >= config.minimum_depth_m)
                              & (valid <= config.maximum_depth_m)]
                row['median_depth_m'] = float(np.median(valid)) if valid.size else 0.0
                rows.append(row)
                masks.append(mask)
            np.savez_compressed(args.output / f'frame-{index}.npz', rgb=snapshot.rgb,
                                masks=np.stack(masks))
            Image.fromarray(snapshot.rgb).save(args.output / f'frame-{index}.png')
            frames.append(dict(index=index, stamp_ns=snapshot.stamp_ns,
                source_frame=snapshot.source_frame, detector_seconds=elapsed, detections=rows))
            print(json.dumps({'frame': index, 'detections': len(rows), 'detector_seconds': elapsed}), flush=True)
    write_json(args.output / 'detections.json', dict(
        scope='Saved physical RGB-D; production YOLOE and camera-frame reconstruction; no robot TF',
        capture_sha256=sha256(args.capture / 'captures.npz'), model_sha256=sha256(model),
        model_path=str(model), frames=frames))


class RecordingModels:
    """Record returned token counts without retaining client configuration."""

    def __init__(self, models: object) -> None:
        self.models = models
        self.calls: list[dict] = []

    def generate_content(self, **kwargs):
        began = time.monotonic()
        result = self.models.generate_content(**kwargs)
        usage = result.usage_metadata
        self.calls.append(dict(seconds=time.monotonic() - began,
            output_tokens=usage.candidates_token_count,
            thinking_tokens=usage.thoughts_token_count,
            response=json.loads(result.text)))
        return result


def classify(args: argparse.Namespace) -> None:
    from types import SimpleNamespace
    from google import genai
    from google.genai import types
    from rclpy.serialization import deserialize_message, serialize_message
    from cleany_interfaces.msg import DetectedObject2D
    from cleany_perception.adapters.gemini_detector import GeminiClassifier, parse_gemini_classifications
    from cleany_perception.core.models import BoundingBox2D, Detection2D
    from cleany_perception.core.tracking import ObjectTracker
    from cleany_perception.inspection_node import InspectionNode
    from cleany_skill_executor.core.automatic_goal import select_automatic_target

    assert 'sorting_reason' not in DetectedObject2D.get_fields_and_field_types()
    spec = importlib.util.spec_from_file_location('baseline_classifier', args.baseline)
    baseline = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(baseline)
    # Keep the prior prompt/schema and API call; adapt its parser to the new
    # local dataclass so the removed explanation is not required as an attribute.
    baseline.parse_gemini_classifications = parse_gemini_classifications
    client = genai.Client(api_key=os.environ['GEMINI_API_KEY'],
                         http_options=types.HttpOptions(timeout=30000))
    models = RecordingModels(client.models)
    proxy = SimpleNamespace(models=models)
    classifiers = {'with_reason': baseline.GeminiClassifier(args.model,
        client_factory=lambda *_: proxy), 'without_reason': GeminiClassifier(args.model,
        client_factory=lambda *_: proxy)}
    data = json.loads((args.input / 'detections.json').read_text())
    progress = args.input / 'classification-progress.json'
    rows = json.loads(progress.read_text()) if args.resume and progress.is_file() else []
    completed = {(r['repeat'], r['frame'], r['mode']) for r in rows}
    last_request = 0.0
    try:
        for repeat in range(args.repeats):
            for frame in data['frames']:
                index = frame['index']
                with np.load(args.input / f'frame-{index}.npz', allow_pickle=False) as archive:
                    rgb, masks = archive['rgb'], archive['masks']
                raw, positions = [], []
                for item, mask in zip(frame['detections'], masks):
                    geometry = item.get('geometry')
                    raw.append(Detection2D(item['label'], item['confidence'], BoundingBox2D(**item['bbox']),
                        detector_label=item['detector_label'], segmentation_mask=mask,
                        median_depth_m=item['median_depth_m'],
                        estimated_width_m=min(geometry['size_m'][:2]) if geometry else 0.0,
                        estimated_height_m=max(geometry['size_m'][:2]) if geometry else 0.0))
                    positions.append(tuple(geometry['center_m']) if geometry else None)
                tracker = ObjectTracker()
                order = ['with_reason', 'without_reason']
                if (repeat + index) % 2:
                    order.reverse()
                for mode in order:
                    if (repeat, index, mode) in completed:
                        continue
                    time.sleep(max(0.0, args.interval_seconds - (time.monotonic() - last_request)))
                    last_request = time.monotonic()
                    start = len(models.calls)
                    began = time.monotonic()
                    classified = classifiers[mode].classify(rgb, raw)
                    elapsed = time.monotonic() - began
                    calls = models.calls[start:]
                    if mode == 'without_reason':
                        assert all(set(item) == {'id', 'label', 'sorting_category'}
                            for item in calls[-1]['response']['objects'])
                    for original, final in zip(raw, classified):
                        assert final.bbox is original.bbox and final.segmentation_mask is original.segmentation_mask
                        assert final.confidence == original.confidence and final.detector_label == original.detector_label
                    tracking = tracker.update('replay', mode, [d.label for d in classified], positions,
                        detector_labels=[d.detector_label for d in classified])
                    message = InspectionNode._detections_message(classified, frame['stamp_ns'],
                        frame['source_frame'], f'{repeat}-{index}-{mode}',
                        [float(np.linalg.norm(p)) if p else None for p in positions], tracking,
                        representative_frame=frame['source_frame'])
                    decoded = deserialize_message(serialize_message(message), type(message))
                    selected = None
                    try:
                        selected = asdict(select_automatic_target(decoded,
                            {'label_destinations': {'*': 'replay_only'}}))
                    except ValueError:
                        pass
                    rows.append(dict(repeat=repeat, frame=index, mode=mode, seconds=elapsed,
                        calls=calls, objects=[dict(label=d.label, detector_label=d.detector_label,
                            category=d.sorting_category) for d in classified],
                        selected=selected, wire_roundtrip=True, geometry_preserved=True))
                    write_json(progress, rows)
                    print(json.dumps({k: rows[-1][k] for k in ('repeat','frame','mode','seconds','objects')}), flush=True)
    finally:
        client.close()
    latency = {}
    for mode in classifiers:
        values = [r['seconds'] for r in rows if r['mode'] == mode]
        latency[mode] = dict(count=len(values), p50_seconds=float(np.median(values)),
            p95_seconds=float(np.percentile(values, 95)),
            output_tokens=[sum(c['output_tokens'] or 0 for c in r['calls']) for r in rows if r['mode'] == mode])
    agreements = []
    for repeat in range(args.repeats):
        for frame in data['frames']:
            pair = [r for r in rows if r['repeat'] == repeat and r['frame'] == frame['index']]
            agreements.append(pair[0]['objects'] == pair[1]['objects'])
    write_json(args.input / 'classification-report.json', dict(model=args.model, rows=rows,
        baseline_sha256=sha256(args.baseline), latency=latency, exact_semantic_agreement=agreements,
        limitations=['Saved frames from one scene; no classification accuracy benchmark.',
            'Camera-frame distances and synthetic destination only; no robot TF or real approval.',
            'No wrist camera, motion planning, or physical actuation.']))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='stage', required=True)
    local = sub.add_parser('detect')
    local.add_argument('--capture', type=Path, required=True)
    local.add_argument('--output', type=Path, required=True)
    local.add_argument('--model-path', type=Path)
    local.add_argument('--indices', type=lambda s: [int(v) for v in s.split(',')], default=[0, 10, 19])
    api = sub.add_parser('classify')
    api.add_argument('--input', type=Path, required=True)
    api.add_argument('--baseline', type=Path, required=True)
    api.add_argument('--model', required=True)
    api.add_argument('--repeats', type=int, default=2)
    api.add_argument('--interval-seconds', type=float, default=10.0)
    api.add_argument('--resume', action='store_true', help='Keep successful requests in the same replay directory')
    args = parser.parse_args()
    if args.stage == 'detect':
        detect(args)
    else:
        if args.repeats < 1 or args.interval_seconds < 0:
            parser.error('--repeats must be positive and --interval-seconds nonnegative')
        classify(args)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        message = str(error)
        key = os.environ.get('GEMINI_API_KEY')
        if key:
            message = message.replace(key, '[redacted]')
        print(f'{type(error).__name__}: {message}', flush=True)
        raise SystemExit(1) from None
