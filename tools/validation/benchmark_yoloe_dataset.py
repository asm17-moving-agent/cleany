#!/usr/bin/env python3
"""Benchmark the deployed adapter on manifest-indexed RGB images without ROS."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import random
import statistics
import time


def distribution(values: list[float]) -> dict:
    """Summarize samples with linearly interpolated percentiles."""
    if not values:
        raise ValueError('At least one timing sample is required')
    ordered = sorted(values)

    def percentile(q: float) -> float:
        position = (len(ordered) - 1) * q
        lower = int(position)
        upper = min(lower + 1, len(ordered) - 1)
        return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)

    return dict(count=len(values), mean=statistics.fmean(values),
                stdev=statistics.stdev(values) if len(values) > 1 else 0.,
                min=ordered[0], p50=percentile(.5), p95=percentile(.95),
                p99=percentile(.99), max=ordered[-1])


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def strip_engine_metadata(data: bytes) -> tuple[dict, bytes]:
    """Extract an Ultralytics engine's JSON prefix for native trtexec loading."""
    size = int.from_bytes(data[:4], 'little', signed=True)
    if not 0 < size < min(len(data) - 4, 1_000_000):
        raise ValueError('Expected an Ultralytics engine metadata prefix')
    metadata = json.loads(data[4:4 + size].decode())
    if not isinstance(metadata, dict):
        raise ValueError('Engine metadata must be an object')
    return metadata, data[4 + size:]


def main() -> None:
    import numpy as np
    from PIL import Image, ImageDraw
    import torch
    import tensorrt
    import ultralytics
    import yaml
    from cleany_perception.adapters.yoloe_detector import YoloeDetector

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--engine', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--dataset-root', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--passes', type=int, default=10)
    parser.add_argument('--warmup', type=int, default=100)
    parser.add_argument('--seed', type=int, default=20261028)
    parser.add_argument('--skip-artifacts', action='store_true',
                        help='Only record timings; omit gallery and native engine copies')
    args = parser.parse_args()
    if min(args.rounds, args.passes, args.warmup) < 1:
        parser.error('rounds, passes and warmup must be positive')
    args.output.mkdir(parents=True, exist_ok=False)
    parameters = yaml.safe_load(args.profile.read_text())['perception_inspector']['ros__parameters']
    samples = []
    sources = []
    for root in args.dataset_root:
        manifest = root / 'manifest.csv'
        sources.append(dict(root=str(root), manifest_sha256=sha256(manifest)))
        for row in csv.DictReader(manifest.open()):
            path = root / row['rgb']
            with Image.open(path) as image:
                rgb = np.asarray(image.convert('RGB')).copy()
            samples.append(dict(dataset=root.name, id=row['sample_id'], path=str(path),
                                sha256=sha256(path), shape=list(rgb.shape), rgb=rgb,
                                metadata=row))
    if not samples:
        raise ValueError('No input images')
    inventory = [{k: v for k, v in sample.items() if k != 'rgb'} for sample in samples]
    write_json(args.output / 'inputs.json', dict(sources=sources, samples=inventory,
        unique_rgb_sha256=len({sample['sha256'] for sample in samples})))
    adapter = YoloeDetector(str(args.engine), parameters['yoloe_classes'],
        device='cuda:0', image_size=parameters['yoloe_image_size'],
        confidence_threshold=parameters['minimum_detection_confidence'],
        iou_threshold=parameters['yoloe_iou_threshold'],
        maximum_detections=parameters['maximum_detections'], require_masks=True,
        engine_execution='ultralytics')
    began = time.perf_counter()
    adapter.prepare()
    torch.cuda.synchronize()
    load_seconds = time.perf_counter() - began
    environment = dict(engine=str(args.engine.resolve()), engine_sha256=sha256(args.engine),
        engine_size_bytes=args.engine.stat().st_size, profile_sha256=sha256(args.profile),
        packages=dict(torch=torch.__version__, cuda=torch.version.cuda,
                      tensorrt=tensorrt.__version__, ultralytics=ultralytics.__version__, numpy=np.__version__),
        gpu=torch.cuda.get_device_name(), torch_threads=torch.get_num_threads(),
        settings={key: parameters[key] for key in ('yoloe_classes', 'yoloe_image_size',
            'minimum_detection_confidence', 'yoloe_iou_threshold', 'maximum_detections')},
        execution_mode=adapter.execution_mode,
        retina_masks=True, batch=1, rect=False, rounds=args.rounds, passes=args.passes,
        warmup=args.warmup, seed=args.seed, load_seconds=load_seconds,
        started_at=datetime.now(timezone.utc).isoformat())
    write_json(args.output / 'environment.json', environment)
    for index in range(args.warmup):
        adapter.detect(samples[index % len(samples)]['rgb'], '')
    torch.cuda.synchronize()
    rows, blocks = [], []
    fields = ['round', 'pass', 'dataset', 'id', 'latency_ms', 'preprocess_ms',
              'inference_ms', 'postprocess_ms', 'detections']
    with (args.output / 'timings.csv').open('w', newline='', buffering=1) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for round_index in range(args.rounds):
            groups = list(dict.fromkeys(sample['dataset'] for sample in samples))
            if round_index % 2:
                groups.reverse()
            for dataset in groups:
                selected = [sample for sample in samples if sample['dataset'] == dataset]
                for index in range(20):
                    adapter.detect(selected[index % len(selected)]['rgb'], '')
                torch.cuda.synchronize()
                start_epoch = time.time()
                began = time.perf_counter()
                block_rows = []
                for pass_index in range(args.passes):
                    ordered = selected.copy()
                    random.Random(args.seed + round_index * args.passes + pass_index).shuffle(ordered)
                    for sample in ordered:
                        torch.cuda.synchronize()
                        start = time.perf_counter()
                        detections = adapter.detect(sample['rgb'], '')
                        torch.cuda.synchronize()
                        elapsed = (time.perf_counter() - start) * 1000
                        speed = adapter._model.predictor.results[0].speed
                        block_rows.append(dict(round=round_index + 1, **{'pass': pass_index + 1},
                            dataset=dataset, id=sample['id'], latency_ms=elapsed,
                            preprocess_ms=speed['preprocess'], inference_ms=speed['inference'],
                            postprocess_ms=speed['postprocess'], detections=len(detections)))
                elapsed = time.perf_counter() - began
                blocks.append(dict(round=round_index + 1, dataset=dataset,
                    start_epoch=start_epoch, end_epoch=time.time(), wall_seconds=elapsed,
                    requests=len(block_rows), throughput_fps=len(block_rows) / elapsed,
                    latency_ms=distribution([row['latency_ms'] for row in block_rows])))
                writer.writerows(block_rows)
                rows.extend(block_rows)
                write_json(args.output / 'blocks.json', blocks)
                print(json.dumps(blocks[-1]), flush=True)
    summaries = []
    for dataset in ['all', *dict.fromkeys(sample['dataset'] for sample in samples)]:
        selected = [row for row in rows if dataset == 'all' or row['dataset'] == dataset]
        selected_blocks = [block for block in blocks if dataset == 'all' or block['dataset'] == dataset]
        summaries.append(dict(dataset=dataset, requests=len(selected),
            throughput_fps=len(selected) / sum(block['wall_seconds'] for block in selected_blocks),
            latency_ms=distribution([row['latency_ms'] for row in selected]),
            stages_ms={key: distribution([row[key + '_ms'] for row in selected])
                       for key in ('preprocess', 'inference', 'postprocess')},
            over_33_33_ms=sum(row['latency_ms'] > 1000 / 30 for row in selected)))
    write_json(args.output / 'summary.json', dict(summaries=summaries, blocks=blocks,
        failures=0, timed_requests=len(rows), unique_inputs=len(samples),
        timing_boundary='RGB uint8 in RAM -> Ultralytics profiled adapter path -> CPU boxes and full-resolution boolean masks; CUDA synchronized; excludes file decoding, ROS, Gemini, depth and robot actions'))
    if args.skip_artifacts:
        print(json.dumps(dict(status='complete', requests=len(rows), images=len(samples))), flush=True)
        return

    # Rendering and validity checks run after the timed blocks.
    gallery = args.output / 'gallery'
    gallery.mkdir()
    output_rows = []
    colors = [(118, 185, 0), (0, 165, 235), (255, 167, 38), (190, 100, 240)]
    for sample_index, sample in enumerate(samples):
        detections = adapter.detect(sample['rgb'], '')
        rendered = sample['rgb'].copy()
        predictions = []
        for index, detection in enumerate(detections):
            mask = detection.segmentation_mask
            if mask is None or mask.shape != rendered.shape[:2] or mask.dtype != np.bool_:
                raise ValueError(f'Invalid output mask: {sample["id"]}')
            color = np.asarray(colors[index % len(colors)])
            rendered[mask] = (rendered[mask] * .65 + color * .35).astype(np.uint8)
            box = detection.bbox
            predictions.append(dict(label=detection.label, confidence=detection.confidence,
                bbox=[box.x_min, box.y_min, box.x_max, box.y_max], mask_pixels=int(mask.sum())))
        image = Image.fromarray(rendered)
        draw = ImageDraw.Draw(image)
        for index, prediction in enumerate(predictions):
            box = prediction['bbox']
            color = colors[index % len(colors)]
            draw.rectangle(box, outline=color, width=2)
            label = f"{prediction['label']} {prediction['confidence']:.2f}"
            draw.text((box[0], max(0, box[1] - 12)), label, fill='white', stroke_width=1, stroke_fill='black')
        filename = f'{sample_index:03d}.jpg'
        image.save(gallery / filename, quality=90)
        output_rows.append(dict(dataset=sample['dataset'], id=sample['id'],
            image='gallery/' + filename, predictions=predictions))
    write_json(args.output / 'predictions.json', output_rows)

    # This preserves the deployed TensorRT payload exactly; only its wrapper is removed.
    metadata, payload = strip_engine_metadata(args.engine.read_bytes())
    (args.output / 'engine.plan').write_bytes(payload)
    tensor = adapter._model.predictor.preprocess([np.ascontiguousarray(samples[0]['rgb'][..., ::-1])])
    tensor.cpu().numpy().tofile(args.output / 'input.bin')
    write_json(args.output / 'trtexec-input.json', dict(sample=inventory[0],
        dtype=str(tensor.dtype), shape=list(tensor.shape), metadata=metadata,
        payload_sha256=sha256(args.output / 'engine.plan'), input_sha256=sha256(args.output / 'input.bin')))
    print(json.dumps(dict(status='complete', requests=len(rows), images=len(samples))), flush=True)


if __name__ == '__main__':
    main()
