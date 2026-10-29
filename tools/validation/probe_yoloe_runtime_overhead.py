#!/usr/bin/env python3
"""Isolate adapter-copy and profiling overhead in a disposable GPU process."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import inspect
import json
from pathlib import Path
import random
import statistics
import textwrap
import time


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> None:
    import numpy as np
    from PIL import Image
    import torch
    import ultralytics
    from ultralytics.utils import ops
    import yaml
    from cleany_perception.adapters import yoloe_detector as module

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path, required=True)
    parser.add_argument('--engine', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    source_dir = args.output / 'source'
    source_dir.mkdir()
    source = inspect.getsource(module)
    (source_dir / 'yoloe_detector.py').write_text(source)
    package = Path(ultralytics.__file__).parent
    for filename in ('engine/predictor.py', 'utils/ops.py',
                     'nn/backends/tensorrt.py', 'models/yolo/segment/predict.py'):
        (source_dir / filename.replace('/', '_')).write_text((package / filename).read_text())
    (source_dir / Path(__file__).name).write_text(Path(__file__).read_text())

    # Compile a separate method; never change the deployed module or its files.
    original_method = module.YoloeDetector.detect
    candidate = textwrap.dedent(inspect.getsource(original_method))
    old = '''        for index, box in enumerate(boxes):
            coordinates = _numpy(box.xyxy).reshape(-1, 4)[0]
            class_id = int(_numpy(box.cls).reshape(-1)[0])
            confidence = float(_numpy(box.conf).reshape(-1)[0])'''
    new = '''        box_data = _numpy(boxes.data)
        for index, box_row in enumerate(box_data):
            coordinates = box_row[:4]
            class_id = int(box_row[-1])
            confidence = float(box_row[-2])'''
    assert candidate.count(old) == 1, 'Adapter changed; inspect the experiment before rerunning'
    candidate = candidate.replace(old, new)
    (source_dir / 'candidate_detect.py').write_text(candidate)
    namespace = vars(module).copy()
    exec(compile(candidate, '<probe-only-bulk-boxes>', 'exec'), namespace)
    bulk_method = namespace['detect']
    profile_time = ops.Profile.time

    parameters = yaml.safe_load(args.profile.read_text())['perception_inspector']['ros__parameters']
    inventory = json.loads((args.benchmark / 'round-1/m/adapter/inputs.json').read_text())['samples']
    samples = []
    for item in inventory:
        path = Path(item['path'])
        assert digest(path.read_bytes()) == item['sha256']
        with Image.open(path) as image:
            samples.append((item, np.asarray(image.convert('RGB')).copy()))
    adapter = module.YoloeDetector(str(args.engine), parameters['yoloe_classes'],
        device='cuda:0', image_size=parameters['yoloe_image_size'],
        confidence_threshold=parameters['minimum_detection_confidence'],
        iou_threshold=parameters['yoloe_iou_threshold'],
        maximum_detections=parameters['maximum_detections'], require_masks=True)
    adapter.prepare()
    backend = adapter._model.predictor.model
    metadata = dict(engine=str(args.engine.resolve()), engine_sha256=digest(args.engine.read_bytes()),
        adapter_sha256=digest(source.encode()), packages=dict(ultralytics=ultralytics.__version__,
        torch=torch.__version__, cuda=torch.version.cuda), source_images=len(samples),
        engine_end2end=backend.end2end, engine_fp16_io=backend.fp16,
        changed_production_files=False, gpu_clocks_changed=False,
        timing='CUDA synchronize before and after complete detect; comparison excluded',
        scope='M only; four balanced rounds, same resident model; exploratory, not ROS')
    (args.output / 'environment.json').write_text(json.dumps(metadata, indent=2) + '\n')

    def fingerprint(detections) -> str:
        h = hashlib.sha256()
        for detection in detections:
            h.update(repr((detection.label, detection.detector_label,
                           detection.confidence, detection.bbox)).encode())
            mask = detection.segmentation_mask
            assert mask.dtype == np.bool_ and not mask.flags.writeable
            h.update(mask.tobytes())
        return h.hexdigest()

    reference = {}
    for item, rgb in samples:
        reference[(item['dataset'], item['id'])] = fingerprint(adapter.detect(rgb, ''))
    variants = ('baseline', 'bulk_boxes', 'no_profile_sync', 'combined')
    orders = [variants[i:] + variants[:i] for i in range(len(variants))]
    rows = []
    clock = next(Path('/sys/class/devfreq').glob('*gpu*/cur_freq'), None)
    try:
        with (args.output / 'timings.csv').open('w', newline='', buffering=1) as stream:
            writer = csv.DictWriter(stream, fieldnames=['round', 'variant', 'dataset', 'id',
                'latency_ms', 'detections', 'gpu_clock_hz', 'exact_output_match'])
            writer.writeheader()
            for round_index, order in enumerate(orders, 1):
                ordered = samples.copy()
                random.Random(20261028 + round_index).shuffle(ordered)
                for variant in order:
                    method = bulk_method if variant in ('bulk_boxes', 'combined') else original_method
                    ops.Profile.time = (lambda self: time.perf_counter()) if variant in (
                        'no_profile_sync', 'combined') else profile_time
                    for _, rgb in ordered[:20]:
                        method(adapter, rgb, '')
                    for item, rgb in ordered:
                        torch.cuda.synchronize()
                        start = time.perf_counter()
                        detected = method(adapter, rgb, '')
                        torch.cuda.synchronize()
                        elapsed = (time.perf_counter() - start) * 1000
                        match = fingerprint(detected) == reference[(item['dataset'], item['id'])]
                        row = dict(round=round_index, variant=variant, dataset=item['dataset'],
                            id=item['id'], latency_ms=elapsed, detections=len(detected),
                            gpu_clock_hz=int(clock.read_text()) if clock else None,
                            exact_output_match=match)
                        writer.writerow(row)
                        rows.append(row)
                        if not match:
                            raise ValueError(f'Output changed: {variant}/{item["id"]}')
                    values = [r['latency_ms'] for r in rows if r['round'] == round_index and r['variant'] == variant]
                    print(json.dumps(dict(round=round_index, variant=variant,
                                          mean_ms=statistics.mean(values))), flush=True)
    finally:
        ops.Profile.time = profile_time
    result = dict(environment=metadata, orders=orders, variants={}, passed=True)
    for variant in variants:
        selected = [r for r in rows if r['variant'] == variant]
        values = [r['latency_ms'] for r in selected]
        result['variants'][variant] = dict(calls=len(selected), mean_ms=statistics.mean(values),
            p50_ms=statistics.median(values), p95_ms=statistics.quantiles(values, n=100, method='inclusive')[94],
            exact_output_matches=sum(r['exact_output_match'] for r in selected),
            gpu_clock_samples=dict(Counter(r['gpu_clock_hz'] for r in selected)),
            round_means_ms=[statistics.mean(r['latency_ms'] for r in selected if r['round'] == i)
                            for i in range(1, 5)])
    (args.output / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result['variants'], indent=2), flush=True)


if __name__ == '__main__':
    main()
