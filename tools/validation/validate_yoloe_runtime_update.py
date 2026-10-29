#!/usr/bin/env python3
"""Verify deployed YOLOE outputs, concurrency, fallback and M latency on Jetson."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import random
import statistics
import sys
import time


def fingerprint(detections) -> str:
    h = hashlib.sha256()
    for item in detections:
        h.update(repr((item.label, item.detector_label, item.confidence, item.bbox)).encode())
        h.update(item.segmentation_mask.tobytes())
        assert not item.segmentation_mask.flags.writeable
    return h.hexdigest()


def main() -> None:
    import numpy as np
    from PIL import Image
    import torch
    import yaml
    from cleany_perception.adapters.yoloe_detector import YoloeDetector, YoloeModelPool

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-adapter', type=Path, required=True)
    parser.add_argument('--benchmark', type=Path, required=True)
    parser.add_argument('--model-root', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ros', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location('baseline_yoloe', args.baseline_adapter)
    baseline = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = baseline
    spec.loader.exec_module(baseline)
    settings = yaml.safe_load(args.profile.read_text())['perception_inspector']['ros__parameters']
    inventory = json.loads((args.benchmark / 'round-1/m/adapter/inputs.json').read_text())['samples']
    samples = []
    for item in inventory:
        path = Path(item['path'])
        assert hashlib.sha256(path.read_bytes()).hexdigest() == item['sha256']
        with Image.open(path) as im:
            samples.append((item, np.asarray(im.convert('RGB')).copy()))
    common = dict(classes=settings['yoloe_classes'], device='cuda:0',
        image_size=settings['yoloe_image_size'], confidence_threshold=.25,
        iou_threshold=.5, maximum_detections=10, require_masks=True)
    result = dict(models={}, latency={}, passed=False,
        baseline_sha256=hashlib.sha256(args.baseline_adapter.read_bytes()).hexdigest())
    timings = []
    for size in ('n', 's', 'm'):
        engine = args.model_root / 'real32' / size / 'active' / f'yoloe-26{size}-seg-real32.engine'
        old = baseline.YoloeDetector(str(engine), **common)
        pool = YoloeModelPool()
        new = YoloeDetector(str(engine), **common, model_pool=pool)
        old.prepare()
        new.prepare()
        assert new.execution_mode == 'async', f'{size}: unexpected fallback'
        reference = []
        for _, rgb in samples:
            expected = fingerprint(old.detect(rgb, ''))
            assert fingerprint(new.detect(rgb, '')) == expected
            reference.append(expected)
        result['models'][size] = dict(images=len(samples), all_exact=True,
            engine_sha256=hashlib.sha256(engine.read_bytes()).hexdigest(), execution_mode=new.execution_mode)
        if size != 'm':
            del old, new, pool
            torch.cuda.empty_cache()
            continue
        with (args.output / 'timings.csv').open('w', newline='', buffering=1) as stream:
            writer = csv.DictWriter(stream, fieldnames=['round', 'mode', 'dataset', 'id', 'latency_ms'])
            writer.writeheader()
            for rnd in range(4):
                order = [('baseline', old), ('async', new)]
                if rnd % 2:
                    order.reverse()
                indices = list(range(len(samples)))
                random.Random(20261029 + rnd).shuffle(indices)
                for mode, adapter in order:
                    for index in indices[:20]:
                        adapter.detect(samples[index][1], '')
                    values = []
                    for index in indices:
                        item, rgb = samples[index]
                        torch.cuda.synchronize()
                        start = time.perf_counter()
                        detections = adapter.detect(rgb, '')
                        torch.cuda.synchronize()
                        elapsed = (time.perf_counter() - start) * 1000
                        assert fingerprint(detections) == reference[index]
                        row = dict(round=rnd + 1, mode=mode, dataset=item['dataset'],
                                   id=item['id'], latency_ms=elapsed)
                        writer.writerow(row)
                        timings.append(row)
                        values.append(elapsed)
                    print(json.dumps(dict(round=rnd + 1, mode=mode, mean_ms=statistics.mean(values))), flush=True)
        assert new.execution_mode == 'async'
        peer_options = dict(common, confidence_threshold=.4, maximum_detections=4)
        peer = YoloeDetector(str(engine), **peer_options, model_pool=pool)
        old_peer = baseline.YoloeDetector(str(engine), **peer_options)
        peer.prepare()
        assert peer._model is new._model and peer._execution is new._execution
        peer_reference = [fingerprint(old_peer.detect(rgb, '')) for _, rgb in samples]
        held = new.detect(samples[1][1], '')
        held_hash = fingerprint(held)
        def infer(i):
            index = (i * 17) % len(samples)
            adapter, expected = (peer, peer_reference) if i % 2 else (new, reference)
            return fingerprint(adapter.detect(samples[index][1], '')) == expected[index]
        with ThreadPoolExecutor(max_workers=3) as executor:
            assert all(executor.map(infer, range(96)))
        assert fingerprint(held) == held_hash
        result['concurrency'] = dict(calls=96, workers=3, per_call_thresholds=True,
                                     held_results_unchanged=True, all_exact=True)

        # Refuse an enqueue without submitting GPU work. The healthy stream
        # drains and all wrappers must share the permanent legacy fallback.
        context = new._execution.runner._context
        class RefusedContext:
            def __getattr__(self, name):
                return getattr(context, name)

            def execute_async_v3(self, stream):
                return False
        new._execution.runner._context = RefusedContext()
        for index, (_, rgb) in enumerate(samples):
            assert fingerprint(new.detect(rgb, '')) == reference[index]
        assert new.execution_mode == peer.execution_mode == 'ultralytics'
        result['fallback'] = dict(injected_enqueue_refusal=True, images=len(samples),
                                  all_exact=True, shared_permanent_fallback=True)
        if args.ros:
            import rclpy
            from rclpy.parameter import Parameter
            from cleany_perception.inspection_node import InspectionNode
            os.environ['CLEANY_VALIDATION_GEMINI_KEY'] = 'offline-validation-no-api-calls'
            overrides = dict(settings, detector_type='yoloe_gemini', segmenter_type='yoloe_seg',
                yoloe_model_path=str(engine), yoloe_device='cuda:0', yoloe_engine_execution='async',
                gemini_api_key_environment='CLEANY_VALIDATION_GEMINI_KEY',
                enable_reference_observation=True, enable_wrist_observation=True)
            rclpy.init(args=[])
            node = None
            try:
                node = InspectionNode(transformer=object(), parameter_overrides=[
                    Parameter(k, value=v) for k, v in overrides.items()])
                wrapped = node._wrist_service.detector
                assert wrapped._adapter.execution_mode == 'async'
                assert wrapped._adapter is node._pipeline._detector._adapter._yoloe
                for index, (_, rgb) in enumerate(samples):
                    assert fingerprint(wrapped.detect(rgb, '')) == reference[index]
                result['ros'] = dict(images=len(samples), all_exact=True,
                    shared_head_wrist_adapter=True, execution_mode=wrapped._adapter.execution_mode,
                    scope='Real ROS node initialization and serialized detector; no camera action or Gemini')
            finally:
                if node is not None:
                    node.destroy_node()
                rclpy.shutdown()
    for mode in ('baseline', 'async'):
        values = [r['latency_ms'] for r in timings if r['mode'] == mode]
        result['latency'][mode] = dict(calls=len(values), mean_ms=statistics.mean(values),
            p50_ms=statistics.median(values), p95_ms=statistics.quantiles(values, n=100, method='inclusive')[94])
    result['passed'] = True
    (args.output / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
