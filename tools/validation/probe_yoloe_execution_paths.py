#!/usr/bin/env python3
"""Compare isolated YOLOE execution paths while preserving production files."""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from contextlib import nullcontext
import csv
import hashlib
import inspect
import json
from pathlib import Path
import random
import statistics
import textwrap
import time


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def distribution(values: list[float]) -> dict:
    return dict(mean=statistics.mean(values), p50=statistics.median(values),
                p95=statistics.quantiles(values, n=100, method='inclusive')[94])


def main() -> None:
    import numpy as np
    from PIL import Image
    import torch
    import ultralytics
    import yaml
    from cleany_perception.adapters import yoloe_detector as module

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path, required=True)
    parser.add_argument('--engine', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--rounds', type=int, default=0)
    parser.add_argument('--variants', nargs='+', default=[
        'baseline', 'bulk', 'direct_sync', 'direct_async', 'direct_graph'],
        choices=['baseline', 'bulk', 'direct_sync', 'direct_async', 'direct_graph'])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    sources = args.output / 'source'
    sources.mkdir()
    source = inspect.getsource(module)
    (sources / 'yoloe_detector.py').write_text(source)
    (sources / Path(__file__).name).write_text(Path(__file__).read_text())

    candidate = textwrap.dedent(inspect.getsource(module.YoloeDetector.detect))
    old = '''        for index, box in enumerate(boxes):
            coordinates = _numpy(box.xyxy).reshape(-1, 4)[0]
            class_id = int(_numpy(box.cls).reshape(-1)[0])
            confidence = float(_numpy(box.conf).reshape(-1)[0])'''
    new = '''        box_data = _numpy(boxes.data)
        for index, box_row in enumerate(box_data):
            coordinates = box_row[:4]
            class_id = int(box_row[-1])
            confidence = float(box_row[-2])'''
    if candidate.count(old) != 1:
        raise ValueError('Adapter changed; inspect the candidate before rerunning')
    candidate = candidate.replace(old, new)
    namespace = vars(module).copy()
    exec(compile(candidate, '<probe-only-bulk-result-copy>', 'exec'), namespace)
    bulk_detect = namespace['detect']
    (sources / 'bulk_detect.py').write_text(candidate)

    class BulkDetector(module.YoloeDetector):
        def detect(self, rgb, query):
            # Keep borrowed TensorRT outputs locked until CPU results own their data.
            with self._model_pool.lock:
                backend = self._get_model().predictor.model.backend
                stream = getattr(backend, '_probe_stream', None)
                with torch.cuda.stream(stream) if stream is not None else nullcontext():
                    result = bulk_detect(self, rgb, query)
                    torch.cuda.current_stream().synchronize()
                    return result

    class DirectDetector(BulkDetector):
        def _predict(self, model, image):
            if model.predictor is None:
                return super()._predict(model, image)
            predictor = model.predictor
            predictor.args.conf = self._confidence_threshold
            predictor.args.iou = self._iou_threshold
            predictor.args.max_det = self._maximum_detections
            predictor.args.retina_masks = True
            predictor.args.rect = False
            bgr = np.ascontiguousarray(image[..., ::-1])
            predictor.batch = (['memory.png'], [bgr], [''])
            # Reuse the pinned version's preprocessing and mask restoration exactly.
            with torch.inference_mode():
                tensor = predictor.preprocess([bgr])
                predictions = predictor.inference(tensor)
                result = predictor.postprocess(predictions, tensor, [bgr])
            predictor.results = result
            return result

    def configure_backend(adapter, mode: str) -> dict:
        backend = adapter._model.predictor.model.backend
        if backend.dynamic or not backend.is_trt10:
            raise ValueError('This experiment requires a static TensorRT 10 engine')
        fixed_input = backend.bindings['images'].data
        backend._probe_stream = torch.cuda.Stream()
        pointers = {name: binding.data.data_ptr() for name, binding in backend.bindings.items()}
        for name, pointer in pointers.items():
            if not backend.context.set_tensor_address(name, pointer):
                raise RuntimeError(f'TensorRT refused binding: {name}')
        graph = None
        if mode == 'graph':
            # A graph gets its own context and persistent buffers. Never mutate
            # its context bindings or shapes after capture.
            stream = backend._probe_stream
            stream.wait_stream(torch.cuda.current_stream())
            graph = torch.cuda.CUDAGraph()
            with torch.cuda.stream(stream), torch.inference_mode():
                fixed_input.zero_()
                for _ in range(3):
                    if not backend.context.execute_async_v3(stream.cuda_stream):
                        raise RuntimeError('TensorRT warmup failed')
                stream.synchronize()
                with torch.cuda.graph(graph, stream=stream):
                    success = backend.context.execute_async_v3(stream.cuda_stream)
            if not success:
                raise RuntimeError('TensorRT graph capture refused execution')
            torch.cuda.current_stream().wait_stream(stream)
            torch.cuda.synchronize()

        def forward(image):
            if (image.shape != fixed_input.shape or image.dtype != fixed_input.dtype
                    or image.device != fixed_input.device or not image.is_contiguous()):
                raise ValueError('Input differs from the fixed execution contract')
            if graph is None:
                if not backend.context.set_tensor_address('images', image.data_ptr()):
                    raise RuntimeError('TensorRT refused input pointer')
                if not backend.context.execute_async_v3(torch.cuda.current_stream().cuda_stream):
                    raise RuntimeError('TensorRT asynchronous execution failed')
            else:
                fixed_input.copy_(image)
                graph.replay()
            return [backend.bindings[name].data for name in sorted(backend.output_names)]

        backend.forward = forward
        backend._probe_graph = graph
        backend._probe_fixed_input = fixed_input
        return dict(mode=mode, captured=graph is not None,
                    input_shape=list(fixed_input.shape), input_dtype=str(fixed_input.dtype))

    parameters = yaml.safe_load(args.profile.read_text())['perception_inspector']['ros__parameters']
    common = dict(model_path=str(args.engine), classes=parameters['yoloe_classes'],
        device='cuda:0', image_size=parameters['yoloe_image_size'],
        confidence_threshold=parameters['minimum_detection_confidence'],
        iou_threshold=parameters['yoloe_iou_threshold'],
        maximum_detections=parameters['maximum_detections'], require_masks=True)
    inventory = json.loads((args.benchmark / 'round-1/m/adapter/inputs.json').read_text())['samples']
    if args.limit:
        inventory = random.Random(20261028).sample(inventory, args.limit)
    samples = []
    for item in inventory:
        path = Path(item['path'])
        if sha256(path.read_bytes()) != item['sha256']:
            raise ValueError(f'Input changed: {path}')
        with Image.open(path) as image:
            samples.append((item, np.asarray(image.convert('RGB')).copy()))

    def fingerprint(detections) -> str:
        digest = hashlib.sha256()
        for detection in detections:
            digest.update(repr((detection.label, detection.detector_label,
                                detection.confidence, detection.bbox)).encode())
            mask = detection.segmentation_mask
            if mask.dtype != np.bool_ or mask.flags.writeable:
                raise ValueError('Mask ownership or dtype changed')
            digest.update(mask.tobytes())
        return digest.hexdigest()

    adapters = {}
    backend_details = {}
    for name in dict.fromkeys(['baseline', *args.variants]):
        cls = module.YoloeDetector if name == 'baseline' else (
            BulkDetector if name == 'bulk' else DirectDetector)
        adapters[name] = cls(**common, model_pool=module.YoloeModelPool())
        adapters[name].prepare()
        if name in ('direct_async', 'direct_graph'):
            backend_details[name] = configure_backend(adapters[name], name.split('_')[1])
    reference = {(item['dataset'], item['id']): fingerprint(adapters['baseline'].detect(rgb, ''))
                 for item, rgb in samples}
    package = Path(ultralytics.__file__).parent
    for filename in ('engine/predictor.py', 'utils/ops.py', 'nn/backends/tensorrt.py',
                     'models/yolo/segment/predict.py'):
        (sources / filename.replace('/', '_')).write_text((package / filename).read_text())
    environment = dict(engine=str(args.engine.resolve()), engine_sha256=sha256(args.engine.read_bytes()),
        adapter_sha256=sha256(source.encode()), packages=dict(ultralytics=ultralytics.__version__,
        torch=torch.__version__, cuda=torch.version.cuda), backend_details=backend_details,
        source_images=len(samples), global_library_patch=False, production_changed=False,
        clocks_changed=False, time_scope='RGB through CPU boxes and full masks; CUDA fenced',
        separate_context_per_variant=True)
    (args.output / 'environment.json').write_text(json.dumps(environment, indent=2) + '\n')
    (args.output / 'inputs.json').write_text(json.dumps(inventory, indent=2) + '\n')
    variants = args.variants
    rounds = args.rounds or len(variants)
    orders = [variants[i % len(variants):] + variants[:i % len(variants)] for i in range(rounds)]
    clock = next(Path('/sys/class/devfreq').glob('*gpu*/cur_freq'), None)
    rows, blocks = [], []
    with (args.output / 'timings.csv').open('w', newline='', buffering=1) as output:
        writer = csv.DictWriter(output, fieldnames=['round', 'variant', 'dataset', 'id',
            'latency_ms', 'detections', 'gpu_clock_hz', 'exact_output_match'])
        writer.writeheader()
        for round_index, order in enumerate(orders, 1):
            ordered = samples.copy()
            random.Random(20261028 + round_index).shuffle(ordered)
            for variant in order:
                adapter = adapters[variant]
                for index in range(20):
                    adapter.detect(ordered[index % len(ordered)][1], '')
                began = time.time()
                for item, rgb in ordered:
                    torch.cuda.synchronize()
                    start = time.perf_counter()
                    detections = adapter.detect(rgb, '')
                    torch.cuda.synchronize()
                    elapsed = (time.perf_counter() - start) * 1000
                    match = fingerprint(detections) == reference[(item['dataset'], item['id'])]
                    row = dict(round=round_index, variant=variant, dataset=item['dataset'], id=item['id'],
                        latency_ms=elapsed, detections=len(detections),
                        gpu_clock_hz=int(clock.read_text()) if clock else None, exact_output_match=match)
                    writer.writerow(row)
                    rows.append(row)
                    if not match:
                        raise ValueError(f'Output changed: {variant}/{item["dataset"]}/{item["id"]}')
                block = dict(round=round_index, variant=variant, start_epoch=began, end_epoch=time.time(),
                    latency_ms=distribution([r['latency_ms'] for r in rows
                        if r['round'] == round_index and r['variant'] == variant]))
                blocks.append(block)
                (args.output / 'blocks.json').write_text(json.dumps(blocks, indent=2) + '\n')
                print(json.dumps(block), flush=True)

    concurrency = {}
    peer_settings = dict(common, confidence_threshold=0.4, maximum_detections=4)
    baseline_peer = module.YoloeDetector(**peer_settings,
        model_pool=adapters['baseline']._model_pool)
    baseline_peer.prepare()
    peer_reference = {(item['dataset'], item['id']): fingerprint(baseline_peer.detect(rgb, ''))
                      for item, rgb in samples}
    for variant in variants:
        if variant == 'baseline':
            continue
        adapter = adapters[variant]
        peer = type(adapter)(**peer_settings, model_pool=adapter._model_pool)
        peer.prepare()
        if peer._model is not adapter._model:
            raise ValueError('Concurrent wrappers must share the same backend')
        retained = adapter.detect(samples[0][1], '')
        retained_hash = fingerprint(retained)
        def infer(index):
            item, rgb = samples[index % len(samples)]
            selected = peer if index % 2 else adapter
            expected = peer_reference if index % 2 else reference
            return fingerprint(selected.detect(rgb, '')) == expected[(item['dataset'], item['id'])]
        with ThreadPoolExecutor(max_workers=3) as executor:
            checks = list(executor.map(infer, range(48)))
        ownership_unchanged = fingerprint(retained) == retained_hash
        if not all(checks) or not ownership_unchanged:
            raise ValueError(f'Concurrent output or retained result changed: {variant}')
        rejected = False
        try:
            adapter.detect(np.zeros((32, 32), dtype=np.uint8), '')
        except module.InspectionFailure:
            rejected = True
        recovered = infer(0)
        if not rejected or not recovered:
            raise ValueError(f'Invalid-input recovery failed: {variant}')
        concurrency[variant] = dict(calls=48, workers=3, all_exact=True,
            shared_backend_different_thresholds=True, serialized_through_cpu_copy=True,
            retained_result_unchanged=ownership_unchanged, invalid_input_recovery=True)

    result = dict(environment=environment, orders=orders, variants={}, concurrency=concurrency, passed=True)
    for variant in variants:
        selected = [r for r in rows if r['variant'] == variant]
        result['variants'][variant] = dict(calls=len(selected),
            latency_ms=distribution([r['latency_ms'] for r in selected]),
            all_outputs_exact=all(r['exact_output_match'] for r in selected),
            gpu_clock_samples=dict(Counter(r['gpu_clock_hz'] for r in selected)))
    (args.output / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result['variants'], indent=2), flush=True)


if __name__ == '__main__':
    main()
