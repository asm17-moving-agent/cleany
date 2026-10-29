#!/usr/bin/env python3
"""Reload retained engines and replay benchmark images after artifact cleanup."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


def main() -> None:
    import numpy as np
    from PIL import Image
    import yaml
    from cleany_perception.adapters.yoloe_detector import YoloeDetector
    from cleany_perception.model_contract import sha256
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from validate_yoloe_trt import matching, serialize

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path, required=True)
    parser.add_argument('--model-root', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--models', nargs='+', choices=('n', 's', 'm'),
                        default=('n', 's', 'm'), help='Retained model sizes to verify.')
    args = parser.parse_args()
    settings = yaml.safe_load(args.profile.read_text())['perception_inspector']['ros__parameters']
    result = dict(started_epoch=time.time(), models={}, total_images=0, passed=False)
    for model in dict.fromkeys(args.models):
        root = args.benchmark / 'round-1' / model / 'adapter'
        inventory = json.loads((root / 'inputs.json').read_text())['samples']
        predictions = json.loads((root / 'predictions.json').read_text())
        environment = json.loads((root / 'environment.json').read_text())
        engine = args.model_root / 'real32' / model / 'active' / f'yoloe-26{model}-seg-real32.engine'
        if sha256(engine) != environment['engine_sha256']:
            raise ValueError(f'Engine changed after benchmark: {model}')
        adapter = YoloeDetector(str(engine), settings['yoloe_classes'], device='cuda:0',
            image_size=settings['yoloe_image_size'], confidence_threshold=settings['minimum_detection_confidence'],
            iou_threshold=settings['yoloe_iou_threshold'], maximum_detections=settings['maximum_detections'],
            require_masks=True)
        adapter.prepare()
        count = 0
        for sample, reference in zip(inventory, predictions, strict=True):
            if (sample['dataset'], sample['id']) != (reference['dataset'], reference['id']):
                raise ValueError('Prediction and input inventory order differ')
            path = Path(sample['path'])
            if sha256(path) != sample['sha256']:
                raise ValueError(f'Input changed: {path}')
            with Image.open(path) as image:
                rgb = np.asarray(image.convert('RGB')).copy()
            rows, _ = serialize(adapter.detect(rgb, ''))
            expected = reference['predictions']
            pairs = matching(expected, rows, strict=True)
            if len(rows) != len(expected) or len(pairs) != len(expected):
                raise ValueError(f'Post-cleanup output differs: {model}/{sample["id"]}')
            count += 1
        result['models'][model] = dict(images=count, engine_sha256=sha256(engine),
            execution_mode=adapter.execution_mode,
            runtime_files={name: (engine.resolve().parent / name).stat().st_size for name in
                           (engine.name, 'manifest.json', 'input.lock.json', 'validation.json')},
            predictions_match=True)
        result['total_images'] += count
        del adapter
        print(json.dumps(dict(model=model, images=count, passed=True)), flush=True)
    result.update(passed=True, finished_epoch=time.time())
    args.output.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
