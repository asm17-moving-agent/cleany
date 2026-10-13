"""Evaluate the deployed YOLOE adapter on held-out synthetic RGB/mask pairs."""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
import yaml

from cleany_perception.adapters.yoloe_detector import YoloeDetector


def _ground_truth(path: Path, shape: tuple[int, int]) -> dict[int, np.ndarray]:
    height, width = shape
    objects = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        values = line.split()
        class_id = int(values[0])
        points = np.asarray(
            [float(value) for value in values[1:]], dtype=np.float64
        )
        if len(points) < 6 or len(points) % 2:
            raise ValueError(f'Invalid polygon in {path}')
        points = points.reshape(-1, 2)
        points[:, 0] *= width
        points[:, 1] *= height
        mask = np.zeros(shape, dtype=np.uint8)
        cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 1)
        if class_id in objects:
            raise ValueError(f'Duplicate class {class_id} in {path}')
        objects[class_id] = mask.astype(bool)
    return objects


def _iou(left: np.ndarray, right: np.ndarray) -> float:
    union = np.count_nonzero(left | right)
    return float(np.count_nonzero(left & right) / union) if union else 0.0


def evaluate(
    dataset: Path, checkpoint: Path, text_encoder_directory: Path,
    *, device: str, confidence: float,
    class_thresholds: tuple[float, ...] = (),
) -> dict:
    config = yaml.safe_load(dataset.read_text(encoding='utf-8'))
    root = Path(config['path'])
    classes = [config['names'][index] for index in range(len(config['names']))]
    images = sorted((root / config['val']).glob('*.png'))
    if not images:
        raise ValueError('No validation images')
    detector = YoloeDetector(
        str(checkpoint), classes, device=device, image_size=640,
        confidence_threshold=confidence, iou_threshold=0.5,
        maximum_detections=10,
        text_encoder_directory=str(text_encoder_directory), require_masks=True,
        class_confidence_thresholds=class_thresholds)
    load_start = time.monotonic()
    detector.prepare()
    load_seconds = time.monotonic() - load_start
    matched = {name: 0 for name in classes}
    total = {name: 0 for name in classes}
    unmatched_detections = 0
    durations = []
    for image_path in images:
        rgb = np.asarray(Image.open(image_path).convert('RGB'))
        truth = _ground_truth(
            root / 'labels/val' / f'{image_path.stem}.txt', rgb.shape[:2]
        )
        start = time.monotonic()
        detections = detector.detect(rgb, '')
        durations.append(time.monotonic() - start)
        used: set[int] = set()
        for class_id, ground_mask in truth.items():
            name = classes[class_id]
            total[name] += 1
            matches = [(index, _iou(ground_mask, detection.segmentation_mask))
                       for index, detection in enumerate(detections)
                       if detection.label == name and index not in used]
            if not matches:
                continue
            index, overlap = max(matches, key=lambda item: item[1])
            if overlap >= 0.5:
                matched[name] += 1
                used.add(index)
        unmatched_detections += len(detections) - len(used)
    return {
        'checkpoint': str(checkpoint), 'confidence': confidence,
        'class_confidence_thresholds': class_thresholds,
        'images': len(images), 'objects': sum(total.values()),
        'model_load_seconds': round(load_seconds, 3),
        'mask_iou_50_recall': {
            name: round(matched[name] / count, 3) if count else None
            for name, count in total.items()},
        'matched_objects': sum(matched.values()),
        'unmatched_detections': unmatched_detections,
        'mean_inference_ms': round(1000 * sum(durations) / len(durations), 1),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--text-encoder-directory', type=Path, required=True)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--confidence', type=float, default=0.25)
    parser.add_argument(
        '--class-thresholds', type=float, nargs='*', default=[]
    )
    args = parser.parse_args()
    result = evaluate(
        args.dataset.expanduser(), args.checkpoint.expanduser(),
        args.text_encoder_directory.expanduser(),
        device=args.device, confidence=args.confidence,
        class_thresholds=tuple(args.class_thresholds),
    )
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
