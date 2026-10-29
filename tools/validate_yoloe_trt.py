#!/usr/bin/env python3
"""Compare saved images through the production segmentation adapter and gate activation."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
import traceback
from typing import Any

import numpy as np
from PIL import Image

from yoloe_trt import DEFAULT_PROFILE, artifact_stem, capture_log, check_environment, write_json
from cleany_perception.adapters.yoloe_detector import YoloeDetector
from cleany_perception.adapters.gemini_detector import parse_gemini_classifications
from cleany_perception.adapters.yoloe_reference_tracker import YoloeReferenceTracker
from cleany_perception.core.models import FailureKind, InspectionFailure
from cleany_perception.model_contract import read_model_manifest, require_hash, sha256
from cleany_perception.model_runtime import load_model_profile


def bbox_iou(left: list[float], right: list[float]) -> float:
    x1, y1, x2, y2 = left
    a1, b1, a2, b2 = right
    intersection = max(0., min(x2, a2) - max(x1, a1)) * max(0., min(y2, b2) - max(y1, b1))
    union = max(0., x2 - x1) * max(0., y2 - y1) + max(0., a2 - a1) * max(0., b2 - b1) - intersection
    return intersection / union if union else 0.


def matching(reference: list[dict], candidate: list[dict], *, strict: bool = False) -> list[tuple[int, int]]:
    """Maximum one-to-one matching; never count one engine box twice."""
    edges = []
    for left in reference:
        compatible = []
        for index, right in enumerate(candidate):
            if left['label'] != right['label']:
                continue
            values = [left['confidence'], right['confidence'], *left['bbox'], *right['bbox']]
            if not all(np.isfinite(values)):
                continue
            iou = bbox_iou(left['bbox'], right['bbox'])
            if strict:
                accepted = (abs(left['confidence'] - right['confidence']) <= 0.01
                            and max(abs(a - b) for a, b in zip(left['bbox'], right['bbox'])) <= 1.)
            else:
                accepted = iou >= 0.5
            if accepted:
                compatible.append((iou, index))
        edges.append([index for _, index in sorted(compatible, reverse=True)])
    assigned: dict[int, int] = {}

    def assign(left: int, seen: set[int]) -> bool:
        for right in edges[left]:
            if right in seen:
                continue
            seen.add(right)
            if right not in assigned or assign(assigned[right], seen):
                assigned[right] = left
                return True
        return False
    for index in range(len(reference)):
        assign(index, set())
    return sorted((left, right) for right, left in assigned.items())


def mask_iou(left: np.ndarray, right: np.ndarray) -> float:
    if left.shape != right.shape:
        raise ValueError('Compared masks must use the same original image coordinates')
    union = np.count_nonzero(left | right)
    return float(np.count_nonzero(left & right) / union) if union else 0.


def serialize(detections: Any) -> tuple[list[dict], list[np.ndarray]]:
    rows, masks = [], []
    for item in detections:
        mask = item.segmentation_mask
        if mask is None or mask.dtype != np.bool_ or not np.any(mask):
            raise ValueError('Comparison requires nonempty boolean instance masks')
        rows.append({'label': item.label, 'confidence': item.confidence,
                     'bbox': [item.bbox.x_min, item.bbox.y_min, item.bbox.x_max, item.bbox.y_max]})
        masks.append(mask)
    return rows, masks


def acceptance(reference_count: int, matched_count: int, mask_ious: list[float],
               timings: dict[str, list[float]], *, strict_rebuild: bool = True) -> dict:
    match_rate = matched_count / reference_count if reference_count else None
    mean_mask = float(np.mean(mask_ious)) if mask_ious else None
    latency = {name: {'p50_ms': float(np.percentile(values, 50)),
                      'p95_ms': float(np.percentile(values, 95)), 'count': len(values)}
               for name, values in timings.items()}
    speedup = 1 - latency['candidate']['p50_ms'] / latency['reference']['p50_ms']
    gates = {'detection_match': match_rate is not None and match_rate >= 0.98,
             'mask_iou': mean_mask is not None and mean_mask >= 0.95,
             'p50_speed': speedup >= 0.10,
             'p95_latency': latency['candidate']['p95_ms'] <= latency['reference']['p95_ms'],
             'strict_rebuild': strict_rebuild}
    return {'passed': all(gates.values()), 'gates': gates,
            'reference_detection_count_conf_ge_030': reference_count,
            'matched_detection_count': matched_count, 'match_rate': match_rate,
            'mean_matched_mask_iou': mean_mask, 'latency': latency, 'p50_reduction': speedup}


def replay_head_labels(adapter: YoloeDetector, samples: list) -> dict:
    """Saved RGB association only; semantic responses are fixtures, not API evidence."""
    outcomes = []
    tracker = YoloeReferenceTracker(adapter)

    def track_outcome(rgb: np.ndarray, detection: Any) -> tuple[Any, str | None]:
        try:
            return tracker.track(rgb, detection, rgb), None
        except InspectionFailure as error:
            if error.kind != FailureKind.MASK:
                raise
            return None, str(error)

    for _, rgb, metadata in samples:
        if metadata['camera'] != 'head':
            continue
        detections = adapter.detect(rgb, '')
        response = {'objects': [{
            'id': index, 'label': 'paper cup' if detection.label == 'cup' else detection.label,
            'sorting_category': 'review',
        } for index, detection in enumerate(detections, 1)]}
        classified = parse_gemini_classifications(json.dumps(response), detections)
        for original, item in zip(detections, classified):
            canonical_mask, canonical_error = track_outcome(rgb, original)
            tracked, normalized_error = track_outcome(rgb, item)
            iou = mask_iou(tracked, canonical_mask) if tracked is not None and canonical_mask is not None else None
            same_association = (iou is not None and iou >= .95) or (
                canonical_error is not None and normalized_error == canonical_error)
            outcomes.append({'sample_id': metadata['sample_id'], 'label': item.label,
                             'geometry_preserved': item.bbox is original.bbox
                             and item.segmentation_mask is original.segmentation_mask,
                             'reference_mask_iou': iou, 'same_association': same_association,
                             'canonical_rejection': canonical_error,
                             'normalized_rejection': normalized_error})
    return {'scope': 'saved RGB head OBSERVE/reference label association; no TF or 3D evidence',
            'gemini_source': 'offline fixtures', 'outcomes': outcomes,
            'passed': any(item['reference_mask_iou'] is not None for item in outcomes)
            and all(item['geometry_preserved'] and item['same_association'] for item in outcomes)}


def activate(version: Path, validation: dict, previous_active: Path | None) -> None:
    manifest_path = version / 'manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    validation_path = version / 'validation.json'
    filename = validation.get('candidate_filename', 'yoloe-26s-seg-real34.engine')
    require_hash(version / filename, validation['candidate_sha256'])
    if not validation['passed']:
        raise ValueError('Failed comparison cannot activate an engine')
    active = version.parent / 'active'
    current = active.resolve() if active.is_symlink() else None
    if current != previous_active:
        raise ValueError('Active version changed during validation; rerun comparison')
    if active.exists() and not active.is_symlink():
        raise ValueError('Active path must be a relative symlink')
    write_json(validation_path, validation)
    manifest.update(status='validated', validation={
        'filename': validation_path.name, 'sha256': sha256(validation_path)})
    write_json(manifest_path, manifest)
    temporary = version.parent / ('.active-' + version.name)
    temporary.symlink_to(version.name)
    temporary.replace(active)


def validate(version: Path, dataset_roots: list[Path], output: Path,
             profile: Path = DEFAULT_PROFILE, *, promote: bool = False) -> dict:
    version = version.resolve()
    output.mkdir(parents=True, exist_ok=False)
    result: dict[str, Any] = {'schema': 'cleany.yoloe_seg_comparison/v1', 'passed': False,
                              'started_at': datetime.now(timezone.utc).isoformat(),
                              'warmup': 5, 'repeats': 3, 'rect': False, 'errors': []}
    try:
        with capture_log(output / 'validation.log'):
            parameters = load_model_profile(profile)
            classes = parameters['yoloe_classes']
            lock = json.loads((version / 'input.lock.json').read_text(encoding='utf-8'))
            engine = version / f'{artifact_stem(lock)}.engine'
            manifest = read_model_manifest(engine, classes, 640, allow_unvalidated=True)
            require_hash(profile, lock['profile']['sha256'])
            result['environment'] = check_environment(lock)
            result.update(candidate_sha256=sha256(engine), candidate_filename=engine.name,
                          lock_sha256=manifest['lock_sha256'])
            reference = engine.with_suffix('.pt')
            active = version.parent / 'active'
            previous_active = active.resolve() if active.is_symlink() else None
            paths = {'reference': reference, 'candidate': engine}
            if previous_active is not None and previous_active != version:
                previous_engine = previous_active / engine.name
                read_model_manifest(previous_engine, classes, 640)
                paths['deployed'] = previous_engine
            result['model_hashes'] = {name: sha256(path) for name, path in paths.items()}
            samples = []
            coverage = {label: [] for label in classes}
            datasets = []
            for root in dataset_roots:
                csv_path = root / 'manifest.csv'
                with csv_path.open(newline='', encoding='utf-8') as stream:
                    rows = list(csv.DictReader(stream))
                if not rows:
                    raise ValueError(f'Empty image dataset: {root}')
                datasets.append({'root': str(root), 'sample_count': len(rows),
                                 'manifest_sha256': sha256(csv_path)})
                for row in rows:
                    image_path = root / row['rgb']
                    with Image.open(image_path) as image:
                        rgb = np.asarray(image.convert('RGB')).copy()
                    key = f'{len(samples):04d}'
                    # Only supplied annotations establish class coverage, not model guesses.
                    present = json.loads(row.get('classes_present') or '[]')
                    if any(label not in classes for label in present):
                        raise ValueError('Dataset coverage label differs from profile')
                    for label in present:
                        coverage[label].append(key)
                    samples.append((key, rgb, {'sample_id': row['sample_id'],
                                   'rgb': str(image_path), 'sha256': sha256(image_path),
                                   'camera': row.get('camera', 'unspecified'),
                                   'classes_present': present}))
            result.update(datasets=datasets, sample_count=len(samples),
                          class_coverage={label: {
                              'available_images': keys,
                              'status': 'available' if keys else 'unverified',
                          } for label, keys in coverage.items()})
            result['sample_inputs'] = [metadata for _, _, metadata in samples]
            adapters = {name: YoloeDetector(
                str(path), classes, device='cuda:0', image_size=640,
                confidence_threshold=0.25, maximum_detections=10,
                iou_threshold=parameters['yoloe_iou_threshold'], require_masks=True,
                allow_unvalidated_engine=name == 'candidate')
                for name, path in paths.items()}
            import torch

            def timed_call(adapter: YoloeDetector, rgb: np.ndarray) -> tuple[Any, float]:
                torch.cuda.synchronize()
                start = time.perf_counter()
                detections = adapter.detect(rgb, '')
                torch.cuda.synchronize()
                return detections, (time.perf_counter() - start) * 1000
            result['startup'] = {}
            for name, adapter in adapters.items():
                start = time.perf_counter()
                adapter.prepare()
                torch.cuda.synchronize()
                load_ms = (time.perf_counter() - start) * 1000
                _, first_ms = timed_call(adapter, samples[0][1])
                result['startup'][name] = {
                    'prepare_ms': load_ms, 'first_request_ms': first_ms,
                    'prepare_includes_engine_probe': paths[name].suffix == '.engine',
                }
                for _ in range(5):
                    timed_call(adapter, samples[0][1])
            reference_count = matched_count = strict_pass_count = 0
            mask_ious: list[float] = []
            timings: dict[str, list[float]] = {name: [] for name in paths}
            observed = {label: {name: 0 for name in paths} for label in classes}
            deployed_pass = True
            with (output / 'raw.jsonl').open('w', encoding='utf-8') as raw:
                for repeat in range(3):
                    for key, rgb, metadata in samples:
                        rows, masks = {}, {}
                        # Alternate timing order to limit drift bias; calls remain sequential.
                        order = list(paths) if repeat % 2 == 0 else list(reversed(paths))
                        for name in order:
                            detections, duration = timed_call(adapters[name], rgb)
                            rows[name], masks[name] = serialize(detections)
                            timings[name].append(duration)
                            for row in rows[name]:
                                observed[row['label']][name] += 1
                        high_ids = [i for i, row in enumerate(rows['reference']) if row['confidence'] >= 0.30]
                        pairs = matching([rows['reference'][i] for i in high_ids], rows['candidate'])
                        ious = [mask_iou(masks['reference'][high_ids[i]], masks['candidate'][j]) for i, j in pairs]
                        mask_ious.extend(ious)
                        reference_count += len(high_ids)
                        matched_count += len(pairs)
                        strict = (len(rows['reference']) == len(rows['candidate'])
                                  and len(matching(rows['reference'], rows['candidate'], strict=True)) == len(rows['reference']))
                        strict_pass_count += int(strict)
                        deployed_strict = None
                        if 'deployed' in rows:
                            deployed_strict = (len(rows['deployed']) == len(rows['candidate'])
                                               and len(matching(rows['deployed'], rows['candidate'], strict=True)) == len(rows['deployed']))
                            deployed_pass &= deployed_strict
                        mask_file = output / f'masks-{repeat}-{key}.npz'
                        np.savez_compressed(mask_file, **{
                            f'{name}_{index}': mask for name, items in masks.items()
                            for index, mask in enumerate(items)})
                        raw.write(json.dumps({'sample': metadata, 'repeat': repeat, 'predictions': rows,
                                              'timings_ms': {name: timings[name][-1] for name in paths},
                                              'matches': [(high_ids[i], j) for i, j in pairs],
                                              'mask_ious': ious, 'strict_parity': strict,
                                              'deployed_strict_parity': deployed_strict,
                                              'masks_file': mask_file.name, 'masks_sha256': sha256(mask_file)}) + '\n')
            result.update(acceptance(reference_count, matched_count, mask_ious, timings,
                                     strict_rebuild=deployed_pass))
            result.update(observed_detections=observed, strict_pass_count=strict_pass_count,
                          strict_comparison_count=len(samples) * 3,
                          previous_active=str(previous_active) if previous_active else None)
            result['raw_sha256'] = sha256(output / 'raw.jsonl')
            cameras = {metadata['camera'] for _, _, metadata in samples}
            result['gates']['dataset_protocol'] = (
                any(dataset['sample_count'] == 100 for dataset in datasets)
                and {'head', 'left_wrist', 'right_wrist'} <= cameras)
            result['head_label_replay'] = replay_head_labels(adapters['candidate'], samples)
            result['gates']['head_label_replay'] = result['head_label_replay']['passed']
            result['passed'] = all(result['gates'].values())
            if promote and result['passed']:
                activate(version, result, previous_active)
                result['activated'] = True
    except Exception as error:
        result['passed'] = False
        result['errors'].append(f'{type(error).__name__}: {error}')
        with (output / 'validation.log').open('a', encoding='utf-8') as log:
            traceback.print_exc(file=log)
    finally:
        result['validation_log_sha256'] = sha256(output / 'validation.log')
        write_json(output / 'comparison.json', result)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version', type=Path, required=True)
    parser.add_argument('--dataset-root', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--activate', action='store_true')
    args = parser.parse_args()
    result = validate(args.version, args.dataset_root, args.output, args.profile, promote=args.activate)
    print(json.dumps({key: result[key] for key in ('passed', 'errors', 'sample_count') if key in result}, indent=2))
    if not result['passed']:
        sys.exit(1)


if __name__ == '__main__':
    main()
