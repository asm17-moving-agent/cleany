"""File-backed contract shared by segmentation export and inference."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Sequence


SCHEMA = 'cleany.yoloe_seg_trt/v1'


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def require_hash(path: Path, expected: str) -> None:
    if sha256(path) != expected:
        raise ValueError(f'SHA-256 mismatch: {path}')


def ordered_names(names: Any) -> list[str]:
    if isinstance(names, dict):
        converted = {int(key): value for key, value in names.items()}
        if set(converted) != set(range(len(converted))):
            raise ValueError('Class IDs must be contiguous from zero')
        return [converted[index] for index in range(len(converted))]
    return list(names)


def check_segmentation_shapes(
    inputs: dict[str, list[int]], outputs: dict[str, list[int]],
    image_size: int, class_count: int,
) -> None:
    if inputs != {'images': [1, 3, image_size, image_size]}:
        raise ValueError(f'Expected static square batch-one input: {inputs}')
    if set(outputs) != {'output0', 'output1'}:
        raise ValueError(f'Segmentation requires detections and prototypes: {outputs}')
    detections, proto = outputs['output0'], outputs['output1']
    if (len(proto) != 4 or proto[0] != 1 or any(x <= 0 for x in proto)
            or len(detections) != 3 or detections[0] != 1):
        raise ValueError(f'Invalid segmentation outputs: {outputs}')
    # YOLO26 end-to-end: xyxy, confidence, class ID, mask coefficients.
    # Older heads: xywh, per-class scores, mask coefficients, anchor axis.
    end_to_end = detections[1] == 300 and detections[2] == 6 + proto[1]
    dense = detections[1] == 4 + class_count + proto[1] and detections[2] > 0
    if not (end_to_end or dense):
        raise ValueError(f'Mask coefficients do not match prototypes: {outputs}')


def read_model_manifest(
    model_path: Path, classes: Sequence[str], image_size: int, *,
    allow_unvalidated: bool = False,
) -> dict[str, Any]:
    manifest_path = model_path.parent / 'manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest.get('schema') != SCHEMA or manifest.get('task') != 'segment':
        raise ValueError('Model manifest must describe YOLOE segmentation')
    if manifest.get('classes') != list(classes):
        raise ValueError('Model manifest class order differs from profile')
    if manifest.get('status') not in ('built_requires_validation', 'validated'):
        raise ValueError('Model build did not finish successfully')
    lock_path = model_path.parent / 'input.lock.json'
    require_hash(lock_path, manifest['lock_sha256'])
    lock = json.loads(lock_path.read_text(encoding='utf-8'))
    if lock['classes'] != list(classes) or lock['export'] != manifest['export']:
        raise ValueError('Model lock differs from manifest')
    check_segmentation_shapes(manifest['inputs'], manifest['outputs'],
                              image_size, len(classes))
    require_hash(model_path, manifest['artifacts'][model_path.name]['sha256'])
    if model_path.suffix == '.engine' and not allow_unvalidated:
        if manifest['status'] != 'validated':
            raise ValueError('TensorRT engine has not passed comparison validation')
        validation_path = model_path.parent / manifest['validation']['filename']
        require_hash(validation_path, manifest['validation']['sha256'])
        validation = json.loads(validation_path.read_text(encoding='utf-8'))
        if (validation.get('passed') is not True
                or validation.get('candidate_sha256') != sha256(model_path)
                or validation.get('lock_sha256') != manifest['lock_sha256']):
            raise ValueError('Validation does not certify this engine and lock')
    return manifest
