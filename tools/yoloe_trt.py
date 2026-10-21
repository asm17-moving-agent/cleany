#!/usr/bin/env python3
"""Prepare fixed-prompt YOLOE segmentation and export a fresh FP16 version."""
from __future__ import annotations

import argparse
from contextlib import contextmanager, redirect_stderr, redirect_stdout
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import shutil
import sys
import time
import traceback
from typing import Any

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'ros2_ws/src/cleany_perception'))
from cleany_perception.model_contract import (  # noqa: E402
    SCHEMA, check_segmentation_shapes, ordered_names, require_hash, sha256,
)
from cleany_perception.model_runtime import load_model_profile  # noqa: E402

DEFAULT_LOCK = REPO / 'configs/models/yoloe-26s-seg-real34.lock.json'
DEFAULT_PROFILE = REPO / 'ros2_ws/src/cleany_perception/config/yoloe_real34.yaml'


def write_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    temporary.replace(path)


@contextmanager
def capture_log(path: Path):
    """Capture Python and native TensorRT output in one original log."""
    with path.open('w', encoding='utf-8') as log:
        sys.stdout.flush()
        sys.stderr.flush()
        saved = [os.dup(descriptor) for descriptor in (1, 2)]
        try:
            os.dup2(log.fileno(), 1)
            os.dup2(log.fileno(), 2)
            with redirect_stdout(log), redirect_stderr(log):
                yield
        finally:
            log.flush()
            for descriptor, original in zip((1, 2), saved):
                os.dup2(original, descriptor)
                os.close(original)


def check_environment(lock: dict[str, Any]) -> dict[str, Any]:
    packages = {name: importlib.metadata.version(name) for name in lock['packages']}
    for name, expected in lock['packages'].items():
        if packages[name] != expected:
            raise ValueError(f'{name}: expected {expected}, found {packages[name]}')
    import torch
    import tensorrt as trt
    import onnx  # noqa: F401
    if not torch.cuda.is_available():
        raise ValueError('CUDA is required for TensorRT export')
    runtime = {
        'python': '.'.join(platform.python_version_tuple()[:2]),
        'tensorrt': trt.__version__, 'cuda': torch.version.cuda,
        'compute_capability': list(torch.cuda.get_device_capability(0)),
    }
    for name, expected in lock['runtime'].items():
        if runtime[name] != expected:
            raise ValueError(f'{name}: expected {expected}, found {runtime[name]}')
    return {'packages': packages, 'runtime': runtime,
            'gpu': torch.cuda.get_device_name(0), 'platform': platform.platform(),
            'container_image_id': os.environ.get('CLEANY_YOLOE_IMAGE_ID', '')}


def load_inputs(lock_path: Path, profile: Path) -> dict[str, Any]:
    lock = json.loads(lock_path.read_text(encoding='utf-8'))
    require_hash(profile, lock['profile']['sha256'])
    parameters = load_model_profile(profile)
    classes = parameters['yoloe_classes']
    if (classes != lock['classes'] or len(classes) != 34
            or len({name.casefold() for name in classes}) != 34
            or any(not isinstance(name, str) or not name.strip() or name != name.strip()
                   for name in classes)):
        raise ValueError('Profile must contain the locked, ordered 34 classes')
    if parameters['yoloe_image_size'] != lock['export']['imgsz']:
        raise ValueError('Profile input size differs from lock')
    return lock


def inspect_onnx(path: Path, image_size: int, class_count: int) -> tuple[dict, dict]:
    import onnx
    graph = onnx.load(str(path))
    onnx.checker.check_model(graph)

    def shapes(items: Any) -> dict[str, list[int]]:
        return {item.name: [dim.dim_value for dim in item.type.tensor_type.shape.dim]
                for item in items}
    inputs, outputs = shapes(graph.graph.input), shapes(graph.graph.output)
    check_segmentation_shapes(inputs, outputs, image_size, class_count)
    return inputs, outputs


def export_version(
    checkpoint: Path, text_encoder: Path, versions: Path,
    lock_path: Path = DEFAULT_LOCK, profile: Path = DEFAULT_PROFILE,
) -> Path:
    checkpoint, text_encoder = checkpoint.resolve(), text_encoder.resolve()
    settings_hash = hashlib.sha256(lock_path.read_bytes()).hexdigest()[:12]
    version = versions.resolve() / (
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + '-' + settings_hash)
    version.mkdir(parents=True, exist_ok=False)
    manifest: dict[str, Any] = {
        'schema': SCHEMA, 'task': 'segment', 'status': 'running',
        'fine_tuned': False, 'embedding_mode': 'fixed_text',
        'started_at': datetime.now(timezone.utc).isoformat(),
        'lock_sha256': sha256(lock_path), 'artifacts': {},
    }
    started = time.monotonic()
    try:
        with capture_log(version / 'export.log'):
            shutil.copyfile(lock_path, version / 'input.lock.json')
            shutil.copyfile(profile, version / 'profile.yaml')
            lock = load_inputs(lock_path, profile)
            manifest.update(classes=lock['classes'], export=lock['export'], source=lock['source'])
            # Verify both trusted binary inputs before loading either pickle.
            require_hash(checkpoint, lock['source']['checkpoint_sha256'])
            require_hash(text_encoder, lock['source']['text_encoder_sha256'])
            manifest['environment'] = check_environment(lock)
            os.environ['YOLO_AUTOINSTALL'] = 'false'
            from ultralytics import YOLOE
            model = YOLOE(str(checkpoint), task='segment')
            if model.task != 'segment' or getattr(model.model.model[-1], 'is_fused', False):
                raise ValueError('An unfused YOLOE segmentation source is required')
            # Text encoder lookup is relative to cwd. Export uses a fresh directory.
            (version / 'mobileclip2_b.ts').symlink_to(text_encoder)
            previous = Path.cwd()
            try:
                os.chdir(version)
                model.to('cuda:0')
                model.set_classes(lock['classes'])
            finally:
                os.chdir(previous)
                (version / 'mobileclip2_b.ts').unlink()
            if ordered_names(model.names) != lock['classes']:
                raise ValueError('Prepared class IDs differ from lock')
            prepared = version / 'yoloe-26s-seg-real34.pt'
            model.save(str(prepared))
            # Export the saved artifact, not a separately re-prompted model.
            model = YOLOE(str(prepared), task='segment')
            engine_path = Path(model.export(**lock['export'])).resolve()
            if engine_path != prepared.with_suffix('.engine'):
                raise ValueError(f'Export escaped version directory: {engine_path}')
            onnx_path = prepared.with_suffix('.onnx')
            manifest['inputs'], manifest['outputs'] = inspect_onnx(
                onnx_path, lock['export']['imgsz'], len(lock['classes']))
            manifest['status'] = 'built_requires_validation'
    except Exception as error:
        manifest.update(status='failed', error=f'{type(error).__name__}: {error}')
        with (version / 'export.log').open('a', encoding='utf-8') as log:
            traceback.print_exc(file=log)
        raise
    finally:
        manifest['duration_seconds'] = time.monotonic() - started
        for path in version.iterdir():
            if path.suffix in ('.pt', '.onnx', '.engine'):
                manifest['artifacts'][path.name] = {
                    'sha256': sha256(path), 'size_bytes': path.stat().st_size}
        manifest['export_log_sha256'] = sha256(version / 'export.log')
        write_json(version / 'manifest.json', manifest)
        print(f'YOLOE version: {version}', flush=True)
    return version


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--text-encoder', type=Path, required=True)
    parser.add_argument('--versions', type=Path, required=True)
    parser.add_argument('--lock', type=Path, default=DEFAULT_LOCK)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    args = parser.parse_args()
    export_version(args.checkpoint, args.text_encoder, args.versions, args.lock, args.profile)


if __name__ == '__main__':
    main()
