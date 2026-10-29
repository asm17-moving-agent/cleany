#!/usr/bin/env python3
"""Remove known YOLOE build-only binaries after validating retained engines."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys
import time

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'ros2_ws/src/cleany_perception'))
from cleany_perception.model_contract import read_model_manifest, sha256


def confined_file(path: Path, boundary: Path) -> bool:
    """Never follow a candidate symlink or an ancestor outside the allowed root."""
    return path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(boundary.resolve())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-root', type=Path, required=True)
    parser.add_argument('--benchmark', type=Path, required=True)
    parser.add_argument('--previous-benchmark', type=Path)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    root = args.model_root.resolve()
    evidence = args.benchmark.resolve()
    suite = json.loads((evidence / 'suite.json').read_text())
    if suite['status'] != 'complete':
        raise ValueError('The comparison must finish before removing its inputs')
    retained = {}
    candidates: dict[Path, Path] = {}
    for model in ('n', 's', 'm'):
        version = (root / 'real32' / model / 'active').resolve(strict=True)
        if not version.is_relative_to(root / 'real32' / model):
            raise ValueError('Active engine escaped its model directory')
        engine = version / f'yoloe-26{model}-seg-real32.engine'
        manifest = json.loads((version / 'manifest.json').read_text())
        read_model_manifest(engine, manifest['classes'], manifest['export']['imgsz'])
        retained[str(engine)] = sha256(engine)
        for suffix in ('.pt', '.onnx'):
            path = engine.with_suffix(suffix)
            if path.exists():
                expected = manifest['artifacts'][path.name]['sha256']
                if sha256(path) != expected:
                    raise ValueError(f'Build artifact changed: {path}')
                candidates[path] = root
        source = manifest['source']
        for filename, key in [(source['checkpoint_filename'], 'checkpoint_sha256'),
                              (source['text_encoder_filename'], 'text_encoder_sha256')]:
            path = root / filename
            if path.exists():
                if sha256(path) != source[key]:
                    raise ValueError(f'Export source changed: {path}')
                candidates[path] = root
    # The abandoned real29 export has no engine and is not a runtime target.
    abandoned = root / 'real29'
    if abandoned.exists():
        if any(abandoned.rglob('*.engine')) or any(abandoned.rglob('active')):
            raise ValueError('real29 now contains a runtime target; inspect it separately')
        for name in ('yoloe-26s-seg-real29.pt', 'yoloe-26s-seg-real29.onnx'):
            for path in abandoned.rglob(name):
                candidates[path] = abandoned
    artifact_roots = [evidence / 'round-1' / model / 'adapter' for model in ('n', 's', 'm')]
    if args.previous_benchmark:
        artifact_roots.append(args.previous_benchmark.resolve() / 'adapter')
    for directory in artifact_roots:
        record = json.loads((directory / 'trtexec-input.json').read_text())
        for name, key in [('engine.plan', 'payload_sha256'), ('input.bin', 'input_sha256')]:
            path = directory / name
            if path.exists():
                if sha256(path) != record[key]:
                    raise ValueError(f'Native benchmark input changed: {path}')
                candidates[path] = directory
    if not all(confined_file(path, boundary) for path, boundary in candidates.items()):
        raise ValueError('A cleanup candidate is a symlink, missing, or outside its boundary')
    entries = [dict(path=str(path), bytes=path.stat().st_size,
                    allocated_bytes=path.stat().st_blocks * 512, sha256=sha256(path))
               for path in sorted(candidates)]
    receipt = dict(timestamp=time.time(), retained_engines=retained, files=entries,
                   planned_bytes=sum(entry['bytes'] for entry in entries),
                   free_bytes_before=shutil.disk_usage(root).free)
    (evidence / 'cleanup-plan.json').write_text(json.dumps(receipt, indent=2) + '\n')
    if not args.execute:
        print(json.dumps(dict(files=len(entries), planned_bytes=receipt['planned_bytes'])), flush=True)
        return
    for entry in entries:
        path = Path(entry['path'])
        if not confined_file(path, candidates[path]) or sha256(path) != entry['sha256']:
            raise ValueError(f'Cleanup candidate changed after planning: {path}')
        path.unlink()
    if not all(sha256(Path(path)) == expected for path, expected in retained.items()):
        raise ValueError('A retained engine changed during cleanup')
    receipt.update(deleted_bytes=receipt['planned_bytes'],
                   deleted_allocated_bytes=sum(entry['allocated_bytes'] for entry in entries),
                   verified_absent=all(not Path(entry['path']).exists() for entry in entries),
                   retained_engines_unchanged=True, free_bytes_after=shutil.disk_usage(root).free)
    (evidence / 'cleanup.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(dict(deleted_files=len(entries), deleted_bytes=receipt['deleted_bytes'],
                         verified_absent=receipt['verified_absent'])), flush=True)


if __name__ == '__main__':
    main()
