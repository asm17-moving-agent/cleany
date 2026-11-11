#!/usr/bin/env python3
"""Run segmentation export/validation with the pinned Jetson image."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

REPO = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('build', 'export', 'validate'))
    parser.add_argument('--image', default='cleany-yoloe-export:jp6.2')
    parser.add_argument('--vision-image', default='cleany-vision:jp6.2')
    parser.add_argument('--model-dir', type=Path, default=Path.home() / 'models/yoloe')
    parser.add_argument('--model-size', choices=('s', 'm'), default='s')
    parser.add_argument('--lock', type=Path)
    parser.add_argument('--profile', type=Path)
    parser.add_argument('--versions', type=Path)
    parser.add_argument('--version', type=Path)
    parser.add_argument('--dataset-root', type=Path, action='append')
    parser.add_argument('--activate', action='store_true')
    args = parser.parse_args()
    if args.command == 'build':
        subprocess.run(['docker', 'build', '--build-arg', f'VISION_IMAGE={args.vision_image}',
                        '-f', str(REPO / 'containers/vision/Dockerfile.yoloe-export'),
                        '-t', args.image, str(REPO / 'containers/vision')], check=True)
        return
    import os
    model_dir = args.model_dir.expanduser().resolve()
    image_id = subprocess.check_output(
        ['docker', 'image', 'inspect', '--format', '{{.Id}}', args.image], text=True).strip()
    command = ['docker', 'run', '--rm', '--runtime=nvidia', '--network=none',
               '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'YOLO_CONFIG_DIR=/tmp/yolo-config',
               '-e', 'MPLCONFIGDIR=/tmp/matplotlib', '-e', f'CLEANY_YOLOE_IMAGE_ID={image_id}',
               '-v', f'{REPO}:{REPO}:ro', '-v', f'{model_dir}:{model_dir}',
               '--entrypoint', 'python3']
    versions = (args.versions or model_dir / 'real32' / args.model_size).expanduser().resolve()
    lock = (args.lock or REPO / f'configs/models/yoloe-26{args.model_size}-seg-real32.lock.json').resolve()
    inputs = json.loads(lock.read_text())
    profile = (args.profile or REPO / 'ros2_ws/src/cleany_perception/config'
               / inputs['profile']['filename']).resolve()
    if args.command == 'export':
        command += [args.image, str(REPO / 'tools/yoloe_trt.py'),
                    '--checkpoint', str(model_dir / inputs['source']['checkpoint_filename']),
                    '--text-encoder', str(model_dir / 'mobileclip2_b.ts'),
                    '--versions', str(versions), '--lock', str(lock), '--profile', str(profile)]
    else:
        if args.version:
            version = args.version.expanduser().resolve()
        else:
            candidates = [path.parent for path in versions.glob('*/manifest.json')
                          if path.parent.name != 'active' and json.loads(path.read_text()).get('status')
                          in ('built_requires_validation', 'validated')]
            if not candidates:
                raise ValueError('No completed segmentation export; run make yoloe-trt-export')
            version = sorted(candidates)[-1]
        if version.parent != versions:
            raise ValueError('Version must be inside the selected versions directory')
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        output = version / ('validation-' + stamp)
        telemetry = version / ('telemetry-' + stamp)
        roots = args.dataset_root or [Path.home() / 'datasets/ocid-cleany-100']
        for root in roots:
            command += ['-v', f'{root.resolve()}:{root.resolve()}:ro']
        command += [args.image, str(REPO / 'tools/validate_yoloe_trt.py'),
                    '--version', str(version), '--output', str(output), '--profile', str(profile)]
        for root in roots:
            command += ['--dataset-root', str(root.resolve())]
        if args.activate:
            command.append('--activate')
        command = [sys.executable, str(REPO / 'tools/record_jetson_telemetry.py'),
                   '--output-root', str(telemetry), '--interval-ms', '200', '--', *command]
        print(f'Comparison: {output}\nTelemetry: {telemetry}', flush=True)
    subprocess.run(command, check=True)


if __name__ == '__main__':
    main()
