#!/usr/bin/env python3
"""Run isolated n/s/m Jetson benchmarks in a balanced, sequential order."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def output(*command: str) -> dict:
    result = subprocess.run(command, capture_output=True, text=True)
    return dict(returncode=result.returncode, stdout=result.stdout, stderr=result.stderr)


def round_order(index: int) -> list[str]:
    names = ['n', 's', 'm']
    shift = index % len(names)
    return names[shift:] + names[:shift]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--model-root', type=Path, default=Path.home() / 'models/yoloe')
    parser.add_argument('--dataset-root', type=Path, action='append', required=True)
    parser.add_argument('--data-root', type=Path, default=Path.home() / 'cleany-data')
    parser.add_argument('--image', default='cleany-vision:jp6.2')
    parser.add_argument('--passes', type=int, default=10)
    parser.add_argument('--native-seconds', type=int, default=30)
    args = parser.parse_args()
    repo, root = args.repo.resolve(), args.output.resolve()
    root.mkdir(parents=True, exist_ok=True)
    if (root / 'suite.json').exists():
        raise ValueError('Choose a fresh benchmark output directory')
    source = root / 'source'
    source.mkdir(exist_ok=True)
    files = ['tools/validation/benchmark_yoloe_dataset.py', 'tools/record_jetson_telemetry.py',
             'tools/validation/benchmark_yoloe_scales.py',
             'ros2_ws/src/cleany_perception/cleany_perception/adapters/yoloe_detector.py',
             'ros2_ws/src/cleany_perception/cleany_perception/model_contract.py',
             'ros2_ws/src/cleany_perception/config/yoloe_real32.yaml',
             'containers/vision/scripts/run-perception']
    hashes = {}
    for relative in files:
        path = repo / relative
        shutil.copy2(path, source / path.name)
        hashes[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    write_json(root / 'host-environment.json', dict(
        started_epoch=time.time(), commit=output('git', '-C', str(repo), 'rev-parse', 'HEAD'),
        source_sha256=hashes, diff=output('git', '-C', str(repo), 'diff', 'HEAD', '--', *files),
        image=output('docker', 'image', 'inspect', '--format', '{{.Id}}', args.image),
        nvpmodel=output('nvpmodel', '-q'), containers=output('docker', 'ps', '--format', '{{.Names}}'),
        system=output('uname', '-a'), packages=output('dpkg-query', '-W', 'nvidia-jetpack', 'nvidia-l4t-core', 'libnvinfer10'),
        gpu_frequency_limits={p.name: p.read_text() for p in Path('/sys/class/devfreq/17000000.gpu').glob('*_freq')
                              if p.name in ('min_freq', 'max_freq')}))
    status = dict(orders=[round_order(i) for i in range(3)], runs=[], status='running',
                  passes=args.passes, native_seconds=args.native_seconds)
    write_json(root / 'suite.json', status)

    def measured(command: list[str], destination: Path, phase: str, model: str, round_number: int) -> None:
        destination.mkdir(parents=True, exist_ok=True)
        full = [sys.executable, str(source / 'record_jetson_telemetry.py'),
                '--output-root', str(destination / 'telemetry'), '--interval-ms', '200', '--', *command]
        entry = dict(phase=phase, model=model, round=round_number,
                     directory=str(destination.relative_to(root)), command=full, started_epoch=time.time())
        write_json(destination / 'command.json', entry)
        print(json.dumps(dict(event='start', **{k: entry[k] for k in ('phase', 'model', 'round')})), flush=True)
        with (destination / 'run.log').open('w') as log:
            result = subprocess.run(full, stdout=log, stderr=subprocess.STDOUT)
        entry.update(returncode=result.returncode, finished_epoch=time.time())
        status['runs'].append(entry)
        write_json(root / 'suite.json', status)
        if result.returncode:
            raise RuntimeError(f'{phase} {model} round {round_number} failed; see {destination}')
        print(json.dumps(dict(event='complete', phase=phase, model=model, round=round_number)), flush=True)

    try:
        for index in range(3):
            for model in round_order(index):
                relative = Path(f'round-{index + 1}') / model
                destination = root / relative
                command = ['docker', 'run', '--rm', '--name', 'cleany-yoloe-scales', '--runtime', 'nvidia',
                    '--network', 'none', '-e', 'YOLO_AUTOINSTALL=false', '-e', 'YOLO_CONFIG_DIR=/tmp/Ultralytics',
                    '-e', 'PYTHONPATH=/workspace/cleany/ros2_ws/src/cleany_perception',
                    '-v', f'{repo}:/workspace/cleany:ro', '-v', f'{repo}:{repo}:ro',
                    '-v', f'{args.data_root}:{args.data_root}:ro', '-v', f'{args.model_root}:/models/yoloe:ro',
                    '-v', f'{root}:/evidence']
                for dataset in args.dataset_root:
                    command += ['-v', f'{dataset}:{dataset}:ro']
                command += ['--entrypoint', 'python3', args.image, '/evidence/source/benchmark_yoloe_dataset.py',
                    '--engine', f'/models/yoloe/real32/{model}/active/yoloe-26{model}-seg-real32.engine',
                    '--profile', '/workspace/cleany/ros2_ws/src/cleany_perception/config/yoloe_real32.yaml',
                    '--output', str(Path('/evidence') / relative / 'adapter'), '--rounds', '1',
                    '--passes', str(args.passes), '--warmup', '100', '--seed', str(20261028 + index * args.passes)]
                for dataset in args.dataset_root:
                    command += ['--dataset-root', str(dataset)]
                if index:
                    command += ['--skip-artifacts']
                measured(command, destination, 'adapter', model, index + 1)
        for index in range(3):
            for model in round_order(index):
                relative = Path('trtexec') / f'round-{index + 1}' / model
                first = Path('/evidence/round-1') / model / 'adapter'
                command = ['docker', 'run', '--rm', '--name', 'cleany-yoloe-scales-native', '--runtime', 'nvidia',
                    '--network', 'none', '-v', f'{root}:/evidence', '--entrypoint', '/usr/src/tensorrt/bin/trtexec',
                    args.image, f'--loadEngine={first}/engine.plan', f'--loadInputs=images:{first}/input.bin',
                    '--warmUp=5000', f'--duration={args.native_seconds}', '--iterations=100', '--infStreams=1',
                    '--percentile=50,95,99', f'--exportTimes=/evidence/{relative}/timings.json']
                measured(command, root / relative, 'trtexec', model, index + 1)
        status['status'] = 'complete'
    except BaseException as error:
        status.update(status='failed', error=f'{type(error).__name__}: {error}')
        raise
    finally:
        write_json(root / 'suite.json', status)


if __name__ == '__main__':
    main()
