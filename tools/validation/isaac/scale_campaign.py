"""Compare CPU and GPU PhysX at fixed cloned environment counts."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from build import SOURCE, source_digest
from campaign import summarize
from run import write


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('output', 'build-root', 'cache-root', 'isaaclab-root', 'isaaclab-deps', 'facility-map'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--counts', nargs='+', type=int, default=[4, 8])
    parser.add_argument('--targets', nargs='+', default=['seat-12', 'seat-03', 'seat-27', 'seat-46'])
    parser.add_argument('--domain', type=int, default=177)
    parser.add_argument('--timeout', type=float, default=2400)
    args = parser.parse_args()
    if args.output.exists() or any(n < 1 for n in args.counts):
        parser.error('Use a new output path and positive environment counts')
    args.output.mkdir(parents=True)
    tools = Path(__file__).parent
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in tools.glob('*.py')}
    digest = source_digest(SOURCE)
    report = {'status': 'running', 'source_digest': digest, 'tool_hashes': hashes,
              'policy': 'One batch per CPU/GPU/count combination; preserve every failure; fixed source and settings',
              'attempts': []}
    write(args.output/'campaign.json', report)
    for count in args.counts:
        for device in ('cpu', 'cuda:0'):
            assert source_digest(SOURCE) == digest
            assert all(hashlib.sha256((tools/name).read_bytes()).hexdigest() == h for name, h in hashes.items())
            label = ('gpu' if device.startswith('cuda') else 'cpu')+f'-{count:02d}'
            output = args.output/label
            targets = [args.targets[i % len(args.targets)] for i in range(count)]
            command = [sys.executable, str(tools/'run_cloned.py'), '--output', str(output),
                       '--physics-device', device, '--domain', str(args.domain),
                       '--timeout', str(args.timeout), '--targets', *targets]
            for name in ('build_root', 'cache_root', 'isaaclab_root', 'isaaclab_deps', 'facility_map'):
                command += ['--'+name.replace('_', '-'), str(getattr(args, name).resolve())]
            report['active'] = {'label': label, 'command': command}
            write(args.output/'campaign.json', report)
            print('START', label, flush=True)
            with (args.output/(label+'-runner.log')).open('w') as log:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
            provenance = json.loads((output/'provenance/run.json').read_text())
            codes = provenance.get('validator_returncodes', [])
            runs = [summarize(output/f'env_{i}', codes[i] if len(codes) == count else result.returncode)
                    for i in range(count)]
            entry = {'label': label, 'physics_device': device, 'num_envs': count, 'targets': targets,
                     'run': str(output.resolve()), 'command': command, 'returncode': result.returncode,
                     'environments': runs, 'passed_without_recovery': all(r['passed_without_recovery'] for r in runs)}
            report['attempts'].append(entry)
            report.pop('active', None)
            write(args.output/'campaign.json', report)
            print('FINISH', label, entry['passed_without_recovery'], flush=True)
    report['status'] = 'complete'
    report['all_passed_without_recovery'] = all(r['passed_without_recovery'] for r in report['attempts'])
    write(args.output/'campaign.json', report)
    if not report['all_passed_without_recovery']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
