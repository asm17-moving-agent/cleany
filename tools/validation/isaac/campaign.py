"""Run a fixed multi-seat matrix, retaining every attempt including failures."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import time

from build import SOURCE, source_digest
from run import write


def read(path: Path) -> dict:
    return json.loads(path.read_text()) if path.exists() else {}


def summarize(output: Path, code: int) -> dict:
    run = read(output / 'provenance/run.json')
    mission = read(output / 'evaluations/mission.json')
    result = mission.get('mission_result', {})
    log_file = output / 'logs/mission.log'
    log = log_file.read_text() if log_file.exists() else ''
    counts = {
        'planner_failures': len(re.findall(r'failed to create plan', log)),
        'recovery_behaviors': len(re.findall(r'Running (?:spin|backup|wait|drive_on_heading)', log)),
        'costmap_clears': len(re.findall(r'Received request to clear', log)),
    }
    states = []
    for event in mission.get('states', []):
        if not states or event['state'] != states[-1]:
            states.append(event['state'])
    entry = {'run': str(output), 'returncode': code, 'run_status': run.get('status'),
             'mission_started': bool(mission.get('offer', {}).get('accepted')) or 'FSM NAVIGATE_TO_TARGET' in log,
             'mission_outcome': result.get('outcome'), 'failure_code': result.get('failure_code'),
             'error': mission.get('error') or run.get('error'),
             'return_result': result.get('return_result'), 'home_error_m': mission.get('home_error_m'),
             'states': states, 'feedback_recoveries': mission.get('navigation_recoveries', {}),
             'elapsed_s': run.get('elapsed_s'), **counts}
    entry['publication_hz_sim'] = {name: value['received_hz_sim']
                                  for name, value in mission.get('publication_timing', {}).items()}
    passed = code == 0 and run.get('status') == 'passed' and mission.get('passed') is True
    entry['passed_without_recovery'] = passed and result.get('outcome') == 'SUCCESS' and not any(counts.values())
    if entry['passed_without_recovery']:
        audit = subprocess.run([sys.executable, str(Path(__file__).with_name('audit.py')),
                                str(output), '--output', str(output/'evaluations/journal-audit.json')],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        entry['journal_audit_passed'] = audit.returncode == 0
        entry['passed_without_recovery'] &= entry['journal_audit_passed']
        if audit.returncode:
            entry['audit_error'] = audit.stderr
    return entry


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--build-root', type=Path, required=True)
    parser.add_argument('--cache-root', type=Path, required=True)
    parser.add_argument('--facility-map', type=Path, required=True)
    parser.add_argument('--targets', nargs='+', default=['seat-12', 'seat-03', 'seat-27', 'seat-46'])
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--timeout', type=float, default=1800.)
    parser.add_argument('--profile', action='store_true')
    parser.add_argument('--publication-profile', choices=['legacy', 'split'], default='split')
    parser.add_argument('--physics-sync', choices=['usd', 'fabric'], default='fabric')
    args = parser.parse_args()
    if args.repeats < 2 or len(set(args.targets)) != len(args.targets):
        parser.error('Use at least two repeats and unique target IDs')
    if args.output.exists():
        parser.error('Use a new output directory to preserve earlier campaigns')
    output = args.output.resolve()
    output.mkdir(parents=True)
    source_hash = source_digest(SOURCE)
    tools_root = Path(__file__).parent
    tool_hashes = {str(p.relative_to(tools_root)): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in tools_root.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    matrix = [{'repeat': repeat, 'target_id': target}
              for repeat in range(1, args.repeats+1) for target in args.targets]
    report = {'status': 'running', 'world': 'facility18', 'scope': 'actual Mission/Nav2; mock perception/planning/manipulation',
              'source_digest': source_hash, 'tool_hashes': tool_hashes,
              'build_manifest': read(args.build_root/'manifest.json'), 'matrix': matrix, 'attempts': [],
              'publication_profile': args.publication_profile, 'physics_sync': args.physics_sync,
              'profile': args.profile,
              'policy': 'Fresh simulator and Mission per attempt; fixed configuration; retain all failures; no retries hidden'}
    write(output/'campaign.json', report)
    for index, cell in enumerate(matrix, 1):
        if source_digest(SOURCE) != source_hash or any(
                hashlib.sha256((tools_root/name).read_bytes()).hexdigest() != digest for name, digest in tool_hashes.items()):
            report['status'] = 'stopped_source_changed'
            write(output/'campaign.json', report)
            raise RuntimeError('Source changed during fixed campaign')
        attempt = output / f"{index:02d}-{cell['target_id']}-r{cell['repeat']}"
        command = [sys.executable, str(tools_root/'run.py'), '--world', 'facility18',
                   '--target-id', cell['target_id'], '--facility-map', str(args.facility_map.resolve()),
                   '--build-root', str(args.build_root.resolve()), '--cache-root', str(args.cache_root.resolve()),
                   '--output', str(attempt), '--timeout', str(args.timeout), '--record-motion', '--require-no-recovery',
                   '--publication-profile', args.publication_profile, '--physics-sync', args.physics_sync]
        if args.profile:
            command.append('--profile')
        report['active'] = {**cell, 'run': str(attempt), 'command': command}
        write(output/'campaign.json', report)
        print(f"START {index}/{len(matrix)} {cell}", flush=True)
        with (output/f'{index:02d}-runner.log').open('w') as log:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        entry = {**cell, **summarize(attempt, result.returncode)}
        if args.profile and entry['passed_without_recovery']:
            analysis = subprocess.run([sys.executable, str(tools_root/'analyze_profile.py'), '--run', str(attempt),
                                       '--output', str(attempt/'evaluations/profile-summary.json')], capture_output=True, text=True)
            entry['profile_analysis_passed'] = analysis.returncode == 0
            if analysis.returncode:
                entry['profile_analysis_error'] = analysis.stderr
        report['attempts'].append(entry)
        report['completed'] = len(report['attempts'])
        report['passed_without_recovery'] = sum(e['passed_without_recovery'] for e in report['attempts'])
        report['by_target'] = {target: dict(Counter('passed' if e['passed_without_recovery'] else 'failed'
                              for e in report['attempts'] if e['target_id'] == target)) for target in args.targets}
        report.pop('active', None)
        write(output/'campaign.json', report)
        print(f"FINISH {index}/{len(matrix)} {json.dumps(entry)}", flush=True)
    report['status'] = 'complete'
    report['all_passed_without_recovery'] = report['passed_without_recovery'] == len(matrix)
    write(output/'campaign.json', report)
    print(json.dumps({'status': report['status'], 'by_target': report['by_target']}), flush=True)
    if not report['all_passed_without_recovery']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
