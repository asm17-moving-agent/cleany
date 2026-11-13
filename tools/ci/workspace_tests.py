#!/usr/bin/env python3
"""패키지별 통신과 파일 경로를 격리해 전체 workspace 검사를 병렬 실행합니다."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

GROUPS = ('navigation', 'manipulation', 'workspace-rest')
SCRIPT = Path(__file__).resolve()
EXCLUSIONS = json.loads(SCRIPT.with_name('workspace_test_exclusions.json').read_text())


def group_packages(names: list[str], group: str) -> list[str]:
    def owner(name):
        if name == 'cleany_navigation':
            return 'navigation'
        if name in ('cleany_manipulation_bt', 'cleany_skill_executor'):
            return 'manipulation'
        return 'workspace-rest'
    return [name for name in names if owner(name) == group and name not in EXCLUSIONS]


def package_environment(root: Path, package: str, index: int) -> dict[str, str]:
    if not 0 <= index < 60:
        raise ValueError('Package domains must stay within 40..99')
    directory = root/'test-environments'/package
    for child in ('tmp', 'ros'):
        (directory/child).mkdir(parents=True, exist_ok=True)
    domain = str(40 + index)
    environment = {
        'ROS_DOMAIN_ID': domain, 'CLEANY_TEST_ROS_DOMAIN_ID': domain,
        'BASE_TEST_ROS_DOMAIN_ID': domain, 'ROS_LOCALHOST_ONLY': '1',
        'ROS_HOME': str(directory/'ros'), 'TMPDIR': str(directory/'tmp'),
        'IGN_PARTITION': f'cleany-ci-{package}', 'GZ_PARTITION': f'cleany-ci-{package}',
        'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1',
        'CTEST_PARALLEL_LEVEL': '1',
    }
    if package == 'cleany_system_supervisor':
        # 각 사례에 별도 domain을 쓰는 기존 fixture의 범위는 다른 패키지와 분리합니다.
        environment.update(SUPERVISOR_GRAPH_DOMAIN_BASE='190', SUPERVISOR_E2E_DOMAIN_BASE='200')
    return environment


def test_package(root: Path, package: str) -> int:
    status = subprocess.run(['colcon', '--log-base', str(root/'log'/package), 'test',
        '--build-base', str(root/'build'), '--install-base', str(root/'install'),
        '--executor', 'sequential', '--return-code-on-test-failure', '--packages-select', package]).returncode
    result = subprocess.run(['colcon', 'test-result', '--test-result-base',
                             str(root/'build'/package), '--verbose']).returncode
    return status or result


def run_package(root: Path, package: str, index: int, group: str, started: float) -> dict:
    environment = package_environment(root, package, index)
    output = root/'checks'/group/'packages'
    name = package.replace('_', '-')
    directory = output/name
    directory.mkdir(parents=True, exist_ok=True)
    record = directory/'result.json'
    record.unlink(missing_ok=True)
    command = [sys.executable, str(SCRIPT.with_name('check.py')), '--root', str(output),
        'run', name, '--kind', 'contract', '--timeout', '600', '--',
        sys.executable, str(SCRIPT), '--package', package]
    begin = time.monotonic() - started
    print(f'Starting {package} (ROS domain {environment["ROS_DOMAIN_ID"]})', flush=True)
    with (directory/'driver.log').open('w') as log:
        code = subprocess.run(command, env={**os.environ, **environment},
                              stdout=log, stderr=subprocess.STDOUT).returncode
    result = json.loads(record.read_text()) if record.exists() else {'status': 'missing'}
    passed = code == 0 and result['status'] == 'passed'
    print(f'{"Passed" if passed else "FAILED"} {package}', flush=True)
    if not passed:
        print(f'::error title={package}::Package tests failed; see {directory}', flush=True)
        failure_log = directory/'output.log'
        if not failure_log.exists():
            failure_log = directory/'driver.log'
        print('\n'.join(failure_log.read_text().splitlines()[-35:]), flush=True)
    return {'package': package, 'status': 'passed' if passed else 'failed',
            'started_seconds': round(begin, 3), 'environment': environment,
            'finished_seconds': round(time.monotonic() - started, 3)}


def run_groups(root: Path, names: list[str], groups: tuple[str, ...], workers: int,
               runner=run_package) -> int:
    started = time.monotonic()
    selected = {group: group_packages(names, group) for group in groups}
    # 준비 실패도 이전 실행의 성공 결과를 남기지 않도록 먼저 표시합니다.
    for group, packages in selected.items():
        output = root/'checks'/group
        output.mkdir(parents=True, exist_ok=True)
        (output/'packages.json').write_text(json.dumps({'all': names, 'selected': packages,
            'excluded': [name for name in names if name in EXCLUSIONS]}, indent=2))
        (output/'result.json').write_text(json.dumps({'status': 'running', 'kind': 'contract'}))
        if not packages:
            raise RuntimeError(f'No packages selected for {group}')
    if len(names) > 60:
        raise ValueError('At most 60 isolated package domains are available')
    tasks = [(package, names.index(package), group)
             for group in groups for package in selected[group]]
    # 가장 긴 조작 검사를 먼저 배정해 마지막에 큰 검사 하나만 남는 일을 줄입니다.
    tasks.sort(key=lambda item: (item[2] != 'manipulation', item[0]))
    results = {group: [] for group in groups}
    with ThreadPoolExecutor(max_workers=workers) as pool:
        pending = {pool.submit(runner, root, package, index, group, started): (package, group)
                   for package, index, group in tasks}
        for future in as_completed(pending):
            package, group = pending[future]
            try:
                result = future.result()
            except Exception as error:
                result = {'package': package, 'status': 'infrastructure_error', 'error': str(error)}
                print(f'::error title={package}::{error}', flush=True)
            results[group].append(result)
    elapsed = round(time.monotonic() - started, 3)
    failed = False
    for group, records in results.items():
        passed = len(records) == len(selected[group]) and all(r['status'] == 'passed' for r in records)
        failed |= not passed
        timings = [r for r in records if 'finished_seconds' in r]
        span = round(max(r['finished_seconds'] for r in timings) -
                     min(r['started_seconds'] for r in timings), 3) if timings else elapsed
        result = {'name': group, 'kind': 'contract', 'status': 'passed' if passed else 'failed',
                  'cwd': str(Path.cwd()), 'elapsed_seconds': span, 'packages': records,
                  'command': shlex.join([sys.executable, str(SCRIPT), group, '--workers', str(workers)]),
                  'failures': [r['package'] + ': ' + r['status'] for r in records if r['status'] != 'passed']}
        (root/'checks'/group/'result.json').write_text(json.dumps(result, indent=2))
    schedule = root/'checks/workspace'
    schedule.mkdir(parents=True, exist_ok=True)
    (schedule/'schedule.json').write_text(json.dumps({'workers': workers, 'elapsed_seconds': elapsed,
        'packages': [record for records in results.values() for record in records]}, indent=2))
    return int(failed)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('group', nargs='?', default='all', choices=['all', *GROUPS])
    parser.add_argument('--workers', type=int, default=min(4, os.cpu_count() or 1))
    parser.add_argument('--package')
    args = parser.parse_args()
    if not 1 <= args.workers <= 4:
        parser.error('--workers must be between 1 and 4')
    root = Path(os.environ['CLEANY_BUILD_ROOT']).resolve()
    if args.package:
        return test_package(root, args.package)
    names = sorted(subprocess.check_output(['colcon', 'list', '--names-only'], text=True).splitlines())
    return run_groups(root, names, GROUPS if args.group == 'all' else (args.group,), args.workers)


if __name__ == '__main__':
    raise SystemExit(main())
