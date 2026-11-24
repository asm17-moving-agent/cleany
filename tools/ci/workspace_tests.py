#!/usr/bin/env python3
"""패키지별 통신과 파일 경로를 격리해 전체 workspace 검사를 병렬 실행합니다."""
from __future__ import annotations

import argparse
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

GROUPS = ('navigation', 'manipulation', 'workspace-rest')
NAVIGATION_RUNTIME = 'ros-runtime'
NAVIGATION_RUNTIME_DOMAIN_BASE = 30
# 최근 CI에서 오래 걸린 패키지를 먼저 배정해 짧은 검사로 남는 자리를 채웁니다.
PACKAGE_PRIORITY = (
    'cleany_skill_executor',
    'cleany_manipulation_bt',
    'cleany_navigation',
    'cleany_description',
    'cleany_moveit_capabilities',
    'cleany_gazebo_sim',
    'cleany_system_supervisor',
    'cleany_moveit_config',
    'cleany_mujoco_sim',
    'cleany_robot_interface',
)
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


def isolated_environment(root: Path, name: str, domain: int) -> dict[str, str]:
    directory = root/'test-environments'/name
    for child in ('tmp', 'ros'):
        (directory/child).mkdir(parents=True, exist_ok=True)
    return {
        'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1',
        'ROS_HOME': str(directory/'ros'), 'TMPDIR': str(directory/'tmp'),
        'IGN_PARTITION': f'cleany-ci-{name}', 'GZ_PARTITION': f'cleany-ci-{name}',
        'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1',
        'CTEST_PARALLEL_LEVEL': '1',
    }


def package_environment(root: Path, package: str, index: int) -> dict[str, str]:
    if not 0 <= index < 60:
        raise ValueError('Package domains must stay within 40..99')
    environment = isolated_environment(root, package, 40 + index)
    domain = environment['ROS_DOMAIN_ID']
    environment.update(CLEANY_TEST_ROS_DOMAIN_ID=domain, BASE_TEST_ROS_DOMAIN_ID=domain)
    if package == 'cleany_base_driver':
        environment['BASE_EKF_TEST_ROS_DOMAIN_ID'] = '26'
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


def run_navigation_runtime(root: Path, started: float) -> dict:
    environment = isolated_environment(root, NAVIGATION_RUNTIME, NAVIGATION_RUNTIME_DOMAIN_BASE)
    environment['CLEANY_TEST_ROS_DOMAIN_BASE'] = str(NAVIGATION_RUNTIME_DOMAIN_BASE)
    command = [sys.executable, str(SCRIPT.with_name('check.py')), '--root', str(root/'checks'),
               'run', NAVIGATION_RUNTIME, '--kind', 'integration', '--timeout', '600', '--',
               'make', '-o', 'build-navigation', 'test-navigation-runtime',
               f'NAVIGATION_BUILD_BASE={root/"build"}',
               f'NAVIGATION_INSTALL_BASE={root/"install"}',
               f'NAVIGATION_LOG_BASE={root/"log"}']
    begin = time.monotonic() - started
    print('Starting common navigation runtime', flush=True)
    status = subprocess.run(command, cwd=SCRIPT.parents[2], env={**os.environ, **environment}).returncode
    return {'name': NAVIGATION_RUNTIME, 'status': 'failed' if status else 'passed',
            'started_seconds': round(begin, 3), 'environment': environment,
            'finished_seconds': round(time.monotonic() - started, 3)}


def run_groups(root: Path, names: list[str], groups: tuple[str, ...], workers: int,
               runner=run_package, *,
               navigation_runner: Callable[[Path, float], dict] | None = None) -> int:
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
    priority = {package: rank for rank, package in enumerate(PACKAGE_PRIORITY)}
    tasks.sort(key=lambda item: (priority.get(item[0], len(priority)), item[0]))
    results = {group: [] for group in groups}
    runtime = None
    if navigation_runner is not None:
        (root/'checks'/NAVIGATION_RUNTIME/'result.json').unlink(missing_ok=True)
    # 실시간 mock MCU watchdog 검사가 끝난 뒤 다른 ROS 검사를 시작합니다.
    base = [task for task in tasks if task[0] == 'cleany_base_driver']
    remaining = [task for task in tasks if task[0] != 'cleany_base_driver']
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for batch in (base, remaining):
            pending = {pool.submit(runner, root, package, index, group, started): (package, group)
                       for package, index, group in batch}
            if batch is remaining and navigation_runner is not None:
                pending[pool.submit(navigation_runner, root, started)] = (NAVIGATION_RUNTIME, None)
            for future in as_completed(pending):
                package, group = pending[future]
                try:
                    result = future.result()
                except Exception as error:
                    result = {'package': package, 'status': 'infrastructure_error', 'error': str(error)}
                    print(f'::error title={package}::{error}', flush=True)
                if group is None:
                    runtime = result
                else:
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
    schedule_result = {'workers': workers, 'elapsed_seconds': elapsed,
                       'packages': [record for records in results.values() for record in records]}
    if runtime is not None:
        schedule_result[NAVIGATION_RUNTIME] = runtime
        failed |= runtime['status'] != 'passed'
    (schedule/'schedule.json').write_text(json.dumps(schedule_result, indent=2))
    return int(failed)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('group', nargs='?', default='all', choices=['all', *GROUPS])
    parser.add_argument('--workers', type=int, default=min(4, os.cpu_count() or 1))
    parser.add_argument('--package')
    parser.add_argument('--with-navigation-runtime', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.workers <= 4:
        parser.error('--workers must be between 1 and 4')
    if args.package and args.with_navigation_runtime:
        parser.error('--with-navigation-runtime cannot be combined with --package')
    root = Path(os.environ['CLEANY_BUILD_ROOT']).resolve()
    if args.package:
        return test_package(root, args.package)
    names = sorted(subprocess.check_output(['colcon', 'list', '--names-only'], text=True).splitlines())
    return run_groups(root, names, GROUPS if args.group == 'all' else (args.group,), args.workers,
                      navigation_runner=run_navigation_runtime if args.with_navigation_runtime else None)


if __name__ == '__main__':
    raise SystemExit(main())
