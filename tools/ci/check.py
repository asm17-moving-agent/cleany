#!/usr/bin/env python3
"""검사 결과와 제한시간 진단을 보존하고 필수 검사 누락을 실패로 처리합니다."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time


def process_tree(root: int) -> list[dict]:
    rows = []
    for entry in Path('/proc').glob('[0-9]*'):
        try:
            stat = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            rows.append({'pid': int(entry.name), 'ppid': int(stat[1]),
                         'start': stat[19], 'state': stat[0],
                         'command': (entry/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')})
        except (OSError, IndexError, ValueError):
            continue
    owned = {root}
    while True:
        expanded = owned | {r['pid'] for r in rows if r['ppid'] in owned}
        if expanded == owned:
            return [r for r in rows if r['pid'] in owned]
        owned = expanded


def capture(rows: list[dict], output: Path, native: bool = True) -> None:
    """검사 프로세스 계보만 수집하며 debugger 실패도 파일로 남깁니다."""
    output.mkdir(parents=True, exist_ok=True)
    (output/'processes.json').write_text(json.dumps(rows, indent=2))
    for row in rows:
        pid = row['pid']
        try:
            (output/f'{pid}-status.txt').write_text((Path('/proc')/str(pid)/'status').read_text())
            exe = (Path('/proc')/str(pid)/'exe').resolve().name
        except OSError:
            continue
        # Python은 pytest faulthandler가 현재 검사 스택을 기록합니다.
        if not native or exe.startswith(('python', 'bash', 'sh', 'gdb')):
            continue
        if not shutil.which('gdb'):
            (output/'gdb-unavailable.txt').write_text('gdb is unavailable; process state only\n')
            continue
        with (output/f'{pid}-threads.txt').open('w') as stream:
            try:
                subprocess.run(['gdb', '-q', '-batch', '-ex', 'set pagination off',
                    '-ex', 'thread apply all bt', '-p', str(pid)], stdout=stream,
                    stderr=subprocess.STDOUT, timeout=5, check=False)
            except subprocess.TimeoutExpired:
                stream.write('\nDebugger exceeded 5 seconds\n')


def signal_owned(identities: dict, signum: int) -> None:
    """새 session으로 분리된 자식도 정리하되 PID가 재사용되었으면 건드리지 않습니다."""
    for pid, started in identities:
        try:
            stat = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()
            if stat[19] == started:
                os.kill(pid, signum)
        except (OSError, IndexError):
            continue


def execute(name: str, kind: str, command: list[str], root: Path,
            timeout: float, diagnostic_after: float) -> int:
    output = root/name
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    result = {'name': name, 'kind': kind, 'command': shlex.join(command),
              'cwd': str(Path.cwd()), 'status': 'running',
              'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
              'dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], text=True))}
    record = output/'result.json'
    record.write_text(json.dumps(result, indent=2))
    env = dict(os.environ)
    # Humble의 pytest 6은 미리 활성화된 faulthandler가 있으면 timeout 설정을 무시합니다.
    env.pop('PYTHONFAULTHANDLER', None)
    env['PYTEST_ADDOPTS'] = env.get('PYTEST_ADDOPTS', '') + ' --durations=15 -o faulthandler_timeout=40'
    seen: dict[tuple, float] = {}
    captured: set[tuple] = set()
    proc = None
    reader = None
    returncode = 1
    status = 'infrastructure_error'
    with (output/'output.log').open('w', buffering=1) as log:
        try:
            proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, errors='replace', env=env, start_new_session=True)
            def forward():
                for line in proc.stdout:
                    log.write(line)
                    print(line, end='', flush=True)
            reader = threading.Thread(target=forward, daemon=True)
            reader.start()
            while proc.poll() is None:
                now = time.monotonic()
                rows = process_tree(proc.pid)
                for row in rows:
                    key = (row['pid'], row['start'])
                    seen.setdefault(key, now)
                    if now-seen[key] >= diagnostic_after and key not in captured:
                        captured.add(key)
                        capture([row], output/f'diagnostic-{row["pid"]}')
                if now-started >= timeout:
                    capture(rows, output/'timeout', native=False)
                    status = 'timeout'
                    returncode = 124
                    break
                time.sleep(0.1)
            else:
                returncode = proc.returncode
                status = 'passed' if returncode == 0 else 'failed'
        except (OSError, KeyboardInterrupt) as error:
            log.write(f'{type(error).__name__}: {error}\n')
            status = 'interrupted' if isinstance(error, KeyboardInterrupt) else 'infrastructure_error'
            returncode = 130 if isinstance(error, KeyboardInterrupt) else 1
        finally:
            if proc is not None:
                for row in process_tree(proc.pid):
                    seen.setdefault((row['pid'], row['start']), time.monotonic())
                # ROS launch가 만든 별도 session과 이 검사 전용 group을 함께 정리합니다.
                signal_owned(seen, signal.SIGTERM)
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
                except ProcessLookupError:
                    pass
                signal_owned(seen, signal.SIGKILL)
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait()
                if reader:
                    reader.join(timeout=2)
    content = (output/'output.log').read_text()
    if status == 'failed' and re.search(r'\*\*\*Timeout|TimeoutExpired', content):
        status = 'timeout'
    result.update(status=status, returncode=returncode, elapsed_seconds=round(time.monotonic()-started, 3),
                  failures=[line[-500:] for line in content.splitlines()
                            if re.search(r'FAILED|ERROR|\*\*\*Timeout|missing_result', line)][-30:])
    record.write_text(json.dumps(result, indent=2))
    return returncode if returncode >= 0 else 128-returncode


def summarize(root: Path, expected: list[str]) -> tuple[str, bool]:
    lines = ['## CI 검사 결과', '', '| 검사 | 구분 | 결과 | 시간 |', '|---|---|---|---|']
    ok = True
    details = []
    for name in expected:
        path = root/name/'result.json'
        try:
            item = json.loads(path.read_text()) if path.exists() else {'status': 'missing'}
            if not isinstance(item, dict) or 'status' not in item:
                item = {'status': 'invalid_result'}
        except (OSError, ValueError):
            item = {'status': 'invalid_result'}
        status = item['status']
        ok &= status == 'passed'
        lines.append(f'| {name} | {item.get("kind", "—")} | {status} | {item.get("elapsed_seconds", "—")} s |')
        if status != 'passed':
            details.extend(['', f'**{name}**', '', '```text', *item.get('failures', []),
                          f'cwd: {item.get("cwd", "unknown")}',
                          item.get('command', '검사 미실행 또는 결과 파일 누락'), '```', ''])
    lines.extend(details)
    manifests = [root/group/'packages.json' for group in ('navigation', 'manipulation', 'workspace-rest')]
    if all(p.exists() for p in manifests):
        records = [json.loads(p.read_text()) for p in manifests]
        all_packages = set(records[0]['all'])
        exclusions = json.loads(Path(__file__).with_name('workspace_test_exclusions.json').read_text())
        excluded = all_packages & set(exclusions)
        selected = [p for r in records for p in r['selected']]
        covered = (all(set(r['all']) == all_packages and set(r.get('excluded', [])) == excluded
                       for r in records)
                   and set(selected) == all_packages - excluded
                   and len(selected) == len(set(selected)))
        ok &= covered
        lines.extend(['', f'Workspace 검사 분할: {"검사 대상 패키지 포함, 중복 없음" if covered else "누락 또는 중복 오류"}'])
        for package in sorted(excluded):
            lines.append(f'CI 제외: {package} — {exclusions[package]}')
    elif any(group in expected for group in ('navigation', 'manipulation', 'workspace-rest')):
        ok = False
        lines.extend(['', 'Workspace 검사 목록 누락: 전체 검사 범위를 확인할 수 없습니다.'])
    lines.extend(['', '상세 로그·스택·재현 명령은 ci-results artifact의 checks/에 있습니다.'])
    return '\n'.join(lines)+'\n', bool(ok)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    sub = parser.add_subparsers(dest='action', required=True)
    run = sub.add_parser('run')
    run.add_argument('name')
    run.add_argument('--kind', choices=['environment', 'build', 'contract', 'integration', 'simulation'], required=True)
    run.add_argument('--timeout', type=float, default=600)
    run.add_argument('--diagnostic-after', type=float, default=45)
    report = sub.add_parser('summary')
    report.add_argument('expected', nargs='+')
    args, command = parser.parse_known_args()
    if args.action == 'summary':
        if command: parser.error('Unexpected arguments')
        text, ok = summarize(args.root, args.expected)
        print(text)
        if os.environ.get('GITHUB_STEP_SUMMARY'):
            with open(os.environ['GITHUB_STEP_SUMMARY'], 'a') as stream: stream.write(text)
        return 0 if ok else 1
    if not re.fullmatch(r'[a-z][a-z0-9-]*', args.name): parser.error('Invalid check name')
    if len(command) < 2 or command[0] != '--': parser.error('Command must follow --')
    if args.timeout <= 0 or args.diagnostic_after <= 0: parser.error('Time limits must be positive')
    return execute(args.name, args.kind, command[1:], args.root, args.timeout, args.diagnostic_after)


if __name__ == '__main__':
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    sys.exit(main())
