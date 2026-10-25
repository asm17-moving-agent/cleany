"""Sample this attempt's processes and shared host load without changing scheduling."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time


def command(args: list[str]) -> str:
    result = subprocess.run(args, text=True, capture_output=True, timeout=5)
    return result.stdout.strip() if result.returncode == 0 else "unavailable: "+result.stderr.strip()


def processes() -> dict:
    result = {}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            stat = (path/'stat').read_text()
            fields = stat[stat.rindex(')')+2:].split()
            result[int(path.name)] = {'name': stat[stat.index('(')+1:stat.rindex(')')],
                'cpu_ticks': int(fields[11])+int(fields[12]), 'start_ticks': int(fields[19]),
                'rss_mib': int(fields[21])*os.sysconf('SC_PAGE_SIZE')/2**20,
                'threads': int(fields[17])}
        except (OSError, ValueError, IndexError):
            continue
    return result


def owned_processes(domain: int, cloned_envs: int = 0) -> dict:
    result = {}
    containers = [(role, f'cleany-e2e-{domain}-{role}') for role in ('isaac', 'ros', 'validate')]
    containers += [(role, f'cleany-e2e-{domain}-env_{i}-{role}')
                   for i in range(cloned_envs) for role in ('ros', 'validate')]
    for role, container in containers:
        output = command(['podman', 'top', container, 'hpid', 'comm'])
        for line in output.splitlines()[1:]:
            parts = line.split()
            if parts and parts[0].isdigit():
                result[int(parts[0])] = role
    return result


def scheduling(pid: int) -> dict:
    samples = []
    try:
        for task in (Path('/proc')/str(pid)/'task').iterdir():
            try:
                run, wait, switches = map(int, (task/'schedstat').read_text().split())
                samples.append({'tid': int(task.name), 'cpu_s': run/1e9,
                                'runqueue_wait_s': wait/1e9, 'timeslices': switches})
            except (OSError, ValueError):
                continue
    except OSError:
        pass
    return {'thread_schedstat': samples}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--timeout', type=float, required=True)
    parser.add_argument('--cloned-envs', type=int, default=0)
    args = parser.parse_args()
    deadline = time.monotonic()+args.timeout
    ticks = os.sysconf('SC_CLK_TCK')
    previous, last = processes(), time.monotonic()
    with (args.output/'evaluations/system-profile.jsonl').open('w', buffering=1) as output:
        while time.monotonic() < deadline and not (args.output/'stop').exists():
            time.sleep(2.)
            owned = owned_processes(args.domain, args.cloned_envs)
            current, now = processes(), time.monotonic()
            records = []
            for pid, item in current.items():
                old = previous.get(pid)
                if old and item['start_ticks'] == old['start_ticks']:
                    records.append({'pid': pid, **item, 'role': owned.get(pid, 'other'),
                        'cpu_percent_one_core': 100*(item['cpu_ticks']-old['cpu_ticks'])/ticks/(now-last)})
            task = [r for r in records if r['role'] != 'other']
            for record in task:
                record.update(scheduling(record['pid']))
            gpu = command(['nvidia-smi', '--query-gpu=utilization.gpu,utilization.memory,memory.used,memory.free,power.draw',
                           '--format=csv,noheader,nounits'])
            gpu_processes = command(['nvidia-smi', 'pmon', '-s', 'um', '-c', '1'])
            output.write(json.dumps({'wall_time': time.time(), 'sample_interval_s': now-last,
                'process_sample_wall_time': time.time()-(time.monotonic()-now),
                'cpu_count': os.cpu_count(), 'load_average': os.getloadavg(),
                'cpu_pressure': Path('/proc/pressure/cpu').read_text(),
                'memory_pressure': Path('/proc/pressure/memory').read_text(),
                'task_processes': task,
                'other_cpu_total_percent_one_core': sum(r['cpu_percent_one_core'] for r in records if r['role']=='other'),
                'other_top_cpu': sorted((r for r in records if r['role']=='other'),
                                        key=lambda r: r['cpu_percent_one_core'], reverse=True)[:8],
                'gpu_columns': ['utilization_gpu_percent', 'utilization_memory_percent', 'used_mib', 'free_mib', 'power_w'],
                'gpu_csv': gpu, 'gpu_per_process_pmon': gpu_processes})+'\n')
            previous, last = current, now


if __name__ == '__main__':
    main()
