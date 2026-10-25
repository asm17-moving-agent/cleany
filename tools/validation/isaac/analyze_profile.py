"""Summarize measured adapter stages and process load for a retained mission."""
from __future__ import annotations

import argparse
import bisect
from collections import defaultdict
import json
from pathlib import Path


def read_lines(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines()]


def analyze(run: Path) -> dict:
    mission = json.loads((run/'evaluations/mission.json').read_text())
    events = mission['states']
    times = [event['wall_time'] for event in events]

    def phase(timestamp: float) -> str:
        index = bisect.bisect_right(times, timestamp)-1
        return events[index]['state'] if index >= 0 else 'PRE_MISSION'

    groups = defaultdict(list)
    windows = read_lines(run/'evaluations/loop-profile.jsonl')
    native_path = run/'evaluations/native-profile.json'
    native = json.loads(native_path.read_text()) if native_path.exists() else {}
    for window in windows:
        start, end = phase(window['wall_start']), phase(window['wall_end'])
        captured = (native.get('wall_start') is not None and native.get('wall_end') is not None
                    and window['wall_start'] < native['wall_end'] and window['wall_end'] > native['wall_start'])
        groups['NATIVE_CAPTURE' if captured else start if start == end else 'TRANSITION'].append(window)
    groups['ALL'] = windows
    groups['NAVIGATION'] = groups['NAVIGATE_TO_TARGET']+groups['RETURN_HOME']
    summary = {}
    for name, rows in groups.items():
        if not rows:
            continue
        wall = sum(row['wall_s'] for row in rows)
        simulation = sum(row['simulation_s'] for row in rows)
        stages = {}
        for stage in dict.fromkeys(stage for row in rows for stage in row['stages']):
            values = [row['stages'][stage] for row in rows if stage in row['stages']]
            total = sum(value['wall']['sum_s'] for value in values)
            cpu = sum(value['thread_cpu']['sum_s'] for value in values)
            count = sum(value['wall']['count'] for value in values)
            stages[stage] = {'wall_s': total, 'main_thread_cpu_s': cpu, 'wall_percent': 100*total/wall,
                'calls': count, 'mean_wall_ms': 1000*total/count, 'mean_main_thread_cpu_ms': 1000*cpu/count,
                'max_wall_ms': 1000*max(value['wall']['max_s'] for value in values)}
        summary[name] = {'windows': len(rows), 'wall_s': wall, 'simulation_s': simulation,
            'rtf': simulation/wall, 'effective_physics_hz': sum(row['loops'] for row in rows)/wall,
            'compute_overrun_percent': 100*sum(row['compute_overrun_loops'] for row in rows)/sum(row['loops'] for row in rows),
            'requested_sleep_s': sum(row['requested_sleep_s'] for row in rows),
            'unattributed_wall_s': sum(row['unattributed_wall_s'] for row in rows), 'stages': stages}
    samples = read_lines(run/'evaluations/system-profile.jsonl')
    nav = [row for row in samples if phase(row['process_sample_wall_time']) in ('NAVIGATE_TO_TARGET', 'RETURN_HOME')]
    processes = defaultdict(list)
    for row in nav:
        for process in row['task_processes']:
            processes[(process['pid'], process['role'], process['name'])].append((row, process))
    identities_path = run/'evaluations/process-identities.json'
    identities = json.loads(identities_path.read_text()) if identities_path.exists() else {}
    process_summary = []
    for (pid, role, name), rows in processes.items():
        duration = sum(row['sample_interval_s'] for row, _ in rows)
        main_wait = main_cpu = main_span = 0.
        # Sum only adjacent samples inside each navigation leg; exclude the mock-work gap.
        for (before, a), (after, b) in zip(rows, rows[1:]):
            span = after['process_sample_wall_time']-before['process_sample_wall_time']
            if span > 2*after['sample_interval_s']:
                continue
            first = next((t for t in a['thread_schedstat'] if t['tid']==pid), None)
            last = next((t for t in b['thread_schedstat'] if t['tid']==pid), None)
            if first and last:
                main_wait += last['runqueue_wait_s']-first['runqueue_wait_s']
                main_cpu += last['cpu_s']-first['cpu_s']
                main_span += span
        process_summary.append({'pid': pid, 'role': role, 'name': name,
            'identity': identities.get(str(pid)), 'samples': len(rows),
            'mean_cpu_percent_one_core': sum(p['cpu_percent_one_core']*r['sample_interval_s'] for r,p in rows)/duration,
            'peak_cpu_percent_one_core': max(p['cpu_percent_one_core'] for _,p in rows),
            'max_rss_mib': max(p['rss_mib'] for _,p in rows),
            'main_thread_runqueue_wait_s': main_wait, 'main_thread_cpu_s': main_cpu,
            'main_thread_observed_wall_s': main_span})
    duration = sum(row['sample_interval_s'] for row in nav)
    role_cpu = {role: sum(sum(p['cpu_percent_one_core'] for p in row['task_processes'] if p['role']==role)*row['sample_interval_s']
                         for row in nav)/duration for role in ('isaac','ros','validate')}
    gpu = [float(row['gpu_csv'].split(',')[0]) for row in nav if not row['gpu_csv'].startswith('unavailable')]
    scene = json.loads((run/'config/room.json').read_text()) if (run/'config/room.json').exists() else {}
    return {'run': str(run.resolve()), 'mission_passed': mission['passed'],
        'configuration': {key: scene.get(key) for key in ('publication_profile','physics_sync','physics_hz','clock_hz','odometry_hz','scan_hz')},
        'scope': 'Single CPU PhysX world; real Mission/Nav2; mock work; stage times include synchronous native callbacks.',
        'timing_policy': 'Exclusive wall and adapter-main-thread CPU stages. Transition and native-capture windows excluded from NAVIGATION loop statistics. Process CPU: 100%=one core. GPU totals include other applications. Main-thread CPU excludes worker threads.',
        'phases': summary, 'navigation_processes': sorted(process_summary, key=lambda p:p['mean_cpu_percent_one_core'], reverse=True),
        'navigation_cpu_by_role_percent_one_core': role_cpu,
        'navigation_other_cpu_percent_one_core': sum(row['other_cpu_total_percent_one_core']*row['sample_interval_s'] for row in nav)/duration,
        'navigation_host_gpu_utilization_percent': {'mean': sum(gpu)/len(gpu), 'max': max(gpu)} if gpu else None,
        'system_samples': len(samples), 'navigation_system_samples': len(nav)}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps(analyze(args.run), indent=2)+'\n')
