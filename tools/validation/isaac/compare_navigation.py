"""Compare actual recorded navigation time, distance, speed and recovery events."""
import argparse
import bisect
import json
import math
from pathlib import Path
import re


def measure(run: Path) -> dict:
    report = json.loads((run / 'evaluations/mission.json').read_text())
    events = report['states']
    event_times = [event['wall_time'] for event in events]
    links = json.loads((run / 'media/links.json').read_text())
    base_index = next(i for i, path in enumerate(links) if path.endswith('/base_link'))
    phases = {phase: [] for phase in ('NAVIGATE_TO_TARGET', 'RETURN_HOME')}
    for line in (run / 'media/motion.jsonl').read_text().splitlines():
        frame = json.loads(line)
        index = bisect.bisect_right(event_times, frame['wall_time'])-1
        phase = events[index]['state'] if index >= 0 else ''
        if phase in phases:
            phases[phase].append((frame['sim_time'], frame['wall_time'], frame['links'][base_index]))
    results = {}
    for phase, frames in phases.items():
        assert len(frames) > 1, phase
        distance = 0.
        speeds = []
        for a, b in zip(frames, frames[1:]):
            ds = math.hypot(b[2][0]-a[2][0], b[2][1]-a[2][1])
            distance += ds
            speeds.append(ds/(b[0]-a[0]))
        duration = frames[-1][0]-frames[0][0]
        results[phase] = {'simulation_s': duration, 'wall_s': frames[-1][1]-frames[0][1],
                         'distance_m': distance, 'average_m_s': distance/duration,
                         'peak_m_s': max(speeds), 'samples': len(frames)}
    log = (run / 'logs/mission.log').read_text()
    return {'run': str(run.resolve()), 'passed': report['passed'], 'home_error_m': report.get('home_error_m'),
            'phases': results, 'navigation_simulation_s': sum(p['simulation_s'] for p in results.values()),
            'planner_failures': len(re.findall(r'failed to create plan', log)),
            'recovery_behaviors': len(re.findall(r'Running (?:spin|backup|wait|drive_on_heading)', log)),
            'costmap_clears': len(re.findall(r'Received request to clear', log)),
            'feedback_recoveries': report.get('navigation_recoveries')}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--runs', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    baseline = measure(args.baseline)
    runs = [measure(path) for path in args.runs]
    for run in runs:
        run['navigation_time_reduction_percent'] = 100*(1-run['navigation_simulation_s']/baseline['navigation_simulation_s'])
    args.output.write_text(json.dumps({'baseline': baseline, 'runs': runs}, indent=2)+'\n')
