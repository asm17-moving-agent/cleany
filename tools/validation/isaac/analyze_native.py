"""Aggregate native trace zones without summing nested inclusive durations."""
from __future__ import annotations

import argparse
from collections import defaultdict
import gzip
import json
from pathlib import Path


def exclusive_main_zones(events: list[dict]) -> list[dict]:
    totals = defaultdict(lambda: {'calls': 0, 'inclusive_us': 0., 'exclusive_us': 0.})
    stack = []
    for event in sorted(events, key=lambda e: (e['ts'], -e['dur'])):
        start, end = event['ts'], event['ts']+event['dur']
        while stack and start >= stack[-1]['end']-1e-6:
            stack.pop()
        is_root = event['name'].startswith('Cleany::PhysicsStep')
        if is_root:
            stack = []
        elif not stack or end > stack[-1]['end']+1e-5:
            continue
        entry = totals[event['name']]
        entry['calls'] += 1
        entry['inclusive_us'] += event['dur']
        entry['exclusive_us'] += event['dur']
        if stack:
            totals[stack[-1]['name']]['exclusive_us'] -= event['dur']
        stack.append({'name': event['name'], 'end': end})
    return sorted(({'name': name, **value} for name, value in totals.items()),
                  key=lambda item: item['exclusive_us'], reverse=True)


def analyze(directory: Path) -> dict:
    metadata = json.loads((directory/'native-profile.json').read_text())
    main = metadata['main_thread_id']
    events = []
    inclusive = defaultdict(lambda: [0, 0.])
    with gzip.open(directory/'native-trace.json.gz', 'rt') as stream:
        for line in stream:
            if not line.startswith('{'):
                continue
            event = json.loads(line.rstrip(',\n'))
            if event.get('ph') != 'X':
                continue
            key = ('main' if event['tid']==main else 'other', event['name'])
            inclusive[key][0] += 1
            inclusive[key][1] += event['dur']
            if event['tid'] == main:
                events.append(event)
    zones = exclusive_main_zones(events)
    roots = [zone for zone in zones if zone['name'].startswith('Cleany::PhysicsStep')]
    root_us = sum(zone['inclusive_us'] for zone in roots)
    calls = sum(zone['calls'] for zone in roots)
    assert calls > 0, 'Trace contains no physics step roots'
    assert abs(sum(zone['exclusive_us'] for zone in zones)-root_us) < .001*root_us
    for zone in zones:
        zone['exclusive_percent_of_step'] = 100*zone['exclusive_us']/root_us
    return {'metadata': metadata, 'physics_steps': calls, 'mean_step_ms': root_us/calls/1000,
        'main_thread_exclusive_zones': zones,
        'inclusive_zones': [{'thread_group': group, 'name': name, 'calls': count,
                            'total_us': total, 'mean_us': total/count}
                           for (group, name), (count, total) in sorted(inclusive.items(), key=lambda item: item[1][1], reverse=True)],
        'interpretation': 'Main-thread exclusive zones partition instrumented PhysicsStep wall time. Inclusive zones overlap; worker thread durations cannot be added to main-thread wall time. Native capture adds overhead.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluations', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps(analyze(args.evaluations), indent=2)+'\n')
