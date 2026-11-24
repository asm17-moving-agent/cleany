"""Read observed milestones from a single operator prepare job and its launch log."""

import json
import re
from pathlib import Path


def _log_tail(path: Path) -> str:
    try:
        with path.open('rb') as stream:
            stream.seek(max(0, stream.seek(0, 2) - 131072))
            return stream.read().decode(errors='replace')
    except OSError:
        return ''


def prepare_progress(log: Path, repo: Path) -> dict:
    """Return observed states only. Every support log must belong to this job."""
    outer = _log_tail(log)
    match = re.search(r'^started hardware_support \(PID \d+, log ([^\n]+/hardware-support\.log)\)$',
                      outer, re.MULTILINE)
    support = ''
    if match:
        path = Path(match[1]).resolve()
        if path.is_relative_to((repo / 'runtime/collection-operator').resolve()):
            support = _log_tail(path)

    stages = {}
    for stage, started, done in (
        ('preflight', 'Collection CPU affinity:', 'TORQUE OFF VERIFIED FOR IDS'),
        ('control', 'before real_control', ''),
        ('planning', 'started hardware_support (', ''),
        ('feedback', 'Waiting for services, actions, 14-joint feedback', ''),
        ('executor', 'before executor startup', ''),
    ):
        if started in outer:
            stages[stage] = {'state': 'active'}
        if done and done in outer:
            stages[stage] = {'state': 'done'}

    events = {'torque_check_off': 'preflight', 'control_startup_ready': 'control', 'planning_sequence_ready': 'planning',
              'collection_readiness': 'feedback', 'manipulation_startup_ready': 'executor',
              'startup_ik_warmup': 'gpu'}
    for line in (outer + '\n' + support).splitlines():
        start = line.find('{"event":')
        if start == -1:
            continue
        try:
            event = json.loads(line[start:])
        except ValueError:
            continue
        stage = events.get(event.get('event'))
        if stage and event.get('status', event.get('outcome')) == 'ready':
            stages[stage] = {'state': 'done', 'seconds': event.get('wall_sec')}

    if support and 'gpu' not in stages:
        phases = re.findall(r'warmup: phase=(\w+) state=started([^\n]*)', support)
        if phases:
            phase, fields = phases[-1]
            arm = re.search(r'\barm=(left|right)\b', fields)
            stages['gpu'] = {'state': 'active', 'phase': phase, 'arm': arm[1] if arm else ''}
    if support and stages.get('planning', {}).get('state') == 'active':
        waits = re.findall(r'Waiting for collection readiness \([^\n]*?\): ([^\n]+)', support)
        if waits:
            stages['planning']['waiting_for'] = waits[-1][:160]
    if 'Repeating the full readiness check' in outer or 'repeating the full readiness check' in outer:
        stages['reuse'] = {'state': 'done'}
    return stages
