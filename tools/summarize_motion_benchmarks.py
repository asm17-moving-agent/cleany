#!/usr/bin/env python3
"""Summarize motion trials without mixing warmup, failure, or collision scopes."""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import statistics
from typing import Any


def latency_statistics(values: list[float]) -> dict[str, Any]:
    if any(not math.isfinite(value) or value < 0 for value in values):
        raise ValueError('Timing values must be finite and nonnegative')
    if not values:
        return {'samples': 0, 'mean_sec': None, 'median_sec': None,
                'min_sec': None, 'max_sec': None, 'p95_sec': None}
    ordered = sorted(values)
    position = (len(ordered)-1)*.95
    lower = int(position)
    upper = min(lower+1, len(ordered)-1)
    p95 = ordered[lower] + (ordered[upper]-ordered[lower])*(position-lower)
    return {'samples': len(values), 'mean_sec': statistics.fmean(values),
            'median_sec': statistics.median(values), 'min_sec': ordered[0],
            'max_sec': ordered[-1], 'p95_sec': p95 if len(values) >= 30 else None}


def telemetry_for_trial(trial: dict, events: list[dict]) -> dict:
    start, end = trial.get('started_unix_sec'), trial.get('ended_unix_sec')
    if start is None or end is None:
        return {'available': False, 'reason': 'Trial has no absolute time markers'}
    selected = [event for event in events if event.get('kind') == 'tegrastats'
                and start <= event['unix_sec'] <= end]
    fields = ('gpu_percent', 'cpu_percent', 'gpu_temperature_c', 'ram_used_mb', 'power_mw')
    output = {'available': bool(selected), 'samples': len(selected)}
    for field in fields:
        values = [float(event[field]) for event in selected if field in event]
        output[field] = {'mean': statistics.fmean(values) if values else None,
                         'peak': max(values) if values else None}
    power = sorted((event['unix_sec'], event['power_mw']/1000.)
                   for event in selected if 'power_mw' in event)
    if len(power) > 1:
        covered = power[-1][0]-power[0][0]
        output['observed_system_energy_j'] = sum(
            (b[0]-a[0])*(a[1]+b[1])/2 for a, b in zip(power, power[1:]))
        output['power_sample_coverage_fraction'] = covered/(end-start) if end > start else None
    else:
        output['observed_system_energy_j'] = None
        output['power_sample_coverage_fraction'] = None
    output['power_scope'] = 'Jetson VDD_IN system power; observed sample interval only'
    return output


def summarize_trials(trials: list[dict], events: list[dict] | None = None) -> dict:
    usable = [trial for trial in trials if not trial.get('is_warmup') and not trial.get('skipped')]
    output = {'warmup_trials': sum(bool(t.get('is_warmup')) for t in trials),
              'skipped_trials': sum(bool(t.get('skipped')) for t in trials)}
    for label, selected in (('cold', [t for t in usable if not t['warm']]),
                             ('warm', [t for t in usable if t['warm']])):
        walls = [float(trial['wall_sec']) for trial in selected]
        counts = [sum(bool(item['success']) for item in trial['items']) for trial in selected]
        if any('successes' in t and t['successes'] != count for t, count in zip(selected, counts)):
            raise ValueError('Trial success count disagrees with its candidate results')
        first = [min(float(item['completion_sec']) for item in trial['items'] if item['success'])
                 for trial in selected if any(item['success'] for item in trial['items'])]
        denominator = sum(len(t['items']) for t in selected)
        total_sec = sum(walls)
        summary = {
            'calls': len(selected), 'calls_with_valid_result': sum(count > 0 for count in counts),
            'call_success_rate': sum(count > 0 for count in counts)/len(selected) if selected else None,
            'evaluated_candidates': denominator, 'valid_candidates': sum(counts),
            'candidate_success_rate': sum(counts)/denominator if denominator else None,
            'wall_time': latency_statistics(walls),
            'first_valid_time_conditional_on_success': latency_statistics(first),
            'evaluated_candidates_per_sec': denominator/total_sec if total_sec else None,
            'primary_ik_fallback_calls': sum(bool(t.get('ik_policy', {}).get('fallback_triggered')) for t in selected),
            'grasp_ik_fallback_candidates': sum(bool(item.get('grasp_fallback_triggered'))
                                               for t in selected for item in t['items']),
        }
        failures: dict[str, int] = {}
        for trial in selected:
            for item in trial['items']:
                if not item['success']:
                    stage = item.get('failure_stage') or 'unclassified'
                    failures[stage] = failures.get(stage, 0)+1
        summary['failure_stages'] = failures
        if events is not None:
            summary['telemetry_per_trial'] = [telemetry_for_trial(t, events) for t in selected]
        output[label] = summary
    return output


def comparison_key(data: dict) -> str | None:
    # Legacy runs lack proof of the replayed source and runtime scene model.
    required = ('source_sha256', 'collision_model_sha256', 'fixture_configuration_sha256')
    if not data.get('scene_collision_model_verified') or not all(data.get(key) for key in required):
        return None
    config = data['config']
    contract = {key: config[key] for key in (
        'position_tolerance_m', 'approach_tolerance_degrees', 'closing_tolerance_degrees',
        'validation_step_rad', 'maximum_validation_states', 'approach_corridor_radius_m')}
    value = {'phase': data['phase'], 'candidate_count': data['actual_candidate_count'],
             'arms':sorted(data.get('arms') or {t['arm'] for t in data.get('trials', [])}),
             **{key:data[key] for key in required}, 'contract': contract}
    if data['phase'] == 'path':
        if not data.get('path_requests_sha256'):
            return None
        value['path_requests_sha256'] = data['path_requests_sha256']
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def read_telemetry(path: Path) -> list[dict]:
    events = []
    for line in path.read_text().splitlines():
        event = json.loads(line)
        timestamp = datetime.fromisoformat(event['timestamp'].replace('Z', '+00:00'))
        if timestamp.tzinfo is None:
            raise ValueError('Telemetry timestamps require an explicit timezone')
        event['unix_sec'] = timestamp.timestamp()
        events.append(event)
    return events


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--telemetry', type=Path)
    args = parser.parse_args()
    events = read_telemetry(args.telemetry) if args.telemetry else None
    summaries = []
    for path in args.results:
        data = json.loads(path.read_text())
        summaries.append({'source': str(path), 'backend': data['backend'], 'phase': data['phase'],
                          'collision_scope': data.get('collision_scope'),
                          'comparison_group': comparison_key(data),
                          'optimizer_settings': {'gpu_config': data.get('gpu_config'),
                             'ik_iterations': data.get('ik_iterations_override'),
                             'trajectory_seeds': data.get('trajectory_seeds'),
                             'joint_plan_graph': data.get('joint_plan_graph', False),
                             'batch_size': data.get('candidate_batch_size'),
                             'fallback_gpu_config': data.get('fallback_gpu_config'),
                             'path_backend': data.get('path_backend'),
                             'warmup_count': data.get('warmup_count'),
                             'cpu_local_fk': data.get('config', {}).get('use_verified_local_fk', False),
                             'cpu_ik_timeout_sec': data.get('config', {}).get('ik_timeout_sec'),
                             'server_ik_timeouts_sec': data.get('server_ik_timeouts_sec'),
                             'cpu_ik_collision_search': data.get('config', {}).get('ik_collision_search', True),
                             'cpu_refinement_iterations': data.get('config', {}).get('pose_refinement_iterations'),
                             'align_wrist_roll': data.get('config', {}).get('align_wrist_roll'),
                             'planning_attempts': data.get('config', {}).get('planning_attempts'),
                             'planning_timeout_sec': data.get('config', {}).get('planning_timeout_sec')},
                          'run_error': data.get('error') or data.get('scene_restore_error'),
                          'initialization_sec': data.get('initialization_sec'),
                          'trials': summarize_trials(data['trials'], events)})
    output = {'summaries': summaries,
              'notes': ['Only matching non-null comparison groups share the recorded input/model/contract.',
                        'CPU FCL and GPU convex collision algorithms remain different.',
                        'First-valid timings are conditional on success; failed calls remain in success-rate denominators.',
                        'p95 is withheld below 30 timing samples.',
                        'No warmup work is included in request timing statistics.']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2))
    print(json.dumps({'summaries': len(summaries), 'output': str(args.output)}))


if __name__ == '__main__':
    main()
