#!/usr/bin/env python3
"""Read-only campaign evidence checks and bounded diagnostic excerpts."""
from pathlib import Path
import csv
import hashlib
import json
import math
import sqlite3
import yaml

ROOT = Path(__file__).resolve().parent


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    runner = json.loads((ROOT / 'runner_source_sha256.json').read_text())
    runner_check = {name: sha(ROOT / name) == digest for name, digest in runner.items()}
    assert all(runner_check.values()), runner_check
    common = json.loads((ROOT / 'common_snapshot_manifest.json').read_text())
    snapshot_check = {name: sha(ROOT / 'common_snapshot' / name) == digest
                      for name, digest in common['files'].items()}
    source_check = {name: sha(Path(common['common_source']) / name) == digest
                    for name, digest in common['files'].items()}
    assert all(snapshot_check.values()), snapshot_check
    assert all(source_check.values()), source_check
    evidence = []
    for result_path in sorted((ROOT / 'final_validation').glob('*/result.json')):
        p = result_path.parent
        r = json.loads(result_path.read_text())
        end = r['navigation_sim_end_s']
        with (p / 'samples.csv').open() as f:
            samples = [x for x in csv.DictReader(f)
                       if x['phase'] == 'comparison_navigation' and float(x['sim_s']) <= end]
        last = samples[-1] if samples else None
        goal = r['goal_map']
        observed = None
        if last:
            gt = [float(last['gt_x']), float(last['gt_y'])]
            tf = [float(last['tf_x']), float(last['tf_y'])]
            observed = {'sim_s': float(last['sim_s']), 'navigation_end_sim_s': end,
                        'age_before_navigation_end_s': end - float(last['sim_s']),
                        'gt_xy': gt, 'tf_xy': tf,
                        'gt_goal_error_m': math.dist(gt, goal[:2]),
                        'tf_goal_error_m': math.dist(tf, goal[:2]),
                        'tf_gt_separation_m': math.dist(tf, gt),
                        'interpretation': 'Last sampled TF/GT observation; not an atomic localization cause proof.'}
        log = (p / 'launch.log').read_text(errors='replace')
        patterns = ['Failed to transform from base_link to ',
                    'Failed to lookup current robot pose',
                    'Failed to get result for compute_path_to_pose in node halt',
                    'Lattice plan rejected:', 'failed to generate a path',
                    'Goal was canceled', 'Optimizer reset', 'Control loop missed its desired rate']
        diagnostics = {}
        for pattern in patterns:
            lines = [line for line in log.splitlines() if pattern in line]
            diagnostics[pattern] = {'count': len(lines), 'first': lines[0] if lines else None,
                                    'last': lines[-1] if lines else None}
        metadata = yaml.safe_load((p / 'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
        databases = []
        for db in sorted((p / 'bag').glob('*.db3')):
            with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as conn:
                databases.append({'name': db.name, 'bytes': db.stat().st_size,
                                  'quick_check': conn.execute('PRAGMA quick_check').fetchone()[0],
                                  'messages': conn.execute('SELECT count(*) FROM messages').fetchone()[0]})
        assert metadata['message_count'] > 0 and databases
        assert all(d['quick_check'] == 'ok' and d['messages'] > 0 for d in databases)
        item = {'goal_index': r['goal_index'], 'goal_name': r['goal_name'],
                'action_status': r.get('action_status'), 'timeout': r.get('timeout', False),
                'error': r.get('error'), 'last_sampled_tf_gt': observed,
                'log_diagnostics': diagnostics,
                'bag_message_count': metadata['message_count'], 'bag_databases': databases}
        (p / 'boundary_evidence.json').write_text(json.dumps(item, indent=2))
        evidence.append(item)
    assert len(evidence) == 8
    (ROOT / 'boundary_evidence.json').write_text(json.dumps({
        'frozen_runner_sha256_verified': runner_check,
        'common_snapshot_sha256_verified_count': len(snapshot_check),
        'common_source_unchanged_sha256_verified_count': len(source_check),
        'goals': evidence}, indent=2))
    print('Frozen source, common snapshot/source, eight bags and TF/GT boundaries audited')


if __name__ == '__main__':
    main()
