#!/usr/bin/env python3
"""Read-only evidence checks for completed runs; writes only generated audit JSON."""
import csv
import hashlib
import json
import sqlite3
from pathlib import Path

ROOT = Path(__file__).resolve().parent
TOPICS = ['/clock', '/ground_truth/odom', '/odom', '/wheel/odom', '/joint_states',
          '/nav2/cmd_vel', '/cmd_vel', '/scan', '/collision_monitor_state', '/tf',
          '/tf_static', '/head_pan/state', '/body_guard_3d/state',
          '/evaluation/depth_points', '/amcl_pose', '/plan',
          '/local_costmap/costmap', '/global_costmap/costmap']


def audit(run):
    result = json.loads((run/'analysis.json').read_text())
    counts = {}
    for db in (run/'bag').glob('*.db3'):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro', uri=True) as conn:
            for topic, count in conn.execute('SELECT topics.name, count(messages.id) FROM topics LEFT JOIN messages ON messages.topic_id=topics.id GROUP BY topics.id'):
                counts[topic] = counts.get(topic, 0)+count
    commands = list(csv.reader((run/'commands.csv').open()))
    final = [row for row in commands if row[2] == '/cmd_vel']
    a = {
        'run': str(run.relative_to(ROOT)),
        'source_commit': json.loads((ROOT/'environment.json').read_text()).get('source_head'),
        'map_sha256': hashlib.sha256((run/'map.pgm').read_bytes()).hexdigest(),
        'motion_config_sha256': hashlib.sha256((run/'config/comparison_motion.yaml').read_bytes()).hexdigest(),
        'bag_topic_counts': counts,
        'missing_or_empty_requested_topics': [t for t in TOPICS if counts.get(t, 0)==0],
        'compute_call_count': result.get('controller_compute',{}).get('all_ms',{}).get('count'),
        'last_final_command': [float(v) for v in final[-1][3:]] if final else None,
        'final_actual_velocity': result.get('final_actual_velocity'),
        'max_actual_wheel_rad_s': result.get('actual_wheel_feedback_rad_s',{}).get('max'),
        'max_final_command_wheel_rad_s': result.get('max_command_wheel_rad_s',{}).get('/cmd_vel'),
        'actual_contact_count': None,
        'collision_monitor_state_note': 'No state messages were recorded when this topic is missing/empty. Probe defaults and zero action samples are not evidence of zero safety interventions.',
        'plan_recording_note': 'A missing plan topic limits retrospective path analysis; do not infer that no path was planned. Goals were accepted and controller compute calls recorded.',
        'sensor_received_age_note': 'Callback receive sim clock minus sensor header stamp, not a synchronized transport-latency measurement; small negative ages reflect callback ordering.',
        'safety_wait_note': 'Adjacent received-state sample durations inside navigation; final unclosed interval omitted. STOP_OBSTACLE is intentional obstacle wait; unavailability and processing overruns are separate.',
    }
    (run/'audit.json').write_text(json.dumps(a, indent=2))
    return a


if __name__ == '__main__':
    reports = [audit(p.parent) for group in ('rpp_rtf1','teb_rtf1','aux_dwb','isolated_rpp','isolated_teb')
               for p in sorted((ROOT/group).glob('*/analysis.json'))]
    (ROOT/'audit_summary.json').write_text(json.dumps(reports, indent=2))
    print(json.dumps([{'run':r['run'], 'missing_topics':r['missing_or_empty_requested_topics'],
                       'last_command':r['last_final_command'], 'wheel_max':r['max_actual_wheel_rad_s']} for r in reports], indent=2))
