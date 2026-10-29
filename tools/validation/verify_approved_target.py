#!/usr/bin/env python3
"""Measure real-camera approval reobservations without dispatching a robot goal."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import time

import rclpy

from cleany_dev_monitor.proposal_ros import ProposalROS
from cleany_dev_monitor.proposals import DEFAULTS, write_json


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--count', type=int, default=5)
    args = parser.parse_args()
    if not 1 <= args.count <= 20:
        parser.error('--count must be between 1 and 20')
    args.output.mkdir(parents=True, exist_ok=False)
    control = Path(os.environ.get('CLEANY_WEB_CONTROL_DIRECTORY',
        str(Path.home() / '.local/state/cleany/dev-monitor/operator')))
    report = dict(started_at=time.time(), robot_motion_requested=False, revalidations=[], success=False)
    adapter = None
    try:
        with (control / 'operation.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            rclpy.init()
            adapter = ProposalROS(args.output / 'operator', DEFAULTS)
            began = time.monotonic()
            proposal = adapter.propose(lambda: False)
            report.update(proposal=proposal, proposal_seconds=time.monotonic() - began)
            if proposal['state'] != 'READY':
                raise ValueError('No eligible target; no approval reobservation was attempted')
            for index in range(args.count):
                began = time.monotonic()
                item = dict(index=index + 1, success=False)
                try:
                    adapter.revalidate(proposal, lambda: False)
                    item['success'] = True
                except Exception as error:
                    item['message'] = str(error)
                item['seconds'] = time.monotonic() - began
                audit = args.output / 'operator/revalidations' / f"{proposal['id']}.json"
                if audit.exists():
                    item['audit'] = json.loads(audit.read_text())
                report['revalidations'].append(item)
                write_json(args.output / 'verification.json', report)
            report['success'] = all(item['success'] for item in report['revalidations'])
    except Exception as error:
        report['message'] = str(error)
    finally:
        if adapter is not None:
            adapter.close()
        if rclpy.ok():
            rclpy.shutdown()
        report['finished_at'] = time.time()
        write_json(args.output / 'verification.json', report)
    print(json.dumps(dict(success=report['success'], report=str(args.output / 'verification.json'),
                         robot_motion_requested=False)))
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
