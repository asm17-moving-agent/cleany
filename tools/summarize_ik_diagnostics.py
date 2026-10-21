#!/usr/bin/env python3
"""Read candidate pose-error evidence from a ROS log without contacting ROS."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
from typing import Iterable


def read_diagnostics(lines: Iterable[str]) -> list[dict]:
    records = []
    for number, line in enumerate(lines, start=1):
        _, marker, payload = line.partition('ik_diagnostic=')
        if not marker:
            continue
        try:
            record = json.loads(payload)
        except json.JSONDecodeError as error:
            raise ValueError(f'Invalid IK diagnostic at line {number}: {error}') from error
        if not isinstance(record, dict):
            raise ValueError(f'IK diagnostic at line {number} must be an object')
        records.append(record)
    return records


def summarize(records: list[dict]) -> dict:
    groups: dict[tuple, dict] = {}
    for record in records:
        target = record['target']
        key = (record.get('selection_id'), record['arm'], record['stage'], record['candidate_index'],
               tuple(target['position_m']), tuple(target['orientation_xyzw']),
               record['gripper_joint_position_rad'])
        group = groups.setdefault(key, {
            'selection_id': record.get('selection_id'),
            'candidate_index': record['candidate_index'], 'arm': record['arm'],
            'stage': record['stage'], 'target': target, 'attempts': 0,
            'best_recorded_alignment': None,
        })
        group['attempts'] += 1
        group['last_outcome'] = record['outcome']
        group['last_backend'] = record['backend']
        # Preserve errors from the same joint solution. Never combine the
        # independent position/approach/closing minima into a fictional pose.
        alignment = record.get('closest_alignment') or record.get('proposal_alignment')
        if alignment is None:
            continue
        limits = record['limits']
        ratios = [alignment[name] / limit if alignment[name] is not None else float('inf')
                  for name, limit in limits.items()]
        rank = (max(ratios), sum(ratios))
        if 'best_rank' not in group or rank < group['best_rank']:
            group['best_rank'] = rank
            group['best_recorded_alignment'] = {
                **alignment, 'backend': record['backend'], 'limits': limits,
                'attempt_outcome': record['outcome'],
            }
        if 'final_alignment' in record:
            group['last_final_alignment'] = record['final_alignment']
    candidates = list(groups.values())
    for group in candidates:
        group.pop('best_rank', None)
    return {
        'available': bool(records), 'diagnostic_records': len(records),
        'attempt_outcomes': dict(Counter(record['outcome'] for record in records)),
        'candidate_outcomes': dict(Counter(group['last_outcome'] for group in candidates)),
        'candidates': candidates,
        'scope': 'Recorded representative poses; pose errors do not prove collision or path validity',
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--json', action='store_true', help='Print the complete structured summary')
    args = parser.parse_args()
    with args.log.open(encoding='utf-8', errors='replace') as stream:
        report = summarize(read_diagnostics(stream))
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        return
    if not report['available']:
        print('이 로그에는 IK 오차 기록이 없습니다. 변경 적용 후 새 실행 로그가 필요합니다.')
        return
    print('arm   candidate stage         last_outcome             best_mm approach_deg closing_deg backend')
    for group in report['candidates']:
        best = group['best_recorded_alignment']
        def value(name: str) -> str:
            number = best.get(name) if best is not None else None
            return f'{number:.2f}' if number is not None else '-'
        print(f"{group['arm']:5} {group['candidate_index']:9} {group['stage']:13} "
              f"{group['last_outcome']:24} {value('position_error_mm'):>7} "
              f"{value('approach_error_degrees'):>12} {value('closing_error_degrees'):>11} "
              f"{best['backend'] if best is not None else '-'}")
    print('best는 기록된 대표 해 중 오차가 작은 해입니다. 충돌·경로 통과 여부는 별도입니다.')


if __name__ == '__main__':
    main()
