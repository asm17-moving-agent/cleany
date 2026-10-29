"""Partition recorded manipulation stages without double-counting nested timers."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import re


PHASES = {'IK_SOLVER_SETUP', 'GPU_IK', 'GPU_FK_ALIGNMENT', 'MOVEIT_COLLISION',
          'FINAL_FK_CHECK', 'FINAL_STATE_CHECK', 'MOVEIT_PLAN'}
LABELS = {
    'ValidateGoal': '입력·포트 확인', 'PrepareTarget': '승인 대상 준비',
    'ReconstructTarget': '대상 3D 복원', 'GenerateGrasp': '파지 후보 생성',
    'SelectArmAndPath': '팔·자세·경로 선택', 'MoveToPregrasp': 'Pregrasp 이동',
    'ApproachObject': '최종 접근', 'GraspObject': '그리퍼 닫기',
    'ConfirmGrasp': '파지 확인', 'LiftObject': '후퇴·들어올리기',
    'ConfirmHeld': '들림 확인', 'CarryObject': '원위치로 이동',
    'CheckPlacementTarget': '놓을 위치 확인', 'OpenGripperAtDestination': '놓기',
    'ConfirmRelease': '해제 처리', 'ReturnArm': '빈 팔 복귀',
    'VerifyPlacedObject': '최종 상태 확인', 'FinalizeSuccess': '결과 확정',
}


def read_events(path: Path) -> list[dict]:
    events = []
    for number, line in enumerate(path.read_text().splitlines(), 1):
        match = re.search(r'\[([0-9]+\.[0-9]+)\]', line)
        if not match:
            continue
        payload = re.search(r'(\{.*\})\s*$', line)
        if payload:
            try:
                data = json.loads(payload[1])
            except json.JSONDecodeError:
                continue
            if not isinstance(data, dict) or not isinstance(data.get('stage'), str):
                continue
        else:
            timer = re.search(r'phase=(\w+) state=done wall_sec=([0-9.]+)', line)
            if not timer or timer[1] not in PHASES:
                continue
            stage = re.search(r'stage=(\w+):', line)
            data = dict(stage='selector_phase', phase=timer[1], wall_sec=float(timer[2]),
                        selector_stage=stage[1] if stage else '')
        events.append(dict(timestamp_s=float(match[1]), line=number, file=path.name, data=data))
    return events


def components(events: list[dict], duration: float) -> dict:
    """Use disjoint logged totals; nominal command time is not measured motor time."""
    parts = dict(planning=0., preparation=0., sensor=0., command=0.)
    for event in events:
        d = event['data']
        stage = d.get('stage', '')
        if stage == 'selector_phase':
            parts['planning'] += d['wall_sec']
        elif stage == 'return_planned':
            parts['planning'] += d['planning_wall_sec']
        elif stage == 'hardware_sensor_scene_refreshed':
            parts['sensor'] += d['wall_sec']
        elif stage.endswith('_trajectory_prepared'):
            parts['preparation'] += d['preparation_wall_sec']
            parts['command'] += d['duration_sec']
    remainder = duration-sum(parts.values())
    if remainder < -1e-6:
        raise ValueError('Component budget exceeds stage duration; inspect source overlap')
    return {**parts, 'unseparated': max(0., remainder)}


def analyze(source: Path) -> dict:
    summary = json.loads((source/'summary.json').read_text())
    raw = json.loads((source/'executions.json').read_text())
    cycles, log_hashes = [], {}
    for cycle in summary['cycles']:
        eid = cycle['execution_id']
        record = next(r for r in raw['executions'] if r['goal']['execution_id'] == eid)
        accepted = record['accepted_at_ns']/1e9
        logs = source/'runtime-logs'/cycle['runtime']
        for name in ('manipulation-action.log', 'hardware-support.log'):
            path = logs/name
            log_hashes[str(path.relative_to(source))] = hashlib.sha256(path.read_bytes()).hexdigest()
        events = sorted(read_events(logs/'manipulation-action.log') +
                        read_events(logs/'hardware-support.log'), key=lambda e: e['timestamp_s'])
        stages, running = [], {}
        for transition in raw['transitions']:
            node = transition['node_id']
            if transition['execution_id'] != eid or node not in LABELS:
                continue
            if transition['status'] == 'RUNNING':
                running.setdefault(node, transition['observed_at_ns'])
            elif transition['status'] in ('SUCCESS', 'FAILURE') and node in running:
                start_ns, end_ns = running.pop(node), transition['observed_at_ns']
                start, end = start_ns/1e9, end_ns/1e9
                selected = [e for e in events if start <= e['timestamp_s'] <= end]
                duration = (end_ns-start_ns)/1e9
                stages.append(dict(node=node, label=LABELS[node], start_s=start-accepted,
                                   duration_s=duration, components=components(selected, duration),
                                   events=selected))
        stages.sort(key=lambda s: s['start_s'])
        total = cycle['execution_s']
        gaps = total-sum(s['duration_s'] for s in stages)
        if gaps < -1e-6 or len(stages) != len(LABELS):
            raise ValueError('Incomplete or overlapping BT stages')
        cycles.append(dict(execution_id=eid, runtime=cycle['runtime'],
                           period=cycle['period'], completed_at=cycle['completed_at'],
                           total_s=total, gap_s=gaps, stages=stages))
    return dict(cycles=cycles, source=str(source.resolve()),
                scope='existing physical success logs; no new physical E2E',
                accounting='Command is nominal trajectory duration. Remainder includes CPU, RPC, '
                'controller overhead, settling and scheduling. Preparation includes its nested model timer.',
                log_sha256=log_hashes,
                source_sha256={name: hashlib.sha256((source/name).read_bytes()).hexdigest()
                               for name in ('executions.json', 'summary.json', 'source-manifest.json')})


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.source)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'analysis.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    with (args.output/'stage-breakdown.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['execution_id', 'stage', 'total_s', 'planning_s', 'preparation_s',
                         'sensor_s', 'nominal_command_s', 'unseparated_s'])
        for cycle in result['cycles']:
            for stage in cycle['stages']:
                writer.writerow([cycle['execution_id'], stage['node'], stage['duration_s'],
                                 *stage['components'].values()])
    for cycle in result['cycles']:
        print(cycle['execution_id'], round(cycle['total_s'], 3),
              {key: round(sum(s['components'][key] for s in cycle['stages']), 3)
               for key in ('planning', 'preparation', 'sensor', 'command', 'unseparated')})


if __name__ == '__main__':
    main()
