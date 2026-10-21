"""Diagnose witnessed feasible targets missed in the diverse survey, without changing runtime policy."""
from collections import defaultdict
from copy import deepcopy
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import time

import pytest

from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution, PoseValue
from cleany_skill_executor.physical_moveit_adapter import MoveItGraspAdapter
from test_curobo_gpu import ros_node
from test_hybrid_gpu import native
from test_hybrid_target_matrix_gpu import baseline_state, solver_count, write_json

pytestmark=pytest.mark.skipif(os.environ.get('CLEANY_RUN_HYBRID_RECOVERY')!='1',
    reason='Enable with make test-motion-hybrid-recovery after the target matrix')


def test_recovery_of_known_feasible_first_pass_misses(native, tmp_path, record_property):
    node,adapter,measured,_=native
    directory=Path(os.environ.get('CLEANY_HYBRID_MATRIX_OUTPUT',str(tmp_path/'matrix')))
    report=json.loads((directory/'report.json').read_text())
    assert report['completed'], 'The complete target survey is required'
    cases={case['case_id']:case for case in json.loads((directory/'targets.json').read_text())}
    first={row['case_id']:row for row in json.loads((directory/'rows.json').read_text()) if row['repeat']==0}
    original_batches=json.loads((directory/'batches.json').read_text())
    missed=[case for case in cases.values() if case['witness_valid'] is True
            and not case['expected_rejection'] and not first[case['case_id']]['ik_accepted']]
    grouped=defaultdict(list)
    for case in missed:
        grouped[(case['arm'],case['opening'],first[case['case_id']]['start_index'])].append(case)
    adapter._config=replace(adapter._config,planning_timeout_sec=4.,planning_attempts=1,
                            state_validity_timeout_sec=5.)
    cpu=MoveItGraspAdapter(node,adapter._config,ik_client=adapter._ik_client,
        fk_client=adapter._fk_client,validity_client=adapter._validity_client,
        plan_client=adapter._plan_client)
    cpu.set_robot_description(adapter._description)
    rows,batches=[],[]
    began=time.monotonic()
    for (arm,opening,start_index),group in grouped.items():
        for offset in range(0,len(group),8):
            chunk=group[offset:offset+8]
            targets=[PoseValue(tuple(case['pose']['position']),tuple(case['pose']['orientation_xyzw']))
                     for case in chunk]
            values_by_seed={}
            for seeds in (16,32):
                adapter.set_current_state(baseline_state(measured,arm,start_index))
                adapter.set_deadline(time.monotonic()+1200.)
                adapter.set_gripper_position(arm,opening)
                before=solver_count(adapter); start=time.monotonic()
                results=adapter._batch(arm,targets,[None]*len(targets),seeds,pad=True)
                values_by_seed[seeds]=results
                batches.append(dict(arm=arm,opening=opening,start_index=start_index,seeds=seeds,
                    case_ids=[case['case_id'] for case in chunk],seconds=time.monotonic()-start,
                    initializes_solver=solver_count(adapter)>before,
                    recovered=sum(value is not None for value in results),
                    statistics=deepcopy(adapter.last_ik_statistics)))
            for index,(case,target) in enumerate(zip(chunk,targets,strict=True)):
                state=baseline_state(measured,arm,start_index)
                cpu.set_current_state(state)
                cpu.set_deadline(time.monotonic()+1200.)
                cpu.set_gripper_position(arm,opening)
                witness=JointSolution(ARM_JOINT_NAMES[arm],tuple(case['witness']))
                assert cpu.state_is_valid(arm,witness), 'Feasibility witness must survive replay'
                expected=cpu.forward_kinematics(arm,witness)
                assert math.dist(expected.position,target.position)<1e-8
                started=time.monotonic()
                proposal=cpu.solve_aligned_ik(arm,target,None)
                result=(proposal if proposal is not None and cpu._feasible_alignment(proposal)
                        and cpu.state_is_valid(arm,proposal.solution) else None)
                elapsed=time.monotonic()-started
                original_batch=original_batches[first[case['case_id']]['batch_index']]
                row=dict(case_id=case['case_id'],arm=arm,category=case['category'],opening=opening,
                    repeat16_recovered=values_by_seed[16][index] is not None,
                    forced32_recovered=values_by_seed[32][index] is not None,
                    cpu_recovered=result is not None,cpu_seconds=elapsed,cpu_plan_passed=False,
                    original_runtime_retried32=bool(original_batch['statistics']['fallback_batches']))
                if proposal is not None:
                    row['cpu_alignment_errors']=dict(position_m=proposal.position_error_m,
                        approach_degrees=proposal.approach_error_degrees,
                        closing_degrees=proposal.closing_error_degrees)
                if result is not None:
                    assert cpu._feasible_alignment(result)
                    assert cpu.state_is_valid(arm,result.solution)
                    planning_start=time.monotonic()
                    path=cpu.plan(arm,result.solution,None)
                    row.update(cpu_plan_passed=path is not None,
                               cpu_plan_seconds=time.monotonic()-planning_start,
                               cpu_solution=list(result.solution.positions))
                rows.append(row)
                write_json(directory/'recovery_rows.json',rows)
                print(f'RECOVERY {len(rows)}/{len(missed)}: {case["case_id"]}, '
                      f'16={row["repeat16_recovered"]} 32={row["forced32_recovered"]} '
                      f'CPU={row["cpu_recovered"]}',flush=True)
    plan_retries=[]
    for key,failed in first.items():
        if not failed.get('plan_attempted') or failed['plan_passed']:
            continue
        case=cases[key]; arm=case['arm']
        goal=JointSolution(ARM_JOINT_NAMES[arm],tuple(failed['solution']))
        attempts=[]
        for retry in range(5):
            cpu.set_current_state(baseline_state(measured,arm,failed['start_index']))
            cpu.set_deadline(time.monotonic()+1200.)
            cpu.set_gripper_position(arm,case['opening'])
            assert cpu.state_is_valid(arm,goal)
            started=time.monotonic()
            path=cpu.plan(arm,goal,None)
            attempts.append(dict(retry=retry,passed=path is not None,
                                 seconds=time.monotonic()-started))
        plan_retries.append(dict(case_id=key,attempts=attempts,
                                 passed=sum(attempt['passed'] for attempt in attempts)))
        print(f'PLAN RECOVERY {key}: {plan_retries[-1]["passed"]}/5',flush=True)
    result=dict(scope='paired diagnostic on known-feasible first-pass IK misses',
        target_count=len(missed),completed=len(rows)==len(missed),seconds=time.monotonic()-began,
        repeat16_recovered=sum(row['repeat16_recovered'] for row in rows),
        forced32_recovered=sum(row['forced32_recovered'] for row in rows),
        cpu_recovered=sum(row['cpu_recovered'] for row in rows),
        cpu_plans_passed=sum(row['cpu_plan_passed'] for row in rows),rows=rows,batches=batches,
        first_pass_plan_failure_retries=plan_retries,
        originally_not_retried32=sum(not row['original_runtime_retried32'] for row in rows),
        runtime_policy_changed=False,physical_motors_accessed=False,
        caveats=['A second independent ROS/GPU process: graph initialization is included where marked.',
                 'Failed targets are regrouped/padded for both 16 and 32; the original batch context differs.',
                 'Forced 32 and CPU candidate IK are diagnostics, not the current deployed fallback policy.',
                 'Only CPU-recovered targets receive an additional CPU plan.'])
    write_json(directory/'recovery.json',result)
    record_property('recovery_output',str(directory/'recovery.json'))
    assert result['completed']
