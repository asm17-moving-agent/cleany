"""Native deployed hybrid profile with failure-only retries and mock arm execution."""
from collections import defaultdict
from copy import deepcopy
import json
import os
from pathlib import Path
import threading
import time

import pytest
import yaml

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, JointSolution, PoseValue, PregraspRequest,
)
from cleany_skill_executor.physical_grasp_selection_node import GraspSelectionNode
from cleany_skill_executor.hybrid_adapter import HybridGraspAdapter
from cleany_skill_executor.trajectory_io import path_to_message
from test_curobo_gpu import ros_node
from test_hybrid_gpu import execute, native
from test_hybrid_target_matrix_gpu import apply_obstacle, baseline_state, solver_count, write_json

pytestmark=pytest.mark.skipif(os.environ.get('CLEANY_RUN_HYBRID_APPLIED')!='1',
    reason='Enable with make test-motion-hybrid-applied-gpu')


def test_deployed_node_profile_retries_failed_targets_and_plans_native_paths(native,tmp_path):
    from ament_index_python.packages import get_package_share_directory
    from moveit_msgs.srv import ApplyPlanningScene
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    node,reference,measured,actions=native
    root=Path(__file__).resolve().parents[4]
    source=Path(os.environ.get('CLEANY_HYBRID_REFERENCE',str(
        root/'benchmark-results/hybrid-target-matrix-20261002')))
    output=Path(os.environ.get('CLEANY_HYBRID_APPLIED_OUTPUT',str(tmp_path/'applied')))
    output.mkdir(parents=True,exist_ok=True)
    cases=json.loads((source/'targets.json').read_text())
    old_first={r['case_id']:r for r in json.loads((source/'rows.json').read_text()) if r['repeat']==0}
    config=Path(get_package_share_directory('cleany_skill_executor'))/'config'
    params={}
    for filename in ('grasp_selection.yaml','pregrasp.yaml','hybrid_motion.yaml'):
        params.update(yaml.safe_load((config/filename).read_text())['grasp_selection_server']['ros__parameters'])
    # This test supplies synthetic targets, with no sensor freshness/visibility oracle.
    params.update(require_sensor_scene=False,require_pregrasp_visibility=False,
                  action_name='test/hybrid_applied/select_reachable')
    selector=GraspSelectionNode(parameter_overrides=[Parameter(name,value=value) for name,value in params.items()])
    adapter=selector._adapter
    assert isinstance(adapter,HybridGraspAdapter)
    assert adapter._hybrid_config.retry_failed_candidates
    assert adapter._hybrid_config.pregrasp_retry_passes==0
    assert adapter._hybrid_config.progressive_pregrasp
    assert not adapter._hybrid_config.cpu_ik_fallback
    assert adapter._hybrid_config.planning_retries==1
    assert adapter._config.validate_motion_paths
    assert adapter._config.velocity_scaling==.1
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(selector)
    thread=threading.Thread(target=executor.spin,daemon=True)
    thread.start()
    apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
    assert apply.wait_for_service(timeout_sec=30.)
    rows,batches,paths,executions=[],[],{},[]
    began=time.monotonic()
    obstacle_active=False
    try:
        end=time.monotonic()+30.
        while not adapter._description:
            assert time.monotonic()<end, 'Runtime node did not receive the model'
            time.sleep(.02)
        assert not selector._description_error
        assert adapter._description==reference._description
        grouped=defaultdict(list)
        for c in cases:
            if c['category']=='random_valid_fk' or (
                    c['category'] in ('interior_fk','outside_reach','tcp_obstacle')
                    and int(c['case_id'].rsplit('/',1)[1])< (8 if c['category']=='interior_fk' else 4)):
                grouped[(c['arm'],c['category'],c['opening'])].append(c)
        for (arm,category,opening),group in grouped.items():
            for offset in range(0,len(group),8):
                chunk=group[offset:offset+8]
                start_index=(offset//8)%2
                state=baseline_state(measured,arm,start_index)
                adapter.set_current_state(state)
                adapter.set_deadline(time.monotonic()+float(params['action_timeout_sec']))
                adapter.set_gripper_position(arm,opening)
                start=JointSolution(ARM_JOINT_NAMES[arm],tuple(
                    dict(zip(state.name,state.position))[n] for n in ARM_JOINT_NAMES[arm]))
                assert adapter.state_is_valid(arm,start)
                if category=='tcp_obstacle':
                    points=[c['pose']['position'] for c in chunk]
                    lower=[min(p[i] for p in points) for i in range(3)]
                    upper=[max(p[i] for p in points) for i in range(3)]
                    apply_obstacle(node,apply,tuple((a+b)/2 for a,b in zip(lower,upper)),
                                   tuple(b-a+.16 for a,b in zip(lower,upper)))
                    obstacle_active=True
                requests=[PregraspRequest(arm,PoseValue(tuple(c['pose']['position']),
                    tuple(c['pose']['orientation_xyzw'])),opening) for c in chunk]
                before=solver_count(adapter); started=time.monotonic()
                adapter.prepare_pregrasps(requests)
                for request in requests:
                    adapter.set_gripper_position(arm, request.gripper_position)
                    adapter.solve_aligned_ik(arm, request.target, None)
                stats=deepcopy(adapter.last_ik_statistics)
                batches.append(dict(arm=arm,category=category,opening=opening,targets=len(chunk),
                    seconds=time.monotonic()-started,initializes_solver=solver_count(adapter)>before,
                    initial_accepted=stats['batch_calls'][0]['accepted_targets'],statistics=stats))
                for case,request in zip(chunk,requests,strict=True):
                    result=adapter._pregrasp_cache[request]
                    row=dict(case_id=case['case_id'],arm=arm,category=category,
                        reference_witness_valid=case['witness_valid'],expected_rejection=case['expected_rejection'],
                        previous_first_pass_ik_accepted=old_first[case['case_id']]['ik_accepted'],
                        ik_accepted=result is not None,plan_passed=False,plan_attempted=False)
                    if result is not None:
                        assert not case['expected_rejection']
                        assert adapter._feasible_alignment(result)
                        assert adapter.state_is_valid(arm,result.solution)
                        started=time.monotonic()
                        before_plan=len(adapter.last_ik_statistics['planning_calls'])
                        path=adapter.plan(arm,result.solution,start)
                        row.update(plan_attempted=True,plan_passed=path is not None,
                            plan_seconds=time.monotonic()-started,
                            plan_statistics=deepcopy(adapter.last_ik_statistics['planning_calls'][before_plan:]),
                            position_error_m=result.position_error_m,
                            approach_error_degrees=result.approach_error_degrees,
                            closing_error_degrees=result.closing_error_degrees)
                        if path is not None and category=='interior_fk':
                            paths[case['case_id']]=(arm,path,state)
                    rows.append(row)
                if obstacle_active:
                    apply_obstacle(node,apply);obstacle_active=False
                write_json(output/'rows.json',rows);write_json(output/'batches.json',batches)
                print(f'APPLIED {len(rows)}/96 goals: {arm}/{category}, '
                      f'initial {stats["batch_calls"][0]["accepted_targets"]}/{len(chunk)}, '
                      f'final {sum(adapter._pregrasp_cache[r] is not None for r in requests)}/{len(chunk)}',flush=True)
        for arm in ARM_JOINT_NAMES:
            selected=[(key,v) for key,v in paths.items() if v[0]==arm][:2]
            assert len(selected)==2
            for key,(_,path,state) in selected:
                current=dict(zip(measured[-1].name,measured[-1].position))
                positions=dict(zip(state.name,state.position))
                setup=JointTrajectory(joint_names=list(ARM_JOINT_NAMES[arm]),points=[
                    JointTrajectoryPoint(positions=[current[n] for n in ARM_JOINT_NAMES[arm]],velocities=[0.]*5),
                    JointTrajectoryPoint(positions=[positions[n] for n in ARM_JOINT_NAMES[arm]],velocities=[0.]*5)])
                setup.points[-1].time_from_start.sec=2
                execute(node,actions[arm],setup,measured)
                error,feedback=execute(node,actions[arm],path_to_message(path),measured)
                executions.append(dict(case_id=key,arm=arm,success=True,
                    maximum_joint_error_rad=error,feedback_messages=feedback))
            current=dict(zip(measured[-1].name,measured[-1].position))
            reset=JointTrajectory(joint_names=list(ARM_JOINT_NAMES[arm]),points=[
                JointTrajectoryPoint(positions=[current[n] for n in ARM_JOINT_NAMES[arm]],velocities=[0.]*5),
                JointTrajectoryPoint(positions=[0.]*5,velocities=[0.]*5)])
            reset.points[-1].time_from_start.sec=2
            execute(node,actions[arm],reset,measured)
        positive=[r for r in rows if not r['expected_rejection']]
        report=dict(completed=len(rows)==96,scope='deployed GraspSelectionNode profile / native CUDA / MoveIt / mock arms',
            seconds=time.monotonic()-began,parameters=params,unique_conditions=len(rows),
            known_feasible_targets=len(positive),accepted=sum(r['ik_accepted'] for r in positive),
            previous_first_pass_accepted=sum(r['previous_first_pass_ik_accepted'] for r in positive),
            initial_accepted=sum(b['initial_accepted'] for b in batches),
            primary_retry_targets=sum(b['statistics']['primary_retry_targets'] for b in batches),
            fallback_targets=sum(b['statistics']['fallback_targets'] for b in batches),
            cpu_fallback_targets=sum(len(b['statistics']['cpu_ik_fallback_calls']) for b in batches),
            cpu_fallback_accepted=sum(c['accepted'] for b in batches for c in b['statistics']['cpu_ik_fallback_calls']),
            expected_rejection_false_accepts=sum(r['ik_accepted'] for r in rows if r['expected_rejection']),
            plan_attempts=sum(r['plan_attempted'] for r in rows),plans_passed=sum(r['plan_passed'] for r in rows),
            executions=executions,rows=rows,batches=batches,physical_motors_accessed=False,
            power_mode_changed=False,clock_limits_changed=False,
            caveats=['Different process/seed stream from the original survey; comparison is descriptive.',
                     'Same stored target poses and two planned starts; perception, OctoMap and visibility are not exercised.',
                     'Native node is constructed from deployed YAML; candidate IK/planning methods are invoked directly.',
                     'Mock arm execution only; gripper opening changes planning geometry.'])
        write_json(output/'report.json',report)
        assert report['completed'] and report['expected_rejection_false_accepts']==0
        assert len(executions)==4
        assert all(r['ik_accepted'] for r in rows if r['category']=='interior_fk')
        assert all(c['witness_valid'] for c in cases if c['category']=='random_valid_fk')
    finally:
        if obstacle_active:
            apply_obstacle(node,apply)
        adapter.close();selector._sensor_scene.close()
        executor.shutdown(timeout_sec=15.);thread.join(timeout=15.)
        assert not thread.is_alive()
        selector.destroy_node();node.destroy_client(apply)
