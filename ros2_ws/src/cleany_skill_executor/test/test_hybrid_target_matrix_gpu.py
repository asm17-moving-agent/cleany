"""Large opt-in native CUDA/MoveIt target survey; only mock arm controllers run."""
from __future__ import annotations

from collections import Counter, defaultdict
from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cleany_skill_executor.core.alignment import axis_errors_degrees
from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, JointSolution, PoseValue, PregraspRequest,
)
from hybrid_target_matrix import (
    TargetCase, cartesian_offsets, joint_samples, orientation_offsets, summarize,
)
from test_curobo_gpu import ros_node
from test_hybrid_gpu import execute, native, wait

pytestmark = pytest.mark.skipif(
    os.environ.get('CLEANY_RUN_HYBRID_MATRIX') != '1',
    reason='Enable with make test-motion-hybrid-targets (actual Orin CUDA required)',
)


def write_json(path: Path, data: object) -> None:
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    temporary.replace(path)


def baseline_state(measured, arm: str, start_index: int = 0):
    state=deepcopy(measured[-1])
    for name in ARM_JOINT_NAMES['left']+ARM_JOINT_NAMES['right']:
        state.position[state.name.index(name)]=0.
    values=((0.,.65,.8,-.4,0.),(0.,1.1,1.7,-.9,0.))[start_index]
    for name,value in zip(ARM_JOINT_NAMES[arm],values,strict=True):
        state.position[state.name.index(name)]=value
    return state


def make_cases(adapter, measured) -> list[TargetCase]:
    cases=[]
    for arm_index,arm in enumerate(ARM_JOINT_NAMES):
        adapter.set_current_state(baseline_state(measured,arm))
        adapter.set_deadline(time.monotonic()+1200.)
        names=ARM_JOINT_NAMES[arm]
        samples=joint_samples([(adapter._joint_limits[n][0],adapter._joint_limits[n][1])
                               for n in names],seed=20261002+arm_index)
        def from_joints(category,number,values,opening=.4):
            adapter.set_gripper_position(arm,opening)
            solution=JointSolution(names,values)
            pose=adapter.forward_kinematics(arm,solution)
            return TargetCase(f'{arm}/{category}/{number:03d}',arm,category,pose,opening,
                              values,adapter.state_is_valid(arm,solution))
        for category,key in (('interior_fk','interior'),('joint_boundary','boundary')):
            cases.extend(from_joints(category,i,values) for i,values in enumerate(samples[key]))
        chosen=[]
        for values in samples['random_pool']:
            opening=(.2,.8,1.4)[len(chosen)%3]
            case=from_joints('random_valid_fk',len(chosen),values,opening)
            if case.witness_valid:
                chosen.append(case)
                if len(chosen)==32:
                    break
        assert len(chosen)==32, f'{arm}: insufficient collision-free reference samples'
        cases.extend(chosen)
        adapter.set_gripper_position(arm,.4)
        center=adapter.forward_kinematics(arm,JointSolution(names,(.1,.7,.85,-.4,.1)))
        for i,offset in enumerate(cartesian_offsets()):
            pose=PoseValue(tuple(p+d for p,d in zip(center.position,offset,strict=True)),
                           center.orientation_xyzw)
            cases.append(TargetCase(f'{arm}/cartesian_grid/{i:03d}',arm,'cartesian_grid',pose))
        for i,(axis,angle) in enumerate(orientation_offsets()):
            delta=np.zeros(3); delta[axis]=angle
            quaternion=(Rotation.from_quat(center.orientation_xyzw)*
                        Rotation.from_rotvec(delta)).as_quat()
            cases.append(TargetCase(f'{arm}/orientation/{i:03d}',arm,'orientation',
                PoseValue(center.position,tuple(float(v) for v in quaternion))))
        # Triangle inequality for a revolute-only chain: every position lies
        # within sum(|fixed translations|) of base_link, independent of IK.
        radius=sum(float(np.linalg.norm(j.translation)) for j in adapter._local_fk[arm].joints)+.5
        vectors=((1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),
                 (0,0,-1),(1,1,1),(-1,-1,1))
        for i,vector in enumerate(vectors):
            norm=math.sqrt(sum(v*v for v in vector))
            pose=PoseValue(tuple(radius*v/norm for v in vector),center.orientation_xyzw)
            cases.append(TargetCase(f'{arm}/outside_reach/{i:03d}',arm,'outside_reach',pose,
                                    expected_rejection=True))
        safe=[case for case in cases if case.arm==arm and case.category=='interior_fk'
              and case.witness_valid][:8]
        assert len(safe)==8
        for category in ('tcp_obstacle','base_obstacle'):
            cases.extend(replace(case,case_id=f'{arm}/{category}/{i:03d}',category=category,
                                 expected_rejection=True) for i,case in enumerate(safe))
    assert len(cases)==336
    return cases


def apply_obstacle(node, client, position=None, dimensions=None):
    obj=CollisionObject(id='hybrid_matrix_obstacle')
    obj.header.frame_id='base_link'
    obj.operation=CollisionObject.REMOVE if position is None else CollisionObject.ADD
    if position is not None:
        pose=Pose(); pose.position.x,pose.position.y,pose.position.z=position
        pose.orientation.w=1.
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(dimensions))]
        obj.primitive_poses=[pose]
    scene=PlanningScene(is_diff=True)
    scene.robot_state.is_diff=True
    scene.world.collision_objects=[obj]
    assert wait(node,client.call_async(ApplyPlanningScene.Request(scene=scene))).success


def base_collision_point(description: str) -> tuple[float, float, float]:
    import trimesh
    from ament_index_python.packages import get_package_share_directory
    collision=ET.fromstring(description).find("link[@name='base_link']/collision")
    mesh=collision.find('geometry/mesh')
    origin=collision.find('origin')
    if mesh is None:
        assert collision.find('geometry/box') is not None
        return tuple(float(v) for v in origin.get('xyz','0 0 0').split())
    package,relative=mesh.get('filename').removeprefix('package://').split('/',1)
    geometry=trimesh.load_mesh(Path(get_package_share_directory(package))/relative,process=False)
    vertex=np.asarray(geometry.vertices[0])*np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
    rotation=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' '))
    xyz=rotation.apply(vertex)+np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
    return tuple(float(v) for v in xyz)


def solver_count(adapter) -> int:
    return sum(len(cache[0]) for gpu in adapter._gpus.values()
               for cache in getattr(gpu._engine,'_batch_ik_caches',{}).values())


def test_diverse_hybrid_target_matrix(native, tmp_path, record_property):
    node,adapter,measured,actions=native
    adapter._config=replace(adapter._config,planning_timeout_sec=4.,planning_attempts=1,
                            state_validity_timeout_sec=5.)
    configured=Path(os.environ.get('CLEANY_HYBRID_MATRIX_OUTPUT',str(tmp_path/'matrix')))
    configured.mkdir(parents=True,exist_ok=True)
    apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
    assert apply.wait_for_service(timeout_sec=30.)
    began=time.monotonic()
    rows,batches,plans,executions=[],[],{},[]
    validity=Counter()
    obstacle_active=False
    original_call=adapter._call
    def traced_call(client,request,timeout):
        response=original_call(client,request,timeout)
        if client is adapter._validity_client:
            validity['requests']+=1
            validity['valid' if response.valid else 'invalid']+=1
        return response
    adapter._call=traced_call
    cases=make_cases(adapter,measured)
    write_json(configured/'targets.json',[case.json() for case in cases])
    metadata=dict(scope='native_hybrid_diverse_targets',sampling_seed=20261002,
        physical_motors_accessed=False,power_mode_changed=False,clock_limits_changed=False,
        sudo_used=False,runtime_model_sha256=hashlib.sha256(adapter._description.encode()).hexdigest(),
        runtime_model='SO-101 + measured head; 14 joints; fixed wheels',
        gpu_config=asdict(adapter._gpu_config),hybrid_config=asdict(adapter._hybrid_config),
        cpu_config=asdict(adapter._config),free_space_repeats=3,obstacle_repeats=1,
        plan_policy='Plan every accepted target once on repeat zero; IK-only on repeats one/two',
        reference_policy='FK witness is known feasible only when full MoveIt state validity passes',
        stationary_opposite_arm='All arm joints held at initial 0 rad',
        perception_models_executed=False,depth_octomap_executed=False,full_collection_flow=False,
        gripper_controllers_executed=False,
        starts=[[0.,.65,.8,-.4,0.],[0.,1.1,1.7,-.9,0.]],
        expected_target_evaluations=944,unique_conditions=336)
    write_json(configured/'metadata.json',metadata)
    grouped=defaultdict(list)
    for case in cases:
        grouped[(case.arm,case.category,case.opening)].append(case)
    base_point=base_collision_point(adapter._description)
    write_json(configured/'base_obstacle.json',{'position':base_point,'dimensions':[.12]*3})
    def checkpoint(phase):
        write_json(configured/'rows.json',rows)
        write_json(configured/'batches.json',batches)
        write_json(configured/'progress.json',dict(phase=phase,completed=len(rows),
            total=944,accepted=sum(row['ik_accepted'] for row in rows),
            plans_attempted=sum(row.get('plan_attempted',False) for row in rows),
            plans_passed=sum(row.get('plan_passed',False) for row in rows),
            elapsed_seconds=time.monotonic()-began))
        print(f'MATRIX {phase}: {len(rows)}/944 targets, '
              f'{sum(row["ik_accepted"] for row in rows)} IK accepted',flush=True)
    try:
        for (arm,category,opening),group in grouped.items():
            obstacle=category.endswith('_obstacle')
            repeats=1 if obstacle else 3
            for repeat in range(repeats):
                for offset in range(0,len(group),8):
                    chunk=group[offset:offset+8]
                    start_index=(offset//8)%2
                    state=baseline_state(measured,arm,start_index)
                    adapter.set_current_state(state)
                    adapter.set_deadline(time.monotonic()+1200.)
                    adapter.set_gripper_position(arm,opening)
                    start=JointSolution(ARM_JOINT_NAMES[arm],tuple(
                        dict(zip(state.name,state.position))[n] for n in ARM_JOINT_NAMES[arm]))
                    assert adapter.state_is_valid(arm,start), f'{arm}: invalid benchmark start'
                    if category=='tcp_obstacle':
                        positions=np.asarray([case.pose.position for case in chunk])
                        center=tuple((positions.min(axis=0)+positions.max(axis=0))/2)
                        dimensions=tuple(positions.max(axis=0)-positions.min(axis=0)+.16)
                        apply_obstacle(node,apply,center,dimensions)
                        obstacle_active=True
                    elif category=='base_obstacle':
                        apply_obstacle(node,apply,base_point,(.12,.12,.12))
                        obstacle_active=True
                    if obstacle:
                        assert all(not adapter.state_is_valid(arm,JointSolution(
                            ARM_JOINT_NAMES[arm],case.witness)) for case in chunk)
                    requests=[PregraspRequest(arm,case.pose,opening) for case in chunk]
                    solvers_before=solver_count(adapter)
                    validity_before=validity.copy()
                    counts_before={count:len(gpu._engine.timings.get('ik_batch_calls',[]))
                                   for count,gpu in adapter._gpus.items()}
                    batch_start=time.monotonic()
                    error=''
                    try:
                        adapter.prepare_pregrasps(requests)
                    except Exception as exc:
                        error=f'{type(exc).__name__}: {exc}'
                    elapsed=time.monotonic()-batch_start
                    statistics=deepcopy(adapter.last_ik_statistics)
                    last_seed=statistics['batch_calls'][-1]['seeds'] if statistics['batch_calls'] else None
                    diagnostics=(deepcopy(adapter._gpus[last_seed].last_batch_diagnostics)
                                 if last_seed is not None else [{}]*len(chunk))
                    gpu_calls=[]
                    for count,gpu in adapter._gpus.items():
                        gpu_calls.extend(dict(call,seed_count=count) for call in
                            gpu._engine.timings.get('ik_batch_calls',[])[counts_before.get(count,0):])
                    batch_index=len(batches)
                    batches.append(dict(arm=arm,category=category,opening=opening,repeat=repeat,
                        targets=len(chunk),seconds=elapsed,initializes_solver=solver_count(adapter)>solvers_before,
                        statistics=statistics,gpu_calls=gpu_calls,cpu_validity=dict(validity-validity_before),
                        infrastructure_error=error))
                    for index,(case,request) in enumerate(zip(chunk,requests,strict=True)):
                        result=adapter._pregrasp_cache.get(request) if not error else None
                        diagnostic=diagnostics[index] if index<len(diagnostics) else {}
                        row=dict(case_id=case.case_id,arm=arm,category=category,opening=opening,
                            repeat=repeat,start_index=start_index,batch_index=batch_index,
                            ik_accepted=result is not None,plan_attempted=False,plan_passed=False,
                            diagnostic=diagnostic,infrastructure_error=error,failure='')
                        if result is None:
                            row['failure']=('infrastructure_error' if error else
                                'cpu_collision_rejected' if diagnostic.get('pose_feasible',0)>0 else
                                'pose_alignment_rejected' if diagnostic.get('evaluated_variants',0)>0 else
                                'no_position_ik')
                        else:
                            adapter.set_gripper_position(arm,opening)
                            pose=adapter.forward_kinematics(arm,result.solution)
                            approach,closing=axis_errors_degrees(case.pose.orientation_xyzw,pose.orientation_xyzw)
                            row.update(position_error_m=math.dist(case.pose.position,pose.position),
                                approach_error_degrees=approach,closing_error_degrees=closing,
                                solution=list(result.solution.positions),pose=asdict(pose))
                            assert row['position_error_m']<=adapter._config.position_tolerance_m
                            assert approach<=adapter._config.approach_tolerance_degrees
                            assert closing<=adapter._config.closing_tolerance_degrees
                            assert adapter.state_is_valid(arm,result.solution)
                            assert not case.expected_rejection, f'Invalid target accepted: {case.case_id}'
                            if repeat==0:
                                row['plan_attempted']=True
                                planning_start=time.monotonic()
                                try:
                                    path=adapter.plan(arm,result.solution,start)
                                    row['plan_passed']=path is not None
                                    if path is not None:
                                        plans[case.case_id]=(path,state)
                                        row['path_points']=len(path.points)
                                        row['path_duration_seconds']=path.times[-1]
                                    else:
                                        row['failure']='cpu_plan_or_path_rejected'
                                except Exception as exc:
                                    row.update(failure='planning_infrastructure_error',
                                               infrastructure_error=f'{type(exc).__name__}: {exc}')
                                row['plan_seconds']=time.monotonic()-planning_start
                        rows.append(row)
                    if obstacle:
                        apply_obstacle(node,apply)
                        obstacle_active=False
                    checkpoint(f'{arm}/{category}/repeat{repeat}/batch{offset//8}')
        assert len(rows)==944
        # Execute up to eight successful trajectories per arm across categories.
        # Setup goals move only GenericSystem into the explicit planned start.
        for arm in ARM_JOINT_NAMES:
            eligible=defaultdict(list)
            for case in cases:
                if case.arm==arm and case.case_id in plans:
                    eligible[case.category].append(case)
            selected=[]
            while len(selected)<8 and any(eligible.values()):
                for category in sorted(eligible):
                    if eligible[category] and len(selected)<8:
                        selected.append(eligible[category].pop(0))
            for case in selected:
                path,state=plans[case.case_id]
                current=dict(zip(measured[-1].name,measured[-1].position))
                positions=dict(zip(state.name,state.position))
                setup=JointTrajectory(joint_names=list(ARM_JOINT_NAMES[arm]),points=[
                    JointTrajectoryPoint(positions=[current[n] for n in ARM_JOINT_NAMES[arm]],velocities=[0.]*5),
                    JointTrajectoryPoint(positions=[positions[n] for n in ARM_JOINT_NAMES[arm]],velocities=[0.]*5)])
                setup.points[-1].time_from_start.sec=2
                execute(node,actions[arm],setup,measured)
                from cleany_skill_executor.trajectory_io import path_to_message
                execution_start=time.monotonic()
                arrival_error,feedback=execute(node,actions[arm],path_to_message(path),measured)
                executions.append(dict(case_id=case.case_id,arm=arm,category=case.category,
                    seconds=time.monotonic()-execution_start,maximum_joint_error_rad=arrival_error,
                    feedback_messages=feedback,success=True))
                write_json(configured/'executions.json',executions)
                checkpoint(f'mock_execution/{case.case_id}')
            current=dict(zip(measured[-1].name,measured[-1].position))
            reset=JointTrajectory(joint_names=list(ARM_JOINT_NAMES[arm]),points=[
                JointTrajectoryPoint(positions=[current[n] for n in ARM_JOINT_NAMES[arm]],velocities=[0.]*5),
                JointTrajectoryPoint(positions=[0.]*5,velocities=[0.]*5)])
            reset.points[-1].time_from_start.sec=2
            execute(node,actions[arm],reset,measured)
    finally:
        if obstacle_active:
            apply_obstacle(node,apply)
        adapter._call=original_call
        node.destroy_client(apply)
        checkpoint('finished')
        report=summarize([case.json() for case in cases],rows,batches)
        report.update(metadata=metadata,seconds=time.monotonic()-began,
            cpu_validity_requests=dict(validity),executions=executions,
            completed=len(rows)==944,scope_limits=[
                'Not a perception/grasp/lift/return experiment; targets are synthetic.',
                'No sensor OctoMap or target OBB; obstacle tests use native MoveIt boxes.',
                'Random reference cases are filtered for validity; Cartesian/orientation goals have no feasibility oracle.',
                'One CPU plan per accepted distinct condition; repetitions are IK only.',
                'Arm mock execution only; gripper opening is query geometry, not a gripper command.'])
        by_arm={}
        for arm in ARM_JOINT_NAMES:
            by_arm[arm]=summarize([case.json() for case in cases if case.arm==arm],
                                 [row for row in rows if row['arm']==arm],
                                 [batch for batch in batches if batch['arm']==arm])
        report['by_arm']=by_arm
        write_json(configured/'report.json',report)
        record_property('matrix_output',str(configured))
    assert report['completed'] and report['infrastructure_errors']==0
    assert report['expected_rejection_false_accepts']==0
    assert len(executions)==16
