"""Opt-in native CPU IK fallback gates; uses mock hardware and real MoveIt."""
from dataclasses import replace
import json
import os
from pathlib import Path
import time

import pytest
from test_curobo_gpu import ros_node
from test_hybrid_gpu import native
from test_hybrid_target_matrix_gpu import apply_obstacle, baseline_state, write_json
from cleany_skill_executor.core.physical_grasp_selection import PoseValue

pytestmark=pytest.mark.skipif(os.environ.get('CLEANY_RUN_HYBRID_APPLIED')!='1',
    reason='Enable with make test-motion-hybrid-applied-gpu')


def test_native_cpu_fallback_recovers_known_targets_and_rejects_collision(native,tmp_path,monkeypatch):
    from moveit_msgs.srv import ApplyPlanningScene
    node,adapter,measured,_=native
    monkeypatch.setattr(adapter, '_hybrid_config',
                       replace(adapter._hybrid_config, cpu_ik_fallback=True))
    root=Path(__file__).resolve().parents[4]
    source=Path(os.environ.get('CLEANY_HYBRID_REFERENCE',str(
        root/'benchmark-results/hybrid-target-matrix-20261002')))
    output=Path(os.environ.get('CLEANY_HYBRID_APPLIED_OUTPUT',str(tmp_path)))
    output.mkdir(parents=True,exist_ok=True)
    cases={c['case_id']:c for c in json.loads((source/'targets.json').read_text())}
    adapter._config=replace(adapter._config,planning_timeout_sec=4.,planning_attempts=1,
                            velocity_scaling=.1,acceleration_scaling=.1)
    selected=('left/random_valid_fk/010','left/random_valid_fk/005','right/random_valid_fk/004')
    rows=[]
    for key in selected:
        case=cases[key];arm=case['arm']
        target=PoseValue(tuple(case['pose']['position']),tuple(case['pose']['orientation_xyzw']))
        adapter.set_current_state(baseline_state(measured,arm))
        adapter.set_deadline(time.monotonic()+480.)
        adapter.set_gripper_position(arm,case['opening'])
        result=adapter._cpu_fallback(arm,target,None)
        assert result is not None and adapter._feasible_alignment(result)
        assert adapter.state_is_valid(arm,result.solution)
        assert adapter.plan(arm,result.solution,None) is not None
        rows.append(dict(case_id=key,accepted=True,plan_passed=True,
            position_error_m=result.position_error_m,
            approach_error_degrees=result.approach_error_degrees,
            closing_error_degrees=result.closing_error_degrees))
    apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
    assert apply.wait_for_service(timeout_sec=30.)
    case=cases[selected[0]];arm=case['arm']
    target=PoseValue(tuple(case['pose']['position']),tuple(case['pose']['orientation_xyzw']))
    try:
        apply_obstacle(node,apply,target.position,(.2,.2,.2))
        adapter.set_current_state(baseline_state(measured,arm))
        adapter.set_deadline(time.monotonic()+480.)
        adapter.set_gripper_position(arm,case['opening'])
        assert adapter._cpu_fallback(arm,target,None) is None
        adapter.set_deadline(time.monotonic()-1.)
        with pytest.raises(InterruptedError):
            adapter._cpu_fallback(arm,target,None)
        write_json(output/'cpu-fallback-native.json',dict(
            scope='native CPU helper of HybridGraspAdapter / full MoveIt scene',
            recovered=3,plans_passed=3,obstacle_rejected=True,deadline_rejected=True,
            physical_motors_accessed=False,cuda_ik_executed=False,rows=rows))
    finally:
        apply_obstacle(node,apply)
        node.destroy_client(apply)
