"""Synthetic proposal/MoveIt ports; no CUDA, DDS or physical motor access."""
from concurrent.futures import Future
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene, RobotTrajectory
from moveit_msgs.action import MoveGroup
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, REQUIRED_JOINT_NAMES, Candidate, GraspSelectionConfig,
    GraspSelector, InfrastructureError, JointSolution, PoseAlignment, PoseValue, PregraspRequest,
)
from cleany_skill_executor.hybrid_adapter import (
    HybridConfig, HybridGraspAdapter, _CandidateGpuAdapter, _CandidateRobotModel,
)
from cleany_skill_executor.curobo_model import RobotModel
from cleany_skill_executor.curobo_adapter import CuRoboGraspAdapter
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter


def completed(result):
    future = Future()
    future.set_result(result)
    return future


class Service:
    def __init__(self, callback):
        self.callback = callback

    def wait_for_service(self, **kwargs):
        return True

    def service_is_ready(self):
        return True

    def call_async(self, request):
        return completed(self.callback(request))


def pose_message(position):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.w = 1.
    return pose


def synthetic_urdf():
    """Toy translation/rotation chains; deliberately not an SO-101 model."""
    parts = ['<robot name="synthetic_pipeline"><link name="base_link"/>']
    for arm, names in ARM_JOINT_NAMES.items():
        parent = 'base_link'
        for i, name in enumerate(names):
            child = f'{arm}_link_{i}'
            kind = 'prismatic' if i < 3 else 'revolute'
            axis = ['1 0 0','0 1 0','0 0 1','1 0 0','0 1 0'][i]
            parts.append(f'<link name="{child}"/><joint name="{name}" type="{kind}">'
                         f'<parent link="{parent}"/><child link="{child}"/><axis xyz="{axis}"/>'
                         '<limit lower="-2" upper="2" velocity="1" effort="1"/></joint>')
            parent = child
        parts.append(f'<link name="{arm}_grasp_tcp"/><joint name="{arm}_tcp_fixed" type="fixed">'
                     f'<parent link="{parent}"/><child link="{arm}_grasp_tcp"/></joint>')
        parts.append(f'<link name="{arm}_jaw"/><joint name="{arm}_gripper_joint" type="revolute">'
                     f'<parent link="{parent}"/><child link="{arm}_jaw"/><axis xyz="1 0 0"/>'
                     '<limit lower="-2" upper="2" velocity="1" effort="1"/></joint>')
    return ''.join(parts)+'</robot>'


def synthetic_head_urdf():
    xml = synthetic_urdf().removesuffix('</robot>')
    parent = 'base_link'
    for name in ('head_pan_joint', 'head_tilt_joint'):
        child = name.removesuffix('_joint')+'_link'
        xml += (f'<link name="{child}"/><joint name="{name}" type="revolute">'
                f'<parent link="{parent}"/><child link="{child}"/><axis xyz="0 1 0"/>'
                '<limit lower="-1" upper="1" velocity="1" effort="1"/></joint>')
        parent = child
    return xml+'</robot>'


class CpuPlanner:
    def __init__(self, trace):
        self.trace = trace

    def wait_for_server(self, **kwargs):
        return True

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, **kwargs):
        assert goal.planning_options.plan_only
        assert goal.request.planner_id == 'RRTConnectkConfigDefault'
        self.trace.append('cpu_rrtconnect_request')
        arm = goal.request.group_name.split('_')[0]
        start = dict(zip(goal.request.start_state.joint_state.name, goal.request.start_state.joint_state.position))
        target = {c.joint_name: c.position for c in goal.request.goal_constraints[0].joint_constraints}
        names = ARM_JOINT_NAMES[arm]
        path = JointTrajectory(joint_names=list(names), points=[
            JointTrajectoryPoint(positions=[start[n] for n in names], velocities=[0.]*5),
            JointTrajectoryPoint(positions=[target[n] for n in names], velocities=[0.]*5)])
        path.points[-1].time_from_start.sec = 1
        result = MoveGroup.Result(error_code=MoveItErrorCodes(val=1),
                                  planned_trajectory=RobotTrajectory(joint_trajectory=path))
        handle = NS(accepted=True,
                    get_result_async=lambda: completed(NS(status=4, result=result)),
                    cancel_goal_async=lambda: completed(NS()))
        return completed(handle)


class SyntheticGpu:
    """Explicit test double with analytic translational IK, never a CUDA result."""
    def __init__(self, count, trace, primary_empty=False):
        self.count, self.trace, self.primary_empty = count, trace, primary_empty
        self._engine = NS(close=lambda: trace.append('gpu_engine_closed'))

    def set_current_state(self, state):
        self.state = state

    def set_gripper_position(self, arm, opening):
        self.opening = opening

    def set_deadline(self, deadline):
        self.deadline = deadline

    def cancel_active(self):
        self.trace.append('gpu_cancel')

    def solve_aligned_batch(self, arm, targets, seeds):
        self.trace.append({'gpu_test_double': self.count, 'batch': len(targets), 'opening': self.opening})
        if self.primary_empty and self.count == 16:
            return [None]*len(targets)
        return [PoseAlignment(JointSolution(ARM_JOINT_NAMES[arm], (*t.position, 0., 0.)),
                              t, 0., 0., 0.) for t in targets]


def make_adapter(*, primary_empty=False, moveit_config=None, hybrid_config=None):
    trace = []
    fk = Service(lambda req: NS(error_code=MoveItErrorCodes(val=1), pose_stamped=[
        PoseStamped(pose=pose_message(tuple(dict(zip(req.robot_state.joint_state.name,
            req.robot_state.joint_state.position))[n] for n in ARM_JOINT_NAMES[req.fk_link_names[0].split('_')[0]][:3])))]))
    valid = Service(lambda req: NS(valid=True,
        constraint_result=[NS(result=True) for _ in req.constraints.visibility_constraints]))
    def no_cpu_ik(request):
        pytest.fail('Candidate IK must not invoke CPU KDL')
    adapter = HybridGraspAdapter(object(), moveit_config or MoveItAdapterConfig(validate_motion_paths=True),
        hybrid_config=hybrid_config or HybridConfig(),
        gpu_factory=lambda description, cfg: SyntheticGpu(cfg.num_seeds, trace, primary_empty),
        ik_client=Service(no_cpu_ik), fk_client=fk, validity_client=valid,
        plan_client=CpuPlanner(trace))
    # The proposal double is not a robot model. Its CPU FK is explicitly synthetic.
    adapter.set_robot_description(synthetic_urdf())
    state = JointState(name=list(REQUIRED_JOINT_NAMES), position=[0.]*12)
    for i, name in enumerate(state.name):
        if name in ARM_JOINT_NAMES['left'][:3]:
            state.position[i] = (.35, .35, .3)[ARM_JOINT_NAMES['left'].index(name)]
    adapter.set_current_state(state)
    return adapter, trace


def requests(count=8, opening=.4):
    return [PregraspRequest('left', PoseValue((.4+i*.001, .28, .3), (0.,0.,0.,1.)), opening)
            for i in range(count)]


def test_grasp_first_gpu_batch_is_final_pose_then_seeded_pregrasp_through_scene_port():
    from cleany_skill_executor.planning_scene import SceneAwarePort
    adapter, trace = make_adapter(hybrid_config=HybridConfig(
        progressive_pregrasp=True, defer_candidate_ik_retries=True,
        group_gripper_openings=True, pregrasp_retry_passes=0, cpu_ik_fallback=False))
    scene = NS(disallow_target_contacts=lambda: None, allow_contacts_for=lambda arm: None)
    gpu = adapter._gpu(16)
    solve = gpu.solve_aligned_batch
    observed = []
    def inspect(arm, targets, seeds):
        observed.append((tuple(t.position for t in targets), tuple(seeds)))
        return solve(arm, targets, seeds)
    gpu.solve_aligned_batch = inspect
    selector = GraspSelector(SceneAwarePort(adapter, scene), GraspSelectionConfig(
        grasp_ik_first=True, maximum_candidates=2, stop_after_first_feasible=True))
    values = [Candidate((.4+i*.001,.2,.3), (0.,-1.,0.), 1.-i*.01, source_index=17+i)
              for i in range(2)]
    selected = selector.select(values, required_arm='left')
    assert selected is not None and selected.candidate_index == 17
    assert observed[0][0] == tuple(v.position for v in values)
    assert observed[0][1] == (None, None)
    assert observed[1][0][0] == pytest.approx((.4,.28,.3))
    assert observed[1][1] == (selected.grasp,)
    emitted = adapter.last_ik_statistics['ik_diagnostics']
    assert [d['stage'] for d in emitted] == ['GRASP_IK', 'GRASP_IK', 'PREGRASP_IK']
    assert emitted[1]['outcome'] == 'deferred_proposal'
    assert adapter.last_ik_statistics['batch_calls'][0]['stage'] == 'grasp'
    assert len([c for c in trace if c == 'cpu_rrtconnect_request']) == 2


@pytest.mark.parametrize('defer_retries', [False, True])
def test_grasp_first_failure_moves_on_without_seed32_or_pregrasp(defer_retries):
    from cleany_skill_executor.planning_scene import SceneAwarePort
    adapter, _ = make_adapter(primary_empty=True, hybrid_config=HybridConfig(
        progressive_pregrasp=True, defer_candidate_ik_retries=defer_retries,
        group_gripper_openings=True, pregrasp_retry_passes=0, cpu_ik_fallback=False))
    scene = NS(disallow_target_contacts=lambda: None, allow_contacts_for=lambda arm: None)
    selector = GraspSelector(SceneAwarePort(adapter, scene), GraspSelectionConfig(
        grasp_ik_first=True, maximum_candidates=2, stop_after_first_feasible=True))
    values = [Candidate((.4+i*.001,.2,.3), (0.,-1.,0.), 1.-i*.01, source_index=i)
              for i in range(2)]
    selected = selector.select(values, required_arm='left')
    assert selected is None
    records = adapter.last_ik_statistics['ik_diagnostics']
    assert [r['candidate_index'] for r in records] == [0, 1]
    assert all(r['stage'] == 'GRASP_IK' and r['seeds'] == 16 for r in records)
    assert len(adapter.last_ik_statistics['batch_calls']) == 1
    assert adapter.last_ik_statistics['fallback_batches'] == 0
    assert set(adapter._gpus) == {16}
    assert adapter.last_ik_statistics['cpu_ik_fallback_calls'] == []


def test_grasp_first_seeded_pregrasp_failure_moves_on_without_retrying_either_endpoint():
    from cleany_skill_executor.planning_scene import SceneAwarePort
    adapter, trace = make_adapter(hybrid_config=HybridConfig(
        progressive_pregrasp=True, defer_candidate_ik_retries=True,
        group_gripper_openings=True, pregrasp_retry_passes=0, cpu_ik_fallback=False))
    gpu = adapter._gpu(16)
    solve = gpu.solve_aligned_batch
    observed = []
    def fail_pregrasp(arm, targets, seeds):
        observed.append((tuple(targets), tuple(seeds)))
        return [None]*len(targets) if any(seed is not None for seed in seeds) else solve(
            arm, targets, seeds)
    gpu.solve_aligned_batch = fail_pregrasp
    values = [Candidate((.4+i*.001,.2,.3), (0.,-1.,0.), 1.-i*.01, source_index=i)
              for i in range(2)]
    scene = NS(disallow_target_contacts=lambda: None, allow_contacts_for=lambda arm: None)
    selected = GraspSelector(SceneAwarePort(adapter, scene),
                             GraspSelectionConfig(grasp_ik_first=True)).select(
        values, required_arm='left')
    assert selected is None
    assert len(observed) == 3
    assert observed[0][1] == (None, None)
    assert [seeds[0].positions[:3] for _, seeds in observed[1:]] == [v.position for v in values]
    for (targets, _), expected in zip(observed[1:], [(.4,.28,.3), (.401,.28,.3)], strict=True):
        assert targets[0].position == pytest.approx(expected)
    assert set(adapter._gpus) == {16}
    assert adapter.last_ik_statistics['fallback_batches'] == 0
    assert adapter.last_ik_statistics['cpu_ik_fallback_calls'] == []
    assert 'cpu_rrtconnect_request' not in trace


def test_primary_pass_tries_other_candidates_before_expensive_retry():
    adapter, trace = make_adapter(hybrid_config=HybridConfig(
        progressive_pregrasp=True, defer_candidate_ik_retries=True,
        pregrasp_retry_passes=0, cpu_ik_fallback=False))
    items = requests(3)
    gpu = adapter._gpu(16)
    full = gpu.solve_aligned_batch
    gpu.solve_aligned_batch = lambda arm, targets, seeds: [None, *full(arm, targets, seeds)[1:]]
    adapter.prepare_pregrasps(items)
    adapter.set_gripper_position('left', .4)
    assert adapter.begin_candidate_pass(False)
    assert adapter.solve_aligned_ik('left', items[0].target, None) is None
    assert adapter.solve_aligned_ik('left', items[1].target, None) is not None
    assert adapter.last_ik_statistics['fallback_batches'] == 0
    assert 32 not in adapter._gpus


def test_deferred_pass_preserves_retry_when_every_primary_fails():
    adapter, _ = make_adapter(primary_empty=True, hybrid_config=HybridConfig(
        progressive_pregrasp=True, defer_candidate_ik_retries=True,
        pregrasp_retry_passes=0, cpu_ik_fallback=False))
    items = requests(2)
    adapter.prepare_pregrasps(items)
    adapter.set_gripper_position('left', .4)
    assert adapter.begin_candidate_pass(False)
    assert all(adapter.solve_aligned_ik('left', r.target, None) is None for r in items)
    assert adapter.last_ik_statistics['fallback_batches'] == 0
    assert adapter.begin_candidate_pass(True)
    assert adapter.solve_aligned_ik('left', items[0].target, None) is not None
    assert adapter.last_ik_statistics['fallback_batches'] == 1
    adapter.set_current_state(adapter._current_state)
    assert not adapter._primary_only and not adapter._deferred_ik
    assert not adapter._pregrasp_alignment_pending


def test_warmup_discards_solutions_and_reuses_engine_across_state_updates():
    adapter, _ = make_adapter()
    calls = []
    gpu = adapter._gpu(16)
    gpu._seed_bases = adapter._seed_bases
    gpu._engine.solve_batch = lambda *args: calls.append(args) or [['invalid discarded proposal']]
    adapter.warmup(['left', 'right'], include_aim=False)
    assert [args[0] for args in calls] == ['left', 'right']
    assert all(len(args[2]) == 8 and args[-1] == {} for args in calls)
    assert not adapter._pregrasp_cache
    assert adapter.last_ik_statistics['primary_batches'] == 0
    engine = gpu._engine
    adapter.set_current_state(adapter._current_state)
    assert adapter._gpu(16)._engine is engine


@pytest.mark.parametrize('progressive', [False, True])
def test_pregrasp_tolerance_reaches_batches_and_cached_gpu_without_relaxing_grasp(progressive):
    adapter, trace = make_adapter(
        moveit_config=MoveItAdapterConfig(position_tolerance_m=.002,
                                         pregrasp_position_tolerance_m=.010),
        hybrid_config=HybridConfig(progressive_pregrasp=progressive,
                                   pregrasp_retry_passes=0, cpu_ik_fallback=False))
    tolerances = []

    class OffsetGpu(SyntheticGpu):
        def solve_aligned_batch(self, arm, targets, seeds):
            tolerances.append(self._config.position_tolerance_m)
            values = []
            for target in targets:
                actual = PoseValue((target.position[0]+.008, *target.position[1:]),
                                   target.orientation_xyzw)
                solution = JointSolution(ARM_JOINT_NAMES[arm], (*actual.position, 0., 0.))
                values.append(PoseAlignment(solution, actual, .008, 0., 0.))
            return values

    def offset_factory(description, cfg):
        assert cfg.ik_position_threshold_m == .010
        return OffsetGpu(cfg.num_seeds, trace)

    adapter._gpu_factory = offset_factory
    items = requests(2)
    adapter.prepare_pregrasps(items)
    adapter.set_gripper_position('left', .4)
    alignments = [adapter.solve_pregrasp_ik('left', item.target) for item in items]
    assert all(value is not None for value in alignments)
    assert [value.position_error_m for value in alignments] == pytest.approx([.008, .008])
    assert tolerances == [.010]
    assert adapter._config.position_tolerance_m == .002

    final_target = PoseValue((.5, .28, .3), (0., 0., 0., 1.))
    assert adapter.solve_aligned_ik('left', final_target, alignments[0].solution) is None
    assert tolerances == [.010, .002, .002]
    assert all(gpu._config.position_tolerance_m == .002 for gpu in adapter._gpus.values())


def test_batch8_seed16_and_cache_are_used_without_cpu_ik():
    adapter, trace = make_adapter()
    items = requests()
    adapter.prepare_pregrasps(items)
    assert adapter.last_ik_statistics['primary_batches'] == 1
    assert adapter.last_ik_statistics['fallback_batches'] == 0
    before = len(trace)
    adapter.set_gripper_position('left', .4)
    assert all(adapter.solve_aligned_ik('left', r.target, None) for r in items)
    assert len(trace) == before
    assert trace[0]['batch'] == 8


def test_zero_primary_candidates_retry_same_targets_with32():
    adapter, trace = make_adapter(primary_empty=True)
    adapter.prepare_pregrasps(requests())
    calls = [t for t in trace if isinstance(t, dict)]
    assert [t['gpu_test_double'] for t in calls] == [16, 16, 32]
    assert adapter.last_ik_statistics['primary_retry_targets'] == 8
    assert adapter.last_ik_statistics['pregrasp_retry_arms'] == ['left']
    assert all(adapter._pregrasp_cache.values())


def test_one_accepted_candidate_does_not_prevent_retries_of_failed_candidates():
    adapter, trace = make_adapter()
    adapter.state_is_valid = lambda arm, q: q.positions[0] == .4
    adapter.prepare_pregrasps(requests())
    assert sum(v is not None for v in adapter._pregrasp_cache.values()) == 1
    assert adapter.last_ik_statistics['fallback_batches'] == 1
    assert adapter.last_ik_statistics['primary_retry_targets'] == 7
    assert adapter.last_ik_statistics['fallback_targets'] == 7
    calls = adapter.last_ik_statistics['batch_calls']
    assert [call['targets'] for call in calls] == [8,7,7]
    assert all(call['padded_targets']==8 for call in calls)
    assert adapter._pregrasp_cache[requests()[0]] is not None


def test_same_seed_retry_stops_when_failed_subset_is_recovered():
    adapter, trace = make_adapter()
    gpu = adapter._gpu(16)
    original = gpu.solve_aligned_batch
    count = 0
    def delayed(arm, targets, seeds):
        nonlocal count
        count += 1
        values=original(arm,targets,seeds)
        return [values[0],*([None]*(len(values)-1))] if count==1 else values
    gpu.solve_aligned_batch=delayed
    adapter.prepare_pregrasps(requests())
    assert all(adapter._pregrasp_cache.values())
    assert adapter.last_ik_statistics['primary_retry_targets']==7
    assert adapter.last_ik_statistics['fallback_batches']==0
    assert len([v for v in trace if isinstance(v,dict)])==2


def empty_gpu_proposals(adapter):
    for count in (16,32):
        adapter._gpu(count).solve_aligned_batch = lambda arm, targets, seeds: [None]*len(targets)


@pytest.mark.parametrize('progressive', [False, True])
def test_default_gpu_failure_skips_cpu_ik_for_pregrasp_and_grasp(progressive):
    adapter, _ = make_adapter(hybrid_config=HybridConfig(
        progressive_pregrasp=progressive, pregrasp_retry_passes=0))
    empty_gpu_proposals(adapter)
    item = requests(1)[0]
    adapter.prepare_pregrasps([item])
    adapter.set_gripper_position(item.arm, item.gripper_position)
    assert adapter.solve_pregrasp_ik(item.arm, item.target) is None
    seed = JointSolution(ARM_JOINT_NAMES[item.arm], (*item.target.position, 0., 0.))
    assert adapter.solve_aligned_ik(item.arm, item.target, seed) is None
    assert adapter.last_ik_statistics['fallback_batches'] > 0
    assert adapter.last_ik_statistics['cpu_ik_fallback_calls'] == []
    assert adapter.last_ik_statistics['cpu_ik_fallback_enabled'] is False
    assert adapter._cpu_ik is None
    assert all(record['backend'] != 'cpu_fallback'
               for record in adapter.last_ik_statistics['ik_diagnostics'])


@pytest.mark.parametrize('reject',['none','alignment','collision','fk'])
def test_cpu_fallback_uses_same_pose_and_scene_gates(monkeypatch,reject):
    adapter, _ = make_adapter(hybrid_config=HybridConfig(cpu_ik_fallback=True))
    empty_gpu_proposals(adapter)
    observed=[]
    def cpu_ik(cpu,arm,target,seed):
        observed.append((cpu,arm,target,seed))
        alignment=PoseAlignment(JointSolution(ARM_JOINT_NAMES[arm],(*target.position,0.,0.)),
                                target,0.,0.,0.)
        if reject=='alignment':
            return PoseAlignment(alignment.solution,target,0.,90.,0.)
        return alignment
    monkeypatch.setattr(MoveItGraspAdapter,'solve_aligned_ik',cpu_ik)
    if reject=='collision':
        adapter.state_is_valid=lambda *args: False
    elif reject=='fk':
        adapter.forward_kinematics=lambda *args: PoseValue((9.,9.,9.),(0.,0.,0.,1.))
    adapter.set_deadline(float('inf'))
    adapter.prepare_pregrasps(requests(1,.6))
    assert bool(adapter._pregrasp_cache[requests(1,.6)[0]])==(reject=='none')
    assert len(observed)==1
    cpu=observed[0][0]
    assert cpu._config==adapter._config
    assert cpu._deadline==adapter._deadline
    assert cpu._gripper_positions=={'left':.6}
    assert cpu._current_state.name==adapter._current_state.name
    assert cpu._ik_client is adapter._ik_client
    assert len(adapter.last_ik_statistics['cpu_ik_fallback_calls'])==1
    diagnostic = adapter.last_ik_statistics['ik_diagnostics'][-1]
    expected = {'none': 'accepted', 'alignment': 'pose_rejected',
                'collision': 'final_collision_rejected', 'fk': 'final_fk_rejected'}
    assert diagnostic['backend'] == 'cpu_fallback'
    assert diagnostic['outcome'] == expected[reject]
    if reject == 'alignment':
        assert diagnostic['proposal_alignment']['approach_error_degrees'] == 90.
        assert diagnostic['proposal_alignment']['violations'] == ['approach_error_degrees']


def test_cpu_fallback_recovers_single_grasp_and_preserves_seed(monkeypatch):
    adapter, _ = make_adapter(hybrid_config=HybridConfig(cpu_ik_fallback=True))
    empty_gpu_proposals(adapter)
    target=requests(1)[0].target
    seed=JointSolution(ARM_JOINT_NAMES['left'],(*target.position,0.,0.))
    observed=[]
    def cpu_ik(cpu,arm,value,passed_seed):
        observed.append(passed_seed)
        return PoseAlignment(seed,value,0.,0.,0.)
    monkeypatch.setattr(MoveItGraspAdapter,'solve_aligned_ik',cpu_ik)
    adapter.set_gripper_position('left',.4)
    assert adapter.solve_aligned_ik('left',target,seed) is not None
    assert observed==[seed]
    assert adapter.last_ik_statistics['grasp_retry_calls']==1


@pytest.mark.parametrize('failure',[InterruptedError,InfrastructureError])
def test_cpu_fallback_propagates_cancel_and_infrastructure_errors(monkeypatch,failure):
    adapter, _ = make_adapter(hybrid_config=HybridConfig(cpu_ik_fallback=True))
    empty_gpu_proposals(adapter)
    def fail(*args):
        if failure is InterruptedError:
            adapter.cancel_active()
        raise failure('injected')
    monkeypatch.setattr(MoveItGraspAdapter,'solve_aligned_ik',fail)
    with pytest.raises(failure):
        adapter.prepare_pregrasps(requests(1))
    assert not any(adapter._pregrasp_cache.values())


def test_cpu_fallback_checks_owner_cancellation_and_head_state(monkeypatch):
    adapter, _ = make_adapter(hybrid_config=HybridConfig(cpu_ik_fallback=True))
    state=deepcopy(adapter._current_state)
    adapter.set_robot_description(synthetic_head_urdf())
    state.name.extend(['head_pan_joint','head_tilt_joint']);state.position.extend([.1,.4])
    adapter.set_current_state(state)
    empty_gpu_proposals(adapter)
    def cpu_ik(cpu,arm,target,seed):
        assert dict(zip(cpu._current_state.name,cpu._current_state.position))['head_tilt_joint']==.4
        adapter.cancel_active()
        cpu._canceled=False
        cpu._check_active()
    monkeypatch.setattr(MoveItGraspAdapter,'solve_aligned_ik',cpu_ik)
    with pytest.raises(InterruptedError):
        adapter.prepare_pregrasps(requests(1))


@pytest.mark.parametrize('recovers',[False,True])
def test_cpu_path_retry_is_bounded_and_keeps_start_goal(monkeypatch,recovers):
    adapter, _ = make_adapter()
    goal=JointSolution(ARM_JOINT_NAMES['left'],(.4,.28,.3,0.,0.))
    observed=[]
    expected=object()
    def plan(cpu,arm,passed_goal,start):
        observed.append((arm,passed_goal,start))
        return expected if recovers and len(observed)==2 else None
    monkeypatch.setattr(MoveItGraspAdapter,'plan',plan)
    assert adapter.plan('left',goal,goal) is (expected if recovers else None)
    assert observed==[('left',goal,goal)]*2
    assert len(adapter.last_ik_statistics['planning_calls'])==2


@pytest.mark.parametrize('failure',[InterruptedError,InfrastructureError])
def test_cpu_path_errors_are_not_retried(monkeypatch,failure):
    adapter, _ = make_adapter()
    observed=[]
    def fail(*args):
        observed.append(True)
        raise failure('injected')
    monkeypatch.setattr(MoveItGraspAdapter,'plan',fail)
    goal=JointSolution(ARM_JOINT_NAMES['left'],(.4,.28,.3,0.,0.))
    with pytest.raises(failure):
        adapter.plan('left',goal,None)
    assert len(observed)==1


def test_cpu_path_retry_respects_deadline_and_can_be_disabled(monkeypatch):
    adapter, _ = make_adapter(hybrid_config=HybridConfig(planning_retries=0))
    goal=JointSolution(ARM_JOINT_NAMES['left'],(.4,.28,.3,0.,0.))
    monkeypatch.setattr(MoveItGraspAdapter,'plan',lambda *args: None)
    assert adapter.plan('left',goal,None) is None
    assert len(adapter.last_ik_statistics['planning_calls'])==1
    adapter.set_deadline(0.)
    with pytest.raises(InterruptedError):
        adapter.plan('left',goal,None)


def test_cpu_fallback_is_cached_and_duplicate_requests_are_not_repeated(monkeypatch):
    adapter, _ = make_adapter(hybrid_config=HybridConfig(cpu_ik_fallback=True))
    empty_gpu_proposals(adapter)
    observed=[]
    def cpu_ik(cpu,arm,target,seed):
        observed.append(target)
        return None
    monkeypatch.setattr(MoveItGraspAdapter,'solve_aligned_ik',cpu_ik)
    item=requests(1)[0]
    adapter.prepare_pregrasps([item]*8)
    assert observed==[item.target]
    adapter.set_gripper_position('left',item.gripper_position)
    assert adapter.solve_aligned_ik('left',item.target,None) is None
    assert len(observed)==1


def test_gpu_initialization_error_does_not_invoke_cpu_fallback(monkeypatch):
    adapter, _ = make_adapter(hybrid_config=HybridConfig(cpu_ik_fallback=True))
    def fail(*args):
        raise RuntimeError('injected CUDA failure')
    adapter._gpu_factory=fail
    with pytest.raises(InfrastructureError,match='GPU IK initialization failed'):
        adapter.prepare_pregrasps(requests(1))
    assert adapter._cpu_ik is None


def test_retry_groups_keep_arm_and_gripper_opening():
    adapter, _ = make_adapter(primary_empty=True)
    left=requests(2,.4)
    right=[PregraspRequest('right',r.target,.6) for r in requests(3)]
    adapter.prepare_pregrasps([*left,*right])
    calls=adapter.last_ik_statistics['batch_calls']
    assert [(c['arm'],c['gripper_position'],c['targets']) for c in calls]==[
        ('left',.4,2),('right',.6,3)]*3
    assert all(adapter._pregrasp_cache.values())
    assert all(value.solution.names==ARM_JOINT_NAMES[key.arm]
               for key,value in adapter._pregrasp_cache.items())


def test_different_openings_are_separate_padded_batches():
    adapter, trace = make_adapter()
    adapter.prepare_pregrasps([*requests(3, .4), *requests(2, .6)])
    calls = adapter.last_ik_statistics['batch_calls']
    assert [(c['targets'], c['padded_targets'], c['gripper_position']) for c in calls] == [(3,8,.4),(2,8,.6)]


def test_progressive_mixed_openings_share_primary_batch_and_keep_actual_validation():
    config = HybridConfig(progressive_pregrasp=True, group_gripper_openings=True,
        adaptive_batch_size=True, pregrasp_retry_passes=0, cpu_ik_fallback=False)
    adapter, trace = make_adapter(hybrid_config=config)
    items = [requests(1, opening)[0] for opening in (.4, .6, .8)]
    # Use distinct poses so each state must be checked at its own opening.
    items = [PregraspRequest(r.arm, PoseValue((.4+i*.001,.28,.3),(0.,0.,0.,1.)),
                            r.gripper_position) for i,r in enumerate(items)]
    expected = {r.target.position: r.gripper_position for r in items}
    observed = []
    def valid(arm, solution):
        opening = adapter._gripper_positions[arm]
        observed.append((solution.positions[:3], opening))
        return opening == expected[solution.positions[:3]]
    adapter.state_is_valid = valid
    adapter.prepare_pregrasps(items)
    assert adapter.last_ik_statistics['primary_batches'] == 0
    adapter.set_gripper_position('left', .4)
    assert adapter.solve_aligned_ik('left', items[0].target, None) is not None
    calls = [c for c in trace if isinstance(c, dict)]
    assert len(calls) == 1 and calls[0]['batch'] == 3 and calls[0]['opening'] == .8
    assert observed == [(items[0].target.position,items[0].gripper_position)]
    assert adapter._gripper_positions['left'] == .4
    assert adapter.last_ik_statistics['batch_calls'][0]['padded_targets'] == 4
    adapter.set_gripper_position('left', .6)
    assert adapter.solve_aligned_ik('left', items[1].target, None) is not None
    assert observed == [(r.target.position,r.gripper_position) for r in items[:2]]
    assert len([c for c in trace if isinstance(c, dict)]) == 1


def test_deferred_proposal_is_rejected_at_its_actual_opening_before_returning():
    adapter, _ = make_adapter(hybrid_config=HybridConfig(progressive_pregrasp=True,
        group_gripper_openings=True, pregrasp_retry_passes=0, cpu_ik_fallback=False))
    items = [*requests(1,.4), *requests(1,.8)]
    adapter.state_is_valid = lambda arm, q: adapter._gripper_positions[arm] == .4
    adapter.prepare_pregrasps(items)
    adapter.set_gripper_position('left', .4)
    assert adapter.solve_aligned_ik('left', items[0].target, None)
    assert adapter._pregrasp_cache[items[1]] is not None  # Unchecked proposal only.
    adapter.set_gripper_position('left', .8)
    assert adapter.solve_aligned_ik('left', items[1].target, None) is None
    assert adapter.last_ik_statistics['fallback_targets'] == 1


def test_progressive_retry_only_computes_requested_failure_and_is_cached():
    adapter, trace = make_adapter(primary_empty=True, hybrid_config=HybridConfig(
        progressive_pregrasp=True, adaptive_batch_size=True,
        pregrasp_retry_passes=0, cpu_ik_fallback=False))
    items = requests(3)
    adapter.prepare_pregrasps(items)
    adapter.set_gripper_position('left', .4)
    assert adapter.solve_aligned_ik('left', items[0].target, None)
    calls = adapter.last_ik_statistics['batch_calls']
    assert [(c['seeds'],c['targets'],c['padded_targets']) for c in calls] == [(16,3,4),(32,1,1)]
    assert adapter.last_ik_statistics['primary_retry_batches'] == 0
    assert adapter._pregrasp_cache[items[1]] is None
    before = len(trace)
    assert adapter.solve_aligned_ik('left', items[0].target, None)
    assert len(trace) == before


def test_deferred_alignment_gets_primary_search_only_when_its_candidate_is_reached():
    adapter, _ = make_adapter(hybrid_config=HybridConfig(
        progressive_pregrasp=True, group_gripper_openings=True,
        pregrasp_retry_passes=0, cpu_ik_fallback=False))
    items = requests(3)
    gpu = adapter._gpu(16)
    full_search = gpu.solve_aligned_batch
    searched = []

    def search(arm, targets, seeds):
        searched.append(list(targets))
        results = full_search(arm, targets, seeds)
        return [results[0], *([None] * (len(results)-1))]

    gpu.solve_aligned_batch = search
    adapter.prepare_pregrasps(items)
    adapter.set_gripper_position('left', .4)
    assert adapter.solve_aligned_ik('left', items[0].target, None) is not None
    assert searched == [[r.target for r in items]]
    assert adapter._pregrasp_alignment_pending == set(items[1:])
    assert adapter.solve_aligned_ik('left', items[1].target, None) is not None
    assert searched == [[r.target for r in items], [items[1].target]]
    assert adapter._pregrasp_alignment_pending == {items[2]}
    assert adapter.last_ik_statistics['fallback_batches'] == 0
    assert adapter.solve_aligned_ik('left', items[1].target, None) is not None
    assert len(searched) == 2
    adapter.set_current_state(adapter._current_state)
    assert not adapter._pregrasp_alignment_pending


def test_deferred_alignment_keeps_gpu_seed_retry_after_primary_failure():
    adapter, _ = make_adapter(primary_empty=True, hybrid_config=HybridConfig(
        progressive_pregrasp=True, pregrasp_retry_passes=0, cpu_ik_fallback=False))
    items = requests(3)
    adapter.prepare_pregrasps(items)
    adapter.set_gripper_position('left', .4)
    assert adapter.solve_aligned_ik('left', items[0].target, None) is not None
    assert adapter.solve_aligned_ik('left', items[1].target, None) is not None
    assert [(call['seeds'], call['targets'])
            for call in adapter.last_ik_statistics['batch_calls']] == [
        (16, 3), (32, 1), (16, 1), (32, 1)]


def test_progressive_cancel_invalidates_pending_work_and_mixed_openings_restore_on_failure():
    adapter, _ = make_adapter(hybrid_config=HybridConfig(progressive_pregrasp=True,
        group_gripper_openings=True, pregrasp_retry_passes=0, cpu_ik_fallback=False))
    items = [*requests(1,.4), *requests(1,.6)]
    adapter.prepare_pregrasps(items)
    adapter.set_gripper_position('left', .4)
    def fail(*args):
        adapter.set_gripper_position('left', .6)
        raise InfrastructureError('injected scene error')
    adapter._gpu(16).solve_aligned_batch = fail
    with pytest.raises(InfrastructureError):
        adapter.solve_aligned_ik('left', items[0].target, None)
    assert adapter._gripper_positions['left'] == .4
    assert not adapter._pregrasp_cache
    adapter.set_current_state(adapter._current_state)
    assert not adapter._pregrasp_requests and not adapter._pregrasp_finalized


def test_opening_grouping_rejects_a_tcp_that_moves_with_the_jaw():
    adapter, _ = make_adapter(hybrid_config=HybridConfig(group_gripper_openings=True,
                                                        progressive_pregrasp=True))
    xml = synthetic_urdf().replace('<parent link="left_link_4"/><child link="left_grasp_tcp"/>',
                                   '<parent link="left_jaw"/><child link="left_grasp_tcp"/>')
    with pytest.raises(ValueError, match='TCP independent'):
        adapter.set_robot_description(xml)


@pytest.mark.parametrize('reject', ['collision', 'fk'])
def test_cpu_final_gate_rejects_gpu_proposals(reject):
    adapter, trace = make_adapter()
    if reject == 'collision':
        adapter.state_is_valid = lambda *args: False
    else:
        adapter.forward_kinematics = lambda *args: PoseValue((9.,9.,9.), (0.,0.,0.,1.))
    adapter.prepare_pregrasps(requests(1))
    assert not any(adapter._pregrasp_cache.values())
    assert adapter.last_ik_statistics['fallback_batches'] == 1


def test_state_update_invalidates_pregrasp_cache():
    adapter, _ = make_adapter()
    adapter.prepare_pregrasps(requests(1))
    adapter.set_current_state(adapter._current_state)
    assert adapter._pregrasp_cache == {}


def test_live_head_measurements_survive_cpu_and_gpu_state_merging():
    adapter, _ = make_adapter()
    state = deepcopy(adapter._current_state)
    adapter.set_robot_description(synthetic_head_urdf())
    with pytest.raises(ValueError, match='incomplete joint state'):
        adapter.set_current_state(state)
    state.name.extend(['head_pan_joint', 'head_tilt_joint'])
    state.position.extend([.1, .4])
    adapter.set_current_state(state)
    adapter.prepare_pregrasps(requests())
    merged = adapter._merged_state(None).joint_state
    values = dict(zip(merged.name, merged.position))
    assert values['head_pan_joint'] == .1 and values['head_tilt_joint'] == .4
    gpu = adapter._gpus[16]
    assert gpu.state.name == merged.name
    state.position[-1] = .5
    adapter.set_current_state(state)
    assert dict(zip(gpu.state.name,gpu.state.position))['head_tilt_joint'] == .5


def test_robot_config_locks_head_joints_and_validates_complete_runtime_state():
    from cleany_skill_executor.curobo_model import CuRoboConfig
    model = RobotModel.__new__(RobotModel)
    model.config = CuRoboConfig(num_seeds=16)
    names = (*REQUIRED_JOINT_NAMES, 'head_pan_joint', 'head_tilt_joint')
    model.limits = dict.fromkeys(names, (-2.,2.))
    model.spheres, model.ignore = {}, {}
    state = dict.fromkeys(names, 0.)
    state['head_tilt_joint'] = .4
    kin = model.robot_config('runtime.urdf','left',state,'left_grasp_tcp')['kinematics']
    assert len(kin['cspace']['joint_names']) == 14
    assert kin['lock_joints']['head_tilt_joint'] == .4
    assert not set(ARM_JOINT_NAMES['left']) & kin['lock_joints'].keys()
    with pytest.raises(ValueError, match='complete'):
        model.validate_state({n:q for n,q in state.items() if n != 'head_pan_joint'})


def test_model_update_recreates_gpu_from_runtime_description(monkeypatch):
    adapter, trace = make_adapter()
    adapter.prepare_pregrasps(requests(1))
    monkeypatch.setattr(MoveItGraspAdapter, 'set_robot_description', lambda *args: None)
    adapter.set_robot_description('new actual runtime model')
    assert adapter._description == 'new actual runtime model'
    assert adapter._gpus == {} and 'gpu_engine_closed' in trace


def test_cancel_prevents_gpu_calls_and_propagates():
    adapter, trace = make_adapter()
    adapter.prepare_pregrasps(requests(1))
    adapter.cancel_active()
    assert 'gpu_cancel' in trace
    with pytest.raises(InterruptedError):
        adapter.prepare_pregrasps(requests(1))


def test_incomplete_gpu_batch_is_an_infrastructure_failure():
    adapter, _ = make_adapter()
    gpu = adapter._gpu(16)
    gpu.solve_aligned_batch = lambda *args: []
    with pytest.raises(InfrastructureError, match='incomplete'):
        adapter.prepare_pregrasps(requests(1))


def test_octomap_is_excluded_from_gpu_service_request_and_cpu_checks_every_state(monkeypatch):
    scene = PlanningScene()
    scene.world.octomap.octomap.data = [1,2,3]
    owner = NS(last_ik_statistics={'gpu_cost_omits_octomap': False},
               state_is_valid=lambda arm, q: q.positions[0] != 0.)
    gpu = _CandidateGpuAdapter.__new__(_CandidateGpuAdapter)
    gpu._owner = owner
    def snapshot(self, *, include_octomap=True):
        assert not include_octomap
        return PlanningScene()
    monkeypatch.setattr(CuRoboGraspAdapter, '_snapshot', snapshot)
    assert list(gpu._snapshot().world.octomap.octomap.data) == []
    assert list(scene.world.octomap.octomap.data) == [1,2,3]
    qs = [JointSolution(ARM_JOINT_NAMES['left'], (v,0.,0.,0.,0.)) for v in (0., .1)]
    assert gpu._valid_batch('left', qs) == [False, True]


def test_selector_uses_gpu_candidate_ik_and_cpu_path_planner():
    adapter, trace = make_adapter()
    adapter.set_target_contacts = lambda arm: None
    selector = GraspSelector(adapter, GraspSelectionConfig(maximum_candidates=8))
    candidates = [Candidate((.4+i*.001,.2,.3), (0.,-1.,0.), 1.-i*.01, source_index=i) for i in range(8)]
    selected = selector.select(candidates, required_arm='left')
    assert selected is not None and selected.candidate_index == 0
    assert trace.count('cpu_rrtconnect_request') == 2
    assert any(isinstance(t,dict) and t['batch'] == 8 for t in trace)


def test_diagnostics_keep_batched_candidate_indices_through_scene_wrapper():
    from cleany_skill_executor.planning_scene import SceneAwarePort

    adapter, _ = make_adapter(hybrid_config=HybridConfig(
        progressive_pregrasp=True, group_gripper_openings=True,
        pregrasp_retry_passes=0, cpu_ik_fallback=False))
    scene = NS(disallow_target_contacts=lambda: None, allow_contacts_for=lambda arm: None)
    selector = GraspSelector(SceneAwarePort(adapter, scene), GraspSelectionConfig(
        maximum_candidates=2, stop_after_first_feasible=True))
    emitted = []
    adapter.set_ik_diagnostic_callback(emitted.append)
    candidates = [Candidate((.4+i*.001,.2,.3), (0.,-1.,0.), 1.-i*.01,
                            source_index=17+i) for i in range(2)]
    assert selector.select(candidates, required_arm='left').candidate_index == 17
    assert [(r['candidate_index'], r['stage'], r['outcome']) for r in emitted] == [
        (17, 'PREGRASP_IK', 'accepted'), (18, 'PREGRASP_IK', 'deferred_proposal'),
        (17, 'GRASP_IK', 'accepted')]
    assert emitted == adapter.last_ik_statistics['ik_diagnostics']
    assert emitted[0]['limits']['position_error_mm'] == 2.


def test_gpu_diagnostics_retain_rejected_approach_error_and_no_solution():
    adapter, _ = make_adapter(hybrid_config=HybridConfig(
        progressive_pregrasp=True, pregrasp_retry_passes=0, cpu_ik_fallback=False))
    for count in (16, 32):
        gpu = adapter._gpu(count)
        def proposals(arm, targets, seeds):
            target = targets[0]
            q = JointSolution(ARM_JOINT_NAMES[arm], (*target.position, 0., 0.))
            return [PoseAlignment(q, target, .001, 42., 0.)]
        gpu.solve_aligned_batch = proposals if count == 16 else lambda *args: [None]
    item = requests(1)[0]
    adapter.prepare_pregrasps([item])
    adapter.set_gripper_position('left', item.gripper_position)
    assert adapter.solve_pregrasp_ik('left', item.target) is None
    primary, retry = adapter.last_ik_statistics['ik_diagnostics']
    assert primary['outcome'] == 'pose_rejected'
    assert primary['closest_alignment']['approach_error_degrees'] == 42.
    assert primary['failure_phase'] == 'GPU_FK_ALIGNMENT'
    assert retry['outcome'] == 'no_ik_solution' and retry['failure_phase'] == 'GPU_IK'
    assert retry['closest_alignment'] is None


def test_diagnostics_preserve_collision_failure_even_when_returned_pose_is_rejected():
    from cleany_skill_executor.core.ik_diagnostics import summarize_alignments

    adapter, _ = make_adapter()
    item = requests(1)[0]
    adapter.set_gripper_position('left', item.gripper_position)
    gpu = adapter._gpu(16)
    q = JointSolution(ARM_JOINT_NAMES['left'], (*item.target.position, 0., 0.))
    good = PoseAlignment(q, item.target, .001, 0., 0.)
    bad = PoseAlignment(q, item.target, .001, 90., 0.)
    gpu.last_batch_diagnostics = [{**summarize_alignments([good, bad], adapter._alignment_limits()),
        'collision_checked': 1, 'collision_valid': 0}]
    gpu.solve_aligned_batch = lambda *args: [bad]
    assert adapter._batch('left', [item.target], [None], 16) == [None]
    diagnostic = adapter.last_ik_statistics['ik_diagnostics'][-1]
    assert diagnostic['outcome'] == 'collision_rejected'
    assert diagnostic['proposal_alignment']['approach_error_degrees'] == 90.
    assert diagnostic['pose_feasible'] == 1


def test_diagnostics_are_bounded_and_reset_with_current_state():
    adapter, _ = make_adapter()
    for i in range(600):
        adapter._record_ik_diagnostic({'candidate_index': i})
    records = adapter.last_ik_statistics['ik_diagnostics']
    assert len(records) == 512 and records[0]['candidate_index'] == 88
    adapter.set_current_state(adapter._current_state)
    assert adapter.last_ik_statistics['ik_diagnostics'] == []


def test_cpu_diagnostics_no_solution_has_unavailable_errors(monkeypatch):
    adapter, _ = make_adapter(hybrid_config=HybridConfig(cpu_ik_fallback=True))
    empty_gpu_proposals(adapter)
    monkeypatch.setattr(MoveItGraspAdapter, 'solve_aligned_ik', lambda *args: None)
    item = requests(1)[0]
    adapter.prepare_pregrasps([item])
    diagnostic = adapter.last_ik_statistics['ik_diagnostics'][-1]
    assert diagnostic['backend'] == 'cpu_fallback'
    assert diagnostic['outcome'] == 'no_ik_solution'
    assert diagnostic['closest_alignment'] is None
    assert diagnostic['solver_collision_filter_enabled']
    assert diagnostic['recorded_at_ns'] > 0


def test_invalid_hybrid_budgets_fail_before_compute():
    for kwargs in ({'batch_size':0},{'ik_iterations':False},{'fallback_seeds':0},
                   {'exclude_opposite_arm_cost':1},
                   {'cpu_ik_fallback':1},{'pregrasp_retry_passes':-1},
                   {'pregrasp_retry_passes':3},{'pregrasp_retry_passes':True},
                   {'planning_retries':3},{'planning_retries':False}):
        with pytest.raises(ValueError):
            HybridConfig(**kwargs)


@pytest.mark.parametrize('arm',['left','right'])
def test_candidate_cost_uses_active_arm_shapes_and_preserves_locked_joints(monkeypatch,arm):
    spheres = {'left_jaw':[1], 'right_jaw':[2], 'base_link':[3]}
    original = {'kinematics':{
        'collision_link_names':list(spheres),'collision_spheres':spheres,
        'self_collision_ignore':{name:list(spheres) for name in spheres},
        'self_collision_buffer':{name:0. for name in spheres},
        'link_names':[], 'lock_joints':{'left_gripper_joint':.4,'right_gripper_joint':0.}}}
    monkeypatch.setattr(RobotModel,'robot_config',lambda *args:deepcopy(original))
    model = _CandidateRobotModel.__new__(_CandidateRobotModel)
    model._exclude_opposite_arm_cost = True
    model._joint_links = ['left_jaw','right_jaw']
    kin = model.robot_config('runtime.urdf',arm,{},f'{arm}_grasp_tcp')['kinematics']
    assert set(kin['collision_link_names']) == {f'{arm}_jaw','base_link'}
    assert kin['lock_joints'] == original['kinematics']['lock_joints']
    assert set(kin['link_names']) == {f'{arm}_grasp_tcp','left_jaw','right_jaw'}
    assert spheres == {'left_jaw':[1], 'right_jaw':[2], 'base_link':[3]}
