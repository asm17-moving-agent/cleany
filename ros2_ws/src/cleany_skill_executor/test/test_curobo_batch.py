"""Candidate ownership and rejection must survive vectorized evaluation."""
from types import SimpleNamespace, ModuleType
from contextlib import nullcontext
import sys

import numpy as np

from cleany_skill_executor.curobo_batch import CuRoboBatchAdapter, CuRoboBatchEngine
from cleany_skill_executor.curobo_engine import CuRoboResources
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution, PoseValue
import pytest


@pytest.mark.parametrize('continuous', [False, True])
def test_batch_continuous_roll_alignment_preserves_owners_and_checks_collisions(continuous):
    from cleany_skill_executor.core.alignment import quaternion_from_rotation
    from cleany_skill_executor.core.urdf_fk import axis_rotation
    from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig
    names = ARM_JOINT_NAMES['right']
    adapter = CuRoboBatchAdapter.__new__(CuRoboBatchAdapter)
    adapter._config = MoveItAdapterConfig(align_wrist_roll=continuous, closing_tolerance_degrees=5.)
    adapter._joint_limits = {names[-1]: (-2., 2., 1.)}
    adapter._current_state = None
    adapter._engine = SimpleNamespace(model=SimpleNamespace(limits={names[-1]: (-2., 2.)}))
    adapter._compute = lambda fn, *args: fn(*args)
    calls = []
    rotation = np.array([[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]])
    def pose(roll):
        return PoseValue((0., 0., 0.), quaternion_from_rotation(axis_rotation((0, 0, 1), roll) @ rotation))
    def fk(arm, tip, solutions, state):
        calls.append(len(solutions))
        return [pose(q.positions[-1]) for q in solutions]
    adapter._engine.fk = fk
    solution = JointSolution(names, (0.,)*5)
    scored = adapter._score_batch('right', [pose(np.radians(17.)), pose(np.radians(70.))],
                                  [[solution], [solution]], {names[-1]: 0.})
    assert len(scored) == 2
    assert all(-2 <= item.solution.positions[-1] <= 2 for group in scored for item in group)
    if continuous:
        assert all(min(item.closing_error_degrees for item in group) < 1e-5 for group in scored)
        assert len(calls) == 2
    else:
        assert all(not adapter._feasible_alignment(item) for group in scored for item in group)
        assert len(calls) == 1
    adapter._valid_batch = lambda arm, solutions: [q.positions[-1] > .5 for q in solutions]
    chosen = adapter._choose_batch('right', scored)
    assert chosen[0] is None
    assert (chosen[1] is not None) == continuous


def test_batch_collision_rejection_preserves_candidate_ownership():
    adapter = CuRoboBatchAdapter.__new__(CuRoboBatchAdapter)
    adapter._feasible_alignment = lambda item: item.aligned
    adapter._valid_batch = lambda arm, solutions: [value != 'blocked' for value in solutions]
    adapter._best_alignment = lambda items: min(items, key=lambda x:x.score)
    good = SimpleNamespace(solution='valid', aligned=True, score=2)
    blocked = SimpleNamespace(solution='blocked', aligned=True, score=1)
    wrong = SimpleNamespace(solution='wrong', aligned=False, score=0)
    assert adapter._choose_batch('left', [[blocked], [wrong, good], []]) == [None, good, None]


def test_empty_batch_does_not_access_ros_or_gpu():
    adapter = CuRoboBatchAdapter.__new__(CuRoboBatchAdapter)
    assert adapter.solve_aligned_batch('left', []) == []


def test_ended_warmup_keeps_solver_but_does_not_start_gpu_solve(monkeypatch):
    module = ModuleType('curobo.types.math')
    module.Pose = SimpleNamespace
    monkeypatch.setitem(sys.modules, module.__name__, module)
    engine = CuRoboBatchEngine.__new__(CuRoboBatchEngine)
    engine.config = SimpleNamespace(num_seeds=16)
    engine.ik_batch_size = 8
    engine._ik, engine._ik_locks, engine._world_keys = {}, {}, {}
    original_cache = engine._ik
    def setup(arm, tip, state, world):
        engine._ik[(arm, tip)] = 'initialized solver'
        return SimpleNamespace(solve_batch=lambda *_: pytest.fail('VLM already finished'))
    engine._solver = setup
    seed = JointSolution(ARM_JOINT_NAMES['left'], (0.,) * 5)
    with pytest.raises(InterruptedError, match='window ended'):
        engine.solve_batch('left', 'left_grasp_tcp', [(0., 0., 0.)], [[seed]], {}, {},
                           continue_work=lambda: False)
    assert engine._ik is original_cache
    assert engine._batch_ik_caches[('left', 'left_grasp_tcp', 8)][0][
        ('left', 'left_grasp_tcp')] == 'initialized solver'


@pytest.mark.parametrize('current_valid', [True, False])
@pytest.mark.parametrize('primary_only', [False, True])
def test_progressive_alignment_refines_only_the_current_candidate(monkeypatch, current_valid, primary_only):
    from cleany_skill_executor.hybrid_adapter import _CandidateGpuAdapter

    monkeypatch.setattr('cleany_skill_executor.curobo_batch.world_from_scene', lambda *a, **k: {})
    adapter = _CandidateGpuAdapter.__new__(_CandidateGpuAdapter)
    adapter._owner = SimpleNamespace(_primary_only=primary_only)
    adapter.validation_targets = {0}
    adapter._scene_operation, adapter._active_scene = nullcontext, None
    adapter._state = lambda: {}
    adapter._seed_bases = lambda arm, seed: ['base']
    adapter._config = SimpleNamespace(aim_tip_offset_m=.1)
    adapter._gpu_config = SimpleNamespace(num_seeds=16)
    adapter._compute = lambda fn, *args: fn(*args)
    calls = []

    def solve(arm, tip, positions, seeds, state, world):
        calls.append((tip, list(positions)))
        if len(calls) == 1:
            return [['direct'] if current_valid else [], [], []]
        return [['aligned']]

    adapter._engine = SimpleNamespace(solve_batch=solve, adaptive_ik_batch_size=True)
    adapter._score_batch = lambda arm, targets, solutions, state, preferred: solutions
    adapter._choose_batch = lambda arm, groups, preferred: [group[0] if group else None for group in groups]
    targets = [PoseValue((float(i), .2, .3), (0., 0., 0., 1.)) for i in range(3)]

    result = adapter.solve_aligned_batch('right', targets)

    assert result == ['direct' if current_valid else None if primary_only else 'aligned', None, None]
    assert [tip for tip, _ in calls] == (
        ['right_grasp_tcp'] if current_valid or primary_only else
        ['right_grasp_tcp', 'right_pregrasp_aim_tip', 'right_grasp_tcp'])
    if not current_valid and not primary_only:
        assert calls[-1][1] == [targets[0].position]
        assert calls[1][1][0] == pytest.approx((0., .1, .3))


def test_iteration_budget_configures_gradient_total_not_outer_replays(monkeypatch):
    modules = {name: ModuleType(name) for name in (
        'curobo.geom.types', 'curobo.geom.sdf.world',
        'curobo.rollout.cost.pose_cost', 'curobo.wrap.reacher.ik_solver')}
    modules['curobo.geom.sdf.world'].CollisionCheckerType = SimpleNamespace(MESH='mesh')
    modules['curobo.geom.types'].WorldConfig = SimpleNamespace(
        from_dict=lambda world: SimpleNamespace(get_collision_check_world=lambda: world))
    modules['curobo.rollout.cost.pose_cost'].PoseCostMetric = SimpleNamespace
    configurations = []
    def config(robot, world, **kwargs):
        configurations.append(kwargs)
        return kwargs
    modules['curobo.wrap.reacher.ik_solver'].IKSolverConfig = SimpleNamespace(load_from_robot_config=config)
    modules['curobo.wrap.reacher.ik_solver'].IKSolver = lambda cfg: SimpleNamespace(
        update_pose_cost_metric=lambda metric: None, world_coll_checker=object())
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    engine = CuRoboBatchEngine.__new__(CuRoboBatchEngine)
    engine.ik_iterations, engine._urdf = 50, 'robot.urdf'
    engine._ik, engine._ik_locks = {}, {}
    engine.resources = CuRoboResources()
    engine._world_keys = {('right', 'right_grasp_tcp'): repr({})}
    engine.config = SimpleNamespace(num_seeds=16, ik_position_threshold_m=.001, use_cuda_graph=True)
    engine.tensor = SimpleNamespace(to_device=np.asarray)
    engine.model = SimpleNamespace(robot_config=lambda *args: {'kinematics': {'lock_joints': {}}})
    engine._solver('right', 'right_grasp_tcp', {}, {})
    assert configurations[0]['grad_iters'] == 50


def test_shared_world_revision_refreshes_cached_solver_after_other_arm_changes_scene(monkeypatch):
    modules = {name: ModuleType(name) for name in (
        'curobo.geom.types', 'curobo.geom.sdf.world',
        'curobo.rollout.cost.pose_cost', 'curobo.wrap.reacher.ik_solver')}
    modules['curobo.geom.sdf.world'].CollisionCheckerType = SimpleNamespace(MESH='mesh')
    modules['curobo.geom.types'].WorldConfig = SimpleNamespace(
        from_dict=lambda world: SimpleNamespace(get_collision_check_world=lambda: world))
    modules['curobo.rollout.cost.pose_cost'].PoseCostMetric = SimpleNamespace
    modules['curobo.wrap.reacher.ik_solver'].IKSolverConfig = SimpleNamespace(
        load_from_robot_config=lambda robot, world, **kwargs: (world,kwargs))
    updates = []; checkers = []
    def make_solver(cfg):
        world, kwargs = cfg
        checker = kwargs['world_coll_checker']
        if checker is None:
            checker = SimpleNamespace(clear_cache=lambda: updates.append('clear'))
            checkers.append(checker)
        return SimpleNamespace(world_coll_checker=checker,
            update_pose_cost_metric=lambda metric: None,
            update_world=lambda world: updates.append(world))
    modules['curobo.wrap.reacher.ik_solver'].IKSolver = make_solver
    for name, module in modules.items(): monkeypatch.setitem(sys.modules,name,module)
    engine = CuRoboBatchEngine.__new__(CuRoboBatchEngine)
    engine._ik, engine._ik_locks = {}, {}
    engine.resources = CuRoboResources()
    engine._urdf = 'robot.urdf'
    engine.config = SimpleNamespace(num_seeds=16, ik_position_threshold_m=.001,use_cuda_graph=True)
    engine.tensor = SimpleNamespace(to_device=np.asarray)
    engine.model = SimpleNamespace(robot_config=lambda *args: {'kinematics': {'lock_joints': {}}})
    scene_a, scene_b = {'scene':'a'}, {'scene':'b'}
    left = engine._solver('left','left_grasp_tcp',{},scene_a)
    right = engine._solver('right','right_grasp_tcp',{},scene_b)
    assert engine._solver('left','left_grasp_tcp',{},scene_a) is left
    assert left.world_coll_checker is right.world_coll_checker
    assert len(checkers) == 1 and updates == ['clear',scene_b,'clear',scene_a]


def test_partial_success_keeps_cuda_batch_shape_and_discards_padding(monkeypatch):
    adapter = CuRoboBatchAdapter.__new__(CuRoboBatchAdapter)
    monkeypatch.setattr('cleany_skill_executor.curobo_batch.world_from_scene', lambda *a, **k: {})
    adapter._scene_operation = nullcontext
    adapter._active_scene = None
    adapter._state = lambda: {}
    adapter._seed_bases = lambda arm, seed: ['seed']
    adapter._compute = lambda function, *args: function(*args)
    adapter._config = SimpleNamespace(aim_tip_offset_m=.1)
    adapter._gpu_config = SimpleNamespace(num_seeds=32)
    calls = []
    def solve(arm, tip, positions, seeds, state, world):
        calls.append((positions, seeds))
        return [['query_0'], ['query_1']]
    adapter._engine = SimpleNamespace(solve_batch=solve)
    scored_sizes = []
    def score(arm, targets, solutions, state, preferred=None):
        scored_sizes.append((len(targets), len(solutions)))
        return [[] for _ in targets]
    adapter._score_batch = score
    selections = iter([['direct_success', None], ['direct_success', 'aim_success']])
    adapter._choose_batch = lambda arm, evaluated, preferred=None: next(selections)
    targets = [PoseValue((.1, .2, .3), (0., 0., 0., 1.)),
               PoseValue((.4, .5, .6), (0., 0., 0., 1.))]
    assert adapter.solve_aligned_batch('right', targets) == ['direct_success', 'aim_success']
    assert [len(positions) for positions, seeds in calls] == [2, 2, 2]
    assert calls[2][0] == [targets[1].position, targets[1].position]
    assert scored_sizes == [(2, 2), (1, 1)]


def test_projection_is_scored_when_aim_ik_has_no_solution(monkeypatch):
    adapter = CuRoboBatchAdapter.__new__(CuRoboBatchAdapter)
    monkeypatch.setattr('cleany_skill_executor.curobo_batch.world_from_scene', lambda *a, **k: {})
    adapter._scene_operation, adapter._active_scene = nullcontext, None
    adapter._state = lambda: {}
    adapter._seed_bases = lambda arm, seed: ['base']
    adapter._config = SimpleNamespace(aim_tip_offset_m=.1)
    adapter._gpu_config = SimpleNamespace(num_seeds=16)
    adapter._compute = lambda fn, *args: fn(*args)
    results = iter([[[]], [[]], [['projected_solution']]])
    adapter._engine = SimpleNamespace(solve_batch=lambda *args: next(results))
    scored = []
    def score(arm, targets, solutions, state, preferred):
        scored.append([list(group) for group in solutions])
        return solutions
    adapter._score_batch = score
    adapter._choose_batch = lambda arm, evaluated, preferred: [group[0] if group else None for group in evaluated]
    target = PoseValue((0., 0., 0.), (0., 0., 0., 1.))
    assert adapter.solve_aligned_batch('right', [target]) == ['projected_solution']
    assert scored == [[[]], [['projected_solution']]]


@pytest.mark.parametrize('batch_size, adaptive, expected_solvers',
                         [(None, False, 2), (8, False, 1), (8, True, 2)])
def test_user_seeds_and_single_target_reuse_fixed_batch_buffers(monkeypatch, batch_size, adaptive, expected_solvers):
    module = ModuleType('curobo.types.math')
    module.Pose = SimpleNamespace
    monkeypatch.setitem(sys.modules, module.__name__, module)
    class ResultTensor:
        def __init__(self, value):
            self.value = value
        def detach(self): return self
        def cpu(self): return self
        def numpy(self): return self.value
    names = ARM_JOINT_NAMES['right']
    engine = CuRoboBatchEngine.__new__(CuRoboBatchEngine)
    engine.config = SimpleNamespace(num_seeds=32)
    engine.ik_batch_size = batch_size
    engine.adaptive_ik_batch_size = adaptive
    engine.tensor = SimpleNamespace(to_device=np.asarray)
    engine.torch = SimpleNamespace(cuda=SimpleNamespace(synchronize=lambda: None))
    engine._ik, engine._ik_locks, engine._world_keys = {}, {}, {}
    original_cache = engine._ik
    constructed = []
    iteration_limits = []
    padded_sizes = []
    def get_solver(arm, tip, state, world):
        if tip not in engine._ik:
            def solve(pose, **kwargs):
                iteration_limits.append(kwargs['newton_iters'])
                count = len(pose.position)
                padded_sizes.append(count)
                return SimpleNamespace(solution=ResultTensor(np.zeros((count,1,5))),
                                       success=ResultTensor(np.ones((count,1),dtype=bool)))
            engine._ik[tip] = SimpleNamespace(joint_names=names, solve_batch=solve)
            constructed.append(engine._ik[tip])
        return engine._ik[tip]
    engine._solver = get_solver
    seed = JointSolution(names, (0.,)*5)
    for count, width in ((2,1), (2,3), (1,3), (2,2)):
        result = engine.solve_batch('right','right_grasp_tcp',[(0.,0.,0.)]*count,
                                    [[seed]*width for _ in range(count)],{}, {})
        assert len(result) == count
        assert engine._ik is original_cache
    assert len(constructed) == expected_solvers
    assert iteration_limits == [None]*4
    assert padded_sizes == ([2,2,1,2] if adaptive or batch_size is None else [8]*4)
    engine.ik_iterations = 100
    engine.solve_batch('right','right_grasp_tcp',[(0.,0.,0.)], [[seed]],{}, {})
    # Per-call override controls outer graph replay count in cuRobo 0.7.8.
    # The total iteration override belongs in IKSolverConfig.grad_iters.
    assert iteration_limits[-1] is None


@pytest.mark.parametrize('iterations', [0, -1, 1.5])
def test_batch_engine_rejects_invalid_iteration_override_before_gpu_access(iterations):
    with pytest.raises(ValueError, match='positive integer'):
        CuRoboBatchEngine(None, None, ik_iterations=iterations)


@pytest.mark.parametrize('seeds', [0, -1, 1.5, True])
def test_batch_engine_rejects_invalid_trajectory_seed_count_before_gpu_access(seeds):
    with pytest.raises(ValueError, match='JS trajectory seed'):
        CuRoboBatchEngine(None, None, trajectory_seeds=seeds)


def test_batch_motion_configures_actual_js_seed_count_and_reuses_shape_buffers(monkeypatch):
    modules = {name: ModuleType(name) for name in (
        'curobo.geom.sdf.world', 'curobo.geom.types', 'curobo.wrap.reacher.motion_gen')}
    modules['curobo.geom.sdf.world'].CollisionCheckerType = SimpleNamespace(MESH='mesh')
    modules['curobo.geom.types'].WorldConfig = SimpleNamespace(
        from_dict=lambda world: SimpleNamespace(get_collision_check_world=lambda: world))
    configurations = []
    def config(robot, world, **kwargs):
        configurations.append(kwargs)
        return kwargs
    modules['curobo.wrap.reacher.motion_gen'].MotionGenConfig = SimpleNamespace(load_from_robot_config=config)
    modules['curobo.wrap.reacher.motion_gen'].MotionGen = lambda value: SimpleNamespace(config=value)
    for name, module in modules.items(): monkeypatch.setitem(sys.modules, name, module)
    engine = CuRoboBatchEngine.__new__(CuRoboBatchEngine)
    engine.model = SimpleNamespace(robot_config=lambda *args: {'kinematics': {}})
    engine._urdf, engine.tensor, engine.trajectory_seeds = 'robot.urdf', None, 4
    engine.config = SimpleNamespace(interpolation_dt=.05, num_seeds=16, use_cuda_graph=True, joint_tolerance_rad=.002)
    first = engine._batch_motion('right', {}, {}, 2)
    assert engine._batch_motion('right', {}, {}, 2) is first
    assert engine._batch_motion('right', {}, {}, 4) is not first
    assert len(configurations) == 2
    assert all(cfg['num_trajopt_noisy_seeds'] == 4 for cfg in configurations)
    assert all(cfg['cspace_threshold'] == .002 for cfg in configurations)


@pytest.mark.parametrize('seeds', [1, 4])
def test_single_joint_plan_uses_experimental_js_seed_configuration_once(monkeypatch, seeds):
    from cleany_skill_executor.curobo_engine import CuRoboEngine
    engine = CuRoboBatchEngine.__new__(CuRoboBatchEngine)
    engine.trajectory_seeds, engine._motion, engine._urdf = seeds, {}, 'robot.urdf'
    engine.model = SimpleNamespace(robot_config=lambda *args: {'kinematics': {'lock_joints': {}}})
    created, delegated = [], []
    motion = object()
    def create(*args):
        created.append(args)
        return motion
    def parent(self, *args):
        delegated.append(args)
        return 'result'
    engine._batch_motion = create
    monkeypatch.setattr(CuRoboEngine, 'plan', parent)
    state = {'right_shoulder_yaw_joint':.1, 'left_shoulder_yaw_joint':.2, 'right_gripper_joint':1.}
    for _ in range(2):
        assert engine.plan('right', None, None, state, {}, .1, .1, 4., 4) == 'result'
    assert len(delegated) == 2
    assert len(created) == (0 if seeds == 1 else 1)
    if seeds == 4:
        assert engine._motion['right'][0] is motion
        assert engine._motion['right'][2] == {'left_shoulder_yaw_joint':.2, 'right_gripper_joint':1.}


@pytest.mark.parametrize('fault', [None, 'collision', 'corridor', 'orientation'])
def test_batch_approach_matches_cpu_gates_with_one_gpu_fk_call(fault):
    from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter
    from cleany_skill_executor.core.physical_grasp_selection import PlannedPath

    config = MoveItAdapterConfig()
    path = PlannedPath(ARM_JOINT_NAMES['right'], ((0.,)*5, (.05,)*5, (.1,)*5))
    start = PoseValue((0., 0., 0.), (0., 0., 0., 1.))
    target = PoseValue((.08, 0., 0.), (0., 0., 0., 1.))
    calls = []
    def poses(solutions):
        return [PoseValue((q.positions[0]*.8, .02 if fault == 'corridor' else 0., 0.),
                          (0., 0., 1., 0.) if fault == 'orientation' else start.orientation_xyzw)
                for q in solutions]
    def valid(solutions):
        return [not (fault == 'collision' and .035 < q.positions[0] < .065) for q in solutions]
    adapter = CuRoboBatchAdapter.__new__(CuRoboBatchAdapter)
    adapter._config = config
    adapter._joint_limits = {name: (-2., 2., 1.) for name in path.joint_names}
    adapter._scene_operation = nullcontext
    adapter._state = lambda: {}
    adapter._valid_batch = lambda arm, solutions: valid(solutions)
    adapter._compute = lambda function, *args: function(*args)
    def fk(arm, tip, solutions, state):
        calls.append(len(solutions))
        return poses(solutions)
    adapter._engine = SimpleNamespace(fk=fk)
    cpu = MoveItGraspAdapter.__new__(MoveItGraspAdapter)
    cpu._config = config
    cpu._joint_limits = adapter._joint_limits
    cpu.state_is_valid = lambda arm, q: valid([q])[0]
    cpu.forward_kinematics = lambda arm, q: poses([q])[0]
    gpu_ok = adapter.validate_approach_path('right', path, start, target)
    cpu_ok = cpu.validate_approach_path('right', path, start, target)
    assert gpu_ok == cpu_ok == (fault is None)
    assert len(calls) == (0 if fault == 'collision' else 1)
    if calls:
        assert calls[0] > 1
