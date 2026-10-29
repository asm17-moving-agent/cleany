"""Matched CPU/CUDA pregrasp refinement and a separate production cuRobo comparison.

No ROS node, scene service or actuator is started. Local solvers share the same
URDF, seed, objective, finite differences, damping, bounds and stopping rules.
CUDA graph setup is reported separately; timed calls include input/output copies.
cuRobo retains its larger production search budget and is labelled separately.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import inspect
import json
from pathlib import Path
import platform
import time

import numpy as np
import torch
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PlanningScene

from cleany_skill_executor.core.alignment import tcp_axes
from cleany_skill_executor.core.physical_grasp_selection import JointSolution, PoseValue
from cleany_skill_executor.core.urdf_fk import UrdfChain
from cleany_skill_executor.curobo_batch import CuRoboBatchAdapter, CuRoboBatchEngine
from cleany_skill_executor.curobo_model import CuRoboConfig
from cleany_skill_executor.hybrid_adapter import HybridConfig, HybridGraspAdapter, _CandidateRobotModel
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig


@dataclass(frozen=True)
class Case:
    label: str
    arm: str
    seed: JointSolution
    seed_pose: PoseValue
    target: PoseValue
    opening: float


def cases_from_records(data: dict) -> list[Case]:
    grasps, cases = {}, []
    for row in data['ik_diagnostics']:
        key = row['arm'], row['candidate_index']
        if row['stage'] == 'GRASP_IK' and row['outcome'] == 'accepted':
            grasps[key] = row['final_alignment']
        if row['stage'] != 'PREGRASP_IK' or key not in grasps:
            continue
        grasp = grasps[key]
        cases.append(Case(f'candidate_{key[1]}_target_{len(cases)}', key[0],
            JointSolution(tuple(grasp['joint_names']), tuple(grasp['joint_positions_rad'])),
            PoseValue(tuple(grasp['fk_position_m']), tuple(grasp['fk_orientation_xyzw'])),
            PoseValue(tuple(row['target']['position_m']), tuple(row['target']['orientation_xyzw'])),
            row['gripper_joint_position_rad']))
    return cases


class CudaRefinement:
    """Benchmark-only translation of refine_pose, with batched FK differences."""

    def __init__(self, chain: UrdfChain, names, state, bounds, dtype, iterations=10):
        self.dtype, self.iterations = dtype, iterations
        self.device = torch.device('cuda')
        self.tensor = lambda value: torch.as_tensor(value, dtype=dtype, device=self.device)
        self.eye3, self.eye5 = self.tensor(np.eye(3)), self.tensor(np.eye(5))
        self.initial_damping = self.tensor(1e-4)
        self.finite_difference = self.tensor(1e-4)
        self.lower, self.upper = self.tensor(bounds).T
        self.seed, self.target = self.tensor([0.]*5), self.tensor([0.]*9)
        self.joints = []
        for joint in chain.joints:
            axis = joint.axis
            x, y, z = axis
            skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
            index = names.index(joint.name) if joint.name in names else None
            fixed = float(state.get(joint.name, 0.))
            self.joints.append((joint.kind, index, self.tensor(fixed),
                self.tensor(joint.translation), self.tensor(joint.rotation),
                self.tensor(axis), self.tensor(skew), self.tensor(skew@skew)))
        self.graph = None

    def fk(self, values):
        position = torch.zeros((values.shape[0], 3), device=self.device, dtype=self.dtype)
        rotation = self.eye3.expand(values.shape[0], 3, 3)
        for kind, index, fixed, translation, origin_rotation, axis, skew, square in self.joints:
            position = position + torch.matmul(rotation, translation)
            rotation = torch.matmul(rotation, origin_rotation)
            if kind == 'fixed':
                continue
            angle = values[:, index] if index is not None else fixed.expand(values.shape[0])
            if kind == 'prismatic':
                position = position + torch.matmul(rotation, (angle[:, None]*axis)[..., None]).squeeze(-1)
            else:
                local = (self.eye3 + torch.sin(angle)[:, None, None]*skew
                         + (1.-torch.cos(angle))[:, None, None]*square)
                rotation = torch.matmul(rotation, local)
        return position, rotation

    def residual(self, values):
        position, rotation = self.fk(values)
        return torch.cat((position-self.target[:3],
            .14*(-rotation[:, :, 1]-self.target[3:6]),
            .08*(rotation[:, :, 0]-self.target[6:])), dim=1)

    def solve_normal(self, matrix, rhs):
        """Solve the damped 5x5 SPD system without an optional cuSOLVER library."""
        augmented = torch.cat((matrix, rhs[:, None]), dim=1)
        for index in range(5):
            pivot = augmented[index]/augmented[index, index]
            augmented = (augmented-augmented[:, index, None]*pivot[None]
                         +self.eye5[:, index, None]*pivot[None])
        return augmented[:, -1]

    def compute(self):
        q = torch.clamp(self.seed, self.lower, self.upper)
        damping = self.initial_damping
        error = self.residual(q[None])[0]
        active = torch.ones((), dtype=torch.bool, device=self.device)
        steps = torch.zeros((), dtype=torch.int32, device=self.device)
        for _ in range(self.iterations):
            steps = steps + active.to(torch.int32)
            delta = torch.minimum(self.finite_difference, (self.upper-self.lower)/2)
            delta = torch.where(q+delta > self.upper, -delta, delta)
            probes = q[None] + self.eye5*delta
            jacobian = ((self.residual(probes)-error)/delta[:, None]).T
            normal = jacobian.T@jacobian+damping*self.eye5
            step = self.solve_normal(normal, -jacobian.T@error)
            step = step*(.25/torch.clamp(torch.max(torch.abs(step)), min=.25))
            trial = torch.clamp(q+step, self.lower, self.upper)
            next_error = self.residual(trial[None])[0]
            better = torch.linalg.vector_norm(next_error) < torch.linalg.vector_norm(error)
            tiny_step = torch.max(torch.abs(trial-q)) < 1e-6
            accept = active & better & ~tiny_step
            next_damping = torch.where(better, torch.clamp(damping/3, min=1e-7),
                                      torch.clamp(damping*10, max=1.))
            q = torch.where(accept, trial, q)
            error = torch.where(accept, next_error, error)
            damping = torch.where(active, next_damping, damping)
            active = active & ~(better & tiny_step) & ~(~better & (damping == 1.)) \
                & ~(torch.linalg.vector_norm(error) < 1e-5)
        return q, steps

    def set_input(self, case: Case):
        approach, closing = tcp_axes(case.target.orientation_xyzw)
        self.seed.copy_(torch.as_tensor(case.seed.positions, dtype=self.dtype))
        self.target.copy_(torch.as_tensor((*case.target.position, *approach, *closing), dtype=self.dtype))

    def capture(self, case: Case):
        began = time.perf_counter()
        self.set_input(case)
        stream = torch.cuda.Stream()
        stream.wait_stream(torch.cuda.current_stream())
        with torch.cuda.stream(stream):
            for _ in range(3):
                self.compute()
        torch.cuda.current_stream().wait_stream(stream)
        torch.cuda.synchronize()
        self.graph = torch.cuda.CUDAGraph()
        with torch.cuda.graph(self.graph):
            self.output, self.steps = self.compute()
        torch.cuda.synchronize()
        return time.perf_counter()-began

    def solve(self, case: Case, *, graph: bool):
        self.set_input(case)
        if graph:
            self.graph.replay()
            values, steps = self.output, self.steps
        else:
            values, steps = self.compute()
        result = tuple(float(v) for v in values.cpu().numpy())
        return JointSolution(case.seed.names, result), int(steps.item())


class GeometryOnlyGpu(CuRoboBatchAdapter):
    """Keep production proposals/scoring; omit common external collision RPCs."""

    @contextmanager
    def _scene_operation(self):
        self._active_scene = PlanningScene()
        try:
            yield
        finally:
            self._active_scene = None

    def _valid_batch(self, arm, solutions):
        return [True]*len(solutions)

    def _alignment_refinement_targets(self, target_count):
        # The grasp-first production pass also defers aim/project retries.
        return set()


class FusedCudaRefinement(CudaRefinement):
    """Same local solver with cuRobo's fused CUDA FK and private batch buffers."""

    def __init__(self, engine, chain, names, state, bounds, arm):
        super().__init__(chain, names, state, bounds, torch.float32)
        # Sharing the scoring model would let varying candidate batch sizes
        # replace buffers referenced by the captured local-refinement graph.
        self.model = deepcopy(engine._full_fk(state))
        self.model.update_batch_size(5, reset_buffers=True)
        self.full_positions = self.tensor([[state[n] for n in self.model.joint_names]]*5)
        self.indices = torch.tensor([self.model.joint_names.index(n) for n in names],
                                    device=self.device, dtype=torch.long)
        self.tip_index = self.model.link_names.index(f'{arm}_grasp_tcp')
        self.gripper_index = self.model.joint_names.index(f'{arm}_gripper_joint')

    def set_input(self, case):
        super().set_input(case)
        self.full_positions[:, self.gripper_index] = case.opening

    def fk(self, values):
        self.full_positions.index_copy_(1, self.indices, values.expand(5, -1))
        result = self.model.get_state(self.full_positions)
        position = result.links_position[:values.shape[0], self.tip_index]
        w, x, y, z = result.links_quaternion[:values.shape[0], self.tip_index].unbind(-1)
        rotation = torch.stack((
            1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
            2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
            2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)), dim=-1).reshape(-1, 3, 3)
        return position, rotation


def distribution(values):
    return {'median_ms': float(np.median(values)), 'p95_ms': float(np.percentile(values, 95)),
            'min_ms': min(values), 'max_ms': max(values)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--statistics', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repetitions', type=int, default=20)
    args = parser.parse_args()
    if args.repetitions < 5:
        parser.error('Use at least five paired repetitions')
    data = json.loads(args.statistics.read_text())
    cases = cases_from_records(data)
    assert cases and len({c.arm for c in cases}) == 1
    arm = cases[0].arm
    xml = args.urdf.read_text()
    state = dict(zip(data['start_joint_state']['names'], data['start_joint_state']['positions_rad']))
    config = MoveItAdapterConfig(use_verified_local_fk=True, pose_refinement_iterations=10,
        position_tolerance_m=.01, pregrasp_position_tolerance_m=.01,
        approach_tolerance_degrees=30., closing_tolerance_degrees=30., align_wrist_roll=True)
    cpu = HybridGraspAdapter(object(), config, hybrid_config=HybridConfig(seeded_local_ik=True),
        ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    cpu.set_robot_description(xml)
    joints = JointState(name=list(state), position=list(state.values()))
    cpu.set_current_state(joints)
    chain = cpu._local_fk[arm]
    names = cases[0].seed.names
    bounds = [cpu._joint_limits[n][:2] for n in names]
    began = time.perf_counter()
    torch.cuda.init()
    torch.cuda.synchronize()
    setup = {'cuda_context_sec': time.perf_counter()-began}
    local = {}
    for label, dtype in [('gpu_local64', torch.float64), ('gpu_local32', torch.float32)]:
        began = time.perf_counter()
        solver = CudaRefinement(chain, names, state, bounds, dtype)
        setup[label+'_construct_sec'] = time.perf_counter()-began
        rng = np.random.default_rng(179)
        for _ in range(12):
            sample = rng.normal(size=(9, 5))
            normal = sample.T@sample+1e-4*np.eye(5)
            rhs = rng.normal(size=5)
            solved = solver.solve_normal(solver.tensor(normal), solver.tensor(rhs)).cpu().numpy()
            assert np.allclose(solved, np.linalg.solve(normal, rhs),
                               atol=1e-10 if dtype == torch.float64 else 1e-4)
        # Compare device FK to independent CPU FK on recorded and interior poses.
        rng = np.random.default_rng(179)
        probes = np.array([c.seed.positions for c in cases] + [
            tuple(rng.uniform(np.array(bounds)[:, 0], np.array(bounds)[:, 1])) for _ in range(12)])
        p, r = solver.fk(solver.tensor(probes))
        p, r = p.cpu().numpy(), r.cpu().numpy()
        maximum = 0.
        for i, q in enumerate(probes):
            cp, cr = chain.pose({**state, **dict(zip(names, q))})
            maximum = max(maximum, float(np.max(np.abs(cp-p[i]))), float(np.max(np.abs(cr-r[i]))))
        assert maximum < (1e-10 if dtype == torch.float64 else 2e-6), maximum
        setup[label+'_fk_max_abs_difference'] = maximum
        setup[label+'_graph_prepare_sec'] = solver.capture(cases[0])
        local[label] = solver
    print(json.dumps({'progress': 'local_graphs_ready', 'setup': setup}), flush=True)

    import xacro
    from ament_index_python.packages import get_package_share_directory
    began = time.perf_counter()
    gpu_config = CuRoboConfig(num_seeds=16, ik_position_threshold_m=.01,
        geometry_cache_directory='/opt/cleany-motion/cache/robot-geometry')
    srdf = Path(get_package_share_directory('cleany_moveit_config'))/'config/cleany.srdf.xacro'
    semantic = xacro.process_file(str(srdf), mappings={'include_head_camera': 'true'}).toxml()
    model = _CandidateRobotModel(xml, semantic, gpu_config, exclude_opposite_arm_cost=True)
    engine = CuRoboBatchEngine(model, gpu_config, ik_iterations=50, ik_batch_size=8)
    gpu = GeometryOnlyGpu(object(), config, gpu_config=gpu_config, engine=engine,
        scene_client=object(), ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    gpu.set_robot_description(xml)
    gpu.set_current_state(joints)
    setup['curobo_construct_sec'] = time.perf_counter()-began
    began = time.perf_counter()
    fused = FusedCudaRefinement(engine, chain, names, state, bounds, arm)
    setup['gpu_fused32_construct_sec'] = time.perf_counter()-began
    maximum = 0.
    for case in cases:
        fused.set_input(case)
        p, r = fused.fk(fused.tensor([case.seed.positions]))
        cp, cr = chain.pose({**state, **dict(zip(names, case.seed.positions))})
        maximum = max(maximum, float(np.max(np.abs(cp-p.cpu().numpy()[0]))),
                      float(np.max(np.abs(cr-r.cpu().numpy()[0]))))
    assert maximum < 2e-6, maximum
    setup['gpu_fused32_fk_max_abs_difference'] = maximum
    setup['gpu_fused32_graph_prepare_sec'] = fused.capture(cases[0])
    local['gpu_fused32'] = fused
    print(json.dumps({'progress': 'fused_graph_ready', 'setup': setup}), flush=True)
    torch.cuda.synchronize()
    methods = ['cpu_local64', 'gpu_local64_graph', 'gpu_local32_graph',
               'gpu_fused32_graph', 'gpu_local32_eager', 'curobo_gpu']
    rows = {c.label: {method: [] for method in methods} for c in cases}
    first_calls = {}

    def prepare(case):
        cpu.set_gripper_position(case.arm, case.opening)
        cpu._remote_forward_kinematics = lambda passed_arm, q: (
            case.seed_pose if passed_arm == case.arm and q == case.seed else
            (_ for _ in ()).throw(AssertionError('Only recorded seed FK parity is available')))
        cpu._verified_fk_arms.clear()
        cpu._verify_local_fk(case.arm, case.seed)
        gpu.set_gripper_position(case.arm, case.opening)

    def run(case, method):
        cpu._fk_cache.clear()
        torch.cuda.synchronize()
        began = time.perf_counter()
        steps = None
        if method == 'cpu_local64':
            result = cpu._refine_seed(case.arm, case.target, case.seed)
        elif method == 'curobo_gpu':
            proposal = gpu.solve_aligned_batch(case.arm, [case.target], [case.seed])[0]
            result = (None if proposal is None else cpu._alignment(proposal.solution, case.target,
                      cpu.forward_kinematics(case.arm, proposal.solution)))
        else:
            label = ('gpu_fused32' if 'fused' in method else
                     'gpu_local64' if '64' in method else 'gpu_local32')
            solution, steps = local[label].solve(case, graph=method.endswith('_graph'))
            result = cpu._alignment(solution, case.target, cpu.forward_kinematics(case.arm, solution))
        torch.cuda.synchronize()
        elapsed = (time.perf_counter()-began)*1000.
        return {'wall_ms': elapsed, 'steps': steps,
            'pose_feasible': result is not None and cpu._feasible_alignment(result),
            'position_error_mm': result.position_error_m*1000. if result is not None else None,
            'approach_error_deg': result.approach_error_degrees if result is not None else None,
            'closing_error_deg': result.closing_error_degrees if result is not None else None,
            'joint_positions_rad': list(result.solution.positions) if result is not None else None}

    try:
        prepare(cases[0])
        for method in methods:
            first_calls[method] = run(cases[0], method)
            print(json.dumps({'progress': 'first_call', 'method': method,
                              'wall_ms': first_calls[method]['wall_ms']}), flush=True)
        for case in cases:
            prepare(case)
            for repetition in range(args.repetitions+2):
                # Rotate method order; discard two paired warmup repetitions.
                order = methods[repetition % len(methods):]+methods[:repetition % len(methods)]
                for method in order:
                    record = run(case, method)
                    if repetition >= 2:
                        rows[case.label][method].append(record)
            a = rows[case.label]['cpu_local64'][0]
            b = rows[case.label]['gpu_local64_graph'][0]
            assert np.max(np.abs(np.array(a['joint_positions_rad'])-b['joint_positions_rad'])) < 1e-7
            assert a['pose_feasible'] == b['pose_feasible']
            print(json.dumps({'progress': 'case_complete', 'case': case.label}), flush=True)
        summary = {}
        for case, records in rows.items():
            summary[case] = {method: {**distribution([r['wall_ms'] for r in values]),
                'pose_successes': sum(r['pose_feasible'] for r in values), 'trials': len(values),
                'example': values[0]} for method, values in records.items()}
        args.output.write_text(json.dumps({'environment': {'device': torch.cuda.get_device_name(),
            'torch': torch.__version__, 'cuda': torch.version.cuda, 'numpy': np.__version__,
            'platform': platform.platform(), 'cpu_threads': torch.get_num_threads()},
            'setup': setup, 'first_calls': first_calls, 'summary': summary, 'raw': rows,
            'cases': [{'label': c.label, 'arm': c.arm, 'seed': c.seed.positions,
                'target_position': c.target.position, 'target_orientation': c.target.orientation_xyzw,
                'opening': c.opening} for c in cases],
            'local_contract': {'iterations': 10, 'seeds': 1, 'finite_difference_rad': 1e-4,
                'position_tolerance_m': .01, 'approach_tolerance_deg': 30., 'closing_tolerance_deg': 30.,
                'objective': 'position + 0.14*approach_axis + 0.08*closing_axis'},
            'curobo_contract': {'iterations': 50, 'seeds': 16, 'padded_targets': 8,
                'aim_project_retry': False, 'world': 'empty environment; built-in robot/mast costs retained'},
            'timing_scope': 'solve, transfers, synchronization, output pose check; excludes external collision RPC',
            'ros_nodes_started': False, 'motor_commands_sent': False, 'collision_checked': False,
            'production_code_changed': False,
            'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in [args.statistics, args.urdf, Path(__file__), Path(inspect.getfile(HybridGraspAdapter))]}}, indent=2))
    finally:
        engine.close()


if __name__ == '__main__':
    main()
