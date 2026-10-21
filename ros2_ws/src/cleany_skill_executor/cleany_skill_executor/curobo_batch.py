"""Batch IK for the hybrid backend and experimental GPU path benchmarks.

Every batch shares an arm, locked joint state and collision world. The runtime
hybrid adapter uses the IK APIs; GPU path batching remains experimental.
The engine uses a single jaw state per batch. Hybrid may group different
openings for proposal generation, then validates each actual opening in MoveIt.
Public IKSolver/TrajOptSolver batch APIs preserve input result order.
"""
from __future__ import annotations

from copy import deepcopy
from contextlib import nullcontext
from typing import Callable, Sequence
import time

import numpy as np

from cleany_skill_executor.core.alignment import tcp_axes
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution, PoseValue
from cleany_skill_executor.curobo_adapter import CuRoboGraspAdapter
from cleany_skill_executor.curobo_engine import CuRoboEngine, CuRoboResources
from cleany_skill_executor.curobo_model import CuRoboConfig, RobotModel
from cleany_skill_executor.curobo_scene import world_from_scene
from cleany_skill_executor.physical_moveit_adapter import MoveItGraspAdapter


class CuRoboBatchEngine(CuRoboEngine):
    def __init__(self, model: RobotModel, config: CuRoboConfig,
                 *, ik_iterations: int | None = None, trajectory_seeds: int = 1,
                 joint_plan_graph: bool = False, ik_batch_size: int | None = None,
                 adaptive_ik_batch_size: bool = False,
                 resources: CuRoboResources | None = None) -> None:
        if ik_iterations is not None and (type(ik_iterations) is not int or ik_iterations < 1):
            raise ValueError('IK iteration count must be a positive integer')
        self.ik_iterations = ik_iterations
        if type(trajectory_seeds) is not int or trajectory_seeds < 1:
            raise ValueError('JS trajectory seed count must be a positive integer')
        self.trajectory_seeds = trajectory_seeds
        self.joint_plan_graph = joint_plan_graph
        if ik_batch_size is not None and (type(ik_batch_size) is not int or ik_batch_size < 1):
            raise ValueError('IK batch size must be a positive integer')
        self.ik_batch_size = ik_batch_size
        if type(adaptive_ik_batch_size) is not bool:
            raise ValueError('Adaptive IK batch switch must be boolean')
        self.adaptive_ik_batch_size = adaptive_ik_batch_size
        super().__init__(model, config, resources=resources)

    def _progress_stage(self, phase, detail):
        progress = getattr(self, 'progress', None)
        return progress.stage(phase, detail) if progress is not None else nullcontext()

    def solve_batch(self, arm: str, tip: str, positions: Sequence,
                    seeds: Sequence[Sequence[JointSolution]], state: dict,
                    world: dict, *, continue_work: Callable[[], bool] | None = None
                    ) -> list[list[JointSolution]]:
        from curobo.types.math import Pose

        if not positions:
            return []
        if len(positions) != len(seeds) or any(not group for group in seeds):
            raise ValueError('Every IK target requires a corresponding nonempty seed group')
        target_count = len(positions)
        batch_size = getattr(self, 'ik_batch_size', None) or target_count
        if target_count > batch_size:
            raise ValueError('IK target count exceeds the configured batch size')
        if getattr(self, 'adaptive_ik_batch_size', False):
            batch_size = min(batch_size, 1 << (target_count-1).bit_length())
        positions = [*positions, *([positions[-1]] * (batch_size-target_count))]
        seeds = [*seeds, *([seeds[-1]] * (batch_size-target_count))]
        began = time.monotonic()
        # Pad ragged user seeds. IKSolver fills the remaining slots up to the
        # fixed num_seeds internally before entering its captured CUDA graph.
        width = min(self.config.num_seeds, max(len(group) for group in seeds))
        # cuRobo 0.7.8 cannot reset captured graphs in every supported Torch
        # build. Own separate solver buffers for each target batch dimension.
        # User seed width is not part of the graph shape (optimized seeds are
        # always config.num_seeds); including it needlessly recaptures graphs.
        if not hasattr(self, '_batch_ik_caches'):
            self._batch_ik_caches = {}
        cache = self._batch_ik_caches.setdefault((arm, tip, len(positions)), ({}, {}, {}))
        previous = self._ik, self._ik_locks, self._world_keys
        self._ik, self._ik_locks, self._world_keys = cache
        try:
            with self._progress_stage('IK_SOLVER_SETUP',
                    f'arm={arm} tip={tip} targets={target_count} '
                    f'padded={batch_size} seeds={self.config.num_seeds}'):
                solver = self._solver(arm, tip, state, world)
        finally:
            self._ik, self._ik_locks, self._world_keys = previous
        if continue_work is not None and not continue_work():
            # Keep the initialized solver for the real request, but do not
            # start another expensive phase after the VLM window has closed.
            raise InterruptedError('IK warmup window ended after solver setup')
        names = tuple(solver.joint_names)
        ordered = []
        for group in seeds:
            rows = [[dict(zip(q.names, q.positions))[n] for n in names]
                    for q in group[:width]]
            ordered.append(rows + [rows[-1]] * (width - len(rows)))
        with self._progress_stage('GPU_IK',
                f'arm={arm} tip={tip} targets={target_count} '
                f'padded={batch_size} seeds={self.config.num_seeds}'):
            result = solver.solve_batch(
                Pose(position=self.tensor.to_device(positions),
                     quaternion=self.tensor.to_device([[1., 0., 0., 0.]] * len(positions))),
                seed_config=self.tensor.to_device(ordered),
                retract_config=self.tensor.to_device([rows[0] for rows in ordered]),
                return_seeds=self.config.num_seeds,
                newton_iters=None,
            )
            self.torch.cuda.synchronize()
        joints = result.solution.detach().cpu().numpy().reshape(len(positions), -1, len(names))
        successes = result.success.detach().cpu().numpy().reshape(len(positions), -1)
        if hasattr(self, 'timings'):
            self.timings.setdefault('ik_batch_calls', []).append({
                'arm': arm, 'tip': tip, 'targets': target_count,
                'padded_targets': len(positions),
                'solver_seeds_per_target': int(getattr(solver, 'num_seeds', self.config.num_seeds)),
                'returned_solutions_per_target': joints.shape[1],
                'position_ik_successes': int(successes[:target_count].sum()),
                'internal_ik_success_includes_constraints': True,
                'position_ik_successes_per_target': successes[:target_count].sum(axis=1).tolist(),
                'configured_gradient_iterations': solver.solver.newton_optimizer.n_iters,
                'inner_iterations': solver.solver.newton_optimizer.inner_iters,
                'outer_iterations': solver.solver.newton_optimizer.outer_iters,
                'wall_sec': time.monotonic() - began,
            })
            # Diagnostics must stay bounded across a long-lived selector.
            del self.timings['ik_batch_calls'][:-128]
        order = [names.index(n) for n in ARM_JOINT_NAMES[arm]]
        return [[JointSolution(ARM_JOINT_NAMES[arm], tuple(float(v) for v in q[order]))
                 for q, ok in zip(group, mask) if ok and np.isfinite(q).all()]
                for group, mask in zip(joints[:target_count], successes[:target_count])]

    def _batch_motion(self, arm: str, state: dict, world: dict, batch_size: int):
        from curobo.geom.sdf.world import CollisionCheckerType
        from curobo.geom.types import WorldConfig
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

        # Separate instances prevent batch CUDA graphs from replacing the
        # current production single-query MotionGen's captured buffers.
        if not hasattr(self, '_batch_motions'):
            self._batch_motions = {}
        key = (arm, batch_size, tuple(sorted(state.items())), repr(world))
        if key not in self._batch_motions:
            self._batch_motions[key] = MotionGen(MotionGenConfig.load_from_robot_config(
                deepcopy(self.model.robot_config(self._urdf, arm, state, f'{arm}_grasp_tcp')),
                WorldConfig.from_dict(world).get_collision_check_world(),
                tensor_args=self.tensor, collision_checker_type=CollisionCheckerType.MESH,
                collision_cache={'obb': 128, 'mesh': 128},
                interpolation_dt=self.config.interpolation_dt,
                num_ik_seeds=self.config.num_seeds, num_trajopt_seeds=4,
                cspace_threshold=self.config.joint_tolerance_rad,
                num_trajopt_noisy_seeds=getattr(self, 'trajectory_seeds', 1),
                use_cuda_graph=self.config.use_cuda_graph,
            ))
        return self._batch_motions[key]

    def plan(self, arm, start, goal, state, world, velocity, acceleration, timeout, attempts):
        # Preserve the existing single-query behavior at the default of one
        # JS seed. Experimental multi-seed runs must configure the JS solver,
        # rather than only the task-space num_trajopt_seeds setting.
        if getattr(self, 'trajectory_seeds', 1) != 1 and arm not in self._motion:
            robot = self.model.robot_config(self._urdf, arm, state, f'{arm}_grasp_tcp')
            motion = self._batch_motion(arm, state, world, 1)
            locks = {name: value for name, value in state.items() if name not in ARM_JOINT_NAMES[arm]}
            self._motion[arm] = (motion, robot, locks)
        return super().plan(arm, start, goal, state, world, velocity, acceleration, timeout, attempts)

    def plan_joint_batch(self, arm: str, starts: Sequence[JointSolution],
                         goals: Sequence[JointSolution], state: dict, world: dict,
                         velocity: float = .1, acceleration: float = .1
                         ) -> list[np.ndarray | None]:
        """One batched joint-goal optimization plus finetuning, no graph retry.

        v0.7.8 exposes plan_single_js but no plan_batch_js. Use MotionGen's
        identically configured JS TrajOpt solvers through solve_batch. The
        shared finetune dt is conservative; success is retained per query.
        Final full-robot collision/endpoint checks are the caller's obligation.
        """
        from curobo.rollout.rollout_base import Goal
        from curobo.types.robot import JointState
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
        from curobo.util.trajectory import get_batch_interpolated_trajectory

        if len(starts) != len(goals) or not starts:
            raise ValueError('A nonempty, matching start/goal batch is required')
        if not 0 < velocity <= 1 or not 0 < acceleration <= 1:
            raise ValueError('Invalid trajectory scaling')
        self.model.validate_state(state)
        began = time.monotonic()
        motion = self._batch_motion(arm, state, world, len(starts))
        names = tuple(motion.kinematics.joint_names)
        def states(items):
            rows = [[dict(zip(q.names, q.positions))[n] for n in names] for q in items]
            return JointState.from_position(self.tensor.to_device(rows), joint_names=list(names))
        goal = Goal(current_state=states(starts), goal_state=states(goals))
        cfg = MotionGenPlanConfig(max_attempts=1)
        solve = time.monotonic()
        initial = motion.js_trajopt_solver.solve_batch(goal, return_all_solutions=True)
        paths: list[np.ndarray | None] = [None] * len(starts)
        finetune_calls = 0
        if bool(initial.success.any().item()):
            dt = initial.optimized_dt[initial.success].min()
            seed = initial.raw_action.clone()
            for attempt in range(cfg.finetune_attempts):
                scaled = self.torch.clamp(dt * cfg.finetune_js_dt_scale * cfg.finetune_dt_decay**attempt,
                                          motion.js_trajopt_solver.minimum_trajectory_dt)
                motion.finetune_js_trajopt_solver.update_solver_dt(float(scaled.item()))
                result = motion.finetune_js_trajopt_solver.solve_batch(
                    goal, seed_traj=seed, return_all_solutions=False,
                    newton_iters=None if attempt == 0 else 4)
                finetune_calls += 1
                success = result.success.detach().cpu().numpy().reshape(-1)
                indices = [i for i, ok in enumerate(success) if ok and paths[i] is None]
                # v0.7.8's Warp interpolator accepts only a scalar raw_dt.
                # Optimization remains batched; retime/interpolate each accepted
                # trajectory on GPU, as MotionGenResult.retime_trajectory does.
                for index in indices:
                    old_dt = result.optimized_dt[index:index+1]
                    new_dt = old_dt / min(velocity, acceleration**.5)
                    retimed = result.solution[index].scale_by_dt(old_dt, new_dt.view(-1, 1))
                    interpolated, last, _ = get_batch_interpolated_trajectory(
                        retimed, new_dt, self.config.interpolation_dt,
                        kind=motion.js_trajopt_solver.interpolation_type,
                        tensor_args=self.tensor, optimize_dt=False)
                    positions = interpolated.position.detach().cpu().numpy()[0]
                    order = [names.index(n) for n in ARM_JOINT_NAMES[arm]]
                    paths[index] = positions[:int(last[0])][:, order].copy()
                if all(path is not None for path in paths):
                    break
                seed = result.optimized_seeds.detach().clone()
        self.torch.cuda.synchronize()
        self.timings.update(batch_total_sec=time.monotonic()-began,
                            batch_optimization_sec=time.monotonic()-solve,
                            batch_finetune_calls=finetune_calls,
                            batch_actual_js_seeds=getattr(motion.js_trajopt_solver, 'num_seeds', None),
                            batch_initial_successes=int(initial.success.sum().item()))
        return paths


class CuRoboBatchAdapter(CuRoboGraspAdapter):
    def validate_approach_path(self, arm, path, start, target):
        """Same cubic samples and axis/corridor gates, with batched GPU FK."""
        from cleany_skill_executor.core.alignment import axis_errors_degrees
        from cleany_skill_executor.core.cartesian import CartesianPose, validate_corridor_samples

        samples = list(self._path_samples(path))
        if not samples:
            return False
        solutions = [JointSolution(path.joint_names, tuple(sample.positions)) for sample in samples]
        with self._scene_operation():
            valid, poses = [], []
            for begin in range(0, len(solutions), 1024):
                group = solutions[begin:begin+1024]
                mask = self._valid_batch(arm, group)
                if len(mask) != len(group) or not all(mask):
                    return False
                valid.extend(mask)
                poses.extend(self._compute(self._engine.fk, arm, f'{arm}_grasp_tcp', group, self._state()))
        if len(valid) != len(solutions) or len(poses) != len(solutions) or not all(valid):
            return False
        for pose in poses:
            approach, closing = axis_errors_degrees(target.orientation_xyzw, pose.orientation_xyzw)
            if (approach > self._config.approach_tolerance_degrees
                    or closing > self._config.closing_tolerance_degrees):
                return False
        try:
            validate_corridor_samples(
                [CartesianPose(pose.position, target.orientation_xyzw) for pose in poses],
                CartesianPose(start.position, target.orientation_xyzw),
                CartesianPose(target.position, target.orientation_xyzw),
                self._config.approach_corridor_radius_m, np.pi, np.pi)
        except ValueError:
            return False
        return True

    def _alignment_refinement_targets(self, target_count: int) -> range | set[int]:
        """Targets that need aim/project searches before this call returns."""
        return range(target_count)

    def solve_aligned_batch(self, arm: str, targets: Sequence[PoseValue],
                            seeds: Sequence[JointSolution | None] | None = None):
        """Batch each stage of the existing direct/aim/project alignment search."""
        if not targets:
            return []
        if seeds is None:
            seeds = [None] * len(targets)
        if len(seeds) != len(targets):
            raise ValueError('Target/seed batch lengths differ')
        with self._scene_operation():
            state = self._state()
            world = world_from_scene(self._active_scene, optimization=True)
            bases = [self._seed_bases(arm, seed) for seed in seeds]
            preferred = [(dict(zip(seed.names, seed.positions))[ARM_JOINT_NAMES[arm][-1]]
                          if seed is not None else state.get(ARM_JOINT_NAMES[arm][-1]))
                         for seed in seeds]
            direct = self._compute(self._engine.solve_batch, arm, f'{arm}_grasp_tcp',
                                   [t.position for t in targets], bases, state, world)
            evaluated = self._score_batch(arm, targets, direct, state, preferred)
            chosen = self._choose_batch(arm, evaluated, preferred)
            refinement_targets = self._alignment_refinement_targets(len(targets))
            pending = [i for i, item in enumerate(chosen)
                       if item is None and i in refinement_targets]
            if not pending:
                return chosen
            # Keep the captured batch dimension fixed despite the number of
            # survivors changing between calls. Padded results are discarded.
            padded = (pending if getattr(self._engine, 'adaptive_ik_batch_size', False)
                      else pending + [pending[-1]] * (len(targets) - len(pending)))
            aim_positions = [tuple(p + self._config.aim_tip_offset_m * a
                                   for p, a in zip(targets[i].position, tcp_axes(targets[i].orientation_xyzw)[0]))
                             for i in padded]
            aims = self._compute(self._engine.solve_batch, arm, f'{arm}_pregrasp_aim_tip',
                                 aim_positions, [bases[i] for i in padded], state, world)
            projected = self._compute(self._engine.solve_batch, arm, f'{arm}_grasp_tcp',
                                      [targets[i].position for i in padded],
                                      [q[:self._gpu_config.num_seeds] or bases[i] for i, q in zip(padded, aims)],
                                      state, world)
            extra = self._score_batch(arm, [targets[i] for i in pending],
                                      [a + p for a, p in zip(aims, projected)][:len(pending)],
                                      state, [preferred[i] for i in pending])
            for index, added in zip(pending, extra):
                evaluated[index].extend(added)
            chosen = self._choose_batch(arm, evaluated, preferred)
            for i, item in enumerate(chosen):
                if item is None:
                    rejected = [q for q in evaluated[i] if not self._feasible_alignment(q)]
                    chosen[i] = self._best_alignment(rejected, preferred[i]) if rejected else None
            return chosen

    def _score_batch(self, arm, targets, solutions, state, preferred_rolls=None):
        # One FK batch derives exact roll angles, then a second scores all
        # bounded variants. Keep candidate ownership through both expansions.
        originals = [q for group in solutions for q in group]
        original_poses = iter(self._compute(
            self._engine.fk, arm, f'{arm}_grasp_tcp', originals, state)
            if self._config.align_wrist_roll and originals else [])
        if preferred_rolls is None:
            preferred_rolls = [state.get(ARM_JOINT_NAMES[arm][-1])] * len(targets)
        variants, owners = [], []
        for index, group in enumerate(solutions):
            preferred_roll = preferred_rolls[index]
            unique = {}
            for q in group:
                candidates = (list(MoveItGraspAdapter._wrist_variants(self, q, preferred_roll))
                              if self._config.align_wrist_roll else list(self._wrist_variants(q)))
                if self._config.align_wrist_roll:
                    candidates += self._aligned_wrist_variants(
                        q, targets[index], next(original_poses), preferred_roll)
                for variant in candidates:
                    unique[tuple(round(v, 7) for v in variant.positions)] = variant
            variants.extend(unique.values())
            owners.extend([index] * len(unique))
        poses = self._compute(self._engine.fk, arm, f'{arm}_grasp_tcp', variants, state)
        output = [[] for _ in targets]
        for i, q, pose in zip(owners, variants, poses, strict=True):
            output[i].append(self._alignment(q, targets[i], pose))
        return output

    def _score(self, arm, target, solutions, state):
        # Full-pipeline grasp IK also needs the continuous wrist alignment.
        return self._score_batch(arm, [target], [solutions], state)[0]

    def _choose_batch(self, arm, evaluated, preferred_rolls=None):
        flat, owners = [], []
        for i, group in enumerate(evaluated):
            good = [item for item in group if self._feasible_alignment(item)]
            flat.extend(good)
            owners.extend([i] * len(good))
        valid = self._valid_batch(arm, [item.solution for item in flat])
        accepted = [[] for _ in evaluated]
        for i, item, ok in zip(owners, flat, valid):
            if ok:
                accepted[i].append(item)
        self.last_batch_diagnostics = [
            {'evaluated_variants': len(group),
             'pose_feasible': sum(self._feasible_alignment(item) for item in group),
             'collision_valid': len(accepted[i]),
             'collision_rejected_examples': [
                 list(item.solution.positions) for owner, item, ok in zip(owners, flat, valid)
                 if owner == i and not ok and hasattr(item.solution, 'positions')][:3]}
            for i, group in enumerate(evaluated)]
        return [(self._best_alignment(group, preferred_rolls[i]) if preferred_rolls is not None
                 else self._best_alignment(group)) if group else None
                for i, group in enumerate(accepted)]
