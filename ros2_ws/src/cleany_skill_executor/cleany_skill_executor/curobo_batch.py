"""Batched candidate IK for the hybrid backend.

Each batch shares an arm, locked joint state and collision world. Hybrid groups
jaw openings for proposal generation and validates actual openings in MoveIt.
"""
from __future__ import annotations

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
                 *, ik_iterations: int | None = None, ik_batch_size: int | None = None,
                 adaptive_ik_batch_size: bool = False,
                 resources: CuRoboResources | None = None) -> None:
        if ik_iterations is not None and (type(ik_iterations) is not int or ik_iterations < 1):
            raise ValueError('IK iteration count must be a positive integer')
        self.ik_iterations = ik_iterations
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
        originals = list(dict.fromkeys(q for group in solutions for q in group))
        # FK depends on the joints and this operation's frozen full state,
        # not on the target that owns a proposal. Never retain poses across calls.
        poses_by_solution = dict(zip(originals, self._compute(
            self._engine.fk, arm, f'{arm}_grasp_tcp', originals, state), strict=True)) \
            if self._config.align_wrist_roll and originals else {}
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
                        q, targets[index], poses_by_solution[q], preferred_roll)
                for variant in candidates:
                    unique[tuple(round(v, 7) for v in variant.positions)] = variant
            variants.extend(unique.values())
            owners.extend([index] * len(unique))
        missing = list(dict.fromkeys(q for q in variants if q not in poses_by_solution))
        if missing:
            poses_by_solution.update(zip(missing, self._compute(
                self._engine.fk, arm, f'{arm}_grasp_tcp', missing, state), strict=True))
        output = [[] for _ in targets]
        for i, q in zip(owners, variants, strict=True):
            output[i].append(self._alignment(q, targets[i], poses_by_solution[q]))
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
