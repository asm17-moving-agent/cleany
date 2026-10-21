"""GPU candidate IK with authoritative MoveIt FK, scene checks and CPU planning.

OctoMap is deliberately excluded only from the GPU proposal cost. Every accepted
IK state and executable path is checked by MoveIt against the complete scene.
Cartesian approach/lift services and actuator execution remain independent.
"""
from __future__ import annotations

from collections import defaultdict
from contextlib import nullcontext
from dataclasses import dataclass, replace
import math
import time
from xml.etree import ElementTree as ET
from typing import Callable, Sequence

from cleany_skill_executor.core.alignment import axis_errors_degrees
from cleany_skill_executor.core.ik_diagnostics import (
    AlignmentLimits, alignment_errors, summarize_alignments,
)
from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, InfrastructureError, JointSolution, PoseAlignment,
    PlannedPath, PoseValue, PregraspRequest,
)
from cleany_skill_executor.curobo_batch import CuRoboBatchAdapter, CuRoboBatchEngine
from cleany_skill_executor.curobo_engine import CuRoboResources
from cleany_skill_executor.curobo_model import CuRoboConfig, RobotModel
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter
from cleany_skill_executor.core.planning_progress import PlanningProgress


@dataclass(frozen=True)
class HybridConfig:
    batch_size: int = 8
    ik_iterations: int = 50
    fallback_seeds: int = 32
    exclude_opposite_arm_cost: bool = True
    retry_failed_candidates: bool = True
    pregrasp_retry_passes: int = 1
    cpu_ik_fallback: bool = False
    planning_retries: int = 1
    adaptive_batch_size: bool = False
    progressive_pregrasp: bool = False
    group_gripper_openings: bool = False
    defer_candidate_ik_retries: bool = False

    def __post_init__(self) -> None:
        if any(type(v) is not int or v < 1 for v in (
                self.batch_size, self.ik_iterations, self.fallback_seeds)):
            raise ValueError('Hybrid batch, iterations and fallback seeds must be positive integers')
        if any(type(v) is not bool for v in (
                self.exclude_opposite_arm_cost, self.retry_failed_candidates,
                self.cpu_ik_fallback, self.adaptive_batch_size, self.progressive_pregrasp,
                self.group_gripper_openings, self.defer_candidate_ik_retries)):
            raise ValueError('Hybrid switches must be boolean')
        if any(type(v) is not int or not 0 <= v <= 2 for v in (
                self.pregrasp_retry_passes, self.planning_retries)):
            raise ValueError('Hybrid retries must be integers between zero and two')


class _CandidateRobotModel(RobotModel):
    def __init__(self, description, srdf, config, *, exclude_opposite_arm_cost):
        super().__init__(description, srdf, config)
        self._exclude_opposite_arm_cost = exclude_opposite_arm_cost
        required = set(self.limits)
        self._joint_links = [joint.find('child').get('link')
            for joint in ET.fromstring(description).findall('joint')
            if joint.get('name') in required]

    def robot_config(self, path, arm, state, tip):
        robot = super().robot_config(path, arm, state, tip)
        if not self._exclude_opposite_arm_cost:
            return robot
        # Preserve both joint chains and measured locked joints, while the GPU
        # candidate cost omits the stationary arm's shapes as requested.
        opposite = 'right' if arm == 'left' else 'left'
        kin = robot['kinematics']
        spheres = {name:value for name,value in kin['collision_spheres'].items()
                   if not name.startswith(opposite+'_')}
        kin.update(collision_link_names=list(spheres), collision_spheres=spheres,
            self_collision_ignore={name:[other for other in others if other in spheres]
                for name,others in kin['self_collision_ignore'].items() if name in spheres},
            self_collision_buffer={name:0. for name in spheres},
            link_names=list(dict.fromkeys([tip,*self._joint_links])))
        return robot


class _CandidateGpuAdapter(CuRoboBatchAdapter):
    def __init__(self, *args, owner: HybridGraspAdapter, **kwargs):
        self._owner = owner
        super().__init__(*args, **kwargs)

    def _progress_stage(self, phase, detail):
        progress = getattr(self._owner, '_progress', None)
        return progress.stage(phase, detail) if progress is not None else nullcontext()

    def _score_batch(self, arm, targets, solutions, state, preferred_rolls=None):
        with self._progress_stage('GPU_FK_ALIGNMENT',
                f'arm={arm} targets={len(targets)} solutions={sum(map(len, solutions))}'):
            return super()._score_batch(arm, targets, solutions, state, preferred_rolls)

    def _snapshot(self):
        # The proposal never consumes OctoMap. Exclude it from the service
        # response instead of transferring and then copying/discarding it.
        self._owner.last_ik_statistics['gpu_cost_omits_octomap'] = True
        return super()._snapshot(include_octomap=False)

    def _valid_batch(self, arm, solutions):
        # This always uses the complete live MoveIt scene, including OctoMap,
        # exact target/link ACM and attached objects. No GPU acceptance shortcut.
        return [self._owner.state_is_valid(arm, solution) for solution in solutions]

    def _choose_valid(self, arm, evaluated):
        return self._choose_batch(arm, [evaluated])[0]

    def _alignment_refinement_targets(self, target_count: int) -> range | set[int]:
        if getattr(getattr(self, '_owner', None), '_primary_only', False):
            return set()
        targets = getattr(self, 'validation_targets', None)
        return range(target_count) if targets is None else targets

    def _choose_batch(self, arm, evaluated, preferred_rolls=None):
        """Check ranked IK states until each target has a live MoveIt witness.

        Only the Hybrid RPC path short-circuits. GPU collision batches and final
        endpoint/path validation keep their existing behavior. Validity results
        are never cached across targets, stages or scene operations.
        """
        with self._progress_stage('MOVEIT_COLLISION',
                f'arm={arm} targets={len(evaluated)} checked=0'):
            return self._choose_moveit_batch(arm, evaluated, preferred_rolls)

    def _choose_moveit_batch(self, arm, evaluated, preferred_rolls=None):
        chosen, diagnostics = [], []
        self._owner._check_active()
        for index, group in enumerate(evaluated):
            validation_targets = getattr(self, 'validation_targets', None)
            proposal_only = validation_targets is not None and index not in validation_targets
            openings = getattr(self, 'validation_openings', None)
            if openings is not None and not proposal_only:
                # GPU proposals share one jaw state; live MoveIt never does.
                self._owner.set_gripper_position(arm, openings[index])
            good = [item for item in group if self._feasible_alignment(item)]
            roll = preferred_rolls[index] if preferred_rolls is not None else None
            ordered = sorted(good, key=self._alignment_sort_key(good, roll)) if good else []
            checked, rejected, selected = 0, [], None
            if proposal_only and ordered:
                selected = ordered[0]
            for item in ordered:
                if proposal_only:
                    break
                self._owner._check_active()
                checked += 1
                progress = getattr(self._owner, '_progress', None)
                if progress is not None:
                    progress.update_detail(f'arm={arm} target={index+1}/{len(evaluated)} '
                                           f'checked={checked}/{len(ordered)}')
                if self._owner.state_is_valid(arm, item.solution):
                    selected = item
                    break
                if len(rejected) < 3:
                    rejected.append(list(item.solution.positions))
            chosen.append(selected)
            diagnostics.append({
                **summarize_alignments(group, AlignmentLimits(
                    self._config.position_tolerance_m,
                    self._config.approach_tolerance_degrees,
                    self._config.closing_tolerance_degrees)),
                'collision_valid': int(selected is not None and not proposal_only),
                'proposal_only': proposal_only,
                'collision_checked': checked,
                'collision_unchecked': len(good) - checked,
                'collision_valid_count_complete': checked == len(good),
                'collision_rejected_examples': rejected,
            })
        self._owner._check_active()
        self.last_batch_diagnostics = diagnostics
        return chosen


class _CandidateCpuAdapter(MoveItGraspAdapter):
    """Share native services and the owning goal's cancellation/deadline."""
    def __init__(self, *args, owner: HybridGraspAdapter, **kwargs):
        self._owner = owner
        super().__init__(*args, **kwargs)

    def _check_active(self) -> None:
        self._owner._check_active()
        super()._check_active()

    def _call(self, client, request, timeout):
        return self._owner._call(client, request, timeout)


class HybridGraspAdapter(MoveItGraspAdapter):
    """The standard selector port: only candidate IK is delegated to cuRobo."""
    def __init__(self, node, config: MoveItAdapterConfig, *,
                 gpu_config: CuRoboConfig = CuRoboConfig(num_seeds=16),
                 hybrid_config: HybridConfig = HybridConfig(),
                 gpu_factory: Callable | None = None, **kwargs):
        super().__init__(node, config, **kwargs)
        if gpu_config.num_seeds >= hybrid_config.fallback_seeds:
            raise ValueError('Fallback seed count must exceed primary seeds')
        self._node = node
        self._gpu_config = gpu_config
        self._hybrid_config = hybrid_config
        self._gpu_factory = gpu_factory or self._create_gpu
        self._gpus = {}
        self._robot_model = None
        self._resources = CuRoboResources()
        self._cpu_ik: _CandidateCpuAdapter | None = None
        self._description = ''
        self._pregrasp_cache = {}
        self._pregrasp_requests: list[PregraspRequest] = []
        self._pregrasp_finalized: set[PregraspRequest] = set()
        self._pregrasp_alignment_pending: set[PregraspRequest] = set()
        self._prepared_grasps = False
        self._progress = PlanningProgress(self._monotonic)
        self._ik_context: tuple[int, str, PoseValue, str] | None = None
        self._ik_diagnostic_callback: Callable[[dict], None] | None = None
        self._primary_only = False
        self._deferred_ik = False
        self._reset_statistics()

    @property
    def prepares_pregrasps_lazily(self) -> bool:
        return self._prepared_grasps or self._hybrid_config.progressive_pregrasp

    def warmup(self, arms: Sequence[str], *, include_aim: bool = True,
               continue_work: Callable[[], bool] | None = None) -> None:
        """Capture primary IK buffers without proposing or executing a motion."""
        from cleany_skill_executor.core.urdf_fk import UrdfChain

        if not arms or any(arm not in ARM_JOINT_NAMES for arm in arms):
            raise ValueError('Warmup requires left/right arms')
        self._check_active()
        if continue_work is not None and not continue_work():
            raise InterruptedError('IK warmup window ended')
        state = dict(zip(self._current_state.name, self._current_state.position, strict=True))
        gpu = self._gpu(self._gpu_config.num_seeds)
        for arm in dict.fromkeys(arms):
            seeds = gpu._seed_bases(arm, None)
            tips = [f'{arm}_grasp_tcp']
            if include_aim:
                tips.append(f'{arm}_pregrasp_aim_tip')
            for tip in tips:
                self._check_active()
                if continue_work is not None and not continue_work():
                    raise InterruptedError('IK warmup window ended')
                position, _ = UrdfChain(self._description, self._config.base_frame, tip).pose(state)
                with self._progress.stage('IK_WARMUP', f'arm={arm} tip={tip}'):
                    # Results are discarded. Actual requests still refresh the
                    # world, locked joints, FK and complete MoveIt scene checks.
                    gpu._engine.solve_batch(arm, tip,
                        [tuple(float(v) for v in position)] * self._hybrid_config.batch_size,
                        [seeds] * self._hybrid_config.batch_size, state, {},
                        **({'continue_work': continue_work} if continue_work is not None else {}))

    def begin_candidate_pass(self, retry: bool) -> bool:
        """Start one grasp-first pass or a legacy pregrasp retry pass."""
        if self._prepared_grasps:
            # Grasp-first selection visits each candidate once. A failed grasp
            # or seeded pregrasp goes straight to the next candidate.
            self._check_active()
            self._primary_only = True
            self._deferred_ik = False
            return False
        if not (self._hybrid_config.defer_candidate_ik_retries
                and self.prepares_pregrasps_lazily):
            return False
        self._check_active()
        self._primary_only = not retry
        if not retry:
            self._deferred_ik = False
            return True
        self._pregrasp_finalized.difference_update(
            r for r, value in self._pregrasp_cache.items() if value is None)
        return self._deferred_ik

    def set_progress_callback(self, callback: Callable[[str], None] | None) -> None:
        self._progress.callback = callback

    def set_ik_diagnostic_callback(self, callback: Callable[[dict], None] | None) -> None:
        self._ik_diagnostic_callback = callback

    def set_ik_context(self, candidate_index: int, arm: str, target: PoseValue,
                       *, pregrasp: bool) -> None:
        self._ik_context = (candidate_index, arm, target,
                            'PREGRASP_IK' if pregrasp else 'GRASP_IK')

    def _alignment_limits(self) -> AlignmentLimits:
        return AlignmentLimits(self._config.position_tolerance_m,
                               self._config.approach_tolerance_degrees,
                               self._config.closing_tolerance_degrees)

    def _ik_diagnostic(self, arm: str, target: PoseValue, opening: float | None,
                       backend: str) -> dict:
        index, stage = -1, 'GRASP_IK'
        for request in self._pregrasp_requests:
            if (request.arm, request.target, request.gripper_position) == (arm, target, opening):
                index = request.candidate_index
                stage = 'GRASP_IK' if self._prepared_grasps else 'PREGRASP_IK'
                break
        if self._ik_context is not None and self._ik_context[1:3] == (arm, target):
            index, _, _, stage = self._ik_context
        return {
            'candidate_index': index, 'arm': arm, 'stage': stage, 'backend': backend,
            'target': {'frame_id': self._config.base_frame,
                       'position_m': list(target.position),
                       'orientation_xyzw': list(target.orientation_xyzw)},
            'gripper_joint_position_rad': opening,
            'limits': self._alignment_limits().as_dict(),
        }

    def _record_ik_diagnostic(self, diagnostic: dict) -> None:
        diagnostic['recorded_at_ns'] = time.time_ns()
        records = self.last_ik_statistics['ik_diagnostics']
        records.append(diagnostic)
        del records[:-512]
        if self._ik_diagnostic_callback is not None:
            self._ik_diagnostic_callback(diagnostic)

    def computation_progress(self) -> str:
        return self._progress.current_message()

    def computation_timings(self) -> list[dict]:
        return list(self._progress.records)

    @property
    def planner_id(self) -> str:
        return 'curobo_ik_moveit_' + self._config.planner_id

    def _reset_statistics(self) -> None:
        self.last_ik_statistics = {
            'backend': 'curobo_ik_moveit', 'primary_seeds': self._gpu_config.num_seeds,
            'fallback_seeds': self._hybrid_config.fallback_seeds,
            'ik_iterations': self._hybrid_config.ik_iterations,
            'batch_size': self._hybrid_config.batch_size,
            'primary_batches': 0, 'fallback_batches': 0,
            'primary_retry_batches': 0, 'primary_retry_targets': 0,
            'retry_failed_candidates': self._hybrid_config.retry_failed_candidates,
            'pregrasp_retry_passes': self._hybrid_config.pregrasp_retry_passes,
            'fallback_targets': 0, 'cpu_ik_fallback_enabled': self._hybrid_config.cpu_ik_fallback,
            'cpu_ik_fallback_calls': [], 'planning_calls': [],
            'pregrasp_retry_arms': [], 'grasp_retry_calls': 0,
            'gpu_cost_omits_octomap': False, 'final_scene_validation': 'moveit',
            'gpu_cost_excludes_opposite_arm': self._hybrid_config.exclude_opposite_arm_cost,
            'batch_calls': [],
            'adaptive_batch_size': self._hybrid_config.adaptive_batch_size,
            'progressive_pregrasp': self._hybrid_config.progressive_pregrasp,
            'group_gripper_openings': self._hybrid_config.group_gripper_openings,
            'ik_diagnostics': [],
            'start_joint_state': {
                'names': list(self._current_state.name) if self._current_state is not None else [],
                'positions_rad': list(self._current_state.position) if self._current_state is not None else [],
            },
        }

    def close(self) -> None:
        for gpu in self._gpus.values():
            close = getattr(gpu._engine, 'close', None)
            if close is not None:
                close()
        self._gpus.clear()
        self._cpu_ik = None
        self._pregrasp_cache.clear()
        self._pregrasp_requests.clear()
        self._pregrasp_finalized.clear()
        self._pregrasp_alignment_pending.clear()
        self._prepared_grasps = False
        self._robot_model = None
        self._resources = CuRoboResources()

    def set_robot_description(self, description: str) -> None:
        if self._hybrid_config.group_gripper_openings:
            joints = {joint.find('child').get('link'): joint
                      for joint in ET.fromstring(description).findall('joint')}
            for arm in ARM_JOINT_NAMES:
                link, visited = f'{arm}_grasp_tcp', set()
                while link in joints:
                    if link in visited:
                        raise ValueError('Robot TCP chain contains a cycle')
                    visited.add(link)
                    joint = joints[link]
                    if joint.get('name') in ('left_gripper_joint', 'right_gripper_joint'):
                        raise ValueError('Grouped openings require a TCP independent of the gripper joint')
                    link = joint.find('parent').get('link')
        super().set_robot_description(description)
        if description != self._description:
            self.close()
            self._description = description

    def set_current_state(self, state) -> None:
        super().set_current_state(state)
        self._primary_only = False
        self._deferred_ik = False
        self._pregrasp_cache.clear()
        self._pregrasp_requests.clear()
        self._pregrasp_finalized.clear()
        self._pregrasp_alignment_pending.clear()
        self._prepared_grasps = False
        self._ik_context = None
        self._progress.reset()
        self._reset_statistics()
        for gpu in self._gpus.values():
            gpu.set_current_state(state)

    def set_deadline(self, deadline: float) -> None:
        super().set_deadline(deadline)
        for gpu in self._gpus.values():
            gpu.set_deadline(deadline)

    def cancel_active(self) -> None:
        super().cancel_active()
        for gpu in self._gpus.values():
            gpu.cancel_active()
        if self._cpu_ik is not None:
            self._cpu_ik.cancel_active()

    def _create_gpu(self, description: str, config: CuRoboConfig):
        from pathlib import Path
        from ament_index_python.packages import get_package_share_directory
        import xacro

        srdf = Path(get_package_share_directory('cleany_moveit_config')) / 'config/cleany.srdf.xacro'
        has_head = any(link.get('name') == 'head_pan_link'
                       for link in ET.fromstring(description).findall('link'))
        semantic = xacro.process_file(str(srdf),
            mappings={'include_head_camera': str(has_head).lower()}).toxml()
        # Build from the actual runtime URDF, including measured head/TCP frames.
        if self._robot_model is None:
            with self._progress.stage('GPU_GEOMETRY',
                    f'mesh_fit_spheres={config.spheres_per_link} link_sphere_budget={config.sphere_budget_per_link}'):
                self._robot_model = _CandidateRobotModel(description, semantic, config,
                    exclude_opposite_arm_cost=self._hybrid_config.exclude_opposite_arm_cost)
        model = self._robot_model.with_config(config)
        engine = CuRoboBatchEngine(model, config,
            ik_iterations=self._hybrid_config.ik_iterations,
            ik_batch_size=self._hybrid_config.batch_size,
            adaptive_ik_batch_size=self._hybrid_config.adaptive_batch_size,
            resources=self._resources)
        engine.progress = self._progress
        try:
            gpu = _CandidateGpuAdapter(
                self._node, self._config, owner=self, gpu_config=config, engine=engine,
                ik_client=self._ik_client, fk_client=self._fk_client,
                validity_client=self._validity_client, plan_client=self._plan_client,
                spin_once=self._spin_once)
            gpu.set_robot_description(description)
            return gpu
        except BaseException:
            engine.close()
            raise

    def _gpu(self, seeds: int):
        self._check_active()
        if not self._description:
            raise InfrastructureError('Runtime robot_description is required for GPU IK')
        if seeds not in self._gpus:
            try:
                with self._progress.stage('GPU_INITIALIZATION', f'seeds={seeds}'):
                    self._gpus[seeds] = self._gpu_factory(
                        self._description, replace(self._gpu_config, num_seeds=seeds,
                            # Proposal success must not discard pregrasp solutions
                            # before the stage-specific FK/alignment checks run.
                            ik_position_threshold_m=max(
                                self._gpu_config.ik_position_threshold_m,
                                self._config.pregrasp_position_tolerance_m or 0.0)))
            except Exception as error:
                raise InfrastructureError(f'GPU IK initialization failed: {error}') from error
        gpu = self._gpus[seeds]
        # Cached engines must use the current stage's alignment tolerance.
        gpu._config = self._config
        gpu.set_current_state(self._current_state)
        gpu.set_deadline(self._deadline)
        return gpu

    def _accept_alignment(self, arm: str, target: PoseValue,
                          result: PoseAlignment | None,
                          diagnostic: dict | None = None) -> PoseAlignment | None:
        self._check_active()
        if diagnostic is not None:
            diagnostic.update(
                proposal_alignment=(alignment_errors(result, self._alignment_limits())
                                    if result is not None else None),
                outcome='no_ik_solution' if result is None else 'pose_rejected',
                failure_phase=('GPU_FK_ALIGNMENT' if diagnostic['backend'].startswith('gpu')
                               else 'CPU_IK_FALLBACK'))
        if result is None or not self._feasible_alignment(result):
            return None
        if result.solution.names != ARM_JOINT_NAMES[arm]:
            raise InfrastructureError('Candidate IK joint ownership mismatch')
        with self._progress.stage('FINAL_FK_CHECK', f'arm={arm}'):
            pose = self.forward_kinematics(arm, result.solution)
        approach, closing = axis_errors_degrees(target.orientation_xyzw, pose.orientation_xyzw)
        alignment = PoseAlignment(result.solution, pose,
            math.dist(target.position, pose.position), approach, closing)
        if diagnostic is not None:
            diagnostic['final_alignment'] = alignment_errors(alignment, self._alignment_limits())
        with self._progress.stage('FINAL_STATE_CHECK', f'arm={arm}'):
            if not self._feasible_alignment(alignment):
                if diagnostic is not None:
                    diagnostic.update(outcome='final_fk_rejected', failure_phase='FINAL_FK_CHECK')
                return None
            valid = self.state_is_valid(arm, result.solution)
            if diagnostic is not None:
                diagnostic.update(outcome='accepted' if valid else 'final_collision_rejected',
                                  failure_phase=None if valid else 'FINAL_STATE_CHECK')
            return alignment if valid else None

    def _cpu_fallback(self, arm: str, target: PoseValue,
                      seed: JointSolution | None) -> PoseAlignment | None:
        self._check_active()
        if not self._hybrid_config.cpu_ik_fallback:
            return None
        if self._cpu_ik is None:
            self._cpu_ik = _CandidateCpuAdapter(self._node, self._config, owner=self,
                ik_client=self._ik_client, fk_client=self._fk_client,
                validity_client=self._validity_client, plan_client=self._plan_client,
                monotonic=self._monotonic, spin_once=self._spin_once)
            self._cpu_ik.set_robot_description(self._description)
        cpu = self._cpu_ik
        cpu._config = self._config
        cpu.set_current_state(self._current_state)
        cpu.set_deadline(self._deadline)
        for selected_arm, opening in self._gripper_positions.items():
            cpu.set_gripper_position(selected_arm, opening)
        began = self._monotonic()
        diagnostic = self._ik_diagnostic(arm, target, self._gripper_positions.get(arm),
                                         'cpu_fallback')
        diagnostic['evaluation_scope'] = 'returned_proposal'
        diagnostic['solver_collision_filter_enabled'] = cpu._config.ik_collision_search
        with self._progress.stage('CPU_IK_FALLBACK', f'arm={arm}'):
            proposal = cpu.solve_aligned_ik(arm, target, seed)
            diagnostic.update(summarize_alignments(
                [proposal] if proposal is not None else [], self._alignment_limits()))
            result = self._accept_alignment(arm, target, proposal, diagnostic)
        diagnostic['wall_sec'] = self._monotonic()-began
        self._record_ik_diagnostic(diagnostic)
        self.last_ik_statistics['cpu_ik_fallback_calls'].append({
            'arm': arm, 'target_position': list(target.position),
            'accepted': result is not None, 'wall_sec': self._monotonic()-began})
        return result

    def _batch(self, arm: str, targets: Sequence[PoseValue],
               seeds: Sequence[JointSolution | None], count: int,
               *, pad: bool = False, retry_pass: int = 0,
               openings: Sequence[float] | None = None,
               validation_targets: set[int] | None = None):
        self._check_active()
        began = time.monotonic()
        gpu = self._gpu(count)
        opening = self._gripper_positions.get(arm)
        actual_openings = list(openings) if openings is not None else [opening]*len(targets)
        if len(actual_openings) != len(targets) or any(o is None for o in actual_openings):
            raise ValueError('Every batched target requires its actual gripper joint position')
        # The TCP is fixed to the wrist in Cleany's URDF. One representative jaw
        # state guides proposals only; it is not a conservative collision proof.
        proposal_opening = max(actual_openings)
        gpu.set_gripper_position(arm, proposal_opening)
        gpu.validation_openings = actual_openings
        gpu.validation_targets = validation_targets
        width = self._hybrid_config.batch_size
        if self._hybrid_config.adaptive_batch_size:
            width = min(width, 1 << (len(targets)-1).bit_length())
        # Engine padding is discarded before FK/scoring/MoveIt validation.
        try:
            with self._progress.stage('CANDIDATE_IK',
                    f'arm={arm} targets={len(targets)} padded={width} seeds={count} retry={retry_pass}'):
                results = gpu.solve_aligned_batch(arm, targets, seeds)
        finally:
            gpu.validation_openings = None
            gpu.validation_targets = None
            if opening is not None:
                self.set_gripper_position(arm, opening)
        if len(results) != len(targets):
            raise InfrastructureError('GPU IK returned an incomplete batch')
        self._check_active()
        label = 'primary_batches' if count == self._gpu_config.num_seeds else 'fallback_batches'
        self.last_ik_statistics[label] += 1
        if retry_pass:
            self.last_ik_statistics['primary_retry_batches'] += 1
            self.last_ik_statistics['primary_retry_targets'] += len(targets)
        if count == self._hybrid_config.fallback_seeds:
            self.last_ik_statistics['fallback_targets'] += len(targets)
        call = {
            'arm': arm, 'targets': len(targets), 'padded_targets': width,
            'stage': ('grasp' if self._prepared_grasps and pad
                      else 'pregrasp' if pad else 'single_target'),
            'seeds': count, 'gripper_position': proposal_opening,
            'actual_gripper_positions': actual_openings, 'retry_pass': retry_pass}
        self.last_ik_statistics['batch_calls'].append(call)
        accepted = []
        gpu_diagnostics = getattr(gpu, 'last_batch_diagnostics', [])
        try:
            for index, (target, result, actual_opening) in enumerate(
                    zip(targets, results, actual_openings, strict=True)):
                diagnostic = self._ik_diagnostic(arm, target, actual_opening,
                    'gpu_primary' if count == self._gpu_config.num_seeds else 'gpu_retry')
                diagnostic.update(seeds=count, retry_pass=retry_pass,
                                  batch_target_index=index,
                                  evaluation_scope='scored_target',
                                  **(gpu_diagnostics[index] if index < len(gpu_diagnostics)
                                     else summarize_alignments(
                                         [result] if result is not None else [],
                                         self._alignment_limits())))
                if validation_targets is not None and index not in validation_targets:
                    accepted.append(result if result is not None
                                    and self._feasible_alignment(result) else None)
                    diagnostic.update(outcome='deferred_proposal' if accepted[-1] is not None
                                      else 'pose_rejected' if diagnostic['evaluated_variants']
                                      else 'no_ik_solution',
                                      failure_phase=None if accepted[-1] is not None
                                      else 'GPU_FK_ALIGNMENT' if diagnostic['evaluated_variants']
                                      else 'GPU_IK')
                    self._record_ik_diagnostic(diagnostic)
                    continue
                self.set_gripper_position(arm, actual_opening)
                accepted.append(self._accept_alignment(arm, target, result, diagnostic))
                if (accepted[-1] is None and diagnostic['pose_feasible']
                        and diagnostic.get('collision_checked', 0)
                        and not diagnostic.get('collision_valid', 0)
                        and 'final_alignment' not in diagnostic):
                    diagnostic.update(outcome='collision_rejected', failure_phase='MOVEIT_COLLISION')
                elif result is None:
                    if diagnostic['evaluated_variants']:
                        diagnostic.update(outcome='pose_rejected', failure_phase='GPU_FK_ALIGNMENT')
                    else:
                        diagnostic.update(outcome='no_ik_solution', failure_phase='GPU_IK')
                self._record_ik_diagnostic(diagnostic)
        finally:
            if opening is not None:
                self.set_gripper_position(arm, opening)
        call.update(accepted_targets=sum(item is not None for i,item in enumerate(accepted)
                                        if validation_targets is None or i in validation_targets),
                    deferred_proposals=sum(item is not None for i,item in enumerate(accepted)
                                           if validation_targets is not None and i not in validation_targets),
                    wall_sec=time.monotonic()-began)
        return accepted

    def prepare_pregrasps(self, requests: Sequence[PregraspRequest]) -> None:
        self._prepared_grasps = False
        with self._pregrasp_alignment():
            self._prepare_pregrasps(requests)

    def prepare_grasps(self, requests: Sequence[PregraspRequest]) -> None:
        """Register final grasp targets; batch lazily under the correct target ACM."""
        self._prepared_grasps = True
        self._prepare_pregrasps(requests)

    def _prepare_pregrasps(self, requests: Sequence[PregraspRequest]) -> None:
        self._pregrasp_cache.clear()
        self._pregrasp_finalized.clear()
        self._pregrasp_alignment_pending.clear()
        self._pregrasp_requests = list(dict.fromkeys(requests))
        self._check_active()
        if self.prepares_pregrasps_lazily:
            return
        groups = defaultdict(list)
        # Different candidate jaw openings cannot share a frozen GPU robot state.
        for request in dict.fromkeys(requests):
            groups[(request.arm, request.gripper_position)].append(request)
        chunks = []
        size = self._hybrid_config.batch_size
        for (arm, opening), group in groups.items():
            for start in range(0, len(group), size):
                chunks.append((arm, opening, group[start:start+size]))
        for arm, opening, group in chunks:
            self.set_gripper_position(arm, opening)
            targets = [r.target for r in group]
            values = self._batch(arm, targets, [None]*len(targets), self._gpu_config.num_seeds, pad=True)
            for request, value in zip(group, values):
                self._pregrasp_cache[request] = value
        if self._hybrid_config.retry_failed_candidates:
            retry_arms = {r.arm for r, value in self._pregrasp_cache.items() if value is None}
        else:
            # Explicit compatibility mode for comparisons with the earlier policy.
            retry_arms = {r.arm for r in requests} - {
                r.arm for r, value in self._pregrasp_cache.items() if value is not None}
        self.last_ik_statistics['pregrasp_retry_arms'] = sorted(retry_arms)
        for retry_pass in range(1, self._hybrid_config.pregrasp_retry_passes+2):
            primary = retry_pass <= self._hybrid_config.pregrasp_retry_passes
            count = self._gpu_config.num_seeds if primary else self._hybrid_config.fallback_seeds
            for arm, opening, group in chunks:
                pending = [r for r in group if arm in retry_arms and self._pregrasp_cache[r] is None]
                if not pending:
                    continue
                self.set_gripper_position(arm, opening)
                values = self._batch(arm, [r.target for r in pending], [None]*len(pending),
                    count, pad=True, retry_pass=retry_pass if primary else 0)
                for request, value in zip(pending, values, strict=True):
                    self._pregrasp_cache[request] = value
        for request in dict.fromkeys(requests):
            if request.arm in retry_arms and self._pregrasp_cache[request] is None:
                self.set_gripper_position(request.arm, request.gripper_position)
                self._pregrasp_cache[request] = self._cpu_fallback(request.arm, request.target, None)

    def solve_aligned_ik(self, arm, target, seed):
        key = PregraspRequest(arm, target, self._gripper_positions.get(arm, math.nan))
        if (seed is None and self.prepares_pregrasps_lazily
                and key in self._pregrasp_requests):
            return self._solve_progressive_pregrasp(key)
        if seed is None and key in self._pregrasp_cache:
            self._check_active()
            return self._pregrasp_cache[key]
        result = self._batch(arm, [target], [seed], self._gpu_config.num_seeds)[0]
        if result is None and self._primary_only:
            self._deferred_ik = True
            return None
        if result is None:
            self.last_ik_statistics['grasp_retry_calls'] += int(seed is not None)
            result = self._batch(arm, [target], [seed], self._hybrid_config.fallback_seeds)[0]
        return result if result is not None else self._cpu_fallback(arm, target, seed)

    def _solve_progressive_pregrasp(self, request: PregraspRequest):
        """Prime compatible primary targets; retry only the requested candidate."""
        self._check_active()
        arm, target = request.arm, request.target
        current_was_checked = False
        if request not in self._pregrasp_cache:
            group = [request, *[r for r in self._pregrasp_requests
                if r != request and r not in self._pregrasp_cache
                and r.arm == arm and (self._hybrid_config.group_gripper_openings
                    or r.gripper_position == request.gripper_position)]]
            group = group[:self._hybrid_config.batch_size]
            values = self._batch(arm, [r.target for r in group], [None]*len(group),
                                 self._gpu_config.num_seeds, pad=True,
                                 openings=[r.gripper_position for r in group],
                                 validation_targets={0})
            self._pregrasp_cache.update(zip(group, values, strict=True))
            # A later candidate may have no direct proposal because its costly
            # aim/project search was deliberately deferred. Preserve its full
            # primary search when selection actually reaches that candidate.
            self._pregrasp_alignment_pending.update(
                r for r, value in zip(group[1:], values[1:], strict=True)
                if value is None)
            current_was_checked = True
        if request in self._pregrasp_alignment_pending and not self._primary_only:
            self._pregrasp_cache[request] = self._batch(
                arm, [target], [None], self._gpu_config.num_seeds,
                openings=[request.gripper_position])[0]
            self._pregrasp_alignment_pending.discard(request)
            current_was_checked = True
        if request in self._pregrasp_finalized:
            return self._pregrasp_cache[request]
        value = self._pregrasp_cache[request]
        if value is not None and not current_was_checked:
            diagnostic = self._ik_diagnostic(arm, target, request.gripper_position, 'gpu_cached')
            diagnostic.update(summarize_alignments([value], self._alignment_limits()))
            value = self._accept_alignment(arm, target, value, diagnostic)
            self._record_ik_diagnostic(diagnostic)
        if value is None and self._primary_only:
            self._deferred_ik = True
            self._pregrasp_alignment_pending.add(request)
        elif value is None:
            retry = self._hybrid_config.retry_failed_candidates or not any(
                r.arm == arm and v is not None for r, v in self._pregrasp_cache.items())
            if retry:
                if arm not in self.last_ik_statistics['pregrasp_retry_arms']:
                    self.last_ik_statistics['pregrasp_retry_arms'].append(arm)
                for retry_pass in range(1, self._hybrid_config.pregrasp_retry_passes+2):
                    primary = retry_pass <= self._hybrid_config.pregrasp_retry_passes
                    count = self._gpu_config.num_seeds if primary else self._hybrid_config.fallback_seeds
                    value = self._batch(arm, [target], [None], count, pad=True,
                                        retry_pass=retry_pass if primary else 0)[0]
                    if value is not None:
                        break
                if value is None:
                    value = self._cpu_fallback(arm, target, None)
        self._check_active()
        self._pregrasp_cache[request] = value
        self._pregrasp_finalized.add(request)
        return value

    def plan(self, arm: str, goal: JointSolution,
             start: JointSolution | None) -> PlannedPath | None:
        for attempt in range(self._hybrid_config.planning_retries+1):
            self._check_active()
            began = self._monotonic()
            with self._progress.stage('MOVEIT_PLAN', f'arm={arm} attempt={attempt}'):
                result = super().plan(arm, goal, start)
            self.last_ik_statistics['planning_calls'].append({
                'arm': arm, 'attempt': attempt, 'accepted': result is not None,
                'route': getattr(self, 'last_plan_route', 'rrt'),
                'wall_sec': self._monotonic()-began})
            if result is not None:
                return result
        return None

    def solve_position_ik(self, arm, position, seed):
        raise InfrastructureError('Hybrid candidate IK requires an aligned TCP target')
