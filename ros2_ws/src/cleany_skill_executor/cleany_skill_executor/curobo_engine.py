"""Optional cuRobo v0.7.8 GPU backend; no ROS or actuator calls."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import importlib.metadata
import time
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES,
    JointSolution,
    PoseValue,
)
from cleany_skill_executor.curobo_model import CuRoboConfig, RobotModel


@dataclass
class CuRoboResources:
    """Resources shared by serial IK calls within one selector, never globally."""
    world_checker: object | None = None
    world_key: str | None = None
    fk_model: object | None = None


class CuRoboEngine:
    def __init__(self, model: RobotModel, config: CuRoboConfig,
                 *, resources: CuRoboResources | None = None) -> None:
        import torch
        from curobo.types.base import TensorDeviceType

        version = importlib.metadata.version("nvidia_curobo")
        if version != "0.7.8":
            raise RuntimeError(f"expected nvidia_curobo 0.7.8, found {version}")
        if not torch.cuda.is_available():
            raise RuntimeError(
                "cuRobo requires a working CUDA-enabled PyTorch installation"
            )
        self.torch = torch
        self.tensor = TensorDeviceType()
        self.model = model
        self.config = config
        self._temporary = TemporaryDirectory(prefix="cleany-curobo-")
        self._urdf = str(Path(self._temporary.name) / "cleany.urdf")
        Path(self._urdf).write_text(model.xml, encoding="utf-8")
        self._ik = {}
        self._motion = {}
        self._ik_locks = {}
        self._world_keys = {}
        self._ik_lock_updates = {}
        self._fk_model = None
        self._validator = None
        self.timings = {}
        self.last_plan_status = "not_run"
        self._owns_resources = resources is None
        self.resources = resources if resources is not None else CuRoboResources()

    def close(self) -> None:
        self._ik.clear()
        self._motion.clear()
        self._ik_locks.clear()
        self._world_keys.clear()
        self._ik_lock_updates.clear()
        getattr(self, '_batch_ik_caches', {}).clear()
        getattr(self, '_batch_motions', {}).clear()
        self._fk_model = None
        self._validator = None
        if self._owns_resources:
            self.resources = CuRoboResources()
        self._temporary.cleanup()

    def _solver(self, arm: str, tip: str, state: dict, world: dict):
        from curobo.geom.types import WorldConfig
        from curobo.geom.sdf.world import CollisionCheckerType
        from curobo.rollout.cost.pose_cost import PoseCostMetric
        from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

        key = (arm, tip)
        if key not in self._ik:
            robot = self.model.robot_config(self._urdf, arm, state, tip)
            solver = IKSolver(
                IKSolverConfig.load_from_robot_config(
                    robot,
                    (WorldConfig.from_dict(world).get_collision_check_world()
                     if self.resources.world_checker is None else None),
                    tensor_args=self.tensor,
                    num_seeds=self.config.num_seeds,
                    position_threshold=self.config.ik_position_threshold_m,
                    # 0.7.8 solve_batch(newton_iters=...) overrides OUTER
                    # graph replays. grad_iters configures the total budget.
                    grad_iters=getattr(self, 'ik_iterations', None),
                    rotation_threshold=10.0,
                    # Outer covers overlap at mechanically close links. IK
                    # checks world obstacles; the adapter owns full-geometry
                    # self-collision acceptance. cuRobo 0.7.8 couples these two
                    # self-collision flags, so disable both explicitly.
                    self_collision_check=False,
                    self_collision_opt=False,
                    collision_checker_type=CollisionCheckerType.MESH,
                    collision_cache={"obb": 128, "mesh": 128},
                    world_coll_checker=self.resources.world_checker,
                    use_cuda_graph=self.config.use_cuda_graph,
                )
            )
            # Preserve Cleany's 5-DOF position-only IK contract. The adapter
            # scores the actual TCP axes and validates collision separately.
            solver.update_pose_cost_metric(
                PoseCostMetric(
                    reach_partial_pose=True,
                    reach_vec_weight=self.tensor.to_device(
                        [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
                    ),
                )
            )
            self._ik[key] = solver
            self._ik_locks[key] = dict(robot["kinematics"]["lock_joints"])
            if self.resources.world_checker is None:
                self.resources.world_checker = solver.world_coll_checker
                self.resources.world_key = repr(world)
        solver = self._ik[key]
        locks = {n: v for n, v in state.items() if n not in ARM_JOINT_NAMES[arm]}
        if locks != self._ik_locks[key]:
            self._update_ik_locked_joints(solver, arm, tip, state, locks, self._ik_locks[key])
            self._ik_locks[key] = locks
        # A single revision belongs to the shared checker. Per-solver keys
        # would incorrectly skip an update after another solver changed it.
        if self.resources.world_key != repr(world):
            solver.world_coll_checker.clear_cache()
            solver.update_world(
                WorldConfig.from_dict(world).get_collision_check_world()
            )
            self.resources.world_key = repr(world)
        return self._ik[key]

    def _update_ik_locked_joints(self, solver, arm, tip, state, locks, previous):
        from .curobo_locked_joints import LockedJointUpdater

        cache = self._ik_lock_updates
        if id(solver) not in cache:
            try:
                cache[id(solver)] = LockedJointUpdater(self.model.xml, solver.kinematics, previous)
            except (ValueError, KeyError, AttributeError):
                # Unsupported layouts retain the full, previously used update.
                cache[id(solver)] = None
        updater = cache[id(solver)]
        if updater is not None:
            updater.update(locks, state)
            self.timings['locked_joint_fast_updates'] = self.timings.get('locked_joint_fast_updates', 0)+1
            return
        from curobo.types.robot import RobotConfig

        robot = self.model.robot_config(self._urdf, arm, state, tip)
        links = list(robot['kinematics']['collision_link_names'])
        robot['kinematics']['self_collision_ignore'] = {n: links for n in links}
        update = RobotConfig.from_dict(robot, self.tensor)
        solver.kinematics.update_kinematics_config(update.kinematics.kinematics_config)
        self.timings['locked_joint_full_updates'] = self.timings.get('locked_joint_full_updates', 0)+1

    def solve(
        self, arm: str, tip: str, position, seeds, state: dict, world=None
    ) -> list[JointSolution]:
        """Propose IK states; the adapter must validate full robot geometry."""
        from curobo.types.math import Pose

        self.model.validate_state(state)
        solver = self._solver(arm, tip, state, world or {})
        names = tuple(solver.joint_names)
        if set(names) != set(ARM_JOINT_NAMES[arm]):
            raise ValueError("cuRobo IK joint ownership differs from MoveIt")
        ordered = [
            [dict(zip(seed.names, seed.positions))[name] for name in names]
            for seed in seeds
        ]
        if not ordered:
            raise ValueError("at least one IK seed is required")
        ordered = ordered[: self.config.num_seeds]
        result = solver.solve_single(
            Pose(
                position=self.tensor.to_device([position]),
                quaternion=self.tensor.to_device([[1.0, 0.0, 0.0, 0.0]]),
            ),
            seed_config=self.tensor.to_device(ordered).reshape(1, -1, len(names)),
            retract_config=self.tensor.to_device([ordered[0]]),
            return_seeds=self.config.num_seeds,
        )
        self.torch.cuda.synchronize()
        solutions = result.solution.reshape(-1, len(names)).detach().cpu().numpy()
        success = result.success.reshape(-1).detach().cpu().numpy()
        order = [names.index(name) for name in ARM_JOINT_NAMES[arm]]
        return [
            JointSolution(ARM_JOINT_NAMES[arm], tuple(float(v) for v in q[order]))
            for q, valid in zip(solutions, success)
            if valid and np.isfinite(q).all()
        ]

    def fk(self, arm: str, tip: str, solutions, state: dict) -> list[PoseValue]:
        if not solutions:
            return []
        self.model.validate_state(state)
        model = self._full_fk(state)
        result = model.get_state(
            self._full_positions(
                solutions[0].names, [q.positions for q in solutions], state
            )
        )
        index = model.link_names.index(tip)
        xyz = result.links_position[:, index].detach().cpu().numpy()
        wxyz = result.links_quaternion[:, index].detach().cpu().numpy()
        return [
            PoseValue(
                tuple(float(v) for v in p), tuple(float(v) for v in q[[1, 2, 3, 0]])
            )
            for p, q in zip(xyz, wxyz)
        ]

    def _full_fk(self, state):
        if self.resources.fk_model is not None:
            self._fk_model = self.resources.fk_model
        if self._fk_model is None:
            from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
            from curobo.types.robot import RobotConfig

            robot = self.model.robot_config(self._urdf, "left", state, "left_grasp_tcp")
            tips = [
                f"{arm}_{tip}"
                for arm in ARM_JOINT_NAMES
                for tip in ("grasp_tcp", "pregrasp_aim_tip", "gripper_frame")
            ]
            robot["kinematics"].update(
                lock_joints={},
                link_names=list(dict.fromkeys(self.model.collision_links + tips)),
                collision_spheres=None,
                collision_link_names=None,
                self_collision_ignore={},
                self_collision_buffer={},
            )
            self._fk_model = CudaRobotModel(
                RobotConfig.from_dict(robot, self.tensor).kinematics
            )
            self.resources.fk_model = self._fk_model
        return self._fk_model

    def _full_positions(self, names, positions, state):
        self.model.validate_state(state)
        model = self._full_fk(state)
        if len(set(names)) != len(names) or not set(names).issubset(model.joint_names):
            raise ValueError("invalid GPU validation joint names")
        q = self.tensor.to_device(positions)
        if q.ndim != 2 or q.shape[1] != len(names) or q.shape[0] == 0:
            raise ValueError("invalid GPU validation shape")
        full = self.tensor.to_device([[state[n] for n in model.joint_names]]).repeat(
            q.shape[0], 1
        )
        full[:, [model.joint_names.index(n) for n in names]] = q
        return full

    def valid_states(self, names, positions, state, world, acm):
        """One GPU batch for bounds, complete robot envelopes and ACM pairs."""
        from types import SimpleNamespace
        from cleany_skill_executor.curobo_collision import GpuCollisionValidator

        began = time.monotonic()
        full = self._full_positions(names, positions, state)
        model = self._full_fk(state)
        limits = self.tensor.to_device(
            [self.model.limits[n] for n in model.joint_names]
        )
        bounds = (
            self.torch.isfinite(full).all(dim=1)
            & (full >= limits[:, 0]).all(dim=1)
            & (full <= limits[:, 1]).all(dim=1)
        )
        result = model.get_state(full)
        bounds &= self.torch.isfinite(result.links_position).all(dim=(1, 2))
        bounds &= self.torch.isfinite(result.links_quaternion).all(dim=(1, 2))
        if self._validator is None:
            self._validator = GpuCollisionValidator(
                self.model, self.tensor, self.config.collision_margin_m
            )
        self._validator.prepare(world, acm)
        indices = [model.link_names.index(n) for n in self.model.collision_links]
        fk = SimpleNamespace(
            links_position=result.links_position[:, indices],
            links_quaternion=result.links_quaternion[:, indices],
        )
        valid = self._validator.valid(fk) & bounds
        output = valid.detach().cpu().numpy()
        self.timings["gpu_validation_sec"] = time.monotonic() - began
        return output

    def plan(
        self,
        arm: str,
        start: JointSolution,
        goal: JointSolution,
        state: dict,
        world: dict,
        velocity: float,
        acceleration: float,
        timeout: float,
        attempts: int,
    ):
        from curobo.geom.sdf.world import CollisionCheckerType
        from curobo.geom.types import WorldConfig
        from curobo.types.robot import JointState
        from curobo.wrap.reacher.motion_gen import (
            MotionGen,
            MotionGenConfig,
            MotionGenPlanConfig,
        )

        self.model.validate_state(state)
        if not 0 < velocity <= 1 or not 0 < acceleration <= 1:
            raise ValueError("trajectory scaling must be in (0, 1]")
        began = time.monotonic()
        world_model = WorldConfig.from_dict(world).get_collision_check_world()
        locks = {
            name: value
            for name, value in state.items()
            if name not in ARM_JOINT_NAMES[arm]
        }
        if arm not in self._motion:
            robot = self.model.robot_config(self._urdf, arm, state, f"{arm}_grasp_tcp")
            motion = MotionGen(
                MotionGenConfig.load_from_robot_config(
                    deepcopy(robot),
                    world_model,
                    tensor_args=self.tensor,
                    collision_checker_type=CollisionCheckerType.MESH,
                    self_collision_check=False, self_collision_opt=False,
                    collision_cache={"obb": 128, "mesh": 128},
                    interpolation_dt=self.config.interpolation_dt,
                    num_ik_seeds=self.config.num_seeds,
                    num_trajopt_seeds=4,
                    cspace_threshold=self.config.joint_tolerance_rad,
                    use_cuda_graph=self.config.use_cuda_graph,
                )
            )
            self._motion[arm] = (motion, robot, locks)
        motion, robot, previous_locks = self._motion[arm]
        if locks != previous_locks:
            # update_locked_joints copies kinematics tensors only. The existing
            # collision-checker's SRDF pair table remains in use. Avoid building
            # another unused pair table while deriving the new fixed transforms.
            update_config = deepcopy(robot)
            links = list(self.model.spheres)
            update_config["kinematics"]["self_collision_ignore"] = {
                link: links for link in links
            }
            motion.update_locked_joints(locks, update_config)
            self._motion[arm] = (motion, robot, locks)
        # v0.7.8 does not disable stale mesh entries when a later scene contains
        # zero meshes. Clear enable masks before loading every complete snapshot.
        if self._world_keys.get(arm) != repr(world):
            motion.clear_world_cache()
            motion.update_world(world_model)
            self._world_keys[arm] = repr(world)
        names = tuple(motion.kinematics.joint_names)

        def joint_state(solution):
            values = dict(zip(solution.names, solution.positions))
            return JointState.from_position(
                self.tensor.to_device([[values[n] for n in names]]),
                joint_names=list(names),
            )

        self.timings["plan_prepare_sec"] = time.monotonic() - began
        self.timings["actual_js_trajectory_seeds"] = getattr(motion.js_trajopt_solver, "num_seeds", None)
        self.timings["requested_planning_attempts"] = attempts
        self.timings["joint_plan_graph"] = getattr(self, 'joint_plan_graph', False)
        solve_began = time.monotonic()
        result = motion.plan_single_js(
            joint_state(start),
            joint_state(goal),
            MotionGenPlanConfig(
                enable_graph=getattr(self, 'joint_plan_graph', False),
                max_attempts=attempts,
                timeout=timeout,
                time_dilation_factor=min(velocity, acceleration**0.5),
            ),
        )
        self.torch.cuda.synchronize()
        self.timings["gpu_plan_sec"] = time.monotonic() - began
        self.timings["motiongen_call_sec"] = time.monotonic() - solve_began
        for key in ("solve_time", "graph_time", "trajopt_time", "finetune_time"):
            self.timings[f"curobo_{key}_sec"] = float(getattr(result, key, 0.0))
        self.last_plan_status = str(result.status)
        if not bool(result.success.item()):
            return None
        trajectory = result.get_interpolated_plan()
        return names, trajectory.position.detach().cpu().numpy()
