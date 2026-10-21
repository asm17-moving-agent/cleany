"""GPU reachability and conservative GPU collision validation of the full scene."""

from __future__ import annotations

import threading
import math
from contextlib import contextmanager

from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rosidl_runtime_py.convert import message_to_ordereddict

from cleany_skill_executor.core.alignment import (
    axis_errors_degrees,
    sampled_indices,
    tcp_axes,
)
from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES,
    InfrastructureError,
    JointSolution,
    PlannedPath,
    TrajectoryAlignment,
)
from cleany_skill_executor.curobo_model import CuRoboConfig, validation_states
from cleany_skill_executor.curobo_scene import world_from_scene
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter


class CuRoboGraspAdapter(MoveItGraspAdapter):
    def __init__(
        self,
        node,
        config: MoveItAdapterConfig,
        *,
        gpu_config: CuRoboConfig = CuRoboConfig(),
        engine=None,
        scene_client=None,
        **kwargs,
    ) -> None:
        super().__init__(node, config, **kwargs)
        if config.base_frame != "base_link":
            raise ValueError("cuRobo model currently requires base_link")
        self._gpu_config = gpu_config
        self._cancel_event = threading.Event()
        self._deadline = math.inf
        self._scene_client = scene_client or node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        if engine is None:
            from pathlib import Path
            from ament_index_python.packages import get_package_share_directory
            import xacro
            from cleany_skill_executor.curobo_engine import CuRoboEngine
            from cleany_skill_executor.curobo_model import RobotModel

            description = Path(get_package_share_directory("cleany_description"))
            moveit = Path(get_package_share_directory("cleany_moveit_config"))
            urdf = xacro.process_file(
                str(description / "urdf/cleany.urdf.xacro"),
                mappings={"include_head_camera": "false"},
            ).toxml()
            model = RobotModel(
                urdf, (moveit / "config/cleany.srdf").read_text(), gpu_config
            )
            engine = CuRoboEngine(model, gpu_config)
        self._engine = engine
        self._active_scene = None
        self.last_timings = {}

    @property
    def planner_id(self) -> str:
        return "curobo_v0.7.8_gpu_validated"

    def set_current_state(self, state) -> None:
        super().set_current_state(state)
        self._cancel_event.clear()

    def cancel_active(self) -> None:
        self._cancel_event.set()
        super().cancel_active()

    def _check_cancel(self) -> None:
        if self._cancel_event.is_set() or self._monotonic() >= self._deadline:
            raise InterruptedError("cuRobo computation canceled; result discarded")

    def set_deadline(self, deadline: float) -> None:
        super().set_deadline(deadline)

    def _state(self) -> dict:
        joint = self._merged_state(None).joint_state
        return dict(zip(joint.name, joint.position, strict=True))

    def _compute(self, function, *args):
        self._check_cancel()
        try:
            result = function(*args)
        except Exception as error:
            raise InfrastructureError(f"cuRobo computation failed: {error}") from error
        self._check_cancel()
        return result

    def solve_position_ik(self, arm, position, seed):
        with self._scene_operation():
            return self._solve_position_gpu(arm, position, seed)

    def _solve_position_gpu(self, arm, position, seed):
        solutions = self._compute(
            self._engine.solve,
            arm,
            f"{arm}_grasp_tcp",
            position,
            self._seed_bases(arm, seed),
            self._state(),
            world_from_scene(self._active_scene, optimization=True),
        )
        valid = self._valid_batch(arm, solutions)
        return next((q for q, ok in zip(solutions, valid) if ok), None)

    def solve_aligned_ik(self, arm, target, seed):
        with self._scene_operation():
            return self._solve_aligned_gpu(arm, target, seed)

    def _solve_aligned_gpu(self, arm, target, seed):
        state = self._state()
        world = world_from_scene(self._active_scene, optimization=True)
        seeds = self._seed_bases(arm, seed)
        direct = self._compute(
            self._engine.solve,
            arm,
            f"{arm}_grasp_tcp",
            target.position,
            seeds,
            state,
            world,
        )
        evaluated = self._score(arm, target, direct, state)
        chosen = self._choose_valid(arm, evaluated)
        if chosen is not None:
            return chosen
        approach, _ = tcp_axes(target.orientation_xyzw)
        aim_position = tuple(
            p + self._config.aim_tip_offset_m * a
            for p, a in zip(target.position, approach)
        )
        aims = self._compute(
            self._engine.solve,
            arm,
            f"{arm}_pregrasp_aim_tip",
            aim_position,
            seeds,
            state,
            world,
        )
        if aims:
            projected = self._compute(
                self._engine.solve,
                arm,
                f"{arm}_grasp_tcp",
                target.position,
                aims[: self._gpu_config.num_seeds],
                state,
                world,
            )
            evaluated += self._score(arm, target, aims + projected, state)
        chosen = self._choose_valid(arm, evaluated)
        if chosen is not None:
            return chosen
        # Return an out-of-tolerance result for the existing selector's error
        # classification, but never return a feasible, collision-invalid one.
        rejected = [item for item in evaluated if not self._feasible_alignment(item)]
        return self._best_alignment(rejected) if rejected else None

    def _score(self, arm, target, solutions, state):
        variants = {}
        for solution in solutions:
            for variant in self._wrist_variants(solution):
                variants[tuple(round(v, 7) for v in variant.positions)] = variant
        joints = list(variants.values())
        poses = self._compute(self._engine.fk, arm, f"{arm}_grasp_tcp", joints, state)
        return [
            self._alignment(q, target, pose)
            for q, pose in zip(joints, poses, strict=True)
        ]

    def _wrist_variants(self, solution):
        name = solution.names[-1]
        lower, upper = self._engine.model.limits[name]
        variants = []
        for offset in self._config.ik_seed_wrist_offsets_degrees:
            positions = list(solution.positions)
            positions[-1] = min(upper, max(lower, positions[-1] + math.radians(offset)))
            candidate = JointSolution(solution.names, tuple(positions))
            if candidate not in variants:
                variants.append(candidate)
        return tuple(variants)

    def _choose_valid(self, arm, evaluated):
        feasible = [item for item in evaluated if self._feasible_alignment(item)]
        valid = self._valid_batch(arm, [item.solution for item in feasible])
        accepted = [item for item, ok in zip(feasible, valid) if ok]
        return self._best_alignment(accepted) if accepted else None

    @contextmanager
    def _scene_operation(self):
        self._check_cancel()
        if self._active_scene is not None:
            yield
            return
        scene = self._snapshot()
        # Reject unsupported data before launching any GPU optimization.
        try:
            world_from_scene(scene)
        except ValueError as error:
            raise InfrastructureError(
                f"cuRobo planning-scene error: {error}"
            ) from error
        self._active_scene = scene
        try:
            yield
            self._check_cancel()
            if self._scene_key(scene) != self._scene_key(self._snapshot()):
                raise InfrastructureError(
                    "planning scene changed during cuRobo validation"
                )
        finally:
            self._active_scene = None

    def _valid_batch(self, arm, solutions):
        if not solutions:
            return []
        names = ARM_JOINT_NAMES[arm]
        if any(q.names != names for q in solutions):
            raise InfrastructureError("GPU validation joint ownership mismatch")
        scene = self._active_scene
        valid = self._compute(
            self._engine.valid_states,
            names,
            [q.positions for q in solutions],
            self._state(),
            world_from_scene(scene),
            scene.allowed_collision_matrix,
        )
        if len(valid) != len(solutions):
            raise InfrastructureError("incomplete GPU collision result")
        if self._gpu_config.moveit_shadow_validation:
            for q, ok in zip(solutions, valid):
                if ok and not MoveItGraspAdapter.state_is_valid(self, arm, q):
                    raise InfrastructureError("GPU/MoveIt collision disagreement")
        return valid

    def state_is_valid(self, arm, solution):
        with self._scene_operation():
            return bool(self._valid_batch(arm, [solution])[0])

    def forward_kinematics(self, arm, solution):
        return self._compute(
            self._engine.fk, arm, f"{arm}_grasp_tcp", [solution], self._state()
        )[0]

    def _snapshot(self, *, include_octomap: bool = True):
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        if include_octomap:
            request.components.components |= PlanningSceneComponents.OCTOMAP
        return self._call(
            self._scene_client, request, self._config.state_validity_timeout_sec
        ).scene

    @staticmethod
    def _scene_key(scene):
        def geometry(value):
            if isinstance(value, dict):
                return {
                    key: geometry({k: v for k, v in item.items() if k != "stamp"})
                    if key == "header" and isinstance(item, dict)
                    else geometry(item)
                    for key, item in value.items()
                }
            if isinstance(value, list):
                return [geometry(item) for item in value]
            return value

        # MoveIt refreshes OctoMap headers even in an empty, unchanged world.
        # Compare semantic geometry/ACM rather than serialized CDR padding or
        # acquisition timestamps. Object frame IDs and poses remain significant.
        return (
            geometry(message_to_ordereddict(scene.world)),
            message_to_ordereddict(scene.allowed_collision_matrix),
            [
                geometry(message_to_ordereddict(obj))
                for obj in scene.robot_state.attached_collision_objects
            ],
        )

    def plan(self, arm, goal, start):
        began = self._monotonic()
        with self._scene_operation():
            path = self._plan_gpu(arm, goal, start)
        self.last_timings = dict(getattr(self._engine, "timings", {}))
        self.last_timings["total_sec"] = self._monotonic() - began
        return path

    def _plan_gpu(self, arm, goal, start):
        self._check_cancel()
        scene = self._active_scene
        try:
            world = world_from_scene(scene, self._config.base_frame, optimization=True)
        except ValueError as error:
            raise InfrastructureError(
                f"cuRobo planning-scene error: {error}"
            ) from error
        state = self._state()
        start = start or JointSolution(
            ARM_JOINT_NAMES[arm], tuple(state[n] for n in ARM_JOINT_NAMES[arm])
        )
        result = self._compute(
            self._engine.plan,
            arm,
            start,
            goal,
            state,
            world,
            self._config.velocity_scaling,
            self._config.acceleration_scaling,
            self._config.planning_timeout_sec,
            self._config.planning_attempts,
        )
        if result is None:
            return None
        names, points = result
        if set(names) != set(ARM_JOINT_NAMES[arm]) or len(names) != 5:
            raise InfrastructureError("cuRobo trajectory changes joint ownership")
        try:
            checks = validation_states(
                points,
                self._gpu_config.validation_step_rad,
                self._gpu_config.maximum_validation_states,
            )
        except ValueError as error:
            raise InfrastructureError(str(error)) from error
        for endpoint, expected in ((checks[0], start), (checks[-1], goal)):
            target = dict(zip(expected.names, expected.positions))
            if (
                max(abs(value - target[name]) for name, value in zip(names, endpoint))
                > self._gpu_config.joint_tolerance_rad
            ):
                raise InfrastructureError(
                    "cuRobo trajectory endpoint differs from requested joints"
                )
        order = [names.index(n) for n in ARM_JOINT_NAMES[arm]]
        solutions = [
            JointSolution(ARM_JOINT_NAMES[arm], tuple(q[i] for i in order))
            for q in checks
        ]
        if not all(self._valid_batch(arm, solutions)):
            return None
        return PlannedPath(
            tuple(names), tuple(tuple(float(v) for v in p) for p in points)
        )

    def trajectory_alignment(self, arm, path, orientation_xyzw, maximum_waypoints):
        indices = sampled_indices(len(path.points), maximum_waypoints)
        if not indices:
            return None
        solutions = [JointSolution(path.joint_names, path.points[i]) for i in indices]
        poses = self._compute(
            self._engine.fk, arm, f"{arm}_grasp_tcp", solutions, self._state()
        )
        errors = [
            axis_errors_degrees(orientation_xyzw, pose.orientation_xyzw)
            for pose in poses
        ]
        return TrajectoryAlignment(
            max(e[0] for e in errors), max(e[1] for e in errors), len(indices)
        )
