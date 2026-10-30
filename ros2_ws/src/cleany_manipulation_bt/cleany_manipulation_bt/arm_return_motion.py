"""Joint connections and synchronized empty-arm return with explicit collision policy."""
from collections import deque
from collections.abc import Callable, Iterator
import json
import math
from pathlib import Path
import time

import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution, PlannedPath
from cleany_skill_executor.core.joint_path import executable_joint_path
from cleany_skill_executor.trajectory_io import path_to_message


def load_return_waypoints(path: str) -> dict[str, tuple[tuple[float, ...], ...]]:
    """Read two fixed poses per arm in ARM_JOINT_NAMES order, in radians."""
    raw = json.loads(Path(path).expanduser().read_text())
    if (not isinstance(raw, dict) or raw.get('frame') != 'base_link'
            or not isinstance(raw.get('arms'), dict)
            or set(raw['arms']) != set(ARM_JOINT_NAMES)
            or raw.get('joint_names') != {arm: list(names) for arm, names in ARM_JOINT_NAMES.items()}):
        raise ValueError('Return waypoints require both arms and explicit base_link joint order')
    result = {}
    for arm, names in ARM_JOINT_NAMES.items():
        poses = raw['arms'][arm]
        if (not isinstance(poses, list) or len(poses) != 2
                or any(not isinstance(q, list) or len(q) != len(names)
                       or any(type(v) not in (int, float) or not math.isfinite(v) for v in q)
                       for q in poses)):
            raise ValueError(f'{arm}: return requires two finite five-joint poses')
        result[arm] = tuple(tuple(float(v) for v in q) for q in poses)
    return result


def _return_candidates(path: PlannedPath) -> Iterator[tuple[tuple[float, ...], ...]]:
    """Try removing bends, then move retained bends toward their local chord."""
    points = path.points
    pending = deque([(0, len(points)-1)])
    while pending:
        first, last = pending.popleft()
        if last <= first+1:
            continue
        yield (*points[:first+1], *points[last:])
        middle = (first+last)//2
        pending.extend(((first, middle), (middle, last)))
    bends = []
    for index in range(1, len(points)-1):
        previous, current, following = (np.asarray(q) for q in points[index-1:index+2])
        chord = following-previous
        squared = float(chord@chord)
        fraction = float(np.clip((current-previous)@chord/squared, 0., 1.)) if squared > 1e-18 else .5
        projection = previous+fraction*chord
        displacement = current-projection
        bends.append((float(np.linalg.norm(displacement)), index, current, projection, displacement))
    for _, index, current, projection, displacement in sorted(bends, key=lambda item: item[0], reverse=True):
        axes = np.argsort(np.abs(displacement))[::-1]
        for remaining in (.5, .25, .75):
            moved = tuple(float(q) for q in projection+remaining*displacement)
            yield (*points[:index], moved, *points[index+1:])
            # A large shoulder swing can be reduced while retaining the elbow
            # clearance that makes the complete waypoint necessary.
            adjusted = current.copy()
            adjusted[axes[0]] -= (1.-remaining)*displacement[axes[0]]
            yield (*points[:index], tuple(float(q) for q in adjusted), *points[index+1:])
        for axis in axes[1:]:
            adjusted = current.copy()
            adjusted[axis] -= .5*displacement[axis]
            yield (*points[:index], tuple(float(q) for q in adjusted), *points[index+1:])


def shorten_return_path(path: PlannedPath, validate: Callable[[PlannedPath], PlannedPath | None],
                        velocity: tuple[float, ...], acceleration: tuple[float, ...],
                        attempts: int, *, known_invalid: tuple[PlannedPath, ...] = (),
                        deadline_monotonic: float | None = None,
                        monotonic: Callable[[], float] = time.monotonic,
                        report: dict | None = None) -> PlannedPath:
    """Keep fully validated improvements within a bounded local search."""
    if isinstance(attempts, bool) or not isinstance(attempts, int) or not 0 <= attempts <= 32:
        raise ValueError('Return optimization attempts must be an integer in 0..32')
    if deadline_monotonic is not None and not math.isfinite(deadline_monotonic):
        raise ValueError('Return optimization deadline must be finite')
    def cost(candidate):
        cubic = executable_joint_path(list(candidate.points)).retimed(velocity, acceleration)
        length = float(np.linalg.norm(np.diff(cubic.positions, axis=0), axis=1).sum())
        return cubic.minimum_duration(velocity, acceleration), length

    def improves(proposed, current):
        return (proposed[0] < current[0]-1e-6 or
                (abs(proposed[0]-current[0]) <= 1e-6 and proposed[1] < current[1]-1e-6))

    def expired():
        return deadline_monotonic is not None and monotonic() >= deadline_monotonic

    best, best_cost = path, cost(path)
    initial_cost = best_cost
    # Rejections can suppress redundant work, but successful scene checks are
    # never cached across the subsequent path transformations.
    seen = {candidate.points for candidate in known_invalid}
    checked = accepted_count = 0
    while checked < attempts and not expired():
        improved = False
        for points in _return_candidates(best):
            if checked >= attempts or expired():
                break
            if points in seen:
                continue
            seen.add(points)
            candidate = PlannedPath(path.joint_names, points)
            candidate_cost = cost(candidate)
            if not improves(candidate_cost, best_cost) or expired():
                continue
            checked += 1
            # Check the transformed cubic including both neighboring junctions;
            # checking only the new straight edge would miss spline changes.
            accepted = validate(candidate)
            if accepted is not None:
                if (accepted.joint_names != path.joint_names or len(accepted.points) < 2
                        or accepted.points[0] != path.points[0] or accepted.points[-1] != path.points[-1]):
                    raise RuntimeError('Return optimization changed joint ownership or endpoints')
                accepted_cost = candidate_cost if accepted.points == candidate.points else cost(accepted)
                if improves(accepted_cost, best_cost):
                    best, best_cost = accepted, accepted_cost
                    accepted_count += 1
                    improved = True
                    break
        if not improved:
            break
    if report is not None:
        report.update(candidate_checks=checked, accepted_improvements=accepted_count,
                      budget_exhausted=expired(),
                      initial_arm_duration_sec=initial_cost[0], final_arm_duration_sec=best_cost[0],
                      initial_joint_length_rad=initial_cost[1], final_joint_length_rad=best_cost[1])
    return best


def move_arm_return(operations, arm: str, home: tuple[float, ...], gripper_closed: float,
                    *, planned_event: str = 'manual_return_planned'):
    started = time.monotonic()
    timings, optimization = {}, {}
    prepare = getattr(operations.real, 'prepare_return_phase', None)
    if prepare is not None:
        prepare(arm)

    def measured_phase(name: str, action: Callable[[], PlannedPath | None]) -> PlannedPath | None:
        began = time.monotonic()
        try:
            return action()
        finally:
            elapsed = time.monotonic() - began
            timings[name + '_wall_sec'] = elapsed
            operations.event('arm_return_planning_phase', arm=arm, phase=name, wall_sec=elapsed)

    fixed = getattr(operations, 'return_waypoints', None)
    check_collisions = (operations.value('bt_arm_return_collision_check_enabled')
                        if fixed is not None else True)
    if type(check_collisions) is not bool:
        raise ValueError('bt_arm_return_collision_check_enabled must be boolean')
    capture_barrier = getattr(operations.real, 'capture_scene_barrier', None)
    scene_barrier = (capture_barrier(arm)
                     if check_collisions and capture_barrier is not None else None)
    state = operations.real.monitor()
    names = ARM_JOINT_NAMES[arm]
    positions = dict(zip(state.name, state.position, strict=True))
    start = tuple(float(positions[name]) for name in names)
    gripper_name = f'{arm}_gripper_joint'
    operations.motion.set_current_state(state)
    # When enabled, collision checks use the open jaw for planning and the
    # combined arm and closing-jaw cubic at physical dispatch.
    operations.motion.set_gripper_position(arm, positions[gripper_name])
    validate = lambda path: operations.motion._validated_cubic_path(arm, path)
    if fixed is not None:
        proposed = PlannedPath(names, (start, *fixed[arm], home))
        # Hardware dispatch always checks the retimed command's joint limits.
        path = (measured_phase('fixed_validation', lambda: validate(proposed))
                if check_collisions else proposed)
        if path is None:
            raise RuntimeError('Fixed arm return failed joint-limit or collision validation; RRT is disabled')
        route, input_points = 'fixed_waypoints', len(proposed.points)
    else:
        direct = PlannedPath(names, (start, home))
        path = measured_phase('direct_validation', lambda: validate(direct))
        route, input_points = 'direct', 2
    if path is None:
        planner_id = operations.value('bt_arm_return_planner_id')
        planning_scaling = operations.value('bt_arm_return_planning_trajectory_scaling')
        path = measured_phase('rrt', lambda: operations.motion.plan(
            arm, JointSolution(names, home), None, planner_id=planner_id,
            trajectory_scaling=planning_scaling))
        if path is None:
            raise RuntimeError('No collision-free arm return path')
        route, input_points = 'rrt', len(path.points)
        limits = operations.real.verify(arm)
        velocity = tuple(min(operations.real.value('hardware_velocity_rad_s'), limits[n].velocity * .8)
                         for n in names)
        acceleration = tuple(min(operations.real.value('hardware_acceleration_rad_s2'),
                                 limits[n].acceleration * .8) for n in names)
        original = path
        budget = float(operations.value('bt_arm_return_optimization_budget_sec'))
        if not math.isfinite(budget) or budget <= 0:
            raise ValueError('Return optimization budget must be finite and positive')
        def optimize():
            deadline = time.monotonic()+budget
            def validate_candidate(candidate):
                return operations.motion._validated_cubic_path(
                    arm, candidate, deadline_monotonic=deadline)
            return shorten_return_path(original, validate_candidate, velocity, acceleration,
                int(operations.value('bt_arm_return_shortcut_attempts')),
                known_invalid=(direct,), deadline_monotonic=deadline, report=optimization)
        path = measured_phase('shortcuts', optimize)
    operations.check_execution()
    operations.event(planned_event, arm=arm, route=route,
                     input_waypoints=input_points, return_waypoints=len(path.points),
                     fixed_waypoints_joint_rad=list(fixed[arm]) if fixed is not None else [],
                     gripper_goal_rad=gripper_closed, collision_checks_enabled=check_collisions,
                     optimization=optimization,
                     planning_wall_sec=time.monotonic() - started, **timings)
    jaw = JointTrajectory(joint_names=[gripper_name], points=[
        JointTrajectoryPoint(positions=[positions[gripper_name]]),
        JointTrajectoryPoint(positions=[gripper_closed])])
    return operations.real.execute_arm_gripper(arm, path_to_message(path), jaw,
        **({'check_collisions': False} if not check_collisions else {}),
        **({'scene_barrier': scene_barrier} if scene_barrier is not None else {}))
