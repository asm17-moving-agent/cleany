"""Seeded Cartesian path generation using normal MoveIt IK/FK/collision services.

No controller command is sent here. The caller must validate Cartesian geometry
and apply its Cartesian speed/acceleration slowdown before execution.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Callable

from moveit_msgs.msg import MotionPlanRequest, RobotTrajectory
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
import yaml
import numpy as np

from cleany_skill_executor.core.cartesian import CartesianPose, interpolate_orientation
from cleany_skill_executor.core.joint_path import CubicJointPath
from cleany_skill_executor.core.pose_refinement import refine_pose
from cleany_skill_executor.core.urdf_fk import UrdfChain
from cleany_skill_executor.core.grasp_selection import quaternion_axis


@dataclass(frozen=True)
class JointMotionLimit:
    lower: float
    upper: float
    velocity: float
    acceleration: float

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.lower, self.upper, self.velocity, self.acceleration)) or
                self.lower >= self.upper or self.velocity <= 0 or self.acceleration <= 0):
            raise ValueError('invalid joint motion limits')


def load_joint_motion_limits(path: str, fallback_acceleration: float) -> dict[str, JointMotionLimit]:
    if not math.isfinite(fallback_acceleration) or fallback_acceleration <= 0:
        raise ValueError('fallback acceleration must be finite and positive')
    values = yaml.safe_load(Path(path).read_text())['joint_limits']
    limits = {}
    for name, item in values.items():
        if not item.get('has_position_limits') or not item.get('has_velocity_limits'):
            raise ValueError(f'missing finite position/velocity limits: {name}')
        limits[name] = JointMotionLimit(
            float(item['min_position']), float(item['max_position']), float(item['max_velocity']),
            float(item['max_acceleration']) if item.get('has_acceleration_limits') else fallback_acceleration)
    return limits


@dataclass(frozen=True)
class SeededCartesianConfig:
    ik_step_m: float = .01
    validation_step_m: float = .001
    validation_joint_step_rad: float = .005
    ik_timeout_sec: float = .2
    maximum_points: int = 2000
    local_refinement_iterations: int = 0

    def __post_init__(self):
        if (not all(math.isfinite(v) and v > 0 for v in (
                self.ik_step_m, self.validation_step_m, self.validation_joint_step_rad, self.ik_timeout_sec)) or
                self.validation_step_m > self.ik_step_m or self.maximum_points < 2
                or not 0 <= self.local_refinement_iterations <= 100):
            raise ValueError('invalid seeded Cartesian sampling configuration')


class SeededCartesianPlanner:
    def __init__(self, solve_ik: Callable, compute_fk: Callable, check_state: Callable,
                 limits: dict[str, JointMotionLimit], config: SeededCartesianConfig):
        self.solve_ik, self.compute_fk, self.check_state = solve_ik, compute_fk, check_state
        self.limits, self.config = limits, config
        self._description = None

    def set_robot_description(self, description: str) -> None:
        self._description = description

    def plan(self, request: MotionPlanRequest, start: CartesianPose,
             target: CartesianPose) -> tuple[RobotTrajectory, list[CartesianPose]]:
        if (not request.start_state.is_diff or len(request.goal_constraints) != 1 or
                len(request.path_constraints.position_constraints) != 1):
            raise ValueError('seeded planner requires a full feedback diff and one Cartesian corridor')
        full_names = list(request.start_state.joint_state.name)
        full_positions = list(request.start_state.joint_state.position)
        if (len(full_names) != len(full_positions) or len(full_names) != len(set(full_names)) or
                not all(math.isfinite(q) for q in full_positions)):
            raise ValueError('invalid full feedback state')
        current = dict(zip(full_names, full_positions, strict=True))
        goal_constraints = request.goal_constraints[0].joint_constraints
        names = [c.joint_name for c in goal_constraints]
        if not names or len(names) != len(set(names)) or not set(names) <= current.keys():
            raise ValueError('invalid joint endpoint constraints')
        first = tuple(current[name] for name in names)
        last = tuple(c.position for c in goal_constraints)
        constraints = request.path_constraints
        tip = constraints.position_constraints[0].link_name
        frame = constraints.position_constraints[0].header.frame_id
        count = max(2, math.ceil(math.dist(start.position, target.position)/self.config.ik_step_m))
        if count+1 > self.config.maximum_points:
            raise ValueError('IK path exceeds sample budget')
        knots = [first]
        chain = None
        if self.config.local_refinement_iterations:
            if self._description is None:
                raise RuntimeError('Runtime URDF required for local Cartesian refinement')
            chain = UrdfChain(self._description, frame, tip)
            reference = GetPositionFK.Request(robot_state=request.start_state, fk_link_names=[tip])
            reference.header.frame_id = frame
            result = self.compute_fk(reference)
            if result.error_code.val != 1 or len(result.pose_stamped) != 1:
                raise RuntimeError('MoveIt FK unavailable for local model verification')
            p, r = chain.pose(current)
            pose = result.pose_stamped[0].pose
            q = pose.orientation
            expected_r = np.column_stack([quaternion_axis((q.x,q.y,q.z,q.w), axis)
                for axis in ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))])
            if (np.linalg.norm(p - (pose.position.x,pose.position.y,pose.position.z)) > 1e-5
                    or np.linalg.norm(r-expected_r) > 1e-5):
                raise RuntimeError('Runtime URDF FK disagrees with MoveIt')

        def robot_state(values):
            state = deepcopy(request.start_state)
            merged = dict(current)
            merged.update(zip(names, values, strict=True))
            state.joint_state.position = [merged[name] for name in full_names]
            return state  # is_diff and all attachment fields are preserved

        for index in range(1, count):
            fraction = index/count
            seed = tuple(a+fraction*(b-a) for a, b in zip(first, last, strict=True))
            if chain is not None:
                target_p = np.array(start.position) + fraction*(np.array(target.position)-start.position)
                def residual(values):
                    positions = dict(current)
                    positions.update(zip(names, values, strict=True))
                    p, _ = chain.pose(positions)
                    # Keep redundant joints near the endpoint interpolation;
                    # position dominates this weak regularization.
                    return np.concatenate((p-target_p, 1e-4*(np.asarray(values)-seed)))
                values = refine_pose(residual, seed,
                    [(self.limits[n].lower,self.limits[n].upper) for n in names],
                    self.config.local_refinement_iterations)
                if np.linalg.norm(residual(values)[:3]) > self.config.validation_step_m/2:
                    raise RuntimeError(f'Local Cartesian position refinement failed at {index}/{count}')
                knots.append(values)
                continue  # Full MoveIt collision/constraint/FK checks follow below.
            query = GetPositionIK.Request()
            ik = query.ik_request
            ik.group_name, ik.ik_link_name = request.group_name, tip
            ik.robot_state = robot_state(seed)
            ik.avoid_collisions = True
            ik.constraints = deepcopy(constraints)
            ik.pose_stamped.header.frame_id = frame
            p, q = ik.pose_stamped.pose.position, ik.pose_stamped.pose.orientation
            p.x, p.y, p.z = [a+fraction*(b-a) for a, b in zip(start.position, target.position, strict=True)]
            q.x, q.y, q.z, q.w = interpolate_orientation(start.orientation, target.orientation, fraction)
            ik.timeout = Duration(seconds=self.config.ik_timeout_sec).to_msg()
            result = self.solve_ik(query)
            if result.error_code.val != 1:
                raise RuntimeError(f'seeded Cartesian IK failed at {index}/{count}: code={result.error_code.val}')
            solved_names = result.solution.joint_state.name
            solved_positions = result.solution.joint_state.position
            if len(solved_names) != len(set(solved_names)) or len(solved_names) != len(solved_positions):
                raise RuntimeError('seeded IK returned malformed joints')
            solved = dict(zip(solved_names, solved_positions, strict=True))
            if not set(names) <= solved.keys():
                raise RuntimeError('seeded IK omitted arm joints')
            knots.append(tuple(solved[name] for name in names))
        knots.append(last)
        for values in knots:
            for name, value in zip(names, values, strict=True):
                limit = self.limits[name]
                if not math.isfinite(value) or not limit.lower <= value <= limit.upper:
                    raise RuntimeError(f'seeded path violates position limit: {name}={value}')
        for scaling in (request.max_velocity_scaling_factor, request.max_acceleration_scaling_factor):
            if not math.isfinite(scaling) or not 0 < scaling <= 1:
                raise ValueError('invalid motion scaling')
        path = CubicJointPath(knots)
        duration = path.minimum_duration(
            [self.limits[n].velocity*request.max_velocity_scaling_factor for n in names],
            [self.limits[n].acceleration*request.max_acceleration_scaling_factor for n in names])
        samples = path.samples(self.config.validation_joint_step_rad,
                               math.ceil(self.config.ik_step_m/self.config.validation_step_m),
                               self.config.maximum_points)
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = names
        poses = []
        for index, sample in enumerate(samples):
            state = robot_state(sample.positions)
            validity = GetStateValidity.Request(group_name=request.group_name)
            validity.robot_state = state
            validity.constraints = deepcopy(constraints)
            checked = self.check_state(validity)
            if not checked.valid:
                contacts = ', '.join(
                    f'{c.contact_body_1}/{c.contact_body_2}'
                    f' position=({c.position.x:.4f},{c.position.y:.4f},{c.position.z:.4f})'
                    f' frame={c.header.frame_id or frame} depth={c.depth:.6f}m'
                    for c in checked.contacts[:8])
                constraints_failed = [(c.result, c.distance) for c in checked.constraint_result if not c.result]
                raise RuntimeError(f'seeded cubic collision/constraint failure at {index}/{len(samples)}: '
                                   f'{contacts}; constraints={constraints_failed}')
            fk = GetPositionFK.Request(robot_state=state, fk_link_names=[tip])
            fk.header.frame_id = frame
            result = self.compute_fk(fk)
            if result.error_code.val != 1 or len(result.pose_stamped) != 1:
                raise RuntimeError('seeded cubic FK unavailable')
            pose = result.pose_stamped[0].pose
            p, q = pose.position, pose.orientation
            poses.append(CartesianPose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
            point = JointTrajectoryPoint(positions=list(sample.positions),
                                         velocities=[v/duration for v in sample.derivatives])
            point.time_from_start = Duration(seconds=sample.progress*duration).to_msg()
            trajectory.joint_trajectory.points.append(point)
        return trajectory, poses
