"""Replay logged jaw endpoints on Jetson without ROS nodes or motor commands."""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import statistics
import sys
import time

import numpy as np
from rclpy.serialization import serialize_message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cleany_skill_executor.core import hardware_execution as hardware
from cleany_skill_executor.core.recorded_trajectory import CommandPoint, RecordedTrajectory


def command(samples, duration, *, compact):
    result = JointTrajectory(joint_names=['right_gripper_joint'])
    for sample in ([samples[0], samples[-1]] if compact else samples):
        point = JointTrajectoryPoint(positions=list(sample.positions),
                                    velocities=[value/duration for value in sample.derivatives])
        ns = round(sample.progress*duration*1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
        result.points.append(point)
    return result


def recorded(trajectory):
    return RecordedTrajectory(tuple(trajectory.joint_names), tuple(
        CommandPoint(p.time_from_start.sec*10**9+p.time_from_start.nanosec,
                     tuple(p.positions), tuple(p.velocities)) for p in trajectory.points))


def stats(values):
    return dict(median_ms=statistics.median(values)*1000,
                p95_ms=float(np.percentile(values, 95))*1000, repeats=len(values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-joint-path', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=100)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('baseline_joint_path', args.baseline_joint_path)
    baseline = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = baseline
    spec.loader.exec_module(baseline)
    current = hardware.executable_joint_path
    rows = []
    for name, start, end in [('close', .8848652492263254, -.35459122738527254),
                             ('open', .1500884518291034, 1.74532919957)]:
        times = {key: [] for key in ('path_before', 'path_after', 'serialize_before', 'serialize_after')}
        limits = {'right_gripper_joint': hardware.PhysicalJointLimit(
            -2., 2., 1., 2., 2048., 1., 0., 0., 4095.)}
        variants = {}
        try:
            for repeat in range(args.repeats+5):
                # Alternate order to reduce warmup and clock drift bias.
                for updated in ((False, True) if repeat % 2 else (True, False)):
                    label = 'after' if updated else 'before'
                    hardware.executable_joint_path = current if updated else baseline.executable_joint_path
                    began = time.perf_counter()
                    samples, duration = hardware.bounded_path(
                        ('right_gripper_joint',), [(start,), (end,)], limits,
                        velocity=.8, acceleration=1.2, step=.01)
                    path_time = time.perf_counter()-began
                    began = time.perf_counter()
                    trajectory = command(samples, duration, compact=updated)
                    payload = serialize_message(trajectory)
                    serialization_time = time.perf_counter()-began
                    if repeat >= 5:
                        times['path_'+label].append(path_time)
                        times['serialize_'+label].append(serialization_time)
                    variants[label] = (samples, duration, trajectory, len(payload))
        finally:
            hardware.executable_joint_path = current
        before, after = variants['before'], variants['after']
        assert len(before[0]) == len(after[0]) and abs(before[1]-after[1]) < 1e-12
        for a, b in zip(before[0], after[0], strict=True):
            assert (a.progress, a.positions, a.derivatives) == (b.progress, b.positions, b.derivatives)
        original, compact = recorded(before[2]), recorded(after[2])
        position_error = velocity_error = 0.
        for t in np.linspace(0, original.points[-1].time_ns, 1001).astype(int):
            a, b = original.sample_at(int(t)), compact.sample_at(int(t))
            position_error = max(position_error, abs(a.positions[0]-b.positions[0]))
            velocity_error = max(velocity_error, abs(a.velocities[0]-b.velocities[0]))
        assert position_error < 1e-8 and velocity_error < 1e-6
        rows.append(dict(stage=name, start_rad=start, end_rad=end,
                         nominal_duration_sec=after[1], validation_samples=len(after[0]),
                         controller_points_before=len(before[2].points),
                         controller_points_after=len(after[2].points),
                         serialized_bytes_before=before[3], serialized_bytes_after=after[3],
                         maximum_curve_position_difference_rad=position_error,
                         maximum_curve_velocity_difference_rad_s=velocity_error,
                         timing={key: stats(value) for key, value in times.items()}))
    result = dict(environment='Jetson native Python, offline replay',
                  motor_commands_sent=False, ros_nodes_started=False,
                  limits=dict(velocity_rad_s=.8, acceleration_rad_s2=1.2), stages=rows)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
