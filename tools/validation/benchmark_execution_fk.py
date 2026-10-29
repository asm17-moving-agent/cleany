"""Benchmark native MoveIt FK versus verified local FK without motion clients."""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import platform
import statistics
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--samples', type=int, default=100)
    args = parser.parse_args()
    if args.repeats < 1 or args.samples < 2:
        parser.error('repeats must be positive and samples must be at least two')
    xml = args.urdf.read_text()
    bounds = {j.get('name'): (float(j.find('limit').get('lower')),
                             float(j.find('limit').get('upper')))
              for j in ET.fromstring(xml).findall('joint') if j.get('type') == 'revolute'}
    rclpy.init()
    node = Node('execution_fk_benchmark')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    # The adapter creates only planning clients. This tool submits FK services.
    adapter = MoveItGraspAdapter(node, MoveItAdapterConfig(),
                                spin_once=lambda delay: time.sleep(min(delay, .01)))
    adapter.set_robot_description(xml)
    state = JointState(name=list(adapter._state_joint_names),
                       position=[sum(bounds[n])/2 for n in adapter._state_joint_names])
    rng = np.random.default_rng(25)
    remote_calls = 0
    remote = adapter._remote_forward_kinematics

    def counted(arm, solution):
        nonlocal remote_calls
        remote_calls += 1
        return remote(arm, solution)

    adapter._remote_forward_kinematics = counted
    report = dict(environment=dict(platform=platform.platform(), python=platform.python_version()),
                  urdf_sha256=hashlib.sha256(xml.encode()).hexdigest(), repeats=args.repeats,
                  samples_per_arm=args.samples, input='seed=25; random interior joint positions',
                  robot_motion=False, arms={})
    try:
        if not adapter._fk_client.wait_for_service(timeout_sec=30.):
            raise RuntimeError('Isolated MoveIt FK service unavailable')
        for arm, names in ARM_JOINT_NAMES.items():
            queries = [JointSolution(names, tuple(rng.uniform(
                bounds[n][0]+.05, bounds[n][1]-.05) for n in names))
                for _ in range(args.samples)]
            times, counts = {False: [], True: []}, {False: [], True: []}
            outputs = {}
            for repeat in range(args.repeats):
                for local in ((False, True) if repeat % 2 == 0 else (True, False)):
                    adapter._config = replace(adapter._config, use_verified_local_fk=local)
                    adapter.set_current_state(state)
                    remote_calls = 0
                    started = time.monotonic()
                    outputs[local] = [adapter.forward_kinematics(arm, q) for q in queries]
                    times[local].append(time.monotonic()-started)
                    counts[local].append(remote_calls)
            errors = [np.linalg.norm(np.asarray(a.position)-b.position)
                      for a, b in zip(outputs[False], outputs[True], strict=True)]
            orientation_errors = [min(np.linalg.norm(np.asarray(a.orientation_xyzw)-b.orientation_xyzw),
                                      np.linalg.norm(np.asarray(a.orientation_xyzw)+b.orientation_xyzw))
                                  for a, b in zip(outputs[False], outputs[True], strict=True)]
            if max(errors) > 1e-8 or max(orientation_errors) > 1e-8:
                raise RuntimeError('Native/local FK mismatch')
            assert counts[False] == [args.samples]*args.repeats
            assert counts[True] == [1]*args.repeats
            report['arms'][arm] = dict(
                remote_median_s=statistics.median(times[False]),
                local_median_s=statistics.median(times[True]),
                remote_samples_s=times[False], local_samples_s=times[True],
                remote_calls=counts[False], verified_local_remote_calls=counts[True],
                max_position_error_m=max(errors), max_quaternion_error=max(orientation_errors))
    finally:
        executor.shutdown(timeout_sec=5.)
        thread.join(timeout=5.)
        node.destroy_node()
        rclpy.try_shutdown()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
