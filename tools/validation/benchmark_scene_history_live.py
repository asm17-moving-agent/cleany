"""Paired before/after sensor waits on identical live, stationary feedback.

Only subscribes to telemetry and reads the planning scene. Never creates an
actuator client. Both adapters receive the same full DynamicJointState frame;
their sensor waits run concurrently against the same live monitor.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import json
from pathlib import Path
import statistics
import sys
import threading
import time
from types import SimpleNamespace as NS

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from control_msgs.msg import DynamicJointState
from sensor_msgs.msg import JointState

from cleany_skill_executor.core.joint_stability import JointStabilityHistory
from cleany_skill_executor.sensor_scene import SensorSceneMonitor


def load(path: Path, name: str):
    name = 'cleany_skill_executor.' + name
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.HardwareExecution


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--after', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rounds', type=int, default=20)
    args = parser.parse_args()
    classes = [load(args.before, 'history_before'), load(args.after, 'history_after')]
    rclpy.init()
    node = Node('scene_history_comparison', parameter_overrides=[
        Parameter('require_sensor_scene', value=True)])
    node.scene_monitor = SensorSceneMonitor(node)
    lock = threading.Lock()
    latest, frames = [None], []
    adapters = []
    settings = {'hardware_endpoint_settle_duration_sec': .3, 'hardware_guard_max_age_sec': .25}
    def measured():
        with lock:
            if latest[0] is None:
                raise RuntimeError('No complete feedback')
            stamp, positions = latest[0]
            if not 0 <= node.get_clock().now().nanoseconds-stamp <= .25e9:
                raise RuntimeError('Stale complete acquisition')
            return JointState(name=list(positions), position=list(positions.values()))
    for _ in classes:
        hw = NS(node=node, _stability_history=JointStabilityHistory(),
            contact_lock=threading.Lock(), contact_histories={}, fault='',
            reuse_scene_barrier=True, motion_submissions=0, communication_recovery_count=0,
            value=settings.__getitem__, monitor=measured, wait_for_settle=lambda _: measured())
        adapters.append(hw)
    def feedback(message):
        for cls, hw in zip(classes, adapters):
            cls.on_contact_feedback(hw, message)
        if any(hw.fault for hw in adapters):
            return
        rows = {name: dict(zip(row.interface_names, row.values))
                for name, row in zip(message.joint_names, message.interface_values)}
        positions = {name: row['position'] for name, row in rows.items() if 'position' in row}
        if len(positions) != 14:
            return
        stamp = adapters[0].complete_feedback_stamp_ns
        with lock:
            latest[0] = (stamp, positions)
            frames.append({'stamp_ns': stamp, 'positions': positions})
    subscription = node.create_subscription(DynamicJointState,
        '/joint_state_broadcaster/dynamic_joint_states', feedback, 10)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    results = []
    try:
        deadline = time.monotonic() + 25.
        while True:
            try:
                measured()
                node.scene_monitor.check()
                if len(frames) >= 50:
                    break
            except RuntimeError:
                pass
            if time.monotonic() >= deadline:
                raise RuntimeError('Live feedback/map did not become ready')
            time.sleep(.05)
        with ThreadPoolExecutor(max_workers=2) as pool:
            for index in range(args.rounds):
                state = measured()
                positions = dict(zip(state.name, state.position))
                now = node.get_clock().now().nanoseconds
                if not adapters[1]._stability_history.stable(list(positions), positions,
                        now_ns=now, duration_sec=.3, maximum_age_sec=.25, velocity_rad_s=.02):
                    raise RuntimeError('Robot is not stationary; abort comparison')
                barriers = [cls.capture_scene_barrier(hw, 'left') for cls, hw in zip(classes, adapters)]
                def wait(pair):
                    cls, hw, barrier = pair
                    began = time.perf_counter()
                    cls.wait_for_fresh_sensor_scene(hw, 'paired_benchmark', barrier=barrier)
                    return time.perf_counter()-began
                durations = list(pool.map(wait, zip(classes, adapters, barriers)))
                results.append({'round': index, 'before_sec': durations[0], 'after_sec': durations[1],
                    'barrier_advance_sec': (barriers[0]['stamp_ns']-barriers[1]['stamp_ns'])/1e9})
                time.sleep(.137)
        output = {'rounds': results, 'median_before_sec': statistics.median(r['before_sec'] for r in results),
            'median_after_sec': statistics.median(r['after_sec'] for r in results),
            'frames': frames, 'motor_commands_sent': False, 'paired_same_live_input': True}
        args.output.write_text(json.dumps(output, indent=2))
        print(json.dumps({k: v for k, v in output.items() if k not in ('frames', 'rounds')}))
    finally:
        node.scene_monitor.close()
        executor.shutdown()
        thread.join(timeout=3.)
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
