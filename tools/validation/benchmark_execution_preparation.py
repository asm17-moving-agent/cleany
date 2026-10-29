"""Compare deployed preparation code and isolated ROS transports without motors."""
from __future__ import annotations

import argparse
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import statistics
import sys
import threading
import time
from types import SimpleNamespace as NS


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def stats(values):
    ordered = sorted(values)
    return dict(median_ms=statistics.median(values)*1000,
                p95_ms=ordered[min(len(ordered)-1, int(len(ordered)*.95))]*1000,
                samples=len(values))


def preparation(baseline, repeats):
    from sensor_msgs.msg import JointState
    from cleany_skill_executor.core.physical_grasp_selection import REQUIRED_JOINT_NAMES
    from cleany_skill_executor.hardware_execution import HardwareExecution
    from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline
    previous = load_module('cleany_skill_executor._preparation_before', baseline/
        'ros2_ws/src/cleany_skill_executor/cleany_skill_executor/hardware_execution.py')
    names = [*REQUIRED_JOINT_NAMES, 'head_pan_joint', 'head_tilt_joint']
    state = JointState(name=names, position=[0.]*len(names),
                       velocity=[0.]*len(names), effort=[0.]*len(names))
    state.header.stamp.sec = 1
    node = NS(state=state, lock=threading.Lock(), mode='hardware',
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000)),
              get_parameter=lambda _: NS(value=.25), check_execution=lambda: None)
    node.measured = lambda: PregraspPipeline.measured(node)
    expected = tuple(name for name in names if name.startswith('right_'))
    samples = [NS(positions=(index*.0005,)*len(expected)) for index in range(515)]
    base = dict(zip(names, state.position))
    hw = NS(node=node, fault='', guard={'state':'ready'}, uncommanded=None,
            value=lambda _: .25, complete_feedback_stamp_ns=1_000_000_000,
            _motion_monitors=(), _active_recovery_generation=None)
    monitor_calls = []
    def monitor():
        # The synthetic source represents continuously refreshed idle feedback.
        hw.guard_time = time.monotonic()
        monitor_calls.append(True)
        return HardwareExecution.monitor(hw)
    hw.monitor = monitor
    timing = {key: [] for key in ('before', 'after')}
    build_time = {key: [] for key in timing}
    outputs, counts = {}, {}
    for repeat in range(repeats+5):
        for key in (('before','after') if repeat % 2 else ('after','before')):
            collected, monitor_calls, first = [], [], []
            began = time.perf_counter()
            def batch(states, _call):
                if not first:
                    first.append(time.perf_counter()-began)
                collected.extend(deepcopy(states))
                return NS(valid=[True]*len(states), contacts=[], first_invalid_index=-1)
            hw._batch_validity = NS(batch_size=64, check=batch)
            hw.guard_time = time.monotonic()
            cls = previous.HardwareExecution if key == 'before' else HardwareExecution
            cls.validate_collision_samples(hw, 'right', expected, samples, base, state)
            elapsed = time.perf_counter()-began
            if repeat >= 5:
                timing[key].append(elapsed)
                build_time[key].append(first[0])
            outputs[key], counts[key] = collected, len(monitor_calls)
    assert outputs['before'] == outputs['after'] and len(outputs['after']) == 515
    return dict(scope='Production methods with synthetic feedback; collision RPC replaced by an all-valid recorder',
                messages_identical=True, states=515, requests=9,
                before_first_rpc={key: stats(value) for key,value in build_time.items()},
                total_including_fake_recorder={key: stats(value) for key,value in timing.items()},
                monitor_calls=counts)


def clients(baseline, repeats):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.action import ActionClient, ActionServer
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from cleany_manipulation_bt.action_tracking import MotionClientPool, quiescent

    previous = load_module('cleany_manipulation_bt._tracking_before', baseline/
        'ros2_ws/src/cleany_manipulation_bt/cleany_manipulation_bt/action_tracking.py')

    context = Context()
    rclpy.init(context=context)
    node = Node('execution_preparation_benchmark', context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    owner = NS(submissions=[], handles=[], execution_context=NS(abort=threading.Event()),
               check_execution=lambda: None)
    created = []
    def factory(kind, name):
        created.append(name)
        return ActionClient(node, kind, name, callback_group=ReentrantCallbackGroup())
    pool = MotionClientPool(owner, factory)
    endpoints = ['/cleany_validation_motion_pool/arm', '/cleany_validation_motion_pool/gripper']
    def execute(handle):
        handle.succeed()
        return FollowJointTrajectory.Result(error_code=0)
    def server(name):
        return ActionServer(node, FollowJointTrajectory, name, execute,
                            callback_group=ReentrantCallbackGroup())
    servers = [server(name) for name in endpoints]
    def wait(future):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(5.):
            raise TimeoutError('Isolated action response timed out')
        return future.result()
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['synthetic_joint']
    goal.trajectory.points = [JointTrajectoryPoint(positions=[0.], velocities=[0.]),
                              JointTrajectoryPoint(positions=[.1], velocities=[0.])]
    goal.trajectory.points[-1].time_from_start.sec = 1
    records = {key: [] for key in ('before', 'after')}
    cold = None
    restart_verified = False
    try:
        for repeat in range(repeats+3):
            for key in (('before','after') if repeat % 2 else ('after','before')):
                began = time.perf_counter()
                leased = [(previous.TrackedClient(owner, factory(FollowJointTrajectory, name))
                           if key == 'before' else pool.acquire(FollowJointTrajectory, name))
                          for name in endpoints]
                made = time.perf_counter()
                assert all(client.wait_for_server(timeout_sec=5.) for client in leased)
                ready = time.perf_counter()
                futures = [client.send_goal_async(goal) for client in leased]
                handles = [wait(future) for future in futures]
                accepted = time.perf_counter()
                assert all(handle.accepted for handle in handles)
                results = [wait(handle.get_result_async()) for handle in handles]
                assert all(result.result.error_code == 0 for result in results)
                # Allow tracking callbacks to finish before requesting lease reuse.
                deadline = time.monotonic()+5.
                while not quiescent(owner.submissions, owner.handles):
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Tracking did not reach terminal state')
                    time.sleep(.001)
                for client in leased:
                    client.destroy()
                ended = time.perf_counter()
                row = dict(acquire_sec=made-began, ready_sec=ready-made,
                           acquire_to_accepted_sec=accepted-began, full_mock_cycle_sec=ended-began)
                if key == 'after' and cold is None:
                    cold = row
                if repeat >= 3:
                    records[key].append(row)
                owner.submissions.clear()
                owner.handles.clear()
        old_transport = pool._clients[(FollowJointTrajectory, endpoints[0])]
        servers[0].destroy()
        servers[0] = server(endpoints[0])
        lease = pool.acquire(FollowJointTrajectory, endpoints[0])
        assert lease.client is old_transport and lease.wait_for_server(timeout_sec=5.)
        handle = wait(lease.send_goal_async(goal))
        assert handle.accepted and wait(handle.get_result_async()).result.error_code == 0
        lease.destroy()
        restart_verified = True
    finally:
        pool.close()
        for instance in servers:
            instance.destroy()
        executor.shutdown(timeout_sec=5.)
        worker.join(timeout=5.)
        node.destroy_node()
        context.shutdown()
    return dict(scope='Real ROS Action clients and immediate synthetic servers, no hardware controllers',
                endpoints=endpoints, cold_first_use=cold, server_restart_reused_transport=restart_verified,
                before={field:stats([r[field] for r in records['before']]) for field in records['before'][0]},
                after={field:stats([r[field] for r in records['after']]) for field in records['after'][0]},
                raw_client_creations=len(created),
                physical_motion_simulated=False, physical_duration_not_measured=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=30)
    args = parser.parse_args()
    result = dict(motor_commands_sent=False, production_ros_endpoints_used=False,
                  preparation=preparation(args.baseline, args.repeats),
                  clients=clients(args.baseline, args.repeats))
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
