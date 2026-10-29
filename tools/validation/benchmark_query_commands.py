"""Measure isolated read RPCs and production command serialization without motors."""
import argparse
import json
import os
from pathlib import Path
import statistics
import shutil
import subprocess
import threading
import time
from types import SimpleNamespace as NS

import numpy as np


def stats(values):
    return dict(median_ms=statistics.median(values)*1000,
                p95_ms=float(np.percentile(values, 95))*1000, repeats=len(values))


def build_jtc_probe(output):
    source, build = output/'jtc-source', output/'jtc-build'
    source.mkdir(parents=True, exist_ok=True)
    shutil.copy2(Path(__file__).with_name('jtc_sparse_command_probe.cpp'), source)
    (source/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(jtc_sparse_probe LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
add_executable(jtc_sparse_probe jtc_sparse_command_probe.cpp)
target_compile_features(jtc_sparse_probe PRIVATE cxx_std_17)
target_link_libraries(jtc_sparse_probe joint_trajectory_controller::joint_trajectory_controller)
''')
    with (output/'jtc-build.log').open('w') as log:
        subprocess.run(['cmake', '-S', str(source), '-B', str(build),
                        '-DPython3_EXECUTABLE=/usr/bin/python3'], stdout=log, stderr=log, check=True)
        subprocess.run(['cmake', '--build', str(build), '-j2'], stdout=log, stderr=log, check=True)
    return (build/'jtc_sparse_probe').resolve()


def queries(repeats):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from std_srvs.srv import Trigger
    from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline
    from cleany_skill_executor.readonly_services import call_readonly_services
    context = Context()
    rclpy.init(context=context)
    controller = Node('query_probe_controller', context=context)
    planner = Node('query_probe_planner', context=context)
    client_node = Node('query_probe_client', context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    for node in (controller, planner, client_node): executor.add_node(node)
    delay = [0.]
    def callback(request, response):
        if delay[0]: time.sleep(delay[0])
        response.success = True
        return response
    servers, clients = [], []
    for i in range(4):
        name = f'/cleany_query_probe/read_{i}'
        servers.append((controller if i < 3 else planner).create_service(Trigger, name, callback))
        clients.append(client_node.create_client(Trigger, name))
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    owner = NS(check_execution=lambda: None)
    def wait(future, timeout, **_):
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout):
            future.cancel()
            raise RuntimeError('Isolated probe RPC timed out')
        return future.result()
    owner.wait = wait
    rows = []
    try:
        assert all(client.wait_for_service(timeout_sec=5.) for client in clients)
        for seconds in (0., .003):
            delay[0] = seconds
            times = dict(sequential=[], parallel=[])
            for repeat in range(repeats+5):
                for mode in (('sequential', 'parallel') if repeat % 2 else ('parallel', 'sequential')):
                    started = time.perf_counter()
                    calls = [(client, Trigger.Request()) for client in clients]
                    if mode == 'parallel': responses = call_readonly_services(owner, calls)
                    else: responses = [PregraspPipeline.call(owner, c, r) for c, r in calls]
                    assert len(responses) == 4 and all(r.success for r in responses)
                    if repeat >= 5: times[mode].append(time.perf_counter()-started)
            rows.append(dict(simulated_work_ms_per_service=seconds*1000,
                             statistics={key: stats(values) for key, values in times.items()}))
    finally:
        executor.shutdown()
        worker.join(5.)
        for node in (client_node, planner, controller): node.destroy_node()
        context.shutdown()
    return dict(scope='Isolated ROS services; first three share one mutually-exclusive server callback group; fourth uses a separate server. Not live controller/model timings.', scenarios=rows)


def commands(inputs, repeats, output, jtc_probe):
    from rclpy.serialization import serialize_message
    from cleany_skill_executor.core.hardware_execution import PhysicalJointLimit, prepare_bounded_path
    from cleany_skill_executor.trajectory_io import command_samples_to_message
    names = tuple(inputs['joint_names'])
    # This historical-geometry timing probe is not an operational range check.
    limits = {n: PhysicalJointLimit(-10., 10., 1., 1.5, 2048., 1., 0., -10000., 10000.) for n in names}
    rows = []
    for stage in ('pregrasp', 'return'):
        path = prepare_bounded_path(names, [tuple(p) for p in inputs[stage+'_positions']], limits,
                                    velocity=.8, acceleration=1.2)
        delay_ns = round(inputs['start_delay_sec']*1e9)
        def messages(samples):
            return [command_samples_to_message(names[:5], samples, path.duration,
                        indices=range(5), delay_ns=delay_ns),
                    command_samples_to_message(names[5:], samples, path.duration,
                        indices=(5,), delay_ns=delay_ns)]
        times = dict(dense=[], sparse=[])
        sizes, outputs = {}, {}
        for repeat in range(repeats+5):
            for mode in (('dense', 'sparse') if repeat % 2 else ('sparse', 'dense')):
                started = time.perf_counter()
                pair = messages(path.samples if mode == 'dense' else path.command_knots)
                blobs = [serialize_message(message) for message in pair]
                if repeat >= 5: times[mode].append(time.perf_counter()-started)
                sizes[mode], outputs[mode] = sum(map(len, blobs)), pair
        evidence = []
        for i, label in enumerate(('arm', 'gripper')):
            before, after = outputs['dense'][i], outputs['sparse'][i]
            end = before.points[-1].time_from_start
            end_ns = end.sec*10**9+end.nanosec
            probes = {int(ns) for ns in np.linspace(0, end_ns, 4001)}
            for point in after.points:
                ns = point.time_from_start.sec*10**9+point.time_from_start.nanosec
                probes.update(t for t in (ns-1, ns, ns+1) if 0 <= t <= end_ns)
            lines = [f'{len(before.joint_names)} {len(before.points)} {len(after.points)} {len(probes)}']
            for message in (before, after):
                for point in message.points:
                    ns = point.time_from_start.sec*10**9+point.time_from_start.nanosec
                    lines.append(' '.join(map(str, (ns, *point.positions, *point.velocities))))
            lines.extend(map(str, sorted(probes)))
            text = '\n'.join(lines)+'\n'
            (output/f'{stage}-{label}-jtc-input.txt').write_text(text)
            result = subprocess.run([str(jtc_probe)], input=text, text=True, capture_output=True, check=True)
            evidence.append(dict(controller=label, **json.loads(result.stdout)))
        rows.append(dict(stage=stage, validation_samples=len(path.samples), duration_sec=path.duration,
            points_per_controller=dict(dense=len(outputs['dense'][0].points), sparse=len(outputs['sparse'][0].points)),
            serialized_bytes=sizes, statistics={key: stats(values) for key, values in times.items()},
            installed_jtc_sampler=evidence))
    return dict(scope='Historical geometry with synthetic generous position/raw bounds, production bounded_path and message encoder; message construction plus ROS serialization only. Not operational path eligibility; no motor or action execution.', stages=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--jtc-probe', type=Path)
    parser.add_argument('--repeats', type=int, default=100)
    args = parser.parse_args()
    assert args.repeats >= 5
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') == '226'
    args.output.mkdir(parents=True, exist_ok=True)
    probe = args.jtc_probe or build_jtc_probe(args.output)
    result = dict(motor_commands_sent=False, controller_actions_sent=False,
        queries=queries(min(40, args.repeats)),
        commands=commands(json.loads(args.inputs.read_text()), args.repeats, args.output, probe))
    (args.output/'benchmark.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
