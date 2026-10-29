"""Compare interrupted VLM preparation with completed startup IK on saved inputs.

Run each mode in a fresh motion-container process. No ROS node or actuator
client is created. The recorded batch targets, locked state and seed policy
are identical; startup preparation is reported separately from request time.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

from sensor_msgs.msg import JointState

from cleany_grasping.gpu_scheduling import GpuLease
from cleany_skill_executor.curobo_model import CuRoboConfig
from cleany_skill_executor.hybrid_adapter import HybridConfig, HybridGraspAdapter
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('interrupted', 'completed'), required=True)
    parser.add_argument('--statistics', type=Path, required=True)
    parser.add_argument('--runtime', type=Path, required=True)
    args = parser.parse_args()
    stats = json.loads(args.statistics.read_text())
    xml = json.loads(args.runtime.read_text())['robot_description']
    state = dict(zip(stats['start_joint_state']['names'],
                     stats['start_joint_state']['positions_rad'], strict=True))
    targets = sorted((r for r in stats['ik_diagnostics'] if r['stage'] == 'GRASP_IK'),
                     key=lambda r: r['batch_target_index'])
    assert len(targets) == 8 and all(r['arm'] == 'left' for r in targets)
    positions = [r['target']['position_m'] for r in targets]
    config = MoveItAdapterConfig(position_tolerance_m=.01,
        pregrasp_position_tolerance_m=.01, approach_tolerance_degrees=30.)
    # The production adapter asks for a scene-client handle during construction;
    # this inert handle cannot issue a service request.
    node = SimpleNamespace(create_client=lambda *args, **kwargs: object())
    adapter = HybridGraspAdapter(node, config,
        gpu_config=CuRoboConfig(num_seeds=16,
            geometry_cache_directory='/opt/cleany-motion/cache/robot-geometry'),
        hybrid_config=HybridConfig(batch_size=8, ik_iterations=50,
            group_gripper_openings=True, pregrasp_retry_passes=0),
        ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    lease = GpuLease('/workspace/cleany/configs/robot/gpu-work.lock')
    phases = []
    try:
        adapter.set_robot_description(xml)
        adapter.set_current_state(JointState(name=list(state), position=list(state.values())))
        adapter.set_deadline(time.monotonic() + 180.)
        adapter.set_progress_callback(phases.append)
        with lease.acquire(foreground=False, blocking=False) as admitted:
            if not admitted:
                raise RuntimeError('Foreground GPU work is active; retry when idle')
            began = time.perf_counter()
            if args.mode == 'interrupted':
                checks = [0]
                def continue_work():
                    checks[0] += 1
                    return checks[0] < 3 and not lease.foreground_pending()
                try:
                    adapter.warmup(['right'], include_aim=False, continue_work=continue_work)
                except InterruptedError:
                    pass
                else:
                    raise AssertionError('Expected yield after right solver setup')
            else:
                adapter.warmup(['right', 'left'], include_aim=False,
                    continue_work=lambda: not lease.foreground_pending())
            preparation = time.perf_counter() - began
            gpu = adapter._gpus[16]
            # Match the production batch's grouped opening, including a locked
            # state update after warmup rather than warming the exact request.
            request_state = {**state, 'left_gripper_joint': stats['batch_calls'][0]['gripper_position']}
            seeds = gpu._seed_bases('left', None)
            engine = gpu._engine
            before_keys = [list(k) for k in engine._batch_ik_caches]
            rows = []
            for index in range(6):
                if lease.foreground_pending():
                    raise InterruptedError('Foreground GPU work requested')
                began = time.perf_counter()
                result = engine.solve_batch('left', 'left_grasp_tcp', positions,
                    [seeds] * 8, request_state, {})
                rows.append({'index': index, 'wall_sec': time.perf_counter() - began,
                    'solutions': [len(group) for group in result]})
            output = {'mode': args.mode, 'preparation_sec': preparation,
                'request_calls': rows, 'before_request_cache_keys': before_keys,
                'phases': phases, 'targets': 8, 'seeds_per_target': 16,
                'input_sha256': hashlib.sha256(args.statistics.read_bytes()).hexdigest(),
                'urdf_sha256': hashlib.sha256(xml.encode()).hexdigest(),
                'motor_commands_sent': False, 'ros_nodes_started': False}
            print('WARMUP_BENCHMARK=' + json.dumps(output), flush=True)
    finally:
        adapter.close()


if __name__ == '__main__':
    main()
