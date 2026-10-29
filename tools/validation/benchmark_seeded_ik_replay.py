"""Replay recorded IK targets without DDS, GPU initialization or motor commands."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import time

from sensor_msgs.msg import JointState

from cleany_skill_executor.core.physical_grasp_selection import JointSolution, PoseValue, PlannedPath
from cleany_skill_executor.hybrid_adapter import HybridConfig, HybridGraspAdapter
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--statistics', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--repetitions', type=int, default=7)
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error('repetitions must be positive')
    data = json.loads(args.statistics.read_text())
    config = MoveItAdapterConfig(use_verified_local_fk=True, pose_refinement_iterations=10,
        position_tolerance_m=.01, approach_tolerance_degrees=30., closing_tolerance_degrees=30.)
    adapter = HybridGraspAdapter(object(), config,
        hybrid_config=HybridConfig(seeded_local_ik=True),
        ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    adapter.set_robot_description(args.urdf.read_text())
    state = data['start_joint_state']
    adapter.set_current_state(JointState(name=state['names'], position=state['positions_rad']))
    grasps, records = {}, []
    for row in data['ik_diagnostics']:
        key = row['arm'], row['candidate_index']
        if row['stage'] == 'GRASP_IK' and row['outcome'] == 'accepted':
            grasps[key] = row
        if row['stage'] != 'PREGRASP_IK' or key not in grasps:
            continue
        grasp = grasps[key]
        recorded = grasp['final_alignment']
        seed = JointSolution(tuple(recorded['joint_names']), tuple(recorded['joint_positions_rad']))
        seed_pose = PoseValue(tuple(recorded['fk_position_m']), tuple(recorded['fk_orientation_xyzw']))
        target = PoseValue(tuple(row['target']['position_m']), tuple(row['target']['orientation_xyzw']))
        # Only compare URDF parity at this recorded seed. This is not a live
        # MoveIt service or a collision-validity witness for the replayed result.
        def recorded_fk(arm, solution):
            assert (arm, solution) == (key[0], seed)
            return seed_pose
        adapter._remote_forward_kinematics = recorded_fk
        adapter._verified_fk_arms.clear()
        adapter.set_gripper_position(key[0], row['gripper_joint_position_rad'])
        samples = []
        for _ in range(args.repetitions):
            adapter._fk_cache.clear()
            began = time.perf_counter()
            result = adapter._refine_seed(key[0], target, seed)
            samples.append(time.perf_counter()-began)
        grasp_target = PoseValue(tuple(grasp['target']['position_m']),
                                 tuple(grasp['target']['orientation_xyzw']))
        path = PlannedPath(seed.names, (result.solution.positions, seed.positions))
        corridor = adapter._corridor_samples_valid(key[0], seed.names,
            adapter._path_samples(path), target, grasp_target)
        records.append({
            'candidate_index': key[1], 'arm': key[0], 'target': row['target'],
            'historical_outcome': row['outcome'],
            'local_wall_sec': samples, 'median_sec': statistics.median(samples),
            'position_error_mm': result.position_error_m*1000.,
            'approach_error_degrees': result.approach_error_degrees,
            'closing_error_degrees': result.closing_error_degrees,
            'pose_feasible': adapter._feasible_alignment(result),
            'direct_approach_geometry_feasible': corridor,
            'collision_checked': False,
            'joint_positions_rad': list(result.solution.positions),
        })
    print(json.dumps({
        'scope': 'Jetson CPU historical-pose replay; no live collision scene or physical run',
        'ros_nodes_started': False, 'motor_commands_sent': False,
        'gpu_initialized': False, 'records': records,
        'input_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in (args.statistics, args.urdf)},
    }, indent=2))


if __name__ == '__main__':
    main()
