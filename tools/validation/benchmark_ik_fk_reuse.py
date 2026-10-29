"""Compare recorded IK proposal scoring on real CUDA, without ROS services."""
from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path
import statistics
import time

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory

from cleany_skill_executor.core.physical_grasp_selection import JointSolution, PoseValue
from cleany_skill_executor.curobo_batch import CuRoboBatchAdapter
from cleany_skill_executor.curobo_engine import CuRoboEngine
from cleany_skill_executor.curobo_model import CuRoboConfig, RobotModel
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--statistics', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = ast.parse(args.before.read_text())
    cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'CuRoboBatchAdapter')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_score_batch')
    namespace = dict(CuRoboBatchAdapter._score_batch.__globals__)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(args.before), 'exec'), namespace)
    before = namespace['_score_batch']
    data = json.loads(args.statistics.read_text())
    state = dict(zip(data['start_joint_state']['names'], data['start_joint_state']['positions_rad']))
    records = [r for r in data['ik_diagnostics']
               if r['stage'] == 'GRASP_IK' and r['backend'] == 'gpu_primary']
    targets, solutions = [], []
    arm = records[0]['arm']
    for row in records:
        if row['arm'] != arm:
            continue
        targets.append(PoseValue(tuple(row['target']['position_m']), tuple(row['target']['orientation_xyzw'])))
        solutions.append([JointSolution(tuple(row[key]['joint_names']), tuple(row[key]['joint_positions_rad']))
                          for key in ('closest_alignment', 'proposal_alignment', 'final_alignment')
                          if row.get(key) is not None])
    gpu_config = CuRoboConfig(num_seeds=16, sphere_model='joint_simplified',
        geometry_cache_directory='/opt/cleany-motion/cache/robot-geometry')
    srdf = Path(get_package_share_directory('cleany_moveit_config'))/'config/cleany.srdf.xacro'
    semantic = xacro.process_file(str(srdf), mappings={'include_head_camera': 'true'}).toxml()
    model = RobotModel(args.urdf.read_text(), semantic, gpu_config)
    engine = CuRoboEngine(model, gpu_config)
    try:
        adapter = CuRoboBatchAdapter.__new__(CuRoboBatchAdapter)
        adapter._config = MoveItAdapterConfig(align_wrist_roll=True)
        adapter._joint_limits = {name: (*bounds, 1.) for name, bounds in model.limits.items()}
        adapter._engine = engine
        counts = []
        def compute(fn, *values):
            counts.append(len(values[2]))
            return fn(*values)
        adapter._compute = compute
        results = {}
        for label, function in [('before', before), ('after', CuRoboBatchAdapter._score_batch)]:
            function(adapter, arm, targets, solutions, state)
            times, work = [], []
            for _ in range(7):
                counts.clear()
                began = time.perf_counter()
                scored = function(adapter, arm, targets, solutions, state)
                times.append(time.perf_counter()-began)
                work.append(sum(counts))
            results[label] = {'median_sec': statistics.median(times), 'wall_sec': times,
                'fk_states_per_call': work, 'scored': scored}
        old, new = results['before'].pop('scored'), results['after'].pop('scored')
        assert [len(g) for g in old] == [len(g) for g in new]
        maximum = 0.
        for a, b in zip(old, new, strict=True):
            for x, y in zip(a, b, strict=True):
                assert x.solution == y.solution
                errors = np.abs(np.array((x.position_error_m, x.approach_error_degrees, x.closing_error_degrees))
                                - (y.position_error_m, y.approach_error_degrees, y.closing_error_degrees))
                maximum = max(maximum, float(errors.max()))
                assert errors[0] < 1e-7 and max(errors[1:]) < 1e-4
        args.output.write_text(json.dumps({'environment': 'Jetson motion container, real CUDA FK',
            'scope': 'Recorded alignment subset, not the complete original seed batch',
            'target_count': len(targets), 'input_solutions': sum(map(len, solutions)),
            'results': results, 'max_score_difference': maximum,
            'ros_nodes_started': False, 'motor_commands_sent': False}, indent=2))
    finally:
        engine.close()


if __name__ == '__main__':
    main()
