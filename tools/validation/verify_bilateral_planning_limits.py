"""Inspect live MoveIt parameters or actual CUDA IK limits without motor access."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def gpu_limits(model_path: Path) -> dict:
    import torch
    import xacro
    from ament_index_python.packages import get_package_share_directory
    from cleany_skill_executor.curobo_batch import CuRoboBatchEngine
    from cleany_skill_executor.curobo_model import CuRoboConfig
    from cleany_skill_executor.hybrid_adapter import _CandidateRobotModel
    from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution
    from cleany_skill_executor.core.urdf_fk import UrdfChain

    xml = model_path.read_text()
    semantic = Path(get_package_share_directory('cleany_moveit_config')) / 'config/cleany.srdf.xacro'
    srdf = xacro.process_file(str(semantic), mappings={'include_head_camera': 'true'}).toxml()
    config = CuRoboConfig(num_seeds=4, use_cuda_graph=False)
    model = _CandidateRobotModel(xml, srdf, config, exclude_opposite_arm_cost=True)
    state = {name: (lower + upper) / 2 for name, (lower, upper) in model.limits.items()}
    engine = CuRoboBatchEngine(model, config, ik_iterations=10, ik_batch_size=2)
    arms = {}
    try:
        for arm, names in ARM_JOINT_NAMES.items():
            solver = engine._solver(arm, f'{arm}_grasp_tcp', state, {})
            position = solver.kinematics.get_joint_limits().position.detach().cpu().numpy()
            values = {}
            for name in names:
                actual = position[:, solver.joint_names.index(name)]
                expected = model.limits[name]
                assert np.allclose(actual, expected, atol=2e-7, rtol=0), (name, actual, expected)
                values[name] = actual.tolist()
            seed = JointSolution(names, tuple(state[name] for name in names))
            target = UrdfChain(xml, 'base_link', f'{arm}_grasp_tcp').pose(state)[0].tolist()
            groups = engine.solve_batch(arm, f'{arm}_grasp_tcp', [target, target],
                                        [[seed], [seed]], state, {})
            for group in groups:
                for solution in group:
                    for name, value in zip(solution.names, solution.positions):
                        low, high = model.limits[name]
                        assert low - 2e-7 <= value <= high + 2e-7
            arms[arm] = {'solver_bounds_rad': values,
                         'ik_solutions_within_bounds': sum(map(len, groups))}
        torch.cuda.synchronize()
    finally:
        engine.close()
    return {'device': torch.cuda.get_device_name(), 'arms': arms}


def moveit_limits(model_path: Path) -> dict:
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python
    from cleany_skill_executor.core.hardware_execution import physical_limits

    xml = model_path.read_text()
    expected = {j.get('name'): [float(j.find('limit').get(k)) for k in ('lower', 'upper')]
                for j in ET.fromstring(xml).findall('joint')
                if j.get('type') == 'revolute' and not j.get('name').startswith('head_')}
    execution_tree = ET.fromstring(xml)
    read_only = execution_tree.find("ros2_control/hardware/param[@name='read_only']")
    if read_only is not None:
        # Exercise the command parser in memory; the running model stays read-only.
        read_only.text = 'false'
    for arm in ('left', 'right'):
        for name, limit in physical_limits(ET.tostring(execution_tree, encoding='unicode'), arm).items():
            assert [limit.lower, limit.upper] == expected[name]
    rclpy.init()
    node = rclpy.create_node('bilateral_limits_verifier')
    try:
        client = node.create_client(GetParameters, '/move_group/get_parameters')
        assert client.wait_for_service(timeout_sec=30), 'MoveIt parameter service unavailable'
        keys = ['robot_description', 'allow_trajectory_execution']
        keys += [f'robot_description_planning.joint_limits.{name}.{key}'
                 for name in expected for key in ('min_position', 'max_position')]
        future = client.call_async(GetParameters.Request(names=keys))
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        assert future.done(), 'MoveIt parameter query timed out'
        values = {key: parameter_value_to_python(value)
            for key, value in zip(keys, future.result().values)}
        assert values['robot_description'] == xml, 'MoveIt loaded a different URDF'
        assert values['allow_trajectory_execution'] is False
        for name, pair in expected.items():
            actual = [values[f'robot_description_planning.joint_limits.{name}.{key}']
                      for key in ('min_position', 'max_position')]
            assert np.allclose(actual, pair, atol=1e-12, rtol=0), (name, actual, pair)
        return {'trajectory_execution_enabled': False, 'executor_bounds_match': True,
                'joints': expected,
                'loaded_model_sha256': hashlib.sha256(xml.encode()).hexdigest()}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', action='store_true')
    args = parser.parse_args()
    result = {'hardware_writes': False,
              'model_sha256': hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
              'gpu' if args.gpu else 'moveit': (gpu_limits if args.gpu else moveit_limits)(args.urdf)}
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
