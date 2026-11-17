#!/usr/bin/env python3
"""Verify GPU grasp endpoint limits against an explicit frozen URDF, without motor access."""
from __future__ import annotations

import argparse
from dataclasses import fields
import hashlib
import json
from pathlib import Path
import time

import numpy as np


def verify_gpu(model_path: Path, parameters: dict, output: Path,
               geometry_cache: Path | None = None) -> dict:
    import torch
    import xacro
    from ament_index_python.packages import get_package_share_directory

    from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution
    from cleany_skill_executor.core.urdf_fk import UrdfChain
    from cleany_skill_executor.curobo_batch import CuRoboBatchEngine
    from cleany_skill_executor.curobo_model import CuRoboConfig
    from cleany_skill_executor.hybrid_adapter import _CandidateRobotModel

    if parameters['motion_backend'] != 'curobo_ik_moveit':
        raise ValueError('Validation requires the GPU hybrid backend')
    if parameters.get('curobo_ik_cpu_fallback', False):
        raise ValueError('CPU IK fallback must remain disabled')
    if not torch.cuda.is_available():
        raise RuntimeError('A CUDA-enabled runtime is required')
    torch.set_num_threads(1)
    xml = model_path.read_text()
    semantic = Path(get_package_share_directory('cleany_moveit_config')) / 'config/cleany.srdf.xacro'
    srdf = xacro.process_file(str(semantic), mappings={'include_head_camera': 'true'}).toxml()
    values = {f.name: parameters['curobo_'+f.name]
              for f in fields(CuRoboConfig) if 'curobo_'+f.name in parameters}
    values.update(num_seeds=parameters['curobo_ik_primary_seeds'],
                  geometry_cache_directory=str(geometry_cache or output/'geometry-cache'))
    config = CuRoboConfig(**values)
    margin = float(parameters['curobo_ik_endpoint_margin_rad'])
    model = _CandidateRobotModel(xml, srdf, config,
        exclude_opposite_arm_cost=parameters['curobo_ik_exclude_opposite_arm_cost'],
        endpoint_margin_rad=margin)
    state = {name: (a+b)/2 for name, (a,b) in model.limits.items()}
    for side in ARM_JOINT_NAMES:
        state[side+'_gripper_joint'] = float(parameters['gripper_open_joint_rad'])
    physical_limits = dict(model.limits)
    engine = CuRoboBatchEngine(model, config,
        ik_iterations=parameters['curobo_ik_iterations'],
        ik_batch_size=parameters['curobo_ik_batch_size'],
        adaptive_ik_batch_size=parameters['curobo_ik_adaptive_batch_size'])
    results = {}
    try:
        for arm, names in ARM_JOINT_NAMES.items():
            began = time.monotonic()
            solver = engine._solver(arm, arm+'_grasp_tcp', state, {})
            limits = solver.kinematics.get_joint_limits().position.detach().cpu().numpy()
            expected = {name: [model.limits[name][0]+margin, model.limits[name][1]-margin]
                        for name in names}
            for name in names:
                actual = limits[:, solver.joint_names.index(name)]
                np.testing.assert_allclose(actual, expected[name], atol=2e-6, rtol=0.)
            seeds, targets = [], []
            chain = UrdfChain(xml, 'base_link', arm+'_grasp_tcp')
            for index in range(parameters['curobo_ik_batch_size']):
                q = dict(state)
                q[names[1]] += .01*(index-3.5)
                seeds.append([JointSolution(names, tuple(q[name] for name in names))])
                targets.append(chain.pose(q)[0].tolist())
            groups = engine.solve_batch(arm, arm+'_grasp_tcp', targets, seeds, state, {})
            if len(groups) != len(targets) or any(not group for group in groups):
                raise RuntimeError('A known-reachable GPU batch target had no solution')
            largest_position_error = 0.
            largest_fk_disagreement = 0.
            for target, group in zip(targets, groups):
                gpu_poses = engine.fk(arm, arm+'_grasp_tcp', group, state)
                for solution, gpu_pose in zip(group, gpu_poses):
                    q = {**state, **dict(zip(solution.names, solution.positions))}
                    largest_fk_disagreement = max(largest_fk_disagreement,
                        float(np.linalg.norm(chain.pose(q)[0]-gpu_pose.position)))
                    largest_position_error = max(largest_position_error,
                        float(np.linalg.norm(chain.pose(q)[0]-target)))
                    for name, value in zip(solution.names, solution.positions):
                        low, high = expected[name]
                        if not low-2e-6 <= value <= high+2e-6:
                            raise RuntimeError('A GPU solution escaped the endpoint margin')
            torch.cuda.synchronize()
            results[arm] = dict(targets=len(targets), solutions=sum(map(len, groups)),
                solver_endpoint_bounds_rad=expected,
                maximum_position_error_m=largest_position_error,
                maximum_gpu_cpu_fk_disagreement_m=largest_fk_disagreement,
                wall_sec=time.monotonic()-began)
            if largest_fk_disagreement > 2e-6:
                raise RuntimeError('GPU FK disagrees with the calibrated CAD chain')
        assert model.limits == physical_limits
        assert model_path.read_text() == xml
    finally:
        engine.close()
    return dict(device=torch.cuda.get_device_name(), backend='curobo_ik_moveit',
        cpu_ik_fallback=False, batch_size=parameters['curobo_ik_batch_size'],
        endpoint_margin_rad=margin, physical_limits_unchanged=True,
        source_urdf_sha256=hashlib.sha256(xml.encode()).hexdigest(),
        motor_access=False, arms=results)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--parameters', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--geometry-cache', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    parameters = json.loads(args.parameters.read_text())
    parameters = parameters.get('effective', parameters)
    result = verify_gpu(args.urdf, parameters, args.output, args.geometry_cache)
    (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
