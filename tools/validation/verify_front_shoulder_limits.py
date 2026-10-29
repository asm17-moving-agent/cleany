"""Check installed front-sector IK and command bounds without motor access."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from sensor_msgs.msg import JointState

from cleany_skill_executor.core.hardware_execution import physical_limits
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution
from cleany_skill_executor.hybrid_adapter import HybridGraspAdapter
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter


BOUNDS = {'left': (-2.16, 0.), 'right': (0., 2.0033789089786476)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', action='store_true')
    args = parser.parse_args()
    xml = args.urdf.read_text()
    tree = ET.fromstring(xml)
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(
        use_verified_local_fk=True, pose_refinement_iterations=10),
        ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    adapter.set_robot_description(xml)
    state = dict.fromkeys(adapter._joint_limits, 0.)
    for arm, q in [('left', (-1.77, 1.22, 1.65, .90, 1.595)),
                   ('right', (1.908, .837, 1.002, 1.278, 1.618))]:
        state.update(zip(ARM_JOINT_NAMES[arm], q, strict=True))
        state[f'{arm}_gripper_joint'] = -.2
    adapter.set_current_state(JointState(name=list(state), position=list(state.values())))
    # This offline check uses one URDF, not a running MoveIt FK service.
    adapter._verify_local_fk = lambda *_: None
    result = {'urdf_sha256': hashlib.sha256(xml.encode()).hexdigest(), 'arms': {},
              'motion_commands': False, 'live_moveit_service': False}
    for arm, bounds in BOUNDS.items():
        names = ARM_JOINT_NAMES[arm]
        limit = tree.find(f"joint[@name='{names[0]}']/limit")
        assert tuple(float(limit.get(k)) for k in ('lower', 'upper')) == bounds
        assert adapter._joint_limits[names[0]][:2] == bounds
        physical = physical_limits(xml, arm)[names[0]]
        assert (physical.lower, physical.upper) == bounds
        seed = JointSolution(names, tuple(state[n] for n in names))
        refined = []
        for offset in np.linspace(-.03, .03, 8):
            target_q = JointSolution(names, (seed.positions[0]+float(offset), *seed.positions[1:]))
            target = adapter.forward_kinematics(arm, target_q)
            alignment = HybridGraspAdapter._refine_seed(adapter, arm, target, seed)
            assert bounds[0] <= alignment.solution.positions[0] <= bounds[1]
            refined.append(alignment.position_error_m)
        result['arms'][arm] = {'bounds_rad': bounds, 'cpu_refinement_targets': 8,
                              'cpu_position_error_max_m': max(refined)}
    if args.gpu:
        import torch
        import xacro
        from ament_index_python.packages import get_package_share_directory
        from cleany_skill_executor.curobo_batch import CuRoboBatchEngine
        from cleany_skill_executor.curobo_model import CuRoboConfig
        from cleany_skill_executor.hybrid_adapter import _CandidateRobotModel

        semantic = Path(get_package_share_directory('cleany_moveit_config'))
        srdf = xacro.process_file(str(semantic/'config/cleany.srdf.xacro'),
                                  mappings={'include_head_camera': 'true'}).toxml()
        config = CuRoboConfig(num_seeds=16, ik_position_threshold_m=.01,
                             geometry_cache_directory='/opt/cleany-motion/cache/front-sector')
        model = _CandidateRobotModel(xml, srdf, config, exclude_opposite_arm_cost=True)
        engine = CuRoboBatchEngine(model, config, ik_iterations=50, ik_batch_size=8)
        result['gpu_device'] = torch.cuda.get_device_name()
        try:
            for arm, bounds in BOUNDS.items():
                names = ARM_JOINT_NAMES[arm]
                seed = JointSolution(names, tuple(state[n] for n in names))
                targets = [adapter._local_fk[arm].pose({**state, names[0]: state[names[0]]+float(d)})[0]
                           for d in np.linspace(-.03, .03, 8)]
                groups = engine.solve_batch(arm, f'{arm}_grasp_tcp',
                    [p.tolist() for p in targets], [[seed]]*8, state, {})
                assert len(groups) == 8 and all(groups), [len(g) for g in groups]
                solver = engine._batch_ik_caches[(arm, f'{arm}_grasp_tcp', 8)][0][(arm, f'{arm}_grasp_tcp')]
                tensor_bounds = solver.kinematics.get_joint_limits().position.detach().cpu().numpy()
                yaw_index = solver.joint_names.index(names[0])
                assert np.allclose(tensor_bounds[:, yaw_index], bounds, atol=1e-7, rtol=0.)
                errors, yaw = [], []
                for group, target in zip(groups, targets, strict=True):
                    for solution in group:
                        value = solution.positions[0]
                        assert bounds[0] <= value <= bounds[1]
                        actual = adapter._local_fk[arm].pose(
                            {**state, **dict(zip(names, solution.positions, strict=True))})[0]
                        errors.append(float(np.linalg.norm(actual-target)))
                        yaw.append(value)
                assert max(errors) <= .01001
                result['arms'][arm].update(gpu_targets=8, gpu_solutions=len(yaw),
                    gpu_solver_bounds_rad=tensor_bounds[:, yaw_index].tolist(),
                    gpu_yaw_min_max_rad=[min(yaw), max(yaw)], gpu_fk_error_max_m=max(errors))
        finally:
            engine.close()
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
