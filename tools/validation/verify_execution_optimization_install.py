"""Verify installed planner wiring and execution settings without starting ROS."""
from __future__ import annotations

import argparse
import hashlib
import importlib
import inspect
import json
from pathlib import Path
from types import SimpleNamespace


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deployment', type=Path, required=True)
    parser.add_argument('--container', action='store_true')
    args = parser.parse_args()
    receipt = json.loads(args.deployment.read_text())
    expected = {row['path']: row['after_sha256'] for row in receipt['files']}
    modules = ['planning_scene', 'core.physical_grasp_selection', 'physical_moveit_adapter',
               'hybrid_adapter', 'curobo_adapter']
    if not args.container:
        modules.extend(['collection_pipeline', 'hardware_execution'])
    records = []
    for name in modules:
        module = importlib.import_module('cleany_skill_executor.'+name)
        path = Path(module.__file__).resolve()
        if args.container and not str(path).startswith('/opt/cleany-motion/workspace/install/'):
            raise RuntimeError('Planner imported outside the prepared container: '+str(path))
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        relative = ('ros2_ws/src/cleany_skill_executor/cleany_skill_executor/'
                    + name.replace('.', '/')+'.py')
        assert digest == expected[relative], relative
        records.append({'module': module.__name__, 'installed_path': str(path), 'sha256': digest})

    from ament_index_python.packages import get_package_share_directory
    import yaml
    from cleany_skill_executor.planning_scene import SceneAwarePort
    from cleany_skill_executor.core.physical_grasp_selection import GraspSelector
    from cleany_skill_executor.physical_moveit_adapter import MoveItGraspAdapter

    calls = []
    adapter = SimpleNamespace(plan_approach=lambda *values: calls.append(values) or 'validated')
    wrapper = SceneAwarePort(adapter, object())
    assert wrapper.plan_approach('right', 'goal', 'start', 'pre_pose', 'goal_pose') == 'validated'
    assert calls == [('right', 'goal', 'start', 'pre_pose', 'goal_pose')]
    assert SceneAwarePort(object(), object()).plan_approach is None
    selection = inspect.getsource(GraspSelector.select)
    assert selection.index('EvaluationStage.PLAN_GRASP') < selection.index('EvaluationStage.PLAN_PREGRASP')
    approach = inspect.getsource(MoveItGraspAdapter.plan_approach)
    assert 'corridor=(start_pose, target_pose)' in approach
    assert 'validate_approach_path(' not in approach

    motion_path = Path(get_package_share_directory('cleany_skill_executor'))/'config/collection_motion.yaml'
    motion = yaml.safe_load(motion_path.read_text())['grasp_selection_server']['ros__parameters']
    assert motion['cartesian_approach_first'] is True
    settings = {'cartesian_approach_first': motion['cartesian_approach_first']}
    if not args.container:
        config_path = Path(get_package_share_directory('cleany_manipulation_bt'))/'config/manipulation_jetson.yaml'
        relative = 'ros2_ws/src/cleany_manipulation_bt/config/manipulation_jetson.yaml'
        assert hashlib.sha256(config_path.read_bytes()).hexdigest() == expected[relative]
        config = yaml.safe_load(config_path.read_text())['/**']['ros__parameters']
        required = {'hardware_return_start_delay_sec': .3,
                    'hardware_release_retreat_min_duration_sec': .25,
                    'hardware_release_retreat_position_tolerance_m': .005,
                    'hardware_release_retreat_orientation_tolerance_deg': 2.}
        for name, value in required.items():
            assert config[name] == value, name
        settings.update(required)
    print(json.dumps({'environment': 'motion_container' if args.container else 'native',
                      'installed_modules': records, 'settings': settings,
                      'scene_wrapper_forwards_approach': True,
                      'approach_before_pregrasp': True,
                      'rrt_final_cubic_combines_corridor_validation': True,
                      'motor_commands_sent': False, 'ros_nodes_started': False,
                      'source_hashes_match_tested_deployment': True}, indent=2))


if __name__ == '__main__':
    main()
