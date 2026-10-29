"""Check installed selection/return source and settings without starting ROS nodes."""
import argparse
import hashlib
import importlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deployment', type=Path, required=True)
    parser.add_argument('--container', action='store_true')
    args = parser.parse_args()
    receipt = json.loads(args.deployment.read_text())
    package = 'cleany_skill_executor'
    root = Path(importlib.import_module(package).__file__).parent
    prefix = f'ros2_ws/src/{package}/{package}/'
    checked = []
    for row in receipt['files']:
        if not row['path'].startswith(prefix):
            continue
        path = root / row['path'][len(prefix):]
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest == row['after_sha256'], row['path']
        if args.container:
            assert str(path.resolve()).startswith('/opt/cleany-motion/workspace/install/'), str(path)
        else:
            # Native colcon --symlink-install imports its build package and
            # resolves individual modules back to the deployed source tree.
            expected_roots = ('/home/cleany/cleany-data/builds/cleany/install/',
                              '/home/cleany/cleany-data/builds/cleany/build/')
            assert str(path.absolute()).startswith(expected_roots), str(path)
            assert str(path.resolve()).startswith('/home/cleany/cleany/ros2_ws/src/'), str(path)
        checked.append(dict(path=str(path.resolve()), sha256=digest))
    assert len(checked) == 6, 'Incomplete installed-source check'
    from ament_index_python.packages import get_package_share_directory
    import yaml
    config_path = Path(get_package_share_directory(package)) / 'config/hybrid_motion.yaml'
    row = next(row for row in receipt['files'] if row['path'].endswith('/config/hybrid_motion.yaml'))
    assert hashlib.sha256(config_path.read_bytes()).hexdigest() == row['after_sha256']
    config = yaml.safe_load(config_path.read_text())['grasp_selection_server']['ros__parameters']
    settings = dict(fast_candidate_paths_first=True, grasp_ik_first=True,
                    stop_after_first_feasible=True, curobo_ik_vlm_warmup_arms=['right', 'left'])
    for key, expected in settings.items():
        assert config[key] == expected, key
    from cleany_skill_executor.core.physical_grasp_selection import GraspSelectionConfig
    GraspSelectionConfig(fast_candidate_paths_first=True, grasp_ik_first=True,
                         stop_after_first_feasible=True)
    print(json.dumps(dict(environment='container' if args.container else 'native',
                         installed_files=checked, config_path=str(config_path), settings=settings,
                         hashes_match=True, motor_commands_sent=False,
                         ros_nodes_started=False), indent=2))


if __name__ == '__main__':
    main()
