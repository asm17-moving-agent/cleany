"""Verify installed return handoff modules and flags without ROS nodes or motion."""
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
    checked = []
    packages = ('cleany_skill_executor',) if args.container else ('cleany_skill_executor', 'cleany_manipulation_bt')
    for package in packages:
        prefix = f'ros2_ws/src/{package}/{package}/'
        for row in receipt['files']:
            if not row['path'].startswith(prefix): continue
            module = package+'.'+row['path'][len(prefix):-3].replace('/', '.')
            path = Path(importlib.import_module(module).__file__)
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            assert digest == row['after_sha256'], row['path']
            if args.container:
                assert str(path.resolve()).startswith('/opt/cleany-motion/workspace/install/'), str(path)
            else:
                assert str(path.absolute()).startswith(('/home/cleany/cleany-data/builds/cleany/install/',
                                                        '/home/cleany/cleany-data/builds/cleany/build/')), str(path)
            checked.append(dict(module=module, path=str(path), resolved=str(path.resolve()), sha256=digest))
    assert len(checked) == (3 if args.container else 5)
    settings = None
    if not args.container:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('cleany_manipulation_bt'))/'config/manipulation_jetson.yaml'
        row = next(row for row in receipt['files'] if row['path'].endswith('/config/manipulation_jetson.yaml'))
        assert hashlib.sha256(path.read_bytes()).hexdigest() == row['after_sha256']
        config = yaml.safe_load(path.read_text())['/**']['ros__parameters']
        settings = {key: config[key] for key in ('hardware_share_return_preparation',
                    'hardware_parallel_model_queries', 'hardware_sparse_cubic_commands')}
        assert all(value is True for value in settings.values())
    print(json.dumps(dict(environment='container' if args.container else 'native',
        hashes_match=True, runtime_files=checked, native_execution_settings=settings,
        motor_commands_sent=False, ros_nodes_started=False), indent=2))


if __name__ == '__main__':
    main()
