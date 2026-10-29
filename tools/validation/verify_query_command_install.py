"""Check deployed RPC/command modules and native Jetson flags without robot nodes."""
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
    root = Path(importlib.import_module('cleany_skill_executor').__file__).parent
    prefix = 'ros2_ws/src/cleany_skill_executor/cleany_skill_executor/'
    files = []
    for row in receipt['files']:
        if not row['path'].startswith(prefix): continue
        path = root / row['path'][len(prefix):]
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest == row['after_sha256'], row['path']
        if args.container:
            assert str(path.resolve()).startswith('/opt/cleany-motion/workspace/install/')
        else:
            assert str(path.absolute()).startswith(('/home/cleany/cleany-data/builds/cleany/install/',
                                                    '/home/cleany/cleany-data/builds/cleany/build/'))
            assert str(path.resolve()).startswith('/home/cleany/cleany/ros2_ws/src/')
        files.append(dict(path=str(path), resolved=str(path.resolve()), sha256=digest))
    assert len(files) == 4
    settings = None
    if not args.container:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('cleany_manipulation_bt'))/'config/manipulation_jetson.yaml'
        row = next(row for row in receipt['files'] if row['path'].endswith('/config/manipulation_jetson.yaml'))
        assert hashlib.sha256(path.read_bytes()).hexdigest() == row['after_sha256']
        config = yaml.safe_load(path.read_text())['/**']['ros__parameters']
        settings = {key: config[key] for key in ('hardware_parallel_model_queries', 'hardware_sparse_cubic_commands')}
        assert all(value is True for value in settings.values())
    from cleany_skill_executor.core.hardware_execution import prepare_bounded_path
    from cleany_skill_executor.readonly_services import call_readonly_services
    from cleany_skill_executor.trajectory_io import command_samples_to_message
    assert all(callable(value) for value in (prepare_bounded_path, call_readonly_services, command_samples_to_message))
    print(json.dumps(dict(environment='container' if args.container else 'native',
                         hashes_match=True, runtime_files=files, native_execution_settings=settings,
                         motor_commands_sent=False, ros_nodes_started=False), indent=2))


if __name__ == '__main__':
    main()
