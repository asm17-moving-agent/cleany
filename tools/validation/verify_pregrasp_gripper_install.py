"""Check installed optimization files and settings without starting robot nodes."""
import argparse
import ast
import hashlib
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cleany_skill_executor
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deployment', type=Path, required=True)
    parser.add_argument('--container', action='store_true')
    args = parser.parse_args()
    receipt = json.loads(args.deployment.read_text())
    root = Path(cleany_skill_executor.__file__).parent
    prefix = 'ros2_ws/src/cleany_skill_executor/cleany_skill_executor/'
    rows = []
    for row in receipt['files']:
        if not row['path'].startswith(prefix):
            continue
        path = root/row['path'][len(prefix):]
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest == row['after_sha256'], row['path']
        if args.container:
            assert str(path.resolve()).startswith('/opt/cleany-motion/workspace/install/')
        rows.append(dict(path=str(path.resolve()), sha256=digest))
    settings = {}
    if not args.container:
        path = Path(get_package_share_directory('cleany_manipulation_bt'))/'config/manipulation_jetson.yaml'
        config = yaml.safe_load(path.read_text())['/**']['ros__parameters']
        for name, value in {'hardware_start_delay_sec': .3,
                            'hardware_state_validity_batch_size': 64,
                            'hardware_return_start_delay_sec': .3}.items():
            assert config[name] == value, name
            settings[name] = value
        tree = ast.parse((root/'hardware_execution.py').read_text())
        for name, expected_value in {'hardware_velocity_rad_s': .8,
                                     'hardware_acceleration_rad_s2': 1.2}.items():
            defaults = [ast.literal_eval(value) for node in ast.walk(tree)
                        if isinstance(node, ast.Dict) for key, value in zip(node.keys, node.values)
                        if isinstance(key, ast.Constant) and key.value == name]
            assert len(defaults) == 1
            settings[name] = config.get(name, defaults[0])
            assert settings[name] == expected_value
        expected = next(row for row in receipt['files'] if row['path'].endswith('manipulation_jetson.yaml'))
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected['after_sha256']
    result = dict(environment='motion_container' if args.container else 'native',
                  installed_files=rows, settings=settings, motor_commands_sent=False,
                  ros_nodes_started=False, source_hashes_match_tested_deployment=True)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
