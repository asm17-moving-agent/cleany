"""Verify installed IK/planning optimizations without starting ROS or motors."""
from __future__ import annotations

import argparse
import hashlib
import importlib
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deployment', type=Path, required=True)
    parser.add_argument('--container', action='store_true')
    args = parser.parse_args()
    receipt = json.loads(args.deployment.read_text())
    expected = {row['path']: row['after_sha256'] for row in receipt['files']}
    records = []
    for name in ('hybrid_adapter', 'curobo_batch', 'physical_moveit_adapter',
                 'physical_grasp_selection_node', 'core.physical_grasp_selection'):
        module = importlib.import_module('cleany_skill_executor.'+name)
        path = Path(module.__file__).resolve()
        if args.container and not str(path).startswith('/opt/cleany-motion/workspace/install/'):
            raise RuntimeError('Module imported outside the prepared container: '+str(path))
        relative = 'ros2_ws/src/cleany_skill_executor/cleany_skill_executor/'+name.replace('.', '/')+'.py'
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest == expected[relative], relative
        records.append({'module': module.__name__, 'path': str(path), 'sha256': digest})
    share = Path(get_package_share_directory('cleany_skill_executor'))
    settings = {}
    for name in ('hybrid_motion.yaml', 'collection_motion.yaml'):
        path = share/'config'/name
        relative = 'ros2_ws/src/cleany_skill_executor/config/'+name
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected[relative], relative
        config = yaml.safe_load(path.read_text())['grasp_selection_server']['ros__parameters']
        required = {'approach_joint_shortcut_enabled': True, 'deduplicate_candidates': True}
        if name == 'hybrid_motion.yaml':
            required['curobo_ik_seeded_local_ik'] = True
        for key, value in required.items():
            assert config[key] == value, key
        settings[name] = required
    print(json.dumps({'environment': 'motion_container' if args.container else 'native',
        'modules': records, 'settings': settings, 'tested_source_hashes_match': True,
        'ros_nodes_started': False, 'motor_commands_sent': False}, indent=2))


if __name__ == '__main__':
    main()
