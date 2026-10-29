"""Verify installed source identity and connection reuse without ROS nodes."""
import argparse
import hashlib
import importlib
import json
from pathlib import Path
from types import SimpleNamespace as NS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deployment', type=Path, required=True)
    parser.add_argument('--container', action='store_true')
    args = parser.parse_args()
    receipt = json.loads(args.deployment.read_text())
    checked = []
    for row in receipt['files']:
        parts = Path(row['path']).parts
        if len(parts) < 5 or parts[:2] != ('ros2_ws', 'src') or parts[2] != parts[3]:
            continue
        package = parts[2]
        if args.container and package != 'cleany_skill_executor':
            continue
        root = Path(importlib.import_module(package).__file__).parent
        path = root.joinpath(*parts[4:])
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest == row['after_sha256'], row['path']
        if args.container:
            assert str(path.resolve()).startswith('/opt/cleany-motion/workspace/install/')
        checked.append(dict(path=str(path.resolve()), sha256=digest))
    assert len(checked) == (2 if args.container else 4), 'Incomplete installed-source check'
    settings = {}
    smoke = None
    if not args.container:
        from ament_index_python.packages import get_package_share_directory
        import yaml
        from cleany_manipulation_bt.action_tracking import MotionClientPool
        owner = NS(check_execution=lambda: None, execution_context=None, submissions=[], handles=[])
        created, destroyed = [], []
        def factory(kind, name):
            created.append(name)
            return NS(destroy=lambda: destroyed.append(name))
        pool = MotionClientPool(owner, factory)
        first = pool.acquire(object, '/offline')
        first.destroy()
        second = pool.acquire(object, '/offline')
        assert second.transport_reused and second.client is first.client
        second.destroy()
        pool.close()
        assert created == destroyed == ['/offline']
        smoke = True
        config = yaml.safe_load((Path(get_package_share_directory('cleany_manipulation_bt'))/
                                'config/manipulation_jetson.yaml').read_text())['/**']['ros__parameters']
        for key, expected in dict(execution_defer_pregrasp_validation=True,
                                  hardware_reuse_settle_history=True,
                                  hardware_start_delay_sec=.3,
                                  hardware_return_start_delay_sec=.3,
                                  hardware_state_validity_batch_size=64).items():
            assert config[key] == expected
            settings[key] = expected
        settings['hardware_state_preparation_monitor_period_sec'] = config.get(
            'hardware_state_preparation_monitor_period_sec', .01)
        assert settings['hardware_state_preparation_monitor_period_sec'] == .01
    print(json.dumps(dict(environment='container' if args.container else 'native',
                         installed_files=checked, settings=settings, pool_smoke_passed=smoke,
                         hashes_match=True, motor_commands_sent=False, ros_nodes_started=False), indent=2))


if __name__ == '__main__':
    main()
