"""Select the commissioned shared provider's ROS transport without initializing ROS."""
from collections.abc import Mapping
from pathlib import Path

SUPPORTED_RMW = ('rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp')


def shared_perception_rmw(settings: Mapping[str, str]) -> str:
    if settings.get('CLEANY_COLLECTION_USE_EXISTING_PERCEPTION') != 'true':
        return ''
    implementation = settings.get('CLEANY_COLLECTION_SHARED_RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    if implementation not in SUPPORTED_RMW:
        raise ValueError('Unsupported shared perception RMW implementation')
    return implementation


if __name__ == '__main__':
    import argparse
    import os
    from cleany_dev_monitor.operator_jobs import runtime_settings

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operator_directory', type=Path)
    parser.add_argument('--job', action='store_true', help='Resolve settings from a job JSON path')
    args = parser.parse_args()
    root = args.operator_directory.parent.parent if args.job else args.operator_directory
    print(shared_perception_rmw({**os.environ, **runtime_settings(root)}))
