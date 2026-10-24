#!/usr/bin/env python3
"""Record only the environment fields needed to reproduce the simulation."""
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess

ROOT = Path(__file__).resolve().parent

def command(*args: str) -> str:
    return subprocess.run(args, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, check=False).stdout.strip()

def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

record = {
    'host': platform.node(), 'platform': platform.platform(),
    'cpu': command('lscpu'), 'memory': command('free', '-b'),
    'container': 'cleany-ros-humble-cdi',
    'source_head': command('git', '-C', str(ROOT/'source'), 'rev-parse', 'HEAD'),
    'source_status': command('git', '-C', str(ROOT/'source'), 'status', '--short'),
    'teb_head': command('git', '-C', '/tmp/cleany-teb-humble', 'rev-parse', 'HEAD'),
    'teb_patch_sha256': sha(ROOT/'teb-compat.patch'),
    'costmap_converter_head': command('git', '-C', '/tmp/cleany-teb-humble/costmap_converter', 'rev-parse', 'HEAD'),
    'runner_sha256': sha(ROOT/'run_comparison.py'),
    'timing_source_sha256': sha(ROOT/'measurement_ws/src/cleany_benchmark/src/timed_controller.cpp'),
    'ros_environment': {k: v for k, v in os.environ.items()
                        if k.startswith(('ROS_', 'RMW_', 'AMENT_', 'COLCON_'))},
    'packages': command('dpkg-query', '-W', '-f=${binary:Package} ${Version}\n',
                        'ros-humble-nav2-controller', 'ros-humble-nav2-regulated-pure-pursuit-controller',
                        'ros-humble-nav2-mppi-controller', 'libignition-gazebo6', 'ros-humble-ros-gz-bridge'),
    'overlays': ['/opt/ros/humble/setup.bash',
                 str(ROOT/'cleany-data/builds/source/install/setup.bash'),
                 '/tmp/cleany-teb-humble/install/setup.bash',
                 str(ROOT/'measurement_ws/install/setup.bash')],
    'run_command': 'python3 run_comparison.py --candidate {teb,rpp} --output {candidate}_rtf1 --domain {222,221} --rtf 1',
    'runtime_variables': {'CLEANY_COMPARISON_REPO': str(ROOT/'source'), 'CLEANY_BENCHMARK_TIMING': '1',
                          'ROS_LOCALHOST_ONLY': '1', 'IGN_IP': '127.0.0.1', 'GZ_IP': '127.0.0.1'},
    'physics_step_s': .002, 'requested_rtf': 1,
    'compute_timing_scope': 'inner controller compute only; steady wall time; parallel scheduler effects included',
}
(ROOT/'environment.json').write_text(json.dumps(record, indent=2))
print(json.dumps({k: record[k] for k in ('host','source_head','teb_head','packages')}, indent=2))
