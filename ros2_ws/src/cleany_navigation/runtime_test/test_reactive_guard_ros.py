"""Actual ROS callbacks, TF and steady timer failure boundaries.

Run explicitly with CLEANY_REACTIVE_PROFILE pointing to the simulated body YAML.
Uses domain 225 and remaps every test topic, including clock/TF, under
/reactive_guard_fault_test. No simulator or base command bridge is launched.
"""
from pathlib import Path
import json
import os
import subprocess
import sys

import pytest


def test_reactive_transport_and_clock_fail_closed(tmp_path):
    profile = os.environ.get('CLEANY_REACTIVE_PROFILE')
    if not profile:
        pytest.skip('Set CLEANY_REACTIVE_PROFILE to an installed simulation body profile')
    env = dict(os.environ, ROS_DOMAIN_ID='225', ROS_LOCALHOST_ONLY='1',
               CLEANY_REACTIVE_EVIDENCE=str(tmp_path))
    result = subprocess.run([sys.executable, str(Path(__file__).with_name('reactive_fault_probe.py'))],
                            env=env, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout+result.stderr
    cases = json.loads((tmp_path/'fault-result.json').read_text())
    assert len(cases) == 27
    assert all(case['pass_'] for case in cases)
