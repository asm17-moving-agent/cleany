"""실제 ROS 콜백, TF와 steady timer의 실패 경계를 검사합니다.

CLEANY_REACTIVE_PROFILE을 시뮬레이션 body YAML로 지정해 명시 실행합니다.
domain 225를 사용하고 clock/TF를 포함한 모든 시험 토픽을
/reactive_guard_fault_test 아래로 remap합니다. 시뮬레이터나 베이스 명령
브리지는 실행하지 않습니다.
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
