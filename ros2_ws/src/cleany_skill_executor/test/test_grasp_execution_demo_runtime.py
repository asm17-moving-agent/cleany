from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from uuid import uuid4

import pytest


if os.environ.get('ROS_DISTRO') is None:
    pytest.skip('ROS 2 environment is not active', allow_module_level=True)


def _log_text(path: Path) -> str:
    try:
        return path.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return ''


def _stop_launch(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.0)


def test_selected_grasp_is_executed_in_mujoco() -> None:
    with tempfile.TemporaryDirectory(
        prefix='cleany_grasp_execution_demo_test_'
    ) as temporary_directory:
        temporary_root = Path(temporary_directory)
        log_path = temporary_root / 'launch.log'
        domain_id = 20 + uuid4().int % 180
        environment = os.environ.copy()
        environment.update(
            {
                'ROS_DOMAIN_ID': str(domain_id),
                'ROS_HOME': str(temporary_root / 'ros_home'),
                'ROS_LOG_DIR': str(temporary_root / 'ros_log'),
            }
        )
        with log_path.open('wb') as launch_log:
            process = subprocess.Popen(
                [
                    'ros2',
                    'launch',
                    'cleany_skill_executor',
                    'grasp_execution_demo.launch.py',
                    'headless:=true',
                    'use_rviz:=false',
                    'demo_start_delay_sec:=0.1',
                    'stage_hold_sec:=0.1',
                ],
                env=environment,
                cwd=temporary_root,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                deadline = time.monotonic() + 120.0
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        pytest.fail(
                            'grasp execution launch exited early with code '
                            f'{process.returncode}\n{_log_text(log_path)[-12000:]}'
                        )
                    text = _log_text(log_path)
                    if 'GRASP EXECUTION DEMO FAILED' in text:
                        pytest.fail(
                            'grasp execution demo reported failure\n'
                            f'{text[-12000:]}'
                        )
                    if 'DEMO COMPLETE:' in text:
                        break
                    time.sleep(0.1)
                else:
                    pytest.fail(
                        'timed out waiting for grasp execution completion\n'
                        f'{_log_text(log_path)[-12000:]}'
                    )
            finally:
                _stop_launch(process)

        text = _log_text(log_path)
        assert 'candidate=0 arm=left stage=PREGRASP_IK: no IK solution' in text
        assert 'candidate=0 arm=right stage=PREGRASP_IK: no IK solution' in text
        assert 'Selected candidate=1 arm=left' in text
        assert 'MoveIt execution succeeded: pre-grasp' in text
        assert 'MoveIt execution succeeded: grasp' in text
        assert 'trajectory execution and joint feedback both succeeded' in text
        assert 'GRASP EXECUTION DEMO FAILED' not in text
