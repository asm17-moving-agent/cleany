"""Select matching middleware before importing ROS nodes."""
import pytest
import json
import os
from pathlib import Path
import subprocess
from shared_perception_transport import shared_perception_rmw


def test_managed_perception_keeps_existing_transport():
    assert shared_perception_rmw({}) == ''
    assert shared_perception_rmw({'CLEANY_COLLECTION_USE_EXISTING_PERCEPTION': 'false'}) == ''


def test_shared_native_provider_defaults_to_fastdds():
    assert shared_perception_rmw({'CLEANY_COLLECTION_USE_EXISTING_PERCEPTION': 'true'}) == 'rmw_fastrtps_cpp'


@pytest.mark.parametrize('implementation', ['rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp'])
def test_explicit_shared_provider_transport(implementation):
    assert shared_perception_rmw({'CLEANY_COLLECTION_USE_EXISTING_PERCEPTION': 'true',
        'CLEANY_COLLECTION_SHARED_RMW_IMPLEMENTATION': implementation}) == implementation


def test_unknown_shared_provider_transport_is_rejected():
    with pytest.raises(ValueError):
        shared_perception_rmw({'CLEANY_COLLECTION_USE_EXISTING_PERCEPTION': 'true',
            'CLEANY_COLLECTION_SHARED_RMW_IMPLEMENTATION': 'unsupported'})


@pytest.mark.parametrize('job', [False, True])
def test_web_and_job_process_resolve_the_same_saved_transport(tmp_path, job):
    repo = Path(__file__).resolve().parents[2]
    settings = tmp_path/'runtime-settings.json'
    settings.write_text(json.dumps({'CLEANY_COLLECTION_USE_EXISTING_PERCEPTION': 'true',
        'GEMINI_API_KEY': 'private-test-value'}))
    settings.chmod(0o600)
    args = [str(tmp_path/'jobs'/'request.json'), '--job'] if job else [str(tmp_path)]
    env = dict(os.environ, PYTHONPATH=str(repo/'ros2_ws/src/cleany_dev_monitor'))
    result = subprocess.run(['python3', str(repo/'tools/shared_perception_transport.py'), *args],
        env=env, capture_output=True, text=True, check=True)
    assert result.stdout == 'rmw_fastrtps_cpp\n'
    assert 'private-test-value' not in result.stdout+result.stderr
