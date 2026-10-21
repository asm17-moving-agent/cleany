import json
import os
from pathlib import Path

import pytest

from cleany_dev_monitor.request import read_request


def payload():
    return dict(schema=1, execution_id='request-one', state='ACTIVE', phase='INSPECTING_SCENE',
        started_at_ns=1, updated_at_ns=1,
        owner_pid=os.getpid(), owner_start=Path(f'/proc/{os.getpid()}/stat').read_text().rsplit(')', 1)[1].split()[19],
        history=[dict(phase='INSPECTING_SCENE', at_ns=1)])


def test_pid_reuse_and_missing_owner_cannot_look_like_live_preparation(tmp_path):
    path = tmp_path / 'request.json'
    data = payload()
    path.write_text(json.dumps(data))
    assert read_request(str(path))['owner_alive']
    data['owner_start'] = 'wrong-start-time'
    path.write_text(json.dumps(data))
    assert not read_request(str(path))['owner_alive']
    data['owner_pid'] = 2147483647
    path.write_text(json.dumps(data))
    assert not read_request(str(path))['owner_alive']


@pytest.mark.parametrize('changes', [{'schema':2}, {'state':'SUCCESS'}, {'history':[{}]}, {'owner_pid':-1}])
def test_invalid_command_observations_are_rejected(tmp_path, changes):
    path = tmp_path / 'request.json'
    path.write_text(json.dumps({**payload(), **changes}))
    with pytest.raises(ValueError):
        read_request(str(path))
