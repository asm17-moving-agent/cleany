"""Read host command progress while ROS runtime startup is still in progress."""

import json
from pathlib import Path


def read_request(path: str) -> dict:
    with Path(path).expanduser().open() as stream:
        data = json.loads(stream.read(32769))
    if (not isinstance(data, dict) or data.get('schema') != 1
            or not isinstance(data.get('execution_id'), str) or not data['execution_id']
            or data.get('state') not in ('ACTIVE', 'FINISHED', 'FAILED')
            or not isinstance(data.get('phase'), str)
            or type(data.get('owner_pid')) is not int or data['owner_pid'] <= 0
            or not isinstance(data.get('owner_start'), str)
            or type(data.get('started_at_ns')) is not int or data['started_at_ns'] <= 0
            or type(data.get('updated_at_ns')) is not int or data['updated_at_ns'] <= 0
            or not isinstance(data.get('history'), list) or len(data['history']) > 16):
        raise ValueError('Invalid operator command observation')
    for entry in data['history']:
        if not isinstance(entry, dict) or not isinstance(entry.get('phase'), str) or type(entry.get('at_ns')) is not int:
            raise ValueError('Invalid command phase history')
    try:
        fields = Path(f"/proc/{data['owner_pid']}/stat").read_text().rsplit(')', 1)[1].split()
        data['owner_alive'] = bool(data['owner_start']) and fields[19] == data['owner_start'] and fields[0] not in ('Z', 'X')
    except (OSError, IndexError):
        data['owner_alive'] = False
    return data
