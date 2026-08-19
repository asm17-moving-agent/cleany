from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest


VISION_DIR = Path(__file__).parents[1]
COMPOSE_FILE = VISION_DIR / 'compose.yaml'
DOCKERFILE = VISION_DIR / 'Dockerfile'
IDENTITY_EXAMPLE = VISION_DIR / 'jetson-identity.env.example'
HOST_CYCLONEDDS = VISION_DIR / 'cyclonedds' / 'host.xml'
CONTAINER_CYCLONEDDS = VISION_DIR / 'cyclonedds' / 'container.xml'


def test_dockerfile_uses_pinned_jetson_l4t_base() -> None:
    first_line = DOCKERFILE.read_text(encoding='utf-8').splitlines()[0]

    assert first_line.startswith(
        'FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0@sha256:'
    )
    assert 'nvidia/cuda' not in first_line


def test_host_cyclonedds_uses_the_fixed_bridge_gateway() -> None:
    configuration = HOST_CYCLONEDDS.read_text(encoding='utf-8')

    assert '<NetworkInterface address="172.30.0.1"' in configuration
    assert '<AllowMulticast>false</AllowMulticast>' in configuration
    assert '<ParticipantIndex>auto</ParticipantIndex>' in configuration
    assert 'autodetermine=' not in configuration


def test_container_cyclonedds_uses_well_known_unicast_ports() -> None:
    configuration = CONTAINER_CYCLONEDDS.read_text(encoding='utf-8')

    assert '<AllowMulticast>false</AllowMulticast>' in configuration
    assert '<ParticipantIndex>auto</ParticipantIndex>' in configuration


def _docker_compose_available() -> bool:
    if shutil.which('docker') is None:
        return False
    result = subprocess.run(
        ['docker', 'compose', 'version'],
        capture_output=True,
        text=True,
        check=False,
    )
    return result.returncode == 0


def _render_compose(identity_file: Path) -> subprocess.CompletedProcess[str]:
    identity_keys = {
        line.split('=', 1)[0]
        for line in IDENTITY_EXAMPLE.read_text(encoding='utf-8').splitlines()
        if line and not line.startswith('#')
    }
    environment = os.environ.copy()
    for key in identity_keys | {'CLEANY_JETSON_IDENTITY_FILE'}:
        environment.pop(key, None)
    environment['CLEANY_JETSON_IDENTITY_FILE'] = str(identity_file)
    return subprocess.run(
        [
            'docker',
            'compose',
            '--project-directory',
            str(VISION_DIR),
            '--env-file',
            str(identity_file),
            '-f',
            str(COMPOSE_FILE),
            'config',
            '--format',
            'json',
        ],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )


@pytest.mark.skipif(
    not _docker_compose_available(),
    reason='Docker Compose is unavailable',
)
def test_compose_separates_anygrasp_assets_and_reserves_gpu_addresses(
    tmp_path: Path,
) -> None:
    identity_file = tmp_path / 'identity.env'
    shutil.copyfile(IDENTITY_EXAMPLE, identity_file)

    result = _render_compose(identity_file)

    assert result.returncode == 0, result.stderr
    configuration = json.loads(result.stdout)
    assert set(configuration['services']) == {'anygrasp', 'perception'}
    anygrasp = configuration['services']['anygrasp']
    perception = configuration['services']['perception']
    network_name = next(iter(anygrasp['networks']))
    anygrasp_network = anygrasp['networks'][network_name]
    perception_network = perception['networks'][network_name]
    assert anygrasp_network['ipv4_address'] == '172.30.0.10'
    assert anygrasp_network['mac_address'] == '02:42:ac:1e:00:0a'
    assert perception_network['ipv4_address'] == '172.30.0.11'
    assert anygrasp.get('network_mode') != 'host'

    anygrasp_volumes = {
        volume['target']: volume for volume in anygrasp['volumes']
    }
    perception_volumes = {
        volume['target']: volume for volume in perception['volumes']
    }
    assert anygrasp_volumes['/opt/anygrasp/license']['read_only'] is True
    assert anygrasp_volumes['/models/anygrasp']['read_only'] is True
    assert '/opt/anygrasp/license' not in perception_volumes
    assert '/models/anygrasp' not in perception_volumes
    assert perception_volumes['/models/sam2']['read_only'] is True
    assert 'ANYGRASP_EXPECTED_FEATURE_ID' not in perception['environment']

    network = configuration['networks'][network_name]['ipam']['config'][0]
    assert network['aux_addresses'] == {
        'motion': '172.30.0.13',
        'vlm': '172.30.0.12',
    }


@pytest.mark.skipif(
    not _docker_compose_available(),
    reason='Docker Compose is unavailable',
)
def test_compose_has_no_identity_defaults_and_rejects_missing_value(
    tmp_path: Path,
) -> None:
    assert '${' in COMPOSE_FILE.read_text(encoding='utf-8')
    assert ':-' not in COMPOSE_FILE.read_text(encoding='utf-8')
    identity_file = tmp_path / 'identity.env'
    identity_file.write_text(
        '\n'.join(
            line
            for line in IDENTITY_EXAMPLE.read_text(
                encoding='utf-8'
            ).splitlines()
            if not line.startswith('ANYGRASP_EXPECTED_FEATURE_ID=')
        ),
        encoding='utf-8',
    )

    result = _render_compose(identity_file)

    assert result.returncode != 0
    assert 'ANYGRASP_EXPECTED_FEATURE_ID' in result.stderr
