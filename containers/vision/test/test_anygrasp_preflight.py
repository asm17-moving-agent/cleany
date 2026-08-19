from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'anygrasp_preflight.py'
SPEC = importlib.util.spec_from_file_location('anygrasp_preflight', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
preflight = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = preflight
SPEC.loader.exec_module(preflight)


def identity_values() -> dict[str, str]:
    return {
        'ANYGRASP_MAC_ADDRESS': '02:42:ac:1e:00:0a',
        'ANYGRASP_IPV4_ADDRESS': '172.30.0.10',
        'ANYGRASP_EXPECTED_FEATURE_ID': 'N11176336906968411287',
        'PERCEPTION_IPV4_ADDRESS': '172.30.0.11',
        'VLM_IPV4_ADDRESS': '172.30.0.12',
        'MOTION_IPV4_ADDRESS': '172.30.0.13',
        'HYBRID_SUBNET': '172.30.0.0/24',
        'HYBRID_GATEWAY': '172.30.0.1',
        'ROS_DOMAIN_ID': '0',
        'ANYGRASP_LICENSE_HOST_DIR': '/var/lib/cleany/anygrasp/license',
        'ANYGRASP_LICENSE_FILE': 'Cleany.lic',
        'ANYGRASP_MODEL_HOST_DIR': '/var/lib/cleany/models/anygrasp',
        'ANYGRASP_CHECKPOINT_PATH': (
            '/models/anygrasp/checkpoint_detection.tar'
        ),
        'SAM2_MODEL_HOST_DIR': '/var/lib/cleany/models/sam2',
        'SAM2_MODEL_CONFIG': 'configs/sam2.1/sam2.1_hiera_s.yaml',
        'SAM2_CHECKPOINT_PATH': '/models/sam2/sam2.1_hiera_small.pt',
    }


def write_identity(path: Path, values: dict[str, str]) -> None:
    path.write_text(
        ''.join(f'{key}={value}\n' for key, value in values.items()),
        encoding='utf-8',
    )
    path.chmod(0o644)


def test_valid_identity_is_strictly_parsed(tmp_path: Path) -> None:
    path = tmp_path / 'identity.env'
    values = identity_values()
    write_identity(path, values)

    assert preflight.validate_identity_file(path, require_root=False) == values


@pytest.mark.parametrize(
    ('key', 'value'),
    (
        ('ANYGRASP_MAC_ADDRESS', '02:42:ac:1e:00:0b'),
        ('ANYGRASP_IPV4_ADDRESS', '172.30.0.99'),
        ('ANYGRASP_EXPECTED_FEATURE_ID', 'N00000000000000000000'),
        ('PERCEPTION_IPV4_ADDRESS', '172.30.0.12'),
    ),
)
def test_migration_controlled_identity_change_is_rejected(
    tmp_path: Path,
    key: str,
    value: str,
) -> None:
    path = tmp_path / 'identity.env'
    values = identity_values()
    values[key] = value
    write_identity(path, values)

    with pytest.raises(preflight.PreflightError, match='migration-controlled'):
        preflight.validate_identity_file(path, require_root=False)


def test_missing_or_unknown_identity_key_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / 'identity.env'
    values = identity_values()
    del values['ROS_DOMAIN_ID']
    values['UNREVIEWED_OVERRIDE'] = 'true'
    write_identity(path, values)

    with pytest.raises(
        preflight.PreflightError,
        match='Missing Jetson identity keys',
    ):
        preflight.validate_identity_file(path, require_root=False)


def test_writable_or_symlink_identity_file_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / 'identity.env'
    write_identity(path, identity_values())
    path.chmod(0o664)
    with pytest.raises(preflight.PreflightError, match='group/other writable'):
        preflight.validate_identity_file(path, require_root=False)

    path.chmod(0o644)
    symlink = tmp_path / 'identity-link.env'
    symlink.symlink_to(path)
    with pytest.raises(preflight.PreflightError, match='regular file'):
        preflight.validate_identity_file(symlink, require_root=False)


def test_environment_override_is_rejected() -> None:
    values = identity_values()
    environment = dict(values)
    environment['ANYGRASP_MAC_ADDRESS'] = '02:42:ac:1e:00:0b'

    with pytest.raises(preflight.PreflightError, match='environment override'):
        preflight.validate_environment(values, environment)


def test_sdk_feature_id_mismatch_is_rejected() -> None:
    with pytest.raises(preflight.PreflightError, match='SDK feature ID'):
        preflight.validate_feature_id(
            'N00000000000000000000',
            'N11176336906968411287',
        )


def test_exact_eth0_identity_is_required() -> None:
    valid = preflight.NetworkInterface(
        'eth0',
        '02:42:ac:1e:00:0a',
        ('172.30.0.10',),
    )
    preflight.validate_interfaces(
        (
            preflight.NetworkInterface(
                'lo',
                '00:00:00:00:00:00',
                ('127.0.0.1',),
            ),
            valid,
        ),
        expected_mac='02:42:ac:1e:00:0a',
        expected_ipv4='172.30.0.10',
    )

    with pytest.raises(preflight.PreflightError, match='exactly one'):
        preflight.validate_interfaces(
            (
                valid,
                preflight.NetworkInterface(
                    'eth1',
                    '02:42:ac:1e:00:0b',
                    (),
                ),
            ),
            expected_mac='02:42:ac:1e:00:0a',
            expected_ipv4='172.30.0.10',
        )


@pytest.mark.parametrize(
    'interface',
    (
        preflight.NetworkInterface(
            'eth0',
            '02:42:ac:1e:00:0b',
            ('172.30.0.10',),
        ),
        preflight.NetworkInterface(
            'eth0',
            '02:42:ac:1e:00:0a',
            ('172.30.0.99',),
        ),
    ),
)
def test_mac_or_ip_override_is_rejected(interface) -> None:
    with pytest.raises(preflight.PreflightError, match='Unexpected eth0'):
        preflight.validate_interfaces(
            (interface,),
            expected_mac='02:42:ac:1e:00:0a',
            expected_ipv4='172.30.0.10',
        )


def test_license_and_model_mounts_must_be_read_only() -> None:
    mounts = preflight.parse_mountinfo(
        '1 0 0:1 / / rw,relatime - ext4 root rw\n'
        '2 1 0:2 / /opt/anygrasp/license ro,relatime - ext4 license ro\n'
        '3 1 0:3 / /models/anygrasp ro,relatime - ext4 models ro\n'
    )
    preflight.validate_read_only_mount(
        Path('/opt/anygrasp/license'),
        mounts,
    )
    preflight.validate_read_only_mount(
        Path('/models/anygrasp/checkpoint_detection.tar'), mounts
    )

    with pytest.raises(
        preflight.PreflightError,
        match='not on a read-only mount',
    ):
        preflight.validate_read_only_mount(
            Path('/models/anygrasp/checkpoint_detection.tar'),
            preflight.parse_mountinfo(
                '1 0 0:1 / / rw,relatime - ext4 root rw\n'
                '2 1 0:2 / /models/anygrasp rw,relatime - ext4 models rw\n'
            ),
        )
