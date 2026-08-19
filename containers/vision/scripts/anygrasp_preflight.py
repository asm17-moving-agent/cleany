#!/usr/bin/env python3
"""Fail-closed runtime identity checks for the licensed AnyGrasp service."""

from __future__ import annotations

import argparse
import ipaddress
import json
import os
import re
import stat
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence


IDENTITY_PATH = Path('/etc/cleany/jetson-identity.env')
PINNED_ANYGRASP_MAC = '02:42:ac:1e:00:0a'
PINNED_ANYGRASP_IP = '172.30.0.10'
PINNED_PERCEPTION_IP = '172.30.0.11'
PINNED_VLM_IP = '172.30.0.12'
PINNED_MOTION_IP = '172.30.0.13'
PINNED_FEATURE_ID = 'N11176336906968411287'

REQUIRED_IDENTITY_KEYS = frozenset(
    {
        'ANYGRASP_MAC_ADDRESS',
        'ANYGRASP_IPV4_ADDRESS',
        'ANYGRASP_EXPECTED_FEATURE_ID',
        'PERCEPTION_IPV4_ADDRESS',
        'VLM_IPV4_ADDRESS',
        'MOTION_IPV4_ADDRESS',
        'HYBRID_SUBNET',
        'HYBRID_GATEWAY',
        'ROS_DOMAIN_ID',
        'ANYGRASP_LICENSE_HOST_DIR',
        'ANYGRASP_LICENSE_FILE',
        'ANYGRASP_MODEL_HOST_DIR',
        'ANYGRASP_CHECKPOINT_PATH',
        'SAM2_MODEL_HOST_DIR',
        'SAM2_MODEL_CONFIG',
        'SAM2_CHECKPOINT_PATH',
    }
)

_KEY_PATTERN = re.compile(r'^[A-Z][A-Z0-9_]*$')
_VALUE_PATTERN = re.compile(r'^[A-Za-z0-9_./:@+,-]+$')
_MAC_PATTERN = re.compile(r'^[0-9a-f]{2}(?::[0-9a-f]{2}){5}$')


class PreflightError(RuntimeError):
    """Raised when the licensed service must not start."""


@dataclass(frozen=True)
class NetworkInterface:
    name: str
    mac_address: str
    ipv4_addresses: tuple[str, ...]


@dataclass(frozen=True)
class Mount:
    mount_point: Path
    options: frozenset[str]


def read_identity(path: Path) -> dict[str, str]:
    try:
        lines = path.read_text(encoding='utf-8').splitlines()
    except OSError as error:
        raise PreflightError(
            f'Cannot read Jetson identity file {path}: {error}'
        ) from error

    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(lines, start=1):
        line = raw_line.strip()
        if not line or line.startswith('#'):
            continue
        if '=' not in line:
            raise PreflightError(
                f'Invalid Jetson identity line {line_number}: '
                'expected KEY=VALUE'
            )
        key, value = line.split('=', 1)
        if (
            not _KEY_PATTERN.fullmatch(key)
            or not value
            or not _VALUE_PATTERN.fullmatch(value)
        ):
            raise PreflightError(f'Invalid Jetson identity line {line_number}')
        if key in values:
            raise PreflightError(f'Duplicate Jetson identity key: {key}')
        values[key] = value

    missing = sorted(REQUIRED_IDENTITY_KEYS - values.keys())
    unknown = sorted(values.keys() - REQUIRED_IDENTITY_KEYS)
    if missing:
        raise PreflightError(
            f'Missing Jetson identity keys: {", ".join(missing)}'
        )
    if unknown:
        raise PreflightError(
            f'Unknown Jetson identity keys: {", ".join(unknown)}'
        )
    validate_identity_values(values)
    return values


def validate_identity_file(
    path: Path,
    *,
    require_root: bool = True,
) -> dict[str, str]:
    try:
        metadata = path.lstat()
    except OSError as error:
        raise PreflightError(
            f'Jetson identity file is required at {path}: {error}'
        ) from error
    if not stat.S_ISREG(metadata.st_mode):
        raise PreflightError(f'Jetson identity must be a regular file: {path}')
    if require_root and metadata.st_uid != 0:
        raise PreflightError(f'Jetson identity must be owned by root: {path}')
    if metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
        raise PreflightError(
            f'Jetson identity must not be group/other writable: {path}'
        )
    return read_identity(path)


def validate_identity_values(values: Mapping[str, str]) -> None:
    pinned = {
        'ANYGRASP_MAC_ADDRESS': PINNED_ANYGRASP_MAC,
        'ANYGRASP_IPV4_ADDRESS': PINNED_ANYGRASP_IP,
        'ANYGRASP_EXPECTED_FEATURE_ID': PINNED_FEATURE_ID,
        'PERCEPTION_IPV4_ADDRESS': PINNED_PERCEPTION_IP,
        'VLM_IPV4_ADDRESS': PINNED_VLM_IP,
        'MOTION_IPV4_ADDRESS': PINNED_MOTION_IP,
    }
    for key, expected in pinned.items():
        if values[key] != expected:
            raise PreflightError(
                f'{key} is migration-controlled: '
                f'expected={expected} actual={values[key]}'
            )
    if not _MAC_PATTERN.fullmatch(values['ANYGRASP_MAC_ADDRESS']):
        raise PreflightError('ANYGRASP_MAC_ADDRESS is invalid')
    if not re.fullmatch(r'N[0-9]+', values['ANYGRASP_EXPECTED_FEATURE_ID']):
        raise PreflightError('ANYGRASP_EXPECTED_FEATURE_ID is invalid')

    try:
        network = ipaddress.ip_network(values['HYBRID_SUBNET'], strict=True)
        gateway = ipaddress.ip_address(values['HYBRID_GATEWAY'])
        addresses = [
            ipaddress.ip_address(values[key])
            for key in (
                'ANYGRASP_IPV4_ADDRESS',
                'PERCEPTION_IPV4_ADDRESS',
                'VLM_IPV4_ADDRESS',
                'MOTION_IPV4_ADDRESS',
            )
        ]
    except ValueError as error:
        raise PreflightError(
            f'Invalid hybrid network identity: {error}'
        ) from error
    addresses_outside_network = any(
        address not in network for address in addresses
    )
    if (
        network.version != 4
        or gateway not in network
        or addresses_outside_network
    ):
        raise PreflightError(
            'Hybrid gateway and service addresses must be in the IPv4 subnet'
        )
    if len({gateway, *addresses}) != 5:
        raise PreflightError(
            'Hybrid gateway and service addresses must be unique'
        )

    try:
        domain_id = int(values['ROS_DOMAIN_ID'])
    except ValueError as error:
        raise PreflightError('ROS_DOMAIN_ID must be an integer') from error
    if not 0 <= domain_id <= 232:
        raise PreflightError('ROS_DOMAIN_ID must be between 0 and 232')

    for key in (
        'ANYGRASP_LICENSE_HOST_DIR',
        'ANYGRASP_MODEL_HOST_DIR',
        'ANYGRASP_CHECKPOINT_PATH',
        'SAM2_MODEL_HOST_DIR',
        'SAM2_CHECKPOINT_PATH',
    ):
        if not Path(values[key]).is_absolute():
            raise PreflightError(f'{key} must be an absolute path')


def validate_environment(
    identity: Mapping[str, str],
    environment: Mapping[str, str],
) -> None:
    runtime_keys = (
        'ANYGRASP_MAC_ADDRESS',
        'ANYGRASP_IPV4_ADDRESS',
        'ANYGRASP_EXPECTED_FEATURE_ID',
        'ANYGRASP_LICENSE_FILE',
        'ANYGRASP_CHECKPOINT_PATH',
        'ROS_DOMAIN_ID',
    )
    for key in runtime_keys:
        actual = environment.get(key)
        if actual != identity[key]:
            raise PreflightError(
                f'AnyGrasp environment override detected for {key}: '
                f'expected={identity[key]} actual={actual or "unset"}'
            )


def validate_interfaces(
    interfaces: Sequence[NetworkInterface],
    *,
    expected_mac: str,
    expected_ipv4: str,
) -> None:
    non_loopback = [
        interface for interface in interfaces if interface.name != 'lo'
    ]
    if len(non_loopback) != 1 or non_loopback[0].name != 'eth0':
        names = (
            ','.join(interface.name for interface in non_loopback) or 'none'
        )
        raise PreflightError(
            'AnyGrasp requires exactly one non-loopback interface named eth0; '
            f'actual={names}'
        )
    interface = non_loopback[0]
    if interface.mac_address.lower() != expected_mac.lower():
        raise PreflightError(
            f'Unexpected eth0 MAC: expected={expected_mac} '
            f'actual={interface.mac_address}'
        )
    if interface.ipv4_addresses != (expected_ipv4,):
        actual = ','.join(interface.ipv4_addresses) or 'none'
        raise PreflightError(
            f'Unexpected eth0 IPv4 set: expected={expected_ipv4} '
            f'actual={actual}'
        )


def runtime_interfaces() -> tuple[NetworkInterface, ...]:
    try:
        result = subprocess.run(
            ['ip', '-j', '-4', 'address', 'show'],
            check=True,
            capture_output=True,
            text=True,
        )
        payload = json.loads(result.stdout)
    except (
        OSError,
        subprocess.CalledProcessError,
        json.JSONDecodeError,
    ) as error:
        raise PreflightError(
            f'Cannot inspect container network interfaces: {error}'
        ) from error

    interfaces = []
    for item in payload:
        name = str(item.get('ifname', ''))
        if not name:
            continue
        try:
            mac = Path(f'/sys/class/net/{name}/address').read_text(
                encoding='ascii'
            ).strip()
        except OSError as error:
            raise PreflightError(
                f'Cannot read MAC for {name}: {error}'
            ) from error
        ipv4_addresses = tuple(
            sorted(
                str(address['local'])
                for address in item.get('addr_info', ())
                if address.get('family') == 'inet' and 'local' in address
            )
        )
        interfaces.append(NetworkInterface(name, mac, ipv4_addresses))
    return tuple(interfaces)


def _unescape_mount_path(value: str) -> Path:
    for escaped, replacement in (
        ('\\040', ' '),
        ('\\011', '\t'),
        ('\\012', '\n'),
        ('\\134', '\\'),
    ):
        value = value.replace(escaped, replacement)
    return Path(value)


def parse_mountinfo(text: str) -> tuple[Mount, ...]:
    mounts = []
    for line in text.splitlines():
        fields = line.split()
        if len(fields) < 7 or '-' not in fields:
            raise PreflightError('Malformed /proc/self/mountinfo entry')
        mounts.append(
            Mount(
                mount_point=_unescape_mount_path(fields[4]),
                options=frozenset(fields[5].split(',')),
            )
        )
    return tuple(mounts)


def read_mountinfo(
    path: Path = Path('/proc/self/mountinfo'),
) -> tuple[Mount, ...]:
    try:
        return parse_mountinfo(path.read_text(encoding='utf-8'))
    except OSError as error:
        raise PreflightError(
            f'Cannot inspect container mounts: {error}'
        ) from error


def validate_read_only_mount(target: Path, mounts: Sequence[Mount]) -> None:
    absolute_target = target.resolve(strict=False)
    candidates = []
    for mount in mounts:
        mount_point = mount.mount_point.resolve(strict=False)
        try:
            absolute_target.relative_to(mount_point)
        except ValueError:
            continue
        candidates.append(mount)
    if not candidates:
        raise PreflightError(f'No mount contains required path: {target}')
    selected = max(candidates, key=lambda mount: len(mount.mount_point.parts))
    if 'ro' not in selected.options:
        raise PreflightError(
            f'Required AnyGrasp path is not on a read-only mount: {target}'
        )


def get_feature_id() -> str:
    try:
        from gsnet import get_feature_id as sdk_get_feature_id
    except (ImportError, OSError) as error:
        raise PreflightError(
            f'AnyGrasp SDK is unavailable: {error}'
        ) from error
    feature_id = str(sdk_get_feature_id()).strip()
    if not feature_id:
        raise PreflightError('AnyGrasp SDK returned an empty feature ID')
    return feature_id


def validate_feature_id(actual: str, expected: str) -> None:
    if actual != expected:
        raise PreflightError(
            'Unexpected AnyGrasp SDK feature ID: '
            f'expected={expected} actual={actual}'
        )


def run_runtime_preflight(*, print_feature_id: bool = False) -> str:
    identity = validate_identity_file(IDENTITY_PATH)
    validate_environment(identity, os.environ)
    validate_interfaces(
        runtime_interfaces(),
        expected_mac=identity['ANYGRASP_MAC_ADDRESS'],
        expected_ipv4=identity['ANYGRASP_IPV4_ADDRESS'],
    )
    mounts = read_mountinfo()
    validate_read_only_mount(IDENTITY_PATH, mounts)
    validate_read_only_mount(
        Path(os.environ['ANYGRASP_LICENSE_DIR']),
        mounts,
    )
    validate_read_only_mount(
        Path(identity['ANYGRASP_CHECKPOINT_PATH']),
        mounts,
    )
    feature_id = get_feature_id()
    validate_feature_id(feature_id, identity['ANYGRASP_EXPECTED_FEATURE_ID'])
    if print_feature_id:
        print(feature_id)
    return feature_id


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--print-feature-id', action='store_true')
    parser.add_argument('--validate-host-identity', type=Path)
    arguments = parser.parse_args()
    try:
        if arguments.validate_host_identity is not None:
            validate_identity_file(arguments.validate_host_identity)
        else:
            run_runtime_preflight(print_feature_id=arguments.print_feature_id)
    except PreflightError as error:
        print(f'AnyGrasp preflight failed: {error}', file=os.sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
