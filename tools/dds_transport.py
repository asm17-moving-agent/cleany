#!/usr/bin/env python3
"""Check or provision Linux receive buffers for the real RGB-D runtime."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import socket
import subprocess

MINIMUM_BYTES = 8 * 1024 * 1024
SYSCTL_FILE = Path('/etc/sysctl.d/90-cleany-dds.conf')
KERNEL_LIMIT = Path('/proc/sys/net/core/rmem_max')


def check_receive_buffer() -> tuple[bool, str]:
    # Some container network namespaces hide this host-wide sysctl.
    try:
        limit = int(KERNEL_LIMIT.read_text())
    except FileNotFoundError:
        limit = None
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, MINIMUM_BYTES)
        # Linux reports twice the requested amount, including bookkeeping space.
        actual = probe.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    ready = (limit is None or limit >= MINIMUM_BYTES) and actual >= 2 * MINIMUM_BYTES
    return ready, (
        f'DDS receive buffer: rmem_max={limit}, requested={MINIMUM_BYTES}, '
        f'Linux SO_RCVBUF={actual}; {"READY" if ready else "INSUFFICIENT"}'
    )


def persistent_limit(current: int, existing: str) -> int:
    """Never reduce a larger current or previously provisioned receive limit."""
    values = [current, MINIMUM_BYTES]
    for line in existing.splitlines():
        key, separator, value = line.split('#', 1)[0].partition('=')
        if separator and key.strip() == 'net.core.rmem_max':
            values.append(int(value.strip()))
    return max(values)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('check', 'apply'))
    args = parser.parse_args()
    if args.command == 'apply':
        if os.geteuid() != 0:
            parser.error('apply requires sudo; no ROS process or motor is started')
        if not KERNEL_LIMIT.exists():
            parser.error('apply must run on the Linux host; this namespace hides rmem_max')
        existing = SYSCTL_FILE.read_text() if SYSCTL_FILE.exists() else ''
        value = persistent_limit(int(KERNEL_LIMIT.read_text()), existing)
        # This dedicated file has a single managed key. Reject unrelated content.
        for line in existing.splitlines():
            content = line.split('#', 1)[0].strip()
            if content and content.partition('=')[0].strip() != 'net.core.rmem_max':
                parser.error(f'{SYSCTL_FILE} contains unmanaged settings; preserve it and review manually')
        SYSCTL_FILE.write_text(
            '# Cleany RGB-D DDS receive buffer; restart ROS processes after applying.\n'
            f'net.core.rmem_max = {value}\n'
        )
        SYSCTL_FILE.chmod(0o644)
        subprocess.run(['sysctl', '-p', str(SYSCTL_FILE)], check=True)
        print('Persistent setting applied. Restart affected ROS host/container processes.')
    ready, message = check_receive_buffer()
    print(message)
    if not ready:
        print(f'Run: sudo python3 {Path(__file__).resolve()} apply')
        print('Then restart affected ROS processes; existing sockets are not resized.')
    return 0 if ready else 1


if __name__ == '__main__':
    raise SystemExit(main())
