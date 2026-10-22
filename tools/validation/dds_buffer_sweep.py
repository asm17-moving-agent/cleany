#!/usr/bin/env python3
"""Compare passive DDS receivers without changing robot or system settings."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
NS = 'https://cdds.io/config'


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))


def receiver_profile(source: Path, destination: Path, size: int) -> None:
    """Copy the transport profile, changing only the receiver's buffer request."""
    tree = ET.parse(source)
    element = tree.find(f'.//{{{NS}}}SocketReceiveBufferSize')
    if element is None:
        raise ValueError('Source profile must declare SocketReceiveBufferSize')
    element.set('min', 'default')
    element.set('max', f'{size}B')
    ET.register_namespace('', NS)
    tree.write(destination, encoding='utf-8', xml_declaration=True)


def own_udp_sockets() -> list[dict]:
    """Read this process's UDP limits and per-socket kernel drops on Linux."""
    sockets = {}
    for entry in Path('/proc/self/fd').iterdir():
        try:
            target = entry.readlink().as_posix()
            if not target.startswith('socket:['):
                continue
            with socket.socket(fileno=os.dup(int(entry.name))) as sock:
                if sock.family not in (socket.AF_INET, socket.AF_INET6) or sock.type != socket.SOCK_DGRAM:
                    continue
                inode = int(target[8:-1])
                sockets[inode] = dict(inode=inode, address=list(sock.getsockname()),
                                      rcvbuf_bytes=sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))
        except OSError:
            continue
    for name in ('udp', 'udp6'):
        for line in (Path('/proc/self/net') / name).read_text().splitlines()[1:]:
            fields = line.split()
            inode = int(fields[9])
            if inode in sockets:
                sockets[inode].update(rx_queue_bytes=int(fields[4].split(':')[1], 16),
                                      drops=int(fields[-1]))
    return sorted(sockets.values(), key=lambda row: row['inode'])


def measure(options: argparse.Namespace) -> None:
    from cleany_dev_monitor.io_probe_node import IOProbe, main as probe_main

    class SocketProbe(IOProbe):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            sockets = own_udp_sockets()
            write_json(self.output/'socket-limits.json', dict(
                requested_bytes=options.buffer_bytes, sockets=sockets,
                linux_expected_rcvbuf_bytes=2*options.buffer_bytes))
            if not sockets or any(s['rcvbuf_bytes'] != 2*options.buffer_bytes for s in sockets):
                raise RuntimeError('Actual SO_RCVBUF differs from the requested Linux limit; inspect socket-limits.json')
            self.socket_timer = self.create_timer(1., self.sample_sockets)
            self.sample_sockets()

        def sample_sockets(self):
            self.writer.put(dict(kind='udp_sockets', t=self.elapsed(), sockets=own_udp_sockets()))

        def finish(self):
            self.socket_timer.cancel()
            self.sample_sockets()
            return super().finish()

    probe_main(['--output', str(options.output), '--seconds', str(options.seconds),
                '--config', str(options.config), '--label', options.label,
                '--repository-root', str(ROOT)], probe_type=SocketProbe)


def sweep(options: argparse.Namespace) -> None:
    if any(n <= 0 for n in options.sizes_kib) or options.rounds < 1:
        raise ValueError('Buffer sizes and rounds must be positive')
    if not math.isfinite(options.seconds) or not math.isfinite(options.warmup) or not 1 <= options.warmup < options.seconds:
        raise ValueError('Require 1 <= warmup < seconds')
    if max(options.sizes_kib)*1024 > int(Path('/proc/sys/net/core/rmem_max').read_text()):
        raise ValueError('Requested buffer exceeds current rmem_max; system settings are never changed')
    options.output.mkdir(parents=True, exist_ok=False)
    git = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    dirty = subprocess.check_output(['git', 'status', '--short'], cwd=ROOT, text=True)
    (options.output/'git-status.txt').write_text(dirty)
    (options.output/'source-profile.xml').write_bytes(options.profile.read_bytes())
    plan = dict(schema=1, started=datetime.now().astimezone().isoformat(), commit=git,
                scope='Passive host receiver only; operating perception receiver and kernel cap unchanged',
                qos='Best Effort, depth 5', sizes_kib=options.sizes_kib, rounds=options.rounds,
                seconds=options.seconds, warmup_s=options.warmup, runs=[], finished=False)
    write_json(options.output/'sweep.json', plan)
    for repeat in range(options.rounds):
        order = options.sizes_kib if repeat % 2 == 0 else list(reversed(options.sizes_kib))
        for size in order:
            label = f'round-{repeat+1}-{size}KiB'
            profile = options.output/f'{label}.xml'
            receiver_profile(options.profile, profile, size*1024)
            output = options.output/label
            env = dict(os.environ, CYCLONEDDS_URI=profile.resolve().as_uri(),
                       OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
            command = [sys.executable, str(Path(__file__).resolve()), 'measure', '--output', str(output),
                       '--seconds', str(options.seconds), '--config', str(options.config),
                       '--buffer-bytes', str(size*1024), '--label', label]
            print(f'{label}: {options.seconds:g}s including {options.warmup:g}s warmup', flush=True)
            with (options.output/f'{label}.log').open('w') as log:
                result = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT)
            plan['runs'].append(dict(label=label, round=repeat+1, size_kib=size, returncode=result.returncode))
            write_json(options.output/'sweep.json', plan)
            if result.returncode:
                raise RuntimeError(f'{label} failed: inspect its log; further runs stopped')
    plan.update(finished=True, ended=datetime.now().astimezone().isoformat())
    write_json(options.output/'sweep.json', plan)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    run = sub.add_parser('run')
    run.add_argument('--profile', type=Path, default=ROOT/'containers/vision/cyclonedds/host.xml')
    run.add_argument('--sizes-kib', nargs='+', type=int, default=[208, 512, 1024, 2048, 4096, 8192])
    run.add_argument('--rounds', type=int, default=2)
    run.add_argument('--warmup', type=float, default=5.)
    child = sub.add_parser('measure')
    child.add_argument('--buffer-bytes', type=int, required=True)
    child.add_argument('--label', default='buffer-observation')
    for command in (run, child):
        command.add_argument('--output', type=Path, required=True)
        command.add_argument('--seconds', type=float, default=35.)
        command.add_argument('--config', type=Path, default=ROOT/'ros2_ws/src/cleany_dev_monitor/config/manipulation_io_probe.json')
    options = parser.parse_args()
    options.output = options.output.resolve()
    (sweep if options.action == 'run' else measure)(options)


if __name__ == '__main__':
    main()
