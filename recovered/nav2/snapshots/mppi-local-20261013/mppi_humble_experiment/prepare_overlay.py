#!/usr/bin/env python3
"""Materialize a pinned MPPI experiment outside the Cleany source tree."""
import argparse, hashlib, json, subprocess, tarfile, urllib.request
from pathlib import Path

UPSTREAM_SHA = "c965b7a36ef48cd7f35f01c1f98883741693d195dae582232d6d0444d2eedab6"
UPSTREAM_URL = "https://github.com/ros-navigation/navigation2/archive/refs/tags/1.1.20.tar.gz"

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('workspace', type=Path)
    parser.add_argument('--patch', type=Path, default=Path(__file__).parent / 'nav2-1.1.20.patch')
    args = parser.parse_args()
    root = args.workspace.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    patch = args.patch.resolve()
    marker = root / 'source_provenance.json'
    expected = {'upstream_sha256': UPSTREAM_SHA,
                'patch_sha256': hashlib.sha256(patch.read_bytes()).hexdigest()}
    source = root / 'src/nav2_mppi_controller'
    if source.exists():
        if not marker.exists() or json.loads(marker.read_text()) != expected:
            raise RuntimeError('Existing source has different provenance; choose a fresh workspace')
        return
    archive = root / 'nav2-1.1.20.tar.gz'
    if not archive.exists():
        urllib.request.urlretrieve(UPSTREAM_URL, archive)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != UPSTREAM_SHA:
        raise RuntimeError('Upstream archive checksum mismatch')
    with tarfile.open(archive) as stream:
        for entry in stream.getmembers():
            prefix = 'navigation2-1.1.20/nav2_mppi_controller/'
            if not entry.name.startswith(prefix):
                continue
            relative = Path(entry.name[len('navigation2-1.1.20/'):])
            if relative.is_absolute() or '..' in relative.parts or entry.issym() or entry.islnk():
                raise RuntimeError('Unexpected archive member')
            entry.name = str(relative)
            stream.extract(entry, root / 'src')
    subprocess.run(['patch', '--batch', '--forward', '-p1', '-i', str(patch)],
                   cwd=source, check=True)
    marker.write_text(json.dumps(expected, indent=2))

if __name__ == '__main__':
    main()
