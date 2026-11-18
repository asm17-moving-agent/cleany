#!/usr/bin/env python3
"""Runner의 GDB를 별도 경로에 묶어 고정 ROS 이미지의 의존성을 바꾸지 않습니다."""
from __future__ import annotations

import argparse
from pathlib import Path
import re
import shutil
import subprocess


def prepare(destination: Path) -> None:
    gdb = shutil.which('gdb')
    if not gdb:
        raise RuntimeError('Runner must provide gdb')
    libraries = subprocess.check_output(['ldd', gdb], text=True)
    if 'not found' in libraries:
        raise RuntimeError(f'GDB dependency missing:\n{libraries}')
    paths = {Path(name) for name in re.findall(r'(/[\w./+\-]+)', libraries)}
    loader = next(path for path in paths if path.name.startswith('ld-linux'))
    destination.mkdir(parents=True, exist_ok=False)
    (destination/'lib').mkdir()
    (destination/'bin').mkdir()
    shutil.copy2(gdb, destination/'gdb')
    for path in paths:
        shutil.copy2(path, destination/'lib'/path.name)
    python_version = re.search(r'libpython(\d+\.\d+)\.so', libraries)
    if python_version:
        stdlib = Path('/usr/lib')/f'python{python_version[1]}'
        shutil.copytree(stdlib, destination/'python/lib'/stdlib.name,
                        ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copytree('/usr/share/gdb', destination/'data')
    # 전용 loader와 라이브러리는 debugger에만 적용하며 검사 프로세스에는 전달하지 않습니다.
    wrapper = destination/'bin/gdb'
    wrapper.write_text(
        '#!/bin/sh\nset -eu\n'
        'root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)\n'
        'unset PYTHONPATH\nexport PYTHONHOME="$root/python"\n'
        f'exec "$root/lib/{loader.name}" --library-path "$root/lib" "$root/gdb" '
        '-nx -iex "set auto-load off" --data-directory "$root/data" "$@"\n')
    wrapper.chmod(0o755)
    version = subprocess.check_output([str(wrapper), '--version'], text=True)
    (destination/'version.txt').write_text(version + '\n' + libraries)
    print(version.splitlines()[0])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    prepare(parser.parse_args().destination.resolve())
