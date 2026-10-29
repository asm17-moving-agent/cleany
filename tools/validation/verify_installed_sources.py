"""Compare installed Python modules and package data with a deployment receipt."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path, PurePosixPath
from typing import Callable


SHARE_DIRECTORIES = {'config', 'launch', 'urdf', 'meshes'}


def module_directory(package: str) -> Path:
    spec = importlib.util.find_spec(package)
    if spec is None or not spec.submodule_search_locations:
        raise ValueError('Installed Python package not found: ' + package)
    return Path(next(iter(spec.submodule_search_locations)))


def share_directory(package: str) -> Path:
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(package))


def verify_sources(
    receipt: dict, *, packages: set[str] | None = None,
    resolved_root: Path | None = None,
    module_root: Callable[[str], Path] = module_directory,
    share_root: Callable[[str], Path] = share_directory,
) -> dict:
    """Check every supported receipt entry, reporting exclusions explicitly.

    Python sources use the active import path; config, launch, URDF and meshes
    use the ament package share. Optional resolved_root rejects files outside
    an expected copied installation. Omit it for native symlink installations.
    No target submodule or ROS node is started.
    """
    checked, skipped, seen, found = [], [], set(), set()
    for row in receipt['files']:
        relative = PurePosixPath(row['path'])
        if relative.is_absolute() or '..' in relative.parts or str(relative) in seen:
            raise ValueError('Invalid or duplicate receipt path: ' + row['path'])
        seen.add(str(relative))
        parts = relative.parts
        if len(parts) < 4 or parts[:2] != ('ros2_ws', 'src'):
            skipped.append(row['path'])
            continue
        package = parts[2]
        if packages is not None and package not in packages:
            skipped.append(row['path'])
            continue
        if parts[3] == package and len(parts) > 4:
            path = module_root(package).joinpath(*parts[4:])
        elif parts[3] in SHARE_DIRECTORIES and len(parts) > 4:
            path = share_root(package).joinpath(*parts[3:])
        else:
            skipped.append(row['path'])
            continue
        resolved = path.resolve(strict=True)
        if resolved_root is not None and not resolved.is_relative_to(resolved_root.resolve()):
            raise ValueError('Installed file is outside the required root: ' + str(resolved))
        digest = hashlib.sha256(resolved.read_bytes()).hexdigest()
        if digest != row['after_sha256']:
            raise ValueError('Installed file differs from receipt: ' + row['path'])
        found.add(package)
        checked.append({'source': row['path'], 'installed': str(path),
                        'resolved': str(resolved), 'sha256': digest})
    if not checked or (packages is not None and packages != found):
        raise ValueError('Receipt has no installed entries for every requested package')
    return {'checked': checked, 'skipped': skipped, 'hashes_match': True}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deployment', type=Path, required=True)
    parser.add_argument('--packages', nargs='+', help='Check only these packages; each must have entries.')
    parser.add_argument('--resolved-root', type=Path, help='Require resolved files below a copied install root.')
    args = parser.parse_args()
    print(json.dumps(verify_sources(json.loads(args.deployment.read_text()),
        packages=set(args.packages) if args.packages else None,
        resolved_root=args.resolved_root), indent=2))


if __name__ == '__main__':
    main()
