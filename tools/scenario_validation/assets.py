"""Resolve scene assets before Gazebo can silently omit missing geometry."""
from __future__ import annotations

import hashlib
from pathlib import Path
import shlex
import shutil
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

LEGACY_ROOT = '/home/ehdrms/workspace/cleany/feat-runtime-integration'
LEGACY_FIXTURE = LEGACY_ROOT + '/artifacts/gui-integration-20261003'


def localize_and_hash(tree: ET.ElementTree, fixture: Path, root: Path,
                      alias_directory: Path | None = None) -> dict[str, str]:
    """Localize file/Fuel URIs and hash meshes plus their material dependencies.

    Fuel must already be cached; this preflight never downloads or substitutes
    geometry. Unknown URI schemes and missing/empty files fail closed.
    """
    hashes: dict[str, str] = {}

    def resolve(uri: str, relative_to: Path) -> Path:
        uri = uri.replace(LEGACY_FIXTURE, str(fixture)).replace(LEGACY_ROOT, str(root))
        parsed = urlparse(uri)
        if parsed.scheme in ('http', 'https'):
            parts = parsed.path.strip('/').split('/')
            if (parsed.hostname != 'fuel.gazebosim.org' or len(parts) < 7
                    or parts[0] != '1.0' or parts[2] != 'models' or parts[5] != 'files'):
                raise ValueError(f'Unsupported remote asset: {uri}')
            cached = Path(parsed.hostname, parts[1].lower(), 'models',
                          parts[3].lower(), parts[4], *parts[6:])
            for home in (Path.home(), *root.parents):
                for cache in ('.ignition/fuel', '.gz/fuel'):
                    candidate = home/cache/cached
                    if candidate.is_file():
                        return candidate.resolve()
            raise FileNotFoundError(f'Fuel asset is not cached: {uri}')
        if parsed.scheme not in ('', 'file') or parsed.netloc:
            raise ValueError(f'Unsupported asset URI: {uri}')
        path = Path(unquote(parsed.path))
        return (path if path.is_absolute() else relative_to/path).resolve()

    def include(path: Path) -> None:
        key = str(path)
        if key in hashes:
            return
        raw = path.read_bytes()
        if not raw:
            raise ValueError(f'Empty scene asset: {path}')
        hashes[key] = hashlib.sha256(raw).hexdigest()
        dependencies: list[str] = []
        if path.suffix.lower() == '.dae':
            document = ET.fromstring(raw)
            dependencies = [e.text.strip() for e in document.findall(
                './/{*}library_images/{*}image/{*}init_from') if e.text]
        elif path.suffix.lower() in ('.obj', '.mtl'):
            for line in raw.decode().splitlines():
                fields = shlex.split(line, comments=True)
                if not fields:
                    continue
                if path.suffix.lower() == '.obj' and fields[0] == 'mtllib':
                    dependencies.extend(fields[1:])
                elif path.suffix.lower() == '.mtl' and (fields[0].startswith('map_')
                                                       or fields[0] in ('bump', 'disp', 'decal')):
                    # The final token is the filename after optional map flags.
                    dependencies.append(fields[-1])
        for dependency in dependencies:
            include(resolve(dependency, path.parent))

    for element in tree.iter('uri'):
        if not element.text or not element.text.strip():
            raise ValueError('Empty scene URI')
        path = resolve(element.text.strip(), fixture)
        include(path)
        # Fortress rejects both escaped and literal non-ASCII URI paths.
        # Stage STL bytes unchanged at an ASCII path. Other mesh formats may
        # depend on relative materials, so do not silently relocate those.
        if alias_directory is not None and not str(path).isascii():
            if path.suffix.lower() != '.stl':
                raise ValueError(f'Non-ASCII mesh with possible relative dependencies: {path}')
            alias_directory.mkdir(parents=True, exist_ok=True)
            alias = alias_directory/(hashes[str(path)]+'.stl')
            if not alias.exists():
                shutil.copyfile(path, alias)
            include(alias)
            if hashes[str(alias)] != hashes[str(path)]:
                raise ValueError(f'Asset copy hash mismatch: {alias}')
            path = alias
        element.text = 'file://' + str(path)
    return hashes
