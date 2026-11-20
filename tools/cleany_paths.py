"""Native install locations shared with the root Makefile."""
import os
from pathlib import Path


def install_base(repository: Path) -> Path:
    explicit = os.environ.get('CLEANY_INSTALL_BASE')
    if explicit:
        return Path(explicit).expanduser().resolve()
    explicit_build = os.environ.get('CLEANY_BUILD_ROOT')
    if explicit_build:
        build = Path(explicit_build).expanduser()
    elif (repository / '.env.workspace-build').is_file():
        build = repository / 'ros2_ws'
    else:
        cache = Path(os.environ.get('XDG_CACHE_HOME') or Path.home() / '.cache').expanduser()
        build = cache / 'cleany' / 'builds' / repository.name
    return (build / 'install').resolve()
