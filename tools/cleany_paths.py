"""Native install locations shared with the root Makefile."""
import os
from pathlib import Path


def install_base(repository: Path) -> Path:
    explicit = os.environ.get('CLEANY_INSTALL_BASE')
    if explicit:
        return Path(explicit).expanduser().resolve()
    data = Path(os.environ.get('CLEANY_DATA_ROOT', repository.parent / 'cleany-data')).expanduser()
    build = Path(os.environ.get('CLEANY_BUILD_ROOT', data / 'builds' / repository.name)).expanduser()
    return (build / 'install').resolve()
