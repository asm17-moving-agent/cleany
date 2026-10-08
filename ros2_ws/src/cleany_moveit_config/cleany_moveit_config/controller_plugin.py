"""Keep Humble controller callbacks' code mapped until node teardown finishes."""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix


def controller_plugin_environment(plugin: str, *, keep_loaded: bool) -> dict[str, str]:
    """Scope the MoveIt 2.5 controller unload workaround to its own process."""
    if not keep_loaded or plugin != 'moveit_simple_controller_manager/MoveItSimpleControllerManager':
        return {}
    library = Path(get_package_prefix('moveit_simple_controller_manager')) / 'lib/libmoveit_simple_controller_manager.so'
    if not library.is_file():
        raise RuntimeError(f'MoveIt controller plugin library is missing: {library}')
    # The loader is destroyed before controller_mgr_node_ in MoveIt 2.5.9.
    # CallbackGroup's weak references can still need plugin-owned control blocks
    # during node destruction. Preloading keeps their code mapped until exit.
    preloads = os.environ.get('LD_PRELOAD', '').strip()
    return {'LD_PRELOAD': ':'.join(value for value in (preloads, str(library)) if value)}
