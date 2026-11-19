"""노드 정리가 끝날 때까지 Humble controller 콜백 코드를 메모리에 유지합니다."""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix


def controller_plugin_environment(plugin: str, *, keep_loaded: bool) -> dict[str, str]:
    """MoveIt 2.5 controller unload 우회 처리를 해당 프로세스에만 적용합니다."""
    if not keep_loaded or plugin != 'moveit_simple_controller_manager/MoveItSimpleControllerManager':
        return {}
    library = Path(get_package_prefix('moveit_simple_controller_manager')) / 'lib/libmoveit_simple_controller_manager.so'
    if not library.is_file():
        raise RuntimeError(f'MoveIt controller plugin library is missing: {library}')
    # MoveIt 2.5.9는 controller_mgr_node_보다 loader를 먼저 파괴합니다.
    # 노드 소멸 중 CallbackGroup의 약한 참조가 plugin 소유 제어 블록을 필요로 할 수
    # 있으므로 미리 로드해 프로세스 종료까지 코드 매핑을 유지합니다.
    preloads = os.environ.get('LD_PRELOAD', '').strip()
    return {'LD_PRELOAD': ':'.join(value for value in (preloads, str(library)) if value)}
