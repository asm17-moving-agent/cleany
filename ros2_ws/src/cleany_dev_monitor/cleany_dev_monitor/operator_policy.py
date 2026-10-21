"""Fail-closed operator capabilities, independent of HTTP and ROS."""

def capabilities(state: dict, *, busy: bool, managed: bool) -> dict[str, str]:
    reason = '다른 실행·준비·복구 작업이 진행 중입니다.' if busy else ''
    if state.get('active_execution_id') or state.get('arm_return_active') or state.get('recovery_active'):
        reason = '실물 실행 또는 복구가 진행 중입니다.'
    unavailable = reason or ('실행기의 최신 상태를 확인할 수 없습니다.' if not state.get('available') else '') or ('' if managed else '관리 런타임을 먼저 준비하세요.')
    guard = state.get('guard') or {}
    ready = state.get('guard_fresh') is True and guard.get('state') == 'ready' and not state.get('hardware_fault')
    software_stop = (state.get('guard_fresh') is True and guard.get('reason') == 'Operator software stop'
                     and guard.get('controllers_stopped') is True and guard.get('actions_cancelled') is True)
    control = unavailable or ('' if ready else '제어기의 최신 준비 상태가 확인되지 않았습니다.')
    return {
        'prepare': reason or ('기존 런타임의 실행 상태를 먼저 확인하세요.' if managed and not state.get('available') else ''),
        'inspect': control,
        'run': control or ('실행 차단을 먼저 해제하세요.' if state.get('inhibited') else '')
                       or ('이전 작업이 종료되지 않았습니다.' if not state.get('backend_ready') else '')
                       or ('보유 물체를 먼저 확인하세요.' if state.get('payload_attached') else ''),
        'return_arm': unavailable or ('' if ready or software_stop else '하드웨어 상태를 확인한 후 복귀하세요.'),
        'recover': control or ('해제할 차단 기록이 없습니다.' if not state.get('blocked') else '')
                           or ('보유 물체가 남아 있습니다.' if state.get('payload_attached') else ''),
        'shutdown': unavailable or ('실행기의 남은 동작을 먼저 종료하세요.' if not state.get('backend_ready') else ''),
    }


def validate_action(action: str, args: dict) -> dict:
    if not isinstance(args, dict):
        raise ValueError('잘못된 요청 인자입니다.')
    if action == 'prepare':
        if set(args) != {'confirm_torque_on'} or args['confirm_torque_on'] is not True:
            raise ValueError('모터 토크 활성화를 확인하세요.')
        return dict(args)
    if action in ('inspect', 'run'):
        if args:
            raise ValueError('추가 인자를 허용하지 않는 명령입니다.')
        return {}
    if action == 'return_arm':
        if set(args) != {'arm', 'confirm_open'} or args['arm'] not in ('both', 'left', 'right') or args['confirm_open'] is not True:
            raise ValueError('복귀할 팔과 그리퍼 열기를 확인하세요.')
        return dict(args)
    if action == 'recover':
        if set(args) != {'execution_id', 'revision', 'stopped', 'empty', 'note'}:
            raise ValueError('차단 기록과 운영자 확인이 필요합니다.')
        if (not isinstance(args['execution_id'], str) or not 1 <= len(args['execution_id']) <= 128
                or type(args['revision']) is not int or args['revision'] < 0
                or args['stopped'] is not True or args['empty'] is not True
                or not isinstance(args['note'], str) or not 1 <= len(args['note'].strip()) <= 500):
            raise ValueError('정지·빈 그리퍼와 확인 내용을 입력하세요.')
        return dict(args)
    if action == 'shutdown':
        if set(args) != {'confirm_torque_off'} or args['confirm_torque_off'] is not True:
            raise ValueError('토크 해제를 확인하세요.')
        return dict(args)
    raise ValueError('허용되지 않은 명령입니다.')
