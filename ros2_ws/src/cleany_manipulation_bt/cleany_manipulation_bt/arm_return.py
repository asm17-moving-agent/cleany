"""Saved operator home poses and an explicit open-then-return operation."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time
from typing import Callable
from uuid import uuid4

from cleany_skill_executor.core.collection import load_arm
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.manipulation.models import Goal

from .backend import ExecutionContext


def selected_arms(arm: str) -> tuple[str, ...]:
    if arm in ('', 'both'):
        return ('left', 'right')
    if arm in ARM_JOINT_NAMES:
        return (arm,)
    raise ValueError('arm must be left, right or both')


def load_home(path: str) -> dict[str, tuple[float, ...]]:
    raw = json.loads(Path(path).expanduser().read_text())
    if (not isinstance(raw, dict) or raw.get('frame') != 'base_link'
            or not isinstance(raw.get('arms'), dict)
            or set(raw['arms']) != set(ARM_JOINT_NAMES)):
        raise ValueError('Saved home must contain both arms in base_link')
    result = {}
    for arm, names in ARM_JOINT_NAMES.items():
        values = raw['arms'][arm]
        if (not isinstance(values, list) or len(values) != len(names)
                or any(type(q) not in (int, float) or not math.isfinite(q) for q in values)):
            raise ValueError(f'{arm}: home must contain five finite joint positions')
        result[arm] = tuple(float(q) for q in values)
    return result


@dataclass(frozen=True)
class ArmReturnResult:
    success: bool
    stop_confirmed: bool
    message: str
    recovery_required: bool = False


def run_return(operations, arms: tuple[str, ...], home: dict[str, tuple[float, ...]],
               abort: threading.Event, feedback: Callable[[str], None]) -> ArmReturnResult:
    """Use the current execution owner, its tracked Actions and physical guards."""
    fault = getattr(operations.real, 'fault', '')
    if fault:
        guard = getattr(operations.real, 'guard', None) or {}
        fresh_ready = (guard.get('state') == 'ready'
                       and time.monotonic() - getattr(operations.real, 'guard_time', 0.)
                       <= operations.real.value('hardware_guard_max_age_sec'))
        if fresh_ready:
            recovery = '제어기 복구 후 실행기에 남은 오류입니다. make manipulation-restart 후 다시 실행하세요.'
        else:
            recovery = ('제어기가 정지 상태입니다. make collection-recover-control 성공 후 '
                        'make manipulation-restart로 실행기도 재시작한 뒤 다시 실행하세요.')
        message = f'{fault}; {recovery} 팔·그리퍼 명령은 보내지 않았습니다.'
        feedback('blocked_before_motion')
        operations.event('manual_arm_return_finished', arms=arms, success=False,
                         stop_confirmed=False, motion_started=False, message=message)
        # A latched guard has stopped publishing joints; issuing another stop
        # cannot manufacture fresh physical-stop evidence or release the latch.
        return ArmReturnResult(False, False, message, recovery_required=True)
    context = ExecutionContext(Goal('operator', 'arm-return', uuid4().hex,
        'return_arm', '', 0, ''), abort=abort)
    operations.execution_context, operations.current_node = context, 'ManualArmReturn'
    operations.deadline = time.monotonic() + operations.value('bt_operation_timeout_sec')
    operations._stopping = False
    # Opening a jaw must remain possible after lost or uncertain contact.
    # Hardware and motion guards continue running through the same boundary.
    previous_releasing = operations.releasing
    operations.releasing = True
    completed, message = False, 'Arm return interrupted'
    try:
        operations.check_execution()
        operations.real.monitor()
        if operations.carrying is not None and operations.carrying['arm'] not in arms:
            raise RuntimeError('An unselected arm still owns a payload; include that arm in the return')
        feedback('checking_stop')
        if not operations._assess_stationary(context, cancel=False):
            raise RuntimeError('Physical stop was not confirmed before arm return')
        configs = {}
        # Validate every selected destination before sending any opening Goal.
        for arm in arms:
            configs[arm] = load_arm(operations.config['arms'][arm], require_bins=False)
            limits = operations.real.verify(arm)
            for name, q in zip(ARM_JOINT_NAMES[arm], home[arm], strict=True):
                limits[name].check(q)
            limits[f'{arm}_gripper_joint'].check(configs[arm].opened)
            limits[f'{arm}_gripper_joint'].check(configs[arm].closed)
        operations.scene_monitor.wait()
        for arm in arms:
            operations.check_execution()
            feedback(f'opening_{arm}')
            operations.jaw(arm, configs[arm].opened, 'manual_open_' + arm,
                           check_collisions=operations.config.get(
                               'gripper_opening_collision_check_enabled', True))
            operations.motion.set_gripper_position(arm, configs[arm].opened)
            operations.event('manual_release_assumed', arm=arm, release_verified=False)
            if operations.carrying is not None and operations.carrying['arm'] == arm:
                operations.detach(assume_released=True)
        # Restore target contact permissions before planning empty-arm returns.
        operations.scene.restore()
        operations.release_grasp_scene_freeze()
        for arm in arms:
            operations.check_execution()
            feedback(f'returning_{arm}')
            state = operations.real.monitor()
            measured = dict(zip(state.name, state.position, strict=True))
            if all(abs(measured[name] - q) <= operations.real.value('hardware_start_tolerance_rad')
                   for name, q in zip(ARM_JOINT_NAMES[arm], home[arm], strict=True)):
                operations.event('manual_return_already_home', arm=arm)
                feedback(f'closing_{arm}')
                operations.jaw(arm, configs[arm].closed, 'manual_close_' + arm)
                operations.motion.set_gripper_position(arm, configs[arm].closed)
                continue
            operations.move_arm_return(arm, home[arm], configs[arm].closed)
            operations.motion.set_gripper_position(arm, configs[arm].closed)
        completed, message = True, 'Arms returned to saved default pose with grippers closed'
    except Exception as error:
        message = str(error)
    finally:
        operations.releasing = previous_releasing
        operations._stopping = True
        feedback('checking_final_stop')
        try:
            confirmed = operations.stop(context).stop_confirmed is True
        except Exception as error:
            confirmed = False
            message += f'; stop assessment failed: {error}'
            operations.real.fail_stop()
        operations._stopping = False
    success = completed and confirmed
    if not confirmed:
        message += '; physical stop unconfirmed'
    operations.event('manual_arm_return_finished', arms=arms, success=success,
                     stop_confirmed=confirmed, message=message)
    return ArmReturnResult(success, confirmed, message)
