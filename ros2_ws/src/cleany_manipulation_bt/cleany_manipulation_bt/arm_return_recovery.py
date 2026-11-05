"""Read-only readiness checks for the operator's managed arm-return command."""
from dataclasses import dataclass
import json
from pathlib import Path
import re
import time
from typing import Callable

from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String


SOFTWARE_STOP = 20
RESTART_EXECUTOR = 21


@dataclass(frozen=True)
class ControlReadiness:
    code: int
    message: str


def classify_control(status: dict) -> ControlReadiness:
    if status.get('state') == 'ready' and status.get('reason') == '':
        return ControlReadiness(0, 'Arm control ready')
    if (status.get('state') == 'fault'
            and status.get('reason') == 'Operator software stop'
            and status.get('controllers_stopped') is True
            and status.get('actions_cancelled') is True):
        return ControlReadiness(SOFTWARE_STOP, 'Operator software stop; recovery required')
    return ControlReadiness(1, 'Arm control unavailable: ' + str(status.get('reason') or status))


def check_driver_stop(log_path: str) -> ControlReadiness:
    """Guard stop may conceal an earlier serial/motor fault; inspect its driver too."""
    try:
        log = Path(log_path).read_text(errors='replace')
    except (OSError, ValueError) as error:
        return ControlReadiness(1, f'제어기 정지 원인을 읽을 수 없습니다: {error}')
    faults = re.findall(r'LATCHED hardware fault: (.*?); no automatic restart', log)
    # Only the first fault is logged by each driver instance. The latest latch
    # in the active launch's stdout therefore identifies its stop cause.
    if not faults or faults[-1] != 'All command controllers stopped':
        reason = faults[-1] if faults else 'driver stop unconfirmed'
        return ControlReadiness(1, f'자동 복구할 수 없는 제어기 오류: {reason}')
    return ControlReadiness(SOFTWARE_STOP, '소프트웨어 정지 확인: 자동 복구 가능')


def probe_control(node, namespace: str, timeout: float,
                  cancelled: Callable[[], bool] = lambda: False) -> ControlReadiness:
    """Receive a live volatile status, never a cached transient-local sample."""
    status = None

    def receive(message):
        nonlocal status
        try:
            value = json.loads(message.data)
            status = value if isinstance(value, dict) else {}
        except (ValueError, TypeError):
            status = {}

    topic = namespace.rstrip('/') + '/real_control/status'
    subscription = node.create_subscription(String, topic, receive, 1)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    deadline = time.monotonic() + timeout
    try:
        while node.context.ok() and time.monotonic() < deadline:
            if cancelled():
                raise InterruptedError('Arm return preparation cancelled')
            executor.spin_once(timeout_sec=.05)
            if status is not None:
                if node.count_publishers(topic) != 1:
                    return ControlReadiness(1, 'Arm control status publisher is not unique')
                result = classify_control(status)
                if result.code != 1:
                    return result
                # Allow in-progress software cancellation/deactivation to finish.
                if not (status.get('state') == 'waiting'
                        or (status.get('state') == 'fault'
                            and status.get('reason') == 'Operator software stop')):
                    return result
        return ControlReadiness(1, 'Fresh arm control status unavailable' if status is None
                                else classify_control(status).message)
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_subscription(subscription)
