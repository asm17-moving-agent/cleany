"""Guard MoveGroup and ExecuteTrajectory goals, including cancellation completion."""
import time
import rclpy


def enforce_motion_guard(node, handle=None, result_future=None, label='motion') -> None:
    check = getattr(node, '_check_motion_guard', None)
    if check is None:
        return
    try:
        check()
    except Exception:
        if handle is not None:
            node.get_logger().error(f'MOTION GUARD cancel requested: {label}')
            cancel_future = handle.cancel_goal_async()
            # Humble ExecuteTrajectory can acknowledge cancellation without
            # forwarding it to the controller. Independently stop the active JTC.
            stop = getattr(node, '_stop_guarded_controller', None)
            if stop is not None:
                stop(result_future)
            node._future(cancel_future, 5., f'{label} guard cancel acknowledgement')
            node._future(result_future, 10., f'{label} guard terminal result')
            node.get_logger().error(f'MOTION GUARD terminal result received: {label}')
        raise


def guarded_action_result(node, handle, timeout: float, label: str):
    future = handle.get_result_async()
    if not hasattr(node, '_check_motion_guard'):
        return node._future(future, timeout, label)
    deadline = time.monotonic()+timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
        enforce_motion_guard(node, handle, future, label)
    enforce_motion_guard(node, handle, future, label)
    if not future.done():
        node._future(handle.cancel_goal_async(), 5., f'{label} timeout cancellation')
        node._future(future, 10., f'{label} timeout terminal result')
        raise RuntimeError(f'{label} timed out')
    return future.result()
