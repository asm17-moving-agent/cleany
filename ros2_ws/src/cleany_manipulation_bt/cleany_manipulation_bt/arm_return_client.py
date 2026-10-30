"""Explicit operator command to open grippers and return to saved defaults."""
import argparse
import json
import math
import signal
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from cleany_interfaces.action import ReturnArm

from .arm_return_recovery import (RESTART_EXECUTOR, SOFTWARE_STOP, check_driver_stop,
                                  probe_control)


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=('both', 'left', 'right'), default='both')
    parser.add_argument('--namespace', default='/')
    parser.add_argument('--timeout', type=float, default=240.)
    parser.add_argument('--check-control', action='store_true',
                        help='Only inspect live control status; send no action')
    parser.add_argument('--control-log', default='',
                        help='Active control launch stdout, required for software-stop recovery')
    parser.add_argument('--control-timeout', type=float, default=5.)
    options, ros_args = parser.parse_known_args(args)
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    if not math.isfinite(options.control_timeout) or options.control_timeout <= 0:
        parser.error('--control-timeout must be finite and positive')
    interrupted = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: interrupted.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    node = client = handle = submitted = None
    finished = False

    def wait(future, timeout, *, interruptible=True):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            if interruptible and interrupted.is_set():
                raise InterruptedError('Arm return interrupted; requesting cancellation')
            rclpy.spin_once(node, timeout_sec=.05)
        if not future.done():
            raise TimeoutError('Arm return response timed out')
        return future.result()

    try:
        rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('arm_return_client')
        if options.check_control:
            readiness = probe_control(node, options.namespace, options.control_timeout,
                                      interrupted.is_set)
            if readiness.code == SOFTWARE_STOP:
                readiness = check_driver_stop(options.control_log)
            if interrupted.is_set():
                raise InterruptedError('Arm return preparation cancelled')
            print(readiness.message, flush=True)
            return readiness.code
        client = ActionClient(node, ReturnArm, options.namespace.rstrip('/') + '/manipulation/return_arm')
        if not client.wait_for_server(timeout_sec=10.):
            raise RuntimeError('팔 복귀 서버가 없습니다. make manipulation-start로 실행기를 준비하세요.')
        print(f'그리퍼 열기 → 팔 복귀와 그리퍼 닫기: {options.arm}', flush=True)
        submitted = client.send_goal_async(ReturnArm.Goal(arm=options.arm),
            feedback_callback=lambda msg: print(msg.feedback.stage, flush=True))
        handle = wait(submitted, 10.)
        if not handle.accepted:
            raise RuntimeError('복귀 요청 거절: 진행 중 작업/복구 확인 또는 기본 자세 설정을 확인하세요.')
        response = wait(handle.get_result_async(), options.timeout)
        finished = True
        result = response.result
        print(json.dumps({'success': result.success, 'stop_confirmed': result.stop_confirmed,
                          'message': result.message}, ensure_ascii=False), flush=True)
        if (result.recovery_required and not interrupted.is_set()
                and response.status == GoalStatus.STATUS_ABORTED):
            return RESTART_EXECUTOR
        return 0 if result.success and result.stop_confirmed else 1
    except Exception as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        try:
            if submitted is not None and not finished:
                # Preserve DDS for late acceptance, cancellation and stop Result.
                if handle is None:
                    handle = wait(submitted, 15., interruptible=False)
                if handle.accepted:
                    wait(handle.cancel_goal_async(), 5., interruptible=False)
                    stopped = wait(handle.get_result_async(), 25., interruptible=False).result
                    print(f'취소 후 정지 확인: {stopped.stop_confirmed}', file=sys.stderr)
        except Exception as error:
            print(f'복귀 취소/정지 확인 실패: {error}', file=sys.stderr)
        finally:
            if client is not None:
                client.destroy()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
