"""Operator confirmation for an exact historical failure; sends no motion Goal."""
from __future__ import annotations

import argparse
import sys

import rclpy
from cleany_interfaces.srv import ConfirmManipulationRecovery


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description='정지와 빈 그리퍼 확인 후 과거 실행 차단을 해제합니다.')
    parser.add_argument('--execution-id', help='차단된 실행 ID; 차단 기록이 하나이면 자동 선택')
    parser.add_argument('--confirm-stopped', action='store_true', help='운영자가 로봇 정지를 확인함')
    parser.add_argument('--confirm-empty', action='store_true', help='운영자가 양쪽 그리퍼가 비었음을 확인함')
    parser.add_argument('--note', default='Operator confirmed stopped robot and empty grippers')
    parser.add_argument('--status', action='store_true', help='차단 기록만 조회')
    options, ros_args = parser.parse_known_args(args)
    if options.confirm_stopped != options.confirm_empty:
        parser.error('--confirm-stopped와 --confirm-empty를 함께 지정해야 합니다.')
    node = None

    def call(request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=20.)
        if not future.done():
            raise TimeoutError('복구 응답 시간 초과; 다시 실행해 상태를 조회하세요.')
        return future.result()

    try:
        rclpy.init(args=ros_args)
        node = rclpy.create_node('manipulation_recovery_client')
        client = node.create_client(ConfirmManipulationRecovery, '/manipulation/confirm_recovery')
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('복구 서비스가 없습니다. 새 실행기를 make manipulation-start로 시작하세요.')
        status = call(ConfirmManipulationRecovery.Request())
        blocked = dict(zip(status.blocked_execution_ids, status.blocked_revisions, strict=True))
        print(f'inhibited={status.inhibited}', flush=True)
        for execution_id, revision in blocked.items():
            print(f'차단 실행: {execution_id}, revision={revision}', flush=True)
        if options.status:
            return 0
        if not blocked:
            if status.inhibited:
                raise RuntimeError('과거 기록 이외의 차단이 남았습니다. 실행기와 하드웨어 상태를 확인하세요.')
            print('차단된 실행 기록이 없습니다.')
            return 0
        execution_id = options.execution_id
        if execution_id is None:
            if len(blocked) != 1:
                raise RuntimeError('여러 기록이 차단 중입니다. --execution-id로 하나를 지정하세요.')
            execution_id = next(iter(blocked))
        if execution_id not in blocked:
            raise RuntimeError('지정한 실행 ID가 현재 차단 목록에 없습니다.')
        if not options.confirm_stopped:
            if not sys.stdin.isatty():
                raise RuntimeError('대화형 터미널에서 실행하거나 운영자 확인 옵션을 지정하세요.')
            answer = input('로봇이 정지했고 양쪽 그리퍼가 비어 있음을 확인했습니까? [yes/NO] ')
            if answer.strip().lower() != 'yes':
                print('복구 확인을 취소했습니다. 차단 상태는 유지됩니다.')
                return 1
        response = call(ConfirmManipulationRecovery.Request(
            execution_id=execution_id, expected_revision=blocked[execution_id],
            robot_stopped_confirmed=True, gripper_empty_confirmed=True, operator_note=options.note))
        print(response.message, flush=True)
        print(f'inhibited={response.inhibited}', flush=True)
        return 0 if response.success and not response.inhibited else 1
    except (RuntimeError, TimeoutError) as error:
        print(str(error), file=sys.stderr)
        if node is None:
            print('ROS 통신 초기화 실패: make collection-start와 make manipulation-start의 성공을 먼저 확인하세요.',
                  file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
