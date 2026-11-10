"""가까운 물체부터 검사하는 실행 순서와 ROS 진입점입니다."""
import rclpy
from .grasp_motion import GraspMotionOperations
from .gripper_operations import LiftRedetectionError

__all__ = ['NearestPregraspCoordinator', 'LiftRedetectionError']


class NearestPregraspCoordinator(GraspMotionOperations):
    """공통 연산에 거리순 물체 탐색과 집기 실행 순서를 연결합니다."""

    def run(self) -> None:
        self._wait_for_pipeline()
        self._run_nearest()


    def _run_nearest(self) -> None:
        detection_result = self._detect_objects()
        attempts = self._attempts(detection_result.detections.detections)
        if not attempts:
            raise RuntimeError(
                'no detection has enough valid depth for autonomous selection'
            )

        failures: list[str] = []
        for attempt in attempts:
            self.get_logger().info(
                f'Trying nearest object_id={attempt.object_id} '
                f'label={attempt.label} distance={attempt.distance_m:.3f}m'
            )
            inspected = self._inspect_selected(
                detection_result.detections.snapshot_id,
                attempt,
            )
            if inspected is None:
                failures.append(f'{attempt.object_id}: inspection')
                continue
            planned = self._plan_grasps(inspected, attempt)
            if planned is None:
                failures.append(f'{attempt.object_id}: no grasp')
                continue
            self._publish_grasp_overlay(planned.candidates)
            selected = self._select_reachable(planned.candidates, attempt)
            if selected is None:
                failures.append(f'{attempt.object_id}: unreachable')
                continue
            self._publish_grasp_overlay(
                planned.candidates,
                selected_index=selected.selected_candidate_index,
                selected_arm=selected.selected_arm,
            )
            if bool(self.get_parameter('plan_only').value):
                self.get_logger().info(
                    'NEAREST PLAN COMPLETE (plan_only): no arm/gripper motion'
                )
                return
            self._execute_pregrasp(selected, attempt)
            if bool(self.get_parameter('execute_grasp_and_lift').value):
                # 물리적인 그리퍼 접촉 전 실패는 선택된 그리퍼를 연 상태로
                # 정지한 pregrasp 자세에 남깁니다.
                self._open_gripper(selected.selected_arm)
                refreshed, refreshed_attempt = self._refresh_selected_grasp(
                    selected, attempt
                )
                self._execute_grasp_and_lift(
                    refreshed, refreshed_attempt
                )
                self.get_logger().info(
                    'NEAREST GRASP COMPLETE: '
                    f'object_id={attempt.object_id} label={attempt.label} '
                    f'distance={attempt.distance_m:.3f}m '
                    f'candidate={refreshed.selected_candidate_index} '
                    f'arm={refreshed.selected_arm}'
                )
                return
            self.get_logger().info(
                'NEAREST PREGRASP COMPLETE: '
                f'object_id={attempt.object_id} label={attempt.label} '
                f'distance={attempt.distance_m:.3f}m '
                f'candidate={selected.selected_candidate_index} '
                f'arm={selected.selected_arm}'
            )
            return
        raise RuntimeError(
            'no distance-valid object produced a reachable pre-grasp; '
            + ', '.join(failures)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NearestPregraspCoordinator()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().fatal(f'NEAREST GRASP PIPELINE FAILED: {error}')
        if bool(
            node.get_parameter('keep_debug_image_alive_on_failure').value
        ):
            node.get_logger().warning(
                'Keeping the failed pipeline alive so the last grasp debug '
                'image remains available'
            )
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
        else:
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
