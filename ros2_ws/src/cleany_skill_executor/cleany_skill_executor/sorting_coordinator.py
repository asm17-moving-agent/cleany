"""분류 정책에 따라 여러 물체를 순서대로 처리하는 ROS 진입점입니다."""
from __future__ import annotations

import rclpy
from cleany_interfaces.srv import ObserveWristTarget

from cleany_skill_executor.core.sorting import Category, execute_sort
from .sorting_operations import (
    SortingOperations, SortTarget, HeldObject, WristTargetNotDetected,
    held_bounding_radius, rotation, object_in_destination,
)

__all__ = ["SortingCoordinator", "SortTarget", "HeldObject", "WristTargetNotDetected",
           "held_bounding_radius", "rotation", "object_in_destination"]


class SortingCoordinator(SortingOperations):
    """공통 연산에 자동 분류 반복과 집기 순서를 연결합니다."""

    def run(self):
        self._stage('starting')
        self._wait_for_pipeline()
        verify_placement = getattr(self, '_placement_verification_enabled', True)
        if verify_placement and not self._verification.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Placement verification port is unavailable')
        self._register_bins()
        if self._wrist_enabled:
            if not self._wrist_client.wait_for_service(timeout_sec=10):
                raise RuntimeError('Wrist observation service unavailable')
            self._switch_camera('head')
        self._home = {arm: self._arm_joint_state(arm)
                      for arm in ('left', 'right')}
        self._stage('search')
        maximum = int(self.get_parameter('sorting_maximum_objects').value)
        empty_confirmations = 0
        minimum_total_placements = 0
        for _ in range(maximum):
            detected = self._next_sorting_detection()
            self._record_pipeline_message('detections', detected)
            choices = self._attempts(detected.detections.detections)
            inspections = {}
            def inspect(attempt):
                if attempt.object_id not in inspections:
                    inspections[attempt.object_id] = self._inspect_selected(
                        detected.detections.snapshot_id, attempt)
                return inspections[attempt.object_id]
            work_count = len(detected.detections.detections)
            prepared = None
            unresolved = len(detected.detections.detections) - len(choices)
            for attempt in choices:
                decision = self._policy.classify_model(
                    attempt.label, attempt.confidence,
                    attempt.sorting_category, attempt.sorting_reason)
                self._current = dict(label=attempt.label,
                                     distance_m=attempt.distance_m,
                                     category=decision.category.value,
                                     destination=decision.destination,
                                     reason=decision.reason)
                if decision.category == Category.REVIEW:
                    unresolved += 1
                    self._stage('review')
                    continue
                inspected = inspect(attempt)
                if inspected is None:
                    unresolved += 1
                    continue
                center = inspected.objects.objects[0].obb_pose.position
                # 이미 놓은 물체는 카메라에 계속 보여도 다시 집지 않습니다.
                # 참값이 아닌 인식 위치를 사용합니다.
                destination = self._bins.get(decision.destination)
                if object_in_destination(inspected.objects.objects[0],destination):
                    work_count -= 1
                    continue
                unresolved += 1
                planned = self._plan_grasps(inspected, attempt)
                if planned is None:
                    self._stage('no_grasp')
                    continue
                # 어깨 베이스가 대칭이므로 base_link Y로 가까운 팔을 정합니다.
                # 해당 팔의 후보를 모두 시도한 뒤 반대 팔을 시도합니다. 목적지는 집기를
                # 제한하지 않으며 운반은 충돌을 고려하는 planner가 별도로 검사합니다.
                arms = ('left', 'right') if center.y >= 0 else ('right', 'left')
                selected = None
                for arm in arms:
                    selected = self._select_reachable(
                        planned.candidates, attempt, required_arm=arm)
                    if selected is not None:
                        break
                if selected is None:
                    self._stage('unreachable')
                    continue
                prepared = SortTarget(attempt, selected)
                break
            minimum_total_placements = max(
                minimum_total_placements, len(self._completed)+work_count)
            if prepared is None:
                if unresolved:
                    raise RuntimeError(f'Work area is not clear: {unresolved} unresolved objects')
                if len(self._completed) < minimum_total_placements:
                    raise RuntimeError('Previously observed work objects are missing without verified placement')
                empty_confirmations += 1
                self._stage('empty_confirmation')
                if empty_confirmations >= int(self.get_parameter('sorting_empty_confirmations').value):
                    break
                continue
            empty_confirmations = 0
            if verify_placement:
                execute_sort(prepared, decision, self, self._stage)
            else:
                execute_sort(prepared, decision, self, self._stage, verify_placement=False)
            self._completed.append(dict(self._current))
        else:
            raise RuntimeError('Sorting action/search budget exhausted before clear-workspace confirmation')
        required = (set(self.get_parameter('sorting_required_categories').value)
                    if getattr(self, '_require_category_coverage', False) else set())
        categories = {item['category'] for item in self._completed}
        if not required <= categories:
            self._stage('incomplete')
            raise RuntimeError(
                'Required sorting paths not verified: '
                + ', '.join(sorted(required - categories)))
        self._stage('mission_complete' if verify_placement else 'mission_complete_unverified')


    def pick(self, target: SortTarget) -> HeldObject:
        self._held_object = None
        self._fixed_release_wrist = {}
        self._lift_completion_stamp_ns = None
        self._execute_pregrasp(target.selected, target.attempt)
        # 선택한 그리퍼는 pregrasp 궤적을 따라 이미 열었으므로
        # 도착 후 별도 시간 기반 열기 명령을 내지 않습니다.
        if getattr(self, '_wrist_enabled', False):
            selected, attempt = target.selected, target.attempt
            self._wait_arm_stationary(selected.selected_arm)
            selected, attempt = self._ensure_fresh_head_before_wrist(selected, attempt)
            if selected is not target.selected:
                target = SortTarget(attempt, selected)
            self._switch_camera(selected.selected_arm)
            self._wrist_reference = self._observe_wrist(selected, ObserveWristTarget.Request.HANDOFF)
            # 원래 헤드 3D 추정을 유지합니다. RGB 일치성은 새 깊이 측정이 아닙니다.
            self._execution_scene.allow_contacts_for(selected.selected_arm)
        else:
            if getattr(self, '_geometry_association_lift', False):
                selected, attempt = self._refresh_selected_grasp(
                    target.selected, target.attempt, match_label=False)
            else:
                selected, attempt = self._refresh_selected_grasp(target.selected, target.attempt)
        self._execute_grasp_and_lift(selected, attempt)
        if self._held_object is None:
            raise RuntimeError('No settled grasp attachment reference')
        self._held_object.target = target
        return self._held_object


    def _next_sorting_detection(self):
        if getattr(self, '_adaptive_camera_rate', False):
            self._set_head_high_rate(True)
        pending = getattr(self, '_return_detection', None)
        self._return_detection = None
        try:
            if pending is None:
                return self._detect_objects()
            self.get_logger().info('Using scene detection requested during return')
            return self._finish_object_detection(pending)
        finally:
            if getattr(self, '_adaptive_camera_rate', False):
                self._set_head_high_rate(False)


def main(args=None):
    rclpy.init(args=args)
    node = SortingCoordinator()
    failed = False
    try:
        node.run()
        if not node.get_parameter('sorting_exit_on_finish').value:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        failed = True
        node._stage('failed', error=str(error))
        node.get_logger().fatal(f'SORTING FAILED: {error}')
        try:
            if not node.get_parameter('sorting_exit_on_finish').value:
                rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
