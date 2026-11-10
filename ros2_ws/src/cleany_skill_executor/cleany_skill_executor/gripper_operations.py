"""그리퍼 명령, 접촉 유지와 들어 올린 물체의 관측을 처리합니다."""
from __future__ import annotations

import math
import time
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from cleany_skill_executor.core.nearest_object import ObjectAttempt
from cleany_skill_executor.core.gripper import is_gripper_contact_stall, opening_to_gripper_position
from cleany_skill_executor.core.grasp_pipeline import ContactDebounce, ContactLimits, ContactSample


class LiftRedetectionError(RuntimeError):
    """새 검출 snapshot에 같은 label의 거리 유효 타깃이 없습니다."""


class GripperOperations:
    """실행 순서를 소유하지 않는 ROS 작업 메서드입니다."""

    def _contact_monitor(self) -> ContactDebounce:
        effort = float(self.get_parameter('contact_effort_threshold').value)
        return ContactDebounce(
            ContactLimits(
                maximum_tcp_distance_m=float(
                    self.get_parameter(
                        'grasp_contact_stop_max_distance_m'
                    ).value
                ),
                minimum_joint_error_rad=float(
                    self.get_parameter(
                        'contact_min_joint_error_rad'
                    ).value
                ),
                maximum_joint_velocity_rad_s=float(
                    self.get_parameter(
                        'contact_max_joint_velocity_rad_s'
                    ).value
                ),
                consecutive_samples=int(
                    self.get_parameter(
                        'contact_consecutive_samples'
                    ).value
                ),
                effort_threshold=effort if effort > 0.0 else None,
            )
        )


    def _contact_sample(
        self, arm: str, target: Pose
    ) -> ContactSample | None:
        state = self._controller_states.get(arm)
        if state is None:
            return None
        if (
            len(state.reference.positions) == len(state.feedback.positions)
            and state.reference.positions
        ):
            errors = tuple(
                float(reference) - float(feedback)
                for reference, feedback in zip(
                    state.reference.positions,
                    state.feedback.positions,
                    strict=True,
                )
            )
        else:
            errors = tuple(float(value) for value in state.error.positions)
        velocities = tuple(float(value) for value in state.feedback.velocities)
        if not errors or not velocities:
            return None
        effort = (
            tuple(float(value) for value in state.feedback.effort)
            if state.feedback.effort
            else None
        )
        # 정지 또는 오차 조건에 부합할 가능성이 있는 표본에만 FK를 조회합니다.
        distance = math.inf
        effort_threshold = float(
            self.get_parameter('contact_effort_threshold').value
        )
        plausible_signal = (
            max(abs(value) for value in errors)
            >= float(
                self.get_parameter('contact_min_joint_error_rad').value
            )
            or (
                effort_threshold > 0.0
                and effort is not None
                and max(abs(value) for value in effort) >= effort_threshold
            )
        )
        if plausible_signal and max(abs(value) for value in velocities) <= float(
            self.get_parameter(
                'contact_max_joint_velocity_rad_s'
            ).value
        ):
            distance = math.dist(
                self._pose_position(self._tcp_pose(arm)),
                self._pose_position(target),
            )
        return ContactSample(distance, errors, velocities, effort)


    def _verify_lift_height(self, attempt: ObjectAttempt, *,
                            minimum_center_z_m: float | None = None):
        minimum = float(
            self.get_parameter('lift_min_center_z_m').value
        )
        if not math.isfinite(minimum):
            raise ValueError('configured lift height must be finite')
        if minimum_center_z_m is not None:
            if not math.isfinite(minimum_center_z_m):
                raise ValueError('observed lift height must be finite')
            minimum = max(minimum, minimum_center_z_m)
        if minimum <= 0.0 and minimum_center_z_m is None:
            return
        detected = self._detect_objects()
        match = next(
            (
                item
                for item in detected.detections.detections
                if item.label == attempt.label and item.distance_valid
            ),
            None,
        )
        if match is None:
            raise LiftRedetectionError(
                f'lift verification could not redetect {attempt.label}'
            )
        post_lift = ObjectAttempt(
            object_id=int(match.object_id),
            label=match.label,
            confidence=float(match.confidence),
            distance_m=float(match.distance_m),
        )
        inspected = self._inspect_selected(
            detected.detections.snapshot_id,
            post_lift,
        )
        if inspected is None:
            raise RuntimeError(
                f'lift verification could not reconstruct {attempt.label}'
            )
        center_z = float(inspected.objects.objects[0].obb_pose.position.z)
        self.get_logger().debug(
            f'Lift verification: label={attempt.label} '
            f'center_z={center_z:.3f}m minimum={minimum:.3f}m'
        )
        if not math.isfinite(center_z) or center_z < minimum:
            raise RuntimeError(
                f'{attempt.label} was not retained after lift: '
                f'center_z={center_z:.3f}m minimum={minimum:.3f}m'
            )
        return inspected


    def _open_gripper(self, arm: str) -> None:
        self._command_gripper(
            arm,
            float(self.get_parameter('gripper_open_position_rad').value),
            'open',
        )


    def _command_gripper(
        self,
        arm: str,
        position: float,
        command: str,
        *,
        allow_contact_stall: bool = False,
        motion_seconds: float | None = None,
        contact_start: float | None = None,
    ) -> bool:
        client = self._grippers[arm]
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f'{arm} gripper controller is unavailable')
        goal = FollowJointTrajectory.Goal()
        joint = f'{arm}_gripper_joint'
        start = self._joint_positions.get(joint, math.nan)
        goal.trajectory.joint_names = [joint]
        point = JointTrajectoryPoint()
        point.positions = [position]
        motion_sec = float(self.get_parameter('gripper_motion_sec').value)
        if motion_seconds is not None:
            motion_sec = motion_seconds
        if not math.isfinite(motion_sec) or motion_sec <= 0:
            raise ValueError('Gripper motion duration must be finite and positive')
        point.time_from_start = Duration(
            seconds=motion_sec
        ).to_msg()
        goal.trajectory.points = [point]
        if allow_contact_stall:
            tolerance = JointTolerance(name=joint, position=-1.0)
            goal.path_tolerance = [tolerance]
            goal.goal_tolerance = [tolerance]
        handle = self._future(
            client.send_goal_async(goal),
            5.0,
            f'{arm} gripper {command} goal response',
        )
        if not handle.accepted:
            raise RuntimeError(
                f'{arm} gripper {command} command was rejected'
            )
        timeout_factor = float(self.get_parameter('gripper_wall_timeout_factor').value)
        if not math.isfinite(timeout_factor) or timeout_factor < 1:
            raise ValueError('Gripper wall timeout factor must be finite and >= 1')
        wrapped = self._future(
            handle.get_result_async(),
            max(10.0, timeout_factor * motion_sec + 5.0),
            f'{arm} gripper {command} result',
        )
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            self._spin_once(timeout_sec=0.05)
        actual = self._joint_positions.get(joint, math.inf)
        self.get_logger().debug(f'GRIPPER FEEDBACK arm={arm} operation={command} start={start:.5f} '
            f'actual={actual:.5f} command={position:.5f} residual={actual-position:.5f} '
            f'velocity={self._joint_velocities.get(joint, math.nan):.5f}')
        succeeded = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.error_code
            == FollowJointTrajectory.Result.SUCCESSFUL
        )
        if allow_contact_stall and succeeded and self._wait_for_gripper_contact(
            arm, start if contact_start is None else contact_start, position
        ):
            self.get_logger().info(
                f'{arm} gripper contact detected: actual={actual:.3f}rad '
                f'command={position:.3f}rad'
            )
            return True
        if not succeeded:
            raise RuntimeError(
                f'{arm} gripper {command} failed: status={wrapped.status} '
                f'code={wrapped.result.error_code}'
            )
        actual = self._joint_positions.get(joint, math.inf)
        if abs(actual - position) > 0.05:
            raise RuntimeError(
                f'{arm} gripper {command} feedback error='
                f'{abs(actual - position):.3f} rad'
            )
        return False


    def _wait_for_gripper_contact(self, arm: str, start: float, command: float) -> bool:
        """추가 모터 명령 없이 닫힘 이후 정착을 관측합니다."""
        timeout = float(self.get_parameter('gripper_contact_feedback_timeout_sec').value)
        duration = float(self.get_parameter('gripper_contact_stable_duration_sec').value)
        if not all(math.isfinite(v) and v > 0 for v in (timeout, duration)):
            raise ValueError('Gripper contact feedback durations must be positive and finite')
        joint = f'{arm}_gripper_joint'
        started = time.monotonic()
        previous = getattr(self, '_gripper_feedback_stamps', {}).get(joint, 0)
        stable_since = None
        samples = 0
        while time.monotonic() - started < timeout:
            self._spin_once(timeout_sec=0.05)
            stamp = getattr(self, '_gripper_feedback_stamps', {}).get(joint, 0)
            if stamp <= previous:
                continue
            previous = stamp
            if self._gripper_contact_stalled(arm, start, command):
                if stable_since is None:
                    stable_since = stamp
                samples += 1
                if samples >= 3 and (stamp - stable_since) / 1e9 >= duration:
                    self.get_logger().debug(
                        f'{arm} gripper stable contact: samples={samples} '
                        f'wait_wall_sec={time.monotonic()-started:.3f}')
                    return True
            else:
                stable_since = None
                samples = 0
                actual = self._joint_positions.get(joint, math.inf)
                velocity = self._joint_velocities.get(joint, math.inf)
                limit = float(self.get_parameter('gripper_contact_max_velocity_rad_s').value)
                if abs(actual-command) <= .05 and abs(velocity) <= limit:
                    return False  # 물체 접촉 없이 완전히 닫힌 상태입니다.
        raise RuntimeError(f'{arm} gripper contact did not stabilize within {timeout:.1f}s')


    def _gripper_contact_stalled(
        self, arm: str, start: float, command: float, *, allow_closing_motion: bool = False
    ) -> bool:
        joint = f'{arm}_gripper_joint'
        return is_gripper_contact_stall(
            allow_closing_motion=allow_closing_motion,
            start=start,
            actual=self._joint_positions.get(joint, math.inf),
            command=command,
            velocity=self._joint_velocities.get(joint, math.inf),
            minimum_motion=float(
                self.get_parameter('gripper_contact_min_motion_rad').value
            ),
            minimum_residual=float(
                self.get_parameter('gripper_contact_min_residual_rad').value
            ),
            maximum_velocity=float(
                self.get_parameter(
                    'gripper_contact_max_velocity_rad_s'
                ).value
            ),
        )


    def _retry_gripper_contact(
        self, arm: str, start: float, position: float, contact: bool,
    ) -> tuple[bool, float]:
        for retry in range(getattr(self, '_gripper_retry_steps', 0)):
            if contact:
                break
            step = float(self.get_parameter('gripper_contact_retry_step_rad').value)
            if not math.isfinite(step) or not 0 < step <= .2:
                raise ValueError('Gripper contact retry step must be in (0, 0.2] rad')
            lower = float(self.get_parameter('gripper_close_position_rad').value)
            next_position = max(lower, position - step)
            if next_position >= position:
                break
            position = next_position
            self._grasp_close_override = position
            self.get_logger().info(
                f'Bounded gripper contact retry {retry + 1}: command={position:.4f}rad')
            contact = self._command_gripper(
                arm, position, 'close', allow_contact_stall=True,
                motion_seconds=float(self.get_parameter('gripper_contact_retry_motion_sec').value),
                contact_start=start)
        return contact, position


    def _candidate_close_position(self, candidate) -> float:
        if getattr(self, '_grasp_close_override', None) is not None:
            return self._grasp_close_override
        fallback = float(
            self.get_parameter('gripper_close_position_rad').value
        )
        if bool(self.get_parameter('gripper_force_full_close').value):
            return fallback
        return opening_to_gripper_position(
            required_opening_m=float(candidate.required_opening_m),
            opening_reduction_m=float(
                self.get_parameter(
                    'gripper_close_opening_reduction_m'
                ).value
            ),
            reference_aperture_m=float(
                self.get_parameter('gripper_aperture_reference_m').value
            ),
            reference_position_rad=float(
                self.get_parameter(
                    'gripper_aperture_reference_position_rad'
                ).value
            ),
            aperture_m_per_rad=float(
                self.get_parameter('gripper_aperture_m_per_rad').value
            ),
            minimum_position_rad=fallback,
            maximum_position_rad=float(
                self.get_parameter('gripper_open_position_rad').value
            ),
        )


