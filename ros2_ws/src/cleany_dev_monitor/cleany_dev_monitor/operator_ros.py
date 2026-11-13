"""Optional operator ROS node, separate from the observation collector."""

import json
import os
import threading
import time
from uuid import uuid4

from cleany_interfaces.srv import CancelManipulation, ConfirmManipulationRecovery
from cleany_interfaces.action import InspectScene
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class OperatorROS:
    def __init__(self):
        # The monitor and detached command workers coexist in the same DDS graph.
        self.node = Node(f'cleany_dev_monitor_operator_{os.getpid()}_{uuid4().hex[:8]}')
        self.state_client = self.node.create_client(Trigger, '/manipulation/control_state')
        self.cancel_client = self.node.create_client(CancelManipulation, '/manipulation/cancel')
        self.recover_client = self.node.create_client(ConfirmManipulationRecovery, '/manipulation/confirm_recovery')
        self.inspection = ActionClient(self.node, InspectScene, '/perception/inspect_scene')
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    @staticmethod
    def call(client, request, timeout=3.):
        if not client.wait_for_service(timeout_sec=.5):
            raise RuntimeError('실행기의 제어 상태 서비스가 없습니다. 실행기 준비 또는 업데이트를 확인하세요.')
        future = client.call_async(request)
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        if not ready.wait(timeout):
            raise TimeoutError('응답을 확인하지 못했습니다. 상태를 다시 조회하세요.')
        return future.result()

    def snapshot(self):
        response = self.call(self.state_client, Trigger.Request())
        if not response.success:
            raise RuntimeError(response.message)
        data = json.loads(response.message)
        if data.get('schema') != 1:
            raise RuntimeError('지원하지 않는 실행기 상태입니다.')
        return data

    def cancel(self, execution_id, mode):
        response = self.call(self.cancel_client, CancelManipulation.Request(execution_id=execution_id, mode=mode))
        return dict(accepted=response.accepted, message=response.message)

    def recover(self, execution_id, revision, note):
        response = self.call(self.recover_client, ConfirmManipulationRecovery.Request(
            execution_id=execution_id, expected_revision=revision,
            robot_stopped_confirmed=True, gripper_empty_confirmed=True, operator_note=note), timeout=20.)
        return dict(success=response.success, inhibited=response.inhibited, message=response.message)

    @staticmethod
    def wait(future, timeout=10.):
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        if not ready.wait(timeout):
            raise TimeoutError('인식 응답을 확인하지 못했습니다.')
        return future.result()

    def inspect_scene(self, cancelled, *, tracking_session_id='', timeout=180.):
        if not self.inspection.wait_for_server(timeout_sec=5.):
            raise RuntimeError('장면 인식 서버가 없습니다.')
        handle = self.wait(self.inspection.send_goal_async(InspectScene.Goal(
            query='Detect visible movable objects on the tabletop.', tracking_session_id=tracking_session_id)))
        if not handle.accepted:
            raise RuntimeError('장면 인식 요청이 거절되었습니다.')
        result = handle.get_result_async()
        deadline = time.monotonic() + timeout
        try:
            while not result.done():
                if cancelled() or time.monotonic() > deadline:
                    raise InterruptedError('장면 인식을 중단합니다.')
                time.sleep(.1)
            inspected = result.result().result
            if not inspected.success:
                raise RuntimeError(inspected.message)
            return inspected.detections
        finally:
            if not result.done():
                self.wait(handle.cancel_goal_async(), 5.)
                self.wait(result, 25.)

    def inspect(self, cancelled):
        scene = self.inspect_scene(cancelled)
        return dict(snapshot_id=scene.snapshot_id,
                    objects=[dict(object_id=item.object_id, label=item.label,
                                  confidence=item.confidence, category=item.sorting_category)
                             for item in scene.detections[:64]])

    def close(self):
        self.executor.shutdown(timeout_sec=3.)
        self.thread.join(timeout=3.)
        self.inspection.destroy()
        self.node.destroy_node()
