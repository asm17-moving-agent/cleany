"""A cancellable operator Action sharing the manipulation server's motion owner."""
import threading
import time

from cleany_interfaces.action import ReturnArm
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future

from .arm_return import ArmReturnResult, load_home, run_return, selected_arms


class ArmReturnServer:
    def __init__(self, node, group, pose_file: str):
        self.node, self.pose_file = node, pose_file
        self.handle = self.result = None
        self.abort = threading.Event()
        self.shutting_down = False
        self._action = ActionServer(node, ReturnArm, 'manipulation/return_arm',
            goal_callback=self._goal, handle_accepted_callback=self._accepted,
            execute_callback=self._execute, cancel_callback=self._cancel, callback_group=group)

    def _goal(self, request):
        core = self.node.core
        with core.lock:
            try:
                selected_arms(request.arm)
                home = load_home(self.pose_file)
                if (self.shutting_down or core.arm_return_active or core._recovery_in_progress
                        or (core.record is not None and core.record.result is None)
                        or self.node.operations is None or not core.backend.ready()):
                    return GoalResponse.REJECT
            except (OSError, ValueError, TypeError, KeyError, RuntimeError) as error:
                self.node.get_logger().error(f'Arm return rejected: {error}')
                return GoalResponse.REJECT
            # Historical inhibition stays intact; hardware readiness is checked
            # by run_return before opening. Reserve before another Goal arrives.
            core.arm_return_active = True
            self.home = home
            self.abort = threading.Event()
            return GoalResponse.ACCEPT

    def _accepted(self, handle):
        with self.node.core.lock:
            self.handle = handle
            self.result = Future()
            if self.shutting_down:
                self.abort.set()
            try:
                work = self.node.backend.worker.submit(run_return, self.node.operations,
                    selected_arms(handle.request.arm), self.home, self.abort, self._feedback)
                work.add_done_callback(self._finished)
            except Exception as error:
                self.result.set_result(ArmReturnResult(False, False, str(error)))
            handle.execute()

    def _feedback(self, stage):
        try:
            self.handle.publish_feedback(ReturnArm.Feedback(stage=stage))
        except Exception as error:
            # Loss of a viewer must not interrupt the independent stop path.
            self.node.get_logger().warning(f'Arm return feedback unavailable: {error}')

    def _finished(self, work):
        try:
            result = work.result()
        except Exception as error:
            result = ArmReturnResult(False, False, str(error))
        self.result.set_result(result)

    def _cancel(self, handle):
        with self.node.core.lock:
            if self.handle is not handle or self.result.done():
                return CancelResponse.REJECT
            self.abort.set()
            return CancelResponse.ACCEPT

    async def _execute(self, handle):
        result = await self.result
        with self.node.core.lock:
            if not result.stop_confirmed:
                self.node.core.inhibited = True
            if handle.is_cancel_requested:
                handle.canceled()
            elif result.success:
                handle.succeed()
            else:
                handle.abort()
            # Retain the last Goal ID until the next request: shutdown may enter
            # drain just as this callback finishes and still needs GetResult.
            self.node.core.arm_return_active = False
        return ReturnArm.Result(success=result.success and not handle.is_cancel_requested,
                               stop_confirmed=result.stop_confirmed, message=result.message,
                               recovery_required=result.recovery_required)

    def request_shutdown(self):
        with self.node.core.lock:
            self.shutting_down = True
            if self.node.core.arm_return_active:
                self.abort.set()

    def destroy(self):
        self.request_shutdown()
        self._action.destroy()

    def drain(self, executor, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        receipt = None
        client = self.node.create_client(ReturnArm.Impl.GetResultService,
                                        'manipulation/return_arm/_action/get_result')
        try:
            while self.node.context.ok() and time.monotonic() < deadline:
                with self.node.core.lock:
                    if receipt is None and self.handle is not None:
                        receipt = client.call_async(ReturnArm.Impl.GetResultService.Request(
                            goal_id=self.handle.goal_id))
                    if (not self.node.core.arm_return_active and receipt is not None
                            and receipt.done() and receipt.exception() is None):
                        return receipt.result().result.stop_confirmed
                executor.spin_once(timeout_sec=.05)
            self.node.operations.real.fail_stop()
            self.node.get_logger().error('Arm return shutdown stop unconfirmed')
            return False
        finally:
            self.node.destroy_client(client)
