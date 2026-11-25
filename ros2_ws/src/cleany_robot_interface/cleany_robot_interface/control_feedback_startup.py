"""Wait for stable protected feedback before starting MoveIt."""
import argparse
import json
import math
import time

import rclpy
from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState, SwitchController,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .control_state_guard import DEFAULT_MAX_STATE_AGE_S


STABLE_FEEDBACK_SEC = 1.
READ_ONLY_DISCOVERY_TIMEOUT_SEC = 5.
READ_ONLY_REPLY_TIMEOUT_SEC = 2.
READ_ONLY_ATTEMPTS = 3


class ControlFeedbackStartupProbe(Node):
    """Require fresh guard and joint feedback through a continuous startup window."""

    def __init__(self):
        super().__init__('collection_control_feedback_startup')
        self.guard_state = None
        self.guard_at = 0.
        self.sample_stamp_ns = None
        self.stable_since = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/real_control/status', self.receive_guard, qos)
        self.create_subscription(JointState, '/joint_states', self.receive_joints, qos)

    def receive_guard(self, message):
        try:
            self.guard_state = json.loads(message.data)['state']
        except (KeyError, TypeError, ValueError):
            self.guard_state = 'invalid'
        now = time.monotonic()
        if self.guard_state != 'ready' or now - self.guard_at > DEFAULT_MAX_STATE_AGE_S:
            self.stable_since = None
        self.guard_at = now

    def receive_joints(self, message):
        stamp = message.header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if ns == self.sample_stamp_ns:
            return
        if (self.sample_stamp_ns is not None
                and not 0 < ns - self.sample_stamp_ns <= DEFAULT_MAX_STATE_AGE_S * 1e9):
            self.stable_since = None
        self.sample_stamp_ns = ns

    def ready(self) -> bool:
        if self.guard_state not in (None, 'waiting', 'ready'):
            raise RuntimeError(f'Real-control guard is not ready: {self.guard_state}')
        now = time.monotonic()
        latest = self.sample_stamp_ns
        fresh = (latest is not None
                 and 0 <= self.get_clock().now().nanoseconds - latest <= DEFAULT_MAX_STATE_AGE_S * 1e9)
        if self.guard_state != 'ready' or now - self.guard_at > DEFAULT_MAX_STATE_AGE_S or not fresh:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        return now - self.stable_since >= STABLE_FEEDBACK_SEC


def activate_cold_controllers(node, controllers, deadline, *, phase="both", before_torque_request=None):
    """Activate hardware, command controllers, or both from verified feedback."""
    if phase not in ('both', 'torque', 'controllers'):
        raise ValueError('Unsupported cold activation phase')
    def wait_for(predicate, until=deadline):
        while rclpy.ok():
            remaining = until - time.monotonic()
            if remaining <= 0.:
                break
            rclpy.spin_once(node, timeout_sec=min(.05, remaining))
            node.ready()  # Surface a latched guard before accepting any service result.
            if predicate():
                return True
        return False

    def call(service, name, request, *, read_only=False, require_feedback=False, before_send=None):
        client = node.create_client(service, '/controller_manager/' + name)
        try:
            discovery_deadline = (min(deadline, time.monotonic() + READ_ONLY_DISCOVERY_TIMEOUT_SEC)
                                  if read_only else deadline)
            if not wait_for(client.service_is_ready, discovery_deadline):
                raise RuntimeError('Cold startup timed out finding ' + name)
            if require_feedback:
                if phase != 'torque':
                    node.stable_since = None
                if not wait_for(node.ready):
                    raise RuntimeError('Cold startup timed out waiting for stable feedback before ' + name)
            attempts = READ_ONLY_ATTEMPTS if read_only else 1
            for attempt in range(attempts):
                if not rclpy.ok() or time.monotonic() >= deadline:
                    break
                reply_deadline = (min(deadline, time.monotonic() + READ_ONLY_REPLY_TIMEOUT_SEC)
                                  if read_only else deadline)
                if before_send is not None:
                    before_send()
                future = client.call_async(request)
                if wait_for(future.done, reply_deadline):
                    result = future.result()
                    if result is None:
                        raise RuntimeError(name + ' returned no result')
                    return result
                client.remove_pending_request(future)
                if attempt + 1 < attempts and rclpy.ok() and time.monotonic() < deadline:
                    node.get_logger().warning(f'No {name} reply; retrying read-only request')
            raise RuntimeError('Cold startup timed out waiting for ' + name + ' reply')
        finally:
            node.destroy_client(client)

    def require_controllers(state):
        reply = call(ListControllers, 'list_controllers', ListControllers.Request(), read_only=True)
        states = {controller.name: controller.state for controller in reply.controller}
        if states.get('joint_state_broadcaster') != 'active' or any(states.get(name) != state for name in controllers):
            raise RuntimeError(f'Expected preloaded controllers {state} and active broadcaster: {states}')

    if phase == 'torque':
        # The broadcaster starts after the inactive command controllers are preloaded.
        require_stable_feedback(node, deadline)
    require_controllers('inactive')
    components = call(ListHardwareComponents, 'list_hardware_components',
                      ListHardwareComponents.Request(), read_only=True)
    expected = 'active' if phase == 'controllers' else 'inactive'
    if [part.state.label for part in components.component if part.name == 'FeetechSystem'] != [expected]:
        raise RuntimeError(f'Expected configured, {expected} FeetechSystem before cold activation')
    if phase != 'controllers':
        request = SetHardwareComponentState.Request(name='FeetechSystem')
        request.target_state.label = 'active'
        request.target_state.id = request.target_state.PRIMARY_STATE_ACTIVE
        result = call(SetHardwareComponentState, 'set_hardware_component_state', request,
                      require_feedback=True, before_send=before_torque_request)
        if not result.ok or result.state.id != request.target_state.id:
            raise RuntimeError('FeetechSystem activation did not reach active state')
        print('Motor hardware active after controller preloading', flush=True)

    if phase == 'torque':
        node.stable_since = None
        require_stable_feedback(node, deadline)
        return

    switch = SwitchController.Request()
    switch.activate_controllers = list(controllers)
    switch.strictness = SwitchController.Request.STRICT
    switch.timeout.sec = max(1, math.ceil(deadline - time.monotonic()))
    if not call(SwitchController, 'switch_controller', switch, require_feedback=True).ok:
        raise RuntimeError('Preloaded command controller group activation failed')
    require_controllers('active')
    node.stable_since = None  # The MoveIt gate must observe feedback after group activation.


def require_stable_feedback(node, deadline):
    """Require a continuous window by the monotonic deadline in seconds."""
    while rclpy.ok():
        remaining = deadline - time.monotonic()
        if remaining <= 0.:
            break
        rclpy.spin_once(node, timeout_sec=min(.05, remaining))
        if node.ready():
            return
    raise RuntimeError('Stable protected joint feedback was not confirmed')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=30.)
    parser.add_argument('--cold-controllers', nargs='+', metavar='NAME')
    options = parser.parse_args(args)
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    rclpy.init(args=[])
    node = ControlFeedbackStartupProbe()
    deadline = time.monotonic() + options.timeout
    try:
        if options.cold_controllers:
            activate_cold_controllers(node, options.cold_controllers, deadline)
        require_stable_feedback(node, deadline)
        print('Protected joint feedback stable; starting MoveIt', flush=True)
        return 0
    except RuntimeError as error:
        print(f'Controller feedback startup failed: {error}', flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
