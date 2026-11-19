"""Observe Nav2 logs/lifecycle/action status and publish a read-only BT snapshot."""
from __future__ import annotations
import json
from pathlib import Path
from hashlib import sha256

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.clock import Clock, ClockType
from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.msg import TransitionEvent
from nav2_msgs.msg import BehaviorTreeLog
from std_msgs.msg import String
from .bt_trace import TreeTrace


class BtTraceNode(Node):
    """Reconcile Nav2 behavior-tree events with a versioned tree descriptor.

    Subscriptions:
        behavior_tree_log (nav2_msgs/msg/BehaviorTreeLog): Node transitions.
        bt_navigator/transition_event (lifecycle_msgs/msg/TransitionEvent): Lifecycle.
        navigate_to_pose/_action/status (action_msgs/msg/GoalStatusArray): Goal state.

    Publishers:
        navigation/bt_trace (std_msgs/msg/String): JSON tree state for observers.

    QoS:
        Action status and trace use reliable, transient-local depth 1.

    Notes:
        Endpoint names are parameter defaults. Startup requires matching XML and
        manifest hashes; the trace does not control the navigator.
    """
    def __init__(self):
        super().__init__('nav2_bt_trace')
        for key, value in {'manifest_file': '', 'xml_file': '', 'output_topic': 'navigation/bt_trace',
                           'log_topic': 'behavior_tree_log', 'navigator': 'bt_navigator',
                           'action': 'navigate_to_pose'}.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in
             ('manifest_file', 'xml_file', 'output_topic', 'log_topic', 'navigator', 'action')}
        manifest = json.loads(Path(p['manifest_file']).read_text())
        if sha256(Path(p['xml_file']).read_bytes()).hexdigest() != manifest['xml_sha256']:
            raise ValueError('BT descriptor differs from runtime XML')
        self.trace = TreeTrace(manifest)
        self.lifecycle_known = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, p['output_topic'], qos)
        self.create_subscription(BehaviorTreeLog, p['log_topic'], self.logs, 10)
        self.create_subscription(TransitionEvent, p['navigator']+'/transition_event',
                                 self.lifecycle, 10)
        self.create_subscription(GoalStatusArray, p['action']+'/_action/status', self.goals, qos)
        self.create_timer(.25, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish()

    def lifecycle(self, msg):
        self.lifecycle_known = True
        self.trace.lifecycle(msg.goal_state.id == 3)

    def logs(self, msg):
        if not msg.event_log:
            return
        # Nav2 can be activated before discovery. A valid log proves activity,
        # but is not a snapshot of the entire tree's initial state.
        if not self.trace.active:
            if self.lifecycle_known:
                return
            self.trace.lifecycle(True)
        self.trace.apply([dict(node=e.node_name, previous=e.previous_status,
                               status=e.current_status,
                               timestamp=e.timestamp.sec+e.timestamp.nanosec/1e9)
                          for e in msg.event_log], msg.timestamp.sec+msg.timestamp.nanosec/1e9)

    def goals(self, msg):
        active = [s for s in msg.status_list if s.status in (1, 2, 3)]
        if len(active) > 1:
            self.trace.reset('복수 주행 요청: 상태를 특정 목표에 연결할 수 없습니다.')
            self.trace.incomplete = True
            self.trace.goal_id = None
            self.trace.goal_status = None
            return
        candidates = active or [s for s in msg.status_list
                                if bytes(s.goal_info.goal_id.uuid).hex() == self.trace.goal_id]
        if candidates:
            s = candidates[0]
            self.trace.goal(bytes(s.goal_info.goal_id.uuid).hex(), s.status)

    def publish(self):
        self.publisher.publish(String(data=json.dumps(self.trace.snapshot(), ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = BtTraceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
