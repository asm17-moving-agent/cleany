"""Read-only evidence service. No serial opens, motor writes, or motion clients."""
import json
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from cleany_interfaces.msg import ManipulationOutcome, WristTrackingStatus
from copy import deepcopy
from cleany_interfaces.srv import CheckManipulation
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from .core.manipulation_evidence import TrackingConfig, RawContactHistory, TrackingSample
from .core.gripper_contact import load_contact_criteria


class ManipulationEvidenceNode(Node):
    """Fuse identified wrist observations with raw motor contact evidence.

    Services:
        /manipulation/check_evidence (CheckManipulation): Read-only server;
            freshness and identity bind evidence to the requested operation.

    Notes:
        Guard faults remain latched. Observations never command the hardware
        or establish calibrated force or torque.
    """
    def __init__(self, **kwargs):
        super().__init__('manipulation_evidence', **kwargs)
        self.declare_parameter('request_timeout_sec', .8)
        self.timeout = float(self.get_parameter('request_timeout_sec').value)
        if not math.isfinite(self.timeout) or not 0 < self.timeout <= 2:
            raise ValueError('Invalid request timeout')
        self.declare_parameter('contact_rule_file', str(Path(get_package_share_directory(
            'cleany_skill_executor'))/'config/gripper_contact_decision.yaml'))
        self.declare_parameter('tracking_maximum_age_sec', .5)
        self.declare_parameter('tracking_maximum_skew_sec', .2)
        criteria = load_contact_criteria(self.get_parameter('contact_rule_file').value)
        self.histories = {arm: RawContactHistory(c, TrackingConfig(
            maximum_age_sec=float(self.get_parameter('tracking_maximum_age_sec').value),
            maximum_skew_sec=float(self.get_parameter('tracking_maximum_skew_sec').value)))
            for arm, c in criteria.items()}
        self.tracking = {}
        self.tracking_high_water = {}
        self.outcomes = {}
        self.guard = None
        self.guard_received = 0.
        self.guard_fault = ''
        self.lock = threading.Lock()
        self.group = ReentrantCallbackGroup()
        self.subscriptions_ = [
            self.create_subscription(DynamicJointState,'/joint_state_broadcaster/dynamic_joint_states',self.on_motor,20,callback_group=self.group),
            self.create_subscription(WristTrackingStatus,'/perception/wrist_tracking_status',self.on_tracking,10,callback_group=self.group),
            self.create_subscription(String,'/real_control/status',self.on_guard,10,callback_group=self.group),
            self.create_subscription(ManipulationOutcome,'/perception/manipulation_outcome',self.on_outcome,10,callback_group=self.group)]
        self.service = self.create_service(CheckManipulation,'/manipulation/check_evidence',self.check,callback_group=self.group)

    def on_guard(self, message):
        with self.lock:
            try:
                data=json.loads(message.data)
                if not isinstance(data,dict):
                    raise ValueError('Malformed guard')
                self.guard=data
                self.guard_received=time.monotonic()
                if data.get('state')=='fault':
                    self.guard_fault=data.get('reason') or 'hardware_fault'
            except (ValueError,TypeError):
                self.guard_fault='malformed_guard'

    def on_motor(self, message):
        with self.lock:
            if len(message.joint_names)!=len(message.interface_values) or len(set(message.joint_names))!=len(message.joint_names):
                for history in self.histories.values():
                    history.invalidate('malformed_dynamic_joint_state')
                return
            rows=dict(zip(message.joint_names,message.interface_values))
            for arm,history in self.histories.items():
                row=rows.get(arm+'_gripper_joint')
                if row is None or len(row.interface_names)!=len(row.values) or len(set(row.interface_names))!=len(row.interface_names):
                    history.invalidate('gripper_telemetry_missing');continue
                values=dict(zip(row.interface_names,row.values))
                history.observe(values)

    def on_tracking(self, message):
        if message.arm not in self.histories:
            return
        sample=TrackingSample(message.header.stamp.sec*10**9+message.header.stamp.nanosec,
            message.arm,message.reference_id,message.source_snapshot_id,message.source_object_id,
            message.valid,message.visible,message.mask_pixels,message.image_pixels)
        with self.lock:
            # Keep the high-water stamp even across zero-stamp failure notices.
            high_water=self.tracking_high_water.get(message.arm,0)
            if not message.valid:
                self.tracking[message.arm]=sample
            elif sample.stamp_ns>high_water:
                self.tracking[message.arm]=sample
                self.tracking_high_water[message.arm]=sample.stamp_ns

    @staticmethod
    def outcome_key(message):
        return (message.arm, message.reference_id, message.wrist_reference_id,
                message.source_snapshot_id, message.source_object_id, message.destination_id)

    def on_outcome(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp <= 0 or stamp > self.get_clock().now().nanoseconds:
            return
        key = self.outcome_key(message)
        with self.lock:
            previous = self.outcomes.get(key)
            old = 0 if previous is None else previous.header.stamp.sec * 10**9 + previous.header.stamp.nanosec
            if stamp > old:
                self.outcomes[key] = deepcopy(message)
                if len(self.outcomes) > 100:
                    del self.outcomes[next(iter(self.outcomes))]

    def check_outcome(self, request, response):
        deadline = time.monotonic() + self.timeout
        while True:
            with self.lock:
                if self.guard_fault or not self.guard or self.guard.get('state') != 'ready' or time.monotonic()-self.guard_received > .25:
                    response.message = self.guard_fault or 'hardware_guard_unavailable'
                    return response
                sample = self.outcomes.get(self.outcome_key(request))
                stamp = 0 if sample is None else sample.header.stamp.sec * 10**9 + sample.header.stamp.nanosec
                fresh = (sample is not None and request.after_stamp_ns < stamp <= self.get_clock().now().nanoseconds
                         and self.get_clock().now().nanoseconds-stamp <= self.histories[request.arm].config.maximum_age_sec * 1e9)
                if fresh:
                    response.header = deepcopy(sample.header)
                    response.success = True
                    response.contact_confirmed = sample.contact_confirmed
                    response.release_confirmed = sample.release_confirmed
                    response.placement_confirmed = sample.placement_confirmed
                    response.message = 'independent_sensor_outcome'
                    return response
            if time.monotonic() >= deadline:
                response.message = 'independent_sensor_outcome_missing_or_stale'
                return response
            time.sleep(.01)

    def check(self, request, response):
        response.arm=request.arm
        response.reference_id=request.reference_id
        response.wrist_reference_id=request.wrist_reference_id
        response.source_snapshot_id=request.source_snapshot_id
        response.source_object_id=request.source_object_id
        response.destination_id=request.destination_id
        if (request.arm not in self.histories or not request.reference_id or not request.source_snapshot_id
                or request.source_object_id<1 or request.after_stamp_ns<=0
                or request.after_stamp_ns>self.get_clock().now().nanoseconds):
            response.message='invalid_request_identity'
            return response
        if request.stage in (request.RELEASE, request.PLACEMENT):
            if request.stage == request.PLACEMENT and not request.destination_id:
                response.message = 'destination_required_for_placement'
                return response
            return self.check_outcome(request, response)
        if request.stage not in (request.GRASP, request.LIFT):
            response.message = 'unknown_evidence_stage'
            return response
        deadline=time.monotonic()+self.timeout
        while True:
            with self.lock:
                if self.guard_fault or not self.guard or self.guard.get('state')!='ready' or time.monotonic()-self.guard_received>.25:
                    response.message=self.guard_fault or 'hardware_guard_unavailable'
                    return response
                evidence=self.histories[request.arm].evaluate(self.tracking.get(request.arm),
                    arm=request.arm,wrist_reference_id=request.wrist_reference_id,
                    snapshot_id=request.source_snapshot_id,object_id=request.source_object_id,
                    after_ns=request.after_stamp_ns,now_ns=self.get_clock().now().nanoseconds)
            if evidence.success or time.monotonic()>=deadline:
                response.success=evidence.success
                response.message=evidence.reason
                response.header.stamp.sec,response.header.stamp.nanosec=divmod(evidence.stamp_ns,10**9)
                response.contact_confirmed=evidence.contact
                response.object_tracked=evidence.tracked
                response.release_confirmed=False
                return response
            time.sleep(.01)


def main(args=None):
    rclpy.init(args=args)
    node=ManipulationEvidenceNode()
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown();node.destroy_node();rclpy.try_shutdown()
