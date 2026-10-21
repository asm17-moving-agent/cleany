"""ROS topic -> evidence service test, isolated DDS domain, synthetic sensors only."""
import json
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from control_msgs.msg import DynamicJointState, InterfaceValue
from cleany_interfaces.msg import WristTrackingStatus
from cleany_interfaces.srv import CheckManipulation
from std_msgs.msg import String

from cleany_skill_executor.manipulation_evidence_node import ManipulationEvidenceNode


def test_ros_evidence_service_fuses_fresh_identity_load_and_tracking():
    context=Context();rclpy.init(context=context,domain_id=176)
    node=ManipulationEvidenceNode(context=context)
    source=Node('synthetic_evidence_test',context=context)
    executor=MultiThreadedExecutor(num_threads=4,context=context)
    executor.add_node(node);executor.add_node(source)
    motor=source.create_publisher(DynamicJointState,'/joint_state_broadcaster/dynamic_joint_states',10)
    camera=source.create_publisher(WristTrackingStatus,'/perception/wrist_tracking_status',10)
    guard=source.create_publisher(String,'/real_control/status',10)
    client=source.create_client(CheckManipulation,'/manipulation/check_evidence')
    control={'empty':False,'visible':True,'guard':'ready','tracking':True,'missing':False}
    def publish():
        stamp=source.get_clock().now().to_msg()
        rows=[]
        for goal in (1358,1372):
            values={
                'feedback_stamp_ns':float(stamp.sec*10**9+stamp.nanosec),
                'current_limit_raw':250., 'position_raw':float(goal+(6 if control['empty'] else 340)),
                'goal_position_raw':float(goal), 'current_raw':1. if control['empty'] else 18.,
                'load_magnitude_raw':36. if control['empty'] else 200.,
                'torque_enabled':1.,'moving_flag':0.,'motor_status_raw':0.,
                'protocol_error_code':0.,'torque_limit_readback_raw':200.,
                'voltage_raw':120.,'temperature_raw':35.}
            if control['missing']:values.pop('current_raw')
            rows.append(InterfaceValue(interface_names=list(values),values=list(values.values())))
        msg=DynamicJointState(joint_names=['left_gripper_joint','right_gripper_joint'],
                             interface_values=rows);msg.header.stamp=stamp;motor.publish(msg)
        if control['tracking']:
            for arm in ('left','right'):
                tracked=WristTrackingStatus(arm=arm,reference_id='wrist',source_snapshot_id='snapshot',
                    source_object_id=1,valid=True,visible=control['visible'],mask_pixels=400,image_pixels=10000)
                tracked.header.stamp=stamp;camera.publish(tracked)
        guard.publish(String(data=json.dumps({'state':control['guard']})))
    timer=source.create_timer(.02,publish)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    def call(stage=1,arm='left',wrist='wrist'):
        req=CheckManipulation.Request(stage=stage,arm=arm,reference_id='head',wrist_reference_id=wrist,
            source_snapshot_id='snapshot',source_object_id=1,after_stamp_ns=source.get_clock().now().nanoseconds)
        future=client.call_async(req);end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:time.sleep(.01)
        assert future.done()
        return future.result()
    try:
        assert client.wait_for_service(timeout_sec=5)
        time.sleep(.5)
        result=call()
        assert result.success and result.contact_confirmed and result.object_tracked
        assert result.wrist_reference_id=='wrist' and result.reference_id=='head'
        assert call(arm='right').contact_confirmed
        control['empty']=True;time.sleep(.45)
        result=call();assert result.success and not result.contact_confirmed
        control['empty']=False;time.sleep(.45)
        assert not call(wrist='wrong').success
        assert not call(stage=3).success  # No release confirmation invented.
        control['missing']=True;time.sleep(.05)
        assert not call().success
        control['missing']=False;time.sleep(.45)
        control['tracking']=False;time.sleep(.55)
        assert not call().success
        control['guard']='fault';time.sleep(.08)
        assert not call().success
        control['guard']='ready';control['tracking']=True;time.sleep(.1)
        assert not call().success  # Fault cannot be cleared by a ready message.
    finally:
        source.destroy_timer(timer);executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        node.destroy_node();source.destroy_node();context.shutdown()


def test_tracking_failure_cannot_be_cleared_with_a_replayed_frame():
    from types import SimpleNamespace
    from copy import deepcopy
    fake=SimpleNamespace(histories={'left':None},tracking={},tracking_high_water={},lock=threading.Lock())
    valid=WristTrackingStatus(arm='left',reference_id='wrist',source_snapshot_id='snapshot',
        source_object_id=1,valid=True,visible=True,mask_pixels=400,image_pixels=10000)
    valid.header.stamp.sec=10
    ManipulationEvidenceNode.on_tracking(fake,valid)
    ManipulationEvidenceNode.on_tracking(fake,WristTrackingStatus(arm='left',valid=False))
    ManipulationEvidenceNode.on_tracking(fake,valid)
    assert not fake.tracking['left'].valid
    new=deepcopy(valid);new.header.stamp.sec=11
    ManipulationEvidenceNode.on_tracking(fake,new)
    assert fake.tracking['left'].valid


def test_independent_outcome_service_binds_object_destination_and_new_sensor_time():
    from cleany_interfaces.msg import ManipulationOutcome
    from cleany_skill_executor.core.collection import check_evidence
    import pytest
    context = Context()
    rclpy.init(context=context, domain_id=178)
    node = ManipulationEvidenceNode(context=context,
        parameter_overrides=[Parameter('request_timeout_sec', value=.1)])
    source = Node('synthetic_outcome_sensor', context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(node)
    executor.add_node(source)
    outcome = source.create_publisher(ManipulationOutcome, '/perception/manipulation_outcome', 10)
    guard = source.create_publisher(String, '/real_control/status', 10)
    client = source.create_client(CheckManipulation, '/manipulation/check_evidence')
    control = {'emit': True, 'placed': True, 'contact': False}
    def publish():
        guard.publish(String(data='{"state":"ready"}'))
        if control['emit']:
            message = ManipulationOutcome(arm='left', reference_id='head', wrist_reference_id='wrist',
                source_snapshot_id='approved', source_object_id=2, destination_id='bin',
                release_confirmed=True, placement_confirmed=control['placed'],
                contact_confirmed=control['contact'])
            message.header.stamp = source.get_clock().now().to_msg()
            outcome.publish(message)
    timer = source.create_timer(.02, publish)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    def call(**changes):
        request = CheckManipulation.Request(stage=CheckManipulation.Request.PLACEMENT,
            arm='left', reference_id='head', wrist_reference_id='wrist',
            source_snapshot_id='approved', source_object_id=2, destination_id='bin',
            after_stamp_ns=source.get_clock().now().nanoseconds)
        for key, value in changes.items():
            setattr(request, key, value)
        future = client.call_async(request)
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.01)
        assert future.done()
        return request, future.result()
    try:
        assert client.wait_for_service(timeout_sec=5)
        time.sleep(.3)
        request, response = call()
        check_evidence(response, request, source.get_clock().now().nanoseconds, .5)
        assert response.placement_confirmed and response.destination_id == 'bin'
        for changes in ({'destination_id':'other'}, {'source_object_id':1},
                        {'reference_id':'other'}, {'wrist_reference_id':'other'}):
            assert not call(**changes)[1].success
        control['placed'] = False
        request, response = call()
        assert response.success and not response.placement_confirmed
        with pytest.raises(RuntimeError, match='placement'):
            check_evidence(response, request, source.get_clock().now().nanoseconds, .5)
        control['contact'] = True
        request, response = call(stage=CheckManipulation.Request.RELEASE)
        with pytest.raises(RuntimeError, match='release'):
            check_evidence(response, request, source.get_clock().now().nanoseconds, .5)
        control['emit'] = False
        time.sleep(.08)
        assert not call()[1].success  # A replayed outcome cannot renew its sensor time.
    finally:
        source.destroy_timer(timer)
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        node.destroy_node()
        source.destroy_node()
        context.shutdown()
