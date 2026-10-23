"""Run the native tree and production Jetson operations over synthetic physical ports."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import time

import pytest
from cleany_interfaces.action import InspectScene, SelectReachableGrasp
from cleany_interfaces.msg import DetectedObject3D, DetectedObject3DArray, GraspCandidate
from cleany_interfaces.srv import CheckManipulation, ObserveObjectReference, ObserveWristTarget, PlanGrasp
from geometry_msgs.msg import Pose, Vector3
from sensor_msgs.msg import JointState
from rclpy.time import Time
from cleany_manipulation_bt._bt_runner import Runner
from cleany_manipulation_bt.backend import WorkerBackend
from cleany_manipulation_bt.core import ExecutionCore
from cleany_manipulation_bt.store import BTExecutionStore
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.manipulation.models import Error, ObjectState, Placement, Status
from test_jetson_operations import operation


@pytest.mark.parametrize('failure', ['', 'grasp', 'placement'])
@pytest.mark.parametrize('skill', ['collect_trash', 'collect_lost_item'])
@pytest.mark.parametrize('return_to_pick', [False, True])
def test_native_jetson_skill_preserves_target_and_requires_real_port_evidence(operation, tmp_path, failure, skill, return_to_pick):
    node, context, snapshot = operation
    if skill == 'collect_lost_item':
        snapshot.detections[1].label = 'computer mouse'
        snapshot.detections[1].sorting_category = 'lost_item'

        context.goal = replace(context.goal, skill_name=skill, destination_id='lost_bin')
        node.config['label_destinations'] = {'computer mouse':'lost_bin'}
    if return_to_pick:
        context.goal = replace(context.goal, destination_id='return_to_pick')
        node.config.update(operation_mode='pick_return_release',
            temporary_contact_verification='raw_feedback', temporary_placement_verification='assume_success',
            label_destinations={'*': 'return_to_pick'})
    node.config.update(frame='base_link', commissioning_verified=True, lift_height_m=.05,
        cartesian_tolerance_m=.005, evidence_max_age_sec=.5, wrist_observation_enabled=True,
        arms={'left': {'gripper': {'measured':True, 'closed_joint_rad':0., 'open_joint_rad':1.,
            'maximum_width_m':.08}, 'home_joint_rad':[0.]*5, 'bins':{context.goal.destination_id:[.1]*5}}})
    values = {'bt_operation_timeout_sec':1., 'bt_selection_timeout_sec':1.,
              'bt_verification_timeout_sec':.03}
    old_value = node.value
    node.value = lambda name: values[name] if name in values else old_value(name)
    node.get_parameter = lambda name: NS(value=.01)
    node.execution_context = None
    node.current_node, node.deadline, node._stopping = '', 0., False
    node.submissions, node.handles, node.tracked_clients = [], [], []
    node.carrying = None
    node.events, node.arm_config, node.wrist_request, node.wrist_reference_id = [], None, None, ''
    trace = []
    node.wrist_cameras = NS(activate=lambda arm: trace.append(f'camera:{arm}:on'),
                           close=lambda: trace.append('camera:off'),
                           request_close=lambda: trace.append('camera:off'))
    pickup_positions, release_positions, evidence_stages = [], [], []
    def stamp():
        return time.time_ns()
    node.get_clock = lambda: NS(now=lambda: Time(nanoseconds=stamp()))
    node.event = lambda name, **kwargs: trace.append(name)
    state = JointState(name=list(ARM_JOINT_NAMES['left'])+['left_gripper_joint'], position=[0.]*6)
    node.measured = lambda: deepcopy(state)
    node.real = NS(fault='', monitor=lambda: deepcopy(state), value=lambda _: .02,
                   verify=lambda _: {n:NS(check=lambda _:None) for n in state.name},
                   closed_contact_position=lambda *_: 0., wait_for_contact=lambda *_:None)
    node.scene_monitor = NS(wait=lambda:None)
    node.scene = NS(restore=lambda:None, begin=lambda *args:None, allow_contacts_for=lambda _:None)
    node._inspection = NS(wait_for_server=lambda **kwargs:True)
    node._selection = NS(wait_for_server=lambda **kwargs:True)
    node._grasp = NS(wait_for_service=lambda **kwargs:True)
    node._snapshot = NS(wait_for_service=lambda **kwargs:True)
    node.reference = object()
    node.wrist = object()
    pose = Pose()
    pose.position.z, pose.orientation.w = .4, 1.
    obj = DetectedObject3D(object_id=2, label=snapshot.detections[1].label, obb_pose=deepcopy(pose),
                           obb_size=Vector3(x=.07,y=.07,z=.1))
    inspected = InspectScene.Result(success=True,
        objects=DetectedObject3DArray(snapshot_id='approved', objects=[obj]))
    inspected.objects.header.frame_id = 'base_link'
    candidate = GraspCandidate(snapshot_id='approved',object_id=2, required_opening_m=.04, target_object=obj)
    selected = SelectReachableGrasp.Result(success=True, selected_arm='left',
        selected_candidate=candidate, gripper_joint_position=.5, selected_candidate_index=0)
    selected.grasp_joint_state = JointState(name=list(ARM_JOINT_NAMES['left']), position=[0.]*5)
    old_call = node.call
    def evidence(request):
        evidence_stages.append(request.stage)
        response = CheckManipulation.Response(success=True, arm=request.arm,
            reference_id=request.reference_id, wrist_reference_id=request.wrist_reference_id,
            source_snapshot_id=request.source_snapshot_id, source_object_id=request.source_object_id,
            destination_id=request.destination_id,
            contact_confirmed=request.stage in (request.GRASP,request.LIFT), object_tracked=True,
            release_confirmed=request.stage in (request.RELEASE,request.PLACEMENT),
            placement_confirmed=request.stage==request.PLACEMENT and failure!='placement')
        response.header.stamp.sec,response.header.stamp.nanosec=divmod(stamp(),10**9)
        return response
    def outcome_async(request):
        from concurrent.futures import Future
        future=Future()
        future.set_result(evidence(request))
        return future
    node.evidence = NS(wait_for_service=lambda **kwargs:True, call_async=outcome_async)
    def call(client, request, *args):
        if client is node.wrist:
            response = ObserveWristTarget.Response(success=True, status=ObserveWristTarget.Response.OK,
                reference_id=request.reference_id or 'wrist-pinned', source_snapshot_id='approved', source_object_id=2)
            response.header.frame_id = 'base_link'
            response.header.stamp = Time(nanoseconds=stamp()).to_msg()
            return response
        if client is node.reference:
            return ObserveObjectReference.Response(success=True, reference_id='pinned',
                source_snapshot_id='approved', source_object_id=2)
        if client is node._grasp:
            assert request.snapshot_id=='approved' and request.object_id==2
            return PlanGrasp.Response(success=True,candidates=[candidate])
        if client is node.evidence:
            return evidence(request)
        return old_call(client,request)
    node.call = call
    def action(client, goal, *args, **kwargs):
        if client is node._inspection:
            assert goal.snapshot_id=='approved' and goal.selected_object_id==2
            return inspected
        assert goal.candidates[0].object_id==2 and goal.required_arm=='left'
        return selected
    node.run_action = action
    observed = ObserveObjectReference.Response(observed_center=deepcopy(pose.position), observed_extent=deepcopy(obj.obb_size))
    observed.header.frame_id = 'base_link'
    node.observe = lambda *args: deepcopy(observed)
    node.observe_arrival = lambda *args: deepcopy(observed)
    node.execute_pregrasp = lambda _: stamp()
    node.pose = lambda _: deepcopy(pose)
    def linear(arm, target, stage, *args, **kwargs):
        pose.position = deepcopy(target.position)
        if stage == 'approach':
            state.position = [.2, .3, .4, .5, .6, state.position[-1]]
            pickup_positions.append(tuple(state.position[:5]))
        elif stage == 'lift':
            state.position = [.8]*5 + [state.position[-1]]
    node.linear = linear
    node.move_joint_pose = lambda arm,positions,stage: (
        setattr(state,'position',list(positions)+[state.position[-1]]), trace.append(stage))
    # Replace the synchronized return port as well as the legacy joint port.
    node.move_arm_return = lambda arm,positions,closed,**options: (
        setattr(state,'position',list(positions)+[closed]), trace.append('return'))
    def jaw(arm, position, stage, **options):
        trace.append(stage)
        if stage == 'release':
            release_positions.append(tuple(state.position[:5]))
    node.jaw = jaw
    def close(_, report):
        trace.append('close')
        if failure=='grasp':
            raise RuntimeError('no contact evidence')
        report.update(grasp_contact_confirmed=True, grasp_assumed=False)
    node.close_until_contact = close
    node.attach = lambda *args: setattr(node,'carrying',{'release_confirmed':False, 'grasp_tcp_pose':deepcopy(pose)})
    node.detach = lambda: setattr(node,'carrying',None)
    node.release_grasp_scene_freeze = lambda:None
    node._assess_stationary = lambda *args,**kwargs:True
    store = BTExecutionStore(str(tmp_path/'jetson.sqlite3'))
    backend = WorkerBackend(node.execute,quiescent=node.quiescent)
    core = ExecutionCore(backend,store,str(Path(__file__).parents[1]/'trees/collect_trash_mujoco.xml'),
                         Runner,execution_profile='jetson', allow_assumed_placement=return_to_pick)
    try:
        assert core.accept(context.goal)[0]
        deadline=time.monotonic()+5.
        while core.record.result is None and time.monotonic()<deadline:
            core.tick()
            time.sleep(.001)
        result=core.record.result
        assert result is not None and result.execution_profile=='jetson'
        assert store.get(context.goal.execution_id).result==result
        assert result.stop_confirmed
        assert trace.index('camera:left:on') < trace.index('close')
        assert 'camera:right:on' not in trace
        assert 'camera:off' in trace
        if not failure or (return_to_pick and failure == 'placement'):
            expected = Placement.ASSUMED if return_to_pick else Placement.CONFIRMED
            assert result.status==Status.SUCCESS and result.placement_state==expected, result
            assert result.object_state==ObjectState.LEFT_GRIPPER and result.arm_recovered
            destination = 'return_to_pickup' if return_to_pick else 'transport'
            assert trace.index('close')<trace.index(destination)<trace.index('release')<trace.index('return')
            assert trace.index('camera:off') < trace.index(destination)
            if return_to_pick:
                assert release_positions == pickup_positions
                assert release_positions[0] != (0.,)*5
                assert tuple(state.position[:5]) == (0.,)*5
                assert node.report['grasp_verified'] and not node.report['grasp_assumed']
                assert CheckManipulation.Request.PLACEMENT not in evidence_stages
                assert 'release and placement assumed' in result.message
        else:
            assert result.status!=Status.SUCCESS
            if failure=='grasp':
                assert result.error_code==Error.GRASP_FAILED and result.failed_substage=='GraspObject', result
                assert 'transport' not in trace and 'release' not in trace
                assert 'return_to_pickup' not in trace
            else:
                assert result.error_code==Error.VERIFICATION_TIMEOUT, result
                assert result.placement_state!=Placement.CONFIRMED and result.arm_recovered
    finally:
        backend.close()
        store.close()
