"""In-process replay of the production collection flow with synthetic transports.

GPU proposals, perception, MoveIt services and controller transport are test
doubles. Production selection, path checks, collection stages, hardware retiming
and FollowJointTrajectory message construction run unchanged. No DDS/USB/CUDA.
"""
from contextlib import contextmanager
from copy import deepcopy
import json
import math
from pathlib import Path
import threading
from types import MethodType, SimpleNamespace as NS

import pytest
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory, MoveItErrorCodes
from cleany_interfaces.action import InspectScene, ValidatePick
from cleany_interfaces.msg import DetectedObject2D, DetectedObject3D, GraspCandidate
from cleany_interfaces.srv import CheckManipulation, PlanGrasp
from cleany_skill_executor.core.gripper_contact import (
    ContactDecision, ContactState, load_contact_criteria,
)

from cleany_skill_executor.collection_pipeline import CollectionPipeline
from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline
from cleany_skill_executor.hardware_execution import HardwareExecution
from cleany_skill_executor.core.hardware_execution import PhysicalJointLimit
from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, GraspSelectionConfig, GraspSelector,
)
from cleany_skill_executor.core.fixed_jaw_clearance import Bounds, ClearanceConfig
from cleany_skill_executor.physical_grasp_selection_node import GraspSelectionNode
from cleany_skill_executor.validate_pick_node import ValidatePickNode
from cleany_skill_executor.planning_scene import SceneAwarePort
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig
from test_hybrid_adapter import make_adapter, pose_message, Service, completed


class ReplayGoal:
    def __init__(self, request):
        self.request = request
        self.is_cancel_requested = False
        self.terminal = ''

    def publish_feedback(self, message):
        pass

    def succeed(self):
        self.terminal = 'succeed'

    def abort(self):
        self.terminal = 'abort'

    def canceled(self):
        self.terminal = 'canceled'


def replay_validation(adapter, measured, now, scene, trace, *, fixed_jaw_clearance=False,
                      direct_grasp=False, direct_grasp_shortcut=False):
    """Run both production action callbacks using native ROS message contracts."""
    selector = GraspSelectionNode.__new__(GraspSelectionNode)
    selector._adapter = adapter
    selector._description_error = ''
    selector._requires_description = True
    selector._description_ready = True
    selector._visibility_tf = NS(lookup_transform=lambda *args,**kwargs:NS(
        header=NS(stamp=now().to_msg()),
        transform=NS(translation=Vector3(x=.8,y=.4,z=.8))))
    selector._current_joint_state = measured
    selector._scene = scene
    selector._scene_port = SceneAwarePort(adapter, scene)
    selector._selector = GraspSelector(selector._scene_port, GraspSelectionConfig(
        maximum_candidates=8, gripper_closed_joint_rad=0., gripper_open_joint_rad=1.,
        asymmetric_gripper_enabled=fixed_jaw_clearance,
        fixed_jaw_tcp_offset_m=.008 if fixed_jaw_clearance else 0.,
        fixed_jaw_additional_bias_m=.010 if fixed_jaw_clearance else 0.,
        pregrasp_motion_enabled=not direct_grasp,
        direct_grasp_shortcut_enabled=direct_grasp_shortcut,
        require_gripper_closure_clearance=True, require_approach_corridor=True,
        require_pregrasp_visibility=True))
    if fixed_jaw_clearance:
        selector._fixed_jaw_config = ClearanceConfig(.005, .010)
        jaw = Bounds((.008, -.006, -.005), (.016, .06, .005))
        selector._fixed_jaw_models = {'left': jaw, 'right': jaw}
    selector._goal_lock = threading.Lock()
    selector._goal_active = True
    selector._sensor_scene = NS(wait=lambda **kwargs:None, check=lambda:None)
    selector_parameters = {'action_timeout_sec':600., 'planning_frame':'base_link',
        'robot_description_startup_timeout_sec':30.,
        'grasp_approach_offset_m':0., 'grasp_lateral_offset_m':0.,
        'grasp_use_aperture_centering':False, 'asymmetric_gripper_enabled':fixed_jaw_clearance,
        'deeper_grasp_labels':[''], 'deeper_grasp_offsets_m':[0.],
        'visibility_camera_frame':'head_camera_rgb_optical_frame',
        'visibility_camera_max_age_sec':.5, 'visibility_padding_m':.003,
        'visibility_cone_sides':16}
    selector.get_parameter = lambda key:NS(value=selector_parameters[key])
    selector.get_clock = lambda:NS(now=now)
    selector.get_logger = lambda:NS(info=trace.append, error=lambda *_:None)
    selector.create_timer = lambda *args, **kwargs: NS()
    selector.destroy_timer = lambda timer: None

    coordinator = ValidatePickNode.__new__(ValidatePickNode)
    coordinator._inspection, coordinator._selection = object(), object()
    coordinator._reference = Service(lambda request:NS(
        success=True, reference_id='synthetic_reference',
        source_snapshot_id=request.source_snapshot_id,
        source_object_id=request.source_object_id))
    coordinator._wait_future = lambda future,*args:future.result()
    coordinator._goal_lock = threading.Lock()
    coordinator._goal_active = True
    coordinator._sensor_scene = selector._sensor_scene
    coordinator._debug_messages = lambda:(None,None)
    coordinator._publish_filter_debug = lambda *args:None
    coordinator.get_logger = selector.get_logger
    parameters = {'action_timeout_sec':600., 'inspection_timeout_sec':60.,
        'selection_timeout_sec':480., 'grasp_timeout_sec':60., 'denylist':['keyboard'],
        'workspace_roi_pixels':[-1.]*4, 'maximum_smaller_extent_m':.095,
        'pin_object_reference':True, 'reference_cleanup_timeout_sec':5.,
        'required_arm':'left', 'evidence_output_directory':''}
    coordinator.get_parameter = lambda key:NS(value=parameters[key])

    obj = DetectedObject3D(object_id=2, label='synthetic_box',
        obb_pose=pose_message((.4,.2,.3)), obb_size=Vector3(x=.04,y=.04,z=.04))
    candidates = []
    for i in range(8):
        candidate = GraspCandidate(snapshot_id='synthetic_capture', object_id=2,
            tcp_pose=pose_message((.4+i*.001-(.008 if fixed_jaw_clearance else 0.),.2,.3)),
            approach_direction=Vector3(x=0.,y=-1.,z=0.),
            required_opening_m=.04, score=1.-i*.01, target_object=deepcopy(obj))
        candidate.header.frame_id = 'base_link'
        candidate.header.stamp = now().to_msg()
        candidates.append(candidate)

    def inspect_or_select(client, goal, *args, **kwargs):
        if client is coordinator._selection:
            trace.append('production_grasp_selection_callback')
            GraspSelectionNode._validate_candidates(goal.candidates,'base_link')
            handle = ReplayGoal(goal)
            selected = selector._execute(handle)
            assert handle.terminal == ('succeed' if selected.success else 'abort'), selected.message
            return selected
        assert client is coordinator._inspection
        response = InspectScene.Result(success=True)
        response.detections.snapshot_id = 'synthetic_capture'
        response.detections.header.stamp = now().to_msg()
        if not goal.snapshot_id:
            trace.append('synthetic_detector_response')
            response.detections.detections = [
                DetectedObject2D(object_id=1,label='keyboard',median_depth_m=.1,
                    x_max=20.,y_max=20.,estimated_width_m=.04,estimated_height_m=.04),
                DetectedObject2D(object_id=2,label='synthetic_box',median_depth_m=.5,
                    x_max=40.,y_max=40.,estimated_width_m=.04,estimated_height_m=.04)]
        else:
            # Production filtering must reject the nearer denied object before YOLOE-seg.
            assert goal.selected_object_id == 2
            trace.append('synthetic_yoloe_mask_response_for_filtered_target')
            response.objects.header.frame_id = 'base_link'
            response.objects.objects = [deepcopy(obj)]
            response.target_cloud.header.frame_id = 'camera_optical_frame'
            response.context_cloud.header.frame_id = 'camera_optical_frame'
        return response
    coordinator._run_action = inspect_or_select
    def anygrasp(request, *args):
        assert request.object_id == 2 and request.target_object_header.frame_id == 'base_link'
        trace.append('synthetic_anygrasp_eight_candidates')
        return PlanGrasp.Response(success=True,candidates=candidates)
    coordinator._run_grasp_service = anygrasp
    handle = ReplayGoal(ValidatePick.Goal(query='synthetic box'))
    trace.append('production_validate_pick_callback')
    selected = coordinator._execute(handle)
    assert handle.terminal == ('succeed' if selected.success else 'abort'), selected.message
    assert selected.selected_detection.object_id == 2 and len(selected.candidates) == 8
    return selected


def replay_pipeline(monkeypatch, *, primary_empty=False, fail_controller=False,
                    visual_servo=False, reject_collision=False, stationary_target=False,
                    approach_obstacle=False, fixed_jaw_clearance=False,
                    approach_collision_check=True, direct_grasp=False,
                    execution_adapter_uninitialized=False, direct_grasp_shortcut=False,
                    direct_grasp_pose_checks=True, grasp_arrival_error=0.):
    adapter, trace = make_adapter(primary_empty=primary_empty, moveit_config=MoveItAdapterConfig(
        validate_motion_paths=True, align_wrist_roll=True, pose_refinement_iterations=10,
        use_verified_local_fk=True))
    adapter.set_target_contacts = lambda arm: None
    if reject_collision:
        adapter._validity_client.callback = lambda request:NS(valid=False,constraint_result=[])
    state = deepcopy(adapter._current_state)
    clock_ns = 10_000_000_000
    controllers = []
    collision_samples_checked = 0
    approach_collision_samples_checked = 0
    approach_state_samples_checked = 0
    cartesian_collision_policies = []
    scene_state = NS(attached=[])
    selected_holder = {}
    node = CollectionPipeline.__new__(CollectionPipeline)
    node.candidate_ik_statistics = deepcopy(adapter.last_ik_statistics)
    node.__dict__.update(vars(NS(mode='hardware', motion=adapter, events=[], carrying=None,
              payload_future=None, payload_request=None, payload_evidence_time=0.,
              wrist_reference_id='', wrist_request=None, wrist_cameras=None, releasing=False,
              destination='', arm_config=None, _grasp_scene_freeze_held=False,
              _scene_cleanup_error='', config={
                  'frame':'base_link', 'operation_mode':'grasp_lift_return_hold',
                  'commissioning_verified':True, 'gripper_close_step_rad':.005,
                  'pregrasp_opening_margin_m':.005, 'lift_height_m':.05,
                  'cartesian_step_m':.005, 'cartesian_tolerance_m':.005,
                  'visual_servo_enabled':False, 'wrist_observation_enabled':True,
                  'approach_collision_check_enabled':approach_collision_check,
                  'evidence_max_age_sec':.5, 'empty_confirmations':2, 'maximum_cycles':1,
                  'label_destinations':{}, 'arms':{'left':{
                      'gripper':{'measured':True,'closed_joint_rad':0.,
                                 'open_joint_rad':1.,'maximum_width_m':.1},
                      'home_joint_rad':'startup','bins':{}}}})))
    if visual_servo:
        # Use the deployed visual approach settings with synthetic observations.
        profile = json.loads((Path(__file__).resolve().parents[4]
            / 'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json').read_text())
        node.config.update({key:value for key,value in profile.items()
                            if key.startswith('visual_')})
        node.config['visual_servo_enabled'] = True
    if stationary_target:
        node.config.update(operation_mode='pick_return_release', assume_stationary_target=True,
                           temporary_contact_verification='raw_feedback',
                           temporary_placement_verification='assume_success',
                           label_destinations={'*': 'return_to_pick'},
                           visual_servo_enabled=False, wrist_observation_enabled=False)
    node.config['pregrasp_motion_enabled'] = not direct_grasp
    node.config['direct_grasp_shortcut_enabled'] = direct_grasp_shortcut
    node.config['direct_grasp_pose_checks_enabled'] = direct_grasp_pose_checks
    if approach_obstacle:
        profile = json.loads((Path(__file__).resolve().parents[4]
            / 'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json').read_text())
        node.config.update({key: value for key, value in profile.items()
                            if key.startswith('approach_collision_')})
        # Exercise the retained optional detour mode, disabled in the profile.
        node.config['approach_collision_check_enabled'] = approach_collision_check
        node.config['approach_collision_replanning_enabled'] = approach_collision_check
    if fixed_jaw_clearance:
        node.config.update(fixed_jaw_clearance_enabled=True,
                           fixed_jaw_collision_meshes=['synthetic_finger.stl'])
        node.description = adapter._description
        monkeypatch.setattr('cleany_skill_executor.collection_pipeline.load_fixed_jaw_bounds',
                            lambda *args: Bounds((.008, -.006, -.005), (.016, .06, .005)))
    def now():
        nonlocal clock_ns
        clock_ns += 1_000_000
        value = clock_ns
        return NS(nanoseconds=value, to_msg=lambda: Time(sec=value//10**9,nanosec=value%10**9))
    def measured():
        state.header.stamp = now().to_msg()
        return deepcopy(state)
    node.measured = measured
    node.get_clock = lambda: NS(now=now)
    node.get_logger = lambda: NS(info=lambda *_:None, error=lambda *_:None, warning=lambda *_:None)
    node.get_parameter = lambda name: NS(value={
        'pipeline_timeout_sec':600.,'maximum_target_translation_m':.01,
        'wrist_checks':3,
        'hardware_path_refresh_attempts':1,'hardware_communication_resume_limit':1}.get(name, .5))
    node.wait = lambda future,*args,**kwargs:future.result()
    node.scene_monitor = NS(wait=lambda **kwargs:None)
    if approach_obstacle:
        node.scene_monitor = NS(wait=lambda **kwargs: trace.append('fresh_scene_for_approach'))
    node.scene = NS(begin=lambda *a:trace.append('target_scene_begin'),
                    restore=lambda:trace.append('target_scene_restore'),
                    allow_contacts_for=lambda arm:trace.append('target_contacts'),
                    disallow_target_contacts=lambda:trace.append('target_contacts_disallowed'),
                    active=False)
    for name in ('run', 'execute_trajectory', 'release_grasp_scene_freeze'):
        setattr(node, name, MethodType(getattr(PregraspPipeline, name), node))
    for name in ('run_collection','execute_pregrasp','before_execution','after_pregrasp','linear',
                 'jaw','move_joint_pose','pose','attach','apply','event',
                 'validate_target_translation','target_translation_limit',
                 'wrist_observation_required','close_until_contact','verify_evidence',
                 'confirm_wrist_target','confirm_held_target'):
        setattr(node,name,MethodType(getattr(CollectionPipeline,name),node))
    node.cartesian_pose = CollectionPipeline.cartesian_pose
    node.verify_simulator = lambda arm: None
    node.validation, node.reference, node.wrist = object(), object(), object()
    node.fk, node.cartesian, node.joint_plan = object(), object(), object()
    node.apply_scene, node.get_scene = object(), object()
    node.evidence = NS(wait_for_service=lambda **_:True)
    def observe(reference_id, after):
        if stationary_target:
            raise AssertionError('Stationary target profile must not request head reobservation')
        return NS(header=Header(stamp=now().to_msg(),frame_id='base_link'),
                  observed_center=Point(x=.4,y=.2,z=.3),
                  observed_extent=Vector3(x=.04,y=.04,z=.04), mask_pixels=200,
                  valid_depth_points=200,valid_depth_fraction=1.,
                  source_snapshot_id='synthetic_capture',source_object_id=1,
                  source_label='synthetic_box',source_detector_label='detector_box',
                  source_confidence=1.)
    node.observe = observe
    limits = {n:PhysicalJointLimit(-2.,2.,1.,2.,2048.,1.,0.,0.,4095.) for n in state.name}
    real = HardwareExecution.__new__(HardwareExecution)
    real.node, real.arm_holds, real.uncommanded = node, {}, {}
    real._sensor_scene_freeze_depth = 0
    real.contact_criteria = load_contact_criteria(
        Path(__file__).resolve().parents[4]/'configs/robot/gripper_contact_decision.yaml')
    real.wait_for_contact = lambda *_: ContactDecision(ContactState.CONTACT, 'synthetic_contact',
        window_sec=.4, minimum_residual_ticks=340., median_current_raw=18., minimum_load_raw=200.)
    real.validity = object()
    real.last_execution_metrics, real.fault, real.stop_report = {}, '', None
    real.verify = lambda arm: limits
    real.monitor = measured
    parameters = {
        'hardware_velocity_rad_s':.8,'hardware_acceleration_rad_s2':1.2,
        'hardware_path_step_rad':.01,'hardware_path_simplification_tolerance_rad':.002,
        'hardware_start_tolerance_rad':.005,'hardware_arrival_tolerance_rad':.07,
        'hardware_start_blend_max_rad':.03,
        'hardware_uncommanded_tolerance_rad':.03,'hardware_settle_time_sec':.000001,
        'hardware_tcp_position_tolerance_m':.008,'hardware_tcp_orientation_tolerance_deg':6.,
        'hardware_command_continuity_max_offset_rad':.06,
        'hardware_start_delay_sec':.01,
        'hardware_release_retreat_min_duration_sec':.25,
        'hardware_endpoint_settle_duration_sec':.000001,
        'hardware_endpoint_settle_timeout_sec':1.,
        'hardware_endpoint_settle_velocity_rad_s':.01,
    }
    real.value = lambda name: parameters[name]
    real.fail_stop = lambda: {'requested':True,'controllers_stopped':True,
                              'physical_stop_verified':False}
    real.begin_sensor_scene_freeze = lambda:trace.append('sensor_scene_freeze')
    real.end_sensor_scene_freeze = lambda:trace.append('sensor_scene_resume')
    @contextmanager
    def frozen():
        yield
    real.frozen_sensor_scene = frozen
    node.real = real
    obstacle_active = False
    original_state_is_valid = adapter.state_is_valid
    def state_is_valid(*args):
        nonlocal approach_state_samples_checked
        if obstacle_active:
            approach_state_samples_checked += 1
        return original_state_is_valid(*args)
    adapter.state_is_valid = state_is_valid
    if approach_obstacle:
        # A synthetic 7mm sphere blocks the middle of the straight approach,
        # but leaves the final 40mm entry clear. This is a toy FK model.
        original_linear = node.linear
        def linear_with_obstacle(*args, **kwargs):
            nonlocal obstacle_active
            obstacle_active = args[2] in ('approach', 'approach_detour')
            try:
                return original_linear(*args, **kwargs)
            finally:
                obstacle_active = False
        node.linear = linear_with_obstacle
    class VirtualActionClient:
        def __init__(self,node,type,name): self.name=name
        def destroy(self): pass
        def wait_for_server(self,**kwargs): return True
        def send_goal_async(self,goal,**kwargs):
            trace.append('virtual_async_dispatch:'+self.name)
            response = run_action(self,goal)
            feedback = kwargs.get('feedback_callback')
            if feedback is not None:
                feedback(NS(feedback=FollowJointTrajectory.Feedback(
                    joint_names=list(goal.trajectory.joint_names),
                    desired=deepcopy(goal.trajectory.points[-1]),
                    actual=deepcopy(goal.trajectory.points[-1]))))
            return completed(NS(accepted=True,
                get_result_async=lambda:completed(NS(
                    status=4 if response.error_code == 0 else 6,result=response)),
                cancel_goal_async=lambda:completed(NS())))
    monkeypatch.setattr(node, 'create_motion_client',
                        lambda action_type, name: VirtualActionClient(node, action_type, name))
    def run_action(client,goal,*args,**kwargs):
        if client is node.validation:
            selected = replay_validation(adapter, measured, now, node.scene, trace,
                                         fixed_jaw_clearance=fixed_jaw_clearance,
                                         direct_grasp=direct_grasp,
                                         direct_grasp_shortcut=direct_grasp_shortcut)
            node.candidate_ik_statistics=deepcopy(adapter.last_ik_statistics)
            selected_holder['value']=selected
            if execution_adapter_uninitialized:
                # The deployed selector runs in another process. Its measured
                # state cannot initialize the Action server's own FK adapter.
                adapter._current_state = None
                trace.append('execution_adapter_without_selector_state')
            return selected
        command=goal.trajectory
        assert command.points and all(p.velocities for p in command.points)
        expected_start = [dict(zip(state.name, state.position))[name] for name in command.joint_names]
        if '_arm_controller/' in client.name and 'left' in real.arm_holds:
            expected_start = real.arm_holds['left'].command
        assert list(command.points[0].positions) == pytest.approx(expected_start)
        controllers.append({'action':client.name,'samples':len(command.points),
                            'joints':list(command.joint_names),
                            'end_positions':list(command.points[-1].positions),
                            'goal_tolerance': [(tol.name, tol.position) for tol in goal.goal_tolerance],
                            'header_stamp_ns':command.header.stamp.sec*10**9+command.header.stamp.nanosec,
                            'planned_duration_sec':command.points[-1].time_from_start.sec
                                + command.points[-1].time_from_start.nanosec/10**9,
                            'maximum_velocity':max(abs(v) for p in command.points for v in p.velocities)})
        if fail_controller:
            return FollowJointTrajectory.Result(error_code=-1,error_string='synthetic injected rejection')
        for name,value in zip(command.joint_names,command.points[-1].positions):
            state.position[state.name.index(name)]=value
        if direct_grasp and '_arm_controller/' in client.name and not any(
                '_arm_controller/' in previous['action'] for previous in controllers[:-1]):
            state.position[state.name.index(command.joint_names[0])] -= grasp_arrival_error
        return FollowJointTrajectory.Result(error_code=0,error_string='')
    node.run_action=run_action
    def call(client,request,*args):
        nonlocal collision_samples_checked, approach_collision_samples_checked
        if client is real.validity:
            if not collision_samples_checked:
                trace.append('production_hardware_cpu_collision_validation')
            collision_samples_checked += 1
            if approach_obstacle and obstacle_active:
                approach_collision_samples_checked += 1
                q = dict(zip(request.robot_state.joint_state.name,
                             request.robot_state.joint_state.position))
                tcp = tuple(q[n] for n in ARM_JOINT_NAMES['left'][:3])
                if math.dist(tcp, (.4, .26, .3)) < .007:
                    trace.append('synthetic_obstacle_rejected_physical_path')
                    return NS(valid=False, contacts=[])
            return adapter._validity_client.callback(request)
        if client is node.fk:
            return adapter._fk_client.callback(request)
        if client is node.cartesian:
            trace.append('cpu_cartesian_request')
            cartesian_collision_policies.append(request.avoid_collisions)
            names=ARM_JOINT_NAMES['left']
            values=dict(zip(state.name,state.position))
            values.update(zip(request.start_state.joint_state.name,
                              request.start_state.joint_state.position, strict=True))
            positions = [[values[n] for n in names]]
            for waypoint in request.waypoints:
                target = waypoint.position
                previous, goal = positions[-1], [target.x,target.y,target.z,0.,0.]
                steps = max(1, math.ceil(math.dist(previous[:3], goal[:3])/request.max_step))
                positions.extend([[a+(b-a)*i/steps for a,b in zip(previous, goal)]
                                  for i in range(1, steps+1)])
            path=JointTrajectory(joint_names=list(names),points=[
                JointTrajectoryPoint(positions=position) for position in positions])
            return NS(error_code=MoveItErrorCodes(val=1),fraction=1.,solution=RobotTrajectory(joint_trajectory=path))
        if client is node.joint_plan:
            trace.append('cpu_return_rrtconnect_request')
            m=request.motion_plan_request; names=ARM_JOINT_NAMES['left']
            values=dict(zip(state.name,state.position))
            target={c.joint_name:c.position for c in m.goal_constraints[0].joint_constraints}
            path=JointTrajectory(joint_names=list(names),points=[
                JointTrajectoryPoint(positions=[values[n] for n in names]),
                JointTrajectoryPoint(positions=[target[n] for n in names])])
            return NS(motion_plan_response=NS(error_code=MoveItErrorCodes(val=1),trajectory=RobotTrajectory(joint_trajectory=path)))
        if client is node.apply_scene:
            scene_state.attached=deepcopy(request.scene.robot_state.attached_collision_objects)
            return NS(success=True)
        if client is node.get_scene:
            return NS(scene=NS(robot_state=NS(attached_collision_objects=scene_state.attached)))
        if client is node.reference:
            trace.append('reference_clear')
            return NS(success=True)
        if client is node.wrist:
            assert request.label == 'synthetic_box'
            assert request.detector_label == 'detector_box'
            return NS(success=True, reference_id=request.reference_id or 'synthetic_wrist',
                      source_snapshot_id=request.source_snapshot_id,
                      source_object_id=request.source_object_id,
                      header=Header(stamp=now().to_msg()))
        if client is node.evidence:
            response = CheckManipulation.Response(success=True, arm=request.arm,
                reference_id=request.reference_id, wrist_reference_id=request.wrist_reference_id,
                source_snapshot_id=request.source_snapshot_id, source_object_id=request.source_object_id,
                contact_confirmed=True, object_tracked=True, message='synthetic_contact_and_identity')
            response.header.stamp = now().to_msg()
            trace.append('synthetic_evidence_'+str(request.stage))
            return response
        raise AssertionError('Unexpected synthetic service')
    node.call=call
    result=node.run_collection('synthetic box')
    result.update(test_scope='in_process_synthetic_transports',real_cuda_executed=False,
                  real_dds_executed=False,physical_motors_accessed=False,
                  real_moveit_executed=False, real_controllers_executed=False,
                  perception_models_executed=False, synthetic_candidate_count=8,
                  hardware_collision_samples_checked=collision_samples_checked,
                  approach_collision_samples_checked=approach_collision_samples_checked,
                  approach_state_samples_checked=approach_state_samples_checked,
                  cartesian_collision_policies=cartesian_collision_policies,
                  robot_model='toy translation/rotation chains, not SO-101',
                  visual_servo_enabled=visual_servo,candidate_ik_statistics=node.candidate_ik_statistics,
                  virtual_controller_commands=controllers,trace=trace)
    return result


@pytest.mark.parametrize('primary_empty',[False,True])
@pytest.mark.parametrize('visual_servo',[False,True])
def test_full_hybrid_collection_replay(monkeypatch,tmp_path,primary_empty,visual_servo):
    result=replay_pipeline(monkeypatch,primary_empty=primary_empty,visual_servo=visual_servo)
    output=tmp_path/'hybrid_pipeline_replay.json'
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    assert result['success'], result['message']
    cycle=result['cycles'][0]
    assert cycle['pregrasp_executed'] and cycle['grasp_executed']
    assert cycle['returned'] and cycle['payload_held']
    assert cycle['lift_height_from_fk_m']==pytest.approx(.05)
    assert not cycle['grasp_assumed'] and cycle['grasp_verified']
    assert cycle['grasp_contact_confirmed']
    stages=[e['stage'] for e in result['events']]
    approach = 'visual_approach_step' if visual_servo else 'approach'
    assert all(s in stages for s in [approach,'close_for_contact','grasp_confirmed','attach','retreat','lift','return_holding_payload'])
    approach_moves = cycle['visual_approach']['steps'] if visual_servo else 1
    assert len(result['virtual_controller_commands'])==6+approach_moves
    arm, jaw = result['virtual_controller_commands'][:2]
    assert arm['header_stamp_ns'] == jaw['header_stamp_ns'] > 0
    assert arm['planned_duration_sec'] == jaw['planned_duration_sec']
    assert all(c['maximum_velocity'] <= .8+1e-8 for c in result['virtual_controller_commands'])
    assert result['hardware_collision_samples_checked'] > 0
    assert result['candidate_ik_statistics']['fallback_batches']==2*int(primary_empty)
    assert result['candidate_ik_statistics']['grasp_retry_calls']==int(primary_empty)


def test_corrected_fixed_jaw_goal_reaches_the_physical_straight_approach_without_detours(monkeypatch):
    result = replay_pipeline(monkeypatch, stationary_target=True, fixed_jaw_clearance=True)
    assert result['success'], result['message']
    cycle = result['cycles'][0]
    clearance = cycle['fixed_jaw_approach_clearance']
    assert clearance['outcome'] == 'executed'
    assert clearance['checked_samples'] > 0
    assert clearance['minimum_gap_m'] >= .005 - 1e-8
    assert 'approach_collision_replanning' not in cycle
    audit = [json.loads(line.removeprefix('fixed_jaw_clearance=')) for line in result['trace']
             if isinstance(line, str) and line.startswith('fixed_jaw_clearance=')]
    assert audit[0]['additional_correction_m'] == pytest.approx(.003)
    assert audit[0]['lateral_offset_m'] == pytest.approx(.025)
    arm_commands = [c for c in result['virtual_controller_commands']
                    if '_arm_controller/' in c['action']]
    # First arm move reaches the corrected pregrasp, the next its straight grasp.
    assert arm_commands[0]['end_positions'][0] == pytest.approx(.417)
    assert arm_commands[1]['end_positions'][0] == pytest.approx(.417)


def test_stationary_target_replay_completes_pick_return_release_without_reobservation(monkeypatch):
    result = replay_pipeline(monkeypatch, stationary_target=True)
    assert result['success'], result['message']
    cycle = result['cycles'][0]
    assert cycle['pregrasp_executed'] and cycle['grasp_executed']
    assert cycle['grasp_contact_confirmed'] and cycle['released'] and cycle['returned']
    assert cycle['target_motion_assumed_stationary'] is True
    assert cycle['head_reobservation_performed'] is False
    assert cycle['placement_state'] == 'ASSUMED'
    assert 'target_observation_before' not in cycle
    assert 'target_observation_after' not in cycle
    assert 'observed_surface_displacement_m' not in cycle
    assert result['hardware_collision_samples_checked'] > 0


def test_direct_grasp_replay_opens_moves_closes_and_lifts_without_pregrasp(monkeypatch):
    result = replay_pipeline(monkeypatch, stationary_target=True, direct_grasp=True)
    assert result['success'], result['message']
    cycle = result['cycles'][0]
    assert cycle['pregrasp_executed'] is False and cycle['direct_grasp_executed'] is True
    assert 'pregrasp_execution_metrics' not in cycle
    assert cycle['grasp_contact_confirmed'] and cycle['released'] and cycle['returned']
    stages = [event['stage'] for event in result['events']]
    assert stages.index('open_before_direct_grasp') < stages.index('direct_grasp')
    assert stages.index('direct_grasp') < stages.index('close_for_contact') < stages.index('lift')
    assert 'retreat' not in stages and 'approach' not in stages
    assert 'lower_to_pickup' in stages and 'lower_to_pregrasp' not in stages
    assert result['hardware_collision_samples_checked'] > 0
    assert not any('stage=PREGRASP_IK' in detail or 'stage=PLAN_PREGRASP' in detail
                   for detail in result['trace'])


def test_direct_grasp_initializes_execution_adapter_separately_from_selector(monkeypatch):
    result = replay_pipeline(monkeypatch, stationary_target=True, direct_grasp=True,
                             execution_adapter_uninitialized=True)
    assert result['success'], result['message']
    cycle = result['cycles'][0]
    assert cycle['pregrasp_executed'] is False and cycle['direct_grasp_executed'] is True
    assert cycle['grasp_contact_confirmed'] and cycle['released'] and cycle['returned']
    assert 'execution_adapter_without_selector_state' in result['trace']
    assert 'stop' not in result


def test_checked_direct_grasp_replay_avoids_rrt_and_pregrasp(monkeypatch):
    result = replay_pipeline(monkeypatch, stationary_target=True, direct_grasp=True,
                             execution_adapter_uninitialized=True, direct_grasp_shortcut=True)
    assert result['success'], result['message']
    cycle = result['cycles'][0]
    assert cycle['direct_grasp_route'] == 'joint_direct'
    assert cycle['grasp_contact_confirmed'] and cycle['released'] and cycle['returned']
    commands = [command for command in result['virtual_controller_commands']
                if '_arm_controller/' in command['action']]
    assert commands[0]['planned_duration_sec'] < 2.
    assert result['hardware_collision_samples_checked'] > 0


@pytest.mark.parametrize('shortcut', [False, True])
@pytest.mark.parametrize('pose_checks', [False, True])
@pytest.mark.parametrize('error', [.025, .12])
def test_grasp_trial_reaches_gripper_close_with_encoder_and_tcp_errors(
        monkeypatch, shortcut, pose_checks, error):
    result = replay_pipeline(monkeypatch, stationary_target=True, direct_grasp=True,
                             direct_grasp_shortcut=shortcut, direct_grasp_pose_checks=pose_checks,
                             grasp_arrival_error=error)
    stages = [event['stage'] for event in result['events']]
    commands = result['virtual_controller_commands']
    arm_goal = next(command for command in commands if '_arm_controller/' in command['action'])
    assert arm_goal['goal_tolerance'] == (
        [] if pose_checks else [(name, -1.) for name in ARM_JOINT_NAMES['left']])
    if pose_checks:
        assert not result['success'] and 'close_for_contact' not in stages
    else:
        assert 'close_for_contact' in stages
        metrics = result['cycles'][0]['direct_grasp_execution_metrics']
        assert metrics['endpoint_accuracy_checks_enabled'] is False
        assert metrics['endpoint_within_tolerance'] is False
        assert metrics['joint_arrival_within_tolerance'] is (error <= .07)
        assert metrics['final_tcp_position_error_m'] == pytest.approx(error)
        assert result['cycles'][0]['grasp_contact_confirmed']
        # The trial override applies only to the grasp arrival. Subsequent
        # lift and return endpoints must still meet the physical TCP tolerance.
        assert not result['success']
        if error < .07:
            assert 'Physical TCP endpoint outside tolerance' in result['message']
            later = [command for command in commands if '_arm_controller/' in command['action']][1:]
            assert later and all(command['goal_tolerance'] == [] for command in later)


def test_grasp_trial_still_stops_on_controller_rejection(monkeypatch):
    result = replay_pipeline(monkeypatch, stationary_target=True, direct_grasp=True,
                             direct_grasp_pose_checks=False, fail_controller=True)
    assert not result['success'] and 'synthetic injected rejection' in result['message']
    assert not any(event['stage'] == 'close_for_contact' for event in result['events'])


def test_virtual_controller_rejection_stops_collection(monkeypatch,tmp_path):
    result=replay_pipeline(monkeypatch,fail_controller=True)
    (tmp_path/'hybrid_pipeline_replay.json').write_text(json.dumps(result,indent=2)+'\n')
    assert not result['success']
    assert 'synthetic injected rejection' in result['message']
    assert result['stop']['requested']
    assert not any(e['stage']=='lift' for e in result['events'])


def test_unchecked_approach_executes_straight_and_restores_checks_for_retreat(monkeypatch):
    result = replay_pipeline(monkeypatch, stationary_target=True, approach_obstacle=True,
                             approach_collision_check=False)
    assert result['success'], result['message']
    cycle = result['cycles'][0]
    assert cycle['approach_collision_checks_enabled'] is False
    assert 'approach_collision_replanning' not in cycle
    assert 'fixed_jaw_approach_clearance' not in cycle
    assert result['approach_collision_samples_checked'] == 0
    assert result['approach_state_samples_checked'] == 0
    assert result['hardware_collision_samples_checked'] > 0
    assert result['cartesian_collision_policies'][0] is False
    assert all(result['cartesian_collision_policies'][1:])
    assert 'synthetic_obstacle_rejected_physical_path' not in result['trace']
    assert any(event['stage'] == 'approach' for event in result['events'])
    assert any(event['stage'] == 'grasp_confirmed' for event in result['events'])


def test_obstructed_approach_replans_then_runs_the_validated_detour(monkeypatch):
    result = replay_pipeline(monkeypatch, stationary_target=True, approach_obstacle=True)
    assert result['success'], result
    recovery = result['cycles'][0]['approach_collision_replanning']
    assert recovery['scene_refreshes'] == 1 and recovery['detour_attempts'] == 1
    assert recovery['selected_route'] == 'closing_plus' and recovery['outcome'] == 'executed'
    assert len(recovery['failures']) == 2
    assert all(failure['phase'] == 'command_path' for failure in recovery['failures'])
    assert result['trace'].count('synthetic_obstacle_rejected_physical_path') == 2
    assert any(event['stage'] == 'approach_detour_executed' for event in result['events'])
    assert result['hardware_collision_samples_checked'] > 0
    assert result['cycles'][0]['head_reobservation_performed'] is False
    assert result['physical_motors_accessed'] is False


def test_cpu_collision_rejection_prevents_any_controller_command(monkeypatch,tmp_path):
    result=replay_pipeline(monkeypatch,reject_collision=True)
    (tmp_path/'hybrid_pipeline_replay.json').write_text(json.dumps(result,indent=2)+'\n')
    assert not result['success']
    assert result['cycles'][0]['error_code'] == ValidatePick.Result.ERROR_MOVEIT
    assert result['virtual_controller_commands'] == []
    assert result['candidate_ik_statistics']['fallback_batches'] == 1
    assert all(call['accepted_targets'] == 0
               for call in result['candidate_ik_statistics']['batch_calls'])
