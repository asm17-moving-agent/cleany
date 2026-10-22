"""Isolated ROS transport test with synthetic perception and Action results."""

import json
import os
import threading
import time
import subprocess
from pathlib import Path
from uuid import uuid4

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('CLEANY_RUN_ROS_TESTS') != '1', reason='ROS opt-in')


def test_approval_accepts_neighbor_loss_but_rejects_target_movement(tmp_path, monkeypatch):
    assert os.environ.get('ROS_DOMAIN_ID', '0') != '0', 'Use an isolated ROS domain'
    import rclpy
    from rclpy.action import ActionServer
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from cleany_interfaces.action import InspectScene, ExecuteManipulationSkill
    from cleany_interfaces.msg import DetectedObject2D, DetectedObject2DArray, ManipulationExecutionRecord
    from cleany_interfaces.srv import GetSceneSnapshot, GetManipulationExecution
    from sensor_msgs.msg import Image
    from std_srvs.srv import Trigger
    from cleany_dev_monitor.proposal_ros import ProposalROS
    from cleany_dev_monitor.proposals import DEFAULTS
    from cleany_dev_monitor.operator_jobs import atomic_json, execute_job

    config = tmp_path / 'config.json'
    config.write_text(json.dumps(dict(operation_mode='pick_return_release', label_destinations={'*': 'return_to_pick'})))
    monkeypatch.setenv('CLEANY_MANIPULATION_COLLECTION_CONFIG', str(config))
    rclpy.init()
    server = rclpy.create_node('proposal_fixture')
    publisher = server.create_publisher(Image, DEFAULTS['proposal_debug_image_topic'],
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    snapshots, calls = {}, []
    position = [.4]
    include_neighbor = [True]

    def inspect(handle):
        calls.append(handle.request)
        message = DetectedObject2DArray()
        message.header.stamp = server.get_clock().now().to_msg()
        message.header.frame_id = 'camera_color_optical_frame'
        message.snapshot_id = f'fixture-{len(calls)}'
        message.tracking_session_id = handle.request.tracking_session_id
        message.tracking_epoch = 'fixture-epoch'
        message.representative_frame = 'base_link'
        item = DetectedObject2D(object_id=1, label='cup', confidence=.95,
            sorting_category='trash', sorting_reason='empty cup', distance_valid=True, distance_m=.4,
            track_id='fixture-track', tracking_state='TRACKED', position_valid=True,
            x_min=10., y_min=10., x_max=70., y_max=90., median_depth_m=.4,
            estimated_width_m=.05, estimated_height_m=.07)
        item.representative_position.x, item.representative_position.z = position[0], .2
        message.detections = [item]
        if include_neighbor[0]:
            neighbor = DetectedObject2D(object_id=2, label='book', confidence=.27,
                sorting_category='lost_item', sorting_reason='book',
                distance_valid=True, distance_m=.8, track_id='neighbor',
                tracking_state='TRACKED', position_valid=True,
                x_min=75., y_min=10., x_max=95., y_max=40., median_depth_m=.8,
                estimated_width_m=.05, estimated_height_m=.07)
            neighbor.representative_position.x, neighbor.representative_position.z = .7, .2
            message.detections.append(neighbor)
        snapshots[message.snapshot_id] = message
        image = Image(header=message.header, height=100, width=100, encoding='rgb8', step=300,
                      data=bytes([120] * 30000))
        publisher.publish(image)
        handle.succeed()
        return InspectScene.Result(success=True, detections=message)

    def lookup(request, response):
        response.found = request.snapshot_id in snapshots
        if response.found: response.detections = snapshots[request.snapshot_id]
        return response

    action = ActionServer(server, InspectScene, '/perception/inspect_scene', execute_callback=inspect)
    server.create_service(GetSceneSnapshot, '/perception/get_scene_snapshot', lookup)
    goals, records = [], {}

    def execute(handle):
        goal = handle.request
        goals.append(goal)
        fields = dict(execution_id=goal.execution_id, execution_profile='jetson', status='SUCCESS',
            error_code='NONE', last_completed_stage='VERIFYING_PLACEMENT', selected_arm='left',
            object_state='LEFT_GRIPPER', placement_state='ASSUMED', stop_confirmed=True, arm_recovered=True)
        records[goal.execution_id] = ManipulationExecutionRecord(**fields, has_result=True,
            record_state='FINISHED', snapshot_id=goal.snapshot_id, object_id=goal.object_id,
            mission_id=goal.mission_id, task_id=goal.task_id, skill_name=goal.skill_name,
            destination_id=goal.destination_id)
        handle.succeed()
        return ExecuteManipulationSkill.Result(**fields)

    def control(_request, response):
        response.success = True
        response.message = json.dumps(dict(schema=1, active_execution_id='', backend_ready=True,
            guard_fresh=True, guard={'state': 'ready'}, inhibited=False))
        return response

    def execution_record(request, response):
        response.found = request.execution_id in records
        if response.found: response.record = records[request.execution_id]
        return response

    manipulation = ActionServer(server, ExecuteManipulationSkill, '/manipulation/execute_skill', execute_callback=execute)
    server.create_service(Trigger, '/manipulation/control_state', control)
    server.create_service(GetManipulationExecution, '/manipulation/get_execution', execution_record)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    adapter = None
    try:
        adapter = ProposalROS(tmp_path, DEFAULTS)
        value = adapter.propose(lambda: False)
        assert value['state'] == 'READY' and value['image_available']
        assert value['scene_object_count'] == 2
        assert (tmp_path / 'proposals' / f"{value['id']}.jpg").read_bytes().startswith(b'\xff\xd8')
        include_neighbor[0] = False
        adapter.revalidate(value, lambda: False)
        position[0] += .05
        with pytest.raises(ValueError, match='변경'):
            adapter.revalidate(value, lambda: False)
        assert len(calls) == 3 and all(request.selected_object_id == 0 for request in calls)
        assert len({request.tracking_session_id for request in calls}) == 1
        position[0] = .4
        (tmp_path / 'enabled').touch()
        (tmp_path / 'jobs').mkdir()
        request_id = str(uuid4())
        job = tmp_path / 'jobs' / f'{request_id}.json'
        atomic_json(job, dict(id=request_id, action='approve', args={'proposal_id': value['id']},
            proposal=value, proposal_policy=DEFAULTS, state='PENDING', created_at=time.time()))
        monkeypatch.setattr('cleany_dev_monitor.operator_jobs.runtime_info', lambda: dict(managed=True, pipeline_busy=False))
        popen = subprocess.Popen
        with job.with_suffix('.log').open('w') as log:
            monkeypatch.setattr('cleany_dev_monitor.operator_jobs.subprocess.Popen',
                lambda command, **kwargs: popen(command, stdout=log, stderr=log, **kwargs))
            execute_job(job, Path(__file__).resolve().parents[4])
        result = json.loads(job.read_text())
        assert result['state'] == 'SUCCEEDED', (result, job.with_suffix('.log').read_text())
        assert len(goals) == 1
        assert goals[0].snapshot_id == value['snapshot_id'] and goals[0].object_id == value['target']['object_id']
        assert goals[0].execution_id == request_id
        assert adapter.store.current()['state'] == 'READY'
        assert adapter.store.current()['id'] == result['next_proposal_id'] != value['id']
        skipped = adapter.propose(lambda: False, previous=adapter.store.current(), skip=True)
        assert skipped['state'] == 'NO_TARGET' and skipped['excluded_tracks'] == ['fixture-track']
        assert len(goals) == 1  # Neither reobservation nor skipping issues a second Goal.
    finally:
        if adapter: adapter.close()
        executor.shutdown(timeout_sec=3.)
        thread.join(timeout=3.)
        action.destroy()
        manipulation.destroy()
        server.destroy_node()
        rclpy.try_shutdown()
