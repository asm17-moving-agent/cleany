"""Probe the actual perception Action and retain its sensor-bound snapshot."""
import json
import argparse
from pathlib import Path
import time
from uuid import uuid4

import rclpy
from rclpy.action import ActionClient
from rosidl_runtime_py.convert import message_to_ordereddict
from cleany_interfaces.action import InspectScene, SelectReachableGrasp
from cleany_interfaces.srv import PlanGrasp

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--reconstruct', action='store_true')
parser.add_argument('--plan', action='store_true', help='Also check grasp reachability without executing motion')
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('cleanup_perception_probe')
client = ActionClient(node,InspectScene,'/perception/inspect_scene')
output = Path('/output/evaluations/planning.json' if args.plan else '/output/evaluations/inspection.json')
report = {'passed':False,'scope':'actual RGB-D perception; no manipulation requested'}


def wait(predicate, timeout):
    end = time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05)
        if predicate():
            return
    raise TimeoutError('Perception checkpoint timeout')


try:
    wait(client.server_is_ready,120.)
    future = client.send_goal_async(InspectScene.Goal(tracking_session_id='probe-'+uuid4().hex))
    wait(future.done,10.)
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError('Inspection rejected')
    future = handle.get_result_async()
    wait(future.done,180.)
    response = future.result()
    report.update(status=response.status, result=message_to_ordereddict(response.result))
    if response.status != 4 or not response.result.success:
        raise RuntimeError(response.result.message)
    if args.reconstruct or args.plan:
        detections = response.result.detections
        if len(detections.detections) != 1:
            raise RuntimeError('Component fixture expects exactly one visible object')
        future = client.send_goal_async(InspectScene.Goal(snapshot_id=detections.snapshot_id,
            selected_object_id=detections.detections[0].object_id))
        wait(future.done,10.)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('Selected-object reconstruction rejected')
        future = handle.get_result_async()
        wait(future.done,60.)
        selected = future.result()
        report['reconstruction'] = dict(status=selected.status,success=selected.result.success,
            message=selected.result.message,objects=message_to_ordereddict(selected.result.objects),
            target_cloud_points=selected.result.target_cloud.width*selected.result.target_cloud.height,
            context_cloud_points=selected.result.context_cloud.width*selected.result.context_cloud.height)
        if selected.status != 4 or not selected.result.success:
            raise RuntimeError(selected.result.message)
        if args.plan:
            planner = node.create_client(PlanGrasp,'/grasp/plan')
            wait(planner.service_is_ready,30.)
            observed = selected.result
            request = PlanGrasp.Request(snapshot_id=detections.snapshot_id,
                object_id=detections.detections[0].object_id,
                target_cloud=observed.target_cloud, context_cloud=observed.context_cloud,
                target_object=observed.objects.objects[0])
            future = planner.call_async(request)
            wait(future.done,60.)
            grasps = future.result()
            report['grasp_candidates'] = dict(success=grasps.success,message=grasps.message,count=len(grasps.candidates))
            if not grasps.success:
                raise RuntimeError(grasps.message)
            selector = ActionClient(node,SelectReachableGrasp,'/grasp/select_reachable')
            wait(selector.server_is_ready,30.)
            report['planning_feedback'] = []
            def feedback(message):
                value = message_to_ordereddict(message.feedback)
                report['planning_feedback'].append(value)
                print(json.dumps(value),flush=True)
            future = selector.send_goal_async(SelectReachableGrasp.Goal(candidates=grasps.candidates),
                                               feedback_callback=feedback)
            wait(future.done,10.)
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('Grasp selection rejected')
            future = handle.get_result_async()
            wait(future.done,180.)
            selection = future.result()
            report['selection'] = message_to_ordereddict(selection.result)
            if selection.status != 4 or not selection.result.success:
                raise RuntimeError(selection.result.message)
    report['passed'] = True
    print(json.dumps(report,ensure_ascii=False),flush=True)
except Exception as error:
    report['error'] = repr(error)
    raise
finally:
    output.write_text(json.dumps(report,indent=2,ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
