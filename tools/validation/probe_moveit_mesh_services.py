"""Replay contained obstacles through isolated move_group ROS services."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/cleany_moveit_config/test'))
from runtime_support import call, moveit_runtime
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
from shape_msgs.msg import SolidPrimitive


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True, type=Path)
    parser.add_argument('--fixtures', default='compatible-fixtures.json')
    parser.add_argument('--all-missed', action='store_true')
    parser.add_argument('--output', default='services.json')
    args = parser.parse_args()
    fixture = json.loads((args.directory/args.fixtures).read_text())
    diagnosis = json.loads((args.directory/'contact-diagnosis.json').read_text())
    selected = [case['index'] for case in (diagnosis['cases'] if args.all_missed else diagnosis['cases'][:6])]
    output = {'driver_started': False, 'trajectory_execution': False, 'models': {}}
    def state(q):
        sample = RobotState()
        sample.joint_state.name = fixture['joint_names']
        sample.joint_state.position = q
        return sample
    for model in fixture['models']:
        label = model['name']
        directory = args.directory / (Path(args.output).stem+'-'+label)
        directory.mkdir(exist_ok=True)
        rows = {'probes': [], 'plans': []}
        with moveit_runtime(directory, head=True, robot_model_file=model['urdf'],
                            collision_detector='cleany/SolidFCL' if model.get('solid') else 'FCL') as (node, process):
            rows['detector'] = 'cleany/SolidFCL' if model.get('solid') else 'FCL'
            validity = node.create_client(GetStateValidity, '/check_state_validity')
            apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
            planner = node.create_client(GetMotionPlan, '/plan_kinematic_path')
            for index in selected:
                probe = fixture['probes'][index]
                obj = CollisionObject(id='contained_probe', operation=CollisionObject.ADD)
                obj.header.frame_id = 'base_link'
                obj.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[probe['world'][0]['radius']])]
                pose = Pose(); pose.orientation.w = 1.
                pose.position.x,pose.position.y,pose.position.z = probe['world'][0]['center']
                obj.primitive_poses = [pose]
                update = ApplyPlanningScene.Request(); update.scene.is_diff = True
                update.scene.world.collision_objects = [obj]
                assert call(node,process,apply,update).success
                result = call(node,process,validity,GetStateValidity.Request(robot_state=state(probe['q'])))
                rows['probes'].append({'index':index,'valid':result.valid,
                    'pairs':[[c.contact_body_1,c.contact_body_2] for c in result.contacts]})
            update = ApplyPlanningScene.Request(); update.scene.is_diff = True
            update.scene.world.collision_objects = [CollisionObject(id='contained_probe',operation=CollisionObject.REMOVE)]
            assert call(node,process,apply,update).success
            if label != 'cad':
                for repeat in range(5):
                    for arm in ('left','right'):
                        path = next(p for p in fixture['paths'] if p['group']==arm+'_grasp_arm')
                        request = GetMotionPlan.Request()
                        request.motion_plan_request.group_name = path['group']
                        request.motion_plan_request.start_state = state(path['start'])
                        request.motion_plan_request.allowed_planning_time = 2.
                        request.motion_plan_request.num_planning_attempts = 1
                        request.motion_plan_request.goal_constraints = [Constraints(joint_constraints=[
                            JointConstraint(joint_name=name,position=value,tolerance_above=.001,tolerance_below=.001,weight=1.)
                            for name,value in zip(fixture['joint_names'],path['goal']) if name.startswith(arm+'_') and 'gripper' not in name])]
                        start = time.perf_counter()
                        response = call(node,process,planner,request).motion_plan_response
                        elapsed = time.perf_counter()-start
                        rows['plans'].append({'repeat':repeat,'warmup':repeat==0,'arm':arm,
                            'rpc_seconds':elapsed,'planning_seconds':response.planning_time,
                            'error':response.error_code.val,'waypoints':len(response.trajectory.joint_trajectory.points)})
                        assert response.error_code.val == response.error_code.SUCCESS
        assert ('Using collision detector:' in (directory/'moveit.log').read_text())
        if model.get('solid'):
            assert all(not r['valid'] for r in rows['probes'])
        output['models'][label] = rows
        (args.directory/args.output).write_text(json.dumps(output,indent=2)+'\n')
        print(label,rows,flush=True)


if __name__ == '__main__':
    main()
