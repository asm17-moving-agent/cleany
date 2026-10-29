"""Audit all installed planning groups with two full-body states, without actuation."""
from pathlib import Path
import sys,json,math,time,traceback,argparse,xml.etree.ElementTree as ET
from copy import deepcopy
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/cleany_moveit_config/test'))
from runtime_support import call,moveit_runtime
from test_head_collision_runtime import state
from verify_moveit_retiming import interpolation_limits
from ament_index_python.packages import get_package_share_directory
from cleany_interfaces.srv import CheckStateValidityBatch
from moveit_msgs.srv import GetMotionPlan,GetStateValidity,ApplyPlanningScene
from moveit_msgs.msg import Constraints,JointConstraint,CollisionObject
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from rosidl_runtime_py.convert import message_to_ordereddict

def samples(current,jt) -> list:
    result=[];joint_index={n:i for i,n in enumerate(current.joint_state.name)}
    def add(q):
        sample=deepcopy(current)
        for name,value in zip(jt.joint_names,q):sample.joint_state.position[joint_index[name]]=float(value)
        result.append(sample)
    add(jt.points[0].positions)
    for a,b in zip(jt.points,jt.points[1:]):
        stamp=lambda p:p.time_from_start.sec+p.time_from_start.nanosec/1e9
        dt=stamp(b)-stamp(a);assert dt>0
        c0=np.array(a.positions);c1=np.array(a.velocities)*dt;c2=np.array(a.accelerations)*dt*dt/2
        dq=np.array(b.positions)-c0-c1-c2;dv=np.array(b.velocities)*dt-c1-2*c2;da=np.array(b.accelerations)*dt*dt-2*c2
        c=np.array([c0,c1,c2,10*dq-4*dv+da/2,-15*dq+7*dv-da,6*dq-3*dv+da/2])
        bound=max(np.abs(sum((k+1)*c[k+1]*math.comb(i,k)/math.comb(4,k) for k in range(i+1))).max() for i in range(5))
        count=max(2,math.ceil(bound/.002))
        for u in np.linspace(0,1,count+1)[1:]:add(np.array([u**k for k in range(6)])@c)
    return result

def main() -> int:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    root=parser.parse_args().output;root.mkdir(parents=True,exist_ok=True)
    configuration=Path(get_package_share_directory('cleany_moveit_config'))/'config'
    limits=yaml.safe_load((configuration/'joint_limits.yaml').read_text())['joint_limits'];defaults=yaml.safe_load((configuration/'ompl_planning.yaml').read_text())
    records=[]
    with moveit_runtime(root,head=True) as (node,process):
        parameters=node.create_client(GetParameters,'/move_group/get_parameters')
        values=call(node,process,parameters,GetParameters.Request(names=['robot_description','robot_description_semantic'])).values
        urdf,srdf=[ET.fromstring(v.string_value) for v in values]
        position_limits={j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper'))) for j in urdf.findall('joint') if j.get('type') in ('revolute','prismatic') and j.find('limit') is not None}
        joints={j.find('child').get('link'):j for j in urdf.findall('joint')};group_xml={g.get('name'):g for g in srdf.findall('group')}
        def variables(name):
            result=[]
            for element in group_xml[name]:
                if element.tag=='chain':
                    link=element.get('tip_link');chain=[]
                    while link!=element.get('base_link'):
                        joint=joints[link]
                        if joint.get('type')!='fixed':chain.append(joint.get('name'))
                        link=joint.find('parent').get('link')
                    result+=chain[::-1]
                elif element.tag=='group':result+=variables(element.get('name'))
                elif element.tag=='joint':result.append(element.get('name'))
            return result
        groups={name:variables(name) for name in group_xml}
        (root/'groups.json').write_text(json.dumps(groups,indent=2)+'\n')
        native=node.create_client(GetStateValidity,'/check_state_validity');batch=node.create_client(CheckStateValidityBatch,'/check_state_validity_batch');planner=node.create_client(GetMotionPlan,'/plan_kinematic_path')
        saved=[]
        for variant in range(2):
            current=state(pan=.25*variant,tilt=-.15*variant)
            for index,name in enumerate(current.joint_state.name):
                if variant and name.endswith(('shoulder_pitch_joint','elbow_pitch_joint')):current.joint_state.position[index]-=.10
            initial=call(node,process,native,GetStateValidity.Request(robot_state=current));assert initial.valid,[(c.contact_body_1,c.contact_body_2) for c in initial.contacts]
            for group,names in groups.items():
                target=dict(zip(current.joint_state.name,current.joint_state.position));side=group.split('_')[0]
                for suffix,delta in ([('shoulder_yaw',.05),('elbow_pitch',-.08)] if not variant else [('shoulder_pitch',-.08),('wrist_pitch',.10),('wrist_roll',-.10)]):target[f'{side}_{suffix}_joint']+=delta
                if f'{side}_gripper_joint' in names:target[f'{side}_gripper_joint']+=.05
                goal=deepcopy(current);goal.joint_state.position=[target[name] for name in goal.joint_state.name]
                goal_valid=call(node,process,native,GetStateValidity.Request(robot_state=goal));assert goal_valid.valid,(group,variant,[(c.contact_body_1,c.contact_body_2) for c in goal_valid.contacts])
                request=GetMotionPlan.Request();m=request.motion_plan_request;m.pipeline_id='ompl';m.planner_id='RRTConnectReturnkConfigDefault' if group.endswith('grasp_arm') else 'RRTConnectkConfigDefault';m.group_name=group;m.start_state=current;m.num_planning_attempts=1;m.allowed_planning_time=4.;m.max_velocity_scaling_factor=.2 if variant else 1.;m.max_acceleration_scaling_factor=.3 if variant else 1.
                m.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=target[n],tolerance_above=.0001,tolerance_below=.0001,weight=1.) for n in names])]
                began=time.monotonic();response=call(node,process,planner,request);wall=time.monotonic()-began
                row={'group':group,'variant':variant,'request':message_to_ordereddict(request),'response':message_to_ordereddict(response),'wall_sec':wall,'passed':False}
                try:
                    result=response.motion_plan_response;assert result.error_code.val==1,result.error_code.val
                    jt=result.trajectory.joint_trajectory;assert set(jt.joint_names)==set(names)
                    assert all(abs(q-target[n])<=.00011 for n,q in zip(jt.joint_names,jt.points[-1].positions))
                    scaled_limits=deepcopy(limits);scaled_defaults=deepcopy(defaults)
                    for kind,scale in [('velocity',m.max_velocity_scaling_factor),('acceleration',m.max_acceleration_scaling_factor)]:
                        scaled_defaults['retiming_default_'+kind+'_limit']*=scale
                        for settings in scaled_limits.values():
                            if 'max_'+kind in settings:settings['max_'+kind]*=scale
                    row['limit_ratios']=interpolation_limits(row['response']['motion_plan_response']['trajectory']['joint_trajectory'],scaled_limits,scaled_defaults,position_limits)
                    states=samples(current,jt);row['checked_samples']=len(states)
                    for start in range(0,len(states),32):
                        check=call(node,process,batch,CheckStateValidityBatch.Request(robot_states=states[start:start+32]));assert check.success and check.first_invalid_index==-1,(start,check.first_invalid_index,[(c.contact_body_1,c.contact_body_2) for c in check.contacts])
                    row['passed']=True
                except AssertionError:row['error']=traceback.format_exc()
                records.append(row);(root/'results.json').write_text(json.dumps(records,indent=2)+'\n');print(group,variant,'PASS' if row['passed'] else row['error'],flush=True)
                if not variant:saved.append(request)
        obstacle=CollisionObject(id='blocked_audit',operation=CollisionObject.ADD);obstacle.header.frame_id='base_link';obstacle.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[4.,4.,4.])];obstacle.primitive_poses=[Pose()];obstacle.primitive_poses[0].orientation.w=1.
        apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene');update=ApplyPlanningScene.Request();update.scene.is_diff=True;update.scene.world.collision_objects=[obstacle];assert call(node,process,apply,update).success
        blocked=[]
        for request in saved:
            result=call(node,process,planner,request).motion_plan_response;assert result.error_code.val!=1 and not result.trajectory.joint_trajectory.points
            blocked.append({'group':request.motion_plan_request.group_name,'code':result.error_code.val})
        (root/'blocked.json').write_text(json.dumps(blocked,indent=2)+'\n')
    print(json.dumps({'tests':len(records),'passed':sum(r['passed'] for r in records),'blocked':len(blocked)}))
    return 0 if len(records)==2*len(groups) and all(row["passed"] for row in records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
