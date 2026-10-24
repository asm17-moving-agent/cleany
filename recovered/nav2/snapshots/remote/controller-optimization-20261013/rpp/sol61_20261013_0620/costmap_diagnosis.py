#!/usr/bin/env python3
"""Offline occupancy/perimeter evidence; native predicate remains authoritative."""
import argparse,ast,csv,json,math,sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def matrix(tr):
    a=yaw(tr.rotation);c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,tr.translation.x],[s,c,tr.translation.y],[0,0,1.]])
def line(a,b):
    x,y=a;xe,ye=b;dx=abs(xe-x);sx=1 if x<xe else -1;dy=-abs(ye-y);sy=1 if y<ye else -1;err=dx+dy
    while True:
        yield x,y
        if x==xe and y==ye:break
        e=2*err
        if e>=dy:err+=dy;x+=sx
        if e<=dx:err+=dx;y+=sy

def diagnose(run,out):
    f=[int(r['sim_time_ns'])*1e-9 for r in csv.DictReader((run/'compute_timing.csv').open()) if r['success']=='0']
    if not f:return
    result=json.loads((run/'result.json').read_text());targets=sorted(set([f[0],f[-1],result['navigation_sim_end_s']-1]))
    db=sqlite3.connect(next((run/'bag').glob('*.db3')));types={i:(n,get_message(t)) for i,n,t in db.execute('SELECT id,name,type FROM topics')}
    clock=0;grid=None;tf={};records=[];wanted={'/clock','/tf','/tf_static','/local_costmap/costmap'}
    params=json.loads((run/'runtime_verified.json').read_text())['parameters']['local_costmap']
    fp=ast.literal_eval(params['footprint']);pad=params['footprint_padding'];fp=np.array([[x+math.copysign(pad,x),y+math.copysign(pad,y),1] for x,y in fp])
    def snapshot(target):
        if grid is None:return {'target_sim_s':target,'available':False}
        frame=grid.header.frame_id.strip('/');base='base_link'
        try:
            if frame=='odom':pose=tf[('odom',base)]
            elif frame=='map':pose=tf[('map','odom')]@tf[('odom',base)]
            else:raise KeyError(frame)
        except KeyError:return {'target_sim_s':target,'available':False,'frame':frame,'tf_edges':list(map(str,tf))}
        info=grid.info;origin=np.array([info.origin.position.x,info.origin.position.y]);vertices=(fp@pose.T)[:,:2];cells=np.floor((vertices-origin)/info.resolution).astype(int)
        data=np.array(grid.data).reshape((info.height,info.width));vals=[];lethal=[]
        for a,b in zip(cells,np.roll(cells,-1,axis=0)):
            for x,y in line(tuple(a),tuple(b)):
                if 0<=x<info.width and 0<=y<info.height:
                    val=int(data[y,x]);vals.append(val)
                    if val==100:lethal.append([int(x),int(y),*((np.array([x+.5,y+.5])*info.resolution)+origin).tolist()])
        goal_yaw=result['goal_map'][2]
        if frame=='odom' and ('map','odom') in tf:goal_yaw-=math.atan2(tf[('map','odom')][1,0],tf[('map','odom')][0,0])
        heading_costs=[]
        current_yaw=math.atan2(pose[1,0],pose[0,0]);delta=math.atan2(math.sin(goal_yaw-current_yaw),math.cos(goal_yaw-current_yaw))
        for u in np.linspace(0,1,11):
            angle=current_yaw+u*delta;c,s=math.cos(angle),math.sin(angle);rotation=np.array([[c,-s],[s,c]])
            vertex=fp[:,:2]@rotation.T+pose[:2,2];p_cells=np.floor((vertex-origin)/info.resolution).astype(int);p_vals=[]
            for aa,bb in zip(p_cells,np.roll(p_cells,-1,axis=0)):
                for xx,yy in line(tuple(aa),tuple(bb)):
                    if 0<=xx<info.width and 0<=yy<info.height:p_vals.append(int(data[yy,xx]))
            heading_costs.append({'fraction_of_goal_rotation':float(u),'yaw_rad':float(angle),'perimeter_max_occupancy':max(p_vals) if p_vals else None,'lethal_perimeter_count':sum(v==100 for v in p_vals)})
        np.savez_compressed(out.with_suffix('.npz') if len(records)==0 else out.with_name(out.stem+f'_{len(records)}.npz'),occupancy=data,vertices=vertices,origin=origin,resolution=info.resolution)
        return {'target_sim_s':target,'clock_sim_s':clock,'costmap_stamp_sim_s':grid.header.stamp.sec+grid.header.stamp.nanosec*1e-9,'frame':frame,'tf_pose_xyyaw':[float(pose[0,2]),float(pose[1,2]),math.atan2(pose[1,0],pose[0,0])],'padded_footprint':fp[:,:2].tolist(),'perimeter_max_occupancy':max(vals) if vals else None,'lethal_perimeter_cells':lethal,'goal_yaw_in_costmap_frame':goal_yaw,'sampled_rotation_to_goal_perimeter_costs':heading_costs,'note':'Nearest published grid/TF at target; occupancy100 represents lethal254. Asynchronous snapshot does not substitute exact native check.'}
    topic_ids=[i for i,(name,_) in types.items() if name in wanted];sql='SELECT topic_id,data FROM messages WHERE topic_id IN ('+','.join(map(str,topic_ids))+') ORDER BY timestamp'
    for i,data in db.execute(sql):
        name,cls=types[i];m=deserialize_message(data,cls)
        if name=='/clock':
            clock=m.clock.sec+m.clock.nanosec*1e-9
            while targets and clock>=targets[0]:records.append(snapshot(targets.pop(0)))
        elif name in ('/tf','/tf_static'):
            for tr in m.transforms:tf[(tr.header.frame_id.strip('/'),tr.child_frame_id.strip('/'))]=matrix(tr.transform)
        elif name=='/local_costmap/costmap':grid=m
        if not targets:break
    out.write_text(json.dumps(records,indent=2));print(json.dumps(records))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();diagnose(a.run,a.output)
