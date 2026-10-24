#!/usr/bin/env python3
"""Read raw evidence; write derived metrics only in this campaign."""
import argparse, ast, csv, importlib.util, json, math, sqlite3
from pathlib import Path
import numpy as np
import yaml
BASE=Path('/home/donglee/workspace/cleany/controller-comparison-20261013')
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('baseline_analysis',HERE/'analysis_base.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
module.ROOT=Path('/home/donglee/workspace/cleany')

def stats(values):
    r=module.stats(values)
    if len(values):r['median']=float(np.median(values))
    return r

def analyze(run, output=None):
    # Avoid changing preserved baseline / handoff artifacts when output is separate.
    r=module.enrich(run)
    start,end=r.get('navigation_sim_start_s'),r.get('navigation_sim_end_s')
    if start is None:return r
    commands=np.asarray(list(csv.reader((run/'commands.csv').open())),dtype=object)
    stage={}
    for topic in ['/nav2/cmd_vel','/pan_ready/cmd_vel','/safety_2d/cmd_vel','/cmd_vel']:
        a=commands[(commands[:,2]==topic)&(commands[:,0].astype(float)>=start)&(commands[:,0].astype(float)<=end)]
        if not len(a):stage[topic]={'count':0};continue
        t=a[:,0].astype(float);v=a[:,3:].astype(float);moving=np.max(abs(v),axis=1)>1e-6
        gaps=np.diff(t);dt=gaps[gaps>0]
        stage[topic]={'count':len(a),'first_sim_s':float(t[0]),'last_sim_s':float(t[-1]),'last_nonzero_sim_s':float(t[moving][-1]) if any(moving) else None,'max_abs_xyz':np.max(abs(v),axis=0).tolist(),'nonzero_count':int(sum(moving)),'period_sim_s':stats(dt),'gaps_over_0_25_sim_s':int(sum(gaps>.25)),'terminal_command':v[-1].tolist()}
    r['command_stages']=stage
    r['controller_model_dt_s']=None
    r['controller_model_dt_reason']='Humble RPP has no model_dt rollout parameter; controller_frequency=20Hz provides rotation acceleration dt.'
    for key in ['nav2_localized_final_pose','nav2_localized_position_error_m','nav2_localized_yaw_error_deg']:r.setdefault(key,None)
    if r['nav2_localized_final_pose'] is None:r['nav2_localized_final_pose_unavailable_reason']='Archived run did not record final localized error separately; Nav2 action status and GT error are available.'
    r['measurement_label']='RPP optimization final frozen one-pass validation' if run.parent.name.startswith('final_') else 'RPP optimization representative diagnostic'
    timeline=[]
    previous={}
    for row in commands:
        t=float(row[0]);topic=row[2]
        if not start<=t<=end:continue
        state='nonzero' if max(abs(float(x)) for x in row[3:])>1e-6 else 'zero'
        if previous.get(topic)!=state:
            timeline.append([t,float(row[1]),topic,state,'received command transition'])
            previous[topic]=state
    for row in csv.DictReader((run/'compute_timing.csv').open()):
        t=int(row['sim_time_ns'])*1e-9
        if start<=t<=end and row['success']=='0':timeline.append([t,int(row['wall_start_ns'])*1e-9,'controller','exception','native compute rejection; correlate launch.log'])
    previous={}
    for d in json.loads((run/'diagnostics.json').read_text()):
        t=d['received_sim_s'];topic=d['topic'];reason=d.get('reason','');decision=d.get('decision',reason)
        if start<=t<=end and previous.get(topic)!=(decision,reason):
            timeline.append([t,None,topic,decision,reason]);previous[topic]=(decision,reason)
    if output is None:
        with (run/'command_event_timeline.csv').open('w') as stream:
            writer=csv.writer(stream);writer.writerow(['received_sim_s','steady_wall_s','stage','state','reason']);writer.writerows(sorted(timeline,key=lambda r:r[0]))
    r['controller_compute']['all_ms']=stats([float(row['compute_us'])/1000 for row in csv.DictReader((run/'compute_timing.csv').open()) if start<=int(row['sim_time_ns'])*1e-9<=end])
    failures=[row for row in csv.DictReader((run/'compute_timing.csv').open()) if row['success']=='0' and start<=int(row['sim_time_ns'])*1e-9<=end]
    r['controller_exception_count']=len(failures)
    timing=[row for row in csv.DictReader((run/'compute_timing.csv').open()) if start<=int(row['sim_time_ns'])*1e-9<=end]
    periods=np.diff([int(row['sim_time_ns'])*1e-9 for row in timing]);r['compute_sim_period_without_replan_gaps_s']=stats(periods[(periods>0)&(periods<0.2)])
    wall_period=np.diff([int(row['wall_start_ns'])*1e-9 for row in timing]);r['compute_wall_period_without_replan_gaps_s']=stats(wall_period[(wall_period>0)&(wall_period<0.2)])
    r['first_controller_exception_sim_s']=int(failures[0]['sim_time_ns'])*1e-9 if failures else None
    raw=np.asarray(list(csv.reader((run/'velocity.csv').open())),float);raw=raw[(raw[:,0]>=start)&(raw[:,0]<=end)]
    _,idx=np.unique(raw[:,0],return_index=True);raw=raw[np.sort(idx)];t=np.arange(raw[0,0],raw[-1,0],.05)
    v=np.column_stack([np.interp(t,raw[:,0],raw[:,i]) for i in [1,2,3]])
    sm=np.column_stack([np.convolve(v[:,i],np.ones(5)/5,mode='same') for i in range(3)])
    accel=np.gradient(sm,.05,axis=0)[5:-5];speed=np.linalg.norm(v[:,:2],axis=1);moving=speed>.01
    r['smoothness'].update(linear_accel_rms_m_s2=float(np.sqrt(np.mean(np.sum(accel[:,:2]**2,axis=1)))),linear_accel_p95_m_s2=float(np.percentile(np.linalg.norm(accel[:,:2],axis=1),95)),angular_accel_rms_rad_s2=float(np.sqrt(np.mean(accel[:,2]**2))),angular_accel_p95_rad_s2=float(np.percentile(abs(accel[:,2]),95)),stop_count=int(sum(~moving[1:]&moving[:-1])))
    r['final_command_box_verified']=all(x<=limit+1e-9 for x,limit in zip(stage['/cmd_vel']['max_abs_xyz'],[.15,.10,.14]))
    r['final_command_wheel_verified']=r['max_command_wheel_rad_s']['/cmd_vel']<=5+1e-9
    r['actual_wheel_verified']=r['actual_wheel_feedback_rad_s'].get('max',float('inf'))<=5+1e-9
    r['last_final_command']=commands[commands[:,2]=='/cmd_vel'][-1,3:].astype(float).tolist()
    r['final_stop_verified']=max(abs(x) for x in r['last_final_command'])<1e-9 and max(abs(x) for x in r['final_actual_velocity'])<1e-6
    target=output or run/'analysis.json';target.write_text(json.dumps(r,indent=2))
    return r

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('runs',type=Path,nargs='+');p.add_argument('--output',type=Path);a=p.parse_args()
    for run in a.runs:
        r=analyze(run,a.output)
        print(json.dumps({k:r.get(k) for k in ['goal_name','terminal_status','position_error_m','yaw_error_deg','navigation_sim_s','first_controller_exception_sim_s','controller_exception_count','command_stages']}))
