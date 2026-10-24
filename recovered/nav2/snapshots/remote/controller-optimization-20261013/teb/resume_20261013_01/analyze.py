#!/usr/bin/env python3
"""Descriptive audits; preserve failed trajectories and every command stop transition."""
import csv
import importlib.util
import json
import re
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
BASE = ROOT.parents[2] / 'controller-comparison-20261013'
# Original enrichment functions are read only; their output targets are our runs.
spec = importlib.util.spec_from_file_location('original_analysis', BASE/'analyze_remote.py')
original = importlib.util.module_from_spec(spec)
spec.loader.exec_module(original)
original.ROOT = ROOT.parent.parent.parent

def statistics(values):
    values = np.asarray(values, float)
    return {'count': len(values), 'median': float(np.median(values)),
            'mean': float(np.mean(values)), 'p95': float(np.percentile(values, 95)),
            'p99': float(np.percentile(values, 99)), 'max': float(np.max(values))} if len(values) else {'count': 0}

def clean(value):
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    if isinstance(value,(float,np.floating)) and not np.isfinite(value):return None
    return value

def audit(run):
    r = original.enrich(run)
    start, end = r.get('navigation_sim_start_s'), r.get('navigation_sim_end_s')
    r['measurement_label'] = 'one-pass final validation' if run.parent.name.startswith('final') else 'single-goal causal pilot'
    if (run/'compute_timing.csv').exists():
        rows = list(csv.DictReader((run/'compute_timing.csv').open()))
        rows = [x for x in rows if start is None or start <= int(x['sim_time_ns'])*1e-9 <= end]
        values = [float(x['compute_us'])/1000 for x in rows]
        r['controller_compute']['all_ms'] = statistics(values)
        r['controller_compute']['exception_count'] = sum(x['success'] == '0' for x in rows)
        r['controller_compute']['success_ms'] = statistics([float(x['compute_us'])/1000 for x in rows if x['success'] == '1'])
        r['controller_compute']['exception_ms'] = statistics([float(x['compute_us'])/1000 for x in rows if x['success'] == '0'])
    commands = list(csv.reader((run/'commands.csv').open())) if (run/'commands.csv').exists() else []
    diagnostics = json.loads((run/'diagnostics.json').read_text()) if (run/'diagnostics.json').exists() else []
    reports = [d for d in diagnostics if d['topic'] == '/body_guard_3d/state']
    topics = sorted({x[2] for x in commands})
    transitions = []
    summaries = {}
    for topic in topics:
        rows = [x for x in commands if x[2] == topic]
        active = [x for x in rows if start is None or start <= float(x[0]) <= end]
        velocity = np.array([[float(v) for v in x[3:]] for x in active])
        intervals = np.diff([float(x[0]) for x in active])
        max_axes = np.max(np.abs(velocity), axis=0).tolist() if len(velocity) else None
        old = None
        for row in rows:
            t, wall = map(float, row[:2]); v = list(map(float, row[3:])); moving = max(map(abs, v)) > 1e-5
            if moving != old:
                previous = [d for d in reports if d['received_sim_s'] <= t]
                d = previous[-1] if previous else {}
                transitions.append({'topic':topic,'sim_s':t,'receive_steady_wall_s':wall,'moving':moving,
                                    'command':v,'nearest_prior_body_guard':{k:d.get(k) for k in ('received_sim_s','decision','reason')}})
                old = moving
        summaries[topic] = {'count':len(active),'max_abs_xyz':max_axes,
                            'box_exceed_count':int(np.sum(np.any(np.abs(velocity) > np.array([.15,.10,.14])+1e-8, axis=1))) if len(velocity) else 0,
                            'sim_interval_s':statistics(intervals[intervals>0]),
                            'last_command':list(map(float, rows[-1][3:])) if rows else None,
                            'last_received_sim_s':float(rows[-1][0]) if rows else None}
    transitions.sort(key=lambda x:x['sim_s'])
    (run/'command_transitions.json').write_text(json.dumps(transitions, indent=2))
    r['command_stage_audit'] = summaries
    pan_reports = [d for d in diagnostics if d['topic']=='/head_pan/state' and (start is None or start <= d['received_sim_s'] <= end)]
    pan_durations={}
    for a,b in zip(pan_reports[:-1],pan_reports[1:]):
        reason=a.get('reason','UNKNOWN'); duration=max(0,b['received_sim_s']-a['received_sim_s'])
        pan_durations[reason]=pan_durations.get(reason,0)+duration
    r['pan_gate_reason_duration_sim_s']=pan_durations
    bearing_steps=[]
    for a,b in zip(pan_reports[:-1],pan_reports[1:]):
        x,y=a.get('requested',[0,0,0])[:2]; u,v=b.get('requested',[0,0,0])[:2]
        if np.hypot(x,y)>.01 and np.hypot(u,v)>.01:
            delta=np.arctan2(v,u)-np.arctan2(y,x)
            bearing_steps.append(abs(float(np.arctan2(np.sin(delta),np.cos(delta)))))
    r['pan_requested_bearing_step_rad']=statistics(bearing_steps)
    log = (run/'launch.log').read_text(errors='replace') if (run/'launch.log').exists() else ''
    reasons = re.findall(r'TEB_POSE_REJECT .*?reason=([^\n]+)', log)
    r['feasibility_rejection_reasons'] = {reason:reasons.count(reason) for reason in sorted(set(reasons))}
    r['feasibility_failed_vertices'] = len(re.findall('TEB_FEASIBILITY sample=vertex', log))
    r['feasibility_failed_interpolated'] = len(re.findall('TEB_FEASIBILITY sample=interpolated', log))
    r['infeasible_reset_count'] = log.count('trajectory is not feasible. Resetting planner')
    r['controller_deadline_warning_count'] = log.count('Control loop missed its desired rate')
    r['final_gt_tolerance_observed'] = bool(r.get('position_error_m',1e9) <= .15 and r.get('yaw_error_deg',1e9) <= 10)
    # Nav2's map pose is distinct from ground-truth pose, which can differ after localization.
    rows = list(csv.DictReader((run/'samples.csv').open())) if (run/'samples.csv').exists() else []
    r['final_nav2_map_xy'] = [float(rows[-1][k]) for k in ('tf_x','tf_y')] if rows else None
    r['stop_count'] = sum(1 for x in transitions if x['topic']=='/cmd_vel' and not x['moving'] and start is not None and start <= x['sim_s'] <= end)
    # Preserve the shared velocity filter; supplement direction changes on the x axis.
    velocity_rows = list(csv.reader((run/'velocity.csv').open())) if (run/'velocity.csv').exists() else []
    if start is not None and len(velocity_rows)>12:
        a = np.asarray(velocity_rows, float)
        a = a[(a[:,0]>=start)&(a[:,0]<=end)]
        _, idx = np.unique(a[:,0], return_index=True); a=a[np.sort(idx)]
        t = np.arange(a[0,0], a[-1,0], .05)
        vx = np.interp(t,a[:,0],a[:,1]); vy = np.interp(t,a[:,0],a[:,2])
        signs = np.sign(vx[np.abs(vx)>.01])
        moving = np.hypot(vx,vy)>.01
        r['smoothness']['forward_reversals'] = int(np.sum(signs[1:]!=signs[:-1]))
        r['smoothness']['motion_stop_count'] = int(np.sum(~moving[1:] & moving[:-1]))
    r=clean(r)
    (run/'analysis.json').write_text(json.dumps(r, indent=2,allow_nan=False))
    return r

def main():
    runs = sorted(ROOT.glob('*/*/result.json'))
    if ROOT.name == 'resume_20261013_01':
        runs += list((ROOT.parent/'sol61_20261013/01_baseline_resume_0512').glob('*/result.json'))
    results = [audit(p.parent) for p in runs]
    (ROOT/'all_runs.json').write_text(json.dumps(results,indent=2))
    for group, rows in [('pilot_results',[r for r in results if not r['artifact_group'].startswith('final')]),
                        ('results',[r for r in results if r['artifact_group'].startswith('final') and r.get('navigation_sim_start_s') is not None]),
                        ('infrastructure_results',[r for r in results if r['artifact_group'].startswith('final') and r.get('navigation_sim_start_s') is None])]:
        (ROOT/(group+'.json')).write_text(json.dumps(rows, indent=2))
        flattened=[]
        for r in rows:
            c=r.get('controller_compute',{}); ms=c.get('all_ms',{}); resource=r['controller_resources']
            row={k:r.get(k) for k in ('artifact_group','goal_index','goal_name','action_status','terminal_status','measurement_completed','arrived','timeout','position_error_m','yaw_error_deg','final_gt_tolerance_observed','navigation_sim_s','navigation_wall_s','achieved_rtf','infeasible_reset_count','stop_count','error')}
            row.update({f'compute_{k}_ms':ms.get(k) for k in ('median','p95','p99','max')})
            row.update(compute_exception_count=c.get('exception_count'),compute_over_50ms=c.get('wall_deadline_exceeded'),
                       cpu_one_core_percent=resource['one_core_cpu_percent'],rss_p95_mb=resource['rss_mb'].get('p95'),
                       wheel_joint_max_rad_s=r['actual_wheel_feedback_rad_s'].get('max'),
                       final_command_wheel_max_rad_s=r.get('max_command_wheel_rad_s',{}).get('/cmd_vel'))
            row.update(r.get('smoothness',{})); flattened.append(row)
        fields=list(dict.fromkeys(k for r in flattened for k in r))
        with (ROOT/(group+'.csv')).open('w') as f:
            writer=csv.DictWriter(f,fieldnames=fields); writer.writeheader(); writer.writerows(flattened)
    print(json.dumps([{k:r.get(k) for k in ('artifact_group','goal_name','terminal_status','navigation_sim_s','position_error_m','yaw_error_deg','infeasible_reset_count','feasibility_rejection_reasons')} | {'compute':r.get('controller_compute',{}).get('all_ms')} for r in results], indent=2))

if __name__ == '__main__':
    main()
