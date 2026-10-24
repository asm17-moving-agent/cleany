#!/usr/bin/env python3
"""Describe completed experiments, retaining aborts and timing failures."""
import csv
import json
import math
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent

def stats(a):
    a = np.asarray(a, float)
    return {'count':len(a), 'mean':float(np.mean(a)), 'p95':float(np.percentile(a,95)),
            'p99':float(np.percentile(a,99)), 'max':float(np.max(a))} if len(a) else {'count':0}

def enrich(run):
    result = json.loads((run/'result.json').read_text())
    result['arrived'] = result.get('action_status') == 4
    result['measurement_completed'] = result.get('completed',False)
    result['artifact_group'] = run.parent.name
    result['run_directory'] = str(run.relative_to(ROOT))
    result['terminal_status'] = ('timeout' if result.get('timeout') else
        {4:'arrived',5:'canceled',6:'aborted'}.get(result.get('action_status'),'infrastructure error'))
    start, end = result.get('navigation_sim_start_s'), result.get('navigation_sim_end_s')
    result['measurement_label'] = 'formal concurrently launched streams; overlap varies' if run.parent.name.endswith('_rtf1') else 'auxiliary isolated run'
    result['error_meaning'] = 'GT arrival error' if result.get('action_status')==4 else 'GT remaining error at abort/timeout; not arrival accuracy'
    if (run/'compute_timing.csv').exists():
        rows = list(csv.DictReader((run/'compute_timing.csv').open()))
        if start is not None:
            rows = [r for r in rows if start <= int(r['sim_time_ns'])*1e-9 <= end]
        all_ms=[float(r['compute_us'])/1000 for r in rows]
        result['controller_compute'] = {
            'scope':'inner plugin compute steady wall milliseconds; scheduling included',
            'all_ms':stats(all_ms),
            'success_ms':stats([float(r['compute_us'])/1000 for r in rows if r['success']=='1']),
            'exception_ms':stats([float(r['compute_us'])/1000 for r in rows if r['success']=='0']),
            'wall_deadline_ms':50., 'wall_deadline_exceeded':sum(x>50 for x in all_ms),
        }
    resources=json.loads((run/'resources.json').read_text()) if (run/'resources.json').exists() else []
    controllers={}
    for sample in resources:
        for p in sample['processes']:
            if '/controller_server ' in p['cmd']:
                controllers.setdefault(p['pid'],[]).append((sample['wall_s'],p['cpu_s'],p['rss_bytes']))
    pairs=[pair for rows in controllers.values() for pair in zip(rows[:-1],rows[1:]) if pair[1][0]>pair[0][0]]
    cpu_seconds=sum(b[1]-a[1] for a,b in pairs)
    observed_wall=sum(b[0]-a[0] for a,b in pairs)
    result['controller_resources']={
        'cpu_observed_seconds':cpu_seconds, 'observed_wall_seconds':observed_wall,
        'one_core_cpu_percent':100*cpu_seconds/observed_wall if observed_wall else None,
        'rss_mb':stats([r[2]/1024**2 for rows in controllers.values() for r in rows]),
        'host':result.get('host'), 'cross_host_ranking_valid':False,
    }
    joints=json.loads((run/'joints.json').read_text()) if (run/'joints.json').exists() else []
    wheel_samples=[]
    for row in joints:
        if start is not None and not start<=row['t']<=end: continue
        wheel_samples.extend(abs(v) for n,v in zip(row['names'],row['velocity']) if n.endswith('wheel_joint'))
    result['actual_wheel_feedback_rad_s']=stats(wheel_samples)
    result['actual_contact_count']=None
    result['actual_contact_note']='No physics contact sensor recorded; projected footprint overlap is a separate geometric estimate.'
    diagnostics=json.loads((run/'diagnostics.json').read_text()) if (run/'diagnostics.json').exists() else []
    reports=[d for d in diagnostics if d['topic']=='/body_guard_3d/state' and (start is None or start<=d['received_sim_s']<=end)]
    durations={};stop_reasons={}
    for a,b in zip(reports[:-1],reports[1:]):
        duration=max(0., b['received_sim_s']-a['received_sim_s'])
        durations[a.get('decision','UNKNOWN')]=durations.get(a.get('decision','UNKNOWN'),0.)+duration
        if a.get('decision','').startswith('STOP'):
            reason=a.get('decision','UNKNOWN')+' / '+a.get('reason','unspecified')
            stop_reasons[reason]=stop_reasons.get(reason,0.)+duration
    result['body_guard_decision_duration_sim_s']=durations
    result['body_guard_stop_reason_duration_sim_s']=stop_reasons
    result['intentional_safety_wait_sim_s']=durations.get('STOP_OBSTACLE',0.)
    result['safety_unavailable_wait_sim_s']=durations.get('STOP_UNAVAILABLE',0.)
    result['safety_processing_overrun_wait_sim_s']=durations.get('STOP_PROCESSING_OVERRUN',0.)
    result['total_safety_stop_sim_s']=sum(d for k,d in durations.items() if k.startswith('STOP'))
    geom=run/'geometry_summary.json'
    if geom.exists():result['footprint_geometry']=json.loads(geom.read_text())
    return result

def main():
    results=[]
    for group in ('rpp_rtf1','teb_rtf1','aux_dwb','isolated_rpp','isolated_teb'):
        for p in sorted((ROOT/group).glob('*/result.json')):
            results.append(enrich(p.parent))
    fields=['candidate','goal_index','goal_name','host','action_status','completed','measurement_completed',
            'arrived','terminal_status','timeout','error_meaning',
            'position_error_cm','arrival_position_error_cm','remaining_position_error_cm','yaw_error_deg',
            'navigation_sim_s','navigation_wall_s','achieved_rtf','cpu_one_core_percent','rss_p95_mb',
            'compute_p95_ms','compute_p99_ms','compute_deadline_exceeded','linear_jerk_rms_m_s3','linear_jerk_p95_m_s3',
            'angular_jerk_rms_rad_s3','angular_jerk_p95_rad_s3','max_actual_wheel_rad_s','max_final_command_wheel_rad_s',
            'intentional_safety_wait_sim_s','safety_unavailable_wait_sim_s',
            'safety_processing_overrun_wait_sim_s','total_safety_stop_sim_s',
            'min_projected_clearance_m','projected_overlap_sample_count','actual_contact_count','measurement_label','error']
    rows=[]
    for r in results:
        resources=r['controller_resources'];compute=r.get('controller_compute',{})
        row={k:r.get(k) for k in fields}
        position_cm=r['position_error_m']*100 if 'position_error_m' in r else None
        row.update(position_error_cm=position_cm,
                   arrival_position_error_cm=position_cm if r.get('action_status')==4 else None,
                   remaining_position_error_cm=position_cm if r.get('action_status')!=4 else None,
                   cpu_one_core_percent=resources['one_core_cpu_percent'],rss_p95_mb=resources['rss_mb'].get('p95'),
                   compute_p95_ms=compute.get('all_ms',{}).get('p95'),compute_p99_ms=compute.get('all_ms',{}).get('p99'),
                   compute_deadline_exceeded=compute.get('wall_deadline_exceeded'),
                   max_actual_wheel_rad_s=r['actual_wheel_feedback_rad_s'].get('max'),
                   max_final_command_wheel_rad_s=r.get('max_command_wheel_rad_s',{}).get('/cmd_vel'))
        geometry=r.get('footprint_geometry',{}).get('navigation',{})
        row.update({k:geometry.get(k) for k in ('min_projected_clearance_m','projected_overlap_sample_count')})
        row.update({k:r.get('smoothness',{}).get(k) for k in fields if 'jerk' in k})
        rows.append(row)
    for name,formal in [('remote_results',True),('auxiliary_results',False)]:
        selected=[(r,row) for r,row in zip(results,rows) if r['artifact_group'].endswith('_rtf1')==formal]
        (ROOT/(name+'.json')).write_text(json.dumps([r for r,_ in selected],indent=2))
        with (ROOT/(name+'.csv')).open('w') as f:
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows([row for _,row in selected])
    print(json.dumps([{k:r.get(k) for k in ('candidate','goal_index','action_status','position_error_cm','yaw_error_deg','navigation_sim_s','compute_p95_ms','compute_p99_ms','error')} for r in rows],indent=2))

if __name__=='__main__':main()
