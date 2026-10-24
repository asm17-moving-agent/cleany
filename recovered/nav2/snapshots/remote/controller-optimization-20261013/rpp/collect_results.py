#!/usr/bin/env python3
"""Produce final phase-separated machine-readable evidence without modifying baseline."""
import csv,importlib.util,json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
CON=HERE/'sol61_20261013_0620';REL=HERE/'relaxed_sol61_20261013'
BASE=HERE.parents[1]/'controller-comparison-20261013'
def load_module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def main():
    phases={};m=load_module('conservative_analysis',CON/'analyze.py')
    baseline=[]
    for run in sorted((BASE/'rpp_rtf1').glob('*/result.json')):
        out=CON/('baseline_'+run.parent.name+'.json');r=m.analyze(run.parent,out);r['measurement_label']='Archived baseline RPP concurrent with TEB on the same host';baseline.append(r)
    phases['baseline']=baseline
    phases['conservative']=[m.analyze(p.parent) for p in sorted((CON/'final_conservative').glob('*/result.json'))]
    relaxed=load_module('relaxed_analysis',REL/'analyze.py')
    phases['relaxed']=[relaxed.analyze(p.parent) for p in sorted((REL/'final_relaxed').glob('*/result.json'))]
    # The analysis module cache is identical in both packages; it contains only generic calculations.
    representatives={'conservative':[m.analyze(p.parent) for p in sorted(CON.glob('0*/2_chair_gap/result.json'))],
                     'relaxed':[relaxed.analyze(p.parent) for p in sorted(REL.glob('0*/*/result.json'))]}
    (HERE/'results.json').write_text(json.dumps({'schema_version':1,'phases':phases,'representatives':representatives},indent=2))
    fields=['phase','goal_name','terminal_status','action_status','measurement_completed','timeout','gt_xy_error_m','gt_yaw_error_deg','nav2_xy_error_m','nav2_yaw_error_deg','navigation_sim_s','navigation_wall_s','achieved_rtf','compute_median_ms','compute_p95_ms','compute_p99_ms','controller_exceptions','compute_over_50ms','cpu_one_core_percent','rss_p95_mb','linear_jerk_rms_m_s3','linear_jerk_p95_m_s3','angular_jerk_rms_rad_s3','angular_jerk_p95_rad_s3','linear_accel_rms_m_s2','angular_accel_rms_rad_s2','stationary_s','stop_count','yaw_reversals','lateral_reversals','final_abs_vx','final_abs_vy','final_abs_wz','max_final_command_wheel_rad_s','max_actual_wheel_rad_s','actual_contact_count','contact_reason','final_stop_verified','error_meaning','directory']
    rows=[]
    for phase,results in phases.items():
        for r in results:
            compute=r.get('controller_compute',{});stats=compute.get('all_ms',{});resources=r.get('controller_resources',{});v=r.get('command_stages',{}).get('/cmd_vel',{}).get('max_abs_xyz',[None]*3)
            row={k:r.get(k) for k in fields};row.update(phase=phase,gt_xy_error_m=r.get('position_error_m'),gt_yaw_error_deg=r.get('yaw_error_deg'),nav2_xy_error_m=r.get('nav2_localized_position_error_m'),nav2_yaw_error_deg=r.get('nav2_localized_yaw_error_deg'),compute_median_ms=stats.get('median'),compute_p95_ms=stats.get('p95'),compute_p99_ms=stats.get('p99'),controller_exceptions=r.get('controller_exception_count'),compute_over_50ms=compute.get('wall_deadline_exceeded'),cpu_one_core_percent=resources.get('one_core_cpu_percent'),rss_p95_mb=resources.get('rss_mb',{}).get('p95'),final_abs_vx=v[0],final_abs_vy=v[1],final_abs_wz=v[2],max_final_command_wheel_rad_s=r.get('max_command_wheel_rad_s',{}).get('/cmd_vel'),max_actual_wheel_rad_s=r.get('actual_wheel_feedback_rad_s',{}).get('max'),contact_reason=r.get('actual_contact_note'),directory=str(HERE.parents[1]/r['run_directory']))
            for k in fields:
                if k in r.get('smoothness',{}):row[k]=r['smoothness'][k]
            rows.append(row)
    with (HERE/'results.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    for phase,path in [('conservative',CON),('relaxed',REL)]:
        (path/'final_results.json').write_text(json.dumps(phases[phase],indent=2))
        with (path/'final_results.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows([r for r in rows if r['phase']==phase])
    print(json.dumps({phase:[{'goal':r['goal_name'],'status':r['terminal_status'],'sim_s':r['navigation_sim_s'],'xy_m':r['position_error_m'],'yaw_deg':r['yaw_error_deg']} for r in results] for phase,results in phases.items()},indent=2))
if __name__=='__main__':main()
